"""TECS 总能量控制（外环，WP2A-8 v2 改造，能量形式）。

参考《PX4固定翼控制器详解(三)——TECS高度、速度控制器》：
    总能量（比能量）        E_s = h + V_TAS²/(2g)
    总能量误差              STE_err = (h_sp - h) + (V_sp² - V²)/(2g)
    总能量变化率需求        STE_rate_sp（由 STE_err 比例生成）
    实际总能量变化率        STE_rate = climb + V·V̇/g
    油门通道：油门控制总能量 → throttle = trim + K_p·ΔSTE_rate + K_i·∫ΔSTE_rate
    升降舵通道：控制动能-势能分配 → 能量平衡误差
                B = γ - V̇/g，B_sp = γ_sp - V̇_sp/g
                θ_cmd = γ_sp 前馈 + K_p·ΔB + K_i·∫ΔB + 配平 + 转弯补偿

长期 TESO action 目标形式（docs §6.11）：
    a = [V_a*, gamma*, turn_rate*, gamma_rate*, cT, Tq]
本模块输出的 throttle_cmd 即 cT 通道的真值来源之一。
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from .types import ControlObservation, ControlReference


class TECSController:
    """总能量控制外环（全张量化，能量形式）。"""

    G = 9.80665

    def __init__(self, cfg: dict):
        self.dt = float(cfg.get("dt", 0.02))
        self.throttle_trim = float(cfg.get("throttle_trim", 0.5))
        self.theta_trim = float(np.deg2rad(cfg.get("theta_trim_deg", 0.0)))

        # 总能量（油门通道）
        self.k_ste = float(cfg.get("energy_kp", 0.3))     # STE_err → STE_rate_sp
        self.kp_thr = float(cfg.get("throttle_kp", 0.5))
        self.ki_thr = float(cfg.get("throttle_ki", 0.1))

        # 空速（能量分配速度侧）
        self.kp_v = float(cfg.get("airspeed_kp", 0.4))    # V_err → V̇_sp
        self.ki_v = float(cfg.get("airspeed_ki", 0.05))

        # 高度 → 爬升率需求（能量分配高度侧）
        self.kp_h = float(cfg.get("height_kp", 0.1))      # h_err → climb_sp

        # 能量平衡（升降舵通道）
        self.kp_b = float(cfg.get("balance_kp", 2.0))
        self.ki_b = float(cfg.get("balance_ki", 0.2))
        self.gamma_ff = float(cfg.get("gamma_ff_gain", 1.0))  # γ_sp 前馈

        # 转弯补偿：盘旋时需增大升力 → 抬机头
        self.bank_comp_gain = float(cfg.get("bank_comp_gain", 0.4))

        # 限幅
        self.climb_sp_limit = float(cfg.get("climb_sp_limit_mps", 5.0))
        self.gamma_limit = float(np.deg2rad(cfg.get("gamma_limit_deg", 20.0)))
        self.v_dot_limit = float(cfg.get("v_dot_limit_mps2", 3.0))
        self.theta_limit = float(np.deg2rad(cfg.get("theta_limit_deg", 20.0)))

        # 微分滤波（V̇ 估计）
        self.tas_tau = float(cfg.get("tas_derivative_tau", 0.2))

        self._integral_thr: Optional[np.ndarray] = None
        self._integral_v: Optional[np.ndarray] = None
        self._integral_b: Optional[np.ndarray] = None
        self._prev_tas: Optional[np.ndarray] = None
        self._prev_v_dot: Optional[np.ndarray] = None
        self._prev_alt: Optional[np.ndarray] = None
        self._prev_climb: Optional[np.ndarray] = None

    def reset(self, batch: int) -> None:
        self._integral_thr = np.zeros(batch)
        self._integral_v = np.zeros(batch)
        self._integral_b = np.zeros(batch)
        self._prev_tas = None
        self._prev_v_dot = np.zeros(batch)
        self._prev_alt = None
        self._prev_climb = np.zeros(batch)

    def _ensure(self, batch: int) -> None:
        if self._integral_thr is None or self._integral_thr.shape[0] != batch:
            self.reset(batch)

    def step(
        self, ref: ControlReference, obs: ControlObservation
    ) -> tuple[np.ndarray, np.ndarray]:
        """返回 (theta_cmd [B] rad, throttle_cmd [B])。

        obs 需要 neu_pos/neu_vel/tas；phi_cmd 转弯补偿由外层传入。
        """
        alt = obs.neu_pos[:, 2]
        climb = obs.neu_vel[:, 2]
        tas = np.maximum(obs.tas, 1e-3)
        v_sp = ref.airspeed_cmd
        h_sp = ref.altitude_cmd
        batch = alt.shape[0]
        self._ensure(batch)
        assert self._integral_thr is not None and self._prev_v_dot is not None

        # ---- 总能量误差与能量变化率 ----
        ste_err = (h_sp - alt) + (v_sp ** 2 - tas ** 2) / (2.0 * self.G)
        ste_rate_sp = self.k_ste * ste_err

        # V̇：低通滤波的数值微分（V̇ = (V_k - V_{k-1})/dt → 一阶滤波）
        if self._prev_tas is None:
            self._prev_tas = tas
        v_dot_raw = (tas - self._prev_tas) / self.dt
        alpha_f = self.dt / (self.tas_tau + self.dt)
        self._prev_v_dot = (1.0 - alpha_f) * self._prev_v_dot + alpha_f * v_dot_raw
        self._prev_tas = tas
        v_dot = np.clip(self._prev_v_dot, -self.v_dot_limit * 2.0, self.v_dot_limit * 2.0)

        # climb 低通（减弱数值噪声）
        if self._prev_climb is None:
            self._prev_climb = climb
        self._prev_climb = (1.0 - alpha_f) * self._prev_climb + alpha_f * climb
        climb_f = self._prev_climb

        ste_rate = climb_f + tas * v_dot / self.G

        # ---- 油门：控制总能量变化率 ----
        rate_err = ste_rate_sp - ste_rate
        self._integral_thr += rate_err * self.dt
        self._integral_thr = np.clip(self._integral_thr, -2.0, 2.0)
        throttle = self.throttle_trim + self.kp_thr * rate_err + self.ki_thr * self._integral_thr
        throttle = np.clip(throttle, 0.0, 1.0)

        # ---- 能量平衡：升降舵控制动能-势能分配 ----
        v_err = v_sp - tas
        self._integral_v += v_err * self.dt
        self._integral_v = np.clip(self._integral_v, -5.0, 5.0)
        v_dot_sp = np.clip(
            self.kp_v * v_err + self.ki_v * self._integral_v,
            -self.v_dot_limit, self.v_dot_limit,
        )

        climb_sp = np.clip(self.kp_h * (h_sp - alt), -self.climb_sp_limit, self.climb_sp_limit)
        gamma_sp = np.clip(climb_sp / np.maximum(tas, 5.0), -self.gamma_limit, self.gamma_limit)
        gamma = np.clip(np.arcsin(np.clip(climb_f / np.maximum(tas, 5.0), -1.0, 1.0)),
                        -self.gamma_limit, self.gamma_limit)

        b_sp = gamma_sp - v_dot_sp / self.G
        b = gamma - v_dot / self.G
        b_err = b_sp - b
        self._integral_b += b_err * self.dt
        self._integral_b = np.clip(self._integral_b, -1.0, 1.0)
        theta_cmd = (self.gamma_ff * gamma_sp + self.theta_trim
                     + self.kp_b * b_err + self.ki_b * self._integral_b)
        theta_cmd = np.clip(theta_cmd, -self.theta_limit, self.theta_limit)

        return theta_cmd, throttle

    def bank_compensation(self, phi: np.ndarray) -> np.ndarray:
        """盘旋转弯升力补偿：θ 增量 ≈ k·(1/cos φ - 1)，rad。"""
        return self.bank_comp_gain * (1.0 / np.maximum(np.cos(phi), 0.3) - 1.0)

    def get_snapshot(self) -> dict:
        if self._integral_thr is None:
            raise RuntimeError("TECS not reset yet")
        return {
            "integral_thr": self._integral_thr.copy(),
            "integral_v": self._integral_v.copy(),
            "integral_b": self._integral_b.copy(),
            "prev_tas": None if self._prev_tas is None else self._prev_tas.copy(),
            "prev_v_dot": self._prev_v_dot.copy(),
            "prev_climb": self._prev_climb.copy(),
        }

    def set_snapshot(self, snap: dict) -> None:
        self._integral_thr = np.array(snap["integral_thr"])
        self._integral_v = np.array(snap["integral_v"])
        self._integral_b = np.array(snap["integral_b"])
        self._prev_tas = None if snap["prev_tas"] is None else np.array(snap["prev_tas"])
        self._prev_v_dot = np.array(snap["prev_v_dot"])
        self._prev_climb = np.array(snap["prev_climb"])
