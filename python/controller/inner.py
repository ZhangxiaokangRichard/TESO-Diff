"""内环串级姿态控制（PX4 双回路结构，WP2A-8 v2 改造）。

参考《PX4固定翼控制器详解(一)(二)》：
    外环角度环（P）：angle_error → 期望机体系角速率 rate_sp（带限幅）
    内环角速率环：期望角加速度前馈 + 速率误差 P/PI
    （固定翼自身有气动阻尼，滚转通道不加 I，俯仰/偏航加小 I 去稳态误差）

    协调转弯（偏航通道前馈）：
        r_ff = g · tan(phi_cmd) · cos(theta) / V_TAS      （爬升转弯乘 cosθ）
    空速缩放（文章第二期 §2）：
        控制项缩放 (V_trim / V_TAS)^2   —— 补偿动压随 V² 变化
        前馈项缩放 (V_trim / V_TAS)     —— 补偿阻尼力矩随 V 线性变化

物理符号（2026-08-30 舵面符号复测，见 experiments/wp2a8/control_sign_probe.py）：
    delta_e 正 → 低头（theta 减小）  → 俯仰通道取负
    delta_a 正 → 左滚（phi 减小）    → 滚转通道取负
    delta_r 正 → 右偏航（psi 增加）  → 偏航通道取正

全部张量化（numpy，无逐实例循环），可直接平移 GPU/Isaac。
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from .pid import PID
from .types import ActuatorCommand, ControlObservation


def wrap_pi(angle: np.ndarray) -> np.ndarray:
    """角度环绕到 [-π, π]。"""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


class InnerLoop:
    """串级姿态内环（全张量化）。"""

    G = 9.80665

    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.dt = float(cfg.get("dt", 0.02))

        # 角度环 P 增益（rad/s per rad）与角速率限幅
        self.k_phi = float(cfg.get("angle_roll_kp", 3.0))
        self.k_theta = float(cfg.get("angle_pitch_kp", 3.0))
        self.k_psi = float(cfg.get("angle_yaw_kp", 2.0))
        self.p_limit = float(np.deg2rad(cfg.get("p_limit_degps", 60.0)))
        self.q_limit = float(np.deg2rad(cfg.get("q_limit_degps", 45.0)))
        self.r_limit = float(np.deg2rad(cfg.get("r_limit_degps", 30.0)))

        # 角速率环参数：kp/ki（速率误差）+ ff（期望角加速度前馈）
        roll = cfg.get("roll", {})
        pitch = cfg.get("pitch", {})
        yaw = cfg.get("yaw", {})
        self.rate_roll = PID(
            kp=roll.get("rate_kp", 0.1), ki=0.0, kd=0.0, dt=self.dt,
            output_limits=roll.get("output_limits", [-1.0, 1.0]),
        )
        self.rate_pitch = PID(
            kp=pitch.get("rate_kp", 0.1), ki=pitch.get("rate_ki", 0.0),
            kd=0.0, dt=self.dt,
            output_limits=pitch.get("output_limits", [-1.0, 1.0]),
        )
        self.rate_yaw = PID(
            kp=yaw.get("rate_kp", 0.1), ki=yaw.get("rate_ki", 0.0),
            kd=0.0, dt=self.dt,
            output_limits=yaw.get("output_limits", [-1.0, 1.0]),
        )
        self.ff_roll = float(roll.get("rate_ff", 0.3))
        self.ff_pitch = float(pitch.get("rate_ff", 0.3))
        self.ff_yaw = float(yaw.get("rate_ff", 0.3))
        self.coord_ff_gain = float(cfg.get("coordinated_turn_ff_gain", 1.0))

        # 空速缩放
        self.v_trim = float(cfg.get("v_trim_mps", 16.0))
        self.scale_min = float(cfg.get("airspeed_scale_min", 0.25))
        self.scale_max = float(cfg.get("airspeed_scale_max", 4.0))

        self.beta_gain = float(cfg.get("beta_gain", 0.5))  # 侧滑阻尼
        self._batch = 0
        self._prev_rate_sp: Optional[np.ndarray] = None  # [B,3] p/q/r 期望角速率

    # ------------------------------------------------------------------ #
    def reset(self, batch: int = 1) -> None:
        self._batch = batch
        self.rate_roll.reset(batch)
        self.rate_pitch.reset(batch)
        self.rate_yaw.reset(batch)
        self._prev_rate_sp = np.zeros((batch, 3))

    # ------------------------------------------------------------------ #
    def _airspeed_scales(self, tas: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """返回 (control_scale, feedforward_scale)，按 V_trim/V_TAS 缩放。"""
        v = np.maximum(tas, 1e-3)
        linear = np.clip(self.v_trim / v, self.scale_min, self.scale_max)
        return linear ** 2, linear

    def coordinated_yaw_rate_ff(
        self, phi_cmd: np.ndarray, obs: ControlObservation
    ) -> np.ndarray:
        """协调转弯偏航角速率前馈 r_ff = g·tan(φ)·cos(θ)/V_TAS。"""
        v = np.maximum(obs.tas, 1e-3)
        return self.coord_ff_gain * self.G * np.tan(phi_cmd) * np.cos(obs.theta) / v

    # ------------------------------------------------------------------ #
    def step(
        self,
        phi_cmd: np.ndarray,
        theta_cmd: np.ndarray,
        psi_cmd: Optional[np.ndarray],
        obs: ControlObservation,
    ) -> ActuatorCommand:
        """期望姿态（rad，[B]）+ 观测 → 舵面归一化指令。

        psi_cmd=None 时偏航通道仅做 beta 阻尼 + 协调转弯前馈（不控航向）。
        """
        batch = obs.u.shape[0]
        if self._batch != batch:
            self.reset(batch)
        assert self._prev_rate_sp is not None

        tas = np.asarray(obs.tas, dtype=np.float64)
        if tas.shape[0] != batch:
            tas = np.linalg.norm(
                np.stack([obs.u, obs.v, obs.w], axis=1), axis=1
            )
        scale_c, scale_f = self._airspeed_scales(tas)

        # ---------- 外环角度 → 期望角速率（P，带限幅） ----------
        err_phi = wrap_pi(phi_cmd - obs.phi)
        err_theta = theta_cmd - obs.theta
        p_sp = np.clip(self.k_phi * err_phi, -self.p_limit, self.p_limit)
        q_sp = np.clip(self.k_theta * err_theta, -self.q_limit, self.q_limit)
        if psi_cmd is not None:
            err_psi = wrap_pi(psi_cmd - obs.psi)
            r_sp = np.clip(self.k_psi * err_psi, -self.r_limit, self.r_limit)
        else:
            r_sp = np.zeros(batch)

        # 协调转弯前馈叠加到偏航角速率（期望航向与滚转解耦）
        r_sp = r_sp + self.coordinated_yaw_rate_ff(phi_cmd, obs)
        r_sp = np.clip(r_sp, -self.r_limit, self.r_limit)

        rate_sp = np.stack([p_sp, q_sp, r_sp], axis=1)  # [B,3]

        # ---------- 内环角速率：前馈 + 误差控制（空速缩放） ----------
        rate_sp_dot = (rate_sp - self._prev_rate_sp) / self.dt
        rate_sp_dot = np.clip(rate_sp_dot, -20.0, 20.0)  # 期望角加速度限幅 rad/s²
        self._prev_rate_sp = rate_sp

        p_dot_sp = rate_sp_dot[:, 0]
        q_dot_sp = rate_sp_dot[:, 1]
        r_dot_sp = rate_sp_dot[:, 2]

        delta_a = -(
            self.ff_roll * scale_f * p_dot_sp
            + self.rate_roll.step(p_sp - obs.p) * scale_c
        )  # 负号：delta_a 正 → 左滚
        delta_e = -(
            self.ff_pitch * scale_f * q_dot_sp
            + self.rate_pitch.step(q_sp - obs.q) * scale_c
        )  # 负号：delta_e 正 → 低头
        delta_r = (
            self.ff_yaw * scale_f * r_dot_sp
            + self.rate_yaw.step(r_sp - obs.r) * scale_c
            - self.beta_gain * obs.beta
        )

        delta_a = np.clip(delta_a, -1.0, 1.0)
        delta_e = np.clip(delta_e, -1.0, 1.0)
        delta_r = np.clip(delta_r, -1.0, 1.0)

        return ActuatorCommand(
            delta_a=delta_a,
            delta_e=delta_e,
            delta_r=delta_r,
            throttle=np.zeros(batch),  # 油门由外环填充
        )

    # ------------------------------------------------------------------ #
    def rates_sp(self) -> Optional[np.ndarray]:
        """最近一次 step 的期望角速率 [B,3]（intermediates 用）。"""
        if self._prev_rate_sp is None:
            return None
        return self._prev_rate_sp.copy()

    def get_snapshot(self) -> dict:
        return {
            "batch": self._batch,
            "roll": self.rate_roll.get_snapshot(),
            "pitch": self.rate_pitch.get_snapshot(),
            "yaw": self.rate_yaw.get_snapshot(),
            "prev_rate_sp": None if self._prev_rate_sp is None
            else self._prev_rate_sp.copy(),
        }

    def set_snapshot(self, snap: dict) -> None:
        self._batch = snap["batch"]
        self.rate_roll.set_snapshot(snap["roll"])
        self.rate_pitch.set_snapshot(snap["pitch"])
        self.rate_yaw.set_snapshot(snap["yaw"])
        self._prev_rate_sp = (
            None if snap["prev_rate_sp"] is None
            else np.array(snap["prev_rate_sp"], dtype=np.float64)
        )

