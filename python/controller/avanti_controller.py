"""AvantiController：全张量化飞行控制器门面（docs §6.12，WP2A-8 实现）。

组合外环（L1 + TECS + 协调转弯）与内环（PID 姿态），对外只暴露：
- reset(batch, state)
- step(ref, obs) -> ActuatorCommand
- intermediates() -> dict[str, np.ndarray]  编码器输入状态张量
- get_snapshot() / set_snapshot()           积分器/滤波器状态

`intermediates()` 每 tick 导出期望航向、航迹误差、油门、期望姿态、
舵面指令等中间张量，全部进入 avanti_eso 编码器输入（docs §6.11/§6.12）。
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from .inner import InnerLoop
from .outer import OuterLoop
from .types import ActuatorCommand, ControlObservation, ControlReference


class AvantiController:
    """全张量化控制器，可直接对 B 个实例并行解算。"""

    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.outer = OuterLoop(cfg.get("outer", {}))
        self.inner = InnerLoop(cfg.get("inner", {}))
        self._batch = 0
        self._last: Optional[dict[str, np.ndarray]] = None

    def reset(self, batch: int, state=None) -> None:
        """重置控制器积分器/滤波器；state 为可选初始状态（接口预留）。"""
        self._batch = batch
        self.outer.reset(batch)
        self.inner.reset(batch)
        self._last = None

    def step(
        self, ref: ControlReference, obs: ControlObservation
    ) -> ActuatorCommand:
        """外环 + 内环一步解算。"""
        batch = obs.u.shape[0]
        if self._batch != batch:
            self.reset(batch)

        chi_cmd, theta_cmd, phi_cmd, throttle, cross_track = self.outer.step(ref, obs)
        command = self.inner.step(phi_cmd, theta_cmd, chi_cmd, obs)
        rates_sp = self.inner.rates_sp()

        # 油门通道：auto(TECS) / manual 覆盖（docs §6.12 throttle_mode）
        if ref.throttle_mode == "manual" and ref.cT_cmd is not None:
            command.throttle = np.array(ref.cT_cmd, dtype=np.float64)
        else:
            command.throttle = throttle

        self._last = {
            "psi_cmd": chi_cmd,
            "theta_cmd": theta_cmd,
            "phi_cmd": phi_cmd,
            "throttle_cmd": throttle,
            "cross_track": cross_track,
            "p_sp": None if rates_sp is None else rates_sp[:, 0],
            "q_sp": None if rates_sp is None else rates_sp[:, 1],
            "r_sp": None if rates_sp is None else rates_sp[:, 2],
            "delta_a": command.delta_a,
            "delta_e": command.delta_e,
            "delta_r": command.delta_r,
            "phi_err": ((phi_cmd - obs.phi + np.pi) % (2 * np.pi)) - np.pi,
            "theta_err": theta_cmd - obs.theta,
            "psi_err": ((chi_cmd - obs.psi + np.pi) % (2 * np.pi)) - np.pi,
        }
        return command

    def intermediates(self) -> dict[str, np.ndarray]:
        """avanti_eso 编码器输入状态张量（docs §6.11）。

        导出期望航向、期望俯仰/坡度、油门、舵面指令与姿态误差。
        """
        if self._last is None:
            raise RuntimeError("intermediates() requires at least one step()")
        return dict(self._last)

    def get_snapshot(self) -> dict:
        """TECS 积分器、PID 积分/微分、L1 滤波器状态的完整快照。"""
        return {
            "outer": self.outer.get_snapshot(),
            "inner": self.inner.get_snapshot(),
        }

    def set_snapshot(self, snap: dict) -> None:
        self.outer.set_snapshot(snap["outer"])
        self.inner.set_snapshot(snap["inner"])
