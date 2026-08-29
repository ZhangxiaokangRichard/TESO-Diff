"""AvantiController：全张量化飞行控制器门面（docs §6.12）。

组合外环（L1 + TECS + 协调转弯）与内环（PID 姿态），对外只暴露：
- reset(batch, state)
- step(ref, obs) -> ActuatorCommand
- intermediates() -> dict[str, np.ndarray]  编码器输入状态张量
- get_snapshot() / set_snapshot()           积分器/滤波器状态

`intermediates()` 每 tick 导出期望航向、航迹误差、油门、期望姿态、
舵面指令等中间张量，全部进入 avanti_eso 编码器输入（docs §6.11/§6.12）。

当前状态（phase2_sp1）：接口与数据结构已冻结；闭环控制律在 WP2A-8 实现。
"""

from __future__ import annotations

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
        self._last_actuator: ActuatorCommand | None = None

    def reset(self, batch: int, state=None) -> None:
        """重置控制器积分器/滤波器；state 为可选初始状态（接口预留）。"""
        self._batch = batch
        self.outer.reset(batch)
        self.inner.reset(batch)
        self._last_actuator = None

    def step(
        self, ref: ControlReference, obs: ControlObservation
    ) -> ActuatorCommand:
        """外环 + 内环一步解算。"""
        chi_cmd, theta_cmd, phi_cmd, throttle_cmd = self.outer.step(ref, obs)
        command = self.inner.step(phi_cmd, theta_cmd, chi_cmd, obs)
        # 手动油门模式覆盖 TECS 输出（docs §6.12 throttle_mode）
        if ref.throttle_mode == "manual" and ref.cT_cmd is not None:
            command.throttle = np.array(ref.cT_cmd, dtype=np.float64)
        else:
            command.throttle = np.array(throttle_cmd, dtype=np.float64)
        self._last_actuator = command
        return command

    def intermediates(self) -> dict[str, np.ndarray]:
        """avanti_eso 编码器输入状态张量（docs §6.11）。"""
        if self._last_actuator is None:
            raise RuntimeError("intermediates() requires at least one step()")
        raise NotImplementedError("WP2A-8")

    def get_snapshot(self) -> dict:
        """TECS 积分器、PID 积分/微分、L1 滤波器状态的完整快照。"""
        raise NotImplementedError("WP2A-8")

    def set_snapshot(self, snap: dict) -> None:
        raise NotImplementedError("WP2A-8")
