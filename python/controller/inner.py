"""内环 PID 姿态控制：期望姿态角 → 三个舵面归一化指令。

WP2A-8 实现。结构（docs §6.11）：
    phi_cmd   → PID → delta_a_cmd（副翼，归一化 [-1,1]）
    theta_cmd → PID → delta_e_cmd（升降舵，归一化 [-1,1]）
    psi_cmd   → PID → delta_r_cmd（方向舵，归一化 [-1,1]）

姿态误差需做角度环绕处理（phi/psi ∈ [-π, π]）。PID 积分/微分状态
必须可 snapshot/restore（counterfactual 要求）。
"""

from __future__ import annotations

import numpy as np

from .pid import PID
from .types import ActuatorCommand, ControlObservation


class InnerLoop:
    """内环姿态控制（接口冻结，实现见 WP2A-8）。"""

    def __init__(self, cfg: dict):
        self.cfg = cfg

    def reset(self, batch: int) -> None:
        raise NotImplementedError("WP2A-8")

    def step(
        self,
        phi_cmd: np.ndarray,
        theta_cmd: np.ndarray,
        psi_cmd: np.ndarray,
        obs: ControlObservation,
    ) -> ActuatorCommand:
        """期望姿态（rad，[B]）+ 观测 → 舵面归一化指令。"""
        raise NotImplementedError("WP2A-8")

    def get_snapshot(self) -> dict:
        raise NotImplementedError("WP2A-8")

    def set_snapshot(self, snap: dict) -> None:
        raise NotImplementedError("WP2A-8")
