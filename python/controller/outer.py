"""外环整合：L1 位置控制 + TECS 总能量控制 + 协调转弯律。

WP2A-8 实现。整合输出：
- chi_cmd（期望航向，来自 L1）
- theta_cmd / gamma_cmd（期望俯仰/航迹倾角，来自 TECS 能量分配）
- phi_cmd（期望坡度，来自协调转弯律）
- throttle_cmd（来自 TECS 总能量）
"""

from __future__ import annotations

import numpy as np

from .coordinated_turn import CoordinatedTurn
from .l1 import L1Controller
from .tecs import TECSController
from .types import ControlObservation, ControlReference


class OuterLoop:
    """外环控制器（接口冻结，实现见 WP2A-8）。"""

    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.l1 = L1Controller(cfg.get("l1", {}))
        self.tecs = TECSController(cfg.get("tecs", {}))
        self.turn = CoordinatedTurn(cfg.get("coordinated_turn", {}))

    def reset(self, batch: int) -> None:
        self.l1.reset(batch)
        self.tecs.reset(batch)

    def step(
        self, ref: ControlReference, obs: ControlObservation
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """返回 (chi_cmd, theta_cmd, phi_cmd, throttle_cmd)，均为 [B] rad/rad/rad/[0,1]。"""
        raise NotImplementedError("WP2A-8")

    def get_snapshot(self) -> dict:
        raise NotImplementedError("WP2A-8")

    def set_snapshot(self, snap: dict) -> None:
        raise NotImplementedError("WP2A-8")
