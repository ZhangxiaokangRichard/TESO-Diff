"""外环整合：L1/NPFG 侧向制导 + TECS 总能量控制 + 协调转弯律（WP2A-8 v2）。

整合输出（PX4 架构口径）：
- psi_cmd（期望空速航向，来自 NPFG 风三角，供内环偏航通道）
- theta_cmd（期望俯仰，来自 TECS 能量平衡 + 转弯升力补偿）
- phi_cmd（期望坡度，来自协调转弯律）
- throttle_cmd（来自 TECS 总能量通道）
"""

from __future__ import annotations

import numpy as np

from .coordinated_turn import CoordinatedTurn
from .l1 import L1Controller
from .tecs import TECSController
from .types import ControlObservation, ControlReference


class OuterLoop:
    """外环控制器（全张量化，PX4 架构）。"""

    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.l1 = L1Controller(cfg.get("l1", {}))
        self.tecs = TECSController(cfg.get("tecs", {}))
        self.turn = CoordinatedTurn(cfg.get("coordinated_turn", {}))

    def reset(self, batch: int) -> None:
        self.l1.reset(batch)
        self.tecs.reset(batch)
        self.turn.reset(batch)

    def step(
        self, ref: ControlReference, obs: ControlObservation
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """返回 (psi_cmd, theta_cmd, phi_cmd, throttle, cross_track)，
        均为 [B]，单位 rad/rad/rad/[0,1]/m。"""
        a_cmd, psi_cmd, cross_track, v_cmd = self.l1.step(ref, obs)
        phi_cmd = self.turn.step(a_cmd, obs)
        theta_cmd, throttle = self.tecs.step(ref, obs)
        # 盘旋转弯升力补偿：n = 1/cos φ，抬机头补偿掉高（docs §2.2/文章三期）
        theta_cmd = theta_cmd + self.tecs.bank_compensation(phi_cmd)
        return psi_cmd, theta_cmd, phi_cmd, throttle, cross_track

    def get_snapshot(self) -> dict:
        return {
            "l1": self.l1.get_snapshot(),
            "tecs": self.tecs.get_snapshot(),
            "turn": self.turn.get_snapshot(),
        }

    def set_snapshot(self, snap: dict) -> None:
        self.l1.set_snapshot(snap["l1"])
        self.tecs.set_snapshot(snap["tecs"])
        self.turn.set_snapshot(snap["turn"])
