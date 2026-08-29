"""TECS 总能量控制（外环）接口定义。

WP2A-8 实现。TECS 由高度与空速误差解算：
- throttle_cmd：总能量误差（动能 + 势能）→ 油门；
- energy_pitch_cmd：能量分配误差（动能 - 势能）→ 俯仰能量指令。

长期 TESO action 目标形式（docs §6.11）：
    a = [V_a*, gamma*, turn_rate*, gamma_rate*, cT, Tq]
本模块输出的 throttle_cmd 即 cT 通道的真值来源之一。
"""

from __future__ import annotations

import numpy as np

from .types import ControlObservation, ControlReference


class TECSController:
    """总能量控制外环（接口冻结，实现见 WP2A-8）。

    能量定义（docs §25）：
        e_total = 0.5 * V_TAS^2 + g * h
    """

    def __init__(self, cfg: dict):
        self.cfg = cfg

    def reset(self, batch: int) -> None:
        """重置能量/高度积分器（快慢能量误差积分）。"""
        raise NotImplementedError("WP2A-8")

    def step(
        self, ref: ControlReference, obs: ControlObservation
    ) -> tuple[np.ndarray, np.ndarray]:
        """返回 (throttle_cmd [B], energy_pitch_cmd [B])。"""
        raise NotImplementedError("WP2A-8")

    def get_snapshot(self) -> dict:
        raise NotImplementedError("WP2A-8")

    def set_snapshot(self, snap: dict) -> None:
        raise NotImplementedError("WP2A-8")
