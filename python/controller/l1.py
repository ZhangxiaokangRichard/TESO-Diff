"""L1 位置控制（外环）接口定义。

WP2A-8 实现。由位置误差（NEU，期望空间位置序列来自任务航线）求：
- 期望横向加速度 a_cmd（L1 参考点算法）；
- 期望航向 chi_cmd。

L1 滤波状态必须可 snapshot/restore（counterfactual 要求，docs §6.12）。
"""

from __future__ import annotations

import numpy as np

from .types import ControlObservation, ControlReference


class L1Controller:
    """L1 位置控制（接口冻结，实现见 WP2A-8）。"""

    def __init__(self, cfg: dict):
        self.cfg = cfg

    def reset(self, batch: int) -> None:
        """重置 L1 滤波/积分状态。"""
        raise NotImplementedError("WP2A-8")

    def step(
        self, ref: ControlReference, obs: ControlObservation
    ) -> tuple[np.ndarray, np.ndarray]:
        """返回 (a_cmd [B], chi_cmd [B])。"""
        raise NotImplementedError("WP2A-8")

    def get_snapshot(self) -> dict:
        raise NotImplementedError("WP2A-8")

    def set_snapshot(self, snap: dict) -> None:
        raise NotImplementedError("WP2A-8")
