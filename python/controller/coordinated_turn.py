"""协调转弯律（外环）接口定义。

WP2A-8 实现。由期望航向变化率 chi_dot_cmd 解算期望坡度 phi_cmd：

    phi_cmd = atan2(V_TAS * chi_dot_cmd, g)

并做最大坡度限幅与滚转角速率限制（防止大坡度指令超包线，docs §49 风险表）。
"""

from __future__ import annotations

import numpy as np

from .types import ControlObservation


class CoordinatedTurn:
    """协调转弯律（接口冻结，实现见 WP2A-8）。"""

    def __init__(self, cfg: dict):
        self.cfg = cfg

    def step(
        self, chi_dot_cmd: np.ndarray, obs: ControlObservation
    ) -> np.ndarray:
        """输入期望航向变化率 [B] rad/s，输出期望坡度 [B] rad。"""
        raise NotImplementedError("WP2A-8")
