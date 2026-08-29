"""Wind stub：风场查询接口（docs §13）。

统一接口：
    W_truth = wind.query(position, time, env_parameters)

输入 position [B,3]（NEU）、time [B]（s），输出 wind_truth [B,3]。
注意：返回的是 flat-Earth 三轴风速（Xe/Ye/Ze，Z 向下为正），与 ABI
风输入一致；NEU 位置输入需在 query 内部转换为 flat-Earth。

Phase 2a 提供 zero / uniform 两种桩用于闭环验证；thermal/shear/
composite/gust 在 Phase 2b isaac/avanti_teso/wind/ 下实现（WP2B-05~09）。
"""

from __future__ import annotations

import numpy as np


class WindStub:
    """零风/恒定风桩（Phase 2a 闭环 harness 用）。"""

    def __init__(self, uniform_flat_earth: tuple[float, float, float] = (0.0, 0.0, 0.0)):
        # flat-Earth 轴风速 (Xe, Ye, Ze)，Z 向下为正，m/s
        self.uniform = np.asarray(uniform_flat_earth, dtype=np.float64)

    def query(
        self,
        position: np.ndarray,
        time: np.ndarray,
        env_parameters: dict | None = None,
    ) -> np.ndarray:
        """position [B,3]（NEU，第 3 分量向上）、time [B] → wind [B,3]（flat-Earth）。"""
        batch = position.shape[0]
        if time.shape[0] != batch:
            raise ValueError("position/time batch mismatch")
        # 恒定风：NEU 第 3 分量（Up）与 flat-Earth Ze（Down）符号相反
        ne = np.broadcast_to(self.uniform[:2], (batch, 2))
        down = -self.uniform[2]
        return np.hstack([ne, np.full((batch, 1), down)])
