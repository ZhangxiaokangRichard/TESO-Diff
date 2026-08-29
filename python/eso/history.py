"""History buffer：滚动保存闭环飞行的编码器输入真值流。

通道（docs §27）：NEU 位置/姿态/机体系速度/TAS/alpha/beta/IMU/估计风/
油门/舵面指令/控制器中间量/任务场景 id/segment energy/sensor health。
时间双通道：全局时间 + 相对时间（segment age、anchor 相对时间，
用于体现动作先后顺序对获能的影响）。

trajectory segment 建议 1.5 s / 3 s primary / 6 s，3 s 窗口、1 s hop
（docs §27）。本模块只做滚动缓存与通道管理，token 化留给 Phase 3。
"""

from __future__ import annotations

from collections import deque
from typing import Deque

import numpy as np


class HistoryBuffer:
    """定长滚动历史缓存（通道名 → deque）。"""

    def __init__(self, window_seconds: float = 6.0, dt: float = 0.02):
        self.window_seconds = window_seconds
        self.dt = dt
        self.maxlen = max(1, int(round(window_seconds / dt)))
        self.channels: dict[str, Deque] = {}

    def append(self, channel: str, value: float | np.ndarray) -> None:
        if channel not in self.channels:
            self.channels[channel] = deque(maxlen=self.maxlen)
        self.channels[channel].append(value)

    def as_arrays(self) -> dict[str, np.ndarray]:
        return {
            name: np.asarray(list(values), dtype=np.float64)
            for name, values in self.channels.items()
        }
