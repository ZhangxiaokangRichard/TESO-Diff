"""TensorRecorder：avanti_eso 张量记录器（docs §6.11）。

闭环 harness（python/simulation/harness.py）每个 dt_plant 调用
append(time, wind_truth, command, controller_intermediates, plant_output,
task_id)，记录编码器输入输出真值流。Phase 2a 验证阶段写入内存/CSV，
Phase 2b 由 dataset/writer 落 HDF5（docs §31）。
"""

from __future__ import annotations

from typing import Any

import numpy as np


class TensorRecorder:
    """逐 tick 追加编码器真值流（接口骨架，落盘格式见 WP2B-20）。"""

    def __init__(self, task_id: int = -1):
        self.task_id = task_id
        self.rows: list[dict[str, Any]] = []

    def append(
        self,
        time_s: float,
        wind_truth: np.ndarray,
        command: Any,
        intermediates: dict[str, np.ndarray],
        plant_output: Any,
    ) -> None:
        """记录一个 dt_plant 的真值切片。

        wind_truth [3]（flat-Earth，m/s）、command（舵面/油门）、
        intermediates（控制器中间量，docs §6.11）、plant_output（FWOutput）。
        """
        self.rows.append(
            {
                "time_s": time_s,
                "wind_truth": np.asarray(wind_truth, dtype=np.float64),
                "command": command,
                "intermediates": intermediates,
                "plant_output": plant_output,
            }
        )
