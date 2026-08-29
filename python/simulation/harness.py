"""闭环 Python harness（docs §6.13，WP2A-9）。

伪代码（docs §6.13）：

    for each dt_plant:
        wind_truth = wind_stub.query(position, time)
        if controller_tick:
            ref = task_route.desired_position(time)      # 期望空间位置
            command = controller.step(ref, observation)  # 张量并行
        plant_input = command + wind_truth
        plant_output = plant.step(plant_input)
        observation = build_observation(plant_output)
        eso_recorder.append(time, wind_truth, command,
                            controller.intermediates(),
                            plant_output, task_id)        # avanti_eso 张量
        log(...)

这一层完全不依赖 Isaac。当前（phase2_sp1）为接口骨架；
闭环实现在 WP2A-9，3 条封闭航迹任务在 run_phase2a_tasks.py。
"""

from __future__ import annotations

import numpy as np

from ..bindings import AvantiInput, AvantiLibrary
from ..controller import AvantiController
from ..eso import TensorRecorder


class ClosedLoopHarness:
    """plant + controller + wind stub + eso recorder 的闭环框架。"""

    def __init__(
        self,
        library: AvantiLibrary,
        controller: AvantiController,
        wind,
        recorder: TensorRecorder,
        dt_plant: float = 0.02,
        controller_rate_hz: float = 50.0,
    ):
        self.library = library
        self.controller = controller
        self.wind = wind
        self.recorder = recorder
        self.dt_plant = dt_plant
        self.controller_interval = max(1, int(round(1.0 / controller_rate_hz / dt_plant)))

    def reset(self) -> None:
        raise NotImplementedError("WP2A-9")

    def run(self, seconds: float) -> None:
        """运行 seconds 秒闭环仿真，逐 tick 记录 eso 张量流。"""
        raise NotImplementedError("WP2A-9")

    @staticmethod
    def _to_avanti_input(command, wind_flat_earth: np.ndarray) -> AvantiInput:
        """控制器输出 + 风真值 → AvantiInput（7 通道）。"""
        inputs = AvantiInput()
        inputs.delta_e_norm = float(command.delta_e[0])
        inputs.delta_a_norm = float(command.delta_a[0])
        inputs.delta_r_norm = float(command.delta_r[0])
        inputs.throttle_01 = float(command.throttle[0])
        inputs.set_wind(tuple(float(value) for value in wind_flat_earth))
        return inputs
