"""闭环 Python harness（docs §6.13，WP2A-9 实现）。

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

这一层完全不依赖 Isaac。控制器/外环按 `controller_rate_hz` 更新，
plant 每 tick 推进 dt_plant。
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

from ..bindings import AvantiInput, AvantiLibrary
from ..controller import AvantiController
from ..controller.types import ControlObservation, ControlReference
from ..eso import TensorRecorder

LOG_FIELDS = (
    "t", "north", "east", "alt", "vn", "ve", "vu",
    "phi", "theta", "psi", "p", "q", "r", "u", "v", "w",
    "tas", "tas_cmd", "alpha", "beta",
    "phi_cmd", "theta_cmd", "psi_cmd", "throttle_cmd",
    "cross_track", "p_sp", "q_sp", "r_sp",
    "delta_a", "delta_e", "delta_r",
    "wind_x", "wind_y", "wind_z",
)


class ClosedLoopHarness:
    """plant + controller + wind stub + eso recorder 的闭环框架。"""

    def __init__(
        self,
        library: AvantiLibrary,
        controller: AvantiController,
        wind,
        task_path: np.ndarray,
        airspeed_cmd: float,
        altitude_cmd: float,
        recorder: Optional[TensorRecorder] = None,
        dt_plant: float = 0.02,
        controller_rate_hz: float = 50.0,
        throttle_mode: str = "auto",
        epoch_s: float = 0.0,
        wind_est_tau_s: float = 1.0,
        wind_est_bias: tuple[float, float] = (0.0, 0.0),
    ):
        self.library = library
        self.controller = controller
        self.wind = wind
        self.task_path = np.asarray(task_path, dtype=np.float64)[None, :, :]  # [1,N,3]
        self.airspeed_cmd = airspeed_cmd
        self.altitude_cmd = altitude_cmd
        self.recorder = recorder
        self.dt_plant = dt_plant
        self.controller_interval = max(1, int(round(1.0 / controller_rate_hz / dt_plant)))
        self.throttle_mode = throttle_mode
        self.epoch_s = epoch_s
        # 飞控风估计（docs §15）：truth → 一阶低通 → 常值偏差，NPFG 制导用
        self.wind_est_tau = wind_est_tau_s
        self.wind_est_bias = np.asarray(wind_est_bias, dtype=np.float64)
        self.wind_est_state: Optional[np.ndarray] = None  # [2] NEU 水平风估计
        self.model = None
        self.log: dict[str, list[float]] = {field: [] for field in LOG_FIELDS}

    def _ref(self) -> ControlReference:
        return ControlReference(
            airspeed_cmd=np.array([self.airspeed_cmd]),
            altitude_cmd=np.array([self.altitude_cmd]),
            path=self.task_path,
            throttle_mode=self.throttle_mode,
        )

    def _record(self, t: float, obs: ControlObservation, command,
                intermediates: dict, wind_flat: np.ndarray) -> None:
        self.log["t"].append(t)
        self.log["north"].append(obs.neu_pos[0, 0])
        self.log["east"].append(obs.neu_pos[0, 1])
        self.log["alt"].append(obs.neu_pos[0, 2])
        self.log["vn"].append(obs.neu_vel[0, 0])
        self.log["ve"].append(obs.neu_vel[0, 1])
        self.log["vu"].append(obs.neu_vel[0, 2])
        self.log["phi"].append(obs.phi[0])
        self.log["theta"].append(obs.theta[0])
        self.log["psi"].append(obs.psi[0])
        self.log["p"].append(obs.p[0])
        self.log["q"].append(obs.q[0])
        self.log["r"].append(obs.r[0])
        self.log["u"].append(obs.u[0])
        self.log["v"].append(obs.v[0])
        self.log["w"].append(obs.w[0])
        self.log["tas"].append(obs.tas[0])
        self.log["tas_cmd"].append(self.airspeed_cmd)
        self.log["alpha"].append(obs.alpha[0])
        self.log["beta"].append(obs.beta[0])
        self.log["phi_cmd"].append(intermediates["phi_cmd"][0])
        self.log["theta_cmd"].append(intermediates["theta_cmd"][0])
        self.log["psi_cmd"].append(intermediates["psi_cmd"][0])
        self.log["throttle_cmd"].append(intermediates["throttle_cmd"][0])
        self.log["cross_track"].append(intermediates["cross_track"][0])
        self.log["p_sp"].append(intermediates["p_sp"][0])
        self.log["q_sp"].append(intermediates["q_sp"][0])
        self.log["r_sp"].append(intermediates["r_sp"][0])
        self.log["delta_a"].append(command.delta_a[0])
        self.log["delta_e"].append(command.delta_e[0])
        self.log["delta_r"].append(command.delta_r[0])
        self.log["wind_x"].append(wind_flat[0, 0])
        self.log["wind_y"].append(wind_flat[0, 1])
        self.log["wind_z"].append(wind_flat[0, 2])

    def run(self, seconds: float) -> dict[str, np.ndarray]:
        """运行 seconds 秒闭环仿真，逐 tick 记录 avanti_eso 张量流。"""
        ticks = int(round(seconds / self.dt_plant))
        self.model = self.library.model(epoch_s=self.epoch_s)
        self.controller.reset(batch=1)

        # 初始 tick：获得首个观测（trim 输入：油门 0.5，其余 0）
        initial = AvantiInput(throttle_01=0.5)
        output = self.model.tick(initial)
        self.log = {field: [] for field in LOG_FIELDS}

        for k in range(ticks):
            t = k * self.dt_plant
            obs = ControlObservation.from_fw_outputs([output])
            pos = obs.neu_pos

            # 风真值（flat-Earth，Ze 向下为正）
            wind_flat = np.asarray(
                self.wind.query(pos, np.array([t])), dtype=np.float64
            )

            # 飞控风估计（docs §15）：truth → 一阶低通 → 偏差（NPFG 制导输入）
            # 注意：truth 风是 flat-Earth（Z 向下），NEU 水平分量 = (Xe, Ye)
            wind_neu_truth = np.array([wind_flat[0, 0], wind_flat[0, 1]])
            if self.wind_est_state is None:
                self.wind_est_state = wind_neu_truth
            alpha_w = self.dt_plant / (self.wind_est_tau + self.dt_plant)
            self.wind_est_state = (
                (1.0 - alpha_w) * self.wind_est_state + alpha_w * wind_neu_truth
            )
            wind_neu_est = self.wind_est_state + self.wind_est_bias
            obs.wind_neu = wind_neu_est[None, :]

            if k % self.controller_interval == 0:
                command = self.controller.step(self._ref(), obs)
                self._last_command = command
                self._last_intermediates = self.controller.intermediates()
            else:
                command = self._last_command

            self._record(t, obs, command, self._last_intermediates, wind_flat)

            inputs = AvantiInput()
            inputs.delta_e_norm = float(command.delta_e[0])
            inputs.delta_a_norm = float(command.delta_a[0])
            inputs.delta_r_norm = float(command.delta_r[0])
            inputs.throttle_01 = float(command.throttle[0])
            inputs.set_wind(tuple(float(value) for value in wind_flat[0]))
            output = self.model.tick(inputs)

            if not output.finite():
                self.log["nan_at_tick"] = [k]  # 标记异常（不中断，供 watchdog）
            if self.recorder is not None:
                self.recorder.append(t, wind_flat[0], command,
                                     self._last_intermediates, output)

        self.model.close()
        self.model = None
        return {key: np.array(value) for key, value in self.log.items()
                if key != "nan_at_tick" and len(value) > 0}

