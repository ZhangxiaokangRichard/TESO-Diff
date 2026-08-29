"""外环盘旋闭环整定评估器（WP2A-8 v2，GA 适应度，NPFG/TECS/协调转弯）。

染色体（16 维实数）与搜索范围：

    0  l1_period_s       [8.0, 20.0]   NPFG 参考距离周期
    1  l1_damping        [0.6, 0.9]    NPFG 阻尼（PX4：0.65~0.85）
    2  throttle_trim     [0.3, 0.8]
    3  theta_trim_deg    [-4.0, 8.0]
    4  energy_kp         [0.1, 1.0]    STE_err → 能量变化率需求
    5  throttle_kp       [0.1, 1.5]
    6  throttle_ki       [0.0, 0.5]
    7  airspeed_kp       [0.1, 1.0]    V_err → V̇_sp
    8  airspeed_ki       [0.0, 0.3]
    9  height_kp         [0.02, 0.3]   h_err → climb_sp
    10 balance_kp        [0.5, 5.0]    能量分配误差 → θ
    11 balance_ki        [0.0, 1.0]
    12 gamma_ff_gain     [0.0, 1.5]    γ_sp 前馈
    13 bank_comp_gain    [0.0, 1.0]    转弯升力补偿
    14 circle_kr         [0.1, 1.5]    PX4 Circle 径向误差增益
    15 circle_kv         [0.1, 2.0]    PX4 Circle 径向速度阻尼

评估剧本：S1 盘旋（半径 60 m，圆心 (0,60)，高度 1000 m，空速 16 m/s），
飞机初始状态 (0,0) 即在圆周上、航向切向，无风，90 s 闭环（4500 tick），
前 15 s 预热不计。内环参数由 tuning_02 整定结果注入。

代价：横向航迹误差 MAE + 高度误差 MAE + 空速误差 MAE + 发散/异常惩罚。
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

from ...bindings import AvantiInput, AvantiLibrary
from ..avanti_controller import AvantiController
from ..types import ControlObservation, ControlReference
from ...simulation.task_route import circle_path

GENE_NAMES = [
    "l1_period_s", "l1_damping",
    "throttle_trim", "theta_trim_deg",
    "energy_kp", "throttle_kp", "throttle_ki",
    "airspeed_kp", "airspeed_ki",
    "height_kp", "balance_kp", "balance_ki",
    "gamma_ff_gain", "bank_comp_gain",
    "circle_kr", "circle_kv",
]

BOUNDS = [
    (8.0, 20.0),     # l1_period_s
    (0.6, 0.9),      # l1_damping
    (0.3, 0.8),      # throttle_trim
    (-4.0, 8.0),     # theta_trim_deg
    (0.1, 1.0),      # energy_kp
    (0.1, 1.5),      # throttle_kp
    (0.0, 0.5),      # throttle_ki
    (0.1, 1.0),      # airspeed_kp
    (0.0, 0.3),      # airspeed_ki
    (0.02, 0.3),     # height_kp
    (0.5, 5.0),      # balance_kp
    (0.0, 1.0),      # balance_ki
    (0.0, 1.5),      # gamma_ff_gain
    (0.0, 1.0),      # bank_comp_gain
    (0.1, 1.5),      # circle_kr
    (0.1, 2.0),      # circle_kv
]

SEED_INDIVIDUALS = [
    np.array([12.0, 0.75, 0.55, 2.0, 0.3, 0.6, 0.15, 0.4, 0.05,
              0.10, 2.0, 0.2, 1.0, 0.4, 0.3, 0.5]),
    np.array([10.0, 0.75, 0.5, 1.0, 0.4, 0.8, 0.2, 0.5, 0.08,
              0.12, 2.5, 0.3, 1.0, 0.5, 0.4, 0.7]),
    np.array([15.0, 0.8, 0.6, 3.0, 0.25, 0.5, 0.1, 0.3, 0.03,
              0.08, 1.5, 0.15, 0.8, 0.3, 0.25, 0.4]),
]

DEG = np.pi / 180.0


def decode(genes: np.ndarray) -> dict:
    """染色体 → outer cfg。"""
    (period, damping, throttle_trim, theta_trim, energy_kp,
     throttle_kp, throttle_ki, airspeed_kp, airspeed_ki, height_kp,
     balance_kp, balance_ki, gamma_ff, bank_comp, circle_kr, circle_kv) = genes
    return {
        "l1": {
            "npfg_enabled": True,
            "period_s": float(period),
            "damping": float(damping),
            "l1_dist_min_m": 20.0,
            "l1_dist_max_m": 300.0,
            "a_cmd_limit_mps2": 15.0,
            "circle_kr": float(circle_kr),
            "circle_kv": float(circle_kv),
        },
        "coordinated_turn": {
            "bank_limit_deg": 30.0,
            "bank_rate_limit_degps": 15.0,
            "dt": 0.02,
        },
        "tecs": {
            "dt": 0.02,
            "throttle_trim": float(throttle_trim),
            "theta_trim_deg": float(theta_trim),
            "energy_kp": float(energy_kp),
            "throttle_kp": float(throttle_kp),
            "throttle_ki": float(throttle_ki),
            "airspeed_kp": float(airspeed_kp),
            "airspeed_ki": float(airspeed_ki),
            "height_kp": float(height_kp),
            "balance_kp": float(balance_kp),
            "balance_ki": float(balance_ki),
            "gamma_ff_gain": float(gamma_ff),
            "bank_comp_gain": float(bank_comp),
            "climb_sp_limit_mps": 5.0,
            "gamma_limit_deg": 20.0,
            "v_dot_limit_mps2": 3.0,
            "theta_limit_deg": 20.0,
            "tas_derivative_tau": 0.2,
        },
    }


class CircleTaskEvaluator:
    """盘旋闭环适应度评估（线程安全：每次 rollout 独立 model 实例）。"""

    def __init__(
        self,
        inner_cfg: dict,
        duration: float = 120.0,
        warmup: float = 20.0,
        radius: float = 60.0,
        altitude: float = 1000.0,
        airspeed: float = 16.0,
        wind_flat_earth: tuple[float, float, float] = (0.0, 0.0, 0.0),
        library_path: Optional[Path] = None,
    ):
        self.inner_cfg = inner_cfg
        self.duration = duration
        self.warmup = warmup
        self.dt = 0.02
        self.wind = np.asarray(wind_flat_earth, dtype=np.float64)
        self.library = AvantiLibrary(library_path)
        # 圆心 (0, radius)：飞机初始状态 (0,0,1000)、航向 0 恰在圆周上且切向
        self.path = circle_path(
            (0.0, radius), radius, altitude, n_pts=721,
            climb_rate=0.0, direction=1, airspeed=airspeed,
        )
        self.altitude_cmd = altitude
        self.airspeed_cmd = airspeed
        self.radius = radius

    def __call__(self, genes: np.ndarray) -> float:
        cost, _ = self.rollout(genes)
        return cost

    def rollout(self, genes: np.ndarray) -> tuple[float, dict]:
        outer_cfg = decode(genes)
        controller = AvantiController({"outer": outer_cfg, "inner": self.inner_cfg})
        controller.reset(1)

        ref = ControlReference(
            airspeed_cmd=np.array([self.airspeed_cmd]),
            altitude_cmd=np.array([self.altitude_cmd]),
            path=self.path[None, :, :],
            throttle_mode="auto",
        )

        ticks = int(self.duration / self.dt)
        log: dict[str, list[float]] = {
            "t": [], "north": [], "east": [], "alt": [], "tas": [],
            "cross_track": [], "phi": [], "theta": [], "psi": [],
            "p": [], "q": [], "r": [], "throttle": [],
            "phi_cmd": [], "theta_cmd": [], "alpha": [], "beta": [],
        }
        nan_seen = False
        divergence = False
        inputs = AvantiInput(throttle_01=0.5)
        with self.library.model(epoch_s=0.0) as model:
            output = model.tick(inputs)
            for k in range(ticks):
                t = k * self.dt
                obs = self._observation(output)
                # 内外环同频 50 Hz（与 harness 行为一致；外环计算开销小）
                command = controller.step(ref, obs)
                inputs = AvantiInput()
                inputs.delta_e_norm = float(command.delta_e[0])
                inputs.delta_a_norm = float(command.delta_a[0])
                inputs.delta_r_norm = float(command.delta_r[0])
                inputs.throttle_01 = float(command.throttle[0])
                inputs.set_wind(tuple(float(v) for v in self.wind))
                output = model.tick(inputs)
                if not output.finite():
                    nan_seen = True
                    break

                # 横向航迹误差（相对圆心半径差）
                north, east = output.position_ne_alt_m[0], output.position_ne_alt_m[1]
                radial = float(np.hypot(north, east - self.radius))
                cross = radial - self.radius
                log["t"].append(t)
                log["north"].append(north)
                log["east"].append(east)
                log["alt"].append(output.position_ne_alt_m[2])
                body = output.velocity_body_mps
                log["tas"].append(float(np.sqrt(body[0] ** 2 + body[1] ** 2 + body[2] ** 2)))
                log["cross_track"].append(cross)
                log["phi"].append(output.euler_rpy_deg[0] * DEG)
                log["theta"].append(output.euler_rpy_deg[1] * DEG)
                log["psi"].append(output.euler_rpy_deg[2] * DEG)
                rates = output.rates_body_radps
                log["p"].append(rates[0])
                log["q"].append(rates[1])
                log["r"].append(rates[2])
                log["throttle"].append(command.throttle[0])
                inter = controller.intermediates()
                log["phi_cmd"].append(inter["phi_cmd"][0])
                log["theta_cmd"].append(inter["theta_cmd"][0])
                log["alpha"].append(output.alpha_rad)
                log["beta"].append(output.beta_rad)
                if np.abs(log["phi"][-1]) > 85.0 * DEG or np.abs(log["theta"][-1]) > 70.0 * DEG:
                    divergence = True
                    break

        arrays = {key: np.array(value) for key, value in log.items()}
        cost = self.cost_from_log(arrays, nan_seen, divergence)
        return cost, arrays

    def _observation(self, output) -> ControlObservation:
        return ControlObservation.from_fw_outputs([output])

    def cost_from_log(self, log: dict, nan_seen: bool, divergence: bool) -> float:
        if nan_seen or len(log["t"]) < 100:
            return 1e6
        mask = log["t"] >= self.warmup
        if not mask.any():
            return 1e6

        cross = np.abs(log["cross_track"][mask])
        alt_err = np.abs(log["alt"][mask] - self.altitude_cmd)
        tas_err = np.abs(log["tas"][mask] - self.airspeed_cmd)

        cost = (0.5 * np.mean(cross)
                + 0.3 * np.mean(alt_err)
                + 0.2 * np.mean(tas_err))

        if divergence:
            cost += 1500.0
        if np.min(log["alt"]) < 0.0:
            cost += 2000.0
        if np.max(np.abs(log["alpha"])) > 0.5:
            cost += 800.0
        if np.max(np.abs(log["beta"])) > 0.5:
            cost += 400.0
        # 尾部不收环则重罚
        tail = log["t"] >= self.duration - 20.0
        if np.mean(np.abs(log["cross_track"][tail])) > 30.0:
            cost += 500.0
        return float(cost)