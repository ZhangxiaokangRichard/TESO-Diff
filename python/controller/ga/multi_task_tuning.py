"""多任务外环整定评估器（WP2A-8 v4，GA 适应度）。

在修正后的三条封闭航迹上联合整定外环（L1/NPFG + TECS + 协调转弯）：
    1. 盘旋 circle R=60（loiter 制导）——60 s
    2. 空间 8 字 figure8 R=80/D=400（NPFG 采样制导）——120 s
    3. 跑马场 racetrack R=80/L=400（NPFG 采样制导）——120 s

染色体（16 维，与 outer_tuning 一致）：
    0  l1_period_s [8,20] 1 l1_damping [0.6,0.9] 2 throttle_trim [0.3,0.8]
    3  theta_trim_deg [-4,8] 4 energy_kp [0.1,1] 5 throttle_kp [0.1,1.5]
    6  throttle_ki [0,0.5] 7 airspeed_kp [0.1,1] 8 airspeed_ki [0,0.3]
    9  height_kp [0.02,0.3] 10 balance_kp [0.5,5] 11 balance_ki [0,1]
    12 gamma_ff_gain [0,1.5] 13 bank_comp_gain [0,1]
    14 circle_kr [0.1,1.5] 15 circle_kv [0.1,2]

批次并行生成测试（batch_benchmark）：
    对一批基因并行 rollout 全部任务，统计并行 vs 串行吞吐与
    sim-seconds / wall-second，验证后续张量并行数据采集的有效性。
"""

from __future__ import annotations

import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Optional

import numpy as np

from ...bindings import AvantiLibrary
from ..avanti_controller import AvantiController
from ..types import ControlReference
from ...simulation import ClosedLoopHarness, WindStub
from ...simulation.task_route import (
    circle_path,
    figure8_spatial_path,
    racetrack_path,
)

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
    (8.0, 20.0), (0.6, 0.9), (0.3, 0.8), (-4.0, 8.0),
    (0.1, 1.0), (0.1, 1.5), (0.0, 0.5), (0.1, 1.0), (0.0, 0.3),
    (0.02, 0.3), (0.5, 5.0), (0.0, 1.0), (0.0, 1.5), (0.0, 1.0),
    (0.1, 1.5), (0.1, 2.0),
]

# 手工种子 + tuning_03 最优基因
SEED_INDIVIDUALS = [
    np.array([20.0, 0.863, 0.533, -1.526, 0.608, 0.361, 0.460,
              1.0, 0.232, 0.166, 1.156, 0.889, 0.718, 0.719, 1.5, 1.581]),
    np.array([12.0, 0.75, 0.55, 2.0, 0.3, 0.6, 0.15, 0.4, 0.05,
              0.10, 2.0, 0.2, 1.0, 0.4, 0.3, 0.5]),
    np.array([10.0, 0.75, 0.5, 1.0, 0.4, 0.8, 0.2, 0.5, 0.08,
              0.12, 2.5, 0.3, 1.0, 0.5, 0.4, 0.7]),
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
            "bank_limit_deg": 35.0,
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


def task_paths() -> list[tuple[str, np.ndarray, float]]:
    """返回 [(task_name, path, airspeed_cmd)]。"""
    return [
        ("circle", circle_path((0.0, 60.0), 60.0, 1000.0,
                               n_pts=721, airspeed=16.0), 16.0),
        ("figure8", figure8_spatial_path((0.0, 0.0), 80.0, 1000.0,
                                         n_pts=1441, center_dist=400.0,
                                         airspeed=16.0), 16.0),
        ("racetrack", racetrack_path((0.0, 80.0), 400.0, 80.0, 1000.0,
                                     n_pts=1441, airspeed=16.0), 16.0),
    ]


class MultiTaskEvaluator:
    """三任务联合适应度评估（线程安全：每次 rollout 独立 model 实例）。"""

    def __init__(self, inner_cfg: dict, library_path: Optional[Path] = None):
        self.inner_cfg = inner_cfg
        self.library = AvantiLibrary(library_path)
        self.tasks = task_paths()

    def __call__(self, genes: np.ndarray) -> float:
        cost, _ = self.rollout(genes)
        return cost

    def rollout(self, genes: np.ndarray) -> tuple[float, dict]:
        outer = decode(genes)
        total = 0.0
        stats = {}
        weights = {"circle": 0.4, "figure8": 0.3, "racetrack": 0.3}
        for name, path, airspeed in self.tasks:
            seconds = {"circle": 60.0, "figure8": 120.0, "racetrack": 120.0}[name]
            warmup = {"circle": 15.0, "figure8": 20.0, "racetrack": 20.0}[name]
            controller = AvantiController({"outer": outer, "inner": self.inner_cfg})
            harness = ClosedLoopHarness(
                self.library, controller, WindStub((0.0, 0.0, 0.0)), path,
                airspeed_cmd=airspeed, altitude_cmd=1000.0,
                controller_rate_hz=50.0,
            )
            log = harness.run(seconds)
            t = np.array(log["t"])
            mask = t >= warmup
            cross = np.abs(np.array(log["cross_track"]))[mask]
            alt = np.array(log["alt"])[mask]
            tas = np.array(log["tas"])[mask]
            nan = bool(np.isnan(tas).any())
            diverged = (np.max(np.abs(np.array(log["phi"]))) > 80.0 * DEG)

            task_cost = float(np.mean(cross) + 0.05 * np.mean(np.abs(alt - 1000.0))
                              + 0.05 * np.mean(np.abs(tas - airspeed)))
            if nan or len(t) < 100:
                task_cost = 1e6
            elif diverged or np.min(np.array(log["alt"])) < 0.0:
                task_cost += 1500.0
            stats[name] = {
                "cross_mae_m": float(np.mean(cross)),
                "alt_rmse_m": float(np.sqrt(np.mean((alt - 1000.0) ** 2))),
                "tas_rmse_mps": float(np.sqrt(np.mean((tas - airspeed) ** 2))),
                "nan": nan,
            }
            total += weights[name] * task_cost
        return float(total), stats

    # ------------------------------------------------------------------ #
    def batch_benchmark(self, genes_batch: list[np.ndarray],
                        workers: int = 8) -> dict:
        """一批次种群的并行数据生成测试（并行 vs 串行吞吐对比）。

        同一批基因分别以串行与并行方式生成全部任务轨迹，
        统计 wall time、加速比与 sim-seconds / wall-second，
        验证批次并行 rollout 对后续数据采集的有效性。
        """
        n = len(genes_batch)

        start = time.perf_counter()
        for g in genes_batch:
            self.rollout(g)
        serial_wall = time.perf_counter() - start

        start = time.perf_counter()
        with ThreadPoolExecutor(max_workers=min(workers, n)) as pool:
            list(pool.map(self.rollout, genes_batch))
        parallel_wall = time.perf_counter() - start

        sim_seconds = n * (60.0 + 120.0 + 120.0)
        return {
            "batch_size": n,
            "workers": min(workers, n),
            "serial_wall_s": float(serial_wall),
            "parallel_wall_s": float(parallel_wall),
            "speedup": float(serial_wall / max(parallel_wall, 1e-6)),
            "sim_seconds_per_wall_second": float(sim_seconds / parallel_wall),
        }
