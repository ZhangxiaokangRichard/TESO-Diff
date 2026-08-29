#!/usr/bin/env python3
"""WP2A-8 控制品质试验：3 条封闭航迹 + 风场抗扰验证（docs §6.14 T11~T13）。

使用 tuning_02（串级内环）+ tuning_03（外环 NPFG/TECS）整定参数：
    T11-circle   盘旋（无风）            120 s
    T11-circle_w 盘旋（北风 3 m/s，NPFG） 120 s
    T12-figure8  8 字盘旋                180 s
    T13-racetrack 跑马场                 180 s

每项试验输出 2×2 图：
    1. 航迹跟踪（2D 俯视 + 期望路径）
    2. 姿态角跟踪（φ/θ/ψ 与指令）
    3. 角速度跟踪（p/q/r 与指令 p_sp/q_sp/r_sp）
    4. 速度跟踪（空速 + NEU 速度分量与指令）

图像中文通过 mpl_style.setup_chinese_font() 保证无乱码。
输出目录：experiments/wp2a8/quality_trials/
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from mpl_style import new_axes, save_figure, setup_chinese_font  # noqa: E402
from python.bindings import AvantiLibrary  # noqa: E402
from python.controller import AvantiController  # noqa: E402
from python.simulation import ClosedLoopHarness, WindStub  # noqa: E402
from python.simulation.task_route import (  # noqa: E402
    build_task_path,
    circle_path,
    figure8_path,
    racetrack_path,
)

OUT_DIR = Path(__file__).resolve().parent / "quality_trials"
DEG = np.pi / 180.0


def load_tuned() -> dict:
    inner = json.load(open(
        PROJECT_ROOT / "experiments/wp2a8/tuning_02_attitude_cascade_ga/result.json"
    ))["inner_cfg"]
    outer = json.load(open(
        PROJECT_ROOT / "experiments/wp2a8/tuning_03_outer_circle_ga/result.json"
    ))["outer_cfg"]
    return {"inner": inner, "outer": outer}


def run_trial(name: str, cfg: dict, path: np.ndarray, wind: WindStub,
              airspeed: float, altitude: float, seconds: float,
              l1_period: float | None = None, l1_damping: float | None = None,
              bank_limit_deg: float | None = None) -> dict:
    print(f"  运行 {name}: {seconds:.0f}s ...")
    outer = json.loads(json.dumps(cfg["outer"]))  # 深拷贝，支持每试验覆盖
    if l1_period is not None:
        outer["l1"]["period_s"] = l1_period
    if l1_damping is not None:
        outer["l1"]["damping"] = l1_damping
    if bank_limit_deg is not None:
        outer["coordinated_turn"]["bank_limit_deg"] = bank_limit_deg
    controller = AvantiController({"outer": outer, "inner": cfg["inner"]})
    harness = ClosedLoopHarness(
        AvantiLibrary(), controller, wind, path,
        airspeed_cmd=airspeed, altitude_cmd=altitude,
        controller_rate_hz=50.0,
    )
    log = harness.run(seconds)
    return log


def plot_trial(name: str, title: str, log: dict, path: np.ndarray,
               out_dir: Path, airspeed_cmd: float = 16.0) -> dict:
    t = log["t"]
    mask = t >= 20.0  # 捕获段不计入统计

    fig, axes = new_axes(2, 2, figsize=(12.5, 8.0))
    (ax_traj, ax_att), (ax_rate, ax_vel) = axes

    # 1) 航迹
    ax_traj.plot(log["north"], log["east"], "b-", label="实际航迹")
    ax_traj.plot(path[:, 0], path[:, 1], "r--", alpha=0.8, label="期望路径")
    ax_traj.plot(log["north"][0], log["east"][0], "go", label="起点")
    ax_traj.set_xlabel("North (m)")
    ax_traj.set_ylabel("East (m)")
    ax_traj.set_title(f"航迹跟踪（{title}）")
    ax_traj.legend(fontsize=8)
    ax_traj.axis("equal")
    ax_traj.grid(True, alpha=0.3)

    # 2) 姿态角跟踪
    ax_att.plot(t, np.array(log["phi_cmd"]) / DEG, "r--", label="φ_cmd")
    ax_att.plot(t, np.array(log["phi"]) / DEG, "b-", label="φ")
    ax_att.plot(t, np.array(log["theta_cmd"]) / DEG, "m--", label="θ_cmd")
    ax_att.plot(t, np.array(log["theta"]) / DEG, "c-", label="θ")
    ax_att.plot(t, np.array(log["psi_cmd"]) / DEG, "y--", label="ψ_cmd")
    ax_att.plot(t, np.array(log["psi"]) / DEG, "g-", label="ψ")
    ax_att.set_xlabel("时间 (s)")
    ax_att.set_ylabel("姿态角 (°)")
    ax_att.set_title("姿态角跟踪")
    ax_att.legend(fontsize=8, ncol=3)
    ax_att.grid(True, alpha=0.3)

    # 3) 角速度跟踪
    ax_rate.plot(t, np.array(log["p_sp"]) / DEG, "r--", label="p_sp")
    ax_rate.plot(t, np.array(log["p"]) / DEG, "b-", label="p")
    ax_rate.plot(t, np.array(log["q_sp"]) / DEG, "m--", label="q_sp")
    ax_rate.plot(t, np.array(log["q"]) / DEG, "c-", label="q")
    ax_rate.plot(t, np.array(log["r_sp"]) / DEG, "y--", label="r_sp")
    ax_rate.plot(t, np.array(log["r"]) / DEG, "g-", label="r")
    ax_rate.set_xlabel("时间 (s)")
    ax_rate.set_ylabel("角速度 (°/s)")
    ax_rate.set_title("角速度跟踪（串级内环）")
    ax_rate.legend(fontsize=8, ncol=3)
    ax_rate.grid(True, alpha=0.3)

    # 4) 速度跟踪
    ax_vel.plot(t, log["tas"], "b-", label="空速 TAS")
    ax_vel.plot(t, log["tas_cmd"], "r--", label="空速指令")
    ax_vel.plot(t, log["vn"], "c-", alpha=0.7, label="vn（北）")
    ax_vel.plot(t, log["ve"], "m-", alpha=0.7, label="ve（东）")
    ax_vel.plot(t, log["vu"], "g-", alpha=0.7, label="vu（天）")
    ax_vel.set_xlabel("时间 (s)")
    ax_vel.set_ylabel("速度 (m/s)")
    ax_vel.set_title("速度跟踪")
    ax_vel.legend(fontsize=8, ncol=3)
    ax_vel.grid(True, alpha=0.3)

    fig.tight_layout()
    out = out_dir / f"trial_{name}.png"
    save_figure(fig, out)

    # 统计
    def rmse(actual, target):
        return float(np.sqrt(np.mean((np.array(actual)[mask] - target) ** 2)))

    cross = np.array(log["cross_track"])
    stats = {
        "trial": name,
        "duration_s": float(t[-1]),
        "cross_mae_m": float(np.mean(np.abs(cross[mask]))),
        "cross_rmse_m": float(np.sqrt(np.mean(cross[mask] ** 2))),
        "alt_rmse_m": rmse(log["alt"], 1000.0),
        "tas_rmse_mps": rmse(log["tas"], airspeed_cmd),
        "phi_track_mae_deg": float(np.mean(np.abs(
            (np.array(log["phi"]) - np.array(log["phi_cmd"]))[mask] / DEG
        ))),
        "theta_track_mae_deg": float(np.mean(np.abs(
            (np.array(log["theta"]) - np.array(log["theta_cmd"]))[mask] / DEG
        ))),
        "max_phi_deg": float(np.max(np.abs(np.array(log["phi"]))) / DEG),
        "nan": bool(np.isnan(np.array(log["tas"])).any()),
    }
    return stats


def main() -> int:
    setup_chinese_font()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    cfg = load_tuned()

    print("== WP2A-8 控制品质试验（PX4 串级 + NPFG/TECS 整定参数）==")
    stats_all: list[dict] = []

    def run_and_plot(name: str, title: str, path: np.ndarray, wind: WindStub,
                     seconds: float, airspeed: float = 16.0,
                     altitude: float = 1000.0, **overrides) -> None:
        log = run_trial(name, cfg, path, wind, airspeed, altitude,
                        seconds, **overrides)
        stats_all.append(plot_trial(name, title, log, path, OUT_DIR,
                                    airspeed_cmd=airspeed))

    # T11 盘旋（无风）：GA 整定场景，圆心 (0,60)，飞机初始即在圆周切向
    run_and_plot("T11_circle", "盘旋（无风）",
                 circle_path((0.0, 60.0), 60.0, 1000.0, n_pts=721, airspeed=16.0),
                 WindStub((0.0, 0.0, 0.0)), 120.0)

    # T11b 盘旋（北风 3 m/s，验证 NPFG 风三角）
    run_and_plot("T11b_circle_wind", "盘旋（北风 3 m/s，NPFG）",
                 circle_path((0.0, 60.0), 60.0, 1000.0, n_pts=721, airspeed=16.0),
                 WindStub((3.0, 0.0, 0.0)), 120.0)

    # T12 8 字（R=80，V=15：最小曲率半径≈26.7m，坡度需求≈41°，限幅 45°）
    run_and_plot("T12_figure8", "8 字盘旋（R=80m，V=15m/s）",
                 figure8_path((0.0, 80.0), 80.0, 1000.0, n_pts=1441, airspeed=15.0),
                 WindStub((0.0, 0.0, 0.0)), 150.0,
                 airspeed=15.0, l1_period=8.0, l1_damping=0.75,
                 bank_limit_deg=45.0)

    # T13 跑马场（直线 160m + 半圆 R=60m，V=15：坡度需求≈21°，限幅 35°）
    run_and_plot("T13_racetrack", "跑马场（L=160m，R=60m，V=15m/s）",
                 racetrack_path((0.0, 40.0), 160.0, 60.0, 1000.0,
                                n_pts=1441, airspeed=15.0),
                 WindStub((0.0, 0.0, 0.0)), 150.0,
                 airspeed=15.0, l1_period=8.0, l1_damping=0.75,
                 bank_limit_deg=35.0)

    summary = {
        "config": {
            "inner_source": "tuning_02_attitude_cascade_ga/result.json",
            "outer_source": "tuning_03_outer_circle_ga/result.json",
        },
        "trials": stats_all,
    }
    (OUT_DIR / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print("\n== 统计 ==")
    for stats in stats_all:
        print(f"  {stats['trial']:16s} cross_mae={stats['cross_mae_m']:6.2f}m "
              f"alt_rmse={stats['alt_rmse_m']:6.2f}m tas_rmse={stats['tas_rmse_mps']:5.2f}m/s "
              f"phi_mae={stats['phi_track_mae_deg']:5.2f}° nan={stats['nan']}")
    print(f"\n结果已保存到 {OUT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
