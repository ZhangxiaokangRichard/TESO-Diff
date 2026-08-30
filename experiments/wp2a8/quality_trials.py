#!/usr/bin/env python3
"""WP2A-8 控制品质试验 v4（轨迹修正 + 多任务整定参数）。

改动（v4）：
1. 轨迹修正：8 字改为空间 8 字（两个大弧 + 两条错高公切线，全路径连续）；
   跑马场保持长轴沿北/圆弧外凸版本；
2. 使用 tuning_04 三任务联合整定参数；
3. 输出一张 2×2 融合图（航迹 / xyz 期望位置 / 体轴姿态角 / 体轴角速率）；
   删除原图 5（体轴速度：TECS 已把空速作为前馈指令，体轴速度无独立价值）；
4. 旧试验图归档到 quality_trials/prevent_logs/。

中文通过 mpl_style.setup_chinese_font() 保证无乱码。
输出目录：experiments/wp2a8/quality_trials/
"""

from __future__ import annotations

import json
import shutil
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
    circle_path,
    figure8_spatial_path,
    racetrack_path,
)

OUT_DIR = Path(__file__).resolve().parent / "quality_trials"
PREVENT_DIR = OUT_DIR / "prevent_logs"
DEG = np.pi / 180.0


def load_tuned() -> dict:
    inner = json.load(open(
        PROJECT_ROOT / "experiments/wp2a8/tuning_02_attitude_cascade_ga/result.json"
    ))["inner_cfg"]
    outer = json.load(open(
        PROJECT_ROOT / "experiments/wp2a8/tuning_04_multitask_ga/result.json"
    ))["outer_cfg"]
    return {"inner": inner, "outer": outer}


def archive_old_figures() -> None:
    """把旧试验图与旧 summary 归档到 prevent_logs/。"""
    PREVENT_DIR.mkdir(parents=True, exist_ok=True)
    for item in OUT_DIR.iterdir():
        if item.is_file():
            shutil.move(str(item), PREVENT_DIR / item.name)
    print(f"  旧图已归档到 {PREVENT_DIR.relative_to(PROJECT_ROOT)}")


def nearest_desired(path: np.ndarray, north: np.ndarray, east: np.ndarray) -> np.ndarray:
    """逐时刻最近路径点（期望 xyz 用，仅用于绘图）。"""
    pts = path[:, :2]
    desired = np.empty((len(north), 3))
    for i, (n, e) in enumerate(zip(north, east)):
        d = (pts[:, 0] - n) ** 2 + (pts[:, 1] - e) ** 2
        desired[i] = path[int(np.argmin(d))]
    return desired


def plot_trial(name: str, title: str, log: dict, path: np.ndarray,
               out: Path) -> None:
    """输出单张 2×2 融合图：航迹 / xyz / 体轴姿态角 / 体轴角速率。"""
    t = log["t"]

    fig, axes = new_axes(2, 2, figsize=(12.5, 8.5))
    (ax_traj, ax_xyz), (ax_att, ax_rate) = axes

    # 1) 航迹（2D 俯视）
    ax_traj.plot(log["north"], log["east"], "b-", label="实际航迹")
    ax_traj.plot(path[:, 0], path[:, 1], "r--", alpha=0.8, label="期望路径")
    ax_traj.plot(log["north"][0], log["east"][0], "go", label="起点")
    ax_traj.set_xlabel("North (m)")
    ax_traj.set_ylabel("East (m)")
    ax_traj.set_title("航迹跟踪（俯视）")
    ax_traj.legend(fontsize=7)
    ax_traj.axis("equal")
    ax_traj.grid(True, alpha=0.3)

    # 2) xyz 期望位置跟踪
    desired = nearest_desired(path, np.array(log["north"]), np.array(log["east"]))
    for key, label in zip(("north", "east", "alt"), ("North", "East", "Altitude")):
        ax_xyz.plot(t, log[key], label=f"实际 {label}")
        ax_xyz.plot(t, desired[:, ("north", "east", "alt").index(key)],
                    "--", alpha=0.8, label=f"期望 {label}")
    ax_xyz.set_xlabel("时间 (s)")
    ax_xyz.set_ylabel("位置 (m)")
    ax_xyz.set_title("xyz 期望位置跟踪")
    ax_xyz.legend(fontsize=7, ncol=2)
    ax_xyz.grid(True, alpha=0.3)

    # 3) 体轴姿态角跟踪（PX4 内环量）
    for key, label in zip(("phi", "theta", "psi"), ("滚转角 φ", "俯仰角 θ", "偏航角 ψ")):
        ax_att.plot(t, np.array(log[key]) / DEG, label=f"{label}")
        ax_att.plot(t, np.array(log[f"{key}_cmd"]) / DEG, "--",
                    alpha=0.8, label=f"{key}_cmd")
    ax_att.set_xlabel("时间 (s)")
    ax_att.set_ylabel("姿态角 (°)")
    ax_att.set_title("体轴姿态角跟踪（PX4 内环）")
    ax_att.legend(fontsize=7, ncol=2)
    ax_att.grid(True, alpha=0.3)

    # 4) 体轴角速率跟踪（串级速率环）
    for key, label in zip(("p", "q", "r"), ("滚转 p", "俯仰 q", "偏航 r")):
        ax_rate.plot(t, np.array(log[key]) / DEG, label=f"{label}")
        ax_rate.plot(t, np.array(log[f"{key}_sp"]) / DEG, "--",
                     alpha=0.8, label=f"{key}_sp")
    ax_rate.set_xlabel("时间 (s)")
    ax_rate.set_ylabel("角速度 (°/s)")
    ax_rate.set_title("体轴角速度跟踪（串级速率环）")
    ax_rate.legend(fontsize=7, ncol=2)
    ax_rate.grid(True, alpha=0.3)

    fig.suptitle(f"{name}：{title}", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    save_figure(fig, out)


def stats_of(name: str, log: dict, airspeed_cmd: float) -> dict:
    t = np.array(log["t"])
    mask = t >= 20.0
    cross = np.abs(np.array(log["cross_track"]))[mask]
    alt = np.array(log["alt"])[mask]
    tas = np.array(log["tas"])[mask]

    def rmse(a, target):
        return float(np.sqrt(np.mean((a - target) ** 2)))

    return {
        "trial": name,
        "duration_s": float(t[-1]),
        "cross_mae_m": float(np.mean(cross)),
        "alt_rmse_m": rmse(alt, 1000.0),
        "tas_rmse_mps": rmse(tas, airspeed_cmd),
        "phi_mae_deg": float(np.mean(np.abs(
            (np.array(log["phi"]) - np.array(log["phi_cmd"]))[mask] / DEG
        ))),
        "theta_mae_deg": float(np.mean(np.abs(
            (np.array(log["theta"]) - np.array(log["theta_cmd"]))[mask] / DEG
        ))),
        "max_phi_deg": float(np.max(np.abs(np.array(log["phi"]))) / DEG),
        "nan": bool(np.isnan(tas).any()),
    }


def main() -> int:
    setup_chinese_font()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    archive_old_figures()
    cfg = load_tuned()

    print("== WP2A-8 品质试验 v4（空间 8 字 + 多任务整定参数）==")
    stats_all: list[dict] = []

    def run_and_plot(name: str, title: str, path: np.ndarray, wind: WindStub,
                     seconds: float, airspeed: float = 16.0) -> None:
        print(f"  运行 {name}: {seconds:.0f}s ...")
        controller = AvantiController({"outer": cfg["outer"], "inner": cfg["inner"]})
        harness = ClosedLoopHarness(
            AvantiLibrary(), controller, wind, path,
            airspeed_cmd=airspeed, altitude_cmd=1000.0,
            controller_rate_hz=50.0,
        )
        log = harness.run(seconds)
        plot_trial(name, title, log, path, OUT_DIR / f"trial_{name}.png")
        stats_all.append(stats_of(name, log, airspeed))

    # T11 盘旋（无风 / 北风 3 m/s，NPFG）
    run_and_plot("T11_circle", "盘旋 R=60（无风）",
                 circle_path((0.0, 60.0), 60.0, 1000.0, n_pts=721, airspeed=16.0),
                 WindStub((0.0, 0.0, 0.0)), 120.0)
    run_and_plot("T11b_circle_wind", "盘旋 R=60（北风 3 m/s）",
                 circle_path((0.0, 60.0), 60.0, 1000.0, n_pts=721, airspeed=16.0),
                 WindStub((3.0, 0.0, 0.0)), 120.0)

    # T12 空间 8 字：R=80，圆心距 400，左圆高于右圆 10 m
    f8 = figure8_spatial_path((0.0, 0.0), 80.0, 1000.0, n_pts=1441,
                              center_dist=400.0, center_dh=10.0, airspeed=16.0)
    run_and_plot("T12_figure8_spatial", "空间 8 字 R=80/D=400（左圆高 10m）",
                 f8, WindStub((0.0, 0.0, 0.0)), 180.0)

    # T13 跑马场：直线 L=400 + 半圆 R=80（长轴沿北，圆弧外凸）
    rt = racetrack_path((0.0, 80.0), 400.0, 80.0, 1000.0,
                        n_pts=1441, airspeed=16.0)
    run_and_plot("T13_racetrack", "跑马场 L=400/R=80（长轴沿北）",
                 rt, WindStub((0.0, 0.0, 0.0)), 180.0)

    summary = {
        "config": {
            "inner_source": "tuning_02_attitude_cascade_ga/result.json",
            "outer_source": "tuning_04_multitask_ga/result.json",
        },
        "note": ("v4：空间 8 字（两圆圆心高度差 10m、切点 ±5m 错高、双公切线全连续）、"
                 "跑马场长轴沿北；loiter setpoint 导航已删除；"
                 "图 5（体轴速度）删除，前 4 图融合为一张 2×2 图。"),
        "trials": stats_all,
    }
    (OUT_DIR / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print("\n== 统计 ==")
    for s in stats_all:
        print(f"  {s['trial']:20s} cross={s['cross_mae_m']:7.2f}m "
              f"alt={s['alt_rmse_m']:6.2f}m tas={s['tas_rmse_mps']:5.2f}m/s "
              f"phi_mae={s['phi_mae_deg']:5.2f}° nan={s['nan']}")
    print(f"\n结果已保存到 {OUT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

