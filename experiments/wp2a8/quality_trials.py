#!/usr/bin/env python3
"""WP2A-8 控制品质试验 v3（问题整改版）。

整改内容：
1. 放大轨迹尺寸（8 字 R80/圆心距 400 双圆切线相接；跑马场 R80/L400），
   减小采样偏差影响——先不调任何参数，直接跑图像数据；
2. 8 字/跑马场提供两种导航变体：
   A. NPFG 采样路径制导（原方案，period/damping 保持 GA 整定值不变）；
   B. loiter 切圆 setpoint 导航（新方案：远离圆心直飞段入口点，
      进入 2R 内 loiter 切入，扫过 sweep 角切下一段，全张量化掩码状态机）；
3. 每项试验输出 5 张图：
   1) 航迹跟踪（2D）；2) xyz 期望位置跟踪；3) 体轴姿态角跟踪（PX4 内环量）；
   4) 体轴角速度跟踪（串级速率环）；5) 体轴速度 u/v/w 跟踪。

中文通过 mpl_style.setup_chinese_font() 保证无乱码。
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
    circle_path,
    figure8_loiter_segments,
    figure8_tangent_path,
    racetrack_loiter_segments,
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
              task_type: str = "path",
              loiter_segments: tuple | None = None) -> dict:
    print(f"  运行 {name}: {seconds:.0f}s ...")
    controller = AvantiController({"outer": cfg["outer"], "inner": cfg["inner"]})
    harness = ClosedLoopHarness(
        AvantiLibrary(), controller, wind, path,
        airspeed_cmd=airspeed, altitude_cmd=altitude,
        controller_rate_hz=50.0,
        task_type=task_type, loiter_segments=loiter_segments,
    )
    return harness.run(seconds)


def nearest_desired(path: np.ndarray, north: np.ndarray, east: np.ndarray) -> np.ndarray:
    """逐时刻最近路径点（期望 xyz 用，仅用于绘图）。"""
    pts = path[:, :2]
    desired = np.empty((len(north), 3))
    for i, (n, e) in enumerate(zip(north, east)):
        d = (pts[:, 0] - n) ** 2 + (pts[:, 1] - e) ** 2
        desired[i] = path[int(np.argmin(d))]
    return desired


def plot_fig_trajectory(name: str, log: dict, path: np.ndarray, out: Path) -> None:
    fig, ax = new_axes(1, 1, figsize=(8.0, 7.0))
    ax.plot(log["north"], log["east"], "b-", label="实际航迹")
    ax.plot(path[:, 0], path[:, 1], "r--", alpha=0.8, label="期望路径")
    ax.plot(log["north"][0], log["east"][0], "go", label="起点")
    ax.set_xlabel("North (m)")
    ax.set_ylabel("East (m)")
    ax.set_title(f"图1 航迹跟踪（{name}）")
    ax.legend(fontsize=8)
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    save_figure(fig, out)


def plot_fig_position_xyz(name: str, log: dict, path: np.ndarray, out: Path) -> None:
    t = log["t"]
    desired = nearest_desired(path, np.array(log["north"]), np.array(log["east"]))
    fig, axes = new_axes(3, 1, figsize=(9.0, 8.0))
    for ax, key, label in zip(axes, ("north", "east", "alt"),
                              ("North", "East", "Altitude")):
        ax.plot(t, log[key], "b-", label=f"实际 {label}")
        ax.plot(t, desired[:, ("north", "east", "alt").index(key)],
                "r--", alpha=0.8, label=f"期望 {label}")
        ax.set_ylabel(f"{label} (m)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
    axes[2].set_xlabel("时间 (s)")
    axes[0].set_title(f"图2 xyz 期望位置跟踪（{name}）")
    fig.tight_layout()
    save_figure(fig, out)


def plot_fig_attitude_body(name: str, log: dict, out: Path) -> None:
    t = log["t"]
    fig, axes = new_axes(3, 1, figsize=(9.0, 8.0))
    for ax, key, label in zip(axes, ("phi", "theta", "psi"),
                              ("滚转角 φ", "俯仰角 θ", "偏航角 ψ")):
        ax.plot(t, np.array(log[key]) / DEG, "b-", label=f"实际 {label}")
        ax.plot(t, np.array(log[f"{key}_cmd"]) / DEG, "r--", label=f"期望 {key}_cmd")
        ax.set_ylabel(f"{label} (°)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
    axes[2].set_xlabel("时间 (s)")
    axes[0].set_title(f"图3 体轴姿态角跟踪（{name}，PX4 内环量）")
    fig.tight_layout()
    save_figure(fig, out)


def plot_fig_rates_body(name: str, log: dict, out: Path) -> None:
    t = log["t"]
    fig, axes = new_axes(3, 1, figsize=(9.0, 8.0))
    for ax, key, sp_key, label in zip(axes, ("p", "q", "r"),
                                      ("p_sp", "q_sp", "r_sp"),
                                      ("滚转角速度 p", "俯仰角速度 q", "偏航角速度 r")):
        ax.plot(t, np.array(log[key]) / DEG, "b-", label=f"实际 {label}")
        ax.plot(t, np.array(log[sp_key]) / DEG, "r--", label=f"期望 {sp_key}")
        ax.set_ylabel(f"{label} (°/s)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
    axes[2].set_xlabel("时间 (s)")
    axes[0].set_title(f"图4 体轴角速度跟踪（{name}，串级速率环）")
    fig.tight_layout()
    save_figure(fig, out)


def plot_fig_velocity_body(name: str, log: dict, out: Path) -> None:
    t = log["t"]
    fig, axes = new_axes(3, 1, figsize=(9.0, 8.0))
    # 体轴速度 u/v/w：u 期望 = 空速指令（协调飞行下 v、w ≈ 0）
    for ax, key, label, cmd in zip(
        axes, ("u", "v", "w"), ("前向速度 u", "侧向速度 v", "法向速度 w"),
        (log["tas_cmd"], np.zeros(len(t)), np.zeros(len(t))),
    ):
        ax.plot(t, log[key], "b-", label=f"实际 {label}")
        ax.plot(t, cmd, "r--", alpha=0.8, label=f"期望 {key}_cmd")
        ax.set_ylabel(f"{label} (m/s)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
    axes[2].set_xlabel("时间 (s)")
    axes[0].set_title(f"图5 体轴速度 u/v/w 跟踪（{name}，TECS 空速 + β 阻尼）")
    fig.tight_layout()
    save_figure(fig, out)


def plot_trial(name: str, log: dict, path: np.ndarray,
               airspeed_cmd: float) -> dict:
    plot_fig_trajectory(name, log, path, OUT_DIR / f"trial_{name}_1_trajectory.png")
    plot_fig_position_xyz(name, log, path, OUT_DIR / f"trial_{name}_2_position_xyz.png")
    plot_fig_attitude_body(name, log, OUT_DIR / f"trial_{name}_3_attitude_body.png")
    plot_fig_rates_body(name, log, OUT_DIR / f"trial_{name}_4_rates_body.png")
    plot_fig_velocity_body(name, log, OUT_DIR / f"trial_{name}_5_velocity_body.png")

    t = np.array(log["t"])
    mask = t >= 20.0
    cross = np.array(log["cross_track"])

    def rmse(actual, target):
        return float(np.sqrt(np.mean((np.array(actual)[mask] - target) ** 2)))

    return {
        "trial": name,
        "duration_s": float(t[-1]),
        "cross_mae_m": float(np.mean(np.abs(cross[mask]))),
        "cross_rmse_m": float(np.sqrt(np.mean(cross[mask] ** 2))),
        "alt_rmse_m": rmse(log["alt"], 1000.0),
        "tas_rmse_mps": rmse(log["tas"], airspeed_cmd),
        "phi_mae_deg": float(np.mean(np.abs(
            (np.array(log["phi"]) - np.array(log["phi_cmd"]))[mask] / DEG
        ))),
        "theta_mae_deg": float(np.mean(np.abs(
            (np.array(log["theta"]) - np.array(log["theta_cmd"]))[mask] / DEG
        ))),
        "v_body_mae_mps": float(np.mean(np.abs(np.array(log["v"])[mask]))),
        "w_body_mae_mps": float(np.mean(np.abs(np.array(log["w"])[mask]))),
        "max_phi_deg": float(np.max(np.abs(np.array(log["phi"]))) / DEG),
        "nan": bool(np.isnan(np.array(log["tas"])).any()),
    }


def main() -> int:
    setup_chinese_font()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    cfg = load_tuned()

    print("== WP2A-8 品质试验 v3（整改：放大轨迹 / A-B 导航变体 / 5 图输出）==")
    stats_all: list[dict] = []

    def run_and_plot(name: str, path: np.ndarray, wind: WindStub, seconds: float,
                     airspeed: float = 16.0, altitude: float = 1000.0,
                     task_type: str = "path",
                     loiter_segments: tuple | None = None) -> None:
        log = run_trial(name, cfg, path, wind, airspeed, altitude, seconds,
                        task_type=task_type, loiter_segments=loiter_segments)
        stats_all.append(plot_trial(name, log, path, airspeed))

    # ---- T11 盘旋（无风/北风，GA 整定场景，基准不变） ----
    run_and_plot("T11_circle",
                 circle_path((0.0, 60.0), 60.0, 1000.0, n_pts=721, airspeed=16.0),
                 WindStub((0.0, 0.0, 0.0)), 120.0)
    run_and_plot("T11b_circle_wind",
                 circle_path((0.0, 60.0), 60.0, 1000.0, n_pts=721, airspeed=16.0),
                 WindStub((3.0, 0.0, 0.0)), 120.0)

    # ---- T12 8 字：R=80，圆心距 400（双圆 + 顶部公切线相接），V=16 ----
    # 中心 (0,-80)：顶部公切线过起点 (0,0)，起点位于切线中点
    f8_path = figure8_tangent_path((0.0, -80.0), 80.0, 1000.0,
                                   n_pts=1441, center_dist=400.0, airspeed=16.0)
    f8_seg = figure8_loiter_segments((0.0, -80.0), 80.0, center_dist=400.0)
    # 变体 A：NPFG 采样路径制导（参数保持整定值不变）
    run_and_plot("T12A_figure8_npfg", f8_path, WindStub((0.0, 0.0, 0.0)), 180.0)
    # 变体 B：loiter 切圆 setpoint 导航（参数不变，仅导航结构变化）
    run_and_plot("T12B_figure8_loiter", f8_path, WindStub((0.0, 0.0, 0.0)), 180.0,
                 task_type="figure8", loiter_segments=f8_seg)

    # ---- T13 跑马场：直线 L=400 + 半圆 R=80（长轴沿北、圆弧外凸修复后），V=16 ----
    # 中心 (0,80)：起点 (0,0) 位于西直线（east=0）上，航向切向朝北
    rt_path = racetrack_path((0.0, 80.0), 400.0, 80.0, 1000.0,
                             n_pts=1441, airspeed=16.0)
    rt_seg = racetrack_loiter_segments((0.0, 80.0), 400.0, 80.0)
    run_and_plot("T13A_racetrack_npfg", rt_path, WindStub((0.0, 0.0, 0.0)), 180.0)
    run_and_plot("T13B_racetrack_loiter", rt_path, WindStub((0.0, 0.0, 0.0)), 180.0,
                 task_type="racetrack", loiter_segments=rt_seg)

    summary = {
        "config": {
            "inner_source": "tuning_02_attitude_cascade_ga/result.json",
            "outer_source": "tuning_03_outer_circle_ga/result.json",
        },
        "note": ("试验 A（NPFG 采样路径）与 B（loiter 切圆 setpoint）均未调整参数；"
                 "8 字 R80/圆心距400（双圆切线相接，公切线过起点）、"
                 "跑马场 R80/L400（长轴沿北、圆弧外凸修复，起点在直线段切向）。"),
        "trials": stats_all,
    }
    (OUT_DIR / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print("\n== 统计 ==")
    for s in stats_all:
        print(f"  {s['trial']:22s} cross={s['cross_mae_m']:7.2f}m "
              f"alt={s['alt_rmse_m']:6.2f}m tas={s['tas_rmse_mps']:5.2f}m/s "
              f"v_body={s['v_body_mae_mps']:5.2f} w_body={s['w_body_mae_mps']:5.2f} "
              f"nan={s['nan']}")
    print(f"\n结果已保存到 {OUT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

