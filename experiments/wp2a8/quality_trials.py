#!/usr/bin/env python3
"""WP2A-8 控制品质试验 v4（轨迹修正 + 多任务整定参数）+ T14/T15 API 层试验。

v4 改动：
1. 轨迹修正：8 字改为空间 8 字（两个大弧 + 两条错高公切线，全路径连续）；
   跑马场保持长轴沿北/圆弧外凸版本；
2. 使用 tuning_04 三任务联合整定参数；
3. 输出一张 2×2 融合图（航迹 / xyz 期望位置 / 体轴姿态角 / 体轴角速率）；
   删除原图 5（体轴速度：TECS 已把空速作为前馈指令，体轴速度无独立价值）；
4. 旧试验图归档到 quality_trials/prevent_logs/。

T14/T15 新增（API 层）：
- T14：FlightController.command_loiter() 指点盘旋，400 s；
- T15：PolylinePathFollower 自适应前视 → HeadingCommand，400 s。

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
from python.bindings import AvantiInput, AvantiLibrary  # noqa: E402
from python.controller import AvantiController  # noqa: E402
from python.controller.controllerAPI import (  # noqa: E402
    LegacySurfaceFlightController as FlightController,
    HeadingCommand,
    LoiterCommand,
    PolylinePathFollower,
)
from python.controller.types import ControlObservation  # noqa: E402
from python.simulation import ClosedLoopHarness, WindStub  # noqa: E402
from python.simulation.harness import LOG_FIELDS  # noqa: E402
from python.simulation.task_route import (  # noqa: E402
    circle_path,
    figure8_spatial_path,
    racetrack_path,
)

OUT_DIR = Path(__file__).resolve().parent / "quality_trials"
PREVENT_DIR = OUT_DIR / "prevent_logs"
DEG = np.pi / 180.0
PATH_DIAG_FIELDS = (
    "path_segment_index", "path_lap_count", "path_progress_m",
    "path_lookahead_m", "path_cross_track_m",
    "path_target_north", "path_target_east", "path_target_alt",
    "chi_cmd_raw", "chi_cmd_limited", "path_command_updated",
)


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


# ---------------------------------------------------------------------------
# API 层试验辅助
# ---------------------------------------------------------------------------

def _flat_figure8_waypoints(
    center_ne: tuple[float, float],
    radius: float,
    altitude: float,
    n_per_circle: int = 100,
) -> np.ndarray:
    """生成平面 8 字导航航点 [N, 3]（NEU）。

    由两个相切圆组成（touching point 即 center_ne）：
    - 左圆圆心 (cx-R, cy)，逆时针遍历；
    - 右圆圆心 (cx+R, cy)，顺时针遍历；
    - 两圆在 center_ne 相切，形成 8 字。
    """
    cx, cy = center_ne
    R = float(radius)
    n = n_per_circle
    theta_l = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    north_l = (cx - R) + R * np.cos(theta_l)
    east_l = cy + R * np.sin(theta_l)
    theta_r = np.linspace(np.pi, -np.pi, n, endpoint=False)
    north_r = (cx + R) + R * np.cos(theta_r)
    east_r = cy + R * np.sin(theta_r)
    north = np.concatenate([north_l, north_r])
    east = np.concatenate([east_l, east_r])
    alt = np.full(n * 2, float(altitude))
    return np.stack([north, east, alt], axis=1)


def run_api_trial(
    fc: FlightController,
    step_hook,
    seconds: float = 400.0,
    wind_neu: tuple[float, float] = (0.0, 0.0),
    dt_plant: float = 0.02,
    controller_rate_hz: float = 50.0,
    airspeed_cmd: float = 16.0,
) -> dict:
    """FlightController 闭环仿真循环，返回与 harness.LOG_FIELDS 兼容的日志字典。

    参数
    ----
    fc : FlightController
        已重置并通过 command_loiter/command_heading 发出初始指令的控制器。
    step_hook : callable(fc, t, obs) | None
        每个控制步调用，用于动态更新 HeadingCommand（8 字航点跟踪）。
        Loiter 模式传 None。
    """
    log: dict[str, list] = {f: [] for f in (*LOG_FIELDS, *PATH_DIAG_FIELDS)}
    ctrl_interval = max(1, int(round(1.0 / controller_rate_hz / dt_plant)))
    n_steps = int(round(seconds / dt_plant))
    wind_flat = np.array([wind_neu[0], wind_neu[1], 0.0])
    wind_est = np.array([wind_neu[0], wind_neu[1]])

    library = AvantiLibrary()
    with library.model(epoch_s=0.0) as model:
        init_inp = AvantiInput(throttle_01=0.5)
        output = model.tick(init_inp)

        last_cmd = None
        last_inter: dict = {}
        last_diag: dict = {}

        for k in range(n_steps):
            t = k * dt_plant
            obs = ControlObservation.from_fw_outputs([output])
            obs.wind_neu = wind_est[None, :]

            if k % ctrl_interval == 0:
                if step_hook is not None:
                    diag = step_hook(fc, t, obs)
                    if diag is not None:
                        last_diag = diag
                last_cmd = fc.step(obs)
                last_inter = fc.intermediates()

            log["t"].append(t)
            log["north"].append(obs.neu_pos[0, 0])
            log["east"].append(obs.neu_pos[0, 1])
            log["alt"].append(obs.neu_pos[0, 2])
            log["vn"].append(obs.neu_vel[0, 0])
            log["ve"].append(obs.neu_vel[0, 1])
            log["vu"].append(obs.neu_vel[0, 2])
            log["phi"].append(obs.phi[0])
            log["theta"].append(obs.theta[0])
            log["psi"].append(obs.psi[0])
            log["p"].append(obs.p[0])
            log["q"].append(obs.q[0])
            log["r"].append(obs.r[0])
            log["u"].append(obs.u[0])
            log["v"].append(obs.v[0])
            log["w"].append(obs.w[0])
            log["tas"].append(obs.tas[0])
            log["tas_cmd"].append(airspeed_cmd)
            log["alpha"].append(obs.alpha[0])
            log["beta"].append(obs.beta[0])

            def _g(key: str) -> float:
                arr = last_inter.get(key)
                return float(arr[0]) if arr is not None else 0.0

            log["phi_cmd"].append(_g("phi_cmd"))
            log["theta_cmd"].append(_g("theta_cmd"))
            log["psi_cmd"].append(_g("psi_cmd"))
            log["throttle_cmd"].append(_g("throttle_cmd"))
            true_cross = last_diag.get("path_cross_track_m")
            log["cross_track"].append(
                abs(float(true_cross)) if true_cross is not None
                else _g("cross_track")
            )
            log["p_sp"].append(_g("p_sp"))
            log["q_sp"].append(_g("q_sp"))
            log["r_sp"].append(_g("r_sp"))
            log["delta_a"].append(float(last_cmd.delta_a[0]) if last_cmd else 0.0)
            log["delta_e"].append(float(last_cmd.delta_e[0]) if last_cmd else 0.0)
            log["delta_r"].append(float(last_cmd.delta_r[0]) if last_cmd else 0.0)
            log["wind_x"].append(float(wind_flat[0]))
            log["wind_y"].append(float(wind_flat[1]))
            log["wind_z"].append(float(wind_flat[2]))
            for key in PATH_DIAG_FIELDS:
                log[key].append(float(last_diag.get(key, np.nan)))

            inp = AvantiInput()
            inp.delta_e_norm = float(last_cmd.delta_e[0]) if last_cmd else 0.0
            inp.delta_a_norm = float(last_cmd.delta_a[0]) if last_cmd else 0.0
            inp.delta_r_norm = float(last_cmd.delta_r[0]) if last_cmd else 0.0
            inp.throttle_01 = float(last_cmd.throttle[0]) if last_cmd else 0.5
            inp.set_wind((float(wind_flat[0]), float(wind_flat[1]), float(wind_flat[2])))
            output = model.tick(inp)

    return {k: np.array(v) for k, v in log.items() if len(v) > 0}


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

    # ------------------------------------------------------------------
    # T14  指点盘旋（FlightController.command_loiter API）
    # ------------------------------------------------------------------
    print("  运行 T14_api_loiter: 400s ...")
    fc14 = FlightController({"outer": cfg["outer"], "inner": cfg["inner"]})
    fc14.reset(1)
    # 圆心在初始位置正北 200 m，半径 60 m；L1 自动检测圆形路径
    fc14.command_loiter(LoiterCommand(
        center=(200.0, 0.0, 1000.0), radius=60.0, airspeed=16.0, timestamp=0.0
    ))
    log14 = run_api_trial(fc14, step_hook=None, seconds=400.0, airspeed_cmd=16.0)
    loiter_ref = circle_path((200.0, 0.0), 60.0, 1000.0, n_pts=721, airspeed=16.0)
    plot_trial("T14_api_loiter", "FlightController 指点盘旋（API Loiter）",
               log14, loiter_ref, OUT_DIR / "trial_T14_api_loiter.png")
    stats_all.append(stats_of("T14_api_loiter", log14, 16.0))

    # ------------------------------------------------------------------
    # T15  8 字飞行（自适应弧长前视 + FlightController.command_heading）
    # ------------------------------------------------------------------
    print("  运行 T15_api_heading_figure8: 400s ...")
    # 8 字圆心在初始位置正北 200 m，半径 80 m；飞机初始向北飞行直达 touching point
    F8_CENTER = (200.0, 0.0)
    F8_R = 80.0
    F8_ALT = 1000.0
    wps = _flat_figure8_waypoints(F8_CENTER, F8_R, F8_ALT, n_per_circle=100)
    follower = PolylinePathFollower(
        wps,
        airspeed=16.0,
        gamma=0.0,
        closed=True,
        lookahead_time_s=2.5,
        lookahead_min_m=35.0,
        lookahead_max_m=55.0,
        cross_track_gain=1.0,
        heading_rate_limit_degps=18.0,
        update_period_s=0.1,
    )
    # 8 字交叉点有两个几何分支；显式指定第 0 段并保留单调弧长进度。
    follower.reset(initial_segment=0, initial_chi=0.0)

    def _t15_hook(fc: FlightController, t: float, obs: ControlObservation) -> dict:
        gs = float(np.linalg.norm(obs.neu_vel[0, :2]))
        fc.command_heading(follower.update(obs.neu_pos[0], t, gs))
        status = follower.status()
        return {
            "path_segment_index": status.segment_index,
            "path_lap_count": status.lap_count,
            "path_progress_m": status.progress_m,
            "path_lookahead_m": status.lookahead_m,
            "path_cross_track_m": status.signed_cross_track_m,
            "path_target_north": status.target_neu[0],
            "path_target_east": status.target_neu[1],
            "path_target_alt": status.target_neu[2],
            "chi_cmd_raw": status.chi_raw,
            "chi_cmd_limited": status.chi_command,
            "path_command_updated": float(status.updated),
        }

    fc15 = FlightController({"outer": cfg["outer"], "inner": cfg["inner"]})
    fc15.reset(1)
    # 初始指令由适配器在 t=0 生成；先提供安全的向北保持值。
    fc15.command_heading(HeadingCommand(airspeed=16.0, gamma=0.0, chi=0.0, timestamp=0.0))
    log15 = run_api_trial(fc15, step_hook=_t15_hook, seconds=400.0, airspeed_cmd=16.0)
    plot_trial("T15_api_heading_figure8_adaptive",
               "自适应弧长前视 → API Heading（无 Dubins）",
               log15, wps, OUT_DIR / "trial_T15_api_heading_figure8_adaptive.png")
    stats15 = stats_of("T15_api_heading_figure8_adaptive", log15, 16.0)
    stats15.update({
        "completed_laps": int(np.nanmax(log15["path_lap_count"])),
        "lookahead_mean_m": float(np.nanmean(log15["path_lookahead_m"])),
    })
    stats_all.append(stats15)

    summary = {
        "config": {
            "inner_source": "tuning_02_attitude_cascade_ga/result.json",
            "outer_source": "tuning_04_multitask_ga/result.json",
        },
        "note": ("v4：空间 8 字（两圆圆心高度差 10m、切点 ±5m 错高、双公切线全连续）、"
                 "跑马场长轴沿北；loiter setpoint 导航已删除；"
                 "图 5（体轴速度）删除，前 4 图融合为一张 2×2 图。"
                 "T14 = FlightController.command_loiter() 指点盘旋 400s；"
                 "T15 = PolylinePathFollower 自适应弧长前视输出 HeadingCommand，"
                 "不使用 Dubins，400s。"),
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

