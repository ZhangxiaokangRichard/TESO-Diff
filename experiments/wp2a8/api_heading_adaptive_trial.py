#!/usr/bin/env python3
"""WP2A-8：自适应路径采样 → HeadingCommand 的 400 s 独立试验。"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from mpl_style import new_axes, save_figure, setup_chinese_font  # noqa: E402
from python.controller import (  # noqa: E402
    LegacySurfaceFlightController as FlightController,
    HeadingCommand,
    PolylinePathFollower,
)
from python.controller.types import ControlObservation  # noqa: E402
from quality_trials import (  # noqa: E402
    OUT_DIR,
    _flat_figure8_waypoints,
    load_tuned,
    plot_trial,
    run_api_trial,
    stats_of,
)


def main() -> int:
    setup_chinese_font()
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    cfg = load_tuned()

    center = (200.0, 0.0)
    radius = 80.0
    altitude = 1000.0
    airspeed = 16.0
    path = _flat_figure8_waypoints(center, radius, altitude, n_per_circle=100)

    follower = PolylinePathFollower(
        path,
        airspeed=airspeed,
        gamma=0.0,
        closed=True,
        lookahead_time_s=2.5,
        lookahead_min_m=35.0,
        lookahead_max_m=55.0,
        cross_track_gain=1.0,
        heading_rate_limit_degps=18.0,
        update_period_s=0.1,
    )
    follower.reset(initial_segment=0, initial_chi=0.0)

    controller = FlightController({"outer": cfg["outer"], "inner": cfg["inner"]})
    controller.reset(1)
    controller.command_heading(HeadingCommand(
        airspeed=airspeed, gamma=0.0, chi=0.0, timestamp=0.0
    ))

    def update_heading(
        fc: FlightController, t: float, obs: ControlObservation
    ) -> dict[str, float]:
        ground_speed = float(np.linalg.norm(obs.neu_vel[0, :2]))
        fc.command_heading(follower.update(obs.neu_pos[0], t, ground_speed))
        state = follower.status()
        return {
            "path_segment_index": state.segment_index,
            "path_lap_count": state.lap_count,
            "path_progress_m": state.progress_m,
            "path_lookahead_m": state.lookahead_m,
            "path_cross_track_m": state.signed_cross_track_m,
            "path_target_north": state.target_neu[0],
            "path_target_east": state.target_neu[1],
            "path_target_alt": state.target_neu[2],
            "chi_cmd_raw": state.chi_raw,
            "chi_cmd_limited": state.chi_command,
            "path_command_updated": float(state.updated),
        }

    print("运行 T15_api_heading_figure8_adaptive: 400 s ...")
    log = run_api_trial(
        controller,
        step_hook=update_heading,
        seconds=400.0,
        controller_rate_hz=50.0,
        airspeed_cmd=airspeed,
    )

    image_path = OUT_DIR / "trial_T15_api_heading_figure8_adaptive.png"
    plot_trial(
        "T15_api_heading_figure8_adaptive",
        "自适应弧长前视 → API Heading（无 Dubins）",
        log,
        path,
        image_path,
    )

    stats = stats_of("T15_api_heading_figure8_adaptive", log, airspeed)
    t = np.asarray(log["t"])
    mask = t >= 20.0
    chi = np.unwrap(np.asarray(log["chi_cmd_limited"]))
    progress = np.asarray(log["path_progress_m"])
    update_flags = np.asarray(log["path_command_updated"])
    update_idx = np.flatnonzero(update_flags > 0.5)
    chi_update = chi[update_idx]
    t_update = t[update_idx]
    chi_update_rate = np.diff(chi_update) / np.diff(t_update)
    chi_tick_step = np.diff(chi)
    stats.update({
        "completed_laps": int(np.nanmax(log["path_lap_count"])),
        "final_progress_m": float(progress[-1]),
        "progress_monotonic": bool(np.all(np.diff(progress) >= -1e-9)),
        "lookahead_mean_m": float(np.nanmean(np.asarray(log["path_lookahead_m"])[mask])),
        "chi_update_rate_max_degps": float(
            np.nanmax(np.abs(chi_update_rate)) * 180.0 / np.pi
        ),
        "chi_tick_step_max_deg": float(
            np.nanmax(np.abs(chi_tick_step)) * 180.0 / np.pi
        ),
        "command_updates": int(np.nansum(update_flags)),
        "altitude_drift_m": float(log["alt"][-1] - log["alt"][0]),
    })

    summary_path = OUT_DIR / "summary_T15_api_heading_adaptive.json"
    summary_path.write_text(
        json.dumps({
            "controller": {
                "inner_source": "tuning_02_attitude_cascade_ga/result.json",
                "outer_source": "tuning_04_multitask_ga/result.json",
            },
            "path_follower": {
                "method": "local segment projection + monotonic arc progress + arc lookahead",
                "dubins": False,
                "lookahead_time_s": 2.5,
                "lookahead_min_m": 35.0,
                "lookahead_max_m": 55.0,
                "cross_track_gain": 1.0,
                "heading_rate_limit_degps": 18.0,
                "update_period_s": 0.1,
            },
            "trial": stats,
        }, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    np.savez_compressed(OUT_DIR / "T15_api_heading_figure8_adaptive_log.npz", **log)

    fig, axes = new_axes(2, 2, figsize=(12.5, 8.5))
    (ax_cross, ax_progress), (ax_chi, ax_lookahead) = axes
    ax_cross.plot(t, log["path_cross_track_m"], linewidth=0.8)
    ax_cross.axhline(0.0, color="k", linewidth=0.6)
    ax_cross.set_title("有符号横侧误差（原始 8 字路径）")
    ax_cross.set_xlabel("时间 (s)")
    ax_cross.set_ylabel("横侧误差 (m)")
    ax_cross.grid(True, alpha=0.3)

    ax_progress.plot(t, log["path_progress_m"], label="累计弧长")
    ax_progress.set_title("单调路径进度")
    ax_progress.set_xlabel("时间 (s)")
    ax_progress.set_ylabel("累计弧长 (m)")
    ax_progress.grid(True, alpha=0.3)

    ax_chi.plot(t, np.unwrap(log["chi_cmd_raw"]) / np.pi * 180.0,
                label="原始前视指令", alpha=0.8)
    ax_chi.plot(t, np.unwrap(log["chi_cmd_limited"]) / np.pi * 180.0,
                "--", label="18°/s 限速后", alpha=0.9)
    ax_chi.set_title("连续化航迹偏角指令")
    ax_chi.set_xlabel("时间 (s)")
    ax_chi.set_ylabel("展开角度 (°)")
    ax_chi.legend(fontsize=8)
    ax_chi.grid(True, alpha=0.3)

    ax_lookahead.plot(t, log["path_lookahead_m"], label="前视距离")
    ax_lookahead.set_title("速度自适应前视距离")
    ax_lookahead.set_xlabel("时间 (s)")
    ax_lookahead.set_ylabel("前视距离 (m)")
    ax_lookahead.set_ylim(30.0, 60.0)
    ax_lookahead.grid(True, alpha=0.3)

    fig.suptitle("T15 自适应路径取样诊断（无 Dubins）", fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    guidance_path = OUT_DIR / "trial_T15_api_heading_figure8_adaptive_guidance.png"
    save_figure(fig, guidance_path)

    print(json.dumps(stats, ensure_ascii=False, indent=2))
    print(f"图像: {image_path}")
    print(f"制导诊断: {guidance_path}")
    print(f"摘要: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
