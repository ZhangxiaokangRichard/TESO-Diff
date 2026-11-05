"""WP2A-endpoint 驱动：四科目（三维）× W0/W1/W2 闭环飞行 + avanti_eso 张量采集与验证。

每个科目创建一个 batch=3 的 AttitudeFleet，三个实例分别处于 W0/W1/W2 固定风场；
风按各实例自身位置逐 tick 查询（已知风真值，无估计噪声）。航迹经
PolylinePathFollower(altitude_follow=True) → HeadingCommand（水平航向 + 路径
斜率对应的航迹倾角），50 Hz，dt=0.02 s。

输出（每个 科目×风场 一条 run）：
    data/eso_<subject>_<W>.npz         avanti_eso 张量数据集（见 dataset_structure.md）
    data/eso_<subject>_<W>.meta.json   冻结参数 + 校验结果
    metrics/summary.json               12 条 run 的跟踪/能量/完整性指标
    figures/*.png                      中文图件（plots.py）

能量真值通道由风真值解析导出（无动力/气动力遥测时的环境获能真值代理）：
    e_a   = ½·TAS² + g·h                    空气相对比机械能（docs §25）
    p_env = g·w_up − v_air·Ẇ                环境比功率（上升项 + 风随时间变化项）
满足 ė_a = (P_thr − P_drag)/m + p_env；W0 时 p_env ≡ 0（零获能基线检查 E0）。
"""

from __future__ import annotations

import argparse
import json
import sys
import time as wallclock
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from python.controller import PolylinePathFollower  # noqa: E402
from python.simulation.attitude_fleet import AttitudeFleet  # noqa: E402

from wind_fields import G, WIND_IDS, make_wind, wind_flat_to_neu  # noqa: E402
from routes import BASE_ALT_M, FOLLOWER_KWARGS, IAS_CMD_MPS, ROUTES, SUBJECTS  # noqa: E402

OUT = Path(__file__).resolve().parent
DT = 0.02
SCHEMA_VERSION = "avanti-eso-wp2a-endpoint-v0.2"

VECTOR_CHANNELS = {
    "truth/wind_flat", "truth/wind_applied_flat", "truth/wind_neu",
    "state/pos_neu", "state/vel_neu", "state/euler_rad",
    "state/rates_body_radps", "action/path_target_neu",
}
SCALAR_CHANNELS = (
    "time/t", "time/t_rel", "time/path_progress_m", "time/lap_count",
    "state/tas_mps", "state/ias_est_mps", "state/alpha_rad", "state/beta_rad",
    "state/chi_rad", "state/gamma_rad",
    "action/phi_cmd_rad", "action/theta_cmd_rad", "action/throttle_cmd",
    "action/chi_cmd_rad", "action/gamma_cmd_rad", "action/altitude_cmd_m",
    "action/ias_cmd_mps", "action/tas_cmd_mps",
    "action/path_cross_track_m", "action/path_lookahead_m",
    "energy/e_a_jpkg", "energy/edot_a_wpkg", "energy/p_env_up_wpkg",
    "energy/p_env_wind_rate_wpkg", "energy/p_env_total_wpkg",
    "energy/E_env_cum_jpkg",
)
EXPECTED_CHANNELS = tuple(sorted(set(SCALAR_CHANNELS) | VECTOR_CHANNELS))


def simulate_subject(subject: str, seconds: float, workers: int = 1):
    """一个科目、batch=3（W0/W1/W2）的闭环采集。返回逐实例原始通道。"""
    route = ROUTES[subject]
    path = route["build"]()
    winds = [make_wind(w) for w in WIND_IDS]
    batch = len(winds)
    ticks = round(seconds / DT)
    rows = []
    started = wallclock.perf_counter()
    with AttitudeFleet(batch=batch, workers=workers) as fleet:
        fc = fleet.controller
        followers = []
        for _ in range(batch):
            follower = PolylinePathFollower(
                path, IAS_CMD_MPS, closed=route["closed"], **FOLLOWER_KWARGS)
            follower.reset(initial_chi=0.0)
            followers.append(follower)
        for _ in range(ticks):
            now = fleet.time
            obs = fleet.observation()
            wind = np.zeros((batch, 3))
            for i, (follower, field) in enumerate(zip(followers, winds)):
                command = follower.update(
                    obs.neu_pos[i], now, float(np.linalg.norm(obs.neu_vel[i, :2])))
                fc.command_heading(command, indices=[i])
                wind[i] = field.query(obs.neu_pos[i][None, :], now)[0]
            record = fleet.step(wind_flat=wind)
            status = [f.status() for f in followers]
            record.update(
                wind_applied_flat=wind,
                path_cross_track=np.array([s.signed_cross_track_m for s in status]),
                path_progress=np.array([s.progress_m for s in status]),
                path_lookahead=np.array([s.lookahead_m for s in status]),
                path_target=np.array([s.target_neu for s in status]),
            )
            rows.append(record)
        gains = fleet.gains
    elapsed = wallclock.perf_counter() - started
    stacked = {k: np.stack([r[k] for r in rows]) for k in rows[0]}  # [T,B] / [T,B,3]
    return stacked, path, winds, gains, elapsed


def build_run_channels(stacked: dict, instance: int, field, lap_length_m: float) -> dict:
    """单实例 → avanti_eso 命名通道（[T] / [T,3]），含能量真值。"""
    i = instance
    t = stacked["t"][:, i]
    pos = np.stack([stacked["north"][:, i], stacked["east"][:, i],
                    stacked["alt"][:, i]], axis=1)
    vel = np.stack([stacked["vn"][:, i], stacked["ve"][:, i],
                    stacked["vu"][:, i]], axis=1)
    wind_applied = stacked["wind_applied_flat"][:, i]        # 本 tick 实际送入 plant
    wind_truth = field.query(pos, t)                          # 采样位置处的场真值
    wind_neu = wind_flat_to_neu(wind_applied)
    tas = stacked["tas"][:, i]
    v_air = vel - wind_neu
    e_a = 0.5 * tas**2 + G * pos[:, 2]
    edot_a = np.gradient(e_a, DT)
    wind_rate = np.gradient(wind_neu, DT, axis=0)
    p_up = G * wind_neu[:, 2]
    p_wind_rate = -np.sum(v_air * wind_rate, axis=1)
    p_total = p_up + p_wind_rate
    progress = stacked["path_progress"][:, i]
    channels = {
        "time/t": t,
        "time/t_rel": t - t[0],
        "time/path_progress_m": progress,
        "time/lap_count": np.floor(progress / lap_length_m),
        "truth/wind_flat": wind_truth,
        "truth/wind_applied_flat": wind_applied,
        "truth/wind_neu": wind_neu,
        "state/pos_neu": pos,
        "state/vel_neu": vel,
        "state/euler_rad": np.stack([stacked["phi"][:, i], stacked["theta"][:, i],
                                     stacked["psi"][:, i]], axis=1),
        "state/rates_body_radps": np.stack([stacked["p"][:, i], stacked["q"][:, i],
                                            stacked["r"][:, i]], axis=1),
        "state/tas_mps": tas,
        "state/ias_est_mps": stacked["ias_est"][:, i],
        "state/alpha_rad": stacked["alpha"][:, i],
        "state/beta_rad": stacked["beta"][:, i],
        "state/chi_rad": stacked["chi"][:, i],
        "state/gamma_rad": stacked["gamma"][:, i],
        "action/phi_cmd_rad": stacked["phi_cmd"][:, i],
        "action/theta_cmd_rad": stacked["theta_cmd"][:, i],
        "action/throttle_cmd": stacked["throttle_cmd"][:, i],
        "action/chi_cmd_rad": stacked["chi_cmd"][:, i],
        "action/gamma_cmd_rad": stacked["gamma_cmd"][:, i],
        "action/altitude_cmd_m": stacked["altitude_cmd"][:, i],
        "action/ias_cmd_mps": stacked["ias_cmd"][:, i],
        "action/tas_cmd_mps": stacked["tas_cmd"][:, i],
        "action/path_cross_track_m": stacked["path_cross_track"][:, i],
        "action/path_lookahead_m": stacked["path_lookahead"][:, i],
        "action/path_target_neu": stacked["path_target"][:, i],
        "energy/e_a_jpkg": e_a,
        "energy/edot_a_wpkg": edot_a,
        "energy/p_env_up_wpkg": p_up,
        "energy/p_env_wind_rate_wpkg": p_wind_rate,
        "energy/p_env_total_wpkg": p_total,
        "energy/E_env_cum_jpkg": np.cumsum(p_total) * DT,
    }
    return channels


def validate_run(subject: str, wind_id: str, ch: dict, field) -> dict:
    """完整性 + 跟踪品质 + 能量物理一致性检查。"""
    t = ch["time/t"]
    result = {
        "subject": subject, "wind": wind_id,
        "ticks": int(len(t)), "seconds": float(len(t) * DT),
        "channels_complete": sorted(ch) == sorted(EXPECTED_CHANNELS),
        "all_finite": bool(all(np.isfinite(v).all() for v in ch.values())),
        "time_uniform_0p02s": bool(np.max(np.abs(np.diff(t) - DT)) < 1e-9),
        "relative_time_zero_start": bool(ch["time/t_rel"][0] == 0.0),
        "progress_monotonic": bool(np.all(np.diff(ch["time/path_progress_m"]) >= -1e-9)),
    }
    valid = t - t[0] > 30.0
    if not np.any(valid):  # 短冒烟运行：全程统计
        valid = np.ones_like(t, dtype=bool)
    steady = t - t[0] > 60.0
    if not np.any(steady):
        steady = valid
    cross = np.abs(ch["action/path_cross_track_m"][steady])
    result.update(
        ias_rmse_mps=float(np.sqrt(np.mean(
            (ch["state/ias_est_mps"][valid] - ch["action/ias_cmd_mps"][valid]) ** 2))),
        alt_rmse_m=float(np.sqrt(np.mean(
            (ch["state/pos_neu"][valid, 2] - ch["action/altitude_cmd_m"][valid]) ** 2))),
        path_cross_mae_m=float(np.mean(cross)),
        path_cross_p95_m=float(np.percentile(cross, 95)),
        completed_laps=int(ch["time/lap_count"][-1]),
        max_bank_deg=float(np.max(np.abs(ch["state/euler_rad"][:, 0])) * 180 / np.pi),
        max_gamma_cmd_deg=float(np.max(np.abs(ch["action/gamma_cmd_rad"])) * 180 / np.pi),
        max_beta_deg=float(np.max(np.abs(ch["state/beta_rad"][valid])) * 180 / np.pi),
        altitude_min_m=float(np.min(ch["state/pos_neu"][:, 2])),
        altitude_max_m=float(np.max(ch["state/pos_neu"][:, 2])),
        throttle_limit_fraction=float(np.mean(
            (ch["action/throttle_cmd"][valid] <= 0.001)
            | (ch["action/throttle_cmd"][valid] >= 0.999))),
        energy_env_total_jpkg=float(ch["energy/E_env_cum_jpkg"][-1]),
        edot_a_mean_wpkg=float(np.mean(ch["energy/edot_a_wpkg"][valid])),
    )
    p_total = ch["energy/p_env_total_wpkg"]
    if wind_id == "W0":
        result["zero_wind_zero_energy_ok"] = bool(
            np.max(np.abs(ch["truth/wind_applied_flat"])) == 0.0
            and np.max(np.abs(p_total)) == 0.0)
    if wind_id == "W1":
        core = np.linalg.norm(
            ch["state/pos_neu"][:, :2] - np.asarray(field.center), axis=1) < field.radius
        in_core = p_total[core & valid]
        throttle = ch["action/throttle_cmd"][valid]
        w_up = ch["truth/wind_neu"][valid, 2]
        corr = float(np.corrcoef(throttle, w_up)[0, 1]) if np.std(w_up) > 0 else None
        result.update(
            thermal_core_tick_fraction=float(np.mean(core)),
            p_env_core_mean_wpkg=float(np.mean(in_core)) if len(in_core) else None,
            p_env_core_positive=bool(len(in_core) and np.mean(in_core) > 0),
            throttle_updraft_corr=corr,
            throttle_relief_consistent=bool(corr is not None and corr < 0),
        )
    if wind_id == "W2":
        dh = ch["state/pos_neu"][:, 2] - field.h0
        p_shear = ch["energy/p_env_wind_rate_wpkg"][valid]
        result.update(
            shear_layer_crossings=int(np.sum(np.diff(np.sign(dh)) != 0)),
            shear_layer_time_fraction=float(np.mean(np.abs(dh) < 2.0 * field.width)),
            p_shear_abs_mean_wpkg=float(np.mean(np.abs(p_shear))),
            p_shear_mean_wpkg=float(np.mean(p_shear)),
        )
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=480.0)
    parser.add_argument("--subjects", nargs="*", default=list(SUBJECTS))
    parser.add_argument("--skip-plots", action="store_true")
    args = parser.parse_args()

    data_dir = OUT / "data"
    metrics_dir = OUT / "metrics"
    data_dir.mkdir(exist_ok=True)
    metrics_dir.mkdir(exist_ok=True)

    summary = {"schema_version": SCHEMA_VERSION, "dt_s": DT,
               "seconds_per_run": args.seconds, "ias_cmd_mps": IAS_CMD_MPS,
               "base_altitude_m": BASE_ALT_M, "runs": []}
    all_runs = {}
    reference_paths = {}
    gains = None
    for subject in args.subjects:
        route = ROUTES[subject]
        print(f"RUNNING {subject} x {'/'.join(WIND_IDS)} {args.seconds}s", flush=True)
        stacked, path, winds, gains, elapsed = simulate_subject(subject, args.seconds)
        reference_paths[subject] = path
        for i, field in enumerate(winds):
            wind_id = field.wind_id
            channels = build_run_channels(stacked, i, field, route["lap_length_m"])
            report = validate_run(subject, wind_id, channels, field)
            report["wall_seconds_batch3"] = elapsed
            run_name = f"eso_{subject}_{wind_id}"
            np.savez_compressed(data_dir / f"{run_name}.npz", **channels)
            meta = dict(
                schema_version=SCHEMA_VERSION, run=run_name,
                subject=subject, scenario=route["scenario"], label=route["label"],
                route_params=route["params"], closed=route["closed"],
                lap_length_m=route["lap_length_m"],
                wind=dict(wind_id=wind_id, params=field.params),
                dt_s=DT, ticks=report["ticks"],
                frames=dict(pos_vel="NEU (North/East/Up-positive), m, m/s",
                            wind_flat="flat-Earth Xe/Ye/Ze (down-positive), m/s",
                            wind_neu="NEU (Up-positive), m/s",
                            angles="rad; euler relative to NEU; body rates FRD"),
                path_following="PolylinePathFollower(altitude_follow=True): heading from "
                               "path plan, flight-path angle from path segment slope "
                               "(clipped to +/-15 deg)",
                known_wind_assumption="controller uses local wind truth (no estimator)",
                wind_transport_delay_ticks=1,
                validation=report,
            )
            (data_dir / f"{run_name}.meta.json").write_text(
                json.dumps(meta, indent=2, ensure_ascii=False), encoding="utf-8")
            summary["runs"].append(report)
            all_runs[(subject, wind_id)] = channels
            print(json.dumps(report), flush=True)

    summary["native_gain_values"] = gains
    (metrics_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

    (data_dir / "sample_preview.txt").write_text(sample_preview(all_runs), encoding="utf-8")

    if not args.skip_plots:
        import plots
        plots.render_all(all_runs, reference_paths, OUT / "figures")
    print("DONE", flush=True)


def sample_preview(all_runs: dict) -> str:
    """抽一条 run 的张量切片做人读样本（figure8 × W2，切变层穿越段）。"""
    key = ("figure8", "W2") if ("figure8", "W2") in all_runs else next(iter(all_runs))
    ch = all_runs[key]
    t = ch["time/t"]
    dh = np.abs(ch["state/pos_neu"][:, 2] - 100.0)
    start = int(np.argmin(dh)) if len(dh) else 0
    rows = [f"sample run: eso_{key[0]}_{key[1]}  (每 25 tick = 0.5 s 抽样，切变层 100 m 附近)",
            f"{'t_s':>8} {'north_m':>9} {'east_m':>9} {'alt_m':>8} {'tas_mps':>8} "
            f"{'w_east_mps':>10} {'gamma_cmd_deg':>13} {'phi_cmd_deg':>11} "
            f"{'throttle':>9} {'p_env_wpkg':>11} {'E_cum_jpkg':>11}"]
    for k in range(max(start - 125, 0), min(start + 126, len(t)), 25):
        rows.append(
            f"{t[k]:8.2f} {ch['state/pos_neu'][k,0]:9.2f} {ch['state/pos_neu'][k,1]:9.2f} "
            f"{ch['state/pos_neu'][k,2]:8.2f} {ch['state/tas_mps'][k]:8.3f} "
            f"{ch['truth/wind_flat'][k,1]:10.3f} "
            f"{np.rad2deg(ch['action/gamma_cmd_rad'][k]):13.2f} "
            f"{np.rad2deg(ch['action/phi_cmd_rad'][k]):11.2f} "
            f"{ch['action/throttle_cmd'][k]:9.3f} "
            f"{ch['energy/p_env_total_wpkg'][k]:11.3f} "
            f"{ch['energy/E_env_cum_jpkg'][k]:11.2f}")
    return "\n".join(rows) + "\n"


if __name__ == "__main__":
    main()
