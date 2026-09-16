#!/usr/bin/env python3
"""WP2A-8 内环角速率通道饱和问题诊断（只读分析，不修改控制器）。

目的：用真实 plant（avanti2.dll）单独验证 T15 8 字试验中观测到的
「姿态角误差 → 期望角速率饱和 → 舵面抖振 → 角速率跟不上」现象。

实验分两部分：
  Part 1  开环舵效辨识：配平后冻结俯仰/油门，施加 aileron 方波与阶跃，
          直接测量滚转轴的控制效能 a、阻尼 b 与满舵可达稳态滚转角速率。
  Part 2  闭环复现：用真实 InnerLoop + 真实 plant 施加 phi_cmd 阶跃，
          复现饱和/抖振，并对比若干候选修正（前馈增益、速率环增益、限幅）。

运行：python experiments/wp2a8/inner_rate_diagnosis.py
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from quality_trials import load_tuned  # noqa: E402
from python.bindings import AvantiInput, AvantiLibrary  # noqa: E402
from python.controller.inner import InnerLoop, wrap_pi  # noqa: E402
from python.controller.types import ControlObservation  # noqa: E402

DEG = np.pi / 180.0
DT = 0.02
V_TRIM = 16.0
THROTTLE_TRIM = 0.524
THETA_TRIM = 7.098 * DEG
OUT_DIR = Path(__file__).resolve().parent / "quality_trials"


def _trim_run(model, inner: InnerLoop, batch: int, seconds: float,
              phi_cmd: float, theta_cmd: float, throttle: float,
              obs_holder: list) -> None:
    """用给定内环把飞机配平一段时间（同时把最新观测量存入 obs_holder）。"""
    n = int(round(seconds / DT))
    for _ in range(n):
        inp = AvantiInput()
        inp.delta_a_norm = float(obs_holder[0])
        inp.delta_e_norm = float(obs_holder[1])
        inp.delta_r_norm = float(obs_holder[2])
        inp.throttle_01 = throttle
        out = model.tick(inp)
        obs = ControlObservation.from_fw_outputs([out])
        cmd = inner.step(np.array([phi_cmd]), np.array([theta_cmd]), None, obs)
        obs_holder[:] = (float(cmd.delta_a[0]), float(cmd.delta_e[0]),
                         float(cmd.delta_r[0]))


def part1_open_loop(cfg: dict) -> dict:
    """配平后冻结俯仰/油门，单独激励 aileron，辨识滚转轴。"""
    print("\n" + "=" * 78)
    print("Part 1  开环滚转舵效辨识（真实 plant，冻结 delta_e/throttle）")
    print("=" * 78)

    inner = InnerLoop(cfg["inner"])
    inner.reset(1)
    library = AvantiLibrary()
    with library.model(epoch_s=0.0) as model:
        holder = [0.0, 0.0, 0.0]
        _trim_run(model, inner, 1, 30.0, 0.0, THETA_TRIM, THROTTLE_TRIM, holder)
        de_trim, dr_trim = holder[1], holder[2]

        # ---- 1a. 方波激励（小幅度，避免大坡度）----
        results: dict[float, tuple[float, float]] = {}
        for amp in (0.10, 0.25, 0.50):
            inner.reset(1)
            with library.model(epoch_s=0.0) as m2:
                holder2 = [0.0, 0.0, 0.0]
                _trim_run(m2, inner, 1, 25.0, 0.0, THETA_TRIM, THROTTLE_TRIM, holder2)
                de2 = holder2[1]
                t_log: list[float] = []
                p_log: list[float] = []
                a_log: list[float] = []
                n = int(8.0 / DT)
                for k in range(n):
                    t = k * DT
                    # 周期 4 s 的方波
                    u = amp if np.sin(2 * np.pi * t / 4.0) >= 0 else -amp
                    inp = AvantiInput()
                    inp.delta_a_norm = u
                    inp.delta_e_norm = de2
                    inp.delta_r_norm = 0.0
                    inp.throttle_01 = THROTTLE_TRIM
                    out = m2.tick(inp)
                    obs = ControlObservation.from_fw_outputs([out])
                    t_log.append(t)
                    p_log.append(float(obs.p[0]))
                    a_log.append(u)
                t_arr = np.array(t_log)
                p_arr = np.array(p_log)
                a_arr = np.array(a_log)
                # 取每半周期尾段（准稳态）的平均 p
                pos = (a_arr > 0) & (t_arr > 2.0)
                neg = (a_arr < 0) & (t_arr > 2.0)
                p_pos = float(np.mean(p_arr[pos]))
                p_neg = float(np.mean(p_arr[neg]))
                gain = (p_pos - p_neg) / (2 * amp)
                results[amp] = (p_pos, p_neg, gain)
                print(f"  方波幅值 ±{amp:.2f}: p(+u)={p_pos/DEG:+7.2f} deg/s  "
                      f"p(-u)={p_neg/DEG:+7.2f} deg/s  "
                      f"等效斜率 d(p)/du={gain/DEG:+7.2f} (deg/s)/unit")

        # ---- 1b. 满舵阶跃（正负各 3 s）----
        inner.reset(1)
        with library.model(epoch_s=0.0) as m3:
            holder3 = [0.0, 0.0, 0.0]
            _trim_run(m3, inner, 1, 25.0, 0.0, THETA_TRIM, THROTTLE_TRIM, holder3)
            de3 = holder3[1]
            steps = [(0.0, 0.0), (-1.0, 3.0), (0.0, 2.0), (1.0, 3.0), (0.0, 2.0),
                     (-0.5, 2.0), (0.0, 1.0), (0.5, 2.0)]
            step_stat: list[dict] = []
            for u, dur in steps:
                n = int(dur / DT)
                tail: list[float] = []
                p_peak = 0.0
                for k in range(n):
                    inp = AvantiInput()
                    inp.delta_a_norm = u
                    inp.delta_e_norm = de3
                    inp.delta_r_norm = 0.0
                    inp.throttle_01 = THROTTLE_TRIM
                    out = m3.tick(inp)
                    obs = ControlObservation.from_fw_outputs([out])
                    p = float(obs.p[0])
                    if k > 0.6 * n:
                        tail.append(p)
                    if abs(p) > abs(p_peak):
                        p_peak = p
                if abs(u) > 1e-9:
                    step_stat.append({
                        "delta_a": u,
                        "p_tail_mean_degps": float(np.mean(tail) / DEG),
                        "p_peak_degps": float(p_peak / DEG),
                    })
                    print(f"  阶跃 delta_a={u:+.2f}: p 尾段均值={np.mean(tail)/DEG:+7.2f} deg/s  "
                          f"峰值={p_peak/DEG:+7.2f} deg/s")
    return {"square_wave": {str(k): [float(x) for x in v] for k, v in results.items()},
            "steps": step_stat}


def part2_closed_loop(cfg: dict, variant: dict, label: str,
                      seconds: float = 25.0, step_at: float = 8.0,
                      phi_step_deg: float = -12.0) -> dict:
    """真实 plant + 真实 InnerLoop：phi_cmd 阶跃下的滚转通道闭环行为。"""
    inner_cfg = json.loads(json.dumps(cfg["inner"]))
    inner_cfg.update(variant.get("inner_top", {}))
    for chan, kv in variant.get("channels", {}).items():
        inner_cfg.setdefault(chan, {})
        inner_cfg[chan].update(kv)

    inner = InnerLoop(inner_cfg)
    inner.reset(1)
    library = AvantiLibrary()
    with library.model(epoch_s=0.0) as model:
        holder = [0.0, 0.0, 0.0]
        _trim_run(model, inner, 1, 6.0, 0.0, THETA_TRIM, THROTTLE_TRIM, holder)

        log = {k: [] for k in ("t", "phi", "phi_cmd", "p", "p_sp", "delta_a",
                               "err_phi", "err_rate")}
        n = int(seconds / DT)
        for k in range(n):
            t = k * DT
            phi_cmd = phi_step_deg * DEG if t >= step_at else 0.0
            inp = AvantiInput()
            inp.delta_a_norm = float(holder[0])
            inp.delta_e_norm = float(holder[1])
            inp.delta_r_norm = float(holder[2])
            inp.throttle_01 = THROTTLE_TRIM
            out = model.tick(inp)
            obs = ControlObservation.from_fw_outputs([out])
            cmd = inner.step(np.array([phi_cmd]), np.array([THETA_TRIM]), None, obs)
            holder[:] = (float(cmd.delta_a[0]), float(cmd.delta_e[0]),
                         float(cmd.delta_r[0]))
            rates = inner.rates_sp()
            log["t"].append(t)
            log["phi"].append(float(obs.phi[0]))
            log["phi_cmd"].append(phi_cmd)
            log["p"].append(float(obs.p[0]))
            log["p_sp"].append(float(rates[0, 0]))
            log["delta_a"].append(float(holder[0]))
            log["err_phi"].append(float(wrap_pi(np.array([phi_cmd]))[0] - obs.phi[0]))
            log["err_rate"].append(float(rates[0, 0] - obs.p[0]))
    log = {k: np.array(v) for k, v in log.items()}
    m = log["t"] >= step_at

    p_lim = inner_cfg.get("p_limit_degps", 60.0)
    sat_rail = np.mean(np.abs(log["p_sp"][m]) >= 0.99 * p_lim * DEG) * 100.0
    sat_da = np.mean(np.abs(log["delta_a"][m]) >= 0.99) * 100.0
    lags = np.corrcoef(log["p_sp"][m][:-1], log["p"][m][1:])[0, 1]
    # 阶跃后 2~8 s 的稳态姿态误差
    tail = (log["t"] >= step_at + 3.0)
    res = {
        "label": label,
        "variant": variant,
        "phi_step_deg": phi_step_deg,
        "phi_err_mae_deg": float(np.mean(np.abs(log["err_phi"][m])) / DEG),
        "phi_err_ss_deg": float(np.mean(log["err_phi"][tail]) / DEG),
        "p_sp_rail_pct": float(sat_rail),
        "delta_a_sat_pct": float(sat_da),
        "rate_track_corr": float(lags),
        "rate_err_mae_degps": float(np.mean(np.abs(log["err_rate"][m])) / DEG),
        "p_mean_abs_degps": float(np.mean(np.abs(log["p"][m])) / DEG),
        "p_sp_mean_abs_degps": float(np.mean(np.abs(log["p_sp"][m])) / DEG),
    }
    print(f"\n  [{label}]")
    print(f"    姿态误差: 全程MAE={res['phi_err_mae_deg']:6.2f}deg  "
          f"稳态={res['phi_err_ss_deg']:+6.2f}deg   (指令 {phi_step_deg:+.1f}deg)")
    print(f"    p_sp 打限幅占比={sat_rail:5.1f}%    |delta_a|>=0.99 占比={sat_da:5.1f}%")
    print(f"    角速率跟踪: corr(p_sp,p)={lags:+.3f}  MAE={res['rate_err_mae_degps']:6.2f}deg/s  "
          f"|p|均值={res['p_mean_abs_degps']:5.2f} vs |p_sp|均值={res['p_sp_mean_abs_degps']:5.2f} deg/s")
    return res


def part1_identify_roll_model(cfg: dict) -> dict:
    """用小幅方波数据做一阶拟合 p_dot = a*u + b*p（真实 plant 开环）。"""
    print("\n" + "-" * 78)
    print("Part 1c  一阶模型拟合（开环 + 小方波激励，真实 plant）")
    print("-" * 78)
    inner = InnerLoop(cfg["inner"])
    inner.reset(1)
    library = AvantiLibrary()
    A_rows: list[list[float]] = []
    Y_rows: list[float] = []
    with library.model(epoch_s=0.0) as m:
        holder = [0.0, 0.0, 0.0]
        _trim_run(m, inner, 1, 25.0, 0.0, THETA_TRIM, THROTTLE_TRIM, holder)
        de = holder[1]
        n = int(12.0 / DT)
        prev_p = None
        for k in range(n):
            t = k * DT
            u = 0.15 * np.sign(np.sin(2 * np.pi * t / 6.0))
            inp = AvantiInput()
            inp.delta_a_norm = float(u)
            inp.delta_e_norm = de
            inp.delta_r_norm = 0.0
            inp.throttle_01 = THROTTLE_TRIM
            out = m.tick(inp)
            obs = ControlObservation.from_fw_outputs([out])
            p = float(obs.p[0])
            if prev_p is not None and k > int(6.0 / DT):
                A_rows.append([u, p])
                Y_rows.append((p - prev_p) / DT)
            prev_p = p
    A = np.array(A_rows)
    Y = np.array(Y_rows)
    c, _, _, _ = np.linalg.lstsq(A, Y, rcond=None)
    pred = A @ c
    r2 = 1 - np.var(Y - pred) / np.var(Y)
    a, b = float(c[0]), float(c[1])
    print(f"  p_dot = a·delta_a + b·p   R2={r2:.3f}")
    print(f"  a = {a:+.4f} rad/s^2 per unit   b = {b:+.4f} 1/s   tau = {-1/b:.3f} s")
    print(f"  满舵稳态滚转角速率 = {-a/b/DEG:+.2f} deg/s")
    print(f"  期望角加速度 = p_dot_sp 所需前馈 ff = 1/|a| = {1/abs(a):.4f}  "
          f"(当前配置 ff_roll = {cfg['inner']['roll']['rate_ff']})")
    return {"a": a, "b": b, "r2": float(r2), "tau_s": float(-1 / b),
            "ss_rate_full_aileron_degps": float(-a / b / DEG),
            "required_ff": float(1 / abs(a)),
            "configured_ff": float(cfg["inner"]["roll"]["rate_ff"])}


def part2b_turn_hold(cfg: dict, variant: dict, label: str,
                     seconds: float = 45.0, phi_hold_deg: float = -16.5,
                     phi_step_deg: float = -10.4) -> dict:
    """复现 T15 稳态盘旋工作点：先建立 -16.5deg 盘旋，再把 phi_cmd 抬到 -10.4deg。

    T15 观测到的工作点即为「实际坡度 -16.5deg、指令 -10.4deg、误差 +6.1deg、
    p_sp 打限幅、p≈0」。本实验检验滚转通道能否把这个 6deg 误差收回。
    """
    inner_cfg = json.loads(json.dumps(cfg["inner"]))
    inner_cfg.update(variant.get("inner_top", {}))
    for chan, kv in variant.get("channels", {}).items():
        inner_cfg.setdefault(chan, {})
        inner_cfg[chan].update(kv)
    inner = InnerLoop(inner_cfg)
    inner.reset(1)

    library = AvantiLibrary()
    settle_s, step_at = 20.0, 25.0
    with library.model(epoch_s=0.0) as model:
        holder = [0.0, 0.0, 0.0]
        _trim_run(model, inner, 1, 6.0, 0.0, THETA_TRIM, THROTTLE_TRIM, holder)
        log = {k: [] for k in ("t", "phi", "phi_cmd", "p", "p_sp", "delta_a",
                               "r", "beta", "err_phi")}
        n = int(seconds / DT)
        for k in range(n):
            t = k * DT
            if t < step_at:
                phi_cmd = phi_hold_deg * DEG
            else:
                phi_cmd = phi_step_deg * DEG
            inp = AvantiInput()
            inp.delta_a_norm = float(holder[0])
            inp.delta_e_norm = float(holder[1])
            inp.delta_r_norm = float(holder[2])
            inp.throttle_01 = THROTTLE_TRIM
            out = model.tick(inp)
            obs = ControlObservation.from_fw_outputs([out])
            cmd = inner.step(np.array([phi_cmd]), np.array([THETA_TRIM]), None, obs)
            holder[:] = (float(cmd.delta_a[0]), float(cmd.delta_e[0]),
                         float(cmd.delta_r[0]))
            rates = inner.rates_sp()
            log["t"].append(t)
            log["phi"].append(float(obs.phi[0]))
            log["phi_cmd"].append(phi_cmd)
            log["p"].append(float(obs.p[0]))
            log["p_sp"].append(float(rates[0, 0]))
            log["delta_a"].append(float(holder[0]))
            log["r"].append(float(obs.r[0]))
            log["beta"].append(float(obs.beta[0]))
            log["err_phi"].append(float(phi_cmd - obs.phi[0]))

    log = {k: np.array(v) for k, v in log.items()}
    m_hold = (log["t"] >= settle_s) & (log["t"] < step_at)
    m_step = log["t"] >= step_at + 5.0     # 阶跃后 5 s 起的稳态
    p_lim = inner_cfg.get("p_limit_degps", 60.0)
    res = {
        "label": label,
        "hold_phi_mean_deg": float(np.mean(log["phi"][m_hold]) / DEG),
        "hold_err_mean_deg": float(np.mean(log["err_phi"][m_hold]) / DEG),
        "hold_p_sp_rail_pct": float(np.mean(np.abs(log["p_sp"][m_hold]) >= 0.99 * p_lim * DEG) * 100),
        "hold_delta_a_sat_pct": float(np.mean(np.abs(log["delta_a"][m_hold]) >= 0.99) * 100),
        "hold_p_mean_degps": float(np.mean(np.abs(log["p"][m_hold])) / DEG),
        "hold_beta_mean_deg": float(np.mean(log["beta"][m_hold]) / DEG),
        "hold_r_mean_degps": float(np.mean(log["r"][m_hold]) / DEG),
        "step_phi_mean_deg": float(np.mean(log["phi"][m_step]) / DEG),
        "step_err_mean_deg": float(np.mean(log["err_phi"][m_step]) / DEG),
        "step_p_sp_rail_pct": float(np.mean(np.abs(log["p_sp"][m_step]) >= 0.99 * p_lim * DEG) * 100),
        "step_delta_a_sat_pct": float(np.mean(np.abs(log["delta_a"][m_step]) >= 0.99) * 100),
        "step_delta_a_mean": float(np.mean(log["delta_a"][m_step])),
        "step_p_mean_degps": float(np.mean(np.abs(log["p"][m_step])) / DEG),
    }
    print(f"\n  [{label}]")
    print(f"    建立 {phi_hold_deg:+.1f}deg 盘旋段: phi={res['hold_phi_mean_deg']:+7.2f}deg "
          f"误差={res['hold_err_mean_deg']:+6.2f}deg  p_sp打限幅={res['hold_p_sp_rail_pct']:5.1f}% "
          f"delta_a饱和={res['hold_delta_a_sat_pct']:5.1f}%  beta={res['hold_beta_mean_deg']:+6.2f}deg "
          f"r={res['hold_r_mean_degps']:+6.2f}deg/s")
    print(f"    指令改到 {phi_step_deg:+.1f}deg 后稳态: phi={res['step_phi_mean_deg']:+7.2f}deg "
          f"→ 剩余坡度误差={res['step_err_mean_deg']:+6.2f}deg")
    print(f"      p_sp打限幅={res['step_p_sp_rail_pct']:5.1f}%  delta_a饱和={res['step_delta_a_sat_pct']:5.1f}%  "
          f"delta_a均值={res['step_delta_a_mean']:+.3f}  |p|均值={res['step_p_mean_degps']:.2f}deg/s")
    return res


def part3_full_trial(cfg: dict, variant: dict, label: str,
                     seconds: float = 400.0) -> dict:
    """完整 T15 8 字闭环试验（真实外环 + 真实 plant），只改内环滚转通道。"""
    from python.controller import (  # noqa: E402
        FlightController,
        HeadingCommand,
        PolylinePathFollower,
    )
    from quality_trials import _flat_figure8_waypoints, run_api_trial  # noqa: E402

    inner_cfg = json.loads(json.dumps(cfg["inner"]))
    inner_cfg.update(variant.get("inner_top", {}))
    for chan, kv in variant.get("channels", {}).items():
        inner_cfg.setdefault(chan, {})
        inner_cfg[chan].update(kv)
    run_cfg = {"outer": cfg["outer"], "inner": inner_cfg}

    center, radius, altitude, airspeed = (200.0, 0.0), 80.0, 1000.0, 16.0
    path = _flat_figure8_waypoints(center, radius, altitude, n_per_circle=100)
    follower = PolylinePathFollower(
        path, airspeed=airspeed, gamma=0.0, closed=True,
        lookahead_time_s=2.5, lookahead_min_m=35.0, lookahead_max_m=55.0,
        cross_track_gain=1.0, heading_rate_limit_degps=18.0, update_period_s=0.1,
    )
    follower.reset(initial_segment=0, initial_chi=0.0)
    fc = FlightController(run_cfg)
    fc.reset(1)
    fc.command_heading(HeadingCommand(airspeed=airspeed, gamma=0.0, chi=0.0,
                                      timestamp=0.0))

    # 可选：覆盖偏航通道的 psi_cmd（检验航向角反馈对侧滑/滚转耦合的影响）
    if variant.get("psi_override_present"):
        _orig = fc._ctrl.inner.step

        def _patched(phi_cmd, theta_cmd, psi_cmd, obs):
            return _orig(phi_cmd, theta_cmd, None, obs)

        fc._ctrl.inner.step = _patched  # type: ignore[assignment]

    def update_heading(c, t, obs):
        gs = float(np.linalg.norm(obs.neu_vel[0, :2]))
        c.command_heading(follower.update(obs.neu_pos[0], t, gs))
        st = follower.status()
        return {"path_cross_track_m": st.signed_cross_track_m}

    log = run_api_trial(fc, step_hook=update_heading, seconds=seconds,
                        controller_rate_hz=50.0, airspeed_cmd=airspeed)
    t = np.asarray(log["t"])
    m = t >= 20.0
    err_phi = (np.asarray(log["phi_cmd"]) - np.asarray(log["phi"]))[m] / DEG
    p_sp = np.asarray(log["p_sp"])[m] / DEG
    p = np.asarray(log["p"])[m] / DEG
    da = np.asarray(log["delta_a"])[m]
    p_lim = inner_cfg.get("p_limit_degps", 60.0)
    _sat = np.abs(da) >= 0.99
    _idx = np.flatnonzero(np.diff(np.r_[0, _sat.astype(int), 0]))
    _bursts = (_idx[1::2] - _idx[0::2]) * 0.02
    res = {
        "label": label,
        "phi_err_mae_deg": float(np.mean(np.abs(err_phi))),
        "phi_err_abs_median_deg": float(np.median(np.abs(err_phi))),
        "p_sp_rail_pct": float(np.mean(np.abs(p_sp) >= 0.99 * p_lim) * 100),
        "delta_a_sat_pct": float(np.mean(np.abs(da) >= 0.99) * 100),
        "delta_a_burst_median_s": float(np.median(_bursts)) if len(_bursts) else 0.0,
        "delta_a_burst_n": int(len(_bursts)),
        "delta_a_mean_abs": float(np.mean(np.abs(da))),
        "rate_track_corr": float(np.corrcoef(p_sp, p)[0, 1]),
        "rate_err_mae_degps": float(np.mean(np.abs(p_sp - p))),
        "p_abs_mean_degps": float(np.mean(np.abs(p))),
        "p_sp_abs_mean_degps": float(np.mean(np.abs(p_sp))),
        "max_phi_deg": float(np.max(np.abs(np.asarray(log["phi"])[m])) / DEG),
        "cross_mae_m": float(np.mean(np.abs(np.asarray(log["path_cross_track_m"])[m]))),
        "beta_mean_deg": float(np.mean(np.abs(np.asarray(log["beta"])[m])) / DEG),
        "beta_max_deg": float(np.max(np.abs(np.asarray(log["beta"])[m])) / DEG),
        "r_abs_mean_degps": float(np.mean(np.abs(np.asarray(log["r"])[m])) / DEG),
    }
    print(f"\n  [{label}]")
    print(f"    坡度跟踪: MAE={res['phi_err_mae_deg']:5.2f}deg 中位={res['phi_err_abs_median_deg']:5.2f}deg "
          f"max|phi|={res['max_phi_deg']:5.1f}deg")
    print(f"    内环饱和: p_sp打限幅={res['p_sp_rail_pct']:5.1f}%  delta_a饱和={res['delta_a_sat_pct']:5.1f}% "
          f"(段数={res['delta_a_burst_n']}, 段长中位={res['delta_a_burst_median_s']*1000:.0f}ms)")
    print(f"    角速率跟踪: corr={res['rate_track_corr']:+.3f}  MAE={res['rate_err_mae_degps']:6.2f}deg/s  "
          f"|p|均={res['p_abs_mean_degps']:5.2f} vs |p_sp|均={res['p_sp_abs_mean_degps']:5.2f} deg/s")
    print(f"    任务指标: 横侧MAE={res['cross_mae_m']:.3f}m  |beta|均={res['beta_mean_deg']:.2f}deg "
          f"(max {res['beta_max_deg']:.1f})  |r|均={res['r_abs_mean_degps']:.2f}deg/s")
    return res


def main() -> int:
    cfg = load_tuned()
    print("配置来源: tuning_02(内环) + tuning_04(外环)")
    print(f"内环 roll: rate_kp={cfg['inner']['roll']['rate_kp']}, "
          f"rate_ff={cfg['inner']['roll']['rate_ff']}, "
          f"angle_roll_kp={cfg['inner']['angle_roll_kp']}, "
          f"p_limit={cfg['inner']['p_limit_degps']} deg/s")

    report: dict = {"open_loop": {}, "variants": []}
    report["open_loop"]["square_and_steps"] = part1_open_loop(cfg)
    report["open_loop"]["first_order_model"] = part1_identify_roll_model(cfg)

    print("\n" + "=" * 78)
    print("Part 2  闭环复现：phi_cmd 阶跃 → 内环滚转通道行为")
    print("=" * 78)
    base_ff = float(cfg["inner"]["roll"]["rate_ff"])
    variants = [
        ("基线（tuning_02 参数）", {}),
        ("去掉角加速度前馈 ff_roll=0", {"channels": {"roll": {"rate_ff": 0.0}}}),
        ("前馈改为 1/|a| 理论值", {"channels": {"roll": {"rate_ff": None}}}),
        ("前馈=0 + 速率环增益×4", {"channels": {"roll": {"rate_ff": 0.0, "rate_kp": 4.0}}}),
        ("限幅降到 30 deg/s", {"inner_top": {"p_limit_degps": 30.0}}),
    ]
    model = report["open_loop"]["first_order_model"]
    for label, var in variants:
        v = json.loads(json.dumps(var))
        if "channels" in v and "roll" in v["channels"] and \
                v["channels"]["roll"].get("rate_ff", "x") is None:
            v["channels"]["roll"]["rate_ff"] = round(model["required_ff"], 4)
        report["variants"].append(part2_closed_loop(cfg, v, label))

    print("\n" + "=" * 78)
    print("Part 2b  复现 T15 稳态盘旋工作点：能否收回 6deg 坡度误差")
    print("=" * 78)
    ff_theory = round(model["required_ff"], 4)
    variants_b = [
        ("基线（tuning_02 参数）", {}),
        ("ff_roll=0（关闭角加速度前馈）", {"channels": {"roll": {"rate_ff": 0.0}}}),
        ("p_limit=30deg/s（与可达角速率匹配）", {"inner_top": {"p_limit_degps": 30.0}}),
        ("ff_roll=0 + p_limit=30", {"channels": {"roll": {"rate_ff": 0.0}},
                                    "inner_top": {"p_limit_degps": 30.0}}),
        ("ff_roll=0 + 速率环增益×4", {"channels": {"roll": {"rate_ff": 0.0,
                                                            "rate_kp": 4.0}}}),
    ]
    report["turn_hold"] = [part2b_turn_hold(cfg, v, lb) for lb, v in variants_b]

    print("\n" + "=" * 78)
    print("Part 3  完整 T15 8 字闭环 400 s 对照（只改内环滚转通道）")
    print("=" * 78)
    variants_c = [
        ("基线（tuning_02 参数，复现 T15）", {}),
        ("偏航去掉航向角反馈 psi_cmd=None", {"psi_override_present": True}),
        ("ff_roll=0", {"channels": {"roll": {"rate_ff": 0.0}}}),
        ("p_limit=30deg/s", {"inner_top": {"p_limit_degps": 30.0}}),
    ]
    report["full_trial"] = [part3_full_trial(cfg, v, lb, seconds=200.0)
                            for lb, v in variants_c]

    out = OUT_DIR / "diagnosis_inner_rate.json"
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2),
                   encoding="utf-8")
    print(f"\n诊断报告: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
