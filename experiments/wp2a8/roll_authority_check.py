#!/usr/bin/env python3
"""滚转轴权限复核（修正版）：先用真实外环配平到 T15 工作点（V≈16 m/s、平飞）,
再冻结 delta_e/throttle 做 aileron 方波，测量真实的滚转控制效能。"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from quality_trials import load_tuned  # noqa: E402
from python.bindings import AvantiInput, AvantiLibrary  # noqa: E402
from python.controller import LegacySurfaceFlightController as FlightController, HeadingCommand  # noqa: E402
from python.controller.types import ControlObservation  # noqa: E402

DEG = np.pi / 180.0
DT = 0.02


def main() -> int:
    """俯仰/油门/偏航闭合（真实外环），只把副翼开路做方波激励。"""
    cfg = load_tuned()
    library = AvantiLibrary()
    with library.model(epoch_s=0.0) as model:
        fc = FlightController({"outer": cfg["outer"], "inner": cfg["inner"]})
        fc.reset(1)
        fc.command_heading(HeadingCommand(airspeed=16.0, gamma=0.0, chi=0.0,
                                          timestamp=0.0))
        cmd = None
        obs = None
        # ---- 阶段 1：配平 90 s（副翼仍由控制器给出）----
        for _ in range(int(90.0 / DT)):
            inp = AvantiInput()
            if cmd is not None:
                inp.delta_a_norm = float(cmd.delta_a[0])
                inp.delta_e_norm = float(cmd.delta_e[0])
                inp.delta_r_norm = float(cmd.delta_r[0])
                inp.throttle_01 = float(cmd.throttle[0])
            else:
                inp.throttle_01 = 0.5
            out = model.tick(inp)
            obs = ControlObservation.from_fw_outputs([out])
            cmd = fc.step(obs)
        print("配平状态: tas=%.3f m/s  alt=%.1f m  alpha=%.2f deg  theta=%.2f deg  "
              "delta_e=%+.3f  throttle=%.3f"
              % (float(obs.tas[0]), float(obs.neu_pos[0, 2]),
                 float(obs.alpha[0]) / DEG, float(obs.theta[0]) / DEG,
                 float(cmd.delta_e[0]), float(cmd.throttle[0])))

        print("\n副翼方波（周期 8 s）→ 稳态滚转角速率（俯仰/油门/偏航保持闭环）:")
        print(f"{'delta_a':>9} {'p(+u)':>10} {'p(-u)':>10} {'增益 (deg/s)/unit':>19} "
              f"{'tas':>8} {'alpha':>8}")
        gains = []
        for amp in (0.05, 0.10, 0.20, 0.35):
            pos, neg = [], []
            tas_log, al_log = [], []
            n = int(24.0 / DT)
            for k in range(n):
                t = k * DT
                u = amp if np.sin(2 * np.pi * t / 8.0) >= 0 else -amp
                inp = AvantiInput()
                inp.delta_a_norm = float(u)
                inp.delta_e_norm = float(cmd.delta_e[0])
                inp.delta_r_norm = float(cmd.delta_r[0])
                inp.throttle_01 = float(cmd.throttle[0])
                out = model.tick(inp)
                obs = ControlObservation.from_fw_outputs([out])
                cmd = fc.step(obs)
                if t > 16.0:
                    (pos if u > 0 else neg).append(float(obs.p[0]))
                    tas_log.append(float(obs.tas[0]))
                    al_log.append(float(obs.alpha[0]) / DEG)
            gain = (np.mean(pos) - np.mean(neg)) / (2 * amp)
            gains.append(abs(gain))
            print(f"{amp:9.3f} {np.mean(pos)/DEG:+9.2f} {np.mean(neg)/DEG:+9.2f} "
                  f"{gain/DEG:+18.2f} {np.mean(tas_log):7.2f} {np.mean(al_log):7.2f}")

        K = float(np.mean(gains)) / DEG          # rad/s per unit
        print(f"\n控制效能 K_plant = {K:.4f} rad/s per unit = {K/DEG:.2f} (deg/s) per unit")
        print(f"→ |delta_a| = 1 的稳态滚转角速率 ≈ {K/DEG:.1f} deg/s")
        print(f"→ p_limit 配置 = {cfg['inner']['p_limit_degps']:.0f} deg/s"
              f"（可达值的 {cfg['inner']['p_limit_degps']/max(K/DEG,1e-9):.1f} 倍）")
        k_rate = float(cfg["inner"]["roll"]["rate_kp"])
        L = k_rate * K
        print(f"\n速率环 P 增益 k_rate = {k_rate:.3f} → 环路直流增益 = {L:.3f}"
              f" → 线性区角速率跟踪比 = {L/(1+L):.3f}（滚转无积分环节）")
        for tgt in (1.0, 3.0, 5.0):
            print(f"    环路增益 {tgt:.1f} 需 k_rate ≈ {tgt/max(K,1e-9):.2f}")
        ff = float(cfg["inner"]["roll"]["rate_ff"])
        print(f"\nff_roll = {ff:.3f}；理论值 1/K_plant = {1/K:.3f}"
              f" → 当前为理论值的 {ff*K:.2f} 倍")
        print(f"\n角度环 angle_roll_kp = {cfg['inner']['angle_roll_kp']:.2f} rad/s per rad, "
              f"p_limit = {cfg['inner']['p_limit_degps']:.1f} deg/s")
        print(f"→ 角速率打限幅的坡度误差阈值 = {cfg['inner']['p_limit_degps']/cfg['inner']['angle_roll_kp']:.2f} deg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
