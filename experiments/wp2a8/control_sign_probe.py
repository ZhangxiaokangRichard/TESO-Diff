#!/usr/bin/env python3
"""WP2A-8 前置：舵面符号辨识（控制输入通道阶跃响应）。

确定 delta_e/delta_a/delta_r 正指令对 pitch/roll/yaw 的响应方向，
为内环 PID 符号设计与 GA 搜索范围提供依据（对应 docs §6.6 WP2A-3）。

2026-08-30 复测结论（throttle=0.4 基线，见 build_logs/phase2_sp2.md）：
    delta_e 正 → 低头（theta 减小）   → 俯仰通道取负（修正：早期辨识反了）
    delta_a 正 → 左滚（phi 减小）     → 滚转通道取负
    delta_r 正 → 右偏航（psi 增加）   → 偏航通道取正
注意：早期记录（tuning_01 时期）把 delta_e 记为"正 → 抬头"，与复测相反，
已导致单环 PID 时代俯仰通道无法稳定；串级控制器以本复测结论为准。
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "python"))

from bindings.avanti import AvantiInput, AvantiLibrary  # noqa: E402


def run_channel(name: str, value: float, seconds: float = 10.0) -> dict:
    """固定 throttle=0.5，对单个舵面施加阶跃，记录姿态与角速度轨迹。"""
    library = AvantiLibrary()
    inputs = AvantiInput(throttle_01=0.5)
    setattr(inputs, name, value)
    ticks = int(seconds / 0.02)
    t, phi, theta, psi, p, q, r = [], [], [], [], [], [], []
    with library.model(epoch_s=0.0) as aircraft:
        for k in range(ticks):
            out = aircraft.tick(inputs)
            t.append(k * 0.02)
            phi.append(out.euler_rpy_deg[0])
            theta.append(out.euler_rpy_deg[1])
            psi.append(out.euler_rpy_deg[2])
            p.append(out.rates_body_radps[0])
            q.append(out.rates_body_radps[1])
            r.append(out.rates_body_radps[2])
    return {
        "t": np.array(t),
        "phi": np.array(phi),
        "theta": np.array(theta),
        "psi": np.array(psi),
        "p": np.array(p),
        "q": np.array(q),
        "r": np.array(r),
    }


def summarize(name: str, data: dict) -> None:
    print(f"\n== {name} ==  (t=2s -> t=end 区间变化)")
    span = (data["t"] >= 2.0) & (data["t"] <= 9.0)
    for key in ("phi", "theta", "psi", "p", "q", "r"):
        values = data[key][span]
        print(
            f"  {key:5s}: start={values[0]:+9.3f}  end={values[-1]:+9.3f}  "
            f"delta={values[-1] - values[0]:+9.3f}"
        )


def main() -> int:
    cases = [
        ("delta_e_norm", +0.05, "升降舵 +0.05"),
        ("delta_e_norm", -0.05, "升降舵 -0.05"),
        ("delta_a_norm", +0.05, "副翼 +0.05"),
        ("delta_a_norm", -0.05, "副翼 -0.05"),
        ("delta_r_norm", +0.05, "方向舵 +0.05"),
        ("delta_r_norm", -0.05, "方向舵 -0.05"),
    ]
    for name, value, label in cases:
        data = run_channel(name, value)
        summarize(label, data)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
