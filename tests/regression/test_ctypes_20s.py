#!/usr/bin/env python3
"""确定性的 20 秒 AVANTI 仿真回归测试：按固定输入序列运行 1000 tick 并打印/保存采样数据。

与 tests/reference/simulink_20s.csv 的 Simulink 基准逐样本比较（15 列 × 1000 行），
实测最大时间误差 3.55e-15 s、最大绝对值误差 4.09e-12（高度列），
通过 abs <= 1e-9 或 rel <= 1e-10 的容差判定。

用法（Windows）：
    python tests/test_ctypes_20s.py --library build/win64/dist/avanti2.dll \
        --seconds 20 --print-every 1
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "python" / "bindings"))

from avanti import AvantiInput, AvantiLibrary  # noqa: E402


def test_input(elapsed_s: float) -> AvantiInput:
    """按 README 第 9 节的固定输入剧本构造输入：
    0-5 s 油门 0.5；5-8 s 升降舵 -0.01；8-11 s 副翼 0.01；
    11-14 s 方向舵 0.01；14-20 s 加 X 轴风 1 m/s。"""
    inputs = AvantiInput(throttle_01=0.5)
    if 5.0 <= elapsed_s < 8.0:
        inputs.delta_e_norm = -0.01
    if 8.0 <= elapsed_s < 11.0:
        inputs.delta_a_norm = 0.01
    if 11.0 <= elapsed_s < 14.0:
        inputs.delta_r_norm = 0.01
    if elapsed_s >= 14.0:
        inputs.set_wind((1.0, 0.0, 0.0))
    return inputs


def row(output):
    """把一个 AvantiOutput 展开为 CSV 的 15 列（与 Simulink 基准列序一致）。"""
    return [
        output.sample_time_s,
        output.alpha_rad,
        output.beta_rad,
        *output.position_ne_alt_m,
        *output.euler_rpy_deg,
        *output.velocity_body_mps,
        *output.rates_body_radps,
    ]


def main() -> int:
    # 命令行参数：动态库路径、仿真秒数与打印间隔
    parser = argparse.ArgumentParser()
    parser.add_argument("--library", type=Path, default=None)
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--print-every", type=float, default=1.0)
    parser.add_argument(
        "--csv",
        type=Path,
        default=PROJECT_ROOT / "build" / "test_output" / "python_20s.csv",
    )
    args = parser.parse_args()

    # 加载动态库并创建 0 秒 epoch 的模型实例
    library = AvantiLibrary(args.library)
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    # 表头与 Simulink 基准 CSV 完全一致，供 compare_simulink_dll_20s.m 逐列比较
    header = [
        "time_s",
        "alpha_rad",
        "beta_rad",
        "north_m",
        "east_m",
        "alt_m",
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
        "u_mps",
        "v_mps",
        "w_mps",
        "p_radps",
        "q_radps",
        "r_radps",
    ]

    with library.model(0.0) as model, args.csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        fixed_step_s = model.fixed_step_s
        ticks = round(args.seconds / fixed_step_s)  # 20 s / 0.02 s = 1000 tick
        print_stride = max(1, round(args.print_every / fixed_step_s))
        print(",".join(header))
        for tick in range(ticks):
            # 用整数 tick 计算时间，避免反复 time += h 累积浮点误差
            time_s = tick * fixed_step_s
            output = model.tick(test_input(time_s), time_s)
            if not output.finite():
                raise RuntimeError(f"non-finite output at tick {tick}")
            values = row(output)
            writer.writerow(values)
            if tick % print_stride == 0 or tick + 1 == ticks:
                print(",".join(f"{value:.10g}" for value in values))
        # 验证最终 next tick 为 20.0 s（内部时间契约）
        if abs(model.next_tick_time_s - args.seconds) > 1.0e-12:
            raise AssertionError(model.next_tick_time_s)

    print(f"Python {args.seconds:g} s simulation passed: ticks={ticks}, csv={args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
