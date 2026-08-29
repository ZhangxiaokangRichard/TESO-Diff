#!/usr/bin/env python3
"""多实例并行正确性测试：对比串行与线程池并行运行相同实例的结果。

默认 16 个实例、各 20 秒：串行与 ThreadPoolExecutor 并行各执行 16000 次 tick，
要求最终输出向量最大绝对差为 0（容差 1e-12）。
注意：本测试验证的是"并行不互相干扰"的正确性，不是吞吐加速；
小模型单次调用很短时，Python 线程版实测反而更慢（speedup 约 0.66），
生产环境大量实例应改用 C++ 线程池或原生 batch API。

用法（Windows）：
    python tests/test_ctypes_parallel.py --library build/win64/dist/avanti2.dll \
        --instances 16 --seconds 20 --workers 16
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "python" / "bindings"))

from avanti import AvantiInput, AvantiLibrary  # noqa: E402


def input_for(instance: int, elapsed_s: float) -> AvantiInput:
    """为不同实例生成有差异的输入，保证各实例状态确实分离。"""
    inputs = AvantiInput(throttle_01=0.45 + 0.01 * (instance % 5))
    inputs.delta_a_norm = 0.002 * ((instance % 3) - 1)
    inputs.delta_e_norm = -0.001 * (instance % 4)
    if elapsed_s >= 10.0:
        inputs.wind_flat_earth_mps[1] = 0.1 * (instance % 4)
    return inputs


def final_vector(output) -> tuple[float, ...]:
    """取出参与对比的最终状态向量（迎角、侧滑角、位置、姿态、速度、角速度）。"""
    return (
        output.alpha_rad,
        output.beta_rad,
        *output.position_ne_alt_m,
        *output.euler_rpy_deg,
        *output.velocity_body_mps,
        *output.rates_body_radps,
    )


def run_instance(model, instance: int, seconds: float) -> tuple[float, ...]:
    """在一个模型实例上按网格推进 seconds 秒，返回最终状态向量。"""
    fixed_step_s = model.fixed_step_s
    ticks = round(seconds / fixed_step_s)
    output = None
    for tick in range(ticks):
        elapsed_s = tick * fixed_step_s
        output = model.tick(input_for(instance, elapsed_s), elapsed_s)
        if not output.finite():
            raise RuntimeError(f"instance {instance} non-finite at tick {tick}")
    assert output is not None
    return final_vector(output)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--library", type=Path, default=None)
    parser.add_argument("--instances", type=int, default=16)
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--workers", type=int, default=None)
    args = parser.parse_args()

    # 每个实例一个独立 handle；同一 handle 只被一个线程访问
    library = AvantiLibrary(args.library)
    models = [library.model(0.0) for _ in range(args.instances)]
    try:
        # 第一轮：串行推进全部实例
        serial_start = time.perf_counter()
        serial = [run_instance(model, index, args.seconds) for index, model in enumerate(models)]
        serial_elapsed = time.perf_counter() - serial_start

        # 把全部实例重置到同一初态，保证两轮对比条件一致
        for model in models:
            model.reset(0.0)

        # 第二轮：线程池并行推进（每个实例仍只属于一个 worker 线程）
        parallel_start = time.perf_counter()
        with ThreadPoolExecutor(max_workers=args.workers) as executor:
            futures = [
                executor.submit(run_instance, model, index, args.seconds)
                for index, model in enumerate(models)
            ]
            parallel = [future.result() for future in futures]
        parallel_elapsed = time.perf_counter() - parallel_start

        # 逐分量比较串行与并行的最终结果
        max_abs_difference = max(
            abs(left - right)
            for serial_result, parallel_result in zip(serial, parallel)
            for left, right in zip(serial_result, parallel_result)
        )
        if not math.isfinite(max_abs_difference) or max_abs_difference > 1.0e-12:
            raise AssertionError(f"serial/parallel max difference {max_abs_difference}")

        calls = args.instances * round(args.seconds / models[0].fixed_step_s)
        print(f"instances={args.instances} seconds={args.seconds:g} calls={calls}")
        print(f"serial_s={serial_elapsed:.6f} parallel_s={parallel_elapsed:.6f}")
        print(f"speedup={serial_elapsed / parallel_elapsed:.3f}")
        print(f"max_abs_difference={max_abs_difference:.17g}")
        print("Python multi-instance parallel test passed")
    finally:
        for model in models:
            model.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
