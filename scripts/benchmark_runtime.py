#!/usr/bin/env python3
"""6DOF `.so` + Python 封装的吞吐 benchmark（docs §6.16 性能 Gate）。

测量 1/8/32 实例（ThreadPoolExecutor 并行）下的 steps/s 与
simulated seconds / wall second。注意：Phase 1 已实测 Python 线程版本
对单实例没有加速收益（GIL/逐调用开销），本脚本用于记录 Phase 2a 的
性能基线，大规模吞吐结论以 Phase 2b benchmark 为准（docs §6.16/§12）。

用法：
    python scripts/benchmark_runtime.py \
        --library runtime/fixedwing/lib/libavanti2.so \
        --envs 1 8 32 --seconds 60 --workers 8
"""

from __future__ import annotations

import argparse
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "python"))

from bindings.avanti import AvantiInput, AvantiLibrary  # noqa: E402


def run_instance(model, seconds: float) -> int:
    """在一个实例上按网格推进 seconds 秒，返回完成的 tick 数。"""
    inputs = AvantiInput(throttle_01=0.5)
    ticks = round(seconds / model.fixed_step_s)
    for _ in range(ticks):
        model.tick(inputs)
    return ticks


def benchmark(library_path: Path, envs: int, seconds: float, workers: int) -> None:
    library = AvantiLibrary(library_path)
    models = [library.model(epoch_s=100.0 + env * 1000.0) for env in range(envs)]
    try:
        start = time.perf_counter()
        if envs == 1:
            ticks = run_instance(models[0], seconds)
        else:
            with ThreadPoolExecutor(max_workers=min(workers, envs)) as pool:
                results = list(pool.map(lambda m: run_instance(m, seconds), models))
            ticks = sum(results)
        wall = time.perf_counter() - start
        steps_per_s = ticks / wall
        sim_seconds = ticks * 0.02
        print(
            f"envs={envs:>3}  ticks={ticks:>8}  wall={wall:8.3f}s  "
            f"steps/s={steps_per_s:10.1f}  simxrealtime={sim_seconds / wall:7.2f}x"
        )
    finally:
        for model in models:
            model.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--library",
        type=Path,
        default=PROJECT_ROOT / "runtime" / "fixedwing" / "lib" / "libavanti2.so",
    )
    parser.add_argument("--envs", type=int, nargs="+", default=[1, 8, 32])
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args()

    print(f"library={args.library}  seconds={args.seconds}  dt=0.02s")
    for envs in args.envs:
        benchmark(args.library, envs, args.seconds, args.workers)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
