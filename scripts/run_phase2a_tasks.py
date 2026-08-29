#!/usr/bin/env python3
"""Phase 2a 封闭航迹闭环任务入口（WP2A-9/WP2A-10，占位）。

最终目标：在无 Isaac Sim 的 Ubuntu 22.04 上完成 3 条封闭航迹任务的
闭环飞行（T11 盘旋 / T12 8 字 / T13 跑马场）+ avanti_eso 张量收集。

当前状态（phase2_sp1）：
- 闭环控制律（WP2A-8）与闭环 harness（WP2A-9）尚未实现；
- 本脚本只做接口级冒烟：加载 .so、运行 1000 tick trim 冒烟、
  检查输出有限性与时间契约，并验证配置 YAML 齐全。

后续实现顺序（docs §52）：PID 内环 → TECS/协调转弯/L1 外环 →
3 条封闭航迹闭环 → wind stub → avanti_eso 张量收集验证。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "python"))

import yaml  # noqa: E402

from bindings.avanti import AvantiInput, AvantiLibrary  # noqa: E402

REQUIRED_CONFIGS = [
    "config/aircraft/avanti2.yaml",
    "config/controller/controller.yaml",
    "config/task/circle.yaml",
    "config/task/figure8.yaml",
    "config/task/racetrack.yaml",
    "config/task/gps_triangle.yaml",
    "config/environment/no_wind.yaml",
    "config/environment/thermal_only.yaml",
    "config/environment/shear_only.yaml",
    "config/environment/composite.yaml",
    "config/environment/wind_smoke.yaml",
    "config/dataset/smoke.yaml",
]


def check_configs() -> None:
    for path in REQUIRED_CONFIGS:
        config_file = PROJECT_ROOT / path
        if not config_file.is_file():
            raise FileNotFoundError(f"missing config: {path}")
        yaml.safe_load(config_file.read_text(encoding="utf-8"))
    print(f"config check passed: {len(REQUIRED_CONFIGS)} files")


def trim_smoke(library_path: Path, ticks: int = 1000) -> None:
    """无风 + trim 控制冒烟（docs §6.4）：验证无 NaN、时间契约与单位。"""
    library = AvantiLibrary(library_path)
    inputs = AvantiInput(throttle_01=0.5)
    with library.model(epoch_s=100.0) as aircraft:
        step = aircraft.fixed_step_s
        last = None
        for tick in range(ticks):
            output = aircraft.tick(inputs)
            if not output.finite():
                raise RuntimeError(f"non-finite output at tick {tick}")
            last = output
        assert last is not None
        print(
            f"trim smoke passed: {ticks} ticks, dt={step:.3f}s, "
            f"final t={last.sample_time_s:.3f}s, "
            f"alt={last.position_ne_alt_m[2]:.3f}m, "
            f"alpha={last.alpha_rad:.4f}rad, beta={last.beta_rad:.4f}rad"
        )
        assert aircraft.next_tick_time_s == 100.0 + ticks * step


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--library",
        type=Path,
        default=PROJECT_ROOT / "runtime" / "fixedwing" / "lib" / "libavanti2.so",
    )
    parser.add_argument("--task", choices=["circle", "figure8", "racetrack"], default=None)
    args = parser.parse_args()

    check_configs()
    trim_smoke(args.library)

    if args.task:
        raise SystemExit(
            f"task '{args.task}' 闭环飞行在 WP2A-8/WP2A-9 完成后启用（当前为占位）"
        )
    print("run_phase2a_tasks smoke finished (closed-loop tasks: WP2A-8~WP2A-10)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
