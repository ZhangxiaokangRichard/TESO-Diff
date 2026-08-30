#!/usr/bin/env python3
"""WP2A-8 试验 004：三任务联合 GA 整定 + 批次并行生成有效性测试。

流程：
1. 读取 tuning_02 内环整定结果作为固定内环参数；
2. 创建三任务评估器（盘旋 loiter + 空间 8 字 NPFG + 跑马场 NPFG）；
3. 限定记忆 GA 整定外环 16 维参数；
4. 对一批次基因做并行数据生成测试（串行 vs 并行吞吐），
   验证后续张量并行数据采集的有效性；
5. 输出收敛曲线与 result.json。

输出目录：experiments/wp2a8/tuning_04_multitask_ga/
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

EXPERIMENT_DIR = Path(__file__).resolve().parent / "tuning_04_multitask_ga"
PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from mpl_style import new_axes, save_figure, setup_chinese_font  # noqa: E402
from python.controller.ga import LimitedMemoryGA  # noqa: E402
from python.controller.ga.multi_task_tuning import (  # noqa: E402
    BOUNDS,
    GENE_NAMES,
    SEED_INDIVIDUALS,
    MultiTaskEvaluator,
    decode,
)

MU, LAMBDA, GENERATIONS, WORKERS, SEED = 10, 10, 20, 8, 20260830
TUNING_02_RESULT = (
    PROJECT_ROOT / "experiments" / "wp2a8" / "tuning_02_attitude_cascade_ga" / "result.json"
)


def plot_convergence(history: list[dict], out: Path) -> None:
    fig, ax = new_axes(1, 1, figsize=(8.0, 4.5))
    gens = [h["generation"] for h in history]
    ax.plot(gens, [h["best"] for h in history], "o-", label="最优适应度")
    ax.plot(gens, [h["mean"] for h in history], "s-", label="平均适应度")
    ax.plot(gens, [h["worst"] for h in history], "^-", alpha=0.5, label="最差适应度")
    ax.set_xlabel("世代")
    ax.set_ylabel("适应度（越小越好）")
    ax.set_title("限定记忆 GA 收敛曲线（三任务联合整定）")
    ax.legend()
    ax.grid(True, alpha=0.3)
    save_figure(fig, out)


def main() -> int:
    setup_chinese_font()
    EXPERIMENT_DIR.mkdir(parents=True, exist_ok=True)

    print("== WP2A-8 试验 004：三任务联合 GA 整定 ==")
    with open(TUNING_02_RESULT, encoding="utf-8") as f:
        inner_cfg = json.load(f)["inner_cfg"]
    evaluator = MultiTaskEvaluator(inner_cfg)

    print("\n-- 手工种子个体预评估 --")
    for i, genes in enumerate(SEED_INDIVIDUALS):
        cost, stats = evaluator.rollout(genes)
        detail = " ".join(f"{k}={v['cross_mae_m']:.1f}m" for k, v in stats.items())
        print(f"  seed {i}: cost={cost:.2f}  [{detail}]")

    ga = LimitedMemoryGA(
        BOUNDS, mu=MU, lam=LAMBDA, hall_size=20,
        mutation_sigma=0.12, sigma_decay=0.97, tournament_k=3, seed=SEED,
    )
    ga.seed_individuals(SEED_INDIVIDUALS)

    print(f"\n-- GA 运行：μ={MU} λ={LAMBDA} 世代={GENERATIONS} 并行={WORKERS} --")
    start = time.time()
    best = ga.run(GENERATIONS, evaluator, workers=WORKERS, log_every=2)
    elapsed = time.time() - start
    print(f"\nGA 完成：耗时 {elapsed:.1f}s，历史最优适应度 {best.fitness:.3f}")

    cost, stats = evaluator.rollout(best.genes)
    print(f"最优个体复评 cost={cost:.3f}")
    for k, v in stats.items():
        print(f"  {k}: cross={v['cross_mae_m']:.2f}m alt={v['alt_rmse_m']:.2f}m "
              f"tas={v['tas_rmse_mps']:.2f}m/s")

    # ---- 批次并行数据生成测试（μ 个基因 x 8 workers） ----
    print("\n-- 批次并行数据生成有效性测试 --")
    batch = [ind.genes for ind in ga.population]
    bench = evaluator.batch_benchmark(batch, workers=WORKERS)
    print(f"  批次={bench['batch_size']} workers={bench['workers']} "
          f"串行={bench['serial_wall_s']:.1f}s 并行={bench['parallel_wall_s']:.1f}s "
          f"加速比={bench['speedup']:.2f}x "
          f"仿真秒/墙钟秒={bench['sim_seconds_per_wall_second']:.0f}")

    plot_convergence(ga.history, EXPERIMENT_DIR / "ga_convergence.png")

    result = {
        "experiment": "tuning_04_multitask_ga",
        "architecture": "px4_cascade + npfg/loiter + tecs",
        "tasks": ["circle(loiter)", "figure8_spatial(npfg)", "racetrack(npfg)"],
        "inner_cfg_source": str(TUNING_02_RESULT.relative_to(PROJECT_ROOT)),
        "mu": MU, "lambda": LAMBDA, "generations": GENERATIONS,
        "seed": SEED, "elapsed_s": elapsed,
        "best_fitness": best.fitness,
        "genes": {name: float(value) for name, value in zip(GENE_NAMES, best.genes)},
        "outer_cfg": decode(best.genes),
        "per_task_stats": stats,
        "batch_benchmark": bench,
    }
    (EXPERIMENT_DIR / "result.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(f"\n结果已保存到 {EXPERIMENT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
