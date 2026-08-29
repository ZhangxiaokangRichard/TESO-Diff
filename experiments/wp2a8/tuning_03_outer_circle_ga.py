#!/usr/bin/env python3
"""WP2A-8 试验 003：外环 NPFG/TECS/协调转弯的限定记忆 GA 整定（盘旋闭环）。

流程：
1. 读取 tuning_02 内环整定结果作为固定内环参数；
2. 创建盘旋闭环评估器（90 s，圆心 (0,60)，无风）；
3. 注入手工粗调种子个体；
4. 运行限定记忆 GA（μ+λ，Hall of Fame 有限记忆）；
5. 输出收敛曲线、最优个体盘旋轨迹/高度/空速曲线与参数。

输出目录：experiments/wp2a8/tuning_03_outer_circle_ga/
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

EXPERIMENT_DIR = Path(__file__).resolve().parent / "tuning_03_outer_circle_ga"
PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from mpl_style import new_axes, save_figure, setup_chinese_font  # noqa: E402
from python.controller.ga import LimitedMemoryGA  # noqa: E402
from python.controller.ga.outer_tuning import (  # noqa: E402
    BOUNDS,
    GENE_NAMES,
    SEED_INDIVIDUALS,
    CircleTaskEvaluator,
    decode,
)

MU, LAMBDA, GENERATIONS, WORKERS, SEED = 10, 10, 25, 8, 20260830
TUNING_02_RESULT = (
    PROJECT_ROOT / "experiments" / "wp2a8" / "tuning_02_attitude_cascade_ga" / "result.json"
)


def load_inner_cfg() -> dict:
    with open(TUNING_02_RESULT, encoding="utf-8") as f:
        return json.load(f)["inner_cfg"]


def plot_convergence(history: list[dict], out: Path) -> None:
    fig, ax = new_axes(1, 1, figsize=(8.0, 4.5))
    gens = [h["generation"] for h in history]
    ax.plot(gens, [h["best"] for h in history], "o-", label="最优适应度")
    ax.plot(gens, [h["mean"] for h in history], "s-", label="平均适应度")
    ax.plot(gens, [h["worst"] for h in history], "^-", alpha=0.5, label="最差适应度")
    ax.set_xlabel("世代")
    ax.set_ylabel("适应度（越小越好）")
    ax.set_title("限定记忆 GA 收敛曲线（外环盘旋整定）")
    ax.legend()
    ax.grid(True, alpha=0.3)
    save_figure(fig, out)


def plot_best_circle(log: dict, out: Path) -> None:
    t = log["t"]
    fig, axes = new_axes(2, 2, figsize=(11.0, 7.5))
    (ax_traj, ax_alt), (ax_tas, ax_att) = axes

    ax_traj.plot(log["north"], log["east"], "b-", label="实际航迹")
    th = np.linspace(0, 2 * np.pi, 361)
    ax_traj.plot(60 * np.cos(th), 60 + 60 * np.sin(th), "r--", label="期望圆（R=60m）")
    ax_traj.set_xlabel("North (m)")
    ax_traj.set_ylabel("East (m)")
    ax_traj.set_title("盘旋航迹跟踪")
    ax_traj.legend()
    ax_traj.axis("equal")
    ax_traj.grid(True, alpha=0.3)

    ax_alt.plot(t, log["alt"], "b-", label="实际高度")
    ax_alt.axhline(1000.0, color="r", linestyle="--", label="期望高度 1000m")
    ax_alt.set_xlabel("时间 (s)")
    ax_alt.set_ylabel("高度 (m)")
    ax_alt.set_title("高度保持（TECS 能量控制）")
    ax_alt.legend()
    ax_alt.grid(True, alpha=0.3)

    ax_tas.plot(t, log["tas"], "b-", label="实际空速")
    ax_tas.axhline(16.0, color="r", linestyle="--", label="期望空速 16m/s")
    ax_tas.set_xlabel("时间 (s)")
    ax_tas.set_ylabel("空速 (m/s)")
    ax_tas.set_title("空速保持")
    ax_tas.legend()
    ax_tas.grid(True, alpha=0.3)

    ax_att.plot(t, np.array(log["phi"]) * 57.3, label="实际 φ")
    ax_att.plot(t, np.array(log["phi_cmd"]) * 57.3, "--", label="期望 φ_cmd")
    ax_att.plot(t, np.array(log["theta"]) * 57.3, label="实际 θ")
    ax_att.set_xlabel("时间 (s)")
    ax_att.set_ylabel("姿态角 (°)")
    ax_att.set_title("滚转/俯仰跟踪")
    ax_att.legend()
    ax_att.grid(True, alpha=0.3)

    fig.tight_layout()
    save_figure(fig, out)


def main() -> int:
    setup_chinese_font()
    EXPERIMENT_DIR.mkdir(parents=True, exist_ok=True)

    print("== WP2A-8 试验 003：外环 NPFG/TECS 盘旋闭环 GA 整定 ==")
    inner_cfg = load_inner_cfg()
    evaluator = CircleTaskEvaluator(inner_cfg, duration=90.0, warmup=15.0)

    print("\n-- 手工种子个体预评估 --")
    for i, genes in enumerate(SEED_INDIVIDUALS):
        cost, _ = evaluator.rollout(genes)
        print(f"  seed {i}: cost={cost:.3f}")

    ga = LimitedMemoryGA(
        BOUNDS, mu=MU, lam=LAMBDA, hall_size=20,
        mutation_sigma=0.12, sigma_decay=0.97, tournament_k=3, seed=SEED,
    )
    ga.seed_individuals(SEED_INDIVIDUALS)

    print(f"\n-- GA 运行：μ={MU} λ={LAMBDA} 世代={GENERATIONS} 并行={WORKERS} --")
    start = time.time()
    best = ga.run(GENERATIONS, evaluator, workers=WORKERS, log_every=5)
    elapsed = time.time() - start
    print(f"\nGA 完成：耗时 {elapsed:.1f}s，历史最优适应度 {best.fitness:.3f}")

    cost, log = evaluator.rollout(best.genes)
    print(f"最优个体复评 cost={cost:.3f}")

    plot_convergence(ga.history, EXPERIMENT_DIR / "ga_convergence.png")
    plot_best_circle(log, EXPERIMENT_DIR / "best_circle_tracking.png")

    outer_cfg = decode(best.genes)
    result = {
        "experiment": "tuning_03_outer_circle_ga",
        "architecture": "px4_cascade + npfg/loiter + tecs",
        "inner_cfg_source": str(TUNING_02_RESULT.relative_to(PROJECT_ROOT)),
        "mu": MU, "lambda": LAMBDA, "generations": GENERATIONS,
        "seed": SEED, "elapsed_s": elapsed,
        "best_fitness": best.fitness,
        "genes": {name: float(value) for name, value in zip(GENE_NAMES, best.genes)},
        "outer_cfg": outer_cfg,
    }
    (EXPERIMENT_DIR / "result.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(f"\n结果已保存到 {EXPERIMENT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
