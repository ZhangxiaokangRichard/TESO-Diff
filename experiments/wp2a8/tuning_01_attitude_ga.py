#!/usr/bin/env python3
"""WP2A-8 试验 001：内环期望姿态角跟踪 PID 的限定记忆 GA 整定。

流程：
1. 创建姿态跟踪评估器（20 s 方波剧本，无风）；
2. 注入符号辨识得到的手工粗调种子个体；
3. 运行限定记忆 GA（μ+λ，Hall of Fame 有限记忆）；
4. 输出收敛曲线、最优个体跟踪曲线与参数，保存到本试验目录。

输出目录：experiments/wp2a8/tuning_01_attitude_ga/
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import numpy as np

EXPERIMENT_DIR = Path(__file__).resolve().parent / "tuning_01_attitude_ga"
PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from mpl_style import new_axes, save_figure, setup_chinese_font  # noqa: E402
from python.controller.ga import (  # noqa: E402
    BOUNDS,
    GENE_NAMES,
    SEED_INDIVIDUALS,
    AttitudeTrackingEvaluator,
    LimitedMemoryGA,
)
from python.controller.ga.attitude_tuning import decode  # noqa: E402

DEG = np.pi / 180.0

MU, LAMBDA, GENERATIONS, WORKERS, SEED = 10, 10, 30, 8, 42


def plot_convergence(history: list[dict], out: Path) -> None:
    fig, ax = new_axes(1, 1, figsize=(8.0, 4.5))
    gens = [h["generation"] for h in history]
    ax.plot(gens, [h["best"] for h in history], "o-", label="最优适应度")
    ax.plot(gens, [h["mean"] for h in history], "s-", label="平均适应度")
    ax.plot(gens, [h["worst"] for h in history], "^-", alpha=0.5, label="最差适应度")
    ax.set_xlabel("世代")
    ax.set_ylabel("适应度（越小越好）")
    ax.set_title("限定记忆 GA 收敛曲线（内环姿态 PID 整定）")
    ax.legend()
    ax.grid(True, alpha=0.3)
    save_figure(fig, out)


def plot_best_tracking(log: dict, out: Path) -> None:
    t = log["t"]
    fig, axes = new_axes(2, 2, figsize=(11.0, 7.0))
    (ax_phi, ax_theta), (ax_psi, ax_d) = axes

    ax_phi.plot(t, np.array(log["phi_cmd"]) / DEG, "r--", label="期望 φ_cmd")
    ax_phi.plot(t, np.array(log["phi"]) / DEG, "b-", label="实际 φ")
    ax_phi.set_xlabel("时间 (s)")
    ax_phi.set_ylabel("滚转角 (°)")
    ax_phi.set_title("滚转角跟踪（±15° 方波）")
    ax_phi.legend()
    ax_phi.grid(True, alpha=0.3)

    ax_theta.plot(t, np.array(log["theta_cmd"]) / DEG, "r--", label="期望 θ_cmd")
    ax_theta.plot(t, np.array(log["theta"]) / DEG, "b-", label="实际 θ")
    ax_theta.set_xlabel("时间 (s)")
    ax_theta.set_ylabel("俯仰角 (°)")
    ax_theta.set_title("俯仰角跟踪")
    ax_theta.legend()
    ax_theta.grid(True, alpha=0.3)

    ax_psi.plot(t, np.array(log["psi"]) / DEG, "g-", label="实际 ψ")
    ax_psi.axhline(0.0, color="r", linestyle="--", label="期望 ψ_cmd=0")
    ax_psi.set_xlabel("时间 (s)")
    ax_psi.set_ylabel("偏航角 (°)")
    ax_psi.set_title("偏航角保持")
    ax_psi.legend()
    ax_psi.grid(True, alpha=0.3)

    ax_d.plot(t, log["delta_a"], label="δa（副翼）")
    ax_d.plot(t, log["delta_e"], label="δe（升降舵）")
    ax_d.plot(t, log["delta_r"], label="δr（方向舵）")
    ax_d.set_xlabel("时间 (s)")
    ax_d.set_ylabel("归一化指令 [-1,1]")
    ax_d.set_title("舵面指令")
    ax_d.legend()
    ax_d.grid(True, alpha=0.3)

    fig.tight_layout()
    save_figure(fig, out)


def main() -> int:
    setup_chinese_font()
    EXPERIMENT_DIR.mkdir(parents=True, exist_ok=True)

    print("== WP2A-8 试验 001：内环姿态跟踪 GA 整定 ==")
    evaluator = AttitudeTrackingEvaluator(duration=20.0, warmup=4.0)

    # 种子个体预检
    print("\n-- 手工种子个体预评估 --")
    for i, genes in enumerate(SEED_INDIVIDUALS):
        cost, _ = evaluator.rollout(genes)
        print(f"  seed {i}: cost={cost:.2f}")

    ga = LimitedMemoryGA(
        BOUNDS, mu=MU, lam=LAMBDA, hall_size=20,
        mutation_sigma=0.12, sigma_decay=0.97, tournament_k=3, seed=SEED,
    )
    ga.seed_individuals(SEED_INDIVIDUALS)

    print(f"\n-- GA 运行：μ={MU} λ={LAMBDA} 世代={GENERATIONS} 并行={WORKERS} --")
    start = time.time()
    best = ga.run(GENERATIONS, evaluator, workers=WORKERS, log_every=2)
    elapsed = time.time() - start
    print(f"\nGA 完成：耗时 {elapsed:.1f}s，历史最优适应度 {best.fitness:.2f}")

    # 最优个体完整 rollout
    cost, log = evaluator.rollout(best.genes)
    print(f"最优个体复评 cost={cost:.2f}")

    plot_convergence(ga.history, EXPERIMENT_DIR / "ga_convergence.png")
    plot_best_tracking(log, EXPERIMENT_DIR / "best_attitude_tracking.png")

    inner_cfg, theta_trim_rad, throttle = decode(best.genes)
    result = {
        "experiment": "tuning_01_attitude_ga",
        "mu": MU, "lambda": LAMBDA, "generations": GENERATIONS,
        "seed": SEED, "elapsed_s": elapsed,
        "best_fitness": best.fitness,
        "genes": {name: float(value) for name, value in zip(GENE_NAMES, best.genes)},
        "theta_trim_deg": float(theta_trim_rad / DEG),
        "throttle": throttle,
        "inner_cfg": inner_cfg,
    }
    (EXPERIMENT_DIR / "result.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(f"\n结果已保存到 {EXPERIMENT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
