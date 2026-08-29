"""限定记忆遗传算法（Limited-Memory GA，μ+λ）。

“限定记忆”的含义：
1. 种群规模受限：每代只保留 μ 个个体（μ+λ 环境选择）；
2. Hall of Fame 有限：历史最优记忆最多保留 hall_size 个，超限淘汰最差，
   避免记忆无限膨胀（对照全量历史记忆的简单 GA）；
3. 精英保留 + 锦标赛选择 + BLX 算术交叉 + 退火高斯变异。

评估函数按 batch 并行（evaluate_fn(list[genes]) -> list[fitness]），
由调用方保证每个个体评估相互独立。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Optional, Sequence

import numpy as np


@dataclass
class Individual:
    genes: np.ndarray
    fitness: float
    info: dict = field(default_factory=dict)


class LimitedMemoryGA:
    """实数编码限定记忆 GA。"""

    def __init__(
        self,
        bounds: Sequence[tuple[float, float]],
        mu: int = 10,
        lam: int = 10,
        hall_size: int = 20,
        mutation_sigma: float = 0.12,
        sigma_decay: float = 0.97,
        tournament_k: int = 3,
        seed: int = 0,
    ):
        self.bounds = np.asarray(bounds, dtype=np.float64)
        self.mu = mu
        self.lam = lam
        self.hall_size = hall_size
        self.mutation_sigma = mutation_sigma
        self.sigma_decay = sigma_decay
        self.tournament_k = tournament_k
        self.rng = np.random.default_rng(seed)
        self.population: list[Individual] = []
        self.hall_of_fame: list[Individual] = []  # 有限历史最优记忆
        self.generation = 0
        self.history: list[dict] = []
        self.best_ever: Optional[Individual] = None

    # ------------------------------------------------------------------ #
    # 初始化
    # ------------------------------------------------------------------ #
    def _sample_genes(self) -> np.ndarray:
        low, high = self.bounds[:, 0], self.bounds[:, 1]
        return self.rng.uniform(low, high)

    def _clip(self, genes: np.ndarray) -> np.ndarray:
        return np.clip(genes, self.bounds[:, 0], self.bounds[:, 1])

    def seed_individuals(self, genes_list: Sequence[np.ndarray]) -> None:
        """把手工/先验个体注入初始种群（符号辨识得到的粗调值）。"""
        self.population = [
            Individual(self._clip(np.asarray(g, dtype=np.float64)), np.inf)
            for g in genes_list
        ]

    # ------------------------------------------------------------------ #
    # 变异与交叉
    # ------------------------------------------------------------------ #
    def _mutate(self, genes: np.ndarray) -> np.ndarray:
        span = self.bounds[:, 1] - self.bounds[:, 0]
        noise = self.rng.normal(0.0, self.mutation_sigma * span)
        return self._clip(genes + noise)

    def _crossover(self, p1: np.ndarray, p2: np.ndarray) -> np.ndarray:
        # BLX-0.25 风格算术交叉：child = p1 + alpha*(p2-p1)，alpha ~ U(-0.25, 1.25)
        alpha = self.rng.uniform(-0.25, 1.25, size=p1.shape)
        return self._clip(p1 + alpha * (p2 - p1))

    def _tournament(self, pool: list[Individual]) -> Individual:
        picks = [pool[i] for i in
                 self.rng.integers(0, len(pool), size=self.tournament_k)]
        return min(picks, key=lambda ind: ind.fitness)

    # ------------------------------------------------------------------ #
    # Hall of Fame（有限记忆）
    # ------------------------------------------------------------------ #
    def _update_hall(self, evaluated: list[Individual]) -> None:
        self.hall_of_fame.extend(evaluated)
        # 按基因去重（保留更优 fitness）
        unique: dict[bytes, Individual] = {}
        for ind in sorted(self.hall_of_fame, key=lambda x: x.fitness):
            key = np.round(ind.genes, 6).tobytes()
            unique.setdefault(key, ind)
        self.hall_of_fame = sorted(
            unique.values(), key=lambda x: x.fitness
        )[: self.hall_size]  # 记忆有限：只保留最优 hall_size 个

    # ------------------------------------------------------------------ #
    # 主循环
    # ------------------------------------------------------------------ #
    def run(
        self,
        generations: int,
        evaluate_fn: Callable[[list[np.ndarray]], list[float]],
        workers: int = 1,
        log_every: int = 1,
        verbose: bool = True,
    ) -> Individual:
        """运行 GA，返回历史最优个体。"""
        from concurrent.futures import ThreadPoolExecutor

        if len(self.population) < self.mu:
            while len(self.population) < self.mu:
                self.population.append(Individual(self._sample_genes(), np.inf))
            self.population = self.population[: self.mu]

        # 初始评估
        self._evaluate(evaluate_fn, workers)

        for gen in range(1, generations + 1):
            self.generation = gen
            # 1) 锦标赛选亲本 → 交叉 + 变异生成 λ 个子代
            offspring: list[Individual] = []
            for _ in range(self.lam):
                p1 = self._tournament(self.population)
                p2 = self._tournament(self.population)
                child = self._crossover(p1.genes, p2.genes)
                child = self._mutate(child)
                offspring.append(Individual(child, np.inf))
            self._evaluate(evaluate_fn, workers, offspring)

            # 2) 环境选择（μ+λ）：精英保留 + 锦标赛回填
            pool = self.population + offspring
            elite = min(pool, key=lambda ind: ind.fitness)
            survivors = [elite]
            while len(survivors) < self.mu:
                chosen = self._tournament(pool)
                if not any(chosen is s for s in survivors):
                    survivors.append(chosen)
            self.population = survivors

            # 3) 更新有限记忆 HoF
            self._update_hall(self.population)

            # 4) 变异步长退火
            self.mutation_sigma = max(1e-4, self.mutation_sigma * self.sigma_decay)

            best = min(self.population, key=lambda ind: ind.fitness)
            if self.best_ever is None or best.fitness < self.best_ever.fitness:
                self.best_ever = Individual(best.genes.copy(), best.fitness,
                                            dict(best.info))
            fits = [ind.fitness for ind in self.population]
            self.history.append({
                "generation": gen,
                "best": best.fitness,
                "mean": float(np.mean(fits)),
                "worst": max(fits),
                "sigma": self.mutation_sigma,
            })
            if verbose and gen % log_every == 0:
                print(f"  gen {gen:3d}  best={best.fitness:9.2f}  "
                      f"mean={np.mean(fits):9.2f}  worst={max(fits):9.2f}")

        return self.best_ever or min(self.population, key=lambda ind: ind.fitness)

    def _evaluate(
        self,
        evaluate_fn: Callable[[list[np.ndarray]], list[float]],
        workers: int,
        subset: Optional[list[Individual]] = None,
    ) -> None:
        targets = subset if subset is not None else self.population
        genes_batch = [ind.genes for ind in targets]
        if workers > 1 and len(genes_batch) > 1:
            from concurrent.futures import ThreadPoolExecutor

            with ThreadPoolExecutor(max_workers=workers) as pool:
                results = list(pool.map(evaluate_fn, genes_batch))
        else:
            results = [evaluate_fn([g])[0] for g in genes_batch]
        for ind, fitness in zip(targets, results):
            ind.fitness = float(fitness)
