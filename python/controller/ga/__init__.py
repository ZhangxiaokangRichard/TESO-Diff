"""控制器参数 GA 整定包（WP2A-8）。

- limited_memory_ga.py：限定记忆 GA 核心（μ+λ、HoF 有限记忆、并行评估）；
- attitude_tuning.py：串级内环姿态跟踪整定评估器（阶段一，PX4 双回路）；
- outer_tuning.py：外环 NPFG/TECS 盘旋闭环整定评估器（阶段二）。

整定路线（docs §52）：先内环姿态角跟踪，再外环盘旋航迹闭环，
最终实现稳定盘旋航迹的位置/姿态控制。
"""

from .limited_memory_ga import LimitedMemoryGA  # noqa: F401
from .attitude_tuning import (  # noqa: F401
    BOUNDS,
    GENE_NAMES,
    SEED_INDIVIDUALS,
    AttitudeTrackingEvaluator,
    decode,
)
from .outer_tuning import CircleTaskEvaluator  # noqa: F401

__all__ = [
    "LimitedMemoryGA",
    "AttitudeTrackingEvaluator",
    "CircleTaskEvaluator",
    "BOUNDS",
    "GENE_NAMES",
    "SEED_INDIVIDUALS",
    "decode",
]

