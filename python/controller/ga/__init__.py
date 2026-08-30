"""控制器参数 GA 整定包（WP2A-8）。

- limited_memory_ga.py：限定记忆 GA 核心（μ+λ、HoF 有限记忆、并行评估）；
- attitude_tuning.py：串级内环姿态跟踪整定评估器（阶段一，PX4 双回路）；
- outer_tuning.py：外环盘旋闭环整定评估器（阶段二，圆任务）；
- multi_task_tuning.py：三任务联合整定评估器（阶段三，circle+figure8+racetrack，
  含批次并行生成有效性测试）。

整定路线（docs §52）：先内环姿态角跟踪，再外环盘旋闭环，
最后多任务联合整定实现稳定封闭航迹跟踪。
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
from .multi_task_tuning import MultiTaskEvaluator  # noqa: F401

__all__ = [
    "LimitedMemoryGA",
    "AttitudeTrackingEvaluator",
    "CircleTaskEvaluator",
    "MultiTaskEvaluator",
    "BOUNDS",
    "GENE_NAMES",
    "SEED_INDIVIDUALS",
    "decode",
]

