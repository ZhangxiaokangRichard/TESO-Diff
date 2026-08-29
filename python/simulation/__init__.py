"""仿真运行时包（Phase 2a，无 Isaac Sim）。

闭环 Python harness（docs §6.13）把 plant/controller 问题与 Isaac 接口
问题隔离，同时验证 avanti_eso 张量流完整性。
"""

from .harness import ClosedLoopHarness  # noqa: F401
from .wind_stub import WindStub  # noqa: F401

__all__ = ["ClosedLoopHarness", "WindStub"]
