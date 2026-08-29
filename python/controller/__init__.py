"""Python 张量并行飞行控制器包（Phase 2a 核心，不编译控制器 `.so`）。

控制层级（docs/TESO-Diff_phase2.md §6.11）：

    封闭航迹任务（期望空间位置序列）
            ↓
    外环 outer.py：
      L1 位置控制 l1.py         —— 位置误差 → 期望横向加速度/期望航向 χ_cmd
      TECS 总能量控制 tecs.py   —— 高度与空速误差 → throttle_cmd + 俯仰能量指令
      协调转弯律 coordinated_turn.py —— 期望航向变化率 → 期望坡度 phi_cmd
            ↓
    内环 inner.py：
      PID 姿态控制 pid.py       —— phi/theta/psi → δa/δe/δr 归一化指令
            ↓
    AvantiInput（7 通道）→ 6DOF `.so`

全部模块张量化：字段均为 [B] 或 [B,3] 的 numpy 数组，`step()` 内部无逐实例
Python 循环。Phase 2b Isaac/CUDA 后端直接平移复用。

接口约定（docs/TESO-Diff_phase2.md §6.12）：
- `ControlReference`  任务/外环参考输入（batch 化）
- `ControlObservation` 来自 FWOutput 的机体系观测（内部统一 rad）
- `ActuatorCommand`   控制器输出 = AvantiInput 的舵面/油门部分
- `AvantiController`  全张量化控制器门面

当前状态（phase2_sp1）：接口与数据结构已冻结，`PID` 已实现；
TECS / L1 / 协调转弯 / 内外环整合在 WP2A-8 中实现。
"""

from .types import (  # noqa: F401
    ActuatorCommand,
    ControlObservation,
    ControlReference,
)
from .pid import PID  # noqa: F401
from .avanti_controller import AvantiController  # noqa: F401

__all__ = [
    "ActuatorCommand",
    "ControlObservation",
    "ControlReference",
    "PID",
    "AvantiController",
]
