"""avanti_eso 张量收集包（docs §6.11/§6.13）。

闭环飞行过程中连续记录：风场真值、获能动作/控制器中间量、状态与时空
位置（全局时间 + 相对时间），形成编码器输入输出真值流。
"""

from .history import HistoryBuffer  # noqa: F401
from .tensor_recorder import TensorRecorder  # noqa: F401

__all__ = ["HistoryBuffer", "TensorRecorder"]
