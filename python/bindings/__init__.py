"""AVANTI2 6DOF 动态库的 ctypes 封装（复用 phase1 `python/avanti.py`）。

phase1 已冻结稳定 C ABI（`avanti_c_api.h`，版本 0x00010000）：
- `AvantiInput` 7 通道：delta_e/a/r（归一化 [-1,1]）、throttle [0,1]、
  wind_flat_earth_mps[3]（flat-Earth Xe/Ye/Ze，Z 向下为正，m/s）；
- `AvantiOutput` 4 组：alpha/beta（rad）、NEU 位置/速度（Up-positive）、
  Euler（deg）、BODY u,v,w / p,q,r / 导数（SI）；
- 固定步长 dt_plant = 0.02 s（fixed-step ODE4，50 Hz）。

本包不重写 phase1 封装，只做最小路径适配（默认库搜索路径改为
`runtime/fixedwing/lib` / `build/linux64/dist`）。
"""

from .avanti import (  # noqa: F401
    AVANTI_ABI_VERSION,
    AVANTI_OK,
    AvantiError,
    AvantiInput,
    AvantiLibrary,
    AvantiModel,
    AvantiOutput,
    default_library_path,
)

__all__ = [
    "AVANTI_ABI_VERSION",
    "AVANTI_OK",
    "AvantiError",
    "AvantiInput",
    "AvantiLibrary",
    "AvantiModel",
    "AvantiOutput",
    "default_library_path",
]
