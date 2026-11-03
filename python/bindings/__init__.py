"""AVANTI2两种独立C ABI：舵面输入avanti，以及姿态输入avanti_att。

phase1 已冻结稳定 C ABI（`avanti_c_api.h`，版本 0x00010000）：
- `AvantiInput` 7 通道：delta_e/a/r（归一化 [-1,1]）、throttle [0,1]、
  wind_flat_earth_mps[3]（flat-Earth Xe/Ye/Ze，Z 向下为正，m/s）；
- `AvantiOutput` 4 组：alpha/beta（rad）、NEU 位置/速度（Up-positive）、
  Euler（deg）、BODY u,v,w / p,q,r / 导数（SI）；
- 固定步长 dt_plant = 0.02 s（fixed-step ODE4，50 Hz）。

姿态版输入phi/theta(rad)、油门、风，包含生成的姿态环；其14个增益在进程内共享。
两种封装均按操作系统从各自runtime目录或build/dist中查找库。
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

from .avanti_att import (  # noqa: F401,E402
    AVANTI_ATT_ABI_VERSION, AvantiAttError, AvantiAttInput, AvantiAttLibrary,
    AvantiAttModel, AvantiAttOutput, AvantiAttParams,
)
__all__ += ["AVANTI_ATT_ABI_VERSION", "AvantiAttError", "AvantiAttInput",
            "AvantiAttLibrary", "AvantiAttModel", "AvantiAttOutput", "AvantiAttParams"]
