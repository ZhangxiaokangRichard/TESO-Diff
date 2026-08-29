"""AVANTI2 动态库（C ABI）的 ctypes 高层封装。

本模块通过 ctypes 加载项目构建出的共享库
（Windows: build/win64/dist/avanti2.dll，Linux: build/linux64/dist/libavanti2.so），
并对 interface/avanti_c_api.h 定义的稳定 C ABI 做 Python 侧封装。

使用约定：
- 每架飞机 / 每个仿真环境对应一个 AvantiModel 实例；
- 不同的 AvantiModel 对象可以放在不同线程中并行运行（实例间并行）；
- 同一个 AvantiModel 不得被多个线程并发调用（实例内串行）；
- 时间契约：第 k 次 tick 只接受 global_time_s = epoch_s + k * h（h = 0.02 s），
  仿真长度完全由外部调度器决定，动态库自身不睡眠、不自由运行。
"""

from __future__ import annotations

import ctypes as ct
import math
import os
import sys
from pathlib import Path
from typing import Iterable, Optional


AVANTI_ABI_VERSION = 0x00010000  # ABI 版本号，与 avanti_c_api.h 中的宏保持一致
AVANTI_OK = 0  # C ABI 成功状态码


class AvantiError(RuntimeError):
    """AVANTI C ABI 调用失败时抛出的异常。"""


class AvantiInput(ct.Structure):
    """模型根层 7 个输入，必须与 C ABI 的 AvantiInput 布局完全一致。"""

    _fields_ = [
        ("delta_e_norm", ct.c_double),  # 升降舵归一化指令，建议 [-1,1]，映射约 [-12,+12] deg
        ("delta_a_norm", ct.c_double),  # 副翼归一化指令，建议 [-1,1]，映射约 [-10,+10] deg
        ("delta_r_norm", ct.c_double),  # 方向舵归一化指令，建议 [-1,1]，映射约 [-22,+22] deg
        ("throttle_01", ct.c_double),  # 油门，模型根层饱和到 [0,1]
        ("wind_flat_earth_mps", ct.c_double * 3),  # flat-Earth 三轴风速 m/s（Z_e 向下为正）
    ]

    def set_wind(self, values: Iterable[float]) -> "AvantiInput":
        """设置 flat-Earth 三轴风速 (x, y, z)，单位为 m/s，返回自身以便链式调用。"""
        x, y, z = values
        self.wind_flat_earth_mps[:] = (x, y, z)
        return self


class AvantiOutput(ct.Structure):
    """一次 major tick 的输出快照（1 个标量时间 + 2 个标量 + 7 个三维数组）。"""

    _fields_ = [
        ("sample_time_s", ct.c_double),  # 本输出对应的外部全局时间，单位 s
        ("alpha_rad", ct.c_double),  # 迎角，单位 rad
        ("beta_rad", ct.c_double),  # 侧滑角，单位 rad
        ("position_ne_alt_m", ct.c_double * 3),  # North(Xe)、East(Ye)、Altitude(=-Ze)，m
        ("velocity_ne_up_mps", ct.c_double * 3),  # Ve_x、Ve_y、Up(=-Ve_z)，m/s
        ("euler_rpy_deg", ct.c_double * 3),  # roll、pitch、yaw，deg（已由模型转为角度）
        ("velocity_body_mps", ct.c_double * 3),  # 机体系速度 u、v、w，m/s
        ("rates_body_radps", ct.c_double * 3),  # 机体系角速度 p、q、r，rad/s
        ("rates_body_derivative_radps2", ct.c_double * 3),  # p_dot、q_dot、r_dot，rad/s²
        ("acceleration_body_mps2", ct.c_double * 3),  # 机体系加速度 u_dot、v_dot、w_dot，m/s²
    ]

    def finite(self) -> bool:
        """检查输出中的所有标量与数组分量是否均为有限值（无 NaN/Inf）。"""
        scalar_values = [self.sample_time_s, self.alpha_rad, self.beta_rad]
        arrays = [
            self.position_ne_alt_m,
            self.velocity_ne_up_mps,
            self.euler_rpy_deg,
            self.velocity_body_mps,
            self.rates_body_radps,
            self.rates_body_derivative_radps2,
            self.acceleration_body_mps2,
        ]
        return all(math.isfinite(value) for value in scalar_values) and all(
            math.isfinite(value) for array in arrays for value in array
        )

    def clone(self) -> "AvantiOutput":
        """深拷贝一份输出，避免后续 tick 覆盖原缓冲区。"""
        return AvantiOutput.from_buffer_copy(self)


def default_library_path(project_root: Optional[Path] = None) -> Path:
    """按平台默认路径查找 AVANTI 共享库（Windows DLL / Linux SO）。

    TESO-Diff 目录结构下依次尝试：
    1. runtime/fixedwing/lib/ 中随项目分发的预构建产物；
    2. build/linux64/dist、build/win64/dist 中由根 CMakeLists 构建的产物。
    """
    root = project_root or Path(__file__).resolve().parents[2]
    candidates = (
        root / "runtime" / "fixedwing" / "lib" / "libavanti2.so",
        root / "runtime" / "fixedwing" / "lib" / "avanti2.dll",
        root / "build" / "linux64" / "dist" / "libavanti2.so",
        root / "build" / "linux64" / "dist" / "avanti2.so",
        root / "build" / "win64" / "dist" / "avanti2.dll",
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise FileNotFoundError("No AVANTI shared library found: " + ", ".join(map(str, candidates)))


class AvantiLibrary:
    """AVANTI 共享库加载器：负责打开 DLL/SO、绑定 C ABI 函数签名并做版本校验。"""

    def __init__(self, library_path: Optional[os.PathLike[str] | str] = None):
        self.path = Path(library_path) if library_path else default_library_path()
        self.path = self.path.resolve()
        if not self.path.is_file():
            raise FileNotFoundError(self.path)

        # Windows 上把 DLL 所在目录加入搜索路径，保证依赖库能被解析
        self._dll_directory = None
        if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
            self._dll_directory = os.add_dll_directory(str(self.path.parent))
        self.lib = ct.CDLL(str(self.path))
        self._configure()

        # 加载后必须核对 ABI 版本与结构体尺寸，防止头文件/库版本不匹配
        if self.lib.avanti_abi_version() != AVANTI_ABI_VERSION:
            raise AvantiError("AVANTI ABI version mismatch")
        if self.lib.avanti_input_size() != ct.sizeof(AvantiInput):
            raise AvantiError("AvantiInput size mismatch")
        if self.lib.avanti_output_size() != ct.sizeof(AvantiOutput):
            raise AvantiError("AvantiOutput size mismatch")

    def _configure(self) -> None:
        """为每个 C ABI 函数声明参数类型与返回类型，避免 ctypes 默认按 int 截断。"""
        handle = ct.c_void_p
        lib = self.lib
        lib.avanti_abi_version.restype = ct.c_uint32
        lib.avanti_input_size.restype = ct.c_uint32
        lib.avanti_output_size.restype = ct.c_uint32
        lib.avanti_create.argtypes = [ct.c_double, ct.POINTER(handle)]
        lib.avanti_create.restype = ct.c_int32
        lib.avanti_destroy.argtypes = [handle]
        lib.avanti_destroy.restype = ct.c_int32
        lib.avanti_reset.argtypes = [handle, ct.c_double]
        lib.avanti_reset.restype = ct.c_int32
        lib.avanti_get_fixed_step.argtypes = [handle, ct.POINTER(ct.c_double)]
        lib.avanti_get_fixed_step.restype = ct.c_int32
        lib.avanti_get_next_tick_time.argtypes = [handle, ct.POINTER(ct.c_double)]
        lib.avanti_get_next_tick_time.restype = ct.c_int32
        lib.avanti_tick.argtypes = [
            handle,
            ct.c_double,
            ct.POINTER(AvantiInput),
            ct.POINTER(AvantiOutput),
        ]
        lib.avanti_tick.restype = ct.c_int32
        lib.avanti_advance_to.argtypes = [
            handle,
            ct.c_double,
            ct.POINTER(AvantiInput),
            ct.c_uint64,
            ct.POINTER(AvantiOutput),
        ]
        lib.avanti_advance_to.restype = ct.c_int32

    @staticmethod
    def require_ok(status: int, operation: str) -> None:
        """检查 C ABI 状态码，非 0 时抛出 AvantiError。"""
        if status != AVANTI_OK:
            raise AvantiError(f"{operation} failed with AVANTI status {status}")

    def model(self, epoch_s: float = 0.0) -> "AvantiModel":
        """在指定全局 epoch 上创建一个新的模型实例（对应一个 AvantiHandle）。"""
        return AvantiModel(self, epoch_s)


class AvantiModel:
    """一个 AVANTI 模型实例（对应 C 侧一个 AvantiHandle，即一架飞机）。

    内部持有生成的 AVANTI2_ertmodel0827ModelClass 对象，包含该实例的连续状态、
    ODE4 求解器数据与输入输出。不同实例可并行，同一实例不可并发调用。
    支持 with 上下文管理器，退出时自动销毁 C 侧 handle。
    """

    def __init__(self, library: AvantiLibrary, epoch_s: float = 0.0):
        self.library = library
        self.handle = ct.c_void_p()
        library.require_ok(
            library.lib.avanti_create(epoch_s, ct.byref(self.handle)), "avanti_create"
        )

    @property
    def fixed_step_s(self) -> float:
        """模型固定积分步长（当前为 0.02 s）。"""
        value = ct.c_double()
        self.library.require_ok(
            self.library.lib.avanti_get_fixed_step(self.handle, ct.byref(value)),
            "avanti_get_fixed_step",
        )
        return value.value

    @property
    def next_tick_time_s(self) -> float:
        """下一次 tick 期望的全局时间 = epoch_s + 已完成 tick 数 × 步长。"""
        value = ct.c_double()
        self.library.require_ok(
            self.library.lib.avanti_get_next_tick_time(self.handle, ct.byref(value)),
            "avanti_get_next_tick_time",
        )
        return value.value

    def tick(self, inputs: AvantiInput, global_time_s: Optional[float] = None) -> AvantiOutput:
        """推进一个时间网格点（major tick）。

        参数 global_time_s 默认取实例自身的 next_tick_time_s。
        C 侧会校验时间网格与输入有限性，在 major tick 采样输入并保持整个 ODE4
        步长，调用一次生成类 step()，返回该 major tick 的输出快照。
        """
        time_s = self.next_tick_time_s if global_time_s is None else global_time_s
        output = AvantiOutput()
        self.library.require_ok(
            self.library.lib.avanti_tick(
                self.handle, time_s, ct.byref(inputs), ct.byref(output)
            ),
            "avanti_tick",
        )
        return output

    def advance_to(
        self, inputs: AvantiInput, target_time_s: float, max_substeps: int
    ) -> AvantiOutput:
        """在保持同一输入不变的情况下，连续推进多个网格 tick 直至目标时间。

        仅适用于中间子步输入保持不变的追赶场景；若每个子步控制量都变化，
        必须逐次调用 tick()。max_substeps 是防止失控追赶的保护上限。
        """
        if max_substeps < 0:
            raise ValueError("max_substeps must be non-negative")
        output = AvantiOutput()
        self.library.require_ok(
            self.library.lib.avanti_advance_to(
                self.handle,
                target_time_s,
                ct.byref(inputs),
                max_substeps,
                ct.byref(output),
            ),
            "avanti_advance_to",
        )
        return output

    def reset(self, epoch_s: float = 0.0) -> None:
        """销毁并重建模型，回到固化的编译期初值，并把时间零点设为新 epoch。

        固化初值：位置 [0, 0, -1000] m、u/v/w = [14.1, 0, 0.0251] m/s、
        姿态 [0, 0.00178, 0] rad、角速度 [0, 0, 0] rad/s。
        适用于 episode 重置；当前 ABI 不支持任意初始状态设置。
        """
        self.library.require_ok(
            self.library.lib.avanti_reset(self.handle, epoch_s), "avanti_reset"
        )

    def close(self) -> None:
        """销毁 C 侧 handle，释放模型实例内存（幂等）。"""
        if self.handle:
            self.library.require_ok(
                self.library.lib.avanti_destroy(self.handle), "avanti_destroy"
            )
            self.handle = ct.c_void_p()

    def __enter__(self) -> "AvantiModel":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def __del__(self) -> None:
        # 兜底：对象被垃圾回收时尽量释放 C 侧资源
        try:
            self.close()
        except Exception:
            pass
