"""AVANTI2 姿态内环控制器动态库（C ABI）的 ctypes 高层封装。

本模块通过 ctypes 加载项目构建出的共享库
（Windows: build/win64/dist/avanti2_att.dll，
  Linux:   build/linux64/dist/libavanti2_att.so），
并对 interface/avanti_att_c_api.h 定义的稳定 C ABI 做 Python 侧封装。

与 python/avanti.py（AVANTI2_ertmodel0827，舵面指令接口）的区别：
- 根输入是期望姿态角（phi_sp/theta_sp）+ 归一化油门 + 风速，不是舵面偏转；
- 模型内置姿态内环控制器（外环 P + 内环 PID + 协调转弯前馈），自动解算舵面；
- 多出 14 个控制器增益的运行时读写接口（AvantiAttParams）。

使用约定：
- 每架飞机 / 每个仿真环境对应一个 AvantiAttModel 实例（线程间可并行，
  同一实例不可并发调用，语义与 avanti.py 完全相同）；
- 增益（AvantiAttParams）是“进程级”状态，由生成代码的 C++ class 实现方式
  决定（Embedded Coder "C++ class" 目标对独立顶层模型只支持一份 class-static
  参数结构体，不支持逐实例参数）：同一进程内所有 AvantiAttModel / AvantiAttHandle
  共用一份增益，set_params() 立刻影响该进程内所有实例的下一次 tick；
  需要不同增益并行运行时，必须用多个进程（例如 ProcessPoolExecutor），
  不能指望同进程的多线程/多实例获得互不干扰的增益。
"""

from __future__ import annotations

import ctypes as ct
import math
import os
import sys
from pathlib import Path
from typing import Iterable, Optional


AVANTI_ATT_ABI_VERSION = 0x00010000
AVANTI_ATT_OK = 0

# AvantiAttParams 字段顺序，和 avanti_att_c_api.h / matlab/px4att_gain_names.m 一致
PARAM_FIELD_NAMES = (
    "p_phi_sp",
    "p_theta_sp",
    "fw_rr",
    "fw_pr",
    "fw_yr",
    "omega_sp_p_phi",
    "omega_sp_i_phi",
    "omega_sp_d_phi",
    "omega_sp_p_theta",
    "omega_sp_i_theta",
    "omega_sp_d_theta",
    "omega_sp_p_psi",
    "omega_sp_i_psi",
    "omega_sp_d_psi",
)


class AvantiAttError(RuntimeError):
    """AVANTI 姿态内环 C ABI 调用失败时抛出的异常。"""


class AvantiAttInput(ct.Structure):
    """模型根层 4 个输入，必须与 C ABI 的 AvantiAttInput 布局完全一致。"""

    _fields_ = [
        ("phi_sp_rad", ct.c_double),  # 期望滚转角，rad（建议 |value| <= 0.7854）
        ("theta_sp_rad", ct.c_double),  # 期望俯仰角，rad（建议 |value| <= 0.7854）
        ("throttle_01", ct.c_double),  # 归一化油门，模型根层饱和到 [0,1]
        ("wind_flat_earth_mps", ct.c_double * 3),  # flat-Earth 三轴风速 m/s
    ]

    def set_wind(self, values: Iterable[float]) -> "AvantiAttInput":
        """设置 flat-Earth 三轴风速 (x, y, z)，单位为 m/s，返回自身以便链式调用。"""
        x, y, z = values
        self.wind_flat_earth_mps[:] = (x, y, z)
        return self


class AvantiAttOutput(ct.Structure):
    """一次 major tick 的输出快照，字段语义与 avanti.py 的 AvantiOutput 相同
    （共用同一个 Plant 子系统）。"""

    _fields_ = [
        ("sample_time_s", ct.c_double),
        ("alpha_rad", ct.c_double),
        ("beta_rad", ct.c_double),
        ("position_ne_alt_m", ct.c_double * 3),
        ("velocity_ne_up_mps", ct.c_double * 3),
        ("euler_rpy_deg", ct.c_double * 3),
        ("velocity_body_mps", ct.c_double * 3),
        ("rates_body_radps", ct.c_double * 3),
        ("rates_body_derivative_radps2", ct.c_double * 3),
        ("acceleration_body_mps2", ct.c_double * 3),
    ]

    def finite(self) -> bool:
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

    def clone(self) -> "AvantiAttOutput":
        return AvantiAttOutput.from_buffer_copy(self)


class AvantiAttParams(ct.Structure):
    """14 个姿态内环控制器增益，字段顺序与 avanti_att_c_api.h 的
    AvantiAttParams 完全一致。进程内所有实例共享同一份增益。"""

    _fields_ = [(name, ct.c_double) for name in PARAM_FIELD_NAMES]

    def as_dict(self) -> dict[str, float]:
        return {name: getattr(self, name) for name in PARAM_FIELD_NAMES}

    @classmethod
    def from_dict(cls, values: dict[str, float]) -> "AvantiAttParams":
        params = cls()
        for name in PARAM_FIELD_NAMES:
            setattr(params, name, float(values[name]))
        return params


def default_library_path(project_root: Optional[Path] = None) -> Path:
    """按平台默认路径查找姿态内环共享库（Windows DLL / Linux SO）。"""
    root = project_root or Path(__file__).resolve().parents[2]
    if sys.platform == "win32":
        candidates = (root / "runtime/fixedwing_att/lib/avanti2_att.dll",
                      root / "build/win64/dist/avanti2_att.dll")
    else:
        candidates = (root / "runtime/fixedwing_att/lib/libavanti2_att.so",
                      root / "build/linux64/dist/libavanti2_att.so")
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        "No AVANTI attitude-loop shared library found: " + ", ".join(map(str, candidates))
    )


class AvantiAttLibrary:
    """姿态内环共享库加载器：打开 DLL/SO、绑定 C ABI 函数签名并做版本校验。"""

    def __init__(self, library_path: Optional[os.PathLike[str] | str] = None):
        self.path = Path(library_path) if library_path else default_library_path()
        self.path = self.path.resolve()
        if not self.path.is_file():
            raise FileNotFoundError(self.path)

        self._dll_directory = None
        if sys.platform == "win32" and hasattr(os, "add_dll_directory"):
            self._dll_directory = os.add_dll_directory(str(self.path.parent))
        self.lib = ct.CDLL(str(self.path))
        self._configure()

        if self.lib.avanti_att_abi_version() != AVANTI_ATT_ABI_VERSION:
            raise AvantiAttError("AVANTI_ATT ABI version mismatch")
        if self.lib.avanti_att_input_size() != ct.sizeof(AvantiAttInput):
            raise AvantiAttError("AvantiAttInput size mismatch")
        if self.lib.avanti_att_output_size() != ct.sizeof(AvantiAttOutput):
            raise AvantiAttError("AvantiAttOutput size mismatch")
        if self.lib.avanti_att_params_size() != ct.sizeof(AvantiAttParams):
            raise AvantiAttError("AvantiAttParams size mismatch")

    def _configure(self) -> None:
        handle = ct.c_void_p
        lib = self.lib
        lib.avanti_att_abi_version.restype = ct.c_uint32
        lib.avanti_att_input_size.restype = ct.c_uint32
        lib.avanti_att_output_size.restype = ct.c_uint32
        lib.avanti_att_params_size.restype = ct.c_uint32

        lib.avanti_att_create.argtypes = [ct.c_double, ct.POINTER(handle)]
        lib.avanti_att_create.restype = ct.c_int32
        lib.avanti_att_destroy.argtypes = [handle]
        lib.avanti_att_destroy.restype = ct.c_int32
        lib.avanti_att_reset.argtypes = [handle, ct.c_double]
        lib.avanti_att_reset.restype = ct.c_int32
        lib.avanti_att_get_fixed_step.argtypes = [handle, ct.POINTER(ct.c_double)]
        lib.avanti_att_get_fixed_step.restype = ct.c_int32
        lib.avanti_att_get_next_tick_time.argtypes = [handle, ct.POINTER(ct.c_double)]
        lib.avanti_att_get_next_tick_time.restype = ct.c_int32
        lib.avanti_att_tick.argtypes = [
            handle,
            ct.c_double,
            ct.POINTER(AvantiAttInput),
            ct.POINTER(AvantiAttOutput),
        ]
        lib.avanti_att_tick.restype = ct.c_int32
        lib.avanti_att_advance_to.argtypes = [
            handle,
            ct.c_double,
            ct.POINTER(AvantiAttInput),
            ct.c_uint64,
            ct.POINTER(AvantiAttOutput),
        ]
        lib.avanti_att_advance_to.restype = ct.c_int32

        lib.avanti_att_get_params.argtypes = [ct.POINTER(AvantiAttParams)]
        lib.avanti_att_get_params.restype = ct.c_int32
        lib.avanti_att_set_params.argtypes = [ct.POINTER(AvantiAttParams)]
        lib.avanti_att_set_params.restype = ct.c_int32
        lib.avanti_att_get_default_params.argtypes = [ct.POINTER(AvantiAttParams)]
        lib.avanti_att_get_default_params.restype = ct.c_int32

    @staticmethod
    def require_ok(status: int, operation: str) -> None:
        if status != AVANTI_ATT_OK:
            raise AvantiAttError(f"{operation} failed with AVANTI_ATT status {status}")

    def model(self, epoch_s: float = 0.0) -> "AvantiAttModel":
        """在指定全局 epoch 上创建一个新的模型实例（对应一个 AvantiAttHandle）。"""
        return AvantiAttModel(self, epoch_s)

    def get_params(self) -> AvantiAttParams:
        """读取当前生效的 14 个控制器增益（进程级，与具体实例无关）。"""
        params = AvantiAttParams()
        self.require_ok(self.lib.avanti_att_get_params(ct.byref(params)), "avanti_att_get_params")
        return params

    def set_params(self, params: AvantiAttParams) -> None:
        """写入新的 14 个增益；立刻对进程内所有实例的下一次 tick 生效。"""
        self.require_ok(self.lib.avanti_att_set_params(ct.byref(params)), "avanti_att_set_params")

    def get_default_params(self) -> AvantiAttParams:
        """读取代码生成时固化的出厂默认增益（不随 set_params 改变）。"""
        params = AvantiAttParams()
        self.require_ok(
            self.lib.avanti_att_get_default_params(ct.byref(params)),
            "avanti_att_get_default_params",
        )
        return params


class AvantiAttModel:
    """一个姿态内环模型实例（对应 C 侧一个 AvantiAttHandle，即一架飞机）。

    不同实例可并行（线程或进程），同一实例不可并发调用。控制器增益不属于
    某个实例，见 AvantiAttLibrary.get_params/set_params。
    """

    def __init__(self, library: AvantiAttLibrary, epoch_s: float = 0.0):
        self.library = library
        self.handle = ct.c_void_p()
        library.require_ok(
            library.lib.avanti_att_create(epoch_s, ct.byref(self.handle)),
            "avanti_att_create",
        )

    @property
    def fixed_step_s(self) -> float:
        value = ct.c_double()
        self.library.require_ok(
            self.library.lib.avanti_att_get_fixed_step(self.handle, ct.byref(value)),
            "avanti_att_get_fixed_step",
        )
        return value.value

    @property
    def next_tick_time_s(self) -> float:
        value = ct.c_double()
        self.library.require_ok(
            self.library.lib.avanti_att_get_next_tick_time(self.handle, ct.byref(value)),
            "avanti_att_get_next_tick_time",
        )
        return value.value

    def tick(self, inputs: AvantiAttInput, global_time_s: Optional[float] = None) -> AvantiAttOutput:
        time_s = self.next_tick_time_s if global_time_s is None else global_time_s
        output = AvantiAttOutput()
        self.library.require_ok(
            self.library.lib.avanti_att_tick(
                self.handle, time_s, ct.byref(inputs), ct.byref(output)
            ),
            "avanti_att_tick",
        )
        return output

    def advance_to(
        self, inputs: AvantiAttInput, target_time_s: float, max_substeps: int
    ) -> AvantiAttOutput:
        if max_substeps < 0:
            raise ValueError("max_substeps must be non-negative")
        output = AvantiAttOutput()
        self.library.require_ok(
            self.library.lib.avanti_att_advance_to(
                self.handle,
                target_time_s,
                ct.byref(inputs),
                max_substeps,
                ct.byref(output),
            ),
            "avanti_att_advance_to",
        )
        return output

    def reset(self, epoch_s: float = 0.0) -> None:
        """销毁并重建模型，回到固化的编译期初值。不影响控制器增益
        （增益是进程级状态，见 AvantiAttLibrary.set_params）。"""
        self.library.require_ok(
            self.library.lib.avanti_att_reset(self.handle, epoch_s), "avanti_att_reset"
        )

    def close(self) -> None:
        if self.handle:
            self.library.require_ok(
                self.library.lib.avanti_att_destroy(self.handle), "avanti_att_destroy"
            )
            self.handle = ct.c_void_p()

    def __enter__(self) -> "AvantiAttModel":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass
