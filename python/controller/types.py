"""控制器张量数据结构（Phase 2a 冻结接口，docs/TESO-Diff_phase2.md §6.12）。

所有字段均为 numpy 数组：标量通道 shape 为 [B]，向量通道 shape 为 [B,3]。
单位约定（内部统一 SI + rad）：
- 角度一律 rad（来自 FWOutput 的 Euler 为 deg，由上层在构造观测时转换）；
- 舵面输出为归一化指令 [-1,1]，油门 [0,1]（与 AvantiInput 一致）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np


def _check_batch(*arrays: np.ndarray) -> int:
    """校验一组张量的 batch 维一致，返回 batch 大小。"""
    batch = arrays[0].shape[0]
    for array in arrays:
        if array.shape[0] != batch:
            raise ValueError(
                f"batch mismatch: expected {batch}, got {array.shape[0]}"
            )
    return batch


@dataclass
class ControlReference:
    """任务/外环参考输入（batch 化）。

    外环输入为期望空间位置（由任务航线解算给出），输出为期望航向、
    期望航迹倾角/俯仰、期望坡度与油门。第一阶段 primitive 至少支持：
    V_cmd、phi_cmd、gamma_cmd/theta_cmd、yaw_rate_cmd（可选）、
    throttle_mode/cT_cmd、duration。
    """

    airspeed_cmd: np.ndarray          # [B] V_cmd，m/s
    altitude_cmd: np.ndarray          # [B] h_cmd，m（TECS）
    path: np.ndarray                  # [B, N_waypoints, 3] 期望空间位置，NEU
    turn_rate_cmd: Optional[np.ndarray] = None  # [B] rad/s，协调转弯用，可选
    gamma_cmd: Optional[np.ndarray] = None       # [B] rad，期望航迹倾角，可选
    throttle_mode: str = "auto"       # auto(TECS) | manual
    cT_cmd: Optional[np.ndarray] = None          # [B] 手动油门指令 [0,1]

    def __post_init__(self) -> None:
        batch = _check_batch(self.airspeed_cmd, self.altitude_cmd)
        if self.path.shape[0] != batch:
            raise ValueError("path batch mismatch")


@dataclass
class ControlObservation:
    """来自 FWOutput 的机体系 + NEU 观测（内部统一 rad）。

    neu_pos / neu_vel / tas 为 WP2A-8 新增（外环 L1/TECS 需要），
    可选字段：None 时表示未提供。
    """

    u: np.ndarray   # [B] m/s
    v: np.ndarray   # [B] m/s
    w: np.ndarray   # [B] m/s
    p: np.ndarray   # [B] rad/s
    q: np.ndarray   # [B] rad/s
    r: np.ndarray   # [B] rad/s
    phi: np.ndarray    # [B] rad（构造时由 deg 转换）
    theta: np.ndarray  # [B] rad
    psi: np.ndarray    # [B] rad
    alpha: np.ndarray  # [B] rad
    beta: np.ndarray   # [B] rad
    neu_pos: Optional[np.ndarray] = None  # [B,3] North/East/Altitude，m（Up-positive）
    neu_vel: Optional[np.ndarray] = None  # [B,3] Ve_n/Ve_e/Up，m/s
    tas: Optional[np.ndarray] = None       # [B] 真空速近似（无风时 = 机体速度模），m/s
    wind_neu: Optional[np.ndarray] = None  # [B,2] 飞控风估计（North/East 水平分量，m/s）
                                           # NPFG 制导用，WP2A-8 v2 新增

    def __post_init__(self) -> None:
        batch = _check_batch(self.u, self.v, self.w, self.p, self.q, self.r,
                             self.phi, self.theta, self.psi, self.alpha, self.beta)
        for array in (self.neu_pos, self.neu_vel):
            if array is not None and array.shape != (batch, 3):
                raise ValueError(f"neu array shape mismatch: {array.shape}")
        if self.wind_neu is not None and self.wind_neu.shape != (batch, 2):
            raise ValueError(f"wind_neu shape mismatch: {self.wind_neu.shape}")
        if self.tas is not None and self.tas.shape[0] != batch:
            raise ValueError("tas batch mismatch")

    @classmethod
    def from_fw_outputs(cls, outputs) -> "ControlObservation":
        """从一批 python.bindings 输出（Euler 为 deg）构造观测。

        outputs: shape [B] 的 AvantiOutput 序列（每实例一个）。
        """
        batch = len(outputs)
        deg = np.pi / 180.0
        body_vel = np.array([list(o.velocity_body_mps) for o in outputs])
        return cls(
            u=body_vel[:, 0],
            v=body_vel[:, 1],
            w=body_vel[:, 2],
            p=np.array([o.rates_body_radps[0] for o in outputs]),
            q=np.array([o.rates_body_radps[1] for o in outputs]),
            r=np.array([o.rates_body_radps[2] for o in outputs]),
            phi=np.array([o.euler_rpy_deg[0] for o in outputs]) * deg,
            theta=np.array([o.euler_rpy_deg[1] for o in outputs]) * deg,
            psi=np.array([o.euler_rpy_deg[2] for o in outputs]) * deg,
            alpha=np.array([o.alpha_rad for o in outputs]),
            beta=np.array([o.beta_rad for o in outputs]),
            neu_pos=np.array([list(o.position_ne_alt_m) for o in outputs]),
            neu_vel=np.array([list(o.velocity_ne_up_mps) for o in outputs]),
            tas=np.linalg.norm(body_vel, axis=1),
            wind_neu=None,  # 风估计由飞控风估计器（FC wind estimate）填充
        )


@dataclass
class ActuatorCommand:
    """控制器输出 = AvantiInput 的舵面/油门部分。"""

    delta_a: np.ndarray   # [B] 归一化 [-1,1]
    delta_e: np.ndarray   # [B] 归一化 [-1,1]
    delta_r: np.ndarray   # [B] 归一化 [-1,1]
    throttle: np.ndarray  # [B] [0,1]

    def __post_init__(self) -> None:
        batch = _check_batch(self.delta_a, self.delta_e,
                             self.delta_r, self.throttle)
        for name, array in (("delta_a", self.delta_a),
                            ("delta_e", self.delta_e),
                            ("delta_r", self.delta_r)):
            if np.any(np.abs(array) > 1.0):
                raise ValueError(f"{name} exceeds normalized range [-1,1]")
        if np.any(self.throttle < 0.0) or np.any(self.throttle > 1.0):
            raise ValueError("throttle exceeds [0,1]")


@dataclass
class AttitudeCommand:
    """Generated-attitude runtime command. No fabricated surface channels."""
    phi: np.ndarray
    theta: np.ndarray
    throttle: np.ndarray

    def __post_init__(self):
        _check_batch(self.phi, self.theta, self.throttle)
        if not all(np.isfinite(x).all() for x in (self.phi, self.theta, self.throttle)):
            raise ValueError("nonfinite attitude command")
        if np.any(np.abs(self.phi)>=np.pi/2) or np.any(np.abs(self.theta)>=np.pi/2):
            raise ValueError("attitude command outside supported Euler range")
        if np.any((self.throttle<0)|(self.throttle>1)):
            raise ValueError("throttle outside [0,1]")

    def to_inputs(self, wind_flat_earth_mps=None):
        from ..bindings.avanti_att import AvantiAttInput
        batch=len(self.phi)
        wind=np.zeros((batch,3)) if wind_flat_earth_mps is None else np.asarray(wind_flat_earth_mps)
        if wind.shape!=(batch,3) or not np.isfinite(wind).all():
            raise ValueError("wind must be finite [B,3]")
        return [AvantiAttInput(float(self.phi[i]),float(self.theta[i]),float(self.throttle[i])).set_wind(wind[i])
                for i in range(batch)]
