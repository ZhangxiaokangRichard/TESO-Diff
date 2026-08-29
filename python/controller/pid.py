"""张量化 PID 控制器（内环基础，phase2_sp1 已实现）。

对 B 个实例并行解算，无逐实例 Python 循环。积分/微分状态支持
snapshot/restore（counterfactual branching 要求，docs §6.12）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass
class PIDGains:
    """PID 增益（标量广播到全 batch，也可传入 [B] 数组做 gain scheduling）。"""

    kp: float
    ki: float
    kd: float


class PID:
    """位置式 PID，带输出限幅与积分限幅。

    u[k] = kp*e[k] + ki*integral + kd*(e[k]-e[k-1])/dt
    """

    def __init__(
        self,
        kp: float = 0.0,
        ki: float = 0.0,
        kd: float = 0.0,
        dt: float = 0.02,
        output_limits: Optional[Tuple[float, float]] = None,
        integral_limits: Optional[Tuple[float, float]] = None,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.output_limits = output_limits
        self.integral_limits = integral_limits
        self._batch = 0
        self._integral: Optional[np.ndarray] = None
        self._prev_error: Optional[np.ndarray] = None

    def reset(self, batch: int = 1) -> None:
        self._batch = batch
        self._integral = np.zeros(batch, dtype=np.float64)
        self._prev_error = np.zeros(batch, dtype=np.float64)

    def step(self, error: np.ndarray) -> np.ndarray:
        """输入 error [B]，输出控制量 [B]。"""
        if self._integral is None or self._batch != error.shape[0]:
            self.reset(error.shape[0])
        assert self._integral is not None and self._prev_error is not None
        integral = self._integral + error * self.dt
        if self.integral_limits is not None:
            integral = np.clip(integral, *self.integral_limits)
        derivative = (error - self._prev_error) / self.dt
        output = self.kp * error + self.ki * integral + self.kd * derivative
        if self.output_limits is not None:
            output = np.clip(output, *self.output_limits)
        self._integral = integral
        self._prev_error = error
        return output

    def get_snapshot(self) -> dict:
        """导出积分/微分状态（counterfactual 分支必须复制）。"""
        if self._integral is None:
            raise RuntimeError("PID not reset yet")
        return {
            "batch": self._batch,
            "integral": self._integral.copy(),
            "prev_error": self._prev_error.copy(),
        }

    def set_snapshot(self, snap: dict) -> None:
        """恢复积分/微分状态。"""
        self._batch = snap["batch"]
        self._integral = np.array(snap["integral"], dtype=np.float64)
        self._prev_error = np.array(snap["prev_error"], dtype=np.float64)
