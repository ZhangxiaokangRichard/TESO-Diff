"""协调转弯律（外环，WP2A-8 v2）。

参考《PX4固定翼控制器详解(二)》：
    侧向加速度 → 期望坡度：phi_cmd = atan2(a_cmd, g)
    协调转弯偏航角速率：r_ff = g·tan(phi)·cos(theta)/V_TAS
        （等高度转弯 tan φ = V·ψ̇/g；爬升转弯乘 cos θ）

r_ff 在 InnerLoop 内作为偏航通道前馈实现（InnerLoop.coordinated_yaw_rate_ff）。
本模块负责 a_cmd → phi_cmd，含最大坡度限幅与滚转角速率限制。
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from .types import ControlObservation


class CoordinatedTurn:
    """协调转弯律（全张量化）。"""

    G = 9.80665

    def __init__(self, cfg: dict):
        self.bank_limit = float(np.deg2rad(cfg.get("bank_limit_deg", 30.0)))
        self.bank_rate_limit = float(
            np.deg2rad(cfg.get("bank_rate_limit_degps", 15.0))
        )
        self.dt = float(cfg.get("dt", 0.02))
        self._prev_phi_cmd: Optional[np.ndarray] = None

    def reset(self, batch: int = 1) -> None:
        self._prev_phi_cmd = np.zeros(batch)

    def step(self, a_cmd: np.ndarray, obs: ControlObservation) -> np.ndarray:
        """输入期望横向加速度 [B] m/s²，输出期望坡度 [B] rad。

        PX4 口径：phi_cmd = atan2(a_cmd, g)，其中 a_cmd 为 L1/NPFG 横向
        加速度（水平面），坡度限幅 + 滚转角速率限制防超包线。
        """
        phi_cmd = np.arctan2(np.clip(a_cmd, -25.0, 25.0), self.G)
        phi_cmd = np.clip(phi_cmd, -self.bank_limit, self.bank_limit)

        if self._prev_phi_cmd is None or self._prev_phi_cmd.shape[0] != a_cmd.shape[0]:
            self.reset(a_cmd.shape[0])
        assert self._prev_phi_cmd is not None
        delta_max = self.bank_rate_limit * self.dt
        phi_cmd = self._prev_phi_cmd + np.clip(
            phi_cmd - self._prev_phi_cmd, -delta_max, delta_max
        )
        self._prev_phi_cmd = phi_cmd
        return phi_cmd

    def get_snapshot(self) -> dict:
        if self._prev_phi_cmd is None:
            raise RuntimeError("CoordinatedTurn not reset yet")
        return {"prev_phi_cmd": self._prev_phi_cmd.copy()}

    def set_snapshot(self, snap: dict) -> None:
        self._prev_phi_cmd = np.array(snap["prev_phi_cmd"])
