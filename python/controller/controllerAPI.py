"""飞行控制器标准指令接口（WP2A-8 最终步骤，API 层）。

FlightController 包装 AvantiController，提供两种高层控制模式：

    1. LoiterCommand（指点盘旋）
       - 输入：期望圆心 NEU 坐标、期望空速、盘旋半径（≥ MIN_LOITER_RADIUS）
       - 行为：飞向圆心并在接近后保持圆形航迹；L1 圆形制导自动检测

    2. HeadingCommand（直飞模式）
       - 输入：期望空速、航迹倾角 gamma（rad）、航迹偏角 chi（rad，从北顺时针）
       - 行为：按指定空速/gamma/chi 飞行；
              TECS 用动态高度设定点 alt_cmd = h + V·sin(γ)/kp_h 等效 gamma 指令；
              L1 跟踪从当前位置在 chi 方向生成的前视直线路径。

使用::

    cfg = load_tuned()
    fc = FlightController(cfg)
    fc.reset(batch=1)

    # 指点盘旋
    fc.command_loiter(LoiterCommand(center=(0.0, 200.0, 1000.0), radius=60.0, airspeed=16.0))
    actuator = fc.step(obs)   # obs.neu_pos 必须有效

    # 直飞模式
    fc.command_heading(HeadingCommand(airspeed=16.0, gamma=0.0, chi=0.0))
    actuator = fc.step(obs)   # 每 step 动态重新生成前视路径
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .avanti_controller import AvantiController
from .types import ActuatorCommand, ControlObservation, ControlReference

MIN_LOITER_RADIUS: float = 30.0   # m，最小盘旋半径
_HEADING_N_PTS: int = 20          # 直飞模式前视路径点数
_HEADING_PATH_DIST: float = 1000.0  # m，每步生成的前视路径总长度


@dataclass
class LoiterCommand:
    """指点盘旋指令。"""
    center: tuple[float, float, float]  # NEU：(North, East, Altitude) m
    airspeed: float                     # 期望指示空速，m/s
    radius: float                       # 盘旋半径，m
    timestamp: float = 0.0             # 指令时间戳，s


@dataclass
class HeadingCommand:
    """直飞模式指令。"""
    airspeed: float    # 期望指示空速，m/s
    gamma: float       # 期望航迹倾角，rad（正为爬升）
    chi: float         # 期望航迹偏角，rad（从北顺时针）
    timestamp: float = 0.0  # 指令时间戳，s


class FlightController:
    """高层飞行控制器：包装 AvantiController，提供 Loiter/Heading 两种控制模式。

    Loiter 模式：command_loiter() 预生成圆形路径，step() 每次传入同一路径；
                 L1 自动检测圆形（std_r / mean_r < 1e-3）并切换为 PX4 Circle 制导。

    Heading 模式：command_heading() 仅存储指令，不重置积分器；
                  step() 每次从当前位置生成前视直线路径，同时动态更新 TECS 高度设定点：
                  alt_cmd = current_alt + V·sin(γ)/kp_h，实现等效 gamma 跟踪。
    """

    _MODE_NONE = 0
    _MODE_LOITER = 1
    _MODE_HEADING = 2

    def __init__(self, cfg: dict, min_loiter_radius: float = MIN_LOITER_RADIUS):
        self._cfg = cfg
        self._min_r = min_loiter_radius
        self._ctrl = AvantiController(cfg)
        self._batch = 1
        self._mode = self._MODE_NONE
        self._loiter_cmd: Optional[LoiterCommand] = None
        self._heading_cmd: Optional[HeadingCommand] = None
        self._circle_path: Optional[np.ndarray] = None  # [B, N, 3]
        # kp_h：TECS 高度-爬升率增益，用于 heading 模式动态高度设定点
        self._kp_h: float = float(
            cfg.get("outer", {}).get("tecs", {}).get("height_kp", 0.1)
        )

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    def reset(self, batch: int = 1) -> None:
        """重置控制器积分器/滤波器状态，清除当前指令模式。"""
        self._batch = batch
        self._ctrl.reset(batch)
        self._mode = self._MODE_NONE
        self._loiter_cmd = None
        self._heading_cmd = None
        self._circle_path = None

    # ------------------------------------------------------------------
    # 指令接口
    # ------------------------------------------------------------------

    def command_loiter(self, cmd: LoiterCommand) -> None:
        """发出指点盘旋指令。

        半径低于 MIN_LOITER_RADIUS 时自动截幅到最小值。
        预生成圆形路径（721 点）并存储；L1 在 step() 中自动检测圆形。
        """
        r = max(float(cmd.radius), self._min_r)
        path_np = _make_circle(
            center_ne=(cmd.center[0], cmd.center[1]),
            radius=r,
            altitude=cmd.center[2],
            n_pts=721,
        )
        # 扩展为 [B, N, 3]
        self._circle_path = np.tile(path_np[None, :, :], (self._batch, 1, 1))
        self._loiter_cmd = LoiterCommand(
            center=cmd.center,
            airspeed=float(cmd.airspeed),
            radius=r,
            timestamp=float(cmd.timestamp),
        )
        self._mode = self._MODE_LOITER

    def command_heading(self, cmd: HeadingCommand) -> None:
        """发出直飞模式指令。

        只存储指令，不重置 TECS/PID 积分器，保证控制量平滑切换。
        下一次 step() 开始生效。
        """
        self._heading_cmd = HeadingCommand(
            airspeed=float(cmd.airspeed),
            gamma=float(cmd.gamma),
            chi=float(cmd.chi),
            timestamp=float(cmd.timestamp),
        )
        self._mode = self._MODE_HEADING

    # ------------------------------------------------------------------
    # 控制步进
    # ------------------------------------------------------------------

    def step(self, obs: ControlObservation) -> ActuatorCommand:
        """解算一步控制指令，返回归一化舵面 + 油门指令。

        必须在 command_loiter() 或 command_heading() 之后调用。
        Heading 模式要求 obs.neu_pos 非 None。
        """
        if self._mode == self._MODE_LOITER:
            return self._step_loiter(obs)
        if self._mode == self._MODE_HEADING:
            return self._step_heading(obs)
        raise RuntimeError(
            "FlightController.step() called before any command issued; "
            "call command_loiter() or command_heading() first."
        )

    def _step_loiter(self, obs: ControlObservation) -> ActuatorCommand:
        cmd = self._loiter_cmd
        ref = ControlReference(
            airspeed_cmd=np.full(self._batch, cmd.airspeed),
            altitude_cmd=np.full(self._batch, cmd.center[2]),
            path=self._circle_path,
        )
        return self._ctrl.step(ref, obs)

    def _step_heading(self, obs: ControlObservation) -> ActuatorCommand:
        cmd = self._heading_cmd
        if obs.neu_pos is None:
            raise ValueError(
                "HeadingCommand requires obs.neu_pos (NEU position). "
                "Ensure ControlObservation.from_fw_outputs() is used or neu_pos is set."
            )

        pos = obs.neu_pos                # [B, 3]
        n_unit = float(np.cos(cmd.chi))
        e_unit = float(np.sin(cmd.chi))
        dists = np.linspace(0.0, _HEADING_PATH_DIST, _HEADING_N_PTS)  # [n_pts]

        # 真空速（用于动态高度设定点；obs.tas 可能为 None 时退回到指令空速）
        tas = obs.tas if obs.tas is not None else np.full(self._batch, cmd.airspeed)

        # alt_cmd = h + V·sin(γ)/kp_h  →  TECS climb_sp = kp_h·(alt_cmd-h) = V·sin(γ)
        alt_cmd = pos[:, 2] + tas * float(np.sin(cmd.gamma)) / max(self._kp_h, 1e-6)

        # 前视直线路径 [B, n_pts, 3]
        path = np.empty((self._batch, _HEADING_N_PTS, 3))
        path[:, :, 0] = pos[:, 0:1] + dists[None, :] * n_unit  # North
        path[:, :, 1] = pos[:, 1:2] + dists[None, :] * e_unit  # East
        path[:, :, 2] = alt_cmd[:, None]                        # Altitude

        # 重置 L1 参考点索引（路径每步重生成，始终从 idx=0 开始搜索）
        self._ctrl.outer.l1._ref_index = np.zeros(self._batch, dtype=np.int64)

        ref = ControlReference(
            airspeed_cmd=np.full(self._batch, cmd.airspeed),
            altitude_cmd=alt_cmd,
            path=path,
        )
        return self._ctrl.step(ref, obs)

    # ------------------------------------------------------------------
    # 状态导出
    # ------------------------------------------------------------------

    def intermediates(self) -> dict:
        """转发 AvantiController 中间张量（供 avanti_eso 编码器和日志记录）。"""
        return self._ctrl.intermediates()

    def get_snapshot(self) -> dict:
        """导出控制器积分器/滤波器完整快照（counterfactual 用）。"""
        return {
            "controller": self._ctrl.get_snapshot(),
            "mode": self._mode,
        }

    def set_snapshot(self, snap: dict) -> None:
        """恢复控制器快照。"""
        self._ctrl.set_snapshot(snap["controller"])
        # mode 和指令不在快照中：由调用方重新发指令


# --------------------------------------------------------------------------
# 内部辅助（避免从 simulation.task_route 引入循环依赖）
# --------------------------------------------------------------------------

def _make_circle(
    center_ne: tuple[float, float],
    radius: float,
    altitude: float,
    n_pts: int = 721,
) -> np.ndarray:
    """生成平面圆形路径 [N, 3]（NEU，逆时针）。"""
    theta = np.linspace(0.0, 2.0 * np.pi, n_pts)
    north = center_ne[0] + radius * np.cos(theta)
    east = center_ne[1] + radius * np.sin(theta)
    alt = np.full(n_pts, float(altitude))
    return np.stack([north, east, alt], axis=1)
