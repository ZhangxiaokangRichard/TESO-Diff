"""WP2A-8b action API for the generated attitude-loop aircraft.

FlightController: batched NPFG/TECS -> AttitudeCommand (rad, rad, throttle).
HeadingCommand keeps IAS, ground gamma/chi and an absolute activation time.
LoiterCommand approaches the NEU center and latches circle capture within 2R.
LegacySurfaceFlightController retains the historical surface-output backend.
Snapshots here cover the Python outer loop only, not the C++ plant state.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .avanti_controller import AvantiController
from .types import ActuatorCommand, AttitudeCommand, ControlObservation, ControlReference
from .outer import OuterLoop
import copy

MIN_LOITER_RADIUS: float = 30.0   # m，最小盘旋半径
_HEADING_N_PTS: int = 20          # 直飞模式前视路径点数
_HEADING_PATH_DIST: float = 1000.0  # m，每步生成的前视路径总长度


def _wrap_pi(angle):
    """将标量或数组角度环绕到 [-pi, pi]。"""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


@dataclass
class LoiterCommand:
    """指点盘旋指令。"""
    center: tuple[float, float, float]  # NEU：(North, East, Altitude) m
    airspeed: float                     # 期望指示空速，m/s
    radius: float                       # 盘旋半径，m
    timestamp: float = 0.0             # 指令时间戳，s
    direction: int = 1                  # +1 North->East, -1 opposite


@dataclass
class HeadingCommand:
    """直飞模式指令。"""
    airspeed: float    # 期望指示空速，m/s
    gamma: float       # 期望航迹倾角，rad（正为爬升）
    chi: float         # 期望航迹偏角，rad（从北顺时针）
    timestamp: float = 0.0  # 指令时间戳，s


@dataclass(frozen=True)
class PathFollowerStatus:
    """PolylinePathFollower 最近一次更新的诊断状态。"""

    segment_index: int
    lap_count: int
    progress_m: float
    lookahead_m: float
    signed_cross_track_m: float
    target_neu: tuple[float, float, float]
    chi_raw: float
    chi_command: float
    updated: bool


class PolylinePathFollower:
    """将连续采样路径转换为可长期保持的 HeadingCommand。

    与“进入固定半径后切换单个航点”的逻辑不同，本类维护单调弧长进度，
    将当前位置投影到上一进度附近的线段，并在路径弧长前方选择虚拟目标。
    因此错过某个采样点不会卡住，在 8 字交叉点也不会跳到另一分支。

    当前类对应单架 aircraft 对象；Phase 2b 可将相同状态扩展为 batch 张量。
    """

    def __init__(
        self,
        path: np.ndarray,
        airspeed: float,
        gamma: float = 0.0,
        *,
        closed: bool = True,
        lookahead_time_s: float = 2.5,
        lookahead_min_m: float = 30.0,
        lookahead_max_m: float = 60.0,
        cross_track_gain: float = 1.0,
        heading_rate_limit_degps: float = 18.0,
        update_period_s: float = 0.1,
        search_back_segments: int = 2,
        search_forward_segments: int = 20,
        progress_rate_factor: float = 3.0,
    ):
        points = np.asarray(path, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3 or points.shape[0] < 3:
            raise ValueError("path must have shape [N,3] with N >= 3")
        if not np.isfinite(points).all():
            raise ValueError("path contains NaN/Inf")
        if closed and np.linalg.norm(points[0] - points[-1]) < 1e-9:
            points = points[:-1]
        if points.shape[0] < 3:
            raise ValueError("path has too few distinct points")
        if airspeed <= 0.0:
            raise ValueError("airspeed must be positive")
        if lookahead_time_s <= 0.0 or lookahead_min_m <= 0.0:
            raise ValueError("lookahead parameters must be positive")
        if lookahead_max_m < lookahead_min_m:
            raise ValueError("lookahead_max_m must be >= lookahead_min_m")
        if update_period_s <= 0.0:
            raise ValueError("update_period_s must be positive")

        self.path = points
        self.airspeed = float(airspeed)
        self.gamma = float(gamma)
        self.closed = bool(closed)
        self.lookahead_time_s = float(lookahead_time_s)
        self.lookahead_min_m = float(lookahead_min_m)
        self.lookahead_max_m = float(lookahead_max_m)
        self.cross_track_gain = float(cross_track_gain)
        self.heading_rate_limit = float(np.deg2rad(heading_rate_limit_degps))
        self.update_period_s = float(update_period_s)
        self.search_back_segments = max(0, int(search_back_segments))
        self.search_forward_segments = max(1, int(search_forward_segments))
        self.progress_rate_factor = max(1.0, float(progress_rate_factor))

        if self.closed:
            p0 = points
            p1 = np.roll(points, -1, axis=0)
        else:
            p0 = points[:-1]
            p1 = points[1:]
        self._seg_start = p0
        self._seg_vec = p1 - p0
        self._seg_len = np.linalg.norm(self._seg_vec[:, :2], axis=1)
        if np.any(self._seg_len <= 1e-6):
            raise ValueError("path contains a zero-length horizontal segment")
        self._cum = np.concatenate(([0.0], np.cumsum(self._seg_len)))
        self._total_len = float(self._cum[-1])
        self._n_seg = len(self._seg_len)
        self._max_seg_len = float(np.max(self._seg_len))

        self._progress_m = 0.0
        self._last_timestamp: Optional[float] = None
        self._last_chi: Optional[float] = None
        self._last_command: Optional[HeadingCommand] = None
        self._status: Optional[PathFollowerStatus] = None

    def reset(
        self,
        initial_segment: int = 0,
        initial_chi: Optional[float] = None,
        timestamp: Optional[float] = None,
    ) -> None:
        """重置路径进度；initial_segment 用于指定交叉路径的起始分支。"""
        if not 0 <= initial_segment < self._n_seg:
            raise ValueError("initial_segment out of range")
        self._progress_m = float(self._cum[initial_segment])
        self._last_timestamp = None if timestamp is None else float(timestamp)
        self._last_chi = None if initial_chi is None else float(_wrap_pi(initial_chi))
        self._last_command = None
        self._status = None

    def update(
        self,
        position_neu: np.ndarray,
        timestamp: float,
        ground_speed: Optional[float] = None,
    ) -> HeadingCommand:
        """根据当前 NEU 位置更新路径进度并返回 HeadingCommand。"""
        pos = np.asarray(position_neu, dtype=np.float64)
        if pos.shape != (3,) or not np.isfinite(pos).all():
            raise ValueError("position_neu must be a finite [3] vector")
        now = float(timestamp)
        if self._last_timestamp is not None and now < self._last_timestamp - 1e-9:
            raise ValueError("path follower timestamp moved backwards")

        if (
            self._last_timestamp is not None
            and now - self._last_timestamp < self.update_period_s - 1e-9
            and self._last_command is not None
        ):
            if self._status is not None:
                self._status = PathFollowerStatus(
                    **{**self._status.__dict__, "updated": False}
                )
            return self._last_command

        dt = (
            self.update_period_s
            if self._last_timestamp is None
            else max(now - self._last_timestamp, 1e-6)
        )
        speed = self.airspeed if ground_speed is None else max(float(ground_speed), 0.0)
        lookahead = float(np.clip(
            speed * self.lookahead_time_s,
            self.lookahead_min_m,
            self.lookahead_max_m,
        ))

        projection, tangent, progress = self._project_local(pos, speed, dt)
        self._progress_m = progress
        target = self._point_at(progress + lookahead)
        chord = target[:2] - projection[:2]
        if np.linalg.norm(chord) <= 1e-9:
            chord = tangent
        chi_path = float(np.arctan2(chord[1], chord[0]))

        offset = pos[:2] - projection[:2]
        signed_cross = float(tangent[0] * offset[1] - tangent[1] * offset[0])
        correction = float(np.arctan2(
            self.cross_track_gain * signed_cross,
            max(lookahead, 1e-6),
        ))
        chi_raw = float(_wrap_pi(chi_path - correction))

        if self._last_chi is None:
            chi_command = chi_raw
        else:
            delta = float(_wrap_pi(chi_raw - self._last_chi))
            max_delta = self.heading_rate_limit * dt
            chi_command = float(_wrap_pi(
                self._last_chi + np.clip(delta, -max_delta, max_delta)
            ))

        lap_count = int(np.floor(progress / self._total_len)) if self.closed else 0
        progress_mod = progress % self._total_len if self.closed else progress
        segment_index = int(
            min(np.searchsorted(self._cum, progress_mod, side="right") - 1,
                self._n_seg - 1)
        )
        self._last_timestamp = now
        self._last_chi = chi_command
        self._last_command = HeadingCommand(
            airspeed=self.airspeed,
            gamma=self.gamma,
            chi=chi_command,
            timestamp=now,
        )
        self._status = PathFollowerStatus(
            segment_index=segment_index,
            lap_count=lap_count,
            progress_m=progress,
            lookahead_m=lookahead,
            signed_cross_track_m=signed_cross,
            target_neu=(float(target[0]), float(target[1]), float(target[2])),
            chi_raw=chi_raw,
            chi_command=chi_command,
            updated=True,
        )
        return self._last_command

    def status(self) -> PathFollowerStatus:
        if self._status is None:
            raise RuntimeError("PolylinePathFollower.update() has not been called")
        return self._status

    def get_snapshot(self) -> dict:
        """导出路径进度和航向限速状态，供分支 rollout 精确复现。"""
        return {
            "progress_m": self._progress_m,
            "last_timestamp": self._last_timestamp,
            "last_chi": self._last_chi,
            "last_command": None if self._last_command is None else {
                "airspeed": self._last_command.airspeed,
                "gamma": self._last_command.gamma,
                "chi": self._last_command.chi,
                "timestamp": self._last_command.timestamp,
            },
            "status": None if self._status is None else dict(self._status.__dict__),
        }

    def set_snapshot(self, snap: dict) -> None:
        """恢复由 get_snapshot() 导出的路径跟随器状态。"""
        progress = float(snap["progress_m"])
        if progress < 0.0 or (not self.closed and progress > self._total_len + 1e-9):
            raise ValueError("snapshot progress outside path range")
        self._progress_m = progress
        self._last_timestamp = (
            None if snap["last_timestamp"] is None
            else float(snap["last_timestamp"])
        )
        self._last_chi = (
            None if snap["last_chi"] is None else float(snap["last_chi"])
        )
        command = snap["last_command"]
        self._last_command = (
            None if command is None else HeadingCommand(**command)
        )
        status = snap["status"]
        self._status = None if status is None else PathFollowerStatus(**status)

    def _project_local(
        self,
        pos: np.ndarray,
        speed: float,
        dt: float,
    ) -> tuple[np.ndarray, np.ndarray, float]:
        """在受限的前向弧长窗口内投影，避免错点、倒退和交叉串支。"""
        progress_mod = self._progress_m % self._total_len if self.closed else self._progress_m
        current_idx = int(min(
            np.searchsorted(self._cum, progress_mod, side="right") - 1,
            self._n_seg - 1,
        ))
        current_lap = int(np.floor(self._progress_m / self._total_len)) if self.closed else 0
        current_unwrapped = current_lap * self._n_seg + current_idx
        max_advance = max(
            self.progress_rate_factor * max(speed, self.airspeed * 0.5) * dt,
            2.0 * self._max_seg_len,
        )
        min_progress = max(0.0, self._progress_m - self._max_seg_len)
        max_progress = self._progress_m + max_advance

        best = None
        for offset_idx in range(-self.search_back_segments,
                                self.search_forward_segments + 1):
            unwrapped_idx = current_unwrapped + offset_idx
            if not self.closed and not 0 <= unwrapped_idx < self._n_seg:
                continue
            idx = unwrapped_idx % self._n_seg
            lap = int(np.floor(unwrapped_idx / self._n_seg)) if self.closed else 0
            seg_start_progress = lap * self._total_len + self._cum[idx]
            vec_xy = self._seg_vec[idx, :2]
            length = self._seg_len[idx]
            t = float(np.clip(
                np.dot(pos[:2] - self._seg_start[idx, :2], vec_xy)
                / (length * length),
                0.0,
                1.0,
            ))
            candidate_progress = seg_start_progress + t * length
            if candidate_progress < min_progress - 1e-9:
                continue
            if candidate_progress > max_progress + 1e-9:
                continue
            candidate = self._seg_start[idx] + t * self._seg_vec[idx]
            distance_sq = float(np.sum((candidate[:2] - pos[:2]) ** 2))
            key = (distance_sq, -candidate_progress)
            if best is None or key < best[0]:
                tangent = vec_xy / length
                best = (key, candidate, tangent, candidate_progress)

        if best is None:
            progress = self._progress_m
            point, tangent = self._point_and_tangent(progress)
            return point, tangent, progress

        progress = max(self._progress_m, float(best[3]))
        point, tangent = self._point_and_tangent(progress)
        return point, tangent, progress

    def _point_and_tangent(self, progress_m: float) -> tuple[np.ndarray, np.ndarray]:
        if self.closed:
            s = progress_m % self._total_len
        else:
            s = float(np.clip(progress_m, 0.0, self._total_len))
        idx = int(min(
            np.searchsorted(self._cum, s, side="right") - 1,
            self._n_seg - 1,
        ))
        t = float(np.clip(
            (s - self._cum[idx]) / self._seg_len[idx], 0.0, 1.0
        ))
        point = self._seg_start[idx] + t * self._seg_vec[idx]
        tangent = self._seg_vec[idx, :2] / self._seg_len[idx]
        return point, tangent

    def _point_at(self, progress_m: float) -> np.ndarray:
        return self._point_and_tangent(progress_m)[0]


class LegacySurfaceFlightController:
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


def density_ratio(altitude_m):
    """Low-speed IAS/EAS conversion using the model's ISA troposphere law.

    The current ABI has no pressure/density sensor; this is an ISA estimate.
    """
    h=np.clip(np.asarray(altitude_m,dtype=float),-500,11000)
    return (1-.0065*h/288.15)**(9.80665/(287.05287*.0065)-1)


class FlightController:
    """Batched action interface -> generated inner-loop attitude commands.

    Commands can target a subset of batch indices; all outer-loop state is
    per-instance. timestamp is absolute simulation time, including epoch.
    Call step at 50 Hz. Native handles are managed by AttitudeFleet or caller.
    """
    NONE=0
    HEADING=1
    LOITER=2
    APPROACH=0
    CAPTURE=1
    HOLD=2

    def __init__(self,cfg=None,min_loiter_radius=MIN_LOITER_RADIUS):
        if cfg is None:
            from pathlib import Path
            import yaml
            cfg=yaml.safe_load((Path(__file__).resolve().parents[2]/'config/controller/attitude_runtime.yaml').read_text(encoding='utf-8'))
        self.cfg=copy.deepcopy(cfg)
        self.outer=OuterLoop(copy.deepcopy(cfg.get('outer',{})))
        self.dt=float(self.outer.tecs.dt)
        if not np.isclose(self.dt,.02) or not np.isclose(self.outer.turn.dt,.02):
            raise ValueError('generated runtime API requires 0.02 s outer updates')
        if not 0<self.outer.turn.bank_limit<np.pi/2:
            raise ValueError('bank limit must be strictly between 0 and 90 degrees')
        self.min_radius=float(min_loiter_radius)
        if not np.isfinite(self.min_radius) or self.min_radius<=0:
            raise ValueError('minimum radius must be positive')
        self.reset()

    def reset(self,batch=1):
        if not isinstance(batch,int) or batch<1: raise ValueError('positive integer batch required')
        self.batch=batch
        self.outer.reset(batch)
        self._mode=np.zeros(batch,dtype=int)
        self._phase=np.zeros(batch,dtype=int)
        self._center=np.zeros((batch,3))
        self._radius=np.full(batch,self.min_radius)
        self._requested_radius=self._radius.copy()
        self._direction=np.ones(batch)
        self._ias=np.full(batch,16.)
        self._gamma=np.zeros(batch)
        self._chi=np.zeros(batch)
        self._alt_ref=np.full(batch,np.nan)
        self._pending=[]
        self._last_submitted=np.full(batch,-np.inf)
        self._submitted_commands=[None for _ in range(batch)]
        self._last_time=None
        self._intermediates={}

    def _indices(self,indices):
        idx=np.arange(self.batch) if indices is None else np.atleast_1d(np.asarray(indices))
        if idx.ndim!=1 or idx.dtype.kind not in 'iu' or len(idx)==0:
            raise ValueError('indices must be a nonempty integer vector')
        if np.any((idx<0)|(idx>=self.batch)) or len(np.unique(idx))!=len(idx):
            raise ValueError('batch indices out of range or duplicated')
        return idx.astype(int)

    def _submit(self,cmd,indices):
        if not isinstance(cmd,(HeadingCommand,LoiterCommand)):
            raise TypeError('HeadingCommand or LoiterCommand required')
        cmd=copy.deepcopy(cmd)
        if isinstance(cmd,LoiterCommand):
            center=np.asarray(cmd.center,dtype=float)
            if center.shape!=(3,):raise ValueError('center must be NEU [3]')
            cmd.center=tuple(center)
        idx=self._indices(indices)
        # Path adapters may return the same held, timestamped action at 50 Hz.
        # Re-submitting it is idempotent; it must not re-anchor the height path.
        duplicate=np.array([cmd==self._submitted_commands[i] for i in idx])
        idx=idx[~duplicate]
        if not len(idx):return
        values=[cmd.airspeed,cmd.timestamp]
        if isinstance(cmd,LoiterCommand):
            if np.asarray(cmd.center).shape!=(3,):raise ValueError('center must be NEU [3]')
            values.extend([*cmd.center,cmd.radius])
            if cmd.radius<=0 or cmd.direction not in (-1,1):raise ValueError('positive radius and direction +/-1 required')
        else:
            values.extend([cmd.gamma,cmd.chi])
            if abs(cmd.gamma)>self.outer.tecs.gamma_limit:raise ValueError('gamma exceeds TECS operating range')
        if not np.isfinite(values).all() or cmd.airspeed<=0:raise ValueError('finite command with positive IAS required')
        if np.any(cmd.timestamp<self._last_submitted[idx]-1e-9):raise ValueError('command timestamp moved backwards')
        if self._last_time is not None and cmd.timestamp<self._last_time-1e-9:
            raise ValueError('command timestamp is before the last control update')
        self._last_submitted[idx]=cmd.timestamp
        for i in idx:self._submitted_commands[i]=copy.deepcopy(cmd)
        self._pending.append((float(cmd.timestamp),idx,copy.deepcopy(cmd)))
        self._pending.sort(key=lambda item:item[0])

    def command_heading(self,cmd:HeadingCommand,indices=None):
        self._submit(cmd,indices)

    def command_loiter(self,cmd:LoiterCommand,indices=None):
        self._submit(cmd,indices)

    def _activate(self,now,obs):
        keep=[]
        for timestamp,idx,cmd in self._pending:
            if timestamp>now+1e-9:
                keep.append((timestamp,idx,cmd));continue
            self._ias[idx]=cmd.airspeed
            if isinstance(cmd,HeadingCommand):
                fresh=self._mode[idx]!=self.HEADING
                self._alt_ref[idx[fresh]]=obs.neu_pos[idx[fresh],2]
                self._gamma[idx]=cmd.gamma
                self._chi[idx]=_wrap_pi(cmd.chi)
                self._mode[idx]=self.HEADING
            else:
                self._mode[idx]=self.LOITER
                self._phase[idx]=self.APPROACH
                self._center[idx]=cmd.center
                self._requested_radius[idx]=cmd.radius
                self._radius[idx]=max(cmd.radius,self.min_radius)
                self._direction[idx]=cmd.direction
                self._alt_ref[idx]=cmd.center[2]
        self._pending=keep

    def step(self,obs:ControlObservation,timestamp=None)->AttitudeCommand:
        if obs.neu_pos is None or obs.neu_vel is None or obs.tas is None:
            raise ValueError('NEU position/velocity and true airspeed required')
        if len(obs.phi)!=self.batch:raise ValueError('observation batch mismatch; reset(batch) first')
        now=(0. if self._last_time is None else self._last_time+self.dt) if timestamp is None else float(timestamp)
        if not np.isfinite(now):raise ValueError('finite simulation timestamp required')
        tolerance=max(1e-8,64*np.finfo(float).eps*abs(now))
        if self._last_time is not None and not np.isclose(now-self._last_time,self.dt,atol=tolerance,rtol=0):
            raise ValueError('control timestamp must advance exactly 0.02 s')
        self._activate(now,obs)
        if np.any(self._mode==self.NONE):raise RuntimeError('every batch instance needs an active command')
        tas_cmd=self._ias/np.sqrt(density_ratio(obs.neu_pos[:,2]))
        loiter=self._mode==self.LOITER
        heading=~loiter
        # Radius is fixed when commanded except for this conservative IAS->TAS envelope check.
        feasible_radius=tas_cmd**2/(9.80665*np.tan(self.outer.turn.bank_limit))
        self._radius=np.maximum(np.maximum(self._requested_radius,self.min_radius),feasible_radius)
        radial=np.linalg.norm(obs.neu_pos[:,:2]-self._center[:,:2],axis=1)
        entering=loiter&(self._phase==self.APPROACH)&(radial<=2*self._radius)
        self._phase[entering]=self.CAPTURE
        approach=loiter&(self._phase==self.APPROACH)
        tangent=np.arctan2(obs.neu_pos[:,1]-self._center[:,1],obs.neu_pos[:,0]-self._center[:,0])+self._direction*np.pi/2
        tangent_error=_wrap_pi(tangent-np.arctan2(obs.neu_vel[:,1],obs.neu_vel[:,0]))
        close=loiter&(self._phase!=self.APPROACH)&(abs(radial-self._radius)<=.15*self._radius)&(abs(tangent_error)<np.deg2rad(30))
        self._phase[close]=self.HOLD
        self._phase[loiter&(self._phase==self.HOLD)&(abs(radial-self._radius)>.3*self._radius)]=self.CAPTURE
        chi=np.where(approach,np.arctan2(self._center[:,1]-obs.neu_pos[:,1],self._center[:,0]-obs.neu_pos[:,0]),self._chi)
        altitude=np.where(loiter,self._center[:,2],self._alt_ref)
        gamma=np.where(heading,self._gamma,0.)
        reference=ControlReference(tas_cmd,altitude,np.zeros((self.batch,2,3)),gamma_cmd=gamma)
        a_line,chi_air,_,_=self.outer.l1.heading_guidance(chi,reference,obs)
        a_circle,chi_circle,radial_error,_=self.outer.l1.circle_guidance(self._center[:,:2],self._radius,self._direction,reference,obs)
        on_circle=loiter&~approach
        acceleration=np.where(on_circle,a_circle,a_line)
        phi=self.outer.turn.step(acceleration,obs)
        theta,throttle=self.outer.tecs.step(reference,obs)
        theta=np.clip(theta+self.outer.tecs.bank_compensation(phi),-self.outer.tecs.theta_limit,self.outer.tecs.theta_limit)
        self._intermediates={
            'time':np.full(self.batch,now),'phi_cmd':phi.copy(),'theta_cmd':theta.copy(),
            'throttle_cmd':throttle.copy(),'chi_cmd':np.where(on_circle,chi_circle,chi_air),
            'chi_ground_cmd':_wrap_pi(np.where(on_circle,
                np.arctan2(obs.neu_pos[:,1]-self._center[:,1],obs.neu_pos[:,0]-self._center[:,0])+self._direction*np.pi/2,chi)),
            'gamma_cmd':gamma.copy(),'altitude_cmd':altitude.copy(),'ias_cmd':self._ias.copy(),
            'tas_cmd':tas_cmd.copy(),'mode':self._mode.copy(),'loiter_phase':self._phase.copy(),
            'radius_m':self._radius.copy(),'radial_m':radial,'cross_track':np.where(loiter,radial_error,np.nan),
        }
        # Advance an independent trajectory reference, never re-anchor to actual height.
        climb=np.linalg.norm(obs.neu_vel[:,:2],axis=1)*np.tan(gamma)
        self._alt_ref[heading]+=self.dt*climb[heading]
        self._last_time=now
        return AttitudeCommand(phi,theta,throttle)

    def intermediates(self):
        return copy.deepcopy(self._intermediates)

    def get_snapshot(self):
        """Outer/action state only. The generated C++ plant has no snapshot ABI."""
        return copy.deepcopy({'batch':self.batch,'outer':self.outer.get_snapshot(),
            **{k:v for k,v in self.__dict__.items() if k.startswith('_')}})

    def set_snapshot(self,snap):
        snap=copy.deepcopy(snap)
        if snap['batch']!=self.batch:raise ValueError('snapshot batch differs')
        self.outer.set_snapshot(snap.pop('outer'))
        snap.pop('batch')
        for key,value in snap.items():
            if not key.startswith('_'):raise ValueError('unknown snapshot field')
            setattr(self,key,value)
