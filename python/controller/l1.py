"""L1 + NPFG 侧向制导（外环，WP2A-8 v2 改造）。

参考《PX4固定翼控制器详解(五)——L1、NPFG控制器》：
L1 制导律（Park 2004，PX4 实现口径）：
    L1_dist  = (1/π) · damping · period · GS      （参考距离随地速缩放）
    a_cmd    = 4·damping² · GS² / L1_dist · sin η （横向加速度指令）
    K_L1 = 4·damping²，调参先 period 后 damping（0.65~0.85）

NPFG 改进（Stastny & Siegwart 2019，PX4 1.13+）：引入期望空速矢量抵御强风：
    - 由参考方向得期望地面航迹方向 t̂；
    - 风三角求解：a⃗ + w⃗ ∥ t̂ 且 |a⃗| = V_sp
      → a⃗ = s·t̂ - w⃗，s = (w⃗·t̂) + sqrt(V_sp² - |w⃗×t̂|²)；
    - |w⃗×t̂| > V_sp（空速圆外、不可达方向）：退化为对准 t̂ 的最接近解
      a⃗ = V_sp·t̂，避免 L1 用地速导致的失控。

盘旋（circle）任务自动检测并用 PX4 Circle（loiter）模式（文章第五期）：
    a_cmd = direction·(GS²/R + K_r·(radial - R) + K_v·v_r)
    （向心加速度前馈 + 径向误差 PD，v_r = 地速在径向的投影）
    line_angle = 切向方向（供 NPFG 风三角），避免闭路参考点搜索歧义。

注：早期试验的 loiter 切圆 setpoint 导航（figure8/racetrack 用）已在小尺度
轨迹上验证不适用，已删除（见 docs/build_logs/phase2_sp2.md 附录 B）。
figure8/racetrack 统一使用 NPFG 采样路径制导。

期望路径为闭合航迹（盘旋/8 字/跑马场）。
全张量化，无逐实例 Python 循环；状态支持 snapshot（counterfactual）。
"""

from __future__ import annotations

from typing import Optional

import numpy as np

from .inner import wrap_pi
from .types import ControlObservation, ControlReference


class L1Controller:
    """L1 + NPFG 侧向制导（全张量化）。"""

    def __init__(self, cfg: dict):
        # L1（PX4 参数化：period/damping，而非固定 L1 距离）
        self.period = float(cfg.get("period_s", 12.0))
        self.damping = float(cfg.get("damping", 0.75))
        self.l1_dist_min = float(cfg.get("l1_dist_min_m", 20.0))
        self.l1_dist_max = float(cfg.get("l1_dist_max_m", 300.0))
        self.a_cmd_limit = float(cfg.get("a_cmd_limit_mps2", 15.0))
        self.npfg_enabled = bool(cfg.get("npfg_enabled", True))
        # PX4 Circle（loiter）模式径向 PD 参数
        self.circle_kr = float(cfg.get("circle_kr", 0.3))
        self.circle_kv = float(cfg.get("circle_kv", 0.5))
        # 路径参考点索引（NPFG 采样制导状态）
        self._ref_index: Optional[np.ndarray] = None

    def reset(self, batch: int) -> None:
        self._ref_index = np.zeros(batch, dtype=np.int64)

    def _circle_params(self, path: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """检测圆形路径，返回 (centroid [B,2], radius [B], winding [B], is_circle [B])。"""
        centroid = path[:, :, :2].mean(axis=1)
        radii = np.linalg.norm(path[:, :, :2] - centroid[:, None, :], axis=2)
        mean_r = radii.mean(axis=1)
        std_r = radii.std(axis=1)
        is_circle = std_r < 1e-3 * mean_r
        d1 = path[:, 1, :2] - path[:, 0, :2]
        d2 = path[:, 2, :2] - path[:, 1, :2]
        winding = np.sign(d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0])  # +1 逆时针
        winding = np.where(winding == 0, 1.0, winding)
        return centroid, mean_r, winding, is_circle

    def _circle_guidance(
        self, ref: ControlReference, obs: ControlObservation,
        centroid: np.ndarray, radius: np.ndarray, winding: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """PX4 Circle（loiter）模式：径向 PD + 向心前馈。

        返回 (a_cmd, line_angle, cross_track, v_cmd)。
        """
        pos = obs.neu_pos[:, :2]
        vel = obs.neu_vel[:, :2]
        rel = pos - centroid
        radial = np.linalg.norm(rel, axis=1)
        radial_safe = np.maximum(radial, 1e-3)
        n_hat = rel / radial_safe[:, None]                      # 外向单位矢量
        tangent = winding[:, None] * np.stack([-n_hat[:, 1], n_hat[:, 0]], axis=1)  # 航向切线
        v_r = np.sum(vel * n_hat, axis=1)                        # 径向速度（外向 +）
        gs = np.linalg.norm(vel, axis=1)

        cross_track = radial - radius
        a_cmd = winding * (
            gs ** 2 / np.maximum(radius, 1.0)
            + self.circle_kr * cross_track
            + self.circle_kv * v_r
        )
        a_cmd = np.clip(a_cmd, -self.a_cmd_limit, self.a_cmd_limit)
        line_angle = np.arctan2(tangent[:, 1], tangent[:, 0])
        v_cmd = np.asarray(ref.airspeed_cmd, dtype=np.float64).copy()
        return a_cmd, line_angle, cross_track, v_cmd

    def step(
        self, ref: ControlReference, obs: ControlObservation
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """返回 (a_cmd [B], chi_air [B] rad, cross_track [B] m, v_cmd [B] m/s)。

        chi_air 为期望空速矢量方向（NPFG 风三角），直接作为 ψ_cmd。
        """
        path = ref.path  # [B, N, 3]，闭合航迹（NEU）
        batch, n_pts, _ = path.shape
        pos = obs.neu_pos[:, :2]  # [B,2] 水平位置
        vel = obs.neu_vel[:, :2]  # [B,2] 水平地速

        # 圆形任务：PX4 Circle（loiter）模式（闭路参考点搜索对圆有歧义）
        centroid, radius, winding, is_circle = self._circle_params(path)
        if is_circle.all():
            a_cmd, line_angle, cross_track, v_cmd = self._circle_guidance(
                ref, obs, centroid, radius, winding
            )
            # NPFG 风三角作用于切向参考方向
            return self._npfg_wind_triangle(
                a_cmd, line_angle, cross_track, v_cmd, ref, obs
            )

        if self._ref_index is None or self._ref_index.shape[0] != batch:
            self.reset(batch)
        assert self._ref_index is not None

        # 从上一参考点索引起滚动重排路径 [B, N, 3]
        order = (self._ref_index[:, None] + np.arange(n_pts)[None, :]) % n_pts
        rolled = path[np.arange(batch)[:, None], order, :]

        dist = np.linalg.norm(rolled[:, :, :2] - pos[:, None, :], axis=2)
        cross_track = np.min(dist, axis=1)

        # 参考点：距离 >= L1 的候选中，选与当前航向夹角最小的点
        # （航向一致前瞻，避免闭合路径上选中飞机身后/相邻环上的点）
        gs = np.linalg.norm(vel, axis=1)
        l1 = np.clip(
            (1.0 / np.pi) * self.damping * self.period * np.maximum(gs, 1e-3),
            self.l1_dist_min, self.l1_dist_max,
        )
        cond = dist >= l1[:, None]
        found = cond.any(axis=1)
        chi_gs = np.arctan2(vel[:, 1], vel[:, 0])
        ang = np.arctan2(rolled[:, :, 1] - pos[:, None, 1],
                         rolled[:, :, 0] - pos[:, None, 0])
        dev = np.abs(wrap_pi(ang - chi_gs[:, None]))
        masked_dev = np.where(cond, dev, np.inf)
        j = np.where(
            found,
            np.argmin(masked_dev, axis=1),
            np.argmax(dist, axis=1),  # 无候选（路径太小）：取最远点
        )
        self._ref_index = (self._ref_index + j + 1) % n_pts  # 推进，防止回退

        ref_pt = rolled[np.arange(batch), j, :2]  # [B,2]

        # L1 几何：期望地面航迹方向 t̂（参考线方向）
        line_angle = np.arctan2(ref_pt[:, 1] - pos[:, 1],
                                ref_pt[:, 0] - pos[:, 0])
        eta = wrap_pi(line_angle - chi_gs)
        a_cmd = 4.0 * self.damping ** 2 * gs ** 2 / l1 * np.sin(eta)
        a_cmd = np.clip(a_cmd, -self.a_cmd_limit, self.a_cmd_limit)
        v_cmd = np.asarray(ref.airspeed_cmd, dtype=np.float64).copy()

        return self._npfg_wind_triangle(
            a_cmd, line_angle, cross_track, v_cmd, ref, obs
        )

    def _npfg_wind_triangle(
        self,
        a_cmd: np.ndarray,
        line_angle: np.ndarray,
        cross_track: np.ndarray,
        v_cmd: np.ndarray,
        ref: ControlReference,
        obs: ControlObservation,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """NPFG 风三角：期望空速矢量 a⃗ = s·t̂ - w⃗，输出期望空速航向 chi_air。"""
        batch = a_cmd.shape[0]
        t_hat = np.stack([np.cos(line_angle), np.sin(line_angle)], axis=1)  # [B,2]
        v_sp = np.maximum(np.asarray(ref.airspeed_cmd, dtype=np.float64), 1e-3)
        if obs.wind_neu is not None and self.npfg_enabled:
            wind = np.asarray(obs.wind_neu, dtype=np.float64)  # [B,2]
        else:
            wind = np.zeros((batch, 2))

        w_par = np.sum(wind * t_hat, axis=1)          # w⃗·t̂
        w_perp = wind[:, 0] * t_hat[:, 1] - wind[:, 1] * t_hat[:, 0]  # w⃗×t̂ (z)
        disc = v_sp ** 2 - w_perp ** 2
        feasible = disc >= 0.0
        s = np.where(
            feasible,
            w_par + np.sqrt(np.maximum(disc, 0.0)),
            0.0,  # 不可达方向：退化为 a⃗ = V_sp·t̂
        )
        air_vec = np.where(
            feasible[:, None],
            s[:, None] * t_hat - wind,   # 空速圆内：严格风三角
            v_sp[:, None] * t_hat,       # 空速圆外：对准 t̂ 的最接近解
        )
        chi_air = np.arctan2(air_vec[:, 1], air_vec[:, 0])
        v_cmd_out = np.where(feasible, v_sp, np.linalg.norm(air_vec, axis=1))
        return a_cmd, chi_air, cross_track, v_cmd_out

    def get_snapshot(self) -> dict:
        return {"ref_index": None if self._ref_index is None
                else self._ref_index.copy()}

    def set_snapshot(self, snap: dict) -> None:
        self._ref_index = (
            None if snap["ref_index"] is None
            else np.array(snap["ref_index"], dtype=np.int64)
        )
