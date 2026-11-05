"""python 包级测试（Phase 2a，接口层单测）。

覆盖：controller 数据结构、PID 张量解算、串级内环符号/限幅、
NPFG 圆检测与风三角、wind stub 坐标符号约定、空间 8 字路径几何。
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

# 与解释器无关的项目根路径引导（不依赖 CWD/PYTHONPATH）
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from python.controller.pid import PID
from python.controller.controllerAPI import PolylinePathFollower
from python.controller.inner import InnerLoop
from python.controller.l1 import L1Controller
from python.controller.types import (
    ActuatorCommand,
    ControlObservation,
    ControlReference,
)
from python.simulation.wind_stub import WindStub


def _reference(batch: int = 4) -> ControlReference:
    return ControlReference(
        airspeed_cmd=np.full(batch, 16.0),
        altitude_cmd=np.full(batch, 1000.0),
        path=np.zeros((batch, 8, 3)),
    )


def _observation(batch: int = 4) -> ControlObservation:
    zero = np.zeros(batch)
    return ControlObservation(
        u=np.full(batch, 14.1), v=zero.copy(), w=zero.copy(),
        p=zero.copy(), q=zero.copy(), r=zero.copy(),
        phi=zero.copy(), theta=zero.copy(), psi=zero.copy(),
        alpha=zero.copy(), beta=zero.copy(),
        neu_pos=np.zeros((batch, 3)), neu_vel=np.zeros((batch, 3)),
        tas=np.full(batch, 16.0), wind_neu=np.zeros((batch, 2)),
    )


def test_pid_tensorized() -> None:
    pid = PID(kp=1.0, ki=0.1, kd=0.05, dt=0.02,
              output_limits=(-1.0, 1.0), integral_limits=(-5.0, 5.0))
    error = np.array([0.1, 0.2, -0.3, 0.0])
    outputs = [pid.step(error) for _ in range(10)]
    assert all(out.shape == (4,) for out in outputs)
    assert np.all(np.abs(outputs[-1]) <= 1.0)


def test_pid_snapshot_roundtrip() -> None:
    pid = PID(kp=1.0, ki=0.2, kd=0.05, dt=0.02)
    error = np.array([0.1, 0.2])
    pid.step(error)
    pid.step(-error)
    snap = pid.get_snapshot()
    before = pid.step(error).copy()
    pid.set_snapshot(snap)
    after = pid.step(error)
    assert np.allclose(before, after)


def test_actuator_limits() -> None:
    batch = 3
    ActuatorCommand(
        delta_a=np.zeros(batch), delta_e=np.zeros(batch),
        delta_r=np.zeros(batch), throttle=np.full(batch, 0.5),
    )
    try:
        ActuatorCommand(
            delta_a=np.full(batch, 1.5), delta_e=np.zeros(batch),
            delta_r=np.zeros(batch), throttle=np.full(batch, 0.5),
        )
    except ValueError:
        pass
    else:
        raise AssertionError("delta_a beyond [-1,1] must raise")


def test_wind_stub_sign_convention() -> None:
    """零风/恒定风：NEU 输入 → flat-Earth 输出，Z 分量符号相反。"""
    wind = WindStub(uniform_flat_earth=(3.0, -2.0, 1.0))  # Ze 向下 +1
    out = wind.query(np.zeros((2, 3)), np.zeros(2))
    assert np.allclose(out[:, 0], 3.0)
    assert np.allclose(out[:, 1], -2.0)
    assert np.allclose(out[:, 2], -1.0)  # Up-positive 输入 → 取反为 Down


def test_observation_from_fw_outputs_signature() -> None:
    """ControlObservation.from_fw_outputs 接口存在且可导入。"""
    assert callable(ControlObservation.from_fw_outputs)


def test_inner_cascade_sign_convention() -> None:
    """串级内环符号（复测结论）：
    phi 低于期望 → 需要右滚 → delta_a 为负（delta_a 正 → 左滚）；
    theta 低于期望 → 需要抬头 → delta_e 为负（delta_e 正 → 低头）；
    psi 低于期望 → 需要右偏 → delta_r 为正（delta_r 正 → 右偏航）。
    """
    cfg = {
        "dt": 0.02,
        "angle_roll_kp": 3.0, "angle_pitch_kp": 3.0, "angle_yaw_kp": 2.0,
        "p_limit_degps": 60.0, "q_limit_degps": 45.0, "r_limit_degps": 30.0,
        "roll": {"rate_kp": 0.1, "rate_ff": 0.0},
        "pitch": {"rate_kp": 0.1, "rate_ki": 0.0, "rate_ff": 0.0},
        "yaw": {"rate_kp": 0.1, "rate_ki": 0.0, "rate_ff": 0.0},
        "coordinated_turn_ff_gain": 1.0,
        "v_trim_mps": 16.0, "beta_gain": 0.5,
    }
    inner = InnerLoop(cfg)
    inner.reset(1)
    obs = _observation(1)
    command = inner.step(
        np.array([0.2]), np.array([0.1]), np.array([0.3]), obs
    )
    # obs 姿态全 0：phi_err>0（右滚需求）→ delta_a < 0
    assert command.delta_a[0] < 0, "delta_a sign wrong"
    # theta_err>0（抬头需求）→ delta_e < 0（delta_e 正 → 低头）
    assert command.delta_e[0] < 0, "delta_e sign wrong"
    # psi_err>0（右偏需求）→ delta_r > 0
    assert command.delta_r[0] > 0, "delta_r sign wrong"


def test_inner_limits() -> None:
    """大误差下舵面不超限、角速率指令被限幅。"""
    cfg = {
        "dt": 0.02,
        "angle_roll_kp": 10.0, "angle_pitch_kp": 10.0, "angle_yaw_kp": 6.0,
        "p_limit_degps": 60.0, "q_limit_degps": 45.0, "r_limit_degps": 30.0,
        "roll": {"rate_kp": 1.5, "rate_ff": 1.0},
        "pitch": {"rate_kp": 1.5, "rate_ki": 0.0, "rate_ff": 1.0},
        "yaw": {"rate_kp": 1.5, "rate_ki": 0.0, "rate_ff": 1.0},
        "coordinated_turn_ff_gain": 1.0,
        "v_trim_mps": 16.0, "beta_gain": 0.5,
    }
    inner = InnerLoop(cfg)
    inner.reset(1)
    obs = _observation(1)
    for _ in range(5):
        command = inner.step(
            np.array([0.5]), np.array([-0.4]), np.array([0.3]), obs
        )
    assert np.all(np.abs(command.delta_a) <= 1.0)
    assert np.all(np.abs(command.delta_e) <= 1.0)
    assert np.all(np.abs(command.delta_r) <= 1.0)


def test_l1_circle_detection_and_guidance() -> None:
    """圆形路径 → loiter 制导：切向参考方向、向心加速度符号正确。"""
    from python.simulation.task_route import circle_path

    path = circle_path((0.0, 60.0), 60.0, 1000.0, n_pts=721)
    l1 = L1Controller({"npfg_enabled": True, "period_s": 12.0, "damping": 0.75,
                       "circle_kr": 0.3, "circle_kv": 0.5})
    l1.reset(1)
    ref = ControlReference(
        airspeed_cmd=np.array([16.0]), altitude_cmd=np.array([1000.0]),
        path=path[None, :, :],
    )
    obs = _observation(1)
    obs.neu_pos = np.array([[0.0, 0.0, 1000.0]])
    obs.neu_vel = np.array([[16.0, 0.0, 0.0]])
    a_cmd, chi_air, cross_track, v_cmd = l1.step(ref, obs)
    # 位于圆上、切向飞行 → 向心加速度 = V²/R > 0（逆时针 → 右转为正）
    assert abs(a_cmd[0] - 16.0 ** 2 / 60.0) < 0.5
    assert abs(cross_track[0]) < 1.0
    # 无风：期望空速航向 = 切向（北）
    assert abs(chi_air[0]) < 0.05


def test_l1_npfg_wind_triangle() -> None:
    """有风时 NPFG 风三角：期望空速矢量仍指向参考方向。"""
    from python.simulation.task_route import circle_path

    path = circle_path((0.0, 60.0), 60.0, 1000.0, n_pts=721)
    l1 = L1Controller({"npfg_enabled": True})
    l1.reset(1)
    ref = ControlReference(
        airspeed_cmd=np.array([16.0]), altitude_cmd=np.array([1000.0]),
        path=path[None, :, :],
    )
    obs = _observation(1)
    obs.neu_pos = np.array([[0.0, 0.0, 1000.0]])
    obs.neu_vel = np.array([[16.0, 0.0, 0.0]])
    obs.wind_neu = np.array([[0.0, 5.0]])  # 西风 5 m/s（East +5）
    a_cmd, chi_air, cross_track, v_cmd = l1.step(ref, obs)
    # 期望空速矢量 + 风 应平行于北向参考方向
    air_vec = np.array([16.0 * np.cos(chi_air[0]), 16.0 * np.sin(chi_air[0])])
    ground_dir = air_vec + obs.wind_neu[0]
    assert np.abs(np.arctan2(ground_dir[1], ground_dir[0])) < 0.05


def test_figure8_spatial_path_geometry() -> None:
    """空间 8 字路径：高度 ±10（左圆高、右圆低）、无尖点、闭合。"""
    from python.simulation.task_route import figure8_spatial_path

    f8 = figure8_spatial_path((0.0, 0.0), 80.0, 1000.0, n_pts=1441,
                              center_dist=400.0, center_dh=10.0)
    # 高度范围：H-10 ~ H+10（采样略过端点，容差放宽）
    assert abs(f8[:, 2].max() - 1010.0) < 0.1
    assert abs(f8[:, 2].min() - 990.0) < 0.1
    # 水平范围：圆心 ±(200+80)
    assert abs(f8[:, 0].max() - 280.0) < 0.5
    assert abs(f8[:, 0].min() + 280.0) < 0.5
    # 闭合：首尾接近（起点 A_upper 结束点 A_upper）
    assert np.linalg.norm(f8[0] - f8[-1]) < 1.5
    # 无尖点：相邻段方向变化 < 10°
    d = np.diff(f8, axis=0)
    seg = np.linalg.norm(d, axis=1)
    dirs = d / seg[:, None]
    cosang = np.sum(dirs[1:] * dirs[:-1], axis=1)
    assert cosang.min() > np.cos(np.deg2rad(10.0))


def test_polyline_path_follower_lookahead_and_rate_limit() -> None:
    """路径适配器使用前视点，且 Heading 指令按配置限速。"""
    north = np.linspace(0.0, 200.0, 41)
    path = np.stack([north, np.zeros_like(north), np.full_like(north, 1000.0)], axis=1)
    follower = PolylinePathFollower(
        path, airspeed=16.0, closed=False,
        lookahead_time_s=2.5, heading_rate_limit_degps=18.0,
        update_period_s=0.1,
    )
    follower.reset(initial_segment=0, initial_chi=0.0)
    command = follower.update(
        np.array([0.0, 20.0, 1000.0]), timestamp=0.0, ground_speed=16.0
    )
    status = follower.status()
    # 飞机位于路径左侧（East +），指令应向右修正；首步不超过 1.8 deg。
    assert command.chi < 0.0
    assert abs(command.chi) <= np.deg2rad(1.8) + 1e-9
    assert 39.9 <= status.lookahead_m <= 40.1
    assert status.signed_cross_track_m > 19.0

    held = follower.update(
        np.array([0.2, 19.8, 1000.0]), timestamp=0.02, ground_speed=16.0
    )
    assert held == command
    assert not follower.status().updated


def test_polyline_path_follower_crossing_branch_and_progress() -> None:
    """8 字交叉点保持起始分支，弧长进度不跳支、不倒退。"""
    cx, cy, radius = 200.0, 0.0, 80.0
    theta_l = np.linspace(0.0, 2.0 * np.pi, 100, endpoint=False)
    theta_r = np.linspace(np.pi, -np.pi, 100, endpoint=False)
    path = np.stack([
        np.concatenate([(cx - radius) + radius * np.cos(theta_l),
                        (cx + radius) + radius * np.cos(theta_r)]),
        np.concatenate([cy + radius * np.sin(theta_l),
                        cy + radius * np.sin(theta_r)]),
        np.full(200, 1000.0),
    ], axis=1)
    follower = PolylinePathFollower(path, airspeed=16.0, closed=True)
    follower.reset(initial_segment=0, initial_chi=0.0)

    follower.update(np.array([200.0, 0.0, 1000.0]), 0.0, 16.0)
    first = follower.status().progress_m
    follower.update(np.array([199.5, 2.0, 1000.0]), 0.1, 16.0)
    second = follower.status().progress_m

    assert second >= first
    # 两个分支都通过该点，但单次进度不得跳到约半圈后的另一分支。
    assert second - first < 12.0
    assert follower.status().segment_index < 5


def test_polyline_path_follower_snapshot_roundtrip() -> None:
    north = np.linspace(0.0, 100.0, 21)
    path = np.stack([north, np.zeros_like(north), np.full_like(north, 1000.0)], axis=1)
    follower = PolylinePathFollower(path, airspeed=16.0, closed=False)
    follower.reset(initial_chi=0.0)
    follower.update(np.array([5.0, 3.0, 1000.0]), 0.0, 16.0)
    snapshot = follower.get_snapshot()
    expected = follower.update(np.array([7.0, 2.0, 1000.0]), 0.1, 16.0)
    expected_status = follower.status()
    follower.set_snapshot(snapshot)
    actual = follower.update(np.array([7.0, 2.0, 1000.0]), 0.1, 16.0)
    actual_status = follower.status()
    assert actual == expected
    assert actual_status == expected_status


def test_polyline_path_follower_lap_boundary_no_deadlock() -> None:
    """回归（wp2a_endpoint 发现）：progress 恰为整圈倍数时不得死锁。

    旧实现中 `% total` 与 `floor(progress/total)` 在整圈边界的浮点舍入
    可各差一圈，组合出超前一圈的 unwrapped 索引，候选窗口恒为空，
    progress 永久冻结、heading 指令固定（盘旋 7 圈后直飞逃逸）。
    """
    theta = np.linspace(0.0, 2.0 * np.pi, 240, endpoint=False)
    path = np.stack([200.0 + 80.0 * np.cos(theta), 80.0 * np.sin(theta),
                     np.full(240, 100.0)], axis=1)
    follower = PolylinePathFollower(path, airspeed=16.0, closed=True,
                                    lookahead_min_m=35.0, lookahead_max_m=55.0)
    for laps in (1, 7, 13):
        follower.reset(initial_chi=0.0)
        boundary = laps * follower._total_len  # 浮点整圈倍数（易触发舍入不一致）
        snap = follower.get_snapshot()
        snap["progress_m"] = boundary
        snap["last_timestamp"] = 0.0
        snap["last_chi"] = np.pi / 2.0
        follower.set_snapshot(snap)
        progress = []
        for k in range(1, 11):
            arc = 0.3 + 1.6 * k  # 飞机沿圆弧继续前进（16 m/s × 0.1 s 更新）
            angle = arc / 80.0
            pos = np.array([200.0 + 80.5 * np.cos(angle),
                            80.5 * np.sin(angle), 100.0])
            follower.update(pos, 0.1 * k, 16.0)
            progress.append(follower.status().progress_m)
        assert progress[-1] > boundary + 5.0, (
            f"progress deadlocked at lap boundary x{laps}: {progress}")
        assert all(b >= a for a, b in zip(progress, progress[1:]))


def test_polyline_path_follower_altitude_follow() -> None:
    """三维航迹：altitude_follow 输出路径斜率对应的倾角，超限时限幅。"""
    north = np.linspace(0.0, 2000.0, 201)
    path = np.stack([north, np.zeros_like(north), 100.0 + 0.1 * north], axis=1)
    follower = PolylinePathFollower(path, airspeed=16.0, closed=False,
                                    altitude_follow=True)
    follower.reset(initial_chi=0.0)
    command = follower.update(np.array([5.0, 0.0, 100.5]), 0.0, 16.0)
    assert abs(command.gamma - np.arctan(0.1)) < 1e-6

    steep = np.stack([north, np.zeros_like(north), 100.0 + 1.0 * north], axis=1)
    follower = PolylinePathFollower(steep, airspeed=16.0, closed=False,
                                    altitude_follow=True)
    follower.reset(initial_chi=0.0)
    command = follower.update(np.array([5.0, 0.0, 105.0]), 0.0, 16.0)
    assert abs(command.gamma - np.deg2rad(15.0)) < 1e-9

    flat = PolylinePathFollower(path, airspeed=16.0, closed=False, gamma=0.02)
    flat.reset(initial_chi=0.0)
    assert flat.update(np.array([5.0, 0.0, 100.5]), 0.0, 16.0).gamma == 0.02


if __name__ == "__main__":
    test_pid_tensorized()
    test_pid_snapshot_roundtrip()
    test_actuator_limits()
    test_wind_stub_sign_convention()
    test_observation_from_fw_outputs_signature()
    test_inner_cascade_sign_convention()
    test_inner_limits()
    test_l1_circle_detection_and_guidance()
    test_l1_npfg_wind_triangle()
    test_figure8_spatial_path_geometry()
    test_polyline_path_follower_lookahead_and_rate_limit()
    test_polyline_path_follower_crossing_branch_and_progress()
    test_polyline_path_follower_snapshot_roundtrip()
    test_polyline_path_follower_lap_boundary_no_deadlock()
    test_polyline_path_follower_altitude_follow()
    print("python package unit tests passed")
