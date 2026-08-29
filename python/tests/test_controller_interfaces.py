"""python 包级测试（Phase 2a，接口层单测）。

覆盖：controller 数据结构、PID 张量解算、wind stub 坐标符号约定。
"""

from __future__ import annotations

import numpy as np

from python.controller.pid import PID
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


if __name__ == "__main__":
    test_pid_tensorized()
    test_pid_snapshot_roundtrip()
    test_actuator_limits()
    test_wind_stub_sign_convention()
    test_observation_from_fw_outputs_signature()
    print("python package unit tests passed")
