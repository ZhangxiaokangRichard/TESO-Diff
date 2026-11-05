"""WP2A-endpoint 风场试验环境：W0 无风 / W1 热气流 / W2 垂直切变层（docs §16.2）。

统一接口（docs §13）：query(position_neu [B,3], time [B]) -> wind_flat [B,3]。
返回 flat-Earth 轴风速（Xe 北 / Ye 东 / Ze 向下为正），与 ABI 风输入一致；
NEU 位置（第 3 分量向上为正）在内部使用。

W2 为随高度变化的东向风切变层（动态滑翔所需的垂直梯度），水平面内均匀：
W_e(h) = ΔV/2 · (1 + tanh((h − h0)/L))，h0 = 100 m，L = 10 m，ΔV = 4 m/s。
这是 experiments 级的固定场地定义；可复用的参数化风场 API 留到 Phase 2b。
"""

from __future__ import annotations

import numpy as np

G = 9.80665


class ZeroWind:
    """W0：无风，物理零获能基线（docs §17 D0 / E0 检查）。"""

    wind_id = "W0"

    def __init__(self):
        self.params = dict(mechanism="none")

    def query(self, position_neu: np.ndarray, time) -> np.ndarray:
        position_neu = np.atleast_2d(position_neu)
        return np.zeros((position_neu.shape[0], 3))


class GaussianThermal:
    """W1：仅上升气流。水平高斯热气流柱，高度不变（合成热柱，docs §18 D2-W1）。

    w_up(p) = peak · exp(−r²/R²)，r 为到热柱中心的水平距离。无水平分量、无时间演化。
    """

    wind_id = "W1"

    def __init__(self, center_ne=(200.0, 0.0), radius_m=90.0, peak_up_mps=2.0):
        self.center = np.asarray(center_ne, dtype=np.float64)
        self.radius = float(radius_m)
        self.peak = float(peak_up_mps)
        self.params = dict(
            mechanism="gaussian_thermal_column",
            center_ne=list(self.center),
            radius_m=self.radius,
            peak_up_mps=self.peak,
            altitude_dependence="none (synthetic column)",
        )

    def up_mps(self, position_neu: np.ndarray) -> np.ndarray:
        position_neu = np.atleast_2d(position_neu)
        r2 = np.sum((position_neu[:, :2] - self.center) ** 2, axis=1)
        return self.peak * np.exp(-r2 / self.radius**2)

    def query(self, position_neu: np.ndarray, time) -> np.ndarray:
        position_neu = np.atleast_2d(position_neu)
        wind = np.zeros((position_neu.shape[0], 3))
        wind[:, 2] = -self.up_mps(position_neu)  # 上升气流 → Ze（向下为正）取负
        return wind


class VerticalShearLayer:
    """W2：随高度变化的东向风切变层（动态滑翔的垂直风梯度，docs §18 D2-W2）。

    W_e(h) = ΔV/2 · (1 + tanh((h − h0)/L))：低于 h0 近似无风，高于 h0 为 ΔV 东风，
    切变层厚度约 2L。水平面内均匀，静态（不随时间变化），无垂直分量。
    """

    wind_id = "W2"

    def __init__(self, layer_altitude_m=100.0, half_width_m=10.0, delta_v_mps=4.0):
        self.h0 = float(layer_altitude_m)
        self.width = float(half_width_m)
        self.dv = float(delta_v_mps)
        self.params = dict(
            mechanism="tanh_vertical_shear_layer",
            layer_altitude_m=self.h0,
            transition_half_width_m=self.width,
            delta_v_east_mps=self.dv,
            horizontal_uniformity="uniform in North/East",
            vertical_component="none",
        )

    def east_mps(self, position_neu: np.ndarray) -> np.ndarray:
        position_neu = np.atleast_2d(position_neu)
        return 0.5 * self.dv * (1.0 + np.tanh((position_neu[:, 2] - self.h0) / self.width))

    def query(self, position_neu: np.ndarray, time) -> np.ndarray:
        position_neu = np.atleast_2d(position_neu)
        wind = np.zeros((position_neu.shape[0], 3))
        wind[:, 1] = self.east_mps(position_neu)  # flat-Earth Ye = 东向
        return wind


WIND_IDS = ("W0", "W1", "W2")


def make_wind(wind_id: str):
    if wind_id == "W0":
        return ZeroWind()
    if wind_id == "W1":
        return GaussianThermal()
    if wind_id == "W2":
        return VerticalShearLayer()
    raise ValueError(f"unknown wind id: {wind_id}")


def wind_flat_to_neu(wind_flat: np.ndarray) -> np.ndarray:
    """flat-Earth (Xe,Ye,Ze down+) → NEU (N,E,Up+)。"""
    wind_neu = np.array(wind_flat, dtype=np.float64, copy=True)
    wind_neu[..., 2] *= -1.0
    return wind_neu
