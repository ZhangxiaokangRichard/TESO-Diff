"""任务航线生成（Phase 2a）：封闭航迹期望空间位置序列（NEU）。

S1 盘旋 / S2 8 字 / S3 跑马场 / S4 GPS 三角。
路径为闭合序列：path[-1] 与 path[0] 相同（供 L1 循环搜索与绘图）。
"""

from __future__ import annotations

import numpy as np


def circle_path(
    center_ne: tuple[float, float],
    radius: float,
    altitude: float,
    n_pts: int = 721,
    climb_rate: float = 0.0,
    direction: int = 1,
    airspeed: float = 16.0,
) -> np.ndarray:
    """盘旋航迹 [N,3]（NEU）。climb_rate 为正盘旋上升。"""
    theta = np.linspace(0.0, 2.0 * np.pi, n_pts)
    theta = theta * direction  # direction=1 逆时针（从上往下看）
    lap_time = 2.0 * np.pi * radius / max(airspeed, 1e-3)
    t = theta / (2.0 * np.pi) * lap_time
    north = center_ne[0] + radius * np.cos(theta)
    east = center_ne[1] + radius * np.sin(theta)
    alt = altitude + climb_rate * t
    return np.stack([north, east, alt], axis=1)


def figure8_path(
    center_ne: tuple[float, float],
    radius: float,
    altitude: float,
    n_pts: int = 1441,
    climb_rate: float = 0.0,
    airspeed: float = 16.0,
) -> np.ndarray:
    """8 字航迹 [N,3]（两个相切圆，lemniscate 参数化）。"""
    theta = np.linspace(0.0, 2.0 * np.pi, n_pts)
    scale = radius / (1.0 + np.sin(theta) ** 2)
    north = center_ne[0] + scale * np.cos(theta)
    east = center_ne[1] + scale * np.sin(theta) * np.cos(theta)
    total = 2.0 * np.pi * radius
    lap_time = total / max(airspeed, 1e-3)
    t = theta / (2.0 * np.pi) * lap_time
    alt = altitude + climb_rate * t
    return np.stack([north, east, alt], axis=1)


def racetrack_path(
    center_ne: tuple[float, float],
    straight: float,
    radius: float,
    altitude: float,
    n_pts: int = 1441,
    climb_rate: float = 0.0,
    airspeed: float = 16.0,
) -> np.ndarray:
    """跑马场航迹 [N,3]：两直线 + 两个半圆，直道长轴沿北向。

    顺序（顺时针）：北半圆（东行）→ 东直线（南行）→ 南半圆（西行）→ 西直线（北行）。
    """
    cx, cy = center_ne
    half = straight / 2.0
    arc = np.pi * radius  # 单个半圆弧长
    perimeter = 2.0 * straight + 2.0 * arc
    s = np.linspace(0.0, perimeter, n_pts)

    # 分段定义：(起点弧长, 终点弧长, 参数化函数)
    segments = [
        # 北半圆：中心 (cx, +half)，从西端 (cx-R, +half) 逆时针到东端 (cx+R, +half)
        (0.0, arc,
         lambda u: (cx + radius * np.cos(np.pi + u / radius),
                    cy + half + radius * np.sin(np.pi + u / radius))),
        # 东直线：从 (cx+R, +half) 向南到 (cx+R, -half)
        (arc, arc + straight,
         lambda u: (cx + radius, cy + half - 2.0 * half * (u - arc) / straight)),
        # 南半圆：中心 (cx, -half)，从东端逆时针到西端（途经南侧最远点）
        (arc + straight, 2.0 * arc + straight,
         lambda u: (cx + radius * np.cos((u - arc - straight) / radius),
                    cy - half + radius * np.sin((u - arc - straight) / radius))),
        # 西直线：从 (cx-R, -half) 向北到 (cx-R, +half)
        (2.0 * arc + straight, perimeter,
         lambda u: (cx - radius,
                    cy - half + 2.0 * half * (u - 2.0 * arc - straight) / straight)),
    ]

    def point_at(distance: float) -> np.ndarray:
        d = distance % perimeter
        for start, end, fn in segments:
            if d <= end or end >= perimeter:
                return np.array(fn(min(d, end)))
        return np.array(fn(perimeter))

    pts = np.array([point_at(d) for d in s])
    lap_time = perimeter / max(airspeed, 1e-3)
    t = s / perimeter * lap_time
    alt = altitude + climb_rate * t
    return np.stack([pts[:, 0], pts[:, 1], alt], axis=1)


def build_task_path(cfg: dict, n_pts: int = 721) -> np.ndarray:
    """按任务配置生成闭合路径 [N,3]。"""
    task_type = cfg.get("type", "circle")
    center = tuple(cfg.get("center_ne", (0.0, 0.0)))
    altitude = float(cfg.get("altitude_m", 1000.0))
    climb = float(cfg.get("climb_rate_mps", 0.0))
    airspeed = float(cfg.get("airspeed_mps", 16.0))
    if task_type == "circle":
        return circle_path(
            center, float(cfg.get("radius_m", 60.0)), altitude,
            n_pts=n_pts, climb_rate=climb,
            direction=int(cfg.get("direction", 1)), airspeed=airspeed,
        )
    if task_type == "figure8":
        return figure8_path(center, float(cfg.get("radius_m", 60.0)), altitude,
                            n_pts=n_pts, climb_rate=climb, airspeed=airspeed)
    if task_type == "racetrack":
        return racetrack_path(
            center, float(cfg.get("straight_m", 200.0)),
            float(cfg.get("radius_m", 60.0)), altitude,
            n_pts=n_pts, climb_rate=climb, airspeed=airspeed,
        )
    raise ValueError(f"unsupported task type: {task_type}")
