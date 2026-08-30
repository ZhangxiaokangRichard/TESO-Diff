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

    # 分段定义：(起点弧长, 终点弧长, 参数化函数)——长轴沿北向
    segments = [
        # 北半圆：中心 (cx+half, cy)，从西端 (cx+half, cy-R) 经北侧最远点
        # (cx+half+R, cy) 到东端 (cx+half, cy+R)：圆弧外凸向上（东行）
        (0.0, arc,
         lambda u: (cx + half + radius * np.sin(np.pi - u / radius),
                    cy + radius * np.cos(np.pi - u / radius))),
        # 东直线：从 (cx+half, cy+R) 向南到 (cx-half, cy+R)
        (arc, arc + straight,
         lambda u: (cx + half - 2.0 * half * (u - arc) / straight,
                    cy + radius)),
        # 南半圆：中心 (cx-half, cy)，从东端 (cx-half, cy+R) 经南侧最远点
        # (cx-half-R, cy) 到西端 (cx-half, cy-R)：圆弧外凸向下（西行）
        (arc + straight, 2.0 * arc + straight,
         lambda u: (cx - half + radius * np.sin(-(u - arc - straight) / radius),
                    cy + radius * np.cos(-(u - arc - straight) / radius))),
        # 西直线：从 (cx-half, cy-R) 向北到 (cx+half, cy-R)
        (2.0 * arc + straight, perimeter,
         lambda u: (cx - half + 2.0 * half * (u - 2.0 * arc - straight) / straight,
                    cy - radius)),
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


def figure8_tangent_path(
    center_ne: tuple[float, float],
    radius: float,
    altitude: float,
    n_pts: int = 1441,
    center_dist: float | None = None,
    climb_rate: float = 0.0,
    airspeed: float = 16.0,
) -> np.ndarray:
    """双圆切线相接 8 字航迹 [N,3]（两个圆 + 上切线连接段，往返成闭合 8 字）。

    - 圆 A：中心 (cx - D/2, cy)，顺时针整圈，起点/终点在其正上方切点
      （顺时针在切点处速度朝北，与上切线段连续）；
    - 上切线段：从 A 正上方到 B 正上方（长度 D，两圆等半径时外公切线
      平行于圆心连线）；
    - 圆 B：中心 (cx + D/2, cy)，顺时针整圈；
    - 上切线段返回 A，闭合（每圈仅 B 出口一次掉头）。

    center_dist=D 默认 2R（两圆相切于中点）。R 与 D 均放大后，采样偏差
    对航迹跟踪的影响显著减小（对应 WP2A-8 品质试验问题整改）。
    """
    cx, cy = center_ne
    dist = 2.0 * radius if center_dist is None else float(center_dist)
    circ = 2.0 * np.pi * radius
    perimeter = 2.0 * circ + 2.0 * dist
    s = np.linspace(0.0, perimeter, n_pts)

    ax, bx = cx - dist / 2.0, cx + dist / 2.0

    def point_at(d: float) -> np.ndarray:
        d = d % perimeter
        if d < circ:  # 圆 A 顺时针（起点正上方，θ=π/2 递减）
            theta = np.pi / 2.0 - d / radius
            return np.array([ax + radius * np.cos(theta),
                             cy + radius * np.sin(theta)])
        if d < circ + dist:  # 上切线 A→B
            f = (d - circ) / dist
            return np.array([ax + (bx - ax) * f, cy + radius])
        if d < 2.0 * circ + dist:  # 圆 B 顺时针（起点正上方，θ=π/2 递减）
            theta = np.pi / 2.0 - (d - circ - dist) / radius
            return np.array([bx + radius * np.cos(theta),
                             cy + radius * np.sin(theta)])
        # 上切线 B→A
        f = (d - 2.0 * circ - dist) / dist
        return np.array([bx + (ax - bx) * f, cy + radius])

    pts = np.array([point_at(d) for d in s])
    lap_time = perimeter / max(airspeed, 1e-3)
    t = s / perimeter * lap_time
    alt = altitude + climb_rate * t
    return np.stack([pts[:, 0], pts[:, 1], alt], axis=1)


def figure8_loiter_segments(
    center_ne: tuple[float, float], radius: float,
    center_dist: float | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
    """双圆切线 8 字 → loiter 切圆段解析。

    返回 (centers [2,2], entries [2,2], exits [2,2], directions [2],
    sweeps [2], radius)。段 0：圆 A 逆时针整圈（入口=出口=顶部切点）；
    段 1：圆 B 顺时针整圈；两段之间由顶部公切线直飞连接。
    """
    cx, cy = center_ne
    dist = 2.0 * radius if center_dist is None else float(center_dist)
    ax, bx = cx - dist / 2.0, cx + dist / 2.0
    centers = np.array([[ax, cy], [bx, cy]], dtype=np.float64)
    entries = np.array([[ax, cy + radius], [bx, cy + radius]], dtype=np.float64)
    exits = entries.copy()  # 整圈：出口 = 入口（顶部切点）
    directions = np.array([-1.0, -1.0], dtype=np.float64)  # 均顺时针（切点速度朝北）
    sweeps = np.array([2.0 * np.pi, 2.0 * np.pi], dtype=np.float64)
    return centers, entries, exits, directions, sweeps, float(radius)


def racetrack_loiter_segments(
    center_ne: tuple[float, float], straight: float, radius: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
    """跑马场 → loiter 切圆段解析。

    返回 (centers [2,2], entries [2,2], exits [2,2], directions [2],
    sweeps [2], radius)。段 0：北半圆（西端入口，顺时针经顶部到东端出口，
    扫 π）；段 1：南半圆（东端入口，顺时针经底部到西端出口，扫 π）。
    直飞段自动落在两半圆的切线（东/西直线）上。
    """
    cx, cy = center_ne
    half = straight / 2.0
    centers = np.array([[cx + half, cy], [cx - half, cy]], dtype=np.float64)
    entries = np.array([[cx + half, cy - radius],
                        [cx - half, cy + radius]], dtype=np.float64)
    exits = np.array([[cx + half, cy + radius],
                      [cx - half, cy - radius]], dtype=np.float64)
    directions = np.array([1.0, 1.0], dtype=np.float64)  # 北/南半圆均逆时针
    sweeps = np.array([np.pi, np.pi], dtype=np.float64)
    return centers, entries, exits, directions, sweeps, float(radius)


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
        # 双圆切线相接 8 字（figure8_tangent_path）
        return figure8_tangent_path(
            center, float(cfg.get("radius_m", 80.0)), altitude,
            center_dist=float(cfg.get("center_dist_m", 2.0 * cfg.get("radius_m", 80.0))),
            n_pts=n_pts, climb_rate=climb, airspeed=airspeed,
        )
    if task_type == "racetrack":
        return racetrack_path(
            center, float(cfg.get("straight_m", 400.0)),
            float(cfg.get("radius_m", 80.0)), altitude,
            n_pts=n_pts, climb_rate=climb, airspeed=airspeed,
        )
    raise ValueError(f"unsupported task type: {task_type}")
