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


def figure8_spatial_path(
    center_ne: tuple[float, float],
    radius: float,
    altitude: float,
    n_pts: int = 1441,
    center_dist: float = 400.0,
    center_dh: float = 10.0,
    climb_rate: float = 0.0,
    airspeed: float = 16.0,
) -> np.ndarray:
    """空间 8 字航迹 [N,3]（NEU）：两个大圆弧 + 两条公切线，全路径连续。

    几何（WP2A-8 v4 修正）：
    - 左圆（A）圆心 (cx-D/2, cy)，高度 H+5；右圆（B）圆心 (cx+D/2, cy)，
      高度 H-5（左圆高于右圆，高度差 center_dh=10 m）；
    - 每圆两个切点：上端点 = 圆心 +5 m 高度，下端点 = 圆心 -5 m 高度；
      切点方位角 α = acos(2R/D)（内公切线几何，保证切线与圆相切且接点
      切向连续）；
    - 切线 1：A 上端点 (H+10) → B 下端点 (H-10)（俯冲 20 m）；
    - 切线 2：B 上端点 (H) → A 下端点 (H)（水平，穿过两圆之间）；
    - A 大弧：A 下端点顺时针绕 312.8° 到 A 上端点（螺旋爬升 10 m）；
    - B 大弧：B 下端点逆时针绕 312.8° 到 B 上端点（螺旋爬升 10 m）；
    - 遍历顺序：切线1 → B 大弧 → 切线2 → A 大弧 → 切线1 闭合，
      各接点切向一致（无尖点）。

    切线 1/2 在水平投影上为 X 形，中点 (0,0) 处空间交于一点 (0,0,H)
    （对称内公切线的必然结果）；飞行沿时间顺序只经过一条切线，不影响跟踪。
    """
    cx, cy = center_ne
    h_center = altitude
    half = center_dist / 2.0
    ax, bx = cx - half, cx + half
    h_a, h_b = h_center + center_dh / 2.0, h_center - center_dh / 2.0
    alpha = np.arccos(np.clip(2.0 * radius / center_dist, 0.0, 1.0))
    r_cos, r_sin = radius * np.cos(alpha), radius * np.sin(alpha)

    # 四个切点（NEU 水平 + 高度）
    a_upper = np.array([ax + r_cos, cy + r_sin, h_a + 5.0])
    a_lower = np.array([ax + r_cos, cy - r_sin, h_a - 5.0])
    b_upper = np.array([bx - r_cos, cy + r_sin, h_b + 5.0])
    b_lower = np.array([bx - r_cos, cy - r_sin, h_b - 5.0])

    t1 = b_lower - a_upper                      # 切线 1（俯冲）
    t2 = a_lower - b_upper                      # 切线 2（水平）
    len1 = float(np.linalg.norm(t1))
    len2 = float(np.linalg.norm(t2))
    sweep = 2.0 * np.pi - 2.0 * alpha           # 大弧扫角
    arc_len = sweep * radius                    # 单段大弧长
    perimeter = len1 + len2 + 2.0 * arc_len
    s = np.linspace(0.0, perimeter, n_pts)

    # 分段累计弧长
    s1 = len1
    s2 = len1 + arc_len                         # B 大弧结束
    s3 = len1 + arc_len + len2                  # 切线 2 结束

    pts = np.empty((n_pts, 3))
    mask_t1 = s <= len1
    mask_b = (s > len1) & (s <= s2)
    mask_t2 = (s > s2) & (s <= s3)
    mask_a = s > s3

    # 切线 1：A_upper → B_lower
    pts[mask_t1] = a_upper + (s[mask_t1] / len1)[:, None] * t1
    # B 大弧：逆时针从 π+α 扫到 π-α+2π，高度 hB-5 → hB+5
    u = (s[mask_b] - len1) / arc_len
    theta = np.pi + alpha + sweep * u
    pts[mask_b, 0] = bx + radius * np.cos(theta)
    pts[mask_b, 1] = cy + radius * np.sin(theta)
    pts[mask_b, 2] = h_b - 5.0 + 10.0 * u
    # 切线 2：B_upper → A_lower（水平）
    pts[mask_t2] = b_upper + ((s[mask_t2] - s2) / len2)[:, None] * t2
    # A 大弧：顺时针从 -α 扫到 -2π+α，高度 hA-5 → hA+5
    u = (s[mask_a] - s3) / arc_len
    theta = -alpha - sweep * u
    pts[mask_a, 0] = ax + radius * np.cos(theta)
    pts[mask_a, 1] = cy + radius * np.sin(theta)
    pts[mask_a, 2] = h_a - 5.0 + 10.0 * u
    return pts


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
        # 空间 8 字（两个大弧 + 两条错高公切线）
        return figure8_spatial_path(
            center, float(cfg.get("radius_m", 80.0)), altitude,
            center_dist=float(cfg.get("center_dist_m", 400.0)),
            center_dh=float(cfg.get("center_dh_m", 10.0)),
            n_pts=n_pts, climb_rate=climb, airspeed=airspeed,
        )
    if task_type == "racetrack":
        return racetrack_path(
            center, float(cfg.get("straight_m", 400.0)),
            float(cfg.get("radius_m", 80.0)), altitude,
            n_pts=n_pts, climb_rate=climb, airspeed=airspeed,
        )
    raise ValueError(f"unsupported task type: {task_type}")
