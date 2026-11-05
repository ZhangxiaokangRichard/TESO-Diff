"""WP2A-endpoint 四条三维期望航迹（NEU，单位 m）。

统一约定：参考高度以起点 100 m 为基准（与飞机初始高度一致，使高度参考与
航迹高度在起点重合）；闭合航迹净爬升为零；爬升率由路径弧段斜率经
PolylinePathFollower(altitude_follow=True) 转为航迹倾角指令。

S1 螺旋线（开放）：圆心 (200,0)，半径 80 m，爬升角 atan(0.05)≈2.86°，12 圈。
S2 空间 8 字（闭合）：两圆圆心高度相差 50 m（上圆 125 m / 下圆 75 m，弧段
   在圆心高度 ±5 m 内螺旋），两圆心水平相距 400 m；起点取两条公切线的交点
   (200,0,100)。几何由 python.simulation.task_route.figure8_spatial_path 给出。
S3 跑马场（闭合）：北端半圆圆心高 100 m、南端半圆圆心高 50 m（相差 50 m），
   两直道线性过渡。
S4 GPS 三角（闭合）：边长 600 m 的等边三角形，圆角半径 60 m；三个圆角圆心高度
   依次为 100 / 150 / 200 m（递增 50 m），直线段线性过渡，闭合段下降 100 m。
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from python.simulation.task_route import figure8_spatial_path, racetrack_path  # noqa: E402

BASE_ALT_M = 100.0
IAS_CMD_MPS = 16.0
HELIX_CENTER = (200.0, 0.0)
HELIX_RADIUS_M = 80.0
HELIX_SLOPE = 0.05            # 爬升 / 水平路程，对应 γ≈2.86°
HELIX_LAPS = 12
RACE_CENTER = (200.0, 0.0)
RACE_STRAIGHT_M = 820.0
RACE_RADIUS_M = 80.0
RACE_CENTER_DH_M = 50.0
FIG8_CENTER_DIST_M = 1000.0
FIG8_RADIUS_M = 80.0
TRI_SIDE_M = 1850.0
TRI_CORNER_R_M = 60.0
TRI_CORNER_DH_M = 50.0
TRI_CENTROID = (200.0, 0.0)


def horizontal_perimeter(path: np.ndarray, closed: bool) -> float:
    """水平投影路程长度（闭合时计入首尾连接）。"""
    total = float(np.sum(np.linalg.norm(np.diff(path[:, :2], axis=0), axis=1)))
    if closed:
        total += float(np.linalg.norm(path[0, :2] - path[-1, :2]))
    return total


def helix_path() -> np.ndarray:
    """开放螺旋线：逆时针，爬升斜率 HELIX_SLOPE，首点高度 100 m。"""
    n = HELIX_LAPS * 240 + 1
    theta = np.linspace(0.0, 2.0 * np.pi * HELIX_LAPS, n)
    arc = HELIX_RADIUS_M * theta
    return np.column_stack([
        HELIX_CENTER[0] + HELIX_RADIUS_M * np.cos(theta),
        HELIX_CENTER[1] + HELIX_RADIUS_M * np.sin(theta),
        BASE_ALT_M + HELIX_SLOPE * arc,
    ])


def figure8_3d_path() -> np.ndarray:
    """空间 8 字：两圆心高度相差 50 m，起点为两切线交点 (200,0,100)。"""
    path = figure8_spatial_path(
        (200.0, 0.0), FIG8_RADIUS_M, BASE_ALT_M, n_pts=1441,
        center_dist=FIG8_CENTER_DIST_M, center_dh=RACE_CENTER_DH_M,
    )[:-1]
    i0 = int(np.argmin(np.linalg.norm(path - np.array([200.0, 0.0, BASE_ALT_M]), axis=1)))
    return np.roll(path, -i0, axis=0)


def racetrack_3d_path() -> np.ndarray:
    """跑马场：北端半圆 100 m、南端半圆 50 m，直道线性过渡。"""
    n = 1441
    planar = racetrack_path(RACE_CENTER, straight=RACE_STRAIGHT_M, radius=RACE_RADIUS_M,
                            altitude=BASE_ALT_M, n_pts=n, climb_rate=0.0)[:-1]
    arc = np.pi * RACE_RADIUS_M
    st = RACE_STRAIGHT_M
    perimeter = 2.0 * st + 2.0 * arc
    s = np.linspace(0.0, perimeter, n)[:-1]
    north_top = BASE_ALT_M
    south_low = BASE_ALT_M - RACE_CENTER_DH_M
    alt = np.where(
        s < arc, north_top,
        np.where(
            s < arc + st, north_top - RACE_CENTER_DH_M * (s - arc) / st,
            np.where(
                s < 2.0 * arc + st, south_low,
                south_low + RACE_CENTER_DH_M * (s - 2.0 * arc - st) / st,
            ),
        ),
    )
    planar[:, 2] = alt
    return planar


def _triangle_vertices() -> np.ndarray:
    cx, cy = TRI_CENTROID
    rc = TRI_SIDE_M / np.sqrt(3.0)
    return np.array([
        [cx - rc, cy],
        [cx + rc / 2.0, cy - TRI_SIDE_M / 2.0],
        [cx + rc / 2.0, cy + TRI_SIDE_M / 2.0],
    ])


def gps_triangle_path() -> np.ndarray:
    """等边三角形圆角航线（逆序无关），三个圆角高度 100/150/200 m，闭合。"""
    vertices = _triangle_vertices()
    corner_alt = BASE_ALT_M + TRI_CORNER_DH_M * np.arange(3)
    rc = TRI_CORNER_R_M
    n = len(vertices)
    arcs = []
    for i in range(n):
        p = vertices[i]
        a = vertices[(i - 1) % n]
        b = vertices[(i + 1) % n]
        u1 = (a - p) / np.linalg.norm(a - p)
        u2 = (b - p) / np.linalg.norm(b - p)
        phi = np.arccos(np.clip(np.dot(u1, u2), -1.0, 1.0))
        tangent_dist = rc / np.tan(phi / 2.0)
        t_in = p + u1 * tangent_dist
        t_out = p + u2 * tangent_dist
        bisector = (u1 + u2) / np.linalg.norm(u1 + u2)
        center = p + bisector * rc / np.sin(phi / 2.0)
        a_in = np.arctan2(t_in[1] - center[1], t_in[0] - center[0])
        a_out = np.arctan2(t_out[1] - center[1], t_out[0] - center[0])
        delta = (a_out - a_in + np.pi) % (2.0 * np.pi) - np.pi
        n_arc = max(6, int(round(abs(delta) * rc / 4.0)))
        theta = a_in + delta * np.linspace(0.0, 1.0, n_arc)
        pts = center + rc * np.column_stack([np.cos(theta), np.sin(theta)])
        arcs.append(dict(t_in=t_in, t_out=t_out, pts=pts))
    ne_pieces, alt_pieces = [], []
    for i in range(n):
        j = (i + 1) % n
        ne_pieces.append(arcs[i]["pts"])
        alt_pieces.append(np.full(len(arcs[i]["pts"]), corner_alt[i]))
        start, end = arcs[i]["t_out"], arcs[j]["t_in"]
        n_line = max(2, int(round(np.linalg.norm(end - start) / 4.0)))
        frac = np.linspace(0.0, 1.0, n_line)
        line = start + (end - start) * frac[:, None]
        ne_pieces.append(line[1:-1])
        alt_pieces.append(corner_alt[i] + (corner_alt[j] - corner_alt[i]) * frac[1:-1])
    ne = np.vstack(ne_pieces)
    alt = np.concatenate(alt_pieces)
    return np.column_stack([ne, alt])


ROUTES = {
    "helix": dict(
        scenario="S1", label="螺旋线", build=helix_path, closed=False,
        lap_length_m=2.0 * np.pi * HELIX_RADIUS_M,
        params=dict(center_ne=list(HELIX_CENTER), radius_m=HELIX_RADIUS_M,
                    climb_slope=HELIX_SLOPE, gamma_deg=float(np.degrees(np.arctan(HELIX_SLOPE))),
                    laps=HELIX_LAPS, start_altitude_m=BASE_ALT_M),
    ),
    "figure8": dict(
        scenario="S2", label="空间8字", build=figure8_3d_path, closed=True,
        params=dict(circle_centers_ne=[[200 - FIG8_CENTER_DIST_M / 2, 0],
                                       [200 + FIG8_CENTER_DIST_M / 2, 0]],
                    radius_m=FIG8_RADIUS_M,
                    center_altitude_m=[BASE_ALT_M + 25.0, BASE_ALT_M - 25.0],
                    center_dh_m=RACE_CENTER_DH_M, start_ne_alt=[200, 0, BASE_ALT_M]),
    ),
    "racetrack": dict(
        scenario="S3", label="跑马场", build=racetrack_3d_path, closed=True,
        params=dict(center_ne=list(RACE_CENTER), straight_m=RACE_STRAIGHT_M,
                    radius_m=RACE_RADIUS_M, end_arc_altitude_m=[BASE_ALT_M, BASE_ALT_M - RACE_CENTER_DH_M],
                    center_dh_m=RACE_CENTER_DH_M),
    ),
    "gps_triangle": dict(
        scenario="S4", label="GPS三角", build=gps_triangle_path, closed=True,
        params=dict(vertices_ne=_triangle_vertices().tolist(), side_m=TRI_SIDE_M,
                    corner_radius_m=TRI_CORNER_R_M,
                    corner_altitude_m=[BASE_ALT_M + TRI_CORNER_DH_M * k for k in range(3)]),
    ),
}

for _route in ROUTES.values():
    _path = _route["build"]()
    _route["lap_length_m"] = _route.get("lap_length_m", horizontal_perimeter(_path, _route["closed"]))
    _route["max_climb_slope"] = float(np.max(np.abs(np.diff(_path[:, 2]))
                                          / np.maximum(np.linalg.norm(np.diff(_path[:, :2], axis=0), axis=1), 1e-9)))

SUBJECTS = tuple(ROUTES)

FOLLOWER_KWARGS = dict(lookahead_min_m=35.0, lookahead_max_m=55.0, altitude_follow=True)
