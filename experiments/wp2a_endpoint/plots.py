"""WP2A-endpoint 出图（中文图题与图例）。

颜色按实体固定（Okabe–Ito 色盲友好）：风场 W0 灰 / W1 朱红 / W2 蓝；科目各一色。
获能功率为极性量：发散色图 RdBu_r，0 居中；风速为幅值量：单色渐变。
由 run_endpoint.py 调用 render_all() 生成；也可独立运行，从 data/*.npz 重绘。
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
for _p in (ROOT, HERE, ROOT / "experiments" / "wp2a8"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from mpl_style import save_figure, setup_chinese_font  # noqa: E402
from wind_fields import make_wind  # noqa: E402
from routes import BASE_ALT_M, ROUTES, SUBJECTS  # noqa: E402

WIND_STYLE = {
    "W0": ("W0 无风", "#666666"),
    "W1": ("W1 热气流", "#D55E00"),
    "W2": ("W2 垂直切变层", "#0072B2"),
}
SUBJECT_COLORS = {"helix": "#009E73", "figure8": "#CC79A7",
                  "racetrack": "#56B4E9", "gps_triangle": "#E69F00"}
THERMAL_CENTER = (200.0, 0.0)
THERMAL_RADIUS_M = 90.0
HEAT_BIN_NE_M = 30.0
HEAT_BIN_H_M = 5.0
RAD2DEG = 180.0 / np.pi


def _plan_extent(refs: dict, margin: float = 120.0):
    pts = np.vstack(list(refs.values()))
    n_lo, n_hi = pts[:, 0].min() - margin, pts[:, 0].max() + margin
    e_lo, e_hi = pts[:, 1].min() - margin, pts[:, 1].max() + margin
    return (n_lo, n_hi), (e_lo, e_hi)


def _route_overlay(ax, refs: dict):
    for subject, path in refs.items():
        ax.plot(path[:, 1], path[:, 0], color=SUBJECT_COLORS[subject], lw=1.3,
                label=f"参考航迹·{ROUTES[subject]['label']}")
    theta = np.linspace(0, 2 * np.pi, 200)
    ax.plot(THERMAL_CENTER[1] + THERMAL_RADIUS_M * np.cos(theta),
            THERMAL_CENTER[0] + THERMAL_RADIUS_M * np.sin(theta),
            color="#D55E00", lw=0.8, ls=":", label="热柱范围 R=90 m")


def _plan_axes(ax, refs: dict, n_lim, e_lim, title: str):
    _route_overlay(ax, refs)
    ax.plot(0, 0, "k^", ms=7, label="起点 (0, 0, 100 m)")
    ax.set(xlabel="东 (m)", ylabel="北 (m)", title=title, xlim=e_lim, ylim=n_lim)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)


def plot_wind_environment(refs: dict, out_dir: Path):
    n_lim, e_lim = _plan_extent(refs)
    ne = np.linspace(n_lim[0], n_lim[1], 180)
    ee = np.linspace(e_lim[0], e_lim[1], 180)
    nn, eg = np.meshgrid(ne, ee, indexing="ij")
    pos = np.column_stack([nn.ravel(), eg.ravel(), np.full(nn.size, 100.0)])

    fig, ax = plt.subplots(figsize=(9, 8), constrained_layout=True)
    _plan_axes(ax, refs, n_lim, e_lim, "W0 无风环境（平面 高度 100 m）")
    ax.legend(fontsize=7, loc="upper left")
    save_figure(fig, out_dir / "wind_env_W0.png")

    field = make_wind("W1")
    w_up = field.up_mps(pos).reshape(nn.shape)
    fig, ax = plt.subplots(figsize=(9, 8), constrained_layout=True)
    mesh = ax.pcolormesh(eg[0], nn[:, 0], w_up, cmap="Oranges", shading="auto", vmin=0.0)
    fig.colorbar(mesh, ax=ax, label="上升气流 w_up (m/s)")
    cs = ax.contour(eg, nn, w_up, levels=[0.5, 1.0, 1.5], colors="#8a4a00", linewidths=0.6)
    ax.clabel(cs, fontsize=7, fmt="%.1f")
    _plan_axes(ax, refs, n_lim, e_lim, "W1 热气流环境（高斯热柱，中心 (200, 0)，R=90 m，峰值 2.0 m/s）")
    ax.legend(fontsize=7, loc="upper left")
    save_figure(fig, out_dir / "wind_env_W1.png")

    field = make_wind("W2")
    alt_band = (45.0, 205.0)
    h = np.linspace(20.0, 420.0, 400)
    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(14, 7.5), constrained_layout=True,
                                   gridspec_kw=dict(width_ratios=[1, 1.35]))
    east = field.east_mps(np.column_stack([np.zeros_like(h), np.zeros_like(h), h]))
    ax0.axhspan(*alt_band, color="#dbe9f4", label="航迹高度范围 45-205 m")
    ax0.plot(east, h, color=WIND_STYLE["W2"][1], lw=2.0, label="东向风速 W_e(h)")
    ax0.axhline(field.h0, color="#444444", ls="--", lw=0.9,
                label=f"切变层中心 h0={field.h0:.0f} m")
    ax0.set(xlabel="东向风速 (m/s)", ylabel="高度 (m)", title="W2 风速剖面：随高度的东向切变",
            ylim=(20, 420), xlim=(-0.3, 4.6))
    ax0.grid(alpha=0.25)
    ax0.legend(fontsize=8, loc="lower right")

    nn_h = np.linspace(n_lim[0], n_lim[1], 60)
    hh = np.linspace(20.0, 420.0, 200)
    ng, hg = np.meshgrid(nn_h, hh, indexing="ij")
    east_grid = field.east_mps(np.column_stack([ng.ravel(), np.zeros(ng.size), hg.ravel()]))
    mesh = ax1.pcolormesh(nn_h, hh, east_grid.reshape(ng.shape).T, cmap="Blues",
                          shading="auto", vmin=0.0, vmax=4.0)
    fig.colorbar(mesh, ax=ax1, label="东向风速 (m/s)")
    for subject, path in refs.items():
        ax1.plot(path[:, 0], path[:, 2], color=SUBJECT_COLORS[subject], lw=1.3,
                 label=f"参考航迹·{ROUTES[subject]['label']}")
    ax1.plot(0, 100.0, "k^", ms=7, label="起点 (0, 0, 100 m)")
    ax1.set(xlabel="北 (m)", ylabel="高度 (m)", title="W2 北-高剖面：风场与参考航迹投影",
            ylim=(20, 420))
    ax1.grid(alpha=0.25)
    ax1.legend(fontsize=7, loc="upper right")
    fig.suptitle("W2 垂直切变层试验环境（东向风随高度变化，水平面内均匀）", fontsize=11)
    save_figure(fig, out_dir / "wind_env_W2.png")


def plot_subject_trajectories(all_runs: dict, refs: dict, out_dir: Path):
    for subject in SUBJECTS:
        runs = {w: all_runs[(subject, w)] for w in WIND_STYLE if (subject, w) in all_runs}
        if not runs:
            continue
        ref = refs[subject]
        n_lim, e_lim = _plan_extent({subject: ref}, margin=80.0)
        fig, axes = plt.subplots(2, 3, figsize=(17, 10), constrained_layout=True)
        a = axes.ravel()
        a[0].plot(ref[:, 1], ref[:, 0], "k--", lw=0.9, label="参考航迹")
        for w, ch in runs.items():
            label, color = WIND_STYLE[w]
            t = ch["time/t_rel"]
            a[0].plot(ch["state/pos_neu"][:, 1], ch["state/pos_neu"][:, 0], color=color, lw=0.9, label=label)
            a[1].plot(t, ch["state/pos_neu"][:, 2], color=color, lw=0.9, label=label)
            a[1].plot(t, ch["action/altitude_cmd_m"], color=color, lw=0.7, ls="--", alpha=0.6)
            a[2].plot(t, ch["state/ias_est_mps"], color=color, lw=0.8, label=label)
            a[3].plot(t, ch["action/path_cross_track_m"], color=color, lw=0.8, label=label)
            a[4].plot(t, ch["energy/p_env_total_wpkg"], color=color, lw=0.8, label=label)
            a[5].plot(t, ch["energy/E_env_cum_jpkg"], color=color, lw=1.1, label=label)
        first = next(iter(runs.values()))
        a[2].plot(first["time/t_rel"], first["action/ias_cmd_mps"], "k--", lw=0.8, label="指令空速")
        a[0].set(title="地面轨迹（东-北平面）", xlabel="东 (m)", ylabel="北 (m)",
                 xlim=e_lim, ylim=n_lim)
        a[0].set_aspect("equal")
        a[1].set(title="高度（实线：实际；虚线：高度参考）", ylabel="高度 (m)")
        a[2].set(title="指示空速", ylabel="m/s")
        a[3].set(title="航迹横侧偏差（带符号）", ylabel="m")
        a[4].set(title="环境比功率 p_env = g·w_up - v_air·dW/dt", ylabel="W/kg")
        a[5].set(title="累计环境比能", ylabel="J/kg")
        for ax in a:
            ax.grid(alpha=0.25)
            ax.legend(fontsize=7, loc="best")
        for ax in a[1:]:
            ax.set_xlabel("时间 (s)")
        fig.suptitle(f"{ROUTES[subject]['label']}（{ROUTES[subject]['scenario']}）· "
                     "W0 / W1 / W2 · 三维期望航迹 · 生成姿态内环 + 批量 NPFG/TECS · 50 Hz",
                     fontsize=11)
        save_figure(fig, out_dir / f"traj_{subject}.png")


def plot_subject_3d(all_runs: dict, refs: dict, out_dir: Path):
    for subject in SUBJECTS:
        runs = {w: all_runs[(subject, w)] for w in WIND_STYLE if (subject, w) in all_runs}
        if not runs:
            continue
        ref = refs[subject]
        n_lim, e_lim = _plan_extent({subject: ref}, margin=80.0)
        z_lo = min(ref[:, 2].min(), *(c["state/pos_neu"][:, 2].min() for c in runs.values()))
        z_hi = max(ref[:, 2].max(), *(c["state/pos_neu"][:, 2].max() for c in runs.values()))
        span_h = max(n_lim[1] - n_lim[0], e_lim[1] - e_lim[0])
        span_z = max(z_hi - z_lo, 1.0)
        k = max(1.0, 0.3 * span_h / span_z)
        fig = plt.figure(figsize=(11, 9), constrained_layout=True)
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(ref[:, 1], ref[:, 0], ref[:, 2], "k--", lw=1.0, label="参考航迹")
        for w, ch in runs.items():
            label, color = WIND_STYLE[w]
            pos = ch["state/pos_neu"]
            ax.plot(pos[:, 1], pos[:, 0], pos[:, 2], color=color, lw=0.9, label=label)
        ax.scatter([0], [0], [BASE_ALT_M], color="k", marker="^", s=40, label="起点")
        ax.set_xlabel("东 (m)")
        ax.set_ylabel("北 (m)")
        ax.set_zlabel("高度 (m)")
        ax.set_xlim(e_lim)
        ax.set_ylim(n_lim)
        ax.set_zlim(z_lo - 5, z_hi + 5)
        ax.set_box_aspect((e_lim[1] - e_lim[0], n_lim[1] - n_lim[0], (span_z + 10) * k))
        ax.legend(fontsize=8, loc="upper left")
        ax.set_title(f"{ROUTES[subject]['label']}（{ROUTES[subject]['scenario']}）三维航迹 · "
                     f"高度显示放大 {k:.1f} 倍", fontsize=11)
        save_figure(fig, out_dir / f"traj3d_{subject}.png")


def _bin_mean(x, y, values, x_edges, y_edges):
    total, _, _ = np.histogram2d(x, y, bins=[x_edges, y_edges], weights=values)
    count, _, _ = np.histogram2d(x, y, bins=[x_edges, y_edges])
    with np.errstate(invalid="ignore", divide="ignore"):
        mean = np.where(count > 0, total / np.maximum(count, 1), np.nan)
    return np.ma.masked_invalid(mean)


def _heat_scale(values_list):
    peak = max(float(np.percentile(np.abs(v), 98)) for v in values_list if len(v))
    return max(peak, 0.5)


def plot_energy_heatmaps(all_runs: dict, refs: dict, out_dir: Path):
    n_lim, e_lim = _plan_extent(refs)
    ne_edges = (np.arange(n_lim[0], n_lim[1] + HEAT_BIN_NE_M, HEAT_BIN_NE_M),
                np.arange(e_lim[0], e_lim[1] + HEAT_BIN_NE_M, HEAT_BIN_NE_M))
    h_edges = np.arange(20.0, 420.0 + HEAT_BIN_H_M, HEAT_BIN_H_M)
    for wind_id in WIND_STYLE:
        runs = {s: all_runs[(s, wind_id)] for s in SUBJECTS if (s, wind_id) in all_runs}
        if not runs:
            continue
        pos = {s: c["state/pos_neu"] for s, c in runs.items()}
        val = {s: c["energy/p_env_total_wpkg"] for s, c in runs.items()}
        merged_pos = np.vstack(list(pos.values()))
        merged_val = np.concatenate(list(val.values()))
        scale = _heat_scale(list(val.values()))
        label = WIND_STYLE[wind_id][0]

        for plane in ("NE", "NH"):
            if plane == "NH" and wind_id != "W2":
                continue
            fig, axes = plt.subplots(2, 3, figsize=(17, 10.5), constrained_layout=True)
            a = axes.ravel()
            panels = [("全部科目（合并）", merged_pos, merged_val)]
            panels += [(ROUTES[s]["label"], pos[s], val[s]) for s in runs]
            mesh = None
            for ax, (title, p, v) in zip(a[:5], panels):
                if plane == "NE":
                    mean = _bin_mean(p[:, 0], p[:, 1], v, *ne_edges)
                    mesh = ax.pcolormesh(ne_edges[1], ne_edges[0], mean, cmap="RdBu_r",
                                         vmin=-scale, vmax=scale, shading="auto")
                    ax.set(xlabel="东 (m)", ylabel="北 (m)", xlim=e_lim, ylim=n_lim)
                    ax.set_aspect("equal")
                else:
                    mean = _bin_mean(p[:, 0], p[:, 2], v, ne_edges[0], h_edges)
                    mesh = ax.pcolormesh(ne_edges[0], h_edges, mean.T, cmap="RdBu_r",
                                         vmin=-scale, vmax=scale, shading="auto")
                    ax.set(xlabel="北 (m)", ylabel="高度 (m)", xlim=n_lim, ylim=(20, 420))
                ax.set_title(title)
                ax.grid(alpha=0.2)
            ax = a[5]
            if plane == "NE":
                if wind_id == "W1":
                    field = make_wind("W1")
                    ax.set_title("风场真值：上升气流 w_up (m/s)")
                    ne = np.linspace(n_lim[0], n_lim[1], 120)
                    ee = np.linspace(e_lim[0], e_lim[1], 120)
                    nn, eg = np.meshgrid(ne, ee, indexing="ij")
                    grid = np.column_stack([nn.ravel(), eg.ravel(), np.full(nn.size, 100.0)])
                    ax.pcolormesh(ee, ne, field.up_mps(grid).reshape(nn.shape), cmap="Oranges",
                                  shading="auto", vmin=0.0)
                else:
                    ax.set_title("风场真值：无风")
                for subject, path in refs.items():
                    ax.plot(path[:, 1], path[:, 0], color=SUBJECT_COLORS[subject], lw=1.1)
                ax.set(xlabel="东 (m)", ylabel="北 (m)", xlim=e_lim, ylim=n_lim)
                ax.set_aspect("equal")
            else:
                ax.set_title("风场真值：东向风速 (m/s)")
                nh = np.linspace(n_lim[0], n_lim[1], 60)
                hh = np.linspace(20.0, 420.0, 200)
                ng, hg = np.meshgrid(nh, hh, indexing="ij")
                east = make_wind("W2").east_mps(np.column_stack(
                    [ng.ravel(), np.zeros(ng.size), hg.ravel()]))
                ax.pcolormesh(nh, hh, east.reshape(ng.shape).T, cmap="Blues",
                              shading="auto", vmin=0.0, vmax=4.0)
                for subject, path in refs.items():
                    ax.plot(path[:, 0], path[:, 2], color=SUBJECT_COLORS[subject], lw=1.1)
                ax.set(xlabel="北 (m)", ylabel="高度 (m)", xlim=n_lim, ylim=(20, 420))
            ax.grid(alpha=0.2)
            fig.colorbar(mesh, ax=axes, shrink=0.8,
                         label="沿航迹平均环境比功率 p_env (W/kg)")
            plane_name = "东-北平面" if plane == "NE" else "北-高剖面"
            fig.suptitle(f"{label} · 全局获能热力图（{plane_name}，分箱 "
                         f"{HEAT_BIN_NE_M:.0f} m{' × ' + str(HEAT_BIN_H_M) + ' m' if plane == 'NH' else ''}）"
                         "· 四科目 × 480 s · 由风真值解析导出的标签", fontsize=11)
            save_figure(fig, out_dir / f"heatmap_{wind_id}_{plane}.png")


def render_all(all_runs: dict, refs: dict, out_dir: Path):
    setup_chinese_font()
    out_dir.mkdir(exist_ok=True)
    plot_wind_environment(refs, out_dir)
    plot_subject_trajectories(all_runs, refs, out_dir)
    plot_subject_3d(all_runs, refs, out_dir)
    plot_energy_heatmaps(all_runs, refs, out_dir)


def load_runs(data_dir: Path) -> dict:
    runs = {}
    for npz in sorted(data_dir.glob("eso_*.npz")):
        subject, wind_id = npz.stem[len("eso_"):].rsplit("_", 1)
        with np.load(npz) as archive:
            runs[(subject, wind_id)] = {k: archive[k] for k in archive.files}
    return runs


if __name__ == "__main__":
    loaded = load_runs(HERE / "data")
    references = {s: ROUTES[s]["build"]() for s in SUBJECTS}
    render_all(loaded, references, HERE / "figures")
    print("replotted", len(loaded), "runs")
