"""内环串级姿态跟踪整定评估器（WP2A-8 v2，GA 适应度）。

染色体（13 维实数）与搜索范围（PX4 串级架构）：

    0  angle_roll_kp   [1.0, 10.0]   角度环 P（rad/s per rad）
    1  angle_pitch_kp  [1.0, 10.0]
    2  angle_yaw_kp    [0.5, 6.0]
    3  roll_rate_kp    [0.05, 1.5]   角速率环 P
    4  roll_rate_ff    [0.0, 1.0]    角加速度前馈
    5  pitch_rate_kp   [0.05, 1.5]
    6  pitch_rate_ki   [0.0, 4.0]    俯仰速率积分（建立配平舵量）
    7  pitch_rate_ff   [0.0, 1.0]
    8  yaw_rate_kp     [0.05, 1.5]
    9  yaw_rate_ki     [0.0, 4.0]
    10 yaw_rate_ff     [0.0, 1.0]
    11 theta_trim_deg  [-6.0, 15.0]
    12 throttle        [0.2, 0.6]

物理符号（舵面符号复测实测）在 InnerLoop 内部固定：delta_a/delta_e 负反馈、
delta_r 正反馈；GA 只整定量级。

评估剧本（固定、可复现，两阶段）：
    阶段 1（0~8 s）：平飞稳定（phi_cmd=0、theta_cmd=trim、psi_cmd=0）
    阶段 2（8~25 s）：phi_cmd = ±15° 方波（每 5 s 切换），theta/psi 不变
    仿真 25 s（1250 tick），无风。

代价：前 5 s 预热不计；姿态误差 MAE 加权 + 角速率惩罚 + 超速/超限惩罚。
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

from ...bindings import AvantiInput, AvantiLibrary
from ..inner import InnerLoop, wrap_pi
from ..types import ControlObservation

GENE_NAMES = [
    "angle_roll_kp", "angle_pitch_kp", "angle_yaw_kp",
    "roll_rate_kp", "roll_rate_ff",
    "pitch_rate_kp", "pitch_rate_ki", "pitch_rate_ff",
    "yaw_rate_kp", "yaw_rate_ki", "yaw_rate_ff",
    "theta_trim_deg", "throttle",
]

BOUNDS = [
    (1.0, 10.0),     # angle_roll_kp
    (1.0, 10.0),     # angle_pitch_kp
    (0.5, 6.0),      # angle_yaw_kp
    (0.05, 1.5),     # roll_rate_kp
    (0.0, 1.0),      # roll_rate_ff
    (0.05, 1.5),     # pitch_rate_kp
    (0.0, 4.0),      # pitch_rate_ki
    (0.0, 1.0),      # pitch_rate_ff
    (0.05, 1.5),     # yaw_rate_kp
    (0.0, 4.0),      # yaw_rate_ki
    (0.0, 1.0),      # yaw_rate_ff
    (-6.0, 15.0),    # theta_trim_deg
    (0.2, 0.6),      # throttle
]

# 手工粗调种子（PX4 常用量级：角度环 2~5、速率环 0.1~0.3、FF 0.3）
SEED_INDIVIDUALS = [
    np.array([3.0, 3.0, 2.0, 0.10, 0.30, 0.10, 0.10, 0.30,
              0.10, 0.10, 0.30, 2.0, 0.5]),
    np.array([4.0, 4.0, 2.5, 0.15, 0.40, 0.15, 0.15, 0.40,
              0.15, 0.15, 0.40, 3.0, 0.55]),
    np.array([2.0, 2.0, 1.5, 0.08, 0.20, 0.08, 0.05, 0.20,
              0.08, 0.05, 0.20, 1.0, 0.5]),
]

DEG = np.pi / 180.0


def decode(genes: np.ndarray) -> tuple[dict, float, float]:
    """染色体 → (InnerLoop cfg, theta_trim_rad, throttle)。"""
    (angle_roll_kp, angle_pitch_kp, angle_yaw_kp,
     roll_rate_kp, roll_rate_ff,
     pitch_rate_kp, pitch_rate_ki, pitch_rate_ff,
     yaw_rate_kp, yaw_rate_ki, yaw_rate_ff,
     theta_trim_deg, throttle) = genes
    inner_cfg = {
        "dt": 0.02,
        "angle_roll_kp": angle_roll_kp,
        "angle_pitch_kp": angle_pitch_kp,
        "angle_yaw_kp": angle_yaw_kp,
        "p_limit_degps": 60.0,
        "q_limit_degps": 45.0,
        "r_limit_degps": 30.0,
        "roll": {"rate_kp": roll_rate_kp, "rate_ff": roll_rate_ff},
        "pitch": {"rate_kp": pitch_rate_kp, "rate_ki": pitch_rate_ki,
                  "rate_ff": pitch_rate_ff},
        "yaw": {"rate_kp": yaw_rate_kp, "rate_ki": yaw_rate_ki,
                "rate_ff": yaw_rate_ff},
        "coordinated_turn_ff_gain": 1.0,
        "v_trim_mps": 16.0,
        "beta_gain": 0.5,
    }
    return inner_cfg, theta_trim_deg * DEG, float(throttle)


def fast_observation(out) -> ControlObservation:
    """单输出快速构造观测（避免通用构造器的列表推导开销）。"""
    body = out.velocity_body_mps
    rates = out.rates_body_radps
    eul = out.euler_rpy_deg
    neu_pos = out.position_ne_alt_m
    neu_vel = out.velocity_ne_up_mps
    tas = float(np.sqrt(body[0] ** 2 + body[1] ** 2 + body[2] ** 2))
    return ControlObservation(
        u=np.array([body[0]]), v=np.array([body[1]]), w=np.array([body[2]]),
        p=np.array([rates[0]]), q=np.array([rates[1]]), r=np.array([rates[2]]),
        phi=np.array([eul[0]]) * DEG, theta=np.array([eul[1]]) * DEG,
        psi=np.array([eul[2]]) * DEG,
        alpha=np.array([out.alpha_rad]), beta=np.array([out.beta_rad]),
        neu_pos=np.array([neu_pos]), neu_vel=np.array([neu_vel]),
        tas=np.array([tas]),
    )


class AttitudeTrackingEvaluator:
    """串级内环姿态跟踪适应度评估（线程安全：每次 rollout 独立 model 实例）。"""

    def __init__(self, duration: float = 25.0, warmup: float = 5.0,
                 library_path: Optional[Path] = None):
        self.duration = duration
        self.warmup = warmup
        self.dt = 0.02
        self.library = AvantiLibrary(library_path)

    def __call__(self, genes: np.ndarray) -> float:
        """并行评估入口：单个染色体 → 适应度（越小越好）。"""
        cost, _ = self.rollout(genes)
        return cost

    def phi_cmd_at(self, t: float) -> float:
        """两阶段剧本：0~8 s 平飞，8 s 后 ±15° 方波（每 5 s 切换）。"""
        if t < 8.0:
            return 0.0
        return 15.0 * DEG if int((t - 8.0) // 5.0) % 2 == 0 else -15.0 * DEG

    def rollout(self, genes: np.ndarray) -> tuple[float, dict]:
        """运行姿态跟踪剧本，返回 (代价, 轨迹日志)。"""
        inner_cfg, theta_trim, throttle = decode(genes)
        inner = InnerLoop(inner_cfg)
        inner.reset(1)

        ticks = int(self.duration / self.dt)
        fields = ["t", "phi", "theta", "psi", "p", "q", "r",
                  "phi_cmd", "theta_cmd", "delta_a", "delta_e", "delta_r",
                  "alt", "alpha", "tas"]
        log = {key: [] for key in fields}

        inputs = AvantiInput(throttle_01=throttle)
        nan_seen = False
        with self.library.model(epoch_s=0.0) as model:
            output = model.tick(inputs)
            for k in range(ticks):
                t = k * self.dt
                obs = fast_observation(output)
                phi_cmd = self.phi_cmd_at(t)
                theta_cmd = theta_trim
                command = inner.step(
                    np.array([phi_cmd]), np.array([theta_cmd]),
                    np.array([0.0]), obs,
                )
                command.throttle = np.array([throttle])

                log["t"].append(t)
                log["phi"].append(obs.phi[0])
                log["theta"].append(obs.theta[0])
                log["psi"].append(obs.psi[0])
                log["p"].append(obs.p[0])
                log["q"].append(obs.q[0])
                log["r"].append(obs.r[0])
                log["phi_cmd"].append(phi_cmd)
                log["theta_cmd"].append(theta_cmd)
                log["delta_a"].append(command.delta_a[0])
                log["delta_e"].append(command.delta_e[0])
                log["delta_r"].append(command.delta_r[0])
                log["alt"].append(obs.neu_pos[0, 2])
                log["alpha"].append(obs.alpha[0])
                log["tas"].append(obs.tas[0])

                inputs = AvantiInput(throttle_01=throttle)
                inputs.delta_e_norm = float(command.delta_e[0])
                inputs.delta_a_norm = float(command.delta_a[0])
                inputs.delta_r_norm = float(command.delta_r[0])
                output = model.tick(inputs)
                if not output.finite():
                    nan_seen = True
                    break

        arrays = {key: np.array(value) for key, value in log.items()}
        cost = self.cost_from_log(arrays, nan_seen)
        return cost, arrays

    def cost_from_log(self, log: dict, nan_seen: bool) -> float:
        if nan_seen or len(log["t"]) < 10:
            return 1e6
        mask = log["t"] >= self.warmup
        if not mask.any():
            return 1e6

        phi_err = wrap_pi(log["phi"] - log["phi_cmd"]) / DEG
        theta_err = (log["theta"] - log["theta_cmd"]) / DEG
        psi_err = wrap_pi(log["psi"]) / DEG  # psi_cmd = 0
        rate_mag = (np.abs(log["p"]) + np.abs(log["q"]) + np.abs(log["r"])) / DEG

        cost = (1.0 * np.mean(np.abs(phi_err[mask]))
                + 1.0 * np.mean(np.abs(theta_err[mask]))
                + 0.4 * np.mean(np.abs(psi_err[mask]))
                + 0.1 * np.mean(rate_mag[mask]))

        # 超限惩罚
        if np.max(np.abs(log["phi"])) > 80.0 * DEG:
            cost += 1500.0
        if np.max(np.abs(log["theta"])) > 60.0 * DEG:
            cost += 1500.0
        if np.min(log["alt"]) < 0.0:
            cost += 2000.0
        if np.max(np.abs(log["alpha"])) > 0.5:
            cost += 800.0
        if np.max(log["tas"]) > 28.0:  # 超速俯冲惩罚
            cost += 1000.0
        if np.min(log["tas"]) < 6.0:   # 失速惩罚
            cost += 1000.0
        tail = log["t"] >= self.duration - 2.0
        if np.mean(np.abs(phi_err[tail])) > 20.0:
            cost += 300.0
        return float(cost)
