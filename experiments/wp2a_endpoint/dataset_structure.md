# avanti-eso 动作-轨迹获能数据集结构说明（wp2a_endpoint）

`schema_version = avanti-eso-wp2a-endpoint-v0.1`，2026-11-04。

每条 run 是一个 `.npz`（`data/eso_<subject>_<wind>.npz`）加一个同名 `.meta.json`。
run 粒度 = 一个科目 × 一个风场的 300 s 连续闭环飞行（50 Hz，T=15000 tick），
即 docs/TESO-Diff_phase2.md §16 的"任务航线场景 S × 风场矩阵 W"一个单元。
12 条 run：{figure8, loiter_climb, racetrack, gps_triangle} × {W0, W1, W2}。

本数据集是 Phase 2a 的**验证级落盘格式**（npz 内存镜像），通道命名已按
§31 HDF5 层级预分组（`time/ truth/ state/ action/ energy/`）；Phase 2b
WP2B-20 writer 落 HDF5 时按下表映射，不改变语义。

## 通道表

形状 `[T]` 标量、`[T,3]` 向量，T=15000。全部 float64、SI 单位、无 NaN/Inf。

| npz 通道 | 形状 | 单位/坐标系 | 语义 | §31 对应 |
|---|---|---|---|---|
| `time/t` | [T] | s | 全局仿真时间（plant tick 采样时刻，严格 0.02 s 步进） | `/history/timestamp` |
| `time/t_rel` | [T] | s | 相对时间（run 起点为 0；动作先后顺序通道） | `/history/relative_time` |
| `time/path_progress_m` | [T] | m | 沿航线单调弧长进度（含圈数累计） | `/history/relative_time` 辅助 |
| `time/lap_count` | [T] | - | 已完成圈数 | episode 分段辅助 |
| `truth/wind_flat` | [T,3] | m/s，flat-Earth Xe/Ye/Ze（Ze 向下为正） | 采样位置处的风场真值 | `/truth/wind_truth` |
| `truth/wind_applied_flat` | [T,3] | 同上 | 该 tick 实际送入 plant 的风（取自上一 tick 观测位置，一拍输运延迟） | `/truth/wind_truth` |
| `truth/wind_neu` | [T,3] | m/s，NEU（Up 为正） | `wind_applied` 的 NEU 表示 | 同上（调试视图） |
| `state/pos_neu` | [T,3] | m，NEU | North/East/Altitude | `/raw/neu` |
| `state/vel_neu` | [T,3] | m/s，NEU | 地速矢量 | `/raw/neu` |
| `state/euler_rad` | [T,3] | rad | phi/theta/psi（NEU 参考） | `/raw/neu`（deg→rad 已转换） |
| `state/rates_body_radps` | [T,3] | rad/s，FRD 机体系 | p/q/r | `/raw/body` |
| `state/tas_mps` | [T] | m/s | 真空速 = \|地速 − 风\|（已知风） | `/history/obs` |
| `state/ias_est_mps` | [T] | m/s | IAS ≈ TAS·√(ISA 密度比) | `/history/obs` |
| `state/alpha_rad` `state/beta_rad` | [T] | rad | 迎角/侧滑角 | `/raw/alpha` `/raw/beta` |
| `state/chi_rad` `state/gamma_rad` | [T] | rad | 实际地面航迹偏角/倾角 | `/history/obs` |
| `action/phi_cmd_rad` `action/theta_cmd_rad` | [T] | rad | 外环输出姿态指令（送生成内环） | `/history/action` |
| `action/throttle_cmd` | [T] | [0,1] | 油门指令（cT 通道真值来源） | `/history/action` |
| `action/chi_cmd_rad` `action/gamma_cmd_rad` | [T] | rad | 期望空速航向 / 期望航迹倾角 | `/history/controller_intermediates` |
| `action/altitude_cmd_m` | [T] | m | TECS 独立高度参考（爬升科目为积分斜坡） | `/history/controller_intermediates` |
| `action/ias_cmd_mps` `action/tas_cmd_mps` | [T] | m/s | 空速指令（IAS 原值 / TAS 换算） | `/history/controller_intermediates` |
| `action/path_cross_track_m` | [T] | m | 路径带符号横侧偏差（follower 投影） | `/history/controller_intermediates` |
| `action/path_lookahead_m` | [T] | m | 前视距离 | 同上 |
| `action/path_target_neu` | [T,3] | m，NEU | 前视虚拟目标点 | 同上 |
| `energy/e_a_jpkg` | [T] | J/kg | 空气相对比机械能 e_a = ½·TAS² + g·h | `/history/energy_segment` 原料 |
| `energy/edot_a_wpkg` | [T] | W/kg | e_a 数值微分（np.gradient，未滤波） | 同上 |
| `energy/p_env_up_wpkg` | [T] | W/kg | 静态上升气流项 g·w_up | `/label/Y_E` 真值原料 |
| `energy/p_env_wind_rate_wpkg` | [T] | W/kg | 风梯度项 −v_air·Ẇ（动态滑翔项） | 同上 |
| `energy/p_env_total_wpkg` | [T] | W/kg | 环境比功率 p_env = g·w_up − v_air·Ẇ | 同上 |
| `energy/E_env_cum_jpkg` | [T] | J/kg | ∫p_env dt 累计环境比能 | 同上 |

## 能量通道的物理依据与边界

由 e_a = ½·v_air² + g·h 全微分可得严格恒等式：

```text
ė_a = (P_thrust − P_drag)/m + g·w_up − v_air·Ẇ
                              └───── p_env（环境项，由风真值解析导出）─────┘
```

当前 ABI 无气动力/推力遥测，因此数据集保存 **p_env 环境项真值**（风真值
解析可得，W0 时严格为 0），而不虚构 (P_thrust−P_drag) 分解；docs §25 的
Y_E（扣阻力/发动机的轨迹级标签）留待 Phase 2b 能量核算模块（WP2B-17）。
`Ẇ` 为沿航迹全导数，由 `truth/wind_applied_flat` 的 NEU 表示按 dt=0.02 s
数值微分；静态风场下等于 (v_ground·∇)W。

## meta.json 字段

`schema_version / run / subject / scenario(S1~S4) / route_params /
gamma_cmd_rad / wind{wind_id, params} / dt_s / ticks / frames（坐标系冻结
说明）/ known_wind_assumption / wind_transport_delay_ticks=1 / validation
（该 run 的全部校验结果，与 metrics/summary.json 同源）`。

## 读取示例

```python
import numpy as np
with np.load("data/eso_figure8_W1.npz") as d:
    t_rel = d["time/t_rel"]            # [T] 相对时间
    pos = d["state/pos_neu"]           # [T,3]
    act = np.stack([d["action/phi_cmd_rad"], d["action/theta_cmd_rad"],
                    d["action/throttle_cmd"]], axis=1)   # [T,3] 动作
    p_env = d["energy/p_env_total_wpkg"]                 # [T] 获能功率真值
```

人读样本片段见 [data/sample_preview.txt](data/sample_preview.txt)
（figure8 × W1 热气流穿越段：w_up 升至 1.96 m/s 时 p_env → 19.2 W/kg，
油门同步从 0.24 收到 0，即 TECS 以怠速置换环境获能）。

## 与通用层的关系（docs §50.3）

`state/* + action/* + energy/*` 构成"观测—动作—获能"三元组，不绑定
TESO-Diff 特有结构；`/anchor`、`/query`（counterfactual 层）在本阶段
不生成，待 Phase 2b snapshot/branch 模块。相对时间已按双通道保存
（`time/t` 与 `time/t_rel`/`path_progress`），满足 §27 时间双通道要求。
