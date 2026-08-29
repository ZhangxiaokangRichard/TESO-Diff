# TESO-Diff Phase 2 工程工作计划
## ——从 AVANTI2 6DOF `.so` / Python 张量并行控制器 到 Isaac Sim / Isaac Lab 并行数据集生成

**文档版本：** v1.1  
**阶段名称：** TESO-Diff Phase 2  
**目标系统：** 固定翼自主获能滑翔 Trajectory-conditioned Energy Space Observer with Diffusion（TESO-Diff）  
**主要开发环境：**
- Phase 2a：Ubuntu 22.04，无 Isaac Sim
- Phase 2b：数据/编码服务器，Ubuntu + Isaac Sim / Isaac Lab
- 当前动力学参考模型：AVANTI2 非线性 6DOF，Simulink/Embedded Coder 生成
- 当前动力学运行形式：共享库 `.so`（phase1 已冻结 C ABI，并有 Python ctypes 封装 `python/avanti.py`）
- 后续控制器运行形式：Python 张量并行控制器类（外环 TECS + 协调转弯律 + L1 位置控制，内环 PID 姿态控制），不单独编译控制器 `.so`
- 数据生成目标：并行、可复现、可分层、可扩展、支持 counterfactual branching 的 TESO-Diff 训练/验证/测试数据

---

# 0. 文档目的

Phase 2 的任务不是开始训练最终 TESO-Diff 网络，而是完成 **“可规模化生成可信 TESO 数据的仿真基础设施”**。

Phase 2 结束时，应形成以下闭环：

```text
                       ┌───────────────────────────────┐
                       │      Environment / Wind       │
                       │ W_truth(p,t,env_id,seed)      │
                       └──────────────┬────────────────┘
                                      │
                                      ▼
┌──────────────────┐       ┌──────────────────────────┐
│ TESO Query /     │       │   AVANTI2 6DOF Plant    │
│ Primitive Sampler├──────►│   reference .so / GPU   │
└────────┬─────────┘       └────────────┬─────────────┘
         │                              │
         ▼                              ▼
┌──────────────────┐       ┌──────────────────────────┐
│ Attitude /       │──────►│ Aircraft state / truth   │
│ Flight Controller│       │ NEU, BODY, α, β          │
│ (Python tensor)  │       └────────────┬─────────────┘
└──────────────────┘                    │
                                      ▼
                       ┌───────────────────────────────┐
                       │ Sensor / estimator emulation  │
                       │ GNSS/IMU/Pitot/Wind estimate  │
                       └──────────────┬────────────────┘
                                      │
                                      ▼
                       ┌───────────────────────────────┐
                       │   TESO history construction   │
                       │ H_t / segment / confidence    │
                       └──────────────┬────────────────┘
                                      │
                                      ▼
                       ┌───────────────────────────────┐
                       │ Counterfactual branch rollout │
                       │ Nq feasible primitives        │
                       └──────────────┬────────────────┘
                                      │
                                      ▼
                       ┌───────────────────────────────┐
                       │ Energy-space labels / dataset │
                       │ Y_E, feasible, unsafe, truth  │
                       └───────────────────────────────┘
```

本阶段重点是：

1. **把动力学模型从 Simulink 开发环境转化为 Ubuntu 可复用的运行时模型；**
2. **完成 Python 张量并行的姿态/飞行内环控制器（TECS + 协调转弯 + L1 外环、PID 内环）；**
3. **在无 Isaac Sim 的 Ubuntu 22.04 上完成 3 条封闭航迹任务的闭环飞行、小规模并行与 avanti_eso 张量数据收集验证；**
4. **在 Isaac Lab 中建立 Direct workflow 并行环境；**
5. **建立风场、传感器、估计误差、动作 primitive、能量标签、counterfactual rollout、数据写盘等模块；**
6. **生成从最简单的无风数据到复合非定常风场的规范数据集；**
7. **形成长时序训练数据、独立验证集、OOD 测试集；**
8. **为 Phase 3 的 TESO Encoder / Query Decoder / Diffusion 正式训练做好全部数据和接口准备；**
9. **冻结数据—模型接口**：数据结构与编解码接口（history token 化输入、位置/动作/时序查询解码输出）必须能直接对接 Phase 3 的 TESO-Diff 模型，并兼容其他获能画像 RL 算法；
10. **提供 Flask 可视化配置与数据生成控制台**，使训练脚本的启动与参数选择在本地浏览器完成。

---

# 1. Phase 2 的核心技术原则

## 1.1 保留非线性 6DOF 作为物理参考模型

当前已经完成 Simulink 非线性 6DOF 模型并初步完成构建测试，因此 Phase 2 不再回退到“只使用线性状态空间模型”作为唯一仿真对象。

推荐模型层级：

```text
F0：Simulink 原始非线性模型
       ↓ 数值参考
F1：Simulink Generated C++ / .so
       ↓ Ubuntu reference backend
F2：Torch / Warp GPU batch backend（如果吞吐需要）
       ↓ 正式大规模数据生成
```

其中：

- `.so` 是 **数值参考实现和第一版仿真 backend**；
- 是否需要 Warp/Torch GPU 重写，由 Phase 2b benchmark 决定；
- 不允许在没有数值回归测试的情况下直接用 GPU 重写版本替换 `.so`。

## 1.2 训练阶段不采用 ROS2 服务逐环境调用

Phase 2 的大规模仿真链路不使用：

```text
Isaac env
   ↓ ROS2 service
6DOF
```

ROS2 留到 SIL、HIL、PX4/飞控集成、部署系统、实时软件架构验证。Phase 2 训练环境优先使用进程内 Python/C++ 调用或者 CUDA tensor / Warp kernel。

## 1.3 6DOF `.so` 是 CPU 库，不自动等于 GPU batch

必须始终保持这一认识：

```text
.so 成功
≠
可以高效处理 4096 个 GPU 环境
```

Phase 2b 必须实测单实例 step latency、8/32/128/512 实例吞吐、Python→C ABI 调用成本、多线程扩展性、GPU↔CPU 数据搬运成本。

根据 benchmark 决定正式数据生成 backend：

### Backend A — CPU `.so`
适合数值验证、少量并行、smoke test、高保真校准和一部分 reference 数据。

### Backend B — Warp / Torch CUDA
适合数百至数千并行环境、counterfactual branching、百万级以上数据生成。

两者必须保持同一状态、输入、风场、时间定义。

## 1.4 控制器用 Python 张量类实现，不编译控制器 `.so`

phase1 已证明 6DOF 模型可从 Python（ctypes）稳定声明类并逐 tick 调用，因此控制器**不需要**再走 C++ 编译并行路线。控制器采用适合并行计算的 Python 类（内部全部张量化，可对 batch 实例一次性解算）：

```text
期望空间位置（航迹任务）
        ↓
外环：TECS（总能量控制）+ 协调转弯律 + L1 位置控制
        ↓  期望航向/航迹倾角/油门/期望姿态角
内环：PID 姿态控制（phi/theta/psi → δa/δe/δr）
        ↓
AvantiInput（7 通道）→ 6DOF `.so`
```

控制器解算出的期望航向、油门、期望姿态角、航迹误差等**中间量全部作为 avanti_eso（扩张状态观测器）编码器输入的状态张量**保存。张量化意味着同一份控制器代码在 Phase 2a 小规模并行与 Phase 2b Isaac/CUDA 后端均可复用，避免重复实现两套控制律。

---

# 2. 当前已知系统接口基线

当前 AVANTI2 6DOF 顶层模型：`AVANTI2_ertmodel0827`。phase1 已冻结稳定 C ABI（`interface/avanti_c_api.h`，ABI 版本 `0x00010000`），本节以它为准。

## 2.1 输入，共 7 个（`AvantiInput`，56 bytes）

```text
delta_e_norm      升降舵归一化指令，建议 [-1,1]，映射约 [-12,+12] deg
delta_a_norm      副翼归一化指令，建议 [-1,1]，映射约 [-10,+10] deg
delta_r_norm      方向舵归一化指令，建议 [-1,1]，映射约 [-22,+22] deg
throttle_01       油门/推力指令，模型饱和到 [0,1]
wind_flat_earth_mps[0..2]  flat-Earth 轴风速 X_e / Y_e / Z_e，m/s
```

必须冻结单位、正负号、舵面正方向、油门范围、风速所属坐标系和角度单位。注意三个舵面输入是**归一化指令，不是弧度**。风速所属 flat-Earth 轴（Aerospace Blockset 约定 `Z_e` 向下为正），Isaac 世界坐标接入前必须完成世界轴 → flat-Earth 轴的显式映射。建议所有运行时库内部统一 SI：m、s、m/s、rad、rad/s、kg、N、N·m。

## 2.2 输出，共 4 组（`AvantiOutput`，192 bytes）

```text
alpha           迎角，rad
beta            侧滑角，rad
NEU（全局）:
    position_ne_alt_m[3]      North=Xe、East=Ye、Altitude=-Ze，m —— 第 3 个分量向上为正
    velocity_ne_up_mps[3]     Ve_x、Ve_y、Up=-Ve_z，m/s —— 第 3 个分量向上为正
    euler_rpy_deg[3]          roll、pitch、yaw，deg（注意是度不是弧度）
BODY（机体系）:
    velocity_body_mps[3]      u, v, w，m/s
    rates_body_radps[3]       p, q, r，rad/s
    rates_body_derivative_radps2[3]   p_dot, q_dot, r_dot，rad/s²
    acceleration_body_mps2[3] u_dot, v_dot, w_dot，m/s²
```

**关键澄清：** 根端口名虽为 `NED`，但第 3 个位置/速度分量已在模型内取反为 Up-positive，C ABI 命名（`..._alt` / `..._up`）已作提示，即全局输出实际是 **NEU**。不要把它当作标准 NED 的 Down-positive 分量。

建议在 Python 封装中不要直接暴露生成结构体，而映射为命名的稳定数据结构，例如：

```python
class FWInput:    # 与 C ABI AvantiInput 一一对应
    delta_a: float    # 归一化 [-1,1]
    delta_e: float
    delta_r: float
    throttle: float   # [0,1]
    wind: list[float] # flat-Earth Xe/Ye/Ze，m/s

class FWNEU:      # 全局输出，Up-positive
    n, e, alt: float      # m
    phi, theta, psi: float  # deg

class FWBody:
    u, v, w: float        # m/s
    p, q, r: float        # rad/s

class FWOutput:
    alpha: float      # rad
    beta: float       # rad
    neu: FWNEU
    body: FWBody
```

Phase 2a 只在此冻结接口之上工作，不重新发明新的 C/C++ 结构。

---

# 3. 时间系统设计：Phase 2 必须首先冻结

## 3.1 Plant 固定积分时间步

phase1 已冻结 `dt_plant = 0.02 s`（fixed-step ODE4，50 Hz）。一次 `plant.step()` 必须严格定义为模型内部积分时间推进 `dt_plant`。外部程序不再对状态做额外积分。

## 3.2 Isaac 全局时间不直接替代 Plant 积分器

推荐：

```text
env_step_count = k
t_global = t_reset + k * dt_env
```

若 `dt_env == dt_plant`，每次 Isaac 环境 step 执行一次 wind query、controller update 和 plant.step()。

若 `dt_env = N * dt_plant`，则内部循环 N 个 plant step。Phase 2 不允许使用不可控墙钟时间驱动动力学。

## 3.3 推荐时间层级

初始建议：

```text
plant integration: 50 Hz, dt_plant = 0.02 s
inner controller: 50 Hz（与 plant 同频）或 25 Hz
outer controller / L1: 5~20 Hz
IMU: 100~200 Hz
Pitot: 50 Hz
GNSS: 10~20 Hz
FC wind estimate: 20~50 Hz
avanti_eso history update: 10~20 Hz
high-level primitive: 0.5~3 s 持续
```

最终值由控制器和数据需求决定。

## 3.4 Counterfactual 时间语义

一个 anchor 必须包含：

```text
state(t_anchor)
controller_internal_state(t_anchor)
actuator_state(t_anchor)
engine_state(t_anchor)
environment_id
environment_phase(t_anchor)
```

复制为 Nq 个 branch，每个 branch 从相同 `t_anchor` 开始，随后使用 `t_branch = t_anchor + j * dt_plant`。不能因为 branch 执行顺序不同而改变风场时间。

---

# 4. 坐标系标准

## 4.1 Plant 内部与 ABI 输出

- Plant 内部：Aerospace Blockset flat-Earth 约定，`X_e` 北、`Y_e` 东、`Z_e` 向下为正；机体系 FRD（Body x forward / y right / z down）。不要为了 TESO 强行改动经过验证的 6DOF 内部坐标。
- **ABI 输出（冻结）：全局输出是 NEU。** `position_ne_alt_m[2] = -Z_e`（Altitude 向上为正）、`velocity_ne_up_mps[2] = -V_e_z`（Up 向上为正）。Euler 输出为 deg。
- 风输入为 flat-Earth 轴（`Z_e` 向下为正），与全局输出坐标系不同，必须在 adapter 层显式区分。

## 4.2 Dataset 同时保存 Raw + Canonical

phase1 的输出已经是 Up-positive（NEU），**无需再做 NED→ENU 转换**。数据集 raw 直接保存：

```text
truth/neu/*      North、East、Altitude（m）、NEU 速度（m/s）、Euler（deg）
truth/body/*     u, v, w、p, q, r、p_dot…、u_dot…（SI）
alpha, beta      rad
```

canonical 层（供网络训练）另存局部 ENU + 航迹对齐坐标（按 PDF §3.2：`+z` 与重力反向一致、水平面绕竖直轴旋转使当前航迹方向为局部 `+x`）。姿态不通过逐角符号翻转实现，统一用 quaternion/DCM 做 frame transform，推荐训练数据优先保存 quaternion，同时保留 Euler 供调试。

---

# 5. Phase 2 总体分区

```text
Phase 2a
Ubuntu 22.04
无 Isaac Sim

目标：
.so + Python 张量控制器（TECS/L1/协调转弯 + PID）
3 条封闭航迹闭环飞行
小规模并行 + avanti_eso 张量数据收集
完整冒烟、数值、长时序、多实例测试

            ↓ Gate A

Phase 2b
Isaac Sim / Isaac Lab 编码机 + 数据服务器

目标：
并行环境
任务航线场景 × 风场矩阵
sensor/estimator
counterfactual branching
dataset generator
规范数据
长时序
test/OOD

            ↓ Gate B

Phase 3
TESO 网络正式训练
```

---

# 6. Phase 2a — Ubuntu 22.04 无 Isaac Sim 冒烟测试

## 6.1 Phase 2a 目标

必须在没有 Isaac Sim 的情况下证明：

1. 6DOF `.so` 可在 Ubuntu 正确编译、不依赖 MATLAB runtime；
2. `.so` 可创建/销毁多个独立 aircraft instance，Python（复用 phase1 `python/avanti.py`）可稳定调用；
3. 单步积分时间定义明确（`dt_plant = 0.02 s`），初始状态正确，wind input 正确；
4. **Python 张量并行控制器可闭环**：外环 TECS + 协调转弯律 + L1 位置控制、内环 PID 姿态控制；
5. **3 条封闭航迹任务闭环飞行通过**：盘旋、8 字盘旋、跑马场（GPS 三角留到 Phase 2b 数据集扩展）；
6. 小规模并行（1→8→32 实例）下闭环正确、无串扰；
7. **avanti_eso（扩张状态观测器）张量数据收集验证**：闭环飞行过程中连续记录风场真值、获能动作/控制器中间量、状态与时空位置（相对时间），形成编码器输入输出真值流；
8. 长时间运行不出现 NaN / 状态异常；reset 可重复；snapshot/clone 能满足 counterfactual；
9. benchmark 明确 `.so` + Python 控制器在小规模并行下的 steps/s、CPU、memory。

## 6.2 推荐 Phase 2a 项目目录

```text
TESO-Diff/
├── README.md
├── docs/
│   ├── TESO-Diff_phase2.md
│   ├── fixedwing_interface.md
│   ├── controller_interface.md
│   └── dataset_schema.md
├── config/
│   ├── aircraft/avanti2.yaml
│   ├── controller/controller.yaml          # TECS / L1 / 协调转弯 / PID 参数
│   ├── task/                                # 封闭航迹任务定义
│   │   ├── circle.yaml
│   │   ├── figure8.yaml
│   │   ├── racetrack.yaml
│   │   └── gps_triangle.yaml
│   ├── environment/
│   │   ├── no_wind.yaml
│   │   ├── thermal_only.yaml
│   │   ├── shear_only.yaml
│   │   ├── composite.yaml
│   │   └── wind_smoke.yaml
│   └── dataset/smoke.yaml
├── runtime/
│   ├── fixedwing/
│   │   ├── generated/
│   │   ├── wrapper/
│   │   ├── include/
│   │   └── CMakeLists.txt
│   └── common/
├── python/
│   ├── bindings/                            # 复用 phase1 python/avanti.py ctypes 封装
│   ├── controller/                          # 纯 Python 张量并行控制器（无 C++ 编译）
│   │   ├── tecs.py
│   │   ├── coordinated_turn.py
│   │   ├── l1.py
│   │   ├── pid.py
│   │   ├── outer.py
│   │   └── inner.py
│   ├── eso/                                 # avanti_eso 张量收集
│   │   ├── history.py
│   │   └── tensor_recorder.py
│   ├── simulation/
│   └── tests/
├── tests/
│   ├── cpp/
│   ├── reference/
│   └── regression/
├── isaac/
├── dataset/
│   ├── schema/
│   ├── smoke/
│   └── formal/
└── scripts/
    ├── build_all.sh
    ├── run_phase2a_smoke.sh
    ├── run_phase2a_tasks.py               # 3 条封闭航迹闭环 + eso 张量收集
    └── benchmark_runtime.py
```

## 6.3 WP2A-0：Ubuntu 工具链确认

确认：

```bash
lsb_release -a
gcc --version
g++ --version
cmake --version
python3 --version
```

检查：

```bash
ldd libfixedwing.so
```

目标是 `libfixedwing.so` 无需 MATLAB 即可运行。

## 6.4 WP2A-1：6DOF `.so` 单实例冒烟测试

最小测试：

```text
reset
↓
trim control
↓
wind = 0
↓
1000 step
↓
输出状态
```

验收：无 crash、无 NaN/Inf、alpha/beta 合理、trim 附近无快速发散、模拟时间正确、输出单位正确。

## 6.5 WP2A-2：时间步一致性测试

Python 端：

```python
for k in range(N):
    t = k * dt
    output = plant.step(input)
```

记录 `k`、`t_global`、`plant_internal_time`（若可读取），要求时间不随步数漂移。Isaac timestamp 不应直接覆盖 ODE solver 内部状态；它主要用于 wind field、environment motion、sensor timestamps 和 dataset timestamps。

## 6.6 WP2A-3：输入通道逐项测试

逐一激励：

```text
delta_a_norm pulse
delta_e_norm pulse
delta_r_norm pulse
throttle step
wind flat-Earth X_e step
wind flat-Earth Y_e step
wind flat-Earth Z_e step
```

形成 `control_sign_test.md`，记录每个正输入对应的实际响应方向。

## 6.7 WP2A-4：Python binding

phase1 已有 ctypes 封装 `python/avanti.py`（`AvantiLibrary`、`AvantiInput`、`AvantiOutput`、`library.model()` 上下文管理器），Phase 2a 直接复用并扩展，不重写。

最终接口建议：

```text
fw_create()
fw_destroy()
fw_reset(handle, reset_state)
fw_step(handle, input, output)
fw_get_snapshot()
fw_set_snapshot()
```

## 6.8 WP2A-5：多实例测试

创建 A/B/C 三个独立实例，给予不同输入，同时推进 10000 steps，确认无串扰。随后做 1、8、32、128、256 实例 benchmark，记录 steps/s、simulated seconds / wall second、CPU utilization、memory。

## 6.9 WP2A-6：reset / determinism

同一 seed、initial state、control sequence、wind sequence 运行两遍，要求 bitwise equal 或严格数值容差内一致。

## 6.10 WP2A-7：snapshot / clone

Counterfactual branching 至少需要复制：6DOF continuous state、quaternion/attitude internal state、actuator state、engine lag state、controller integrators、controller filters、sensor estimator state（如 branch 需要）以及 local simulation time。

理想 API：`fw_get_snapshot` / `fw_set_snapshot`。

若 generated class 暂时不方便暴露完整状态，Phase 2a 可用多实例 reset 作为临时方案，但 Phase 2b 正式 counterfactual 前必须解决高效 clone。

## 6.11 WP2A-8：Python 张量并行飞行控制器

控制器**不编译 `.so`**，用纯 Python 张量类实现（内部 numpy/torch 张量化，可对 batch 实例一次解算）。控制层级：

```text
封闭航迹任务（期望空间位置序列）
        ↓
外环：
  L1 位置控制       —— 由位置误差求期望横向加速度/期望航向 χ_cmd
  TECS 总能量控制   —— 由高度与空速误差解算油门 throttle_cmd 与俯仰能量指令
  协调转弯律        —— 由期望航向变化率求期望坡度 phi_cmd
        ↓
内环：
  PID 姿态控制      —— phi_cmd/theta_cmd/psi_cmd → δa/δe/δr 归一化指令
        ↓
AvantiInput（7 通道）→ 6DOF `.so`
```

外环输入为**期望空间位置**（由任务航线解算给出），输出为期望航向、期望航迹倾角/俯仰、期望坡度与油门；内环把期望姿态角转换为三个舵面归一化指令。第一阶段 primitive 至少支持 V_cmd、phi_cmd、gamma_cmd 或 theta_cmd、yaw_rate_cmd（可选）、throttle_mode/cT_cmd、duration。

控制器解算出的**全部中间量**（期望航向、航迹误差、油门、期望姿态角、舵面指令、协调转弯半径等）都作为 avanti_eso 编码器输入状态张量的一部分记录。

长期 TESO action 建议逐步靠近：

```text
a = [V_a*, gamma*, turn_rate*, gamma_rate*, cT, Tq]
```

而不是直接学习舵偏。

## 6.12 Python 控制器类接口建议

```python
class ControlReference:      # 任务/外环参考输入（batch 化：字段均为 [B] 或 [B,3]）
    airspeed_cmd: Tensor     # V_cmd
    altitude_cmd: Tensor     # h_cmd（TECS）
    path: Tensor             # 期望空间位置序列 [B, N_waypoints, 3]，NEU
    turn_rate_cmd: Tensor    # 协调转弯用，可选
    throttle_mode: str       # 自动(TECS)/手动

class ControlObservation:    # 来自 FWOutput 的机体系观测
    u, v, w: Tensor
    p, q, r: Tensor
    phi, theta, psi: Tensor  # deg→rad 在内部统一
    alpha, beta: Tensor

class ActuatorCommand:       # 控制器输出 = AvantiInput 的舵面/油门部分
    delta_a: Tensor          # 归一化 [-1,1]
    delta_e: Tensor
    delta_r: Tensor
    throttle: Tensor         # [0,1]

class AvantiController:      # 全张量化，可直接对 B 个实例并行
    def __init__(self, cfg): ...
    def reset(self, batch: int, state: Tensor): ...
    def step(self, ref: ControlReference, obs: ControlObservation) -> ActuatorCommand: ...
    def intermediates(self) -> dict[str, Tensor]: ...  # 编码器输入状态张量
    def get_snapshot(self) / def set_snapshot(self, snap): ...  # 积分器/滤波器状态
```

要求：

- `step()` 只做张量运算，无逐实例 Python 循环；
- `intermediates()` 每 tick 可导出期望航向、航迹误差、油门、期望姿态、舵面指令等中间张量；
- snapshot 必须覆盖 TECS 积分器、PID 积分/微分状态、L1 滤波器状态。

## 6.13 WP2A-9：闭环 Python Harness

```text
for each dt_plant:
    wind_truth = wind_stub.query(position, time)
    if controller_tick:
        ref = task_route.desired_position(time)      # 期望空间位置
        command = controller.step(ref, observation)  # 张量并行
    plant_input = command + wind_truth
    plant_output = plant.step(plant_input)
    observation = build_observation(plant_output)
    eso_recorder.append(time, wind_truth, command,
                        controller.intermediates(),
                        plant_output, task_id)        # avanti_eso 张量
    log(...)
```

这一层完全不依赖 Isaac，用于把 plant/controller 问题与 Isaac 接口问题隔离，同时验证 avanti_eso 张量流的完整性。

## 6.14 WP2A-10：闭环飞行冒烟矩阵

至少测试：

- T00：无风 + trim
- T01：无风 + bank command `0→15°→0`
- T02：`0→-15°→0`
- T03：airspeed command 变化
- T04：恒定水平风
- T05：恒定垂直风
- T06：step gust
- T07：30° bank sustained
- T08：30 min simulated flight
- T09：1 h simulated flight
- T10：随机可执行 primitive sequence
- T11：**盘旋封闭航迹**（无风/上升气流），N 圈不掉线
- T12：**8 字盘旋封闭航迹**（无风/上升气流）
- T13：**跑马场封闭航迹**（两直线 + 两半圆），含高度剖面
- T14：**小规模并行** 8 实例同时执行 3 条航迹，avanti_eso 张量完整、无串扰

## 6.15 Phase 2a 长时序稳定性

运行 10 min、30 min、1 h、3 h simulated time，监测 NaN、Inf、quaternion norm、speed、alpha、beta、altitude、actuator saturation、engine state、energy drift。无风、无动力、理想滑翔状态不得出现不合理“凭空获能”。封闭航迹任务长时间跟踪不得出现航迹误差发散。

## 6.16 Phase 2a 性能 Gate

至少输出：

| Backend | Env | steps/s | simulated×realtime | CPU | memory |
|---|---:|---:|---:|---:|---:|
| `.so` 裸模型 | 1 | | | | |
| `.so` 裸模型 | 8 | | | | |
| `.so` + Python 张量控制器 | 8 | | | | |
| `.so` + Python 张量控制器 | 32 | | | | |
| `.so` + Python 张量控制器 | 128 | | | | |
| `.so` + Python 张量控制器 | 512 | | | | |

如果 Nq branching × env_count 达到正式数据量时 CPU 端明显不足，则 Phase 2b 需要 Warp/Torch backend（控制器张量代码可直接平移）。

## 6.17 Phase 2a Gate A 验收标准

- [ ] Ubuntu `.so` build 成功
- [ ] 不依赖 MATLAB runtime
- [ ] 7 输入 / 4 输出映射冻结（含 NEU 输出、归一化舵面、flat-Earth 风输入）
- [ ] 单位冻结
- [ ] 坐标系冻结（全局输出 NEU / Up-positive）
- [ ] step 时间语义冻结（`dt_plant = 0.02 s`）
- [ ] reset 可重复
- [ ] 多实例无串扰
- [ ] Python 张量控制器闭环成功（TECS/L1/协调转弯 + PID）
- [ ] 3 条封闭航迹（盘旋 / 8 字 / 跑马场）闭环飞行通过
- [ ] avanti_eso 张量数据收集通过（风真值 / 动作 / 控制器中间量 / 状态 / 相对时间）
- [ ] 无风 trim smoke 通过
- [ ] 风输入 smoke 通过
- [ ] 30 min+ 长时序无 NaN
- [ ] snapshot/clone 方案明确
- [ ] benchmark 完成
- [ ] Python wrapper 测试通过
- [ ] dataset logger 原型通过

只有 Gate A 通过，才进入 Phase 2b。

---

# 7. Phase 2b — Isaac Sim / Isaac Lab 并行数据生成

## 7.1 Phase 2b 目标

在 Isaac 环境中完成：DirectRLEnv / Direct workflow、aircraft batch manager、wind truth、environment randomization、**Python 张量控制器（Phase 2a 冻结，直接平移）**、任务航线场景生成器、sensor model、FC wind estimate、history builder、primitive sampler、snapshot、counterfactual branches、energy label、data writer，以及按"任务航线场景 × 风场矩阵"组织的 smoke/formal/long-horizon/validation/test/OOD dataset。

## 7.2 Isaac Sim 在 Phase 2 的角色

Isaac 不负责“自动提供固定翼气动”。Isaac 负责 parallel environment management、CUDA tensor、scene/time management、visual debugging、randomization、dataset orchestration、optional terrain/obstacle/sensor simulation。AVANTI2 的 aero/propulsion/6DOF/controller 仍由项目自己的模型实现。

## 7.3 推荐 Isaac Lab 环境结构

```text
isaac/
├── avanti_teso/
│   ├── __init__.py
│   ├── env_cfg.py
│   ├── env.py
│   ├── backend/
│   │   ├── base.py
│   │   ├── so_backend.py
│   │   └── warp_backend.py
│   ├── wind/
│   │   ├── base.py
│   │   ├── no_wind.py
│   │   ├── uniform.py
│   │   ├── thermal.py
│   │   ├── shear.py
│   │   ├── gust.py
│   │   ├── dynamic.py
│   │   └── composite.py
│   ├── control/
│   │   ├── primitive.py
│   │   └── scheduler.py
│   ├── task/                          # 任务航线场景（投影面随机参数化）
│   │   ├── base.py
│   │   ├── circle.py                  # 盘旋上升/下降
│   │   ├── figure8.py                 # 8 字型盘旋上升/下降
│   │   ├── racetrack.py               # 跑马场形
│   │   ├── gps_triangle.py            # GPS 三角（3 圆角顶点 + 3 直线）
│   │   └── projector.py               # 投影面随机角度/坡度参数化
│   ├── sensors/
│   │   ├── gnss.py
│   │   ├── imu.py
│   │   ├── pitot.py
│   │   └── wind_estimator.py
│   ├── energy/
│   │   ├── accounting.py
│   │   └── label.py
│   ├── branch/
│   │   ├── snapshot.py
│   │   └── counterfactual.py
│   ├── dataset/
│   │   ├── schema.py
│   │   ├── writer.py
│   │   ├── shard.py
│   │   └── validate.py
│   └── tests/
└── scripts/
    ├── smoke_isaac.py
    ├── generate_dataset.py
    └── benchmark_env.py
```

## 7.4 Phase 2b 开发顺序

严格按 1 env → 8 env → 32 env → 128 env → 512 env → 1024+ 扩展。每一阶段重新检查数值、时间、reset、memory、吞吐。

## 7.5 Flask 可视化配置与数据生成控制台

**定位：** Phase 2a 结束后、Phase 2b 并行测试（32/512 env benchmark 与 counterfactual 正确性）通过后启动。训练过程是并行、参数化的，因此把训练脚本的启动与参数选择设计为一个本地 Flask 项目，在浏览器中完成数据生成任务的配置、启动与监控，方便后续数据训练。

**功能：**

- **可视化配置**：浏览器表单中配置任务场景（S1~S4 及形状参数、投影面角度区间）、风场矩阵（W0~W3 及参数域）、扰动/阵风参数、数据集族（D0~D8）、规模（anchors × branches）、seed 范围与 shard 策略，生成并校验 YAML；
- **任务管理**：启动 / 暂停 / 续跑数据生成任务（对接 §45 日志与 §46 checkpoint/resume），实时展示进度、throughput 与 shard 状态；
- **质量看板**：展示每个 shard 的 §37 质量指标、manifest 摘要、轨迹可视化预览（2D/3D 航迹 + 风场叠加）。

**边界：** Flask 只负责参数编排与任务调度，不进入数值路径；生成核心仍是 Isaac/backend 脚本，保证可复现性（同一 YAML 命令行重放 bitwise 一致）。

目录建议：

```text
server/
├── app.py                    # Flask 入口
├── tasks/
│   ├── config_forms.py       # 表单 → YAML 校验
│   ├── runner.py             # 后台启动/暂停/续跑
│   └── monitor.py            # shard 状态与质量指标
├── templates/                # 页面
└── static/                   # 轨迹可视化
```

**验收：** 在本地浏览器完成一次 smoke 数据集（S1 × W0/W1）的配置、启动、监控与下载；生成结果与命令行重放一致。

---

# 8. WP2B-0：Isaac 环境版本冻结

在数据服务器记录 Ubuntu、NVIDIA driver、CUDA、Isaac Sim version、Isaac Lab commit/tag、PyTorch、Warp、Python、GPU，生成 `environment.lock.md`。正式数据集生成周期不得随意升级 Isaac/CUDA。

---

# 9. WP2B-1：1-env Isaac smoke

第一阶段不做视觉。环境只负责 step/reset/clock/plant backend/controller backend。检查 Isaac t、Python t、plant t 三者一致。

---

# 10. WP2B-2：CPU `.so` backend

实现：

```python
class SoFixedWingBackend:
    reset(...)
    step(...)
    get_state(...)
    snapshot(...)
    restore(...)
```

这样上层环境完全不知道 backend 是 `.so`、Warp 还是 Torch。

---

# 11. WP2B-3：并行 tensor contract

即使第一版 `.so` 是 CPU，Isaac 上层接口也应按 batch 设计：

```text
state   [B, Nx]
control [B, 4]
wind    [B, 3]
time    [B]
done    [B]
```

输出：`alpha [B]`、`beta [B]`、`neu [B,...]`（North/East/Altitude，Up-positive）、`body [B,6]`。不要把长期接口设计成 Python 的 per-env step 循环。

---

# 12. WP2B-4：是否迁移 GPU 的判定

运行 32/128/512 env，并加入 Nq=16、32 的 counterfactual equivalent workload。如果 CPU `.so` 不能达到目标 throughput，实现 `WarpFixedWingBackend` 或 `TorchFixedWingBackend`。

GPU implementation 必须通过 randomized one-step regression、10 s rollout、trim regression、gust regression，与 `.so` 对比。

---

# 13. Wind Field Architecture

风场接口统一：

```text
W_truth = wind.query(position, time, env_parameters)
```

输入 `position [B,3]`、`time [B]`，输出 `wind_truth [B,3]`。未来可扩展 air_density、temperature、turbulence latent。TESO 不直接看到 `W_truth`。

---

# 14. Privileged Truth 与 TESO Observation 分离

## Simulator truth

保存 exact aircraft state、W_truth、environment parameters、aero force、propulsive force、environment work、mechanism id、thermal center、shear layer geometry 等。

## TESO observation

只能使用未来实机可获得/可估计的信息：GNSS、IMU、Pitot TAS、attitude estimate、FC wind estimate、throttle、control command、sensor health、aircraft token。

必须保证 `W_hat_FC != W_truth`，不能把 simulator truth wind 直接送进 TESO history。

---

# 15. FC Wind Estimate 模型

第一版可以用：truth wind → low-pass → bias → correlated noise → delay → dropout。参数随机化 bias、lag、noise、sample rate、packet drop。后续可替换为真实飞控 estimator。

---

# 16. 数据集总体架构：任务航线场景 × 风场矩阵

Phase 2b 数据集不再只按风场机制切分，而是按 **“任务航线场景（flight task scenario）× 风场矩阵（wind matrix）”** 的二维结构组织。每条数据都产生于一条参数化的封闭航迹任务闭环飞行，这样才能收集到**完整的编码器输入输出真值**：风场环境、获能动作（primitive 与控制器中间量）、飞机状态、时空位置（时间按 PDF 的相对时间/age 处理后体现动作先后顺序对获能的影响）。

```text
任务航线场景 S × 风场矩阵 W
├── S1 盘旋上升/下降        ├── W0 无风
├── S2 8 字盘旋上升/下降    ├── W1 仅上升气流（热气流）
├── S3 跑马场形              ├── W2 仅梯度切变风
├── S4 GPS 三角              └── W3 复合（上升气流 + 梯度切变）
│                           每个 W 均可叠加扰动参数：
└── 形状投影面随机角度/坡度   ├── 环境扰动风（背景风、湍流）
                             └── 阵风 gust（参数化）
```

## 16.1 任务航线场景参数化（投影面随机旋转）

四种封闭航迹形状先在“形状局部平面”上定义：

- **S1 盘旋上升/下降**：圆/螺旋，半径、圈数、爬升率/下降率；
- **S2 8 字型盘旋上升/下降**：两圆相切 8 字，两圆半径、旋转方向、总爬升/下降梯度；
- **S3 跑马场形**：两段直线 + 两个半圆，直线长度、半圆半径、高度剖面；
- **S4 GPS 三角**：三个圆角顶点 + 三条直线，边长、圆角半径、各边爬升/下降坡度。

每个形状的投影面在空间中与坐标系参数化随机角度：以局部平面绕竖直/水平轴旋转（roll/pitch of pattern plane）+ 整体坡度/漂移方向随机化，使同一形状产生任意空间朝向、任意爬升/下降方向的航迹族。航迹点随后交给控制器作为**期望空间位置序列**。随机量必须进入 manifest 并可按 seed 重放。

## 16.2 风场矩阵

- **W0 无风**：物理零获能基线，验证 energy label 与控制器本身；
- **W1 仅上升气流**：热气流/上升带（中心、半径、峰值 Wz、形状、漂移），无水平剪切；
- **W2 仅梯度切变风**：水平/垂直切变层（ΔV、厚度、方向、高度），无上升气流；
- **W3 复合**：W1 + W2 同时存在，机制参数独立随机，保存 mechanism truth。

**扰动参数在所有 W0~W3 上叠加**（不单独成族）：

- 环境扰动风：背景水平风（0~12 m/s）、低频湍流/随机扰动；
- 阵风 gust：脉冲阵风、阵风锋、三轴 gust（1~8 m/s），作为参数化选项开启。

这样每个“场景 × 风场 × 扰动参数”组合都生成一批 episode；同一场景在四种风场下重复，使编码器能学到“同一条任务航线下不同环境的表现差异”，并显式记录动作先后顺序（相对时间）对获能的影响。

## 16.3 编码器输入输出真值收集

每个环境每个 tick 必须保存完整真值对：

| 类别 | 内容 |
|---|---|
| 风场环境真值 | `W_truth(p,t)`、mechanism id、机制参数、扰动/gust 参数 |
| 获能动作 | 任务 primitive、控制器中间量（期望航向/航迹误差/油门/期望姿态/舵面指令） |
| 状态 | NEU 位置/速度、body u,v,w,p,q,r、alpha/beta、Euler、quaternion、能量量 |
| 时空位置 | 全局位置 + 相对时间（segment age、anchor 相对时间），处理后体现动作先后顺序 |
| 能量标签 | `Y_E`、`feasible`、`unsafe`、`terminal`、`tracking_error`、`P_unsafe` |

---

# 17. 基线数据集（物理与控制器检验）

## D0 — 无风 / 物理零获能基线

环境 `W=0`。内容：各任务场景 S1~S4 下 unpowered glide、powered level、turn、climb/descent、random feasible primitives。目标是验证无环境能输入时环境获能标签接近 0。Powered 情况必须扣除发动机能量。

## D0b — Uniform Wind Baseline

`W=constant`，例如水平风 0~12 m/s。目的：验证恒定均匀风本身不会被错误识别为可持续环境获能结构，排除 ground-speed change 被误当成 environment energy gain。

## D1 — 飞行动作可行性 / Controller Envelope

环境 no wind / weak wind。在 S1~S4 航迹族上扫描 V_cmd、bank_cmd、gamma_cmd、yaw_rate_cmd、duration，以及投影面随机角度极限值。标签 tracking_error、actuator_saturation、stall、overspeed、unsafe、feasible。用于构造状态相关可执行 primitive 集 A(s)，并确定各场景投影面参数的可飞范围。

---

# 18. D2 — 静态获能数据集（场景 × 静态风矩阵）

“静态”指 wind field 不显式随时间变化，但可以随空间变化。**核心结构：S1~S4 × {W0, W1, W2, W3} × 扰动参数，全部叠加投影面随机化。**

## D2-W1 Thermal

推荐第一版参数域：center、radius 30~120 m、peak vertical speed 0.5~5 m/s、shape type、vertical extent、background wind。可采用 Gaussian、Wharington-like、core+edge 等模型。目标不是一次得到真实大气精确模型，而是覆盖单机制 Energy Space；同一航迹族在无热气流/有热气流的对比数据同时保留。

## D2-W2 Shear

静态 horizontal/vertical shear、tanh layer、piecewise layer。初始参数可取 ΔV 2~15 m/s、thickness 5~80 m、layer orientation random、layer altitude random。目标是支持 dynamic-soaring-type energy extraction 的基本空间结构。

## D2-W3 Composite

thermal+shear 组合。机制参数独立随机、机制真值分开保存；任务场景与投影面同样随机。用于验证单机制证据叠加时标签的可分解性。

## D2-Vertical Structure

updraft corridor、downdraft region、ridge-like synthetic lift、localized vertical jet。没有真实地形模型时必须标记为 synthetic mechanism。

---

# 19. D3 — 动态获能画像数据集（场景 × 动态风矩阵）

风场显式依赖 t，即 W(p,t)。同样按 S1~S4 × 动态风 × 扰动参数组织。

## D3-Moving Thermal

thermal center drift、strength variation、radius evolution、birth/decay。初始参数可取 drift 0~8 m/s、lifetime 20~300 s，并随机 strength/radius modulation。

## D3-Moving Shear

layer translation、layer rotation、gradient strength change。

## D3-Gust

localized gust、temporal gust、gust front、3-axis gust，建议覆盖约 1~8 m/s，并另保留 OOD 更强扰动。作为扰动参数叠加在静态/动态风场上时，必须单独记录 gust 真值通道。

## D3-Turbulent / Stochastic

第一版可使用 band-limited stochastic wind、correlated random field、Dryden-like temporal disturbance，主要用于训练 uncertainty、trend、confidence，而不是作为最终真实湍流模型。

---

# 20. D4 — 复合风场数据集

组合 thermal+background wind、thermal+shear、thermal+gust、shear+gust、moving thermal+moving shear、thermal+shear+stochastic 等，任务场景仍为 S1~S4。正式 TESO-Diff diffusion 多模态训练主要依赖此类数据。建议每个环境包含 2~4 mechanism，并保存 mechanism truth。

---

# 21. D5 — Sensor / Estimator Domain Randomization

对 GNSS、IMU、Pitot、attitude、wind estimate、throttle signal 加入 bias、noise、delay、rate mismatch、dropout、quantization，用于训练 c_confidence 和真实飞行鲁棒性。随机化叠加于所有场景/风场数据集之上。

---

# 22. D6 — Aircraft Domain Randomization

随机 mass、inertia、CG、S、b、c、aero coefficients、polar、servo dynamics、engine lag、prop efficiency。初始范围建议 nominal ±5~10%，之后逐步扩展。TESO 输入 aircraft token。

---

# 23. D7 — OOD / Held-out Test

不是随机从训练分布抽样，而是明确保留未见区域。例如训练 thermal radius 30~90，test 90~130；训练 shear ΔV 2~10，test 10~16；训练组合 thermal+gust / shear+gust，测试 thermal+shear+gust。**任务场景侧同样保留 OOD：训练用部分投影面角度区间，测试用未见的更大坡度/旋转角、以及 GPS 三角等训练外场景。** 也应包括 unseen sensor bias、unseen aircraft parameters、unseen mechanism composition。

---

# 24. D8 — 高保真场 / 后续真实飞行

Phase 2 先预留接口。未来 CFD、LES、WRF、measured field、flight logs 统一转换为 `wind.query(p,t)`，避免修改 TESO pipeline。真实航迹数据同样挂接任务场景标注。

---

# 25. 能量标签体系

核心空气相对比机械能：

```text
e_a = 0.5 * V_TAS^2 + g h
```

轨迹级环境能量：

```text
Y_E = Δe_a + ∫(p_D + p_ctrl)dt - ∫p_eng dt
```

其中 `p_D = D * V_TAS / m`，发动机比功 `p_eng = V_TAS * a_eng`。发动机贡献不得进入 environmental energy。

---

# 26. Energy Label 必须执行的单元测试

- E0：无风 + 无动力，drag compensation 后 `Y_E ≈ 0`
- E1：无风 + 有动力，扣除 engine 后 `Y_E ≈ 0`
- E2：只改变 throttle，不得产生“环境获能”
- E3：静态 thermal，进入上升气流时 `Y_E > 0` 合理
- E4：downdraft 时 `Y_E < 0`
- E5：simulator direct environment work 与能量平衡结果一致
- E6：dt 与 dt/2 标签收敛

---

# 27. History / avanti_eso 编码器数据构造

建议 history 包含 position history（NEU）、attitude（Euler+quaternion）、body velocity、TAS、alpha/beta、IMU、estimated wind、throttle、actuator command、**控制器中间量（期望航向/航迹误差/油门/期望姿态角，来自 Phase 2a 张量控制器）**、**任务场景 id 与投影面参数**、segment energy、sensor health。

时间一律保存双通道：全局时间（对齐 Isaac/风场时间）+ **相对时间**（segment age、anchor 相对时间）。相对时间经过处理后用于体现动作先后顺序对获能的影响（同一场景下先做哪个动作、后做哪个动作的收益差异）。

trajectory segment 建议 1.5 s、3 s primary、6 s；例如 3 s window、1 s hop。

---

# 28. 归一化参数

建议 `V_ref = best-glide speed`，并定义：

```text
T_ref = V_ref / g
L_ref = V_ref^2 / g
E_ref = V_ref^2
```

数据集应保存 raw SI，normalized 最好在 loader 中统一处理；不要只保存 normalized，以免未来无法重定义 normalization。

---

# 29. Counterfactual Query Dataset

这是 TESO-Diff 最关键的数据生成环节。

## 29.1 Anchor

连续正常飞行生成 history，在 `t_anchor` 记录 snapshot。

## 29.2 Sample Nq primitives

建议 Nq=8/16/32/64。Query 可写为 `q=(Δr, Δt, s, a, Tq)`，动作 primitive 采用 V_cmd、gamma_cmd、turn_rate_cmd/bank_cmd、gamma_rate_cmd、throttle_cmd 等。

## 29.3 Expand state

逻辑上：

```text
[B,Nx]
  ↓ expand
[B,Nq,Nx]
  ↓ flatten
[B*Nq,Nx]
```

每个 branch 使用相同 environment truth、anchor time、aircraft snapshot，只改变 primitive。

## 29.4 Rollout

持续 Tq，例如 1.5 s、3 s、6 s，计算 Y_E_gt、feasible、unsafe、tracking error、minimum margin。

## 29.5 Branch output

```text
query        [B,Nq,Dq]
Y_E_gt       [B,Nq]
feasible     [B,Nq]
unsafe       [B,Nq]
terminal     [B,Nq]
```

---

# 30. Dataset Schema

推荐第一版采用 HDF5 shard，因为 Python 成熟、支持 chunk/compression、易检查和独立 shard。后续如果并发写入压力大，再评估 Zarr。

## 30.1 数据集族定义总表

| 数据集族 | 任务场景 | 风场 | 扰动参数 | 主要产出 | 用途 |
|---|---|---|---|---|---|
| D0 | S1~S4 | W0 无风 | — | 零获能基线 episode、`Y_E≈0` 验证样本 | 能量核算单元测试、编码器基线 |
| D0b | S1~S4 | 均匀背景风 0~12 m/s | — | 均匀风基线样本 | 排除地速变化伪获能 |
| D1 | S1~S4 + 投影面扫描 | 无风 / 弱风 | — | tracking_error、saturation、feasible/unsafe | 可执行 primitive 集 A(s)、投影面可飞域 |
| D2 | S1~S4 | W0/W1/W2/W3 静态 | 可选 | 静态 Energy Space 标签 + 真值流 | Query Decoder 第一版训练 |
| D3 | S1~S4 | 动态 W(p,t) | 可选 | trend / nonstationary 标签 | history 编码器趋势分支 |
| D4 | S1~S4 | 2~4 机制复合 | 可选 | 多模态获能画像 | diffusion 多模态训练 |
| D5 | 全部 | 任意 | 传感器随机化 | 有噪机载观测 + 真值 | c_confidence、鲁棒性 |
| D6 | 全部 | 任意 | 机型随机化 | 跨机型样本 | aircraft token 泛化 |
| D7 | 留出的场景/投影面/参数区 | OOD 区域 | OOD | 未见分布样本 | ID/OOD test，生成后冻结 |
| long-sequence | S1~S4 多圈/多边 | 任意 | 任意 | 60 s ~ 600 s+ 连续飞行 | history encoder、trend、temporal OOD |

说明：D 族与“任务航线场景 × 风场矩阵”是正交的两个切分维度——D 族描述**数据集用途/阶段**，S×W 描述**单条数据的生成条件**；同一份 D2 数据内部按 S1~S4 × W0~W3 全覆盖组织。

---

# 31. 建议 HDF5 层级

```text
/meta
    schema_version
    generator_commit
    aircraft_config_hash
    controller_config_hash
    task_scenario_id            # S1~S4 及形状参数
    projector_params            # 投影面随机角度/坡度
    wind_config_hash
    wind_matrix_class           # W0/W1/W2/W3
    disturbance_params          # 扰动风 + 阵风参数
    isaac_version
    seed
    dt_plant
    dt_sensor
    coordinate_frame            # NEU / flat-Earth / body，明确约定

/episode
    episode_id
    env_id
    task_scenario_id
    mechanism_id
    start_time
    length

/history
    obs
    action                       # primitive + 舵面指令
    controller_intermediates     # 期望航向/航迹误差/油门/期望姿态角
    wind_est
    sensor_health
    energy_segment
    timestamp                    # 全局时间
    relative_time                # segment age / anchor 相对时间

/anchor
    state_truth
    timestamp
    relative_time
    aircraft_token
    environment_token
    task_scenario_id

/query
    primitive
    horizon
    target

/label
    Y_E
    feasible
    unsafe
    terminal
    tracking_error

/truth
    wind_truth
    mechanism_params
    aero_force
    prop_force
    environment_work
    environment_params

/raw
    neu                          # North/East/Altitude，Up-positive
    body
    alpha
    beta
```

## 31.1 HDF5 字段明细表（对接 Phase 3 编解码接口）

| HDF5 路径 | shape（示意） | 语义 | Phase 3 接口角色 |
|---|---|---|---|
| `/history/obs` | `[T, D_obs]` | 机载观测（20 Hz 重采样） | Trajectory Encoder 输入（token 化前的原始序列） |
| `/history/action` | `[T, D_act]` | primitive + 舵面/油门 | Encoder 动作通道 |
| `/history/controller_intermediates` | `[T, D_ctrl]` | 期望航向/航迹误差/油门/期望姿态 | Encoder 状态张量（附加通道） |
| `/history/energy_segment` | `[N_seg, D_seg]` | 每片段 `Y_E`、`σ_E` | token `z_k` 的显式通道 |
| `/history/timestamp` | `[T]` | 全局时间 | 时间对齐（对齐风场/传感器） |
| `/history/relative_time` | `[T]` / `[N_seg]` | segment age、anchor 相对时间 | 动作先后顺序特征 |
| `/anchor/state_truth` | `[N_a, Nx]` | 反事实起点状态 | counterfactual 起点 |
| `/query/primitive` | `[N_a, Nq, Dq]` | 候选 primitive `q` | Query Decoder 查询输入 |
| `/query/horizon` | `[N_a, Nq]` | 查询时长 `Tq` | 查询时长条件 |
| `/label/Y_E` | `[N_a, Nq]` | 环境获能真值 | 监督标签（式 17） |
| `/label/{feasible,unsafe,terminal}` | `[N_a, Nq]` | 安全/可执行标签 | 约束与校准标签 |
| `/truth/wind_truth` | `[T, 3]` | 隐藏风场真值 | 仅 debug/辅助监督，不进部署观测 |
| `/truth/mechanism_params` | `[D_mech]` | 机制参数 | `Z_E^gt` Autoencoder 训练 |
| `/raw/neu` | `[T, 9]` | NEU 位置/速度/Euler | raw SI 保存，供其他算法/再归一化 |
| `/raw/body` | `[T, 12]` | 机体系 u,v,w / p,q,r / 导数 | raw SI 保存 |

约定：raw 层与 label 层是**通用接口**，不绑定 TESO-Diff 特有结构；`/anchor`、`/query`、`controller_intermediates` 是 TESO-Diff 的**附加层**，其他获能画像 RL 算法可忽略。

---

# 32. 数据分片策略

不要生成单个巨大文件。建议：

```text
dataset_D2/
├── shard_000000.h5
├── shard_000001.h5
├── ...
├── index.jsonl
└── manifest.json
```

单 shard 可从 0.5~4 GB 起步，按实际存储系统 benchmark 调整。

---

# 33. Manifest

必须记录 dataset_name、schema_version、creation_date、git commit、number episodes、number anchors、number branches、mechanism distribution、parameter ranges、seed range、checksum。TESO 模型训练结果必须能够追溯到具体 manifest。

---

# 34. 数据集生成阶段计划

## Phase 2b-S0 — Isaac smoke dataset

1~8 env，<10 min wall time，内容 S1 盘旋 × {W0 no wind / W1 thermal}，只验证 schema 和 writer。

## Phase 2b-S1 — Physics sanity dataset

包含 D0/D0b（S1~S4 场景 × 无风/均匀风），验证 energy labels 与控制器闭环。

## Phase 2b-S2 — Primitive feasibility dataset

内容 D1，覆盖 S1~S4 与投影面参数扫描，用于动作范围与 controller envelope。

## Phase 2b-S3 — Static Energy Space dataset

内容 D2，即 S1~S4 × {W0, W1, W2, W3} × 扰动参数；目标是训练/验证 Query Decoder 第一版；强调单机制、参数覆盖均匀、强可解释性。

## Phase 2b-S4 — Dynamic Energy Portrait dataset

内容 D3，使 history 表示开始包含 trend/nonstationarity。

## Phase 2b-S5 — Composite dataset

内容 D4，用于 diffusion multimodality。

## Phase 2b-S6 — Randomization dataset

内容 D5 + D6。

## Phase 2b-S7 — Long-history dataset

专门生成 60 s、120 s、300 s、600 s 甚至更长连续飞行（同一任务场景多圈/多边），用于 history encoder、trend estimator、confidence、environment transition、temporal OOD。

## Phase 2b-S8 — Test / OOD

内容 D7（含任务场景 OOD：未见投影面角度区间与未见场景）。生成后立即冻结，不得参与 normalization fitting、hyperparameter selection 或 training。

---

# 35. 推荐的正式规模控制方法

不要一开始规定“必须一亿条”。按有效 anchor 数量 × query branches 衡量。例如 100,000 anchors × 32 queries = 3.2M counterfactual labels，已经是很大的第一版 Energy Space dataset。

建议阶段：smoke 10^2~10^3 anchors；small 10^4；medium 10^5；formal 根据训练收敛和覆盖评估扩大到 10^6+。规模由 coverage、model error、storage、throughput 共同决定。

---

# 36. 数据质量检查

每个 shard 生成后自动执行 NaN count、Inf count、range、histogram、energy consistency、time monotonicity、coordinate consistency、query validity、feasible ratio、unsafe ratio。禁止先生成数 TB 再检查。

---

# 37. 自动质量指标

每 shard 输出类似：

```json
{
  "num_samples": 0,
  "nan_count": 0,
  "energy_mean": 0,
  "energy_std": 0,
  "positive_ratio": 0,
  "feasible_ratio": 0,
  "unsafe_ratio": 0,
  "stall_ratio": 0,
  "overspeed_ratio": 0
}
```

---

# 38. 物理异常 watchdog

在线判定 V 过小/过大、|alpha| 超限、|beta| 超限、|pqr| 极端、altitude invalid、quaternion norm invalid、NaN 等。异常 branch 应标记 terminal=true、unsafe=true，而不是简单删除，因为 unsafe label 本身有价值。

---

# 39. Train / Validation / Test Split

禁止仅随机按 row split。应该按 environment seed、environment parameter region、aircraft parameter、mechanism composition 拆分。

- Train：覆盖主分布
- Validation：同类机制，不同 seed + 部分不同参数
- Test-ID：训练分布内但完全独立 seed
- Test-OOD：训练之外的 parameter / combination / aircraft / sensor fault

---

# 40. 数据集版本

建议 `TESO-D0-v0.1`、`TESO-D1-v0.1`、`TESO-D2-v0.1` 等。任何 energy label、wind equation、controller、aircraft model、sensor model、schema 的改变必须升版本。

---

# 41. Phase 2b 性能优化顺序

严格按：正确性 → 减少 Python per-env loop → batch memory layout → counterfactual flatten → CPU parallel `.so` → Warp/Torch GPU → asynchronous dataset writer → compression tuning。

---

# 42. GPU Counterfactual 目标结构

正式高吞吐 backend 最终建议：

```text
state       [B,Nx]      CUDA
snapshot    [B,Nx]
expand      [B,Nq,Nx]
flatten     [B*Nq,Nx]
wind query
controller
plant
energy
全部 CUDA resident
reduce over horizon
reshape     [B,Nq]
```

这是 TESO-Diff 相比单轨迹数据生成最重要的计算优化。

---

# 43. CPU `.so` 与 GPU backend 的数值回归

随机生成 10,000 states / controls / winds，比较 1-step、10-step、100-step 的 alpha、beta、NEU、BODY、energy，设置容差。若长 horizon divergence 来自积分差异，应优先检查短时 derivative/one-step、一致的统计分布和 energy label。

---

# 44. 推荐配置系统

所有实验使用 YAML，例如：

```yaml
experiment:
  name: D2_circle_thermal_v01
  seed: 10001

simulation:
  dt_plant: 0.02
  num_envs: 512

aircraft:
  config: avanti2_nominal

controller:
  config: tecs_l1_pid_v01
  outer_rate: 20        # Hz，外环更新频率
  inner_rate: 50        # Hz，与 plant 同频

task:
  type: circle           # circle | figure8 | racetrack | gps_triangle
  radius: [40, 90]       # m
  climb_rate: [-1.5, 1.5]  # m/s，正为盘旋上升
  projector:
    roll_deg: [-15, 15]
    pitch_deg: [-10, 10]
    azimuth_deg: [0, 360]

wind:
  matrix_class: W1       # W0 none | W1 thermal | W2 shear | W3 composite
  type: thermal
  radius: [30, 120]
  strength: [0.5, 5.0]
  disturbance:
    background_wind: [0, 6]   # m/s
    turbulence: off
    gust: off

dataset:
  anchors: 100000
  branches: 32
  horizons: [1.5, 3.0, 6.0]
```

---

# 45. 日志

每次生成任务保存 stdout.log、config.yaml、git_commit.txt、manifest.partial.json、metrics.json。即使任务中断也能恢复。

---

# 46. Checkpoint / Resume

正式数据任务必须支持 resume from shard N。Seed 分配必须 shard-independent，例如 `seed = base_seed + shard_id * stride + local_env`。

---

# 47. Phase 2b 编码任务分解

建议工单：

```text
WP2B-00  Isaac 版本与环境锁定
WP2B-01  1-env DirectRLEnv skeleton
WP2B-02  .so backend integration
WP2B-03  batch interface
WP2B-04  clock/reset
WP2B-05  wind base API
WP2B-06  no wind / uniform wind
WP2B-07  thermal
WP2B-08  shear
WP2B-09  dynamic wind + gust/扰动参数
WP2B-10  Python 张量控制器 integration（Phase 2a 冻结版本平移）
WP2B-11  task scenario base（形状局部平面定义）
WP2B-12  projector：投影面随机角度/坡度参数化
WP2B-13  circle / figure8 / racetrack / gps_triangle 场景
WP2B-14  sensor emulation
WP2B-15  wind estimator
WP2B-16  history buffer（含控制器中间量、相对时间）
WP2B-17  energy accounting
WP2B-18  snapshot
WP2B-19  counterfactual branch
WP2B-20  HDF5 writer
WP2B-21  dataset validator
WP2B-22  32-env smoke
WP2B-23  512-env benchmark
WP2B-24  GPU backend decision
WP2B-25  D0/D0b（场景 × 无风/均匀风）
WP2B-26  D1（controller envelope + 投影面可飞范围）
WP2B-27  D2（场景 × W0~W3 静态风矩阵）
WP2B-28  D3（场景 × 动态风）
WP2B-29  D4（复合风）
WP2B-30  D5/D6
WP2B-31  long sequence
WP2B-32  test/OOD（含场景/投影面 OOD）
WP2B-33  dataset freeze
WP2B-34  Flask 可视化配置与数据生成控制台（配置/任务管理/质量看板）
```

---

# 48. Phase 2b 每个数据集的 Exit Criteria

一个 dataset family 只有满足以下条件才视为完成：

- [ ] generator config 已冻结
- [ ] schema version 已冻结
- [ ] physics sanity test 通过
- [ ] 无 NaN / Inf
- [ ] energy label unit test 通过
- [ ] parameter coverage 检查通过（含任务场景/投影面/风场矩阵/扰动参数维度）
- [ ] train/val/test seed 隔离
- [ ] manifest 完整
- [ ] shard checksum 完整
- [ ] 随机抽取 100 个 episode 可重放
- [ ] dataset validator 全绿
- [ ] 至少一次人工轨迹可视化检查

---

# 49. 关键风险表

| 风险 | 症状 | 对策 |
|---|---|---|
| `.so` CPU 太慢 | env 扩展后吞吐不增长 | Warp/Torch backend |
| Python per-env loop | CPU 100% GPU 空闲 | batch API |
| Simulink 多实例非 reentrant | 多 aircraft 串状态 | C++ class/state audit |
| reset 不完整 | episode 初始状态漂移 | snapshot/reset 全状态 |
| controller 不能大坡度跟踪 | primitive unsafe 比例高 | gain scheduling / 投影面可飞范围限制 |
| 控制器张量化引入数值差异 | 与标量控制器结果不一致 | 逐 tick 标量/张量对拍回归 |
| 风 truth 泄漏进 TESO | 训练效果异常好 | truth/obs strict separation |
| engine power 被当环境能 | throttle 与 Y_E 强相关 | energy unit tests |
| NEU/flat-Earth/body 三套坐标系混淆 | 热气流方向错误、高度符号反 | frame adapter + unit test，禁止隐式转换 |
| 投影面随机化超出可飞包线 | 大量 branch unsafe | D1 先冻结可飞参数域 |
| 相对时间丢失 | 无法学习动作先后顺序影响 | history 双时间通道强制校验 |
| Isaac 时间与 plant 时间漂移 | 动态风 label 错误 | integer timestep scheduler |
| 数据量过大但覆盖差 | 模型泛化差 | coverage first（场景×风场矩阵×扰动参数） |
| 单一 mechanism 过拟合 | composite 失败 | D4 curriculum |
| 数据 writer 成瓶颈 | GPU 等 I/O | async writer/sharding |
| OOD 泄漏 | test 失去意义 | seed/parameter lock |

---

# 50. Phase 2 与 Phase 3 的边界

## 50.1 Phase 3 的模型构成与最终输出

Phase 3 训练的 TESO-Diff 模型由三部分构成：

```text
编码器（Trajectory / History Encoder）  —— 把 HDF5 的 history 数据 token 化
    ↓  CTESO / trajectory-energy tokens
条件潜空间 Diffusion                    —— 生成 Energy Affordance latent Z_E 后验
    ↓  K 个 latent 样本
解码器（Energy Query Decoder）          —— 按查询输出
```

解码器的最终输出不是“风速图”，而是一个**推理矩阵**：以滑翔机当前时刻位置为中心、指定空间分辨率的网格上，条件于**位置、动作、时序**的潜在获能区域分布（`µE/σE/p+` 场或 latent grid `Z_E`）。Phase 2 不训练该模型，但数据与接口必须为其服务。

## 50.2 Phase 2 必须交付的数据—模型接口

1. **token 化输入接口**：`/history` 各通道 → 分段（3 s 窗口 / 1 s hop）→ `X*_k` → trajectory-energy token `z_k`。Phase 2 交付固定字段、固定采样率（20 Hz）与分段约定；
2. **查询解码接口**：`q = (Δr, Δt, s, a, Tq)` → `Y_E` 分布。数据集中 `/query` 与 `/label` 一一对应，保证 Phase 3 Query Decoder 可直接训练；
3. **latent 真值约定**：Phase 2 同时保存生成 `Z_E^gt` 所需的真值（风场参数、反事实 rollout、direct environment work），供 Phase 3 训练 Energy Autoencoder；
4. **归一化约定**：raw SI 永久保存，归一化（`V_ref / L_ref / T_ref / E_ref`）在 loader 统一完成，不在生成端固化。

## 50.3 兼容其他获能画像 RL 算法的设计

数据集的 raw 层（`/raw/*`）与标签层（`/label/*`）不绑定 TESO-Diff 特有结构：其他获能画像 RL 算法可直接读取“观测—动作—获能标签”三元组，无需理解 token/diffusion 约定；TESO-Diff 特有的中间结构（`/anchor`、`/query`、`controller_intermediates`）作为附加层，不阻塞通用读取。D 族/S×W 参数化定义也为这些算法提供可复用的试验场景与基准数据集。

Phase 2 不要求最终 Diffusion 模型达到效果。Phase 2 结束时必须提供 loader-ready 的 TESO dataset artifacts、冻结的 schema 与上述编解码接口约定。Phase 3 才开始 Trajectory Encoder → Energy AE → Query Decoder → Conditional latent diffusion → distillation → RL integration。

---

# 51. Phase 2 最终交付物

## Runtime

```text
libavanti2_6dof.so
Python bindings（复用/扩展 phase1 python/avanti.py）
Python 张量控制器包（TECS / 协调转弯 / L1 + PID，batch 并行）
avanti_eso 张量收集模块
```

如最终采用 GPU，再增加 Warp/Torch fixedwing backend。

## Isaac

```text
AvantiTESOEnv
task scenario modules（circle / figure8 / racetrack / gps_triangle + projector）
wind modules
sensor modules
branch engine
energy label engine
dataset writer
Flask 数据生成控制台（可视化配置 / 任务管理 / 质量看板）
```

## Dataset

至少：D0 no-wind zero-energy、D0b uniform wind baseline、D1 primitive feasibility（含投影面可飞域）、D2 场景×静态风矩阵 Energy Space、D3 场景×动态风 Energy Portrait、D4 场景×复合风、D5 sensor randomization、D6 aircraft randomization、D7 OOD/test（含场景/投影面 OOD）、long-sequence dataset。所有数据集带任务场景标注与编码器真值流（风场、动作、状态、时空相对位置）。

## Tests

C++ unit tests、Python integration tests、控制器标量/张量对拍测试、Isaac smoke tests、energy label tests、coordinate tests（NEU/flat-Earth/body）、time synchronization tests、backend regression tests、dataset validation tests。

## Documents

fixedwing_interface.md、controller_interface.md、task_scenario.md、wind_interface.md、dataset_schema.md（含 §30.1/§31.1 表格）、encoder_decoder_interface.md（Phase 2→Phase 3 编解码接口约定）、environment.lock.md、dataset manifests、benchmark report。

---

# 52. Phase 2a → Phase 2b 推荐实际执行顺序

```text
[Phase 2a]

1. Ubuntu compile AVANTI2 .so
2. C/Python API smoke（复用 phase1 python/avanti.py）
3. time-step verification（dt_plant = 0.02 s）
4. multi-instance
5. reset determinism
6. Python 张量控制器：内环 PID → 外环 TECS/协调转弯/L1
7. 3 条封闭航迹闭环（盘旋 / 8 字 / 跑马场）
8. wind stub
9. avanti_eso 张量收集验证
10. long-run
11. benchmark
12. snapshot design

        ↓ Gate A

[Phase 2b]

13. Isaac version lock
14. DirectRLEnv skeleton
15. 1 env .so
16. 8 env
17. no-wind smoke
18. wind base + 扰动/gust 参数
19. task scenario base + projector（投影面随机化）
20. circle / figure8 / racetrack / gps_triangle
21. controller integration（Phase 2a 版本平移）
22. thermal
23. shear
24. energy label
25. history（含控制器中间量、相对时间）
26. snapshot
27. counterfactual
28. writer
29. D0 / D0b
30. D1
31. D2（场景 × 静态风矩阵）
32. dynamic wind
33. D3
34. composite
35. D4
36. sensor/aircraft randomization
37. long sequence
38. OOD/test（含场景/投影面 OOD）
39. dataset freeze
40. Flask 可视化配置与数据生成控制台
```

---

# 53. Phase 2 最终 Gate B

进入 Phase 3 前必须回答“是”：

### Physics

- [ ] 6DOF reference stable?
- [ ] controller stable?
- [ ] wind sign correct?
- [ ] energy accounting correct?
- [ ] powered/unpowered labels correct?
- [ ] NEU / flat-Earth / body 坐标三套约定正确且无隐式混用?

### Simulation

- [ ] deterministic reset?
- [ ] exact time synchronization?
- [ ] multi-env stable?
- [ ] counterfactual clone correct?
- [ ] backend regression passed?
- [ ] 任务场景/投影面随机化可重放?

### Dataset

- [ ] schema frozen?
- [ ] no-wind baseline available?
- [ ] static energy dataset available（场景 × 静态风矩阵）?
- [ ] dynamic energy dataset available（场景 × 动态风）?
- [ ] composite dataset available?
- [ ] 编码器真值流完整（风场/动作/状态/相对时间）?
- [ ] long sequence available?
- [ ] ID test frozen?
- [ ] OOD test frozen（含场景/投影面 OOD）?
- [ ] 数据—模型接口冻结（token 化输入 / query 解码 / latent 真值 / 归一化约定）?
- [ ] 通用层（raw/label）可被其他获能 RL 算法直接读取?

### Performance

- [ ] target throughput measured?
- [ ] CPU/GPU backend decision complete?
- [ ] storage throughput sufficient?
- [ ] shard/restart system tested?
- [ ] Flask 控制台完成 smoke 数据生成验收?

全部通过后，Phase 2 完成。

---

# 54. 当前最近的下一步

依据目前工程进度，最短执行链为：

```text
A. Ubuntu 22.04 编译当前 AVANTI2 generated C++ → .so
B. 完成 create / reset / step / destroy 与 Python 调用（复用 phase1 封装）
C. 验证 trim / wind / pulse / 30 min
D. 实现 Python 张量控制器：PID 内环 + TECS/协调转弯/L1 外环
E. 3 条封闭航迹闭环：controller(张量) + plant.so + wind_stub
F. avanti_eso 张量收集 + 小规模并行（8~32 实例）
G. 完成 multi-instance + timing + snapshot 方案
H. 冻结 Phase 2a interface
I. 转移到 Isaac 数据服务器
J. 只做 1-env DirectRLEnv
K. 任务航线场景 × 风场矩阵，从 D0 no-wind dataset 开始
L. Phase 2b 并行测试通过后，搭建 Flask 可视化配置与数据生成控制台
```

在这条链完成以前，不建议提前编写复杂 diffusion 训练代码。

---

# 55. 总结

Phase 2 的工程核心不是“把飞机放进 Isaac Sim 里飞起来”，而是建立一个能够回答以下问题的可信数据生成系统：

```text
给定过去一段真实可观测飞行历史 H_t，
以及当前环境中一组可执行的未来轨迹 primitive，

不同 primitive 在未来 Tq 时间内
分别可以从环境中获得多少能量？
可信度如何？
是否可执行？
是否安全？
```

因此 Phase 2 的真正最终产物是：

```text
任务航线场景（投影面随机化的封闭航迹任务）
×
风场矩阵（无风/仅上升/仅切变/复合 + 扰动与阵风参数）
↓
history + query primitive + counterfactual rollout
+ energy posterior supervision
```

而 AVANTI2 6DOF、Python 张量控制器（TECS/协调转弯/L1 + PID）、avanti_eso、Isaac Lab 和风场模块都是构建这一监督信号所需的仿真基础设施。

工程上应始终坚持：

```text
reference physics first
→ deterministic runtime
→ parallel execution
→ counterfactual branching
→ validated dataset
→ neural training
```

不要为了并行速度跳过物理回归，也不要为了高保真让 `.so` CPU 调用成为正式数据生成的永久瓶颈。Phase 2b 的 benchmark 将决定最终正式数据生产是继续使用 `.so`，还是把已经冻结并验证过的模型方程迁移到 Warp/Torch CUDA。
