# sim_for_IsaacWorkflow.md

# 6-DOF固定翼动力滑翔机模型与Isaac Lab并行数据生成方案

## 1. 目标

建立TESO-Diff第一阶段数据生成平台：

6-DOF Fixed Wing Simulator + Inner Loop Controller + Isaac Lab Direct
Workflow

输出：

Flight History H + Counterfactual Action Query Q + Energy Ground Truth
Y_E

用于训练：

p(Energy Space \| Flight History)

TESO-Diff论文设计中，WP1要求首先建立6-DOF、风能场、传感器模拟和反事实rollout生成器。训练阶段应先保证能量核算、动作执行和数据覆盖，而不是立即追求最高保真CFD。

------------------------------------------------------------------------

# 2. 软件架构

推荐：

Isaac Lab Vectorized Environment

    Isaac Lab
     |
     +-- FixedWing Environment
     |
     +-- Wind Field Generator
     |
     +-- Sensor Simulator
     |
     +-- Energy Label Generator
     |
     +-- Controller Interface

开发阶段：

ROS2 Service作为模块接口。

最终训练阶段：

ROS2接口转换为GPU Tensor Batch API。

原因：

大规模并行环境中，DDS通信会成为瓶颈。

------------------------------------------------------------------------

# 3. 六自由度固定翼模型

状态：

X=\[x,y,z,u,v,w,phi,theta,psi,p,q,r\]

其中：

位置： (x,y,z)

机体系速度： (u,v,w)

姿态： (phi,theta,psi)

角速度： (p,q,r)

动力学：

m(V_dot+w×V)=F_a+F_g+F_T

Iω_dot+ω×Iω=M

------------------------------------------------------------------------

# 4. 气动模型

采用：

气动数据库 + 插值模型。

输入：

alpha,beta,V,h

输出：

CL,CD,CY,Cm,Cl,Cn

Simulink阶段：

建立：

Atmosphere → Aerodynamics → 6DOF Equation → Actuator → Sensor

完成：

-   Trim配平
-   Linearization
-   State Space Extraction

得到：

纵向模型：

\[u,w,q,theta\]

横侧模型：

\[v,p,r,phi\]

用于控制律设计。

------------------------------------------------------------------------

# 5. 控制架构

TESO采样的动作不直接使用舵面，而使用高层动作primitive。

    TESO Action

    (bank,pitch,airspeed,duration)

            ↓

    Reference Generator

            ↓

    Attitude Controller

            ↓

    Servo

            ↓

    6DOF Aircraft

------------------------------------------------------------------------

# 6. 内环姿态控制

## Pitch

theta_cmd → elevator

## Roll

phi_cmd → aileron

## Yaw

psi_cmd → rudder

控制器：

-   PID
-   LQR
-   Incremental nonlinear control

均可替换。

------------------------------------------------------------------------

# 7. Isaac Lab并行设计

每个环境：

    Aircraft state
    Wind state
    Controller state
    Sensor state
    Energy state

全部GPU tensor化。

RTX3090建议：

初始：

128\~512并行环境。

------------------------------------------------------------------------

# 8. 数据采样流程

Episode:

1.  初始化飞机参数和环境

2.  产生动作primitive

3.  Controller执行

4.  6DOF rollout

5.  计算Energy Label

记录：

History:

-   GNSS
-   TAS
-   IMU
-   attitude
-   angular rate
-   wind estimate
-   throttle

标签：

Y_E

包含：

-   kinetic energy change
-   potential energy change
-   drag loss
-   engine contribution

------------------------------------------------------------------------

# 9. 多类型仿真实验计划

## A 无风有动力

验证发动机分离：

无环境收益。

## B 热气流

随机：

-   thermal radius
-   updraft velocity
-   drift velocity

## C 风切变动态滑翔

随机：

-   gradient
-   shear height
-   direction

## D 湍流

加入：

Dryden gust model。

## E 跨机型

随机：

-   mass
-   wing area
-   aerodynamic polar
-   thrust

用于aircraft token学习。

------------------------------------------------------------------------

# 10. 数据格式

    {
    history,
    aircraft_profile,
    wind_truth,
    wind_est,
    trajectory,
    YE,
    query_action,
    query_energy_GT
    }

对应：

(H_t,Q_t\^gt,A_t)

------------------------------------------------------------------------

# 11. Fidelity路线

F1:

解析风场 + 6DOF

百万级样本。

F2:

复杂时变风场：

LES/CFD/WRF切片。

F3:

真实飞行数据微调。

TESO-Diff强调多simulator fidelity，避免latent只学习单一风场模板。

------------------------------------------------------------------------

# 12. 开发路线

Milestone 1:

Simulink：

-   6DOF
-   气动
-   配平
-   线性化

Milestone 2:

完成：

-   姿态控制
-   primitive动作接口

Milestone 3:

转换：

Simulink → C++/Python → Isaac Lab

Milestone 4:

GPU并行采样。

最终：

    Action Primitive
          |
    Controller
          |
    6DOF Aircraft
          |
    Wind Interaction
          |
    Trajectory Dataset
          |
    TESO-Diff
