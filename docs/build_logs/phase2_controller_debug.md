# Phase 2a / WP2A-8 架构、接口与控制器调试总记录

整理日期：2026-11-02。本文合并build_logs内18份阶段记录，替代原分散日志。最新状态：用户已手动验证当前集成整定结果，暂时接受；这是当前工作基线，不代表扩展包线与协调侧滑问题已经全部解决。

## 1. 工作边界与项目归属

- TESO-Diff负责6DOF运行时、Python控制接口、闭环试验和后续Isaac数据生成。
- Simulink模型与MATLAB整定工具归入 `E:/matlab_Project/AVANTI_4`。现行工具目录为 [matlab](E:/matlab_Project/AVANTI_4/matlab)，原模型仍位于 `ert_model/PX4Controller_ertmodel0827.slx`，本次清理不改动它。
- Python当前NPFG/TECS/姿态控制器与MATLAB姿态控制模型是两条实现链。最后一轮SLX参数的接受不自动等于已移植、整定或验收Python控制器。
- 原日志中的中间参数、单轴“已接受”状态、旧文件路径及旧脚本命令均为历史，当前参数与能力边界以本文末尾集成整定记录为准。

## 2. SP1：架构与运行时移植

Phase1公共头、Embedded Coder C++源、C ABI桥和Linux动态库移入runtime/fixedwing；CMake与ctypes按新目录组织。控制器、风场、harness、ESO记录器与dataset/isaac目录结构建立。历史接口回归：C++/Python 20s通过，16实例一致性差为0，生命周期压力RSS与fd无增长，12个YAML配置校验通过。

冻结契约：plant固定步长ODE4、dt=0.02s，外部时钟逐步推进；7输入为三舵面、油门、三维风。位置输出是NEU（高度Up-positive），不是因为变量名含NED就按Down-positive解释。风输入的垂直符号须遵守ABI。u/v/w和p/q/r是机体系量；phi/theta/psi是相对导航系的ZYX姿态欧拉角。偏航±pi环绕属于表示问题，角度误差使用wrap_pi；旧日志“体轴姿态角”称呼不再沿用。

Windows使用匹配位数的DLL，Linux使用SO；WinError193曾由Windows加载Linux SO引发，应按平台选择动态库并核对依赖，不能仅凭文件存在就加载。

## 3. SP2：Python控制律与GA历史

NPFG/L1提供侧向导航与期望坡度，TECS提供俯仰和油门，协调转弯提供偏航速率参考，角度P与速率FF/P/PI组成内环；支持批量numpy状态、reset与snapshot/restore。numpy批量运算并不代表当前Windows已具备Isaac并行能力。

舵面符号复测：delta_e正为低头、delta_a正为左滚、delta_r正为右偏航。早期升降舵符号误记是俯仰不稳定的原因，不能继续沿用。

GA历史顺序：tuning_02校准串级内环，tuning_03校准圆任务外环，tuning_04联合圆/8字/跑马场。最终GA中L1 period约11.913、damping约0.619；T13跑马场横侧MAE从约11.1m降至2.34m，圆任务约0.73m。早期仅圆任务约0.2m的成绩不代表捕获、8字或跑马场同样通过。loiter-setpoint导航变体曾比采样制导更差，随后撤回；不得按旧附录将其视为现行实现。

## 4. Controller API与连续路径取样

标准接口位于python/controller/controllerAPI.py：LoiterCommand表示中心NEU、空速、半径和时间戳；HeadingCommand表示空速、gamma、chi和时间戳。chi从North起、East为正，角度单位rad。Heading是航迹指令，不能混同姿态pitch/yaw。

T15最初用距离小于25m切换密集航点，造成跳点、错过点后索引卡死、方位翻转及8字交叉分支混淆。修复使用PolylinePathFollower：局部线段投影→单调弧长进度→速度相关前视→有符号横侧误差修正→航向unwrap和限速。默认前视时间2.5s、距离35~55m、横侧修正系数1、航向变化率18deg/s；高层10Hz、底层50Hz。

400s无风T15验证：横侧MAE2.47m，40s后2.43m、95分位2.68m；完成6圈，进度单调；高度RMSE0.99m，空速RMSE0.026m/s。测试含交叉分支保持与snapshot恢复。入口仍是 `python experiments/wp2a8/api_heading_adaptive_trial.py`；现有experiments与Python源码未在本次清理中删除。

Dubins不是本轮取样修复的前提。后续待办：Loiter的APPROACH/CAPTURE/HOLD及2R切圆语义、Heading gamma长期高度轨迹、完整API状态恢复、曲率换向瞬态。早期T15横侧误差记录为0是每步临时直线的伪指标，不能当成实际路径误差。

## 5. Simulink结构整改与IA演进

1. 内环和观测频率保持50Hz；把plant降至20Hz再重复使用旧观测不能产生新的反馈信息，因此未把50/20Hz作为滞后修复方案，也未急于拆分SLX。
2. 对归一化舵面施加±1物理限幅，PI输出为角加速度域限幅，两者单位不同。内部限幅最终为滚转/俯仰/偏航±10/±10/±5；不能按10kgf堵转扭矩直接换算成该数值。
3. 抗积分饱和使用back-calculation和真实舵面裁剪残差回算，Kb/Kt历史设为5；实际施加输出差额须经混控与PI空速缩放的逆映射反馈。前馈、PI及混控矩阵的单位和符号必须一致。
4. 混控错误曾把正向效能元素与元素倒数拼成同一矩阵。应先构造完整B，再整体求逆M=B^-1；根部u=M*angular_accel_cmd，AW使用同一映射。最大舵角须转换rad；plant力矩包含翼展b/弦长c。控制器按用户决定忽略Ixz，而完整plant仍保留惯量耦合。
5. 参考动压使用0.5*rho_ref*S*V_ref^2。已有IAS平方PI缩放与TAS前馈缩放时，避免混控再次重复实时动压缩放。IAS换算用V_TAS*sqrt(rho/1.225)，sqrt(rho)后的增益应为1/sqrt(1.225)。
6. 初期五参数共用内环、分轴阶跃、48/48/32分阶段、滚转D开放及偏航P扩界均属历史。D试验未明显解决滚转上升时间，随后恢复D=0、N=100、50Hz。
7. 单独p_sp速率试验曾用15/30deg/s训练和20/25deg/s验证；大速率45~90deg/s主要受物理舵效约束。20/25deg/s的过渡RMSE还受阶跃瞬时因果延迟影响，不能把未通过简单归因于积分限幅。
8. 25deg俯仰阶跃引发滚转的直接根因是欧拉速率转换矩阵(1,3)/(3,2)负正弦接反。正确矩阵为[[1,0,-sin(theta)],[0,cos(phi),sin(phi)cos(theta)],[0,-sin(phi),cos(phi)cos(theta)]]。仅修正两处连线，30s滚转峰值由61.78deg降为0；0.01deg初始扰动亦收敛。当前集成整定已复核正确矩阵。

上述历史参数与接受注册表已由最后一轮全11参数联合整定取代。清理后不保留旧修改探针、模型变体、备份、分阶段入口或过期warm-start注册表。

## 6. 当前有效工具与复现

工具位于 [E:/matlab_Project/AVANTI_4/matlab](E:/matlab_Project/AVANTI_4/matlab)，保留现行IA核心、斜坡工况/评价、入口、有限批处理和结果后处理。最优参数、完整候选历史、配置与源模型快照、最终逐时刻数据和图像随工具迁移；旧进程日志/偏好设置/缓存/中间checkpoint删除。MAT历史结果中旧绝对路径仅作来源记录，现行入口按传入runDir重建执行路径。

MATLAB R2021a原生堆损坏曾在SDI日志路径反复出现；改用To Workspace结构体记录后，9个基线指标与原记录方式差值为0并完成128代。失败批次仅从完整检查点恢复，剔除未完成代的10次已完成调用，保留2176次正式评价。此处是已验证的规避方式，不宣称彻底修复MATLAB原生问题。

详细操作见外部matlab/README.md。当前参数为用户手动测试后暂时接受；不额外写入SLX。下一步优先处理长时侧滑与扩展包线饱和，再判断是否将控制律移植回Python并重新验收。

## 7. 最后一次集成整定完整记录

## WP2A-8：当前姿态控制全参数斜坡 IA 联合整定

日期：2026-10-29。仅针对本次用户手工修改后的模型快照；未把参数自动写入外部原 SLX。

### 1. 结果与适用范围

完成 **128 代、2176 次候选评价调用**；其中不同候选 1668 个，重复候选由缓存复用。每个新候选在滚转、俯仰、耦合三种斜坡上评价，不是分别把三个通道单独整定后拼接。

原参数 J=15005447.2，亲和度最优 J=6976750，代价下降 **53.51%**。这里代价越低越好；用 A=1/(1+J/1e8) 表示亲和度时，A 从 0.869524 提高到 0.934783。

亲和度最优候选：训练集通过，独立验证集通过；满足全部训练门槛的最优候选：独立验证集通过。二者相同。训练门槛与搜索亲和度是分开的，低代价不自动等于全部指标合格。

**能力边界仍然存在**：25°俯仰诊断的升降舵连续饱和2.64 s，±40°耦合诊断的副翼连续饱和1.78 s，均超过本轮1.5 s门槛；这些扩展工况没有全面通过。60 s盘旋姿态保持通过姿态门槛，但保持末端侧滑约7.72°，回正结束约9.32°，不能称为零侧滑协调盘旋已经合格。

本报告的“可控性”指这些工况内的闭环指令跟随和恢复能力，不是非线性系统的全局可控性证明，也没有计算全状态 Kalman 可控性矩阵。

### 2. 当前模型、参数集和一致性检查

- 原模型：`E:\matlab_Project\AVANTI_4\ert_model\PX4Controller_ertmodel0827.slx`。
- 源文件 SHA256：`d478a6fc18c627b79c4112f7fbabb03426a91edc78f335d8a1ffb75914291dee`。整定使用独立快照和配置副本，原文件结束时哈希一致。
- 当前欧拉角速率到体轴角速度转换矩阵逐点核验通过；三轴 PI 编译后采样时间均为 `[0.02,0]`，plant 的 ode4 步长为0.02 s。
- 释放全部11个参数：角度 P×2、前馈×3、三轴 PI×6。此前的俯仰锁定与偏航锁定均不沿用；三轴D仍为0、N保持当前模型设置。
- 热启动直接读取本次 SLX Model Workspace，不覆盖成旧 search_state.mat 中的参数；旧 accepted_pitch 和全局 warm-start 文件没有改写。
- 全程油门0.7、无风；保持原有舵面及PI限幅，不增加角速度限幅。
- 普通模式基线与快速重启首个候选的亲和度代价完全一致；脚本与模型快照保存在本次运行目录。
- 无效／仿真异常候选调用数：0。保留完整128代历史，每代16个免疫候选加1个退火候选。

### 3. 三类斜坡及独立验证

| 集合 | 时长 | 滚转单通道 | 俯仰单通道 | 滚转／俯仰耦合 |
|---|---:|---|---|---|
| 训练 | 18 s | 0→+30→0→−30→0° | 0→+12→0→−8→0° | 滚转同前，左右坡度保持段均给俯仰+8° |
| 验证 | 21 s | 0→+35→0→−35→0° | 0→+15→0→−10→0° | 滚转同前，坡度保持段给俯仰+10° |

训练节点时刻为 `[0,1,4,7,10,13,16,18]` s；验证为 `[0,1,5,8,12,15,18,21]` s。节点之间线性插值，控制器按50 Hz读取零阶保持样值。训练滚转斜率最高15°/s，俯仰最高4°/s；验证使用不同幅值和斜率。

另作未参与训练的三项诊断：60 s的30°滚转/+8°俯仰保持及回正、25°俯仰斜坡、±40°滚转/+10°俯仰耦合斜坡。保持固定姿态只表示盘旋姿态跟随，不等于航迹、空速或高度已被外环闭环维持。

### 4. 亲和度与门槛

每个有指令变化的姿态轴使用：

```text
C_axis = 0.15 RMSE/A + 0.25 ramp_MAE/A + 0.25 mean_lag_90[s]
       + 0.20 hold_MAE/A + 0.15 overshoot/A
A = max(该轴指令绝对幅值, 3 deg)
C_case = 有指令轴 C_axis 的均值 + 未指令轴串扰惩罚
       + 0.08 mean(body_rate_RMSE / [20,12,15] deg/s)
tracking = mean(C_case) + 0.5 max(C_case)
chatter = max_case(surface_increment_RMS/0.06 + 0.25 increment_peak/0.20)
sat_case = 0.25 duty + max(0,max_dwell-1s) + 2 flip_rate
         + 0.03 min(raw_excess_RMS,10)
saturation = mean(sat_case) + 0.5 max(sat_case)
J = 1e8 * (tracking + 0.02 chatter + 0.005 saturation)
```

90%滞后是实际达到该段90%目标的时刻减去斜坡参考达到90%的时刻；比参考更早达到不记负滞后。未达到时追加惩罚。超调相对于该段最终目标计算；保持误差在保持段开始0.5 s后计入。舵面抖动评价只排除启动前0.5 s，不会把每个斜坡采样变化都当作阶跃而排除整个斜坡。

| 门槛 | 本轮 | 相对之前 |
|---|---:|---|
| 滚转／俯仰RMSE | ≤6°／3.6° | 由5°／3°适度放宽20% |
| 滚转／俯仰超调 | ≤4°／3° | 单独控制，不用降低RMSE权重掩盖超调 |
| 斜坡90%最大滞后 | ≤1 s | 新增动态门槛 |
| 滚转／俯仰保持MAE | ≤3°／2° | 明确保持段跟踪 |
| 未指令轴RMSE | ≤3° | 避免单轴改善伴随明显串扰 |
| 协调偏航体轴速率RMSE | ≤8°/s | 姿态跟随之外继续约束偏航 |
| 舵面最长连续饱和 | ≤1.5 s | 原1 s；第三优先级权重降低 |
| 舵面单步RMS／峰值 | ≤0.06／0.20 | 保留，区分响应快与高频抖动 |
| 相邻采样两端饱和反转 | ≤0.5次/s | 保留 |

退火不随机更改试验目标或门槛，而是对较差候选按温度概率接纳；最优档案不被退火扰动。温度从2降至0.01，另保留克隆变异和随机新个体。
本轮退火接纳 99 次，其中向较差代价移动 48 次。

### 5. 最优参数

| Model Workspace变量 | 原参数 | 亲和度最优 | 全训练门槛通过的最优 | 搜索范围 |
|---|---:|---:|---:|---|
| `P_phi_sp` | 6 | 8 | 8 | [0.1, 8] |
| `P_theta_sp` | 3.938503272 | 8 | 8 | [0.1, 8] |
| `FW_RR` | 37.164549 | 73.5591994785 | 73.5591994785 | [0, 80] |
| `FW_PR` | 12.32245822 | 33.0766501559 | 33.0766501559 | [0, 40] |
| `FW_YR` | 6.1988471 | 5.88724008066 | 5.88724008066 | [-20, 20] |
| `Omega_sp_PPhi` | 35.69142576 | 2.55548530054 | 2.55548530054 | [0.01, 60] |
| `Omega_sp_IPhi` | 30 | 17.3911759678 | 17.3911759678 | [0, 40] |
| `Omega_sp_PTheta` | 9.844092786 | 37.1730053299 | 37.1730053299 | [0.01, 40] |
| `Omega_sp_ITheta` | 7.187330431 | 18.289049312 | 18.289049312 | [0, 30] |
| `Omega_sp_PPsi` | 46.80110083 | 56.6530807318 | 56.6530807318 | [0.01, 70] |
| `Omega_sp_IPsi` | 4.121929618 | 18.30415489 | 18.30415489 | [0, 30] |

D_phi=D_theta=D_psi=0。没有独立偏航角P：偏航体轴速率来自协调转弯和姿态速率坐标变换。

触及搜索边界的最优参数：P_phi_sp, P_theta_sp。触边只说明本次有界搜索的边界解，不构成进一步增益加大的稳定性证明。

### 6. 基线与亲和度最优的逐工况结果

各单元格顺序为“原参数 → 本轮最优”。角度RMSE为度，滞后为秒；横向零指令时仍统计串扰。

| 工况 | 滚转RMSE | 俯仰RMSE | 最大滞后(滚转/俯仰) | 偏航速率RMSE °/s | 最优门槛 |
|---|---|---|---|---|---|
| roll_ramp_train | 1.236 → 0.532 | 0.245 → 0.146 | 0.14/0.00 → 0.06/0.00 | 1.345 → 0.800 | 通过 |
| pitch_ramp_train | 0.000 → 0.000 | 0.777 → 0.285 | 0.00/0.54 → 0.00/0.16 | 0.000 → 0.000 | 通过 |
| coupled_ramp_train | 1.245 → 0.607 | 0.504 → 0.176 | 0.16/0.24 → 0.08/0.06 | 1.615 → 0.942 | 通过 |
| roll_ramp_validate | 1.203 → 0.476 | 0.258 → 0.190 | 0.14/0.00 → 0.04/0.00 | 1.441 → 0.804 | 通过 |
| pitch_ramp_validate | 0.000 → 0.000 | 0.835 → 0.304 | 0.00/0.52 → 0.00/0.16 | 0.000 → 0.000 | 通过 |
| coupled_ramp_validate | 1.441 → 0.964 | 0.498 → 0.180 | 0.20/0.26 → 0.16/0.06 | 1.847 → 1.006 | 通过 |
| turn_hold_30_8 | 0.332 → 0.183 | 0.229 → 0.054 | 0.12/0.24 → 0.08/0.04 | 0.816 → 0.327 | 通过 |
| pitch_ramp_25 | 0.000 → 0.000 | 0.944 → 0.362 | 0.00/0.34 → 0.00/0.14 | 0.000 → 0.000 | 未通过 |
| bank_ramp_40 | 1.751 → 1.251 | 0.471 → 0.185 | 0.24/0.30 → 0.16/0.04 | 2.142 → 1.126 | 未通过 |

未通过项（不隐去训练与诊断结果）：

- 亲和度最优：pitch_ramp_25: saturationDwell；bank_ramp_40: saturationDwell。
- 门槛合格最优：pitch_ramp_25: saturationDwell；bank_ramp_40: saturationDwell。

### 7. 舵面使用、能量与可控性边界

| 最优工况 | 副翼/升降舵/方向舵饱和占比 | 最长饱和 s | 单步RMS/峰值 | 两端反转 次/s | 最低空速 m/s | 最大侧滑 ° |
|---|---|---:|---|---:|---:|---:|
| roll_ramp_validate | 0.00%/0.00%/0.00% | 0.00 | 0.0062/0.1028 | 0.000 | 13.490 | 6.561 |
| pitch_ramp_validate | 0.00%/0.00%/0.00% | 0.00 | 0.0025/0.0876 | 0.000 | 11.389 | 0.000 |
| coupled_ramp_validate | 5.71%/0.00%/0.00% | 1.20 | 0.0081/0.1372 | 0.000 | 13.109 | 6.106 |
| turn_hold_30_8 | 0.00%/0.00%/0.00% | 0.00 | 0.0023/0.0832 | 0.000 | 13.490 | 9.324 |
| pitch_ramp_25 | 0.00%/10.99%/0.00% | 2.64 | 0.0050/0.1881 | 0.000 | 7.284 | 0.000 |
| bank_ramp_40 | 8.36%/0.00%/0.00% | 1.78 | 0.0071/0.1190 | 0.000 | 12.002 | 8.252 |

验证集机体系角速度跟踪RMSE（p/q/r，deg/s）：

- roll_ramp_validate：5.768/2.078/0.804。
- pitch_ramp_validate：0.000/1.374/0.000。
- coupled_ramp_validate：4.555/2.226/1.006。

这些参数针对完整角度—角速度级联优化。角度跟随改善不自动意味着单独p/q/r速率接口也最优；若恢复此前直接覆写omega_sp_b的速率指令入口，应重新做独立速率验证。

**长时侧滑问题独立存在。** 30°滚转/+8°俯仰保持试验中，t=20/40/55 s的β约2.94/6.61/7.72°，此时姿态仍紧跟指令，舵面未饱和；回正结束t=60 s的β约9.32°。本轮评价约束了偏航体轴速率跟随，没有把β=0设为目标或验收门槛。因此，“姿态保持通过”不等于“协调转弯通过”。后续应审查协调转弯参考生成与侧滑/横向加速度反馈，而非仅继续提高PI或把该现象归为角度跟踪不足。

滚转单通道验证的p速率RMSE由基线约4.01°/s升至5.77°/s，而滚转角RMSE由1.20°降至0.48°；这体现了本轮偏重姿态动态性能的集成优化取舍，不应隐去速率环独立跟踪的退化。

结论应限定于上面的幅值、斜率、动力和初始状态。固定油门0.7不等于空速闭环；大俯仰工况即使角度门槛通过，也应结合最低空速和舵面余量判断可用性。未验证低速／大侧滑气动模型有效性，不把仿真收敛直接解释为真实飞机失速区可控。

本轮没有独立偏航角输入，偏航PI/FF通过转弯时的r_sp跟随参与搜索；不能据此声称航向角控制已验证。也未覆盖阵风、传感器噪声、舵机延迟、参数不确定性与不同初始速度。

### 8. 产物和复现

- [亲和度最优参数赋值文件](E:/matlab_Project/AVANTI_4/matlab/ia_results/integrated_ramp_sa128_20261029/best_affinity_parameters.m)。这些赋值需放入模型工作区；单独在基础工作区运行不会覆盖同名Model Workspace变量。
- [训练门槛合格最优参数赋值文件](E:/matlab_Project/AVANTI_4/matlab/ia_results/integrated_ramp_sa128_20261029/best_feasible_parameters.m)。独立验证是否通过以本报告为准。
- [结果JSON](E:/matlab_Project/AVANTI_4/matlab/ia_results/integrated_ramp_sa128_20261029/summary.json)、[完整IA结果MAT](E:/matlab_Project/AVANTI_4/matlab/ia_results/integrated_ramp_sa128_20261029/result.mat)、[完整性审计](E:/matlab_Project/AVANTI_4/matlab/ia_results/integrated_ramp_sa128_20261029/final_audit.json)。
- [工况与评价代码](E:/matlab_Project/AVANTI_4/matlab/ia_px4_ramp_fixture.m)、[联合整定入口](E:/matlab_Project/AVANTI_4/matlab/ia_px4_ramp_run.m)、[有限批处理运行器](E:/matlab_Project/AVANTI_4/matlab/run_px4_ramp128.ps1)。

![亲和度收敛](E:/matlab_Project/AVANTI_4/matlab/ia_results/integrated_ramp_sa128_20261029/affinity_convergence.png)

![roll_ramp_validate](E:/matlab_Project/AVANTI_4/matlab/ia_results/integrated_ramp_sa128_20261029/roll_ramp_validate.png)

![pitch_ramp_validate](E:/matlab_Project/AVANTI_4/matlab/ia_results/integrated_ramp_sa128_20261029/pitch_ramp_validate.png)

![coupled_ramp_validate](E:/matlab_Project/AVANTI_4/matlab/ia_results/integrated_ramp_sa128_20261029/coupled_ramp_validate.png)

![turn_hold_30_8](E:/matlab_Project/AVANTI_4/matlab/ia_results/integrated_ramp_sa128_20261029/turn_hold_30_8.png)

![pitch_ramp_25](E:/matlab_Project/AVANTI_4/matlab/ia_results/integrated_ramp_sa128_20261029/pitch_ramp_25.png)

![bank_ramp_40](E:/matlab_Project/AVANTI_4/matlab/ia_results/integrated_ramp_sa128_20261029/bank_ramp_40.png)

### 执行异常记录

- 第49代之后出现原生执行异常：MATLAB 0xc0000374 heap corruption after IA_CHECKPOINT_ONLY before wrapper completion marker。已核验检查点，共833次已保存候选调用；丢弃的已完成调用数0。续跑措施：generation50 onwards, per-worker MATLAB_PREFDIR, no automatic retry。
- 第62代之后出现原生执行异常：MATLAB 0xc0000374 heap corruption during generation63, after evaluation1058。已核验检查点，共1054次已保存候选调用；丢弃的已完成调用数4。续跑措施：resume generation63 from complete RNG/population/cache, reduce each worker to four generations; no automatic retry。
- 第67代之后出现原生执行异常：MATLAB 0xc0000374 heap corruption during generation68 after evaluation1145, despite four-generation worker。已核验检查点，共1139次已保存候选调用；丢弃的已完成调用数6。续跑措施：replace SDI/Dataset signal logging with To Workspace structure logging only; all nine baseline cases tracking/chatter/saturation/RMSE reproduced exactly, max error=0; resume RNG/population/cache generation68。

进程异常与控制器数值失败分开统计。保留代数仍是完整128代，使用检查点种群、缓存、退火状态和随机数状态续跑；这不代表已彻底修复MATLAB原生堆损坏。

## 被合并的原记录索引

以下原文件已由本文取代并删除；哈希用于确认整理前来源，不表示仍有备份。

|原文件|SHA256|
|---|---|
|phase2_sp1.md|`d107a7f3d756d5d90e76ee13eb1bb68c81f82e30ef37f50dbb0d081ff527ec2f`|
|phase2_sp2.md|`b9c93ca2f6eba3436274280ee286f0e0f07478295aeb03ee2e726b57e8621798`|
|wap2a-controllerAPI_design.md|`df54c76952c87d6da44416935f75f57b65b2548ce51a159cd89412e3beedc389`|
|wp2a-8_debug.md|`75a3c1c6a8a5c7960ae70316baa2b2b7d8d5581e6e75e53657cd22f7dfbf95e5`|
|wp2a-8_integrated_ramp_ia128_20261029.md|`5333f669e2548b452ae33a665f78305deb81b1f1f4abec64784bede18c5aae29`|
|wp2a-8_pitch25_roll_coupling_20261029.md|`0faae1ed84255b238311362619370f00bb1bf538ea15401285cd624ceacf9576`|
|wp2a-8_px4_aw_mixer_manual_20261027.md|`21614d1c9d7300fa24f4ed572409d097db1bd4b065b22aee61c48935bff974be`|
|wp2a-8_px4_ia_20261027.md|`ebe5cc347414eb632d9e6ee3913e3dcd003fe443c70de0ddb4e7bee75ab35152`|
|wp2a-8_px4_ia_priority_20261027.md|`7bd8326ace1dad6b33fe2be0d5f19982aea3f1ff08320703ac9f721e097a515e`|
|wp2a-8_px4_ia_sa128_20261028.md|`c469de643b2e3762340a73715e21ab66d1f009e0f9fd2d248a61925ebe2aaa46`|
|wp2a-8_px4_ia_single_axis_20261027.md|`3d8d4413af957dcb711549bf5b48f6404333de0a8d886ea1577895184a9947fa`|
|wp2a-8_px4_inner_structure_20261027.md|`f4346712d6510fc2268730df0ca8219e58c639832fa7ba004622e51126f40f48`|
|wp2a-8_px4_lateral_d_ia128_20261029.md|`d17f850b1baa418c0fa0ef42757e579057a684a48fc4f4f121f35ef01271d336`|
|wp2a-8_px4_multirate_feasibility_20261027.md|`2f2899f711ca893ba8d6ebbcd9ae247440cb5edea8d6e275ae2fe9e0c16deeb7`|
|wp2a-8_px4_preflight_20261028.md|`fc42480b5798b619e40d2fdeb32afd7f8b7d7d768d379864740e92941ec9f0a4`|
|wp2a-8_px4_roll_rate_ia128_20261029.md|`5681c326a2b0261ca7ffe8da297f4ae732952207b4b375c173354c8ca1e18c08`|
|wp2a-8_px4_split_ia128_20261028.md|`868e467e225c7f3fdbeef27097b17d717d920b550894629b33a6104fc84a92b4`|
|wp2a-8_px4_staged_pitch_accept_20261028.md|`d61e75bba07c278be79a78e56933215012ab54330152e0e91b40a5963d3f6a59`|
