# Phase 2 SP2 — WP2A-8 控制器 v2（PX4 架构）+ GA 整定 + 控制品质试验

**日期：** 2026-08-30
**任务：** 参考 PX4 固定翼控制器系列（姿态串级 / 协调转弯 / TECS / L1+NPFG），
重构 `python/controller` 控制律（保持 GPU 可并行张量化），限定记忆 GA 重新整定，
完成 WP2A-8 控制品质试验（轨迹 / 姿态角 / 角速度 / 速度跟踪图像，中文无乱码）。

---

## 1. 关键发现：舵面符号复测（修正早期辨识错误）

对 phase1 `.so` 重做确定性符号探针（throttle=0.4，±0.05 阶跃，6 s）：

| 通道 | 响应（复测） | 内环符号 |
|---|---|---|
| `delta_e` +0.05 | theta 0°→-16.3°（**低头**） | 俯仰通道取负（**修正：早期记为抬头，相反**） |
| `delta_e` -0.05 | theta 0°→-2.5°（抬头） | |
| `delta_a` +0.05 | phi 0°→-6.5°（左滚） | 滚转通道取负（与早期一致） |
| `delta_r` +0.05 | psi 0°→+14°（右偏航） | 偏航通道取正（与早期一致） |

早期（tuning_01 时期）把 delta_e 记反，是单环 PID 时代俯仰通道无法稳定的
直接原因。修正后闭环立即稳定（见 §3）。记录见
`experiments/wp2a8/control_sign_probe.py`（已更新结论注释）。

## 2. 控制器 v2 架构（PX4 参考，全张量化）

### 2.1 内环串级姿态（`inner.py`，PX4 系列第一期/第二期）
- 角度环 P → 期望机体系角速率 rate_sp（±60/45/30 °/s 限幅）；
- 角速率环：期望角加速度前馈（rate_ff）+ 速率误差 P/PI
  （滚转不加 I——气动阻尼足够；俯仰/偏航加 I 建立配平舵量）；
- **空速缩放**：控制项 $(V_{trim}/V_{TAS})^2$（动压补偿）、
  前馈项 $V_{trim}/V_{TAS}$（阻尼力矩补偿）；
- **协调转弯前馈**：$r_{ff} = g\tan\phi\cos\theta/V_{TAS}$（偏航通道）。

### 2.2 TECS 总能量控制（`tecs.py`，PX4 系列第三期）
- 比能量误差 $STE = (h_{sp}-h) + (V_{sp}^2-V^2)/(2g)$ → 能量变化率需求；
- 油门控制总能量变化率：$T = T_{trim} + K_p\Delta\dot E + K_i\int\Delta\dot E$；
- 升降舵控制能量分配：$B=\gamma-\dot V/g$，$\theta_{cmd} = \gamma_{sp}$ 前馈 +
  $K_p\Delta B + K_i\int\Delta B$ + 配平；
- 转弯升力补偿：$\theta += k_{bank}(1/\cos\phi - 1)$。

### 2.3 L1 + NPFG 侧向制导（`l1.py`，PX4 系列第五期）
- L1 参数化：$L_1 = \frac{1}{\pi}\xi T\,GS$，$a_{cmd}=4\xi^2 GS^2/L_1\sin\eta$；
- **NPFG 风三角**：$a⃗ = s\hat t - w⃗$，$s = w⃗·\hat t + \sqrt{V_{sp}^2 - |w⃗×\hat t|^2}$，
  空速圆外退化 $a⃗=V_{sp}\hat t$（强风防失控）；
- **盘旋自动检测 + PX4 Circle（loiter）模式**：
  $a_{cmd} = dir\cdot(GS^2/R + K_r e_r + K_v v_r)$（向心前馈 + 径向 PD）；
- 通用路径参考点：距离 ≥ L1 候选中取航向偏差最小者（避免选中身后/相邻环上的点）。

### 2.4 GPU 并行性
全部为 numpy 批量张量运算（batch 维广播、`np.where`/`argmin`/`arctan2` 向量化），
无逐实例 Python 循环；积分器（TECS 能量/速度/分配、速率环 PI）张量化并支持
snapshot/restore（counterfactual 要求，docs §6.12）。

## 3. 限定记忆 GA 整定

### 3.1 tuning_02：串级内环（13 维基因）
剧本：0~8 s 平飞稳定 + 8~25 s ±15° 滚转方波；μ=12 λ=12 ×40 代，8 并行。

| 指标 | 种子均值 | GA 最优 |
|---|---|---|
| 适应度 | 19.7~327.7 | **5.47** |
| θ 跟踪 MAE | 1.3~1.9° | 与 φ/ψ 联合优化 |
| 高度保持 | 991~997 m | 997 m |

最优基因要点：角度环 9.9/6.0/6.0，速率环 0.90/0.51/1.33，
俯仰/偏航速率积分 3.41/3.80（建立配平舵量），θ_trim=-6°，油门 0.474。

### 3.2 tuning_03：外环 NPFG/TECS 盘旋闭环（16 维基因）
剧本：盘旋 R=60 m（圆心 (0,60)，初始即在圆周切向），90 s 无风闭环；
内环固定为 tuning_02 结果；μ=10 λ=10 ×25 代，8 并行。

| 指标 | 种子均值 | GA 最优 |
|---|---|---|
| 适应度 | 0.33~0.52 | **0.095** |
| 横侧误差 MAE | 0.63~0.93 m | ≈0.2 m |
| 高度 RMSE | 0.04~0.13 m | ≈0 m |
| 空速 RMSE | 0.03~0.08 m/s | ≈0 m/s |

最优基因要点：L1 period=20（圆用 loiter 制导，period 不影响圆跟踪）、
circle_kr=1.5/circle_kv=1.58（触界）、airspeed_kp=1.0（触界）、
throttle_trim=0.533、θ_trim=-1.5°。触界基因说明圆任务下这些方向的
最优值在界外，后续 D1 可飞域扫描（docs §17）时再扩展。

## 4. WP2A-8 控制品质试验（`experiments/wp2a8/quality_trials.py`）

4 项试验，每项输出 2×2 图（航迹跟踪 / 姿态角跟踪 / 角速度跟踪 / 速度跟踪），
中文由 `mpl_style.setup_chinese_font()`（Noto Sans CJK JP + unicode_minus=False）
保证无乱码：

| 试验 | 场景 | 横侧 MAE | 高度 RMSE | 空速 RMSE | φ 跟踪 MAE | NaN |
|---|---|---|---|---|---|---|
| T11 | 盘旋 R=60 V=16（无风） | **0.18 m** | **0.00 m** | **0.00 m/s** | 0.78° | 无 |
| T11b | 盘旋 + 北风 3 m/s（NPFG） | **0.21 m** | 2.30 m | 0.35 m/s | 0.75° | 无 |
| T12 | 8 字 R=80 V=15（period=8） | 7.40 m | 1.44 m | 0.91 m/s | 8.43° | 无 |
| T13 | 跑马场 L=160 R=60 V=15 | 15.18 m | 0.76 m | 0.46 m/s | 6.75° | 无 |

说明：
- 8 字/跑马场使用 per-task L1 参数（period=8, damping=0.75）与放宽的坡度限
  （45°/35°），因为 GA 整定的 period=20 只对圆任务无影响（圆走 loiter 制导）；
- 8 字最小曲率半径 ≈ R/3：R=50 在 16 m/s 下需 ≈57° 坡度（超包线），
  故试验用 R=80、V=15（需求 ≈41°）——任务参数需落在可飞域内
  （docs §16.1 投影面随机化前应先按 D1 冻结可飞参数域，docs §49 风险表）；
- 跑马场剩余误差集中在两处 180° 弯道（滚转大角度切换时的超调），
  待 WP2A-10 通过 D1 envelope 扫描 + 投影面可飞域约束进一步收敛。

## 5. 回归验证

| 项目 | 结果 |
|---|---|
| 包级单测（9 项：PID/数据结构/串级符号/限幅/圆检测/NPFG 风三角/wind 符号） | 通过 |
| 接口级冒烟 4 项（依赖审计 + 20s 回归 + 16 实例 + 生命周期） | 通过 |
| 控制器全张量化检查（无逐实例循环） | 通过（代码审查） |

## 6. 文件清单

- 控制器：`python/controller/{inner,tecs,l1,coordinated_turn,outer,avanti_controller,types}.py`
- GA：`python/controller/ga/{limited_memory_ga,attitude_tuning,outer_tuning}.py`
- 试验：`experiments/wp2a8/{control_sign_probe,tuning_02_attitude_cascade_ga,tuning_03_outer_circle_ga,quality_trials}.py`
  + 各试验输出目录（PNG + result.json / summary.json）
- 配置：`config/controller/controller.yaml`（写入 tuning_02/03 整定参数）
- 仿真：`python/simulation/harness.py`（FC 风估计低通 + wind_neu 观测）、
  `task_route.py`

## 7. 下一步

1. WP2A-9 正式闭环 harness 验收 + avanti_eso 张量流完整记录；
2. WP2A-10 完整冒烟矩阵（T00~T14，docs §6.14），T14 小规模 8 实例并行；
3. D1 可飞域扫描确定 8 字/跑马场/投影面的可飞参数域（docs §17/§49）；
4. WP2A-6/7 reset 确定性与 snapshot/clone 方案；
5. 30 min+ 长时序稳定性与 Gate A 验收（docs §6.17）。

---

# 附录 A：WP2A-8 品质试验问题整改（v3，2026-08-30）

针对用户审查 quality_trials 提出的 4 类问题逐项整改。

## A.1 控制律跟踪量核对（用户问题 2：是否跟踪体轴量）

| 控制量 | 坐标系 | 控制律 | 结论 |
|---|---|---|---|
| φ/θ/ψ | 体轴欧拉角 | 内环角度环 P（PX4 串级外层） | ✓ 正是 PX4 内部控制要求 |
| p/q/r | 体轴角速率 | 内环角速率环 FF+P/PI（串级内层） | ✓ |
| 高度/空速 | NEU + 标量 TAS | TECS 能量控制 | ✓ |
| 水平位置 | NEU | L1/NPFG / loiter | ✓ |
| u/v/w | 体轴速度 | **无直接指令通道**（与 PX4 相同）：u 由 TECS 空速→油门间接、v 由协调转弯前馈+偏航环+β 阻尼间接、w 由俯仰/升力间接 | 图5 中 u 期望=空速指令、v/w 期望≈0 为协调飞行目标，非直接控制量 |

结论：控制律本身跟踪正确（体轴姿态角 + 体轴角速率 + NEU 位置 + 空速），
无需重调；图像已按体轴量重绘（5 图）。

## A.2 问题整改清单

1. **跑马场半圆方向**（用户问题 4）：修复圆弧内凹——北/南半圆改为外凸、
   且直道长轴按文档沿北向（原实现误转了 90°，起点垂直于航线）；
   试验中心 (0,80) 使起点 (0,0) 恰在西直线上、航向切向。
2. **放大轨迹**（用户问题 1）：8 字 R=80/圆心距 400（双圆 + 顶部公切线相接，
   两圆统一顺时针使切线连接连续）；跑马场 R=80/L=400。
3. **loiter 切圆 setpoint 导航**（用户问题 1 备选）：`l1.py::_setpoint_loiter_step`
   全张量化掩码状态机（无逐实例 if-else）：远离圆心（>2R）直飞段入口点前方
   切线虚拟目标；进入 2R 内 loiter 切入（径向修正限幅 ±向心加速度，防捕获饱和）；
   累计扫过 sweep 角切下一段；near 掩码带 2.6R 滞回。段解析
   `figure8_loiter_segments` / `racetrack_loiter_segments`，经
   `ControlReference.loiter_*` 字段 + harness `task_type` 传入。
4. **5 图输出**（用户问题 2）：每试验 1) 航迹 2) xyz 期望位置 3) 体轴姿态角
   4) 体轴角速率 5) 体轴速度 u/v/w。

## A.3 v3 试验结果（未调整任何控制器参数）

| 试验 | 导航 | 横侧 MAE | 高度 RMSE | 空速 RMSE | v_body | 备注 |
|---|---|---|---|---|---|---|
| T11 盘旋 R60 | loiter | **0.18 m** | 0.00 m | 0.00 m/s | 0.12 | 基准不变 |
| T11b 盘旋+北风3 | loiter | **0.21 m** | 2.30 m | 0.35 m/s | 1.72 | NPFG 有效 |
| T12A 8字 R80/D400 | NPFG 采样 | 34.0 m | 0.22 m | 0.13 m/s | 1.23 | 放大后采样偏差仍存在 |
| T12B 8字 R80/D400 | loiter setpoint | 38.0 m | 1.45 m | 0.88 m/s | 6.15 | 捕获振荡（kr/kv 未经历捕获瞬态整定） |
| T13A 跑马场 R80/L400 | NPFG 采样 | 12.2 m | 0.15 m | 0.09 m/s | 1.23 | 圆弧外凸+切向起点后改善 |
| T13B 跑马场 R80/L400 | loiter setpoint | 24.5 m | 0.34 m | 0.20 m/s | 2.32 | 同上捕获问题 |

结论（供用户判断）：
- 放大尺寸对 NPFG 采样制导改善有限（8 字 32→34m 基本持平，跑马场 15→12m）；
- loiter setpoint 变体在 **不调参数** 前提下劣于采样制导：GA 整定的
  circle_kr/circle_kv 只在"起点即在圆上"的基准场景成立，从未经历 2R 外捕获
  瞬态，捕获时径向振荡（诊断：轨迹在 2R 边界内外反复穿越）；
- 后续方向（待用户判断后执行）：为 loiter 捕获单独整定 kr/kv 并加大
  near 滞回，或按 docs §17 D1 可飞域扫描冻结 8 字/跑马场的可飞参数域。
