# WP2A-8 Controller API 跟踪发散诊断与改进方案

**日期：** 2026-09-06  
**阶段：** Phase 2a / WP2A-8 最终接口校准  
**对象：** `python/controller/controllerAPI.py`、T14 API Loiter、T15 API Heading 8 字试验  
**约束：** 继续使用 tuning_02 内环参数与 tuning_04 外环参数；先修正制导与接口语义，再决定是否重新 GA 整定。

---

## 1. 结论摘要

当前异常的首要原因不是 TECS 或姿态内环失效，而是 API 上层把连续路径转换成 `HeadingCommand` 的方式不正确，以及 Loiter API 缺少明确的捕获状态机。

1. **T14 Loiter 的稳态圆周控制有效，但“指点→切圆”语义尚未实现。**  
   `command_loiter()` 从第一步起就把完整圆路径送入 L1，L1 检测为圆后立即执行 Circle 径向 PD，而不是先飞向指引点、进入 `2R` 后再捕获圆周。因此远场径向误差直接进入 Circle 控制，产生大弧绕入和过冲；进入圆周后则能稳定跟踪。

2. **T15 的稳定蛇形/绕点振荡由单点纯追踪造成。**  
   T15 每个控制周期计算“当前位置指向当前离散航点”的方位角，并以此更新 `HeadingCommand.chi`。这不是连续路径跟踪，而是 pure pursuit 单点追踪。固定翼接近或越过目标点时，目标方位会快速旋转甚至跳变约 180°，导致横向加速度、坡度和偏航指令持续换向。

3. **当前航点推进逻辑既可能突跳，也可能永久卡点。**  
   R=80 m、每圆 100 点时，相邻航点约 5.0 m，而切换阈值是 25 m。进入阈值后会连续多个周期快速跳过若干点；一旦飞机以大于 25 m 的距离错过某一点，索引又不会推进，飞机会持续追逐已经位于身后或侧后的点，形成极限环。

4. **Dubins 不是第一修复项。**  
   当前 8 字几何本身连续，R=80 m 在 V=16 m/s、坡度上限 35° 时可飞。首先应实现“路径投影 + 单调进度 + 弧长前视 + 航向连续化”。Dubins 适合随后用于任意初始姿态到路径的捕获、稀疏航点之间的曲率受限连接，但它本身不能解决 8 字交叉处分支选择、航点进度冻结或近点方位奇异。

5. **T15 的 `cross_track=0` 不是有效结果。**  
   Heading API 每步生成一条从飞机当前位置出发的直线，所以 L1 计算出的最小路径距离天然为零。T15 必须对原始 8 字连续曲线独立计算真实横侧误差。

6. **Heading 的纵向参考也需要修正。**  
   当前 `alt_cmd = h_current + V sin(gamma)/k_h` 每步随实际高度移动。`gamma=0` 时高度参考始终等于当前高度，相当于取消绝对高度恢复，因此 400 s 内出现约 100 m 上漂、记录的高度 RMSE 为 55.2 m。Heading 长动作应维护一个由 `gamma_cmd` 积分得到的内部高度轨迹，而不是每步以当前高度重新锚定。

因此建议的优先级是：

```text
P0 试验与日志可信性
  → P1 Heading 路径进度/前视修复
  → P2 Loiter 捕获状态机
  → P3 Heading 纵向参考与时间戳语义
  → P4 重新试验
  → P5 仅在需要任意姿态/稀疏航点连接时加入 Dubins
  → P6 最后才做局部 GA 整定
```

---

## 2. 当前控制链路与问题边界

当前底层链路为：

```text
Loiter/Heading API
        ↓
ControlReference（路径、V、h）
        ↓
L1/NPFG：位置/航迹 → a_cmd、psi_cmd
        ↓
协调转弯：a_cmd → phi_cmd
        ↓
TECS：V_cmd、h_cmd → theta_cmd、throttle_cmd
        ↓
姿态角环 + 机体系角速率环：phi/theta/psi、p/q/r → 舵面
```

T11 圆、T13 跑马场以及 T14 进入圆周后的结果说明底层闭环可以工作。T15 中内环继续对输入指令做响应，但上层 `chi_cmd` 变化过快且经常反向，把闭环持续激励到原 GA 训练包线之外。因此应先修制导命令生成，不能直接根据 T15 图重调 PID；否则会用内环参数掩盖上层几何错误。

需要澄清：当前协调转弯模块实际执行的是 `a_cmd → phi_cmd`；偏航角速率前馈在内环按 `r_ff = g tan(phi_cmd) cos(theta) / V_TAS` 生成。它不是直接由期望俯仰和滚转角求一个绝对偏航角。

---

## 3. T14 Loiter 诊断

### 3.1 图像证据

`trial_T14_api_loiter.png` 显示：

- 飞机从 `(0, 0)` 出发，圆心为 `(200, 0)`，R=60 m；
- 远场阶段没有近似直线飞向指引点，而是先向 East 负方向大幅绕行，最低约到 -80 m；
- 捕获过程中越过目标圆周外侧，之后才收敛；
- 约 25～30 s 后圆周跟踪稳定，稳态半径误差较小；
- T14 在去除前 20 s 后的径向误差 MAE 约 0.79 m，说明 Circle 稳态律本身不是主要问题。

### 3.2 代码原因

`FlightController._step_loiter()` 始终向 `AvantiController` 传入同一条圆形路径。`L1Controller.step()` 发现 `is_circle.all()` 后，无条件调用 `_circle_guidance()`：

```text
a_cmd = direction × (GS²/R + K_r × (radial-R) + K_v × v_r)
```

初始距离圆心为 200 m，径向误差为 140 m。该大误差从第一步就进入 `circle_kr`，于是控制器立即尝试同时完成向心转弯与径向收敛。这与 API 要求的“先飞向指引点、距离不大于 2R 时开始盘旋”不一致。

### 3.3 改进设计

在 `FlightController` 层加入显式 Loiter 状态，而不是修改已经验证过的 Circle 稳态律：

```text
APPROACH：d_center > 2R
    沿指向圆心的直线制导，固定 h_cmd = center.alt

CAPTURE：进入 d_center <= 2R
    选择与盘旋方向一致的圆周前置/切入点
    对“接近线”和“圆周切向”做连续混合

HOLD：|d_center-R| < e_R 且切向航向误差 < e_chi
    切换到现有 PX4 Circle 径向 PD

RECOVER：HOLD 后若 d_center > 2.5R
    回到 CAPTURE，而不是在 2R 边界来回抖动
```

建议保留 `2R` 作为进入捕获区条件，但不要在边界瞬间硬切绝对航向。可在 `2R→1.2R` 范围内用平滑权重混合接近方向与圆周切向方向，并保持 `phi_cmd`/`chi_cmd` 的角度环绕连续。

Loiter API 还应补充或冻结：

- `direction: int = +1`，明确顺/逆时针；
- 动态最小半径：

  \[
  R_{min}(V)=\max\left(R_{static},\;m\frac{V^2}{g\tan\phi_{max}}\right)
  \]

  其中安全系数 `m` 建议 1.1～1.3。当前 V=16 m/s、`phi_max=35°` 时理论最小半径约 37.3 m，因此固定 `MIN_LOITER_RADIUS=30 m` 对该速度并不总是可飞；R=60 m 本次是可飞的。
- 状态机与指令必须进入 snapshot/restore，满足后续 counterfactual branching。

通用 Dubins 求解器不是 Loiter 捕获的必要条件。第一版采用解析切线/前置点即可；需要从任意姿态严格满足最小转弯半径进入指定方向圆周时，再把捕获段升级为 Dubins CSC。

---

## 4. T15 Heading 8 字诊断

### 4.1 当前调用方式

T15 的外部 hook 执行：

```python
if distance_to_current_wp < 25.0:
    wp_index += 1
chi = atan2(wp.east - pos.east, wp.north - pos.north)
command_heading(V=16, gamma=0, chi=chi)
```

`FlightController._step_heading()` 又以当前位置为起点，沿这个 `chi` 生成 1000 m 直线，并在每一步把 L1 `_ref_index` 清零。

因此实际控制对象不是红色 8 字，而是每 0.02 s 变化一次的“当前位置指向单点”的射线。L1 在这里退化为对瞬时航向误差生成横向加速度，失去了连续路径的横侧误差和沿程信息。

### 4.2 发散机制

1. **近点方位奇异**：目标距离趋近零时，微小位置误差会造成很大的 `chi` 变化；越过点后方位近似翻转 180°。
2. **固定翼不能命中点目标**：飞机受最小转弯半径限制，不能像多旋翼一样停在点上。若错过 25 m 门限，航点索引永久停留。
3. **门限大于点间距**：点间距约 5 m，25 m 门限会造成连续多帧快速推进，`chi_cmd` 呈阶梯突跳。
4. **50 Hz 高层命令过密**：Phase 2 设计建议外环 5～20 Hz；当前航点/方位更新为 50 Hz，且无航向变化率限制和滤波。
5. **8 字交叉歧义**：交叉点附近有两条空间位置相同、路径进度不同的分支。仅按最近距离或目标方位无法判断正确分支，必须保留单调路径进度。
6. **控制包线被持续触发**：T15 记录 `phi` MAE 约 18.45°、`theta` MAE 约 19.19°、最大实际坡度约 66.9°，明显超过 35° 指令限制所对应的正常包线。这是快速反向命令的下游后果，不能据此先调整 PID。

图像中规则重复的小圆/蛇形轨迹正是“绕当前航点形成极限环→偶然进入门限→切到下一点”的典型表现。

### 4.3 Heading API 应保持的语义

`HeadingCommand` 应继续表示一个可长期保持的飞行 primitive：

```text
(V_a_cmd, gamma_cmd, chi_cmd, timestamp)
```

调用者发出命令后，控制器应持续保持该命令，直到收到更新。它不应把“离散目标点”隐式解释为可命中的空间位置。8 字采样点到 Heading 指令的转换应由独立的路径跟随器完成。

同时建议把字段名统一为 `chi` 或 `track_angle`，不要使用 `theta` 表示航迹偏角，因为 `theta` 在现有控制器中已经表示俯仰欧拉角。

---

## 5. 第一优先级：连续路径进度与前视点

### 5.1 推荐算法

对闭合 polyline 预计算累计弧长 `s[i]`，运行时维护每架飞机自己的 `segment_index` 与连续进度 `s_proj`：

1. 只在上一段附近的前向窗口搜索线段，禁止全局跳到交叉处另一分支；
2. 将当前位置正交投影到候选线段，得到 `s_proj` 与有符号横侧误差 `e_ct`；
3. 进度只允许前进，错过航点时通过投影自动跨过，不依赖“进入半径球”；
4. 计算速度相关前视距离：

   \[
   L_{look}=clip(V_g T_{look},L_{min},L_{max})
   \]

   初值可直接采用现有 L1 距离量级。tuning_04 在 V≈16 m/s 时的 L1 距离约为
   `damping × period × V / π ≈ 37.5 m`；建议从 35～50 m 起步。

5. 在 `s_target=s_proj+L_look` 处插值得到虚拟前视点和路径切向 `chi_path`；
6. 发送平滑航迹命令，而不是直接追近点：

   \[
   \chi_{cmd}=\chi_{path}-atan(k_{ct}e_{ct}/\max(V_g,V_{min}))
   \]

   符号按 NEU 路径法向定义校验；也可以先只发送前视点方向，再逐步加入横侧修正。

7. 对角度先 unwrap，再施加物理可行的航向变化率限制：

   \[
   |\dot\chi_{cmd}| \le \frac{g\tan\phi_{max}}{V}
   \]

   V=16 m/s、`phi_max=35°` 时约为 0.43 rad/s（24.6°/s）；工程上可先取 15～20°/s 留出裕度。

8. 路径管理器以 10～20 Hz 更新 HeadingCommand；底层姿态/速率控制仍以 50 Hz 运行并保持最近一次命令。

### 5.2 为什么这一步优于 Dubins

- 8 字本身已经是连续曲线，不需要重新生成另一条几何路径；
- 前视跟踪直接消除“必须命中离散点”的错误约束；
- 单调弧长状态可以明确穿越 8 字交叉点时的分支；
- 可以沿用任意高密度采样路径和后续任务投影面；
- 易于张量化：每个 batch 只维护 `segment_index/s_proj`，在固定前向窗口内批量投影。

建议将其实现为独立的 `PathHeadingAdapter` 或 `PolylinePathFollower`，放在任务/制导层，而不是增加第三种底层控制模式。其输出仍然是标准 `HeadingCommand`，保持 API 边界清晰。

---

## 6. Dubins 基元的合理位置

### 6.1 推荐使用场景

完成第 5 节后，再增加可选 Dubins 预处理器，用于：

- 飞机从任意位置、任意航向进入一条给定路径；
- 两个稀疏航点同时给定进入/离开航向时，生成满足最小转弯半径的连接；
- Loiter 需要严格按指定方向切入圆周；
- TESO 长序列 primitive 之间需要构造确定、可复现、可行的过渡动作。

输入至少应为：

```text
start_pose = (N, E, chi)
goal_pose  = (N, E, chi)
R_min      = f(V, phi_limit)
```

输出应为 CSC/CCC 段及其弧长参数化路径，再交给同一个前视路径跟随器执行。

### 6.2 不应期待 Dubins 解决的问题

Dubins 不会自动解决：

- 8 字交叉点该走哪一支；
- 航点索引卡住或倒退；
- 目标点近距离方位奇异；
- `chi_cmd` 角度 unwrap、限速与时间戳管理；
- gamma/高度参考漂移；
- 曲率突变。标准 Dubins 在直线/圆弧连接处只有切向连续，曲率会阶跃；若后续对滚转平顺性要求更高，应在 Dubins 后加 clothoid/曲率渐变段，而不是直接提高 PID 增益。

因此本阶段的结论是：**Dubins 值得纳入接口层的可选路径基元，但不是修复 T15 的前置条件，也不应替代 L1/NPFG 或路径进度管理。**

---

## 7. Heading 纵向接口修正

当前 `ControlReference.gamma_cmd` 已存在，但 `tecs.py` 没有使用它；API 通过每步移动 `alt_cmd` 间接构造 gamma。建议分两步修改。

### 7.1 最小风险修复

Heading 命令首次生效时锁存内部 `h_ref`，之后按控制器时间步积分：

\[
h_{ref,k+1}=h_{ref,k}+V_{a,cmd}\sin(\gamma_{cmd})\Delta t
\]

- `gamma=0` 时 `h_ref` 固定，恢复高度误差反馈；
- 非零 gamma 时得到连续高度斜坡；
- 更新同一 HeadingCommand 时不重置 TECS 积分器，也不重新锁存高度；
- 模式切换或显式 reset 时才重新初始化 `h_ref`。

### 7.2 后续 TECS 接口升级

让 TECS 显式使用 `gamma_cmd` 作为前馈，同时用积分高度轨迹做慢速漂移修正，例如：

\[
\gamma_{sp}=clip\left(\gamma_{cmd}+k_h\frac{h_{ref}-h}{V},\;\gamma_{min},\gamma_{max}\right)
\]

该修改会改变 TECS 动态，应在路径管理修复后单独做阶跃试验和局部整定，不能与横向修复同时 GA。

### 7.3 时间戳语义

`timestamp` 当前只被保存，没有参与校验和状态更新。标准接口至少需要：

- 拒绝或忽略早于最近命令的时间戳；
- 明确时间戳是命令生效时间，不是持续时间；
- snapshot 包含最近时间戳、当前模式、命令内容、Loiter 状态、路径进度和 `h_ref`；
- 由整数 tick 或统一仿真时钟计算 `dt`，避免浮点累计和 Isaac 全局时间漂移。

---

## 8. 日志与试验口径修正（P0）

在继续改控制器前，先保证试验产物可重复、指标真实。

### 8.1 当前产物问题

- `experiments/wp2a8/quality_trials.py` 当前仍含 Git 冲突标记；
- `experiments/wp2a8/quality_trials/summary.json` 也含冲突标记，不是合法 JSON；
- T15 的 `cross_track` 取自“当前位置起始的临时直线”，恒为零，不能用于评价 8 字跟踪；
- 现有绘图把 `phi/theta/psi` 标为“体轴姿态角”，应改为“姿态欧拉角”；`p/q/r` 才是机体系角速率。

这些问题不改变上述图像诊断，但必须在下一次正式运行前清理，否则试验无法可靠重放。

### 8.2 新增日志字段

建议至少记录：

```text
api_mode / loiter_state
command_timestamp
chi_cmd_raw / chi_cmd_limited / chi_cmd_rate
path_segment_index / s_proj / s_target
target_north / target_east
true_cross_track / signed_cross_track
distance_to_loiter_center / radial_error
turn_radius_required / turn_radius_min
heading_command_update_flag
```

T15 的位置误差必须相对原始连续 8 字 polyline 计算；在交叉点附近，误差计算使用当前路径进度窗口，避免误投影到另一支。

---

## 9. 分步实施计划

### SP2-D0：清理基线与建立可诊断日志

1. 清理 `quality_trials.py` 和 `summary.json` 的冲突标记；
2. 将 T14/T15 从全套 T11～T15 脚本中拆成可单独运行的测试入口，避免每次调试重复运行旧任务；
3. 加入第 8.2 节日志；
4. 保存当前 T14/T15 为 `baseline_api_v1`，不得覆盖；
5. 给统计函数增加按状态分段：approach、capture、steady。

### SP2-D1：只修 T15 路径跟随器

1. 删除“距离小于 25 m 才 `index += 1`”逻辑；
2. 实现局部线段投影、单调弧长进度和弧长前视点；
3. 8 字交叉点通过前向窗口和累计弧长消歧；
4. 路径管理更新率先设 10 Hz，Heading API/底层控制保持 50 Hz；
5. 对 `chi_cmd` unwrap，并施加 15～20°/s 初始限速；
6. 暂不改 tuning_04 参数，重跑 400 s T15。

### SP2-D2：修 Heading gamma/高度参考

1. Heading 模式锁存并积分 `h_ref`；
2. gamma=0 连续直飞 400 s，检查高度漂移；
3. gamma=`0→+5°→0→-5°→0` 阶跃试验；
4. 验证空速、gamma、油门、俯仰、垂直速度，不用当前位置高度作为“期望高度”；
5. 需要时再启用 `ControlReference.gamma_cmd` 的 TECS 前馈。

### SP2-D3：实现 T14 Loiter 捕获状态机

1. 加入 APPROACH/CAPTURE/HOLD/RECOVER；
2. `d>2R` 时执行到圆心的接近制导；
3. `d<=2R` 时选择方向一致的切入前置点，并平滑混合；
4. 稳态后调用现有 Circle 制导；
5. 增加 2R/2.5R 滞回；
6. 以 4 个初始方位、3 个初始航向、R={40,60,80} 做捕获矩阵。

### SP2-D4：判断是否引入 Dubins

满足以下任一条件时实现 Dubins：

- T14 多初始姿态捕获仍存在不可接受的回转或错误盘旋方向；
- 后续 primitive 给出了目标位置与目标航向，而非仅给出连续路径；
- 稀疏航点连接经前视跟踪后仍违反最小转弯半径。

Dubins 作为路径预处理器输出弧长参数化段，不直接输出舵面或替代 NPFG。

### SP2-D5：局部参数整定与回归

几何和状态机固定后，只整定新增参数：

```text
T_look / L_look_min / L_look_max
k_ct
chi_rate_limit
loiter_capture_blend_width
loiter_hold_radial_tol
loiter_recover_radius
```

首先网格扫描，再视需要使用限定记忆 GA。不要把内环 PID、TECS、L1、捕获和前视参数一次性混合优化。

---

## 10. 验收标准

### T14 API Loiter（400 s）

- `d>2R` 阶段不存在 Circle 径向 PD 直接饱和；
- 只发生一次 APPROACH→CAPTURE→HOLD，不在 2R 边界反复切换；
- 进入 HOLD 后径向 MAE ≤ 2 m，建议目标 ≤ 1 m；
- 捕获后不再次越出 2.5R；
- 最大实际坡度不超过允许包线加合理瞬态裕度；
- 高度/空速质量不劣于 T11/T14 当前稳态基线。

### T15 API Heading 8 字（400 s）

- 完成至少 3 个完整 8 字周期，路径进度持续单调且交叉点不串支；
- 不存在对单一航点的持续绕圈；
- 使用原始 8 字曲线计算的横侧 MAE 首轮目标 < 15 m，收敛目标 < 8 m；
- `chi_cmd` 无近点 180° 跳变，变化率不超过配置限制；
- 最大实际坡度回到可飞包线，舵面长期饱和比例显著下降；
- gamma=0 时 400 s 高度漂移 < 10 m，随后再收紧；
- 无 NaN/Inf，snapshot/restore 后控制状态可复现。

### 回归

- T11、T11b、T12、T13 不得因 API 层修改而退化；
- 直飞恒定 `chi/gamma/V` 的单元试验必须先通过，再运行路径适配器；
- 标量 B=1 与 batch 模式逐 tick 对拍一致。

---

## 11. 最终建议

下一步先实现 `PolylinePathFollower → HeadingCommand`，而不是直接引入 Dubins 或重新调整 PID。该修改最小、诊断性最强，也最贴合 Heading API 作为长期动作 primitive 的定位。

完成路径投影、单调进度和前视点后，如果 T15 蛇形消失，就能证明主要问题在离散航点选择；之后再独立处理 Loiter 捕获和 gamma 高度参考。Dubins 放在这些基础能力之后，用作曲率受限的“连接路径生成器”，而不是航点选择器或控制器替代品。
