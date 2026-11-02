# WP2A Controller API 自适应路径取样设计与试验总结

**日期：** 2026-09-06  
**阶段：** Phase 2a / WP2A-8 Controller API 校准  
**范围：** 先修正 Heading 模式的离散路径取样，不引入 Dubins，不重新整定 tuning_02/tuning_04 控制参数。

---

## 1. 本轮结论

T15 旧试验中的稳定蛇形与逐步发散，已经确认并通过路径取样适配修复。根因不是固定的“25 m 数值太大”这么单一，而是原算法把固定翼连续路径跟踪错误地实现成了“必须进入 25 m 球的单点 pure pursuit”：

- 相邻采样点约 5 m，25 m 阈值会造成短时间连续跳点；
- 错过目标点且距离不再小于 25 m 时，索引永久卡住；
- 接近或飞越点目标时，目标方位角快速旋转或翻转；
- 8 字交叉处仅凭位置不能区分前后两个路径分支；
- 50 Hz 重算目标方位把上述不连续持续注入姿态闭环。

本轮使用以下连续路径适配链替换固定半径切点逻辑：

```text
采样路径 polyline
    → 局部线段投影
    → 单调累计弧长进度
    → 速度相关弧长前视点
    → 有符号横侧误差修正
    → 航迹偏角 unwrap + 变化率限制
    → HeadingCommand(V, gamma, chi, timestamp)
    → 原 AvantiController
```

400 s 结果表明，不使用 Dubins 已足以消除 T15 的绕点极限环并稳定完成 8 字飞行。

---

## 2. 实现内容

### 2.1 `PolylinePathFollower`

在 `python/controller/controllerAPI.py` 新增：

- `PolylinePathFollower`：连续采样路径到 HeadingCommand 的状态适配器；
- `PathFollowerStatus`：每次更新的诊断状态；
- `get_snapshot()/set_snapshot()`：保存和恢复路径进度、时间戳及航向限速状态。

`HeadingCommand` 的标准语义没有改变，仍然只表示：

```text
airspeed：期望空速，m/s
gamma：期望航迹倾角，rad
chi：期望航迹偏角，rad（North=0，East=+pi/2）
timestamp：命令生效时间，s
```

路径跟随器是 Heading API 的上游适配器，不构成第三种控制模式，也不改变 NPFG、TECS、协调转弯和姿态 PID 的职责边界。

### 2.2 局部路径投影

路径初始化时预计算每条线段的水平长度与累计弧长。运行时只在当前进度附近的有限前向窗口内搜索投影点：

- 默认向后检查 2 段，用于吸收定位与离散误差；
- 默认向前检查 20 段；
- 单次允许的弧长前进量受地速、更新时间和线段长度共同限制；
- 最终进度不允许倒退。

该设计使飞机错过某个采样点后仍能沿路径继续前进，并阻止 8 字交叉点跳到另一条几何重合但弧长相差约半圈的分支。

### 2.3 速度自适应前视

前视距离为：

\[
L_{look}=clip(V_gT_{look},L_{min},L_{max})
\]

本轮参数：

| 参数 | 数值 |
|---|---:|
| `T_look` | 2.5 s |
| `L_min` | 35 m |
| `L_max` | 55 m |
| V≈16 m/s 时平均前视 | 40.01 m |

该量级与 tuning_04 的 L1 距离一致：

\[
L_1=\frac{\xi TV}{\pi}
\approx\frac{0.619\times11.913\times16}{\pi}
\approx37.5\ m
\]

因此本轮不是重新整定 L1，而是在上游用相同时间尺度选择稳定的虚拟参考点。

### 2.4 横侧修正与航向限速

由当前投影点到弧长前视点的弦方向得到 `chi_path`，以路径当前切向为基准计算有符号横侧误差 `e_ct`：

\[
\chi_{raw}=wrap\left(\chi_{path}
-atan2(k_{ct}e_{ct},L_{look})\right)
\]

本轮 `k_ct=1.0`。航向先做角度环绕连续化，再按 18°/s 限速。路径适配器每 0.1 s 更新一次命令，底层控制器仍以 50 Hz 执行并保持最近命令：

- 高层命令更新率：10 Hz；
- 每次最大航向变化：1.8°；
- 实测最大命令更新率：18.0°/s。

### 2.5 真实横侧误差

旧 Heading API 每步创建一条从当前位置开始的直线，导致 L1 内部 `cross_track` 恒为零。本轮由路径适配器相对原始 8 字路径计算 `path_cross_track_m`，并在 T15 统计时覆盖无意义的临时直线误差。

新增诊断字段：

```text
path_segment_index
path_lap_count
path_progress_m
path_lookahead_m
path_cross_track_m
path_target_north/east/alt
chi_cmd_raw
chi_cmd_limited
path_command_updated
```

---

## 3. T15 400 秒试验

### 3.1 试验条件

| 项目 | 设置 |
|---|---|
| 路径 | 两个相切圆组成的平面 8 字 |
| 圆半径 | 80 m |
| 圆心参考 | `(200, 0)` NE |
| 高度 | 1000 m |
| 空速指令 | 16 m/s |
| gamma 指令 | 0 rad |
| 风 | 0 m/s |
| 时长 | 400 s |
| 内环参数 | tuning_02 |
| 外环参数 | tuning_04 |
| Dubins | 未使用 |

### 3.2 结果

| 指标 | 旧单点追踪 | 自适应弧长前视 |
|---|---:|---:|
| 轨迹状态 | 绕点振荡并持续漂离 | 捕获后稳定重复 8 字 |
| 有效横侧 MAE | 无效（记录为 0） | **2.47 m** |
| 40 s 后横侧 MAE | 未形成稳定路径 | **2.43 m** |
| 40 s 后横侧 95 分位 | 未形成稳定路径 | **2.68 m** |
| 40 s 后最大横侧误差 | 持续发散 | **2.74 m** |
| 完成整圈 | 0 个有效闭合周期 | **6 圈** |
| 路径进度 | 航点卡住/跳变 | **全程单调** |
| 高度 RMSE | 55.20 m | **0.99 m** |
| 400 s 高度漂移 | 约 +100 m | **-1.04 m** |
| 空速 RMSE | 0.737 m/s | **0.026 m/s** |
| phi 跟踪 MAE | 18.45° | **5.94°** |
| theta 跟踪 MAE | 19.19° | **1.88°** |
| 最大实际坡度 | 66.92° | **45.52°** |
| 航向更新率 | 无限制 | **18.0°/s** |
| NaN/Inf | 无 | 无 |

两次复跑得到相同统计值，当前 Windows DLL + Python 3.10 运行具有确定性。

### 3.3 结果解释

路径问题已经得到验证性修复：

- 不再要求飞机命中任一离散采样点；
- 进度不会因错过点目标而冻结；
- 8 字中心交叉处没有串支；
- 捕获瞬态结束后误差形成稳定、有限的周期；
- 平滑横向指令同时消除了旧 T15 的高度和空速大幅偏移，说明此前纵向异常主要由激烈转向及转弯补偿持续激励引起。

目前左右圆上分别存在约 ±2.4 m 的稳定有符号偏差，这是曲率方向改变时基于 Heading 的前视跟踪固有稳态偏差，可在后续通过 `k_ct/L_look` 小范围扫描或加入曲率前馈改善，不需要引入 Dubins。

---

## 4. 尚未解决的问题

### 4.1 执行器与角速率高频抖振

对 60 s 以后数据统计：

| 通道 | `|command| >= 0.99` 占比 | 95 分位绝对值 |
|---|---:|---:|
| 副翼 `delta_a` | 48.6% | 1.00 |
| 升降舵 `delta_e` | 0% | 0.574 |
| 方向舵 `delta_r` | 19.2% | 1.00 |

这一现象在既有 T11/T13 角速率图中也可见，说明它不是新路径适配器单独造成的。路径跟踪虽已稳定，但如此高的舵面饱和比例不适合直接作为最终数据生成控制器。下一轮应检查：

- 角速度前馈差分 `rate_sp_dot` 是否放大 50 Hz 离散阶跃；
- 偏航绝对角跟踪与协调转弯 `r_ff` 是否重复施加；
- `rate_ff`、角速度 P/PI 与空速缩放是否导致采样级极限环；
- 是否应对 `rate_sp` 或舵面加入一阶滤波/舵速限制。

这属于内环控制品质问题，应在本轮路径取样修复冻结后单独诊断，不应通过改变前视距离掩盖。

### 4.2 8 字中心曲率换向

当前路径在相切点方向连续，但曲率从 `+1/R` 直接变为 `-1/R`。协调转弯要求坡度换向，最大实际坡度仍达到 45.5°。短期可通过航向变化率、坡度前馈和局部曲率渐变减小瞬态；本阶段仍不需要 Dubins。若需要曲率连续，应考虑 clothoid/滚转过渡段，而不是标准 Dubins 的直线—圆弧硬连接。

### 4.3 Heading gamma 的长期参考

本次平滑横向制导后 gamma=0 的高度漂移仅 -1.04 m，已满足本轮试验。但当前 API 仍通过“当前高度 + gamma 等效偏置”构造 TECS 高度命令，结构上没有长期高度轨迹状态。该问题保留到路径适配冻结后单独升级，避免一次修改横向和纵向两条链路。

### 4.4 Loiter 捕获

本轮未修改 T14。Loiter 仍需后续实现 APPROACH/CAPTURE/HOLD 状态机，满足“距离大于 2R 时接近指引点，进入 2R 后切圆”的接口定义。

---

## 5. 验证与产物

### 5.1 测试

`python/tests/test_controller_interfaces.py` 新增 3 项测试：

1. 前视距离、有符号横侧修正和 18°/s 航向限速；
2. 8 字交叉点分支保持及进度不跳变；
3. 路径跟随器 snapshot/restore 逐步结果一致。

直接执行结果：

```text
python package unit tests passed
```

当前 Python 环境未安装 `pytest`，因此使用项目原有的直接执行测试入口完成验证；不是测试失败。

### 5.2 代码与试验入口

- `python/controller/controllerAPI.py`：路径适配器实现；
- `python/controller/__init__.py`：公开导出；
- `python/tests/test_controller_interfaces.py`：接口测试；
- `experiments/wp2a8/quality_trials.py`：正式 T15 使用新适配器；
- `experiments/wp2a8/api_heading_adaptive_trial.py`：只运行新 T15，不归档或覆盖旧任务。

独立复现实验命令：

```powershell
python experiments/wp2a8/api_heading_adaptive_trial.py
```

### 5.3 输出

- `quality_trials/trial_T15_api_heading_figure8_adaptive.png`：轨迹、位置、姿态和角速率；
- `quality_trials/trial_T15_api_heading_figure8_adaptive_guidance.png`：横侧误差、路径进度、航向命令和前视距离；
- `quality_trials/summary_T15_api_heading_adaptive.json`：结构化统计；
- `quality_trials/T15_api_heading_figure8_adaptive_log.npz`：完整逐 tick 日志。

---

## 6. 下一步建议

按以下顺序继续 WP2A-8：

1. 冻结本轮 `PolylinePathFollower` 作为 Heading 路径采样基线；
2. 对 `lookahead_time={2.0,2.5,3.0}`、`k_ct={0.8,1.0,1.2}` 做小型网格扫描，目标是保持横侧 MAE 的同时降低中心换向峰值；
3. 单独诊断副翼/方向舵高频饱和，不混合修改路径参数与 PID；
4. 完成 Loiter APPROACH/CAPTURE/HOLD 状态机；
5. 最后处理 Heading gamma 的内部高度轨迹和完整 API snapshot；
6. 上述项目通过后再决定是否为任意姿态捕获增加 Dubins。当前 T15 结果不支持把 Dubins 列为必需依赖。

本轮结论可以概括为：**固定翼不应追踪离散点本身，而应追踪带单调进度的连续路径前视状态。**
