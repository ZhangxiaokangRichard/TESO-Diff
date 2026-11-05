# WP2A-endpoint：四科目 × 三风场 avanti-eso 张量数据集采集与验证

2026-11-04，Windows / Python 3.10 / `avanti2_att.dll`（生成姿态内环 + wp2a8b
冻结外环基线）。本目录是 Phase 2a → 2b 的收尾验证：在 CPU 环境下对
8 字（S2）、盘旋爬升（S1）、跑马场（S3）、GPS 三角（S4）四条封闭航迹，
在 W0 无风 / W1 热气流 / W2 梯度切变三个固定风场中完成 300 秒闭环飞行，
采集并校验 avanti-eso 动作-轨迹获能张量数据集（12 条 run，每条 15000 tick，
50 Hz）。通过标准：通道完整、全程有限、时间严格 0.02 s、弧长进度单调、
W0 零获能严格成立、W1/W2 能量通道物理方向正确。12/12 全部通过。

本目录只做试验与验证，不做可复用 API；风场/场地/记录器/任务入口的
可复用模块化（python/simulation 下）按计划在本试验稳定后另行设计。

## 试验设置

- **对象与控制链**：`AttitudeFleet(batch=3)`，三个实例分别处于 W0/W1/W2；
  Python 批量 NPFG/TECS 外环（wp2a8b 冻结适配参数，经
  `config/controller/attitude_runtime.yaml` 加载）→ C++ 生成姿态内环 + 6DOF。
  四科目统一经 `PolylinePathFollower` → `HeadingCommand` 跟踪；盘旋爬升用
  gamma = atan(0.8/16) ≈ 2.86°（+0.8 m/s 爬升，高度参考独立积分，100→342 m）。
- **场地**（[routes.py](routes.py)，固定几何、无随机量）：基准高度 100 m，
  IAS 指令 16 m/s，起点 (0,0,100)。8 字两相切圆 r=80（圆心 (120,0)/(280,0)，
  交叉点 (200,0)）；盘旋圆 r=80 圆心 (200,0)；跑马场直线 240 + 半圆 r=80
  （复用 `task_route.racetrack_path`）；GPS 三角顶点 (40,0)/(340,−170)/(340,170)、
  圆角 r=70（新实现的相切圆角闭合多边形）。投影面随机化留 Phase 2b projector。
- **风场**（[wind_fields.py](wind_fields.py)，flat-Earth 输出、按实例位置逐
  tick 查询）：W1 高斯热柱，中心 (200,0)、R=90 m、峰值 2.0 m/s、试验高度带内
  按高度不变合成柱处理；W2 静态 tanh 水平切变线，N=200 m 处东风 0→4 m/s、
  过渡半宽 60 m。控制器使用局部风真值（已知风假设，无估计噪声）；风输入
  取自上一 tick 观测位置，存在 1 tick 输运延迟（meta 已记录）。
- **能量真值**：p_env = g·w_up − v_air·Ẇ（由风真值解析导出，推导与边界见
  [dataset_structure.md](dataset_structure.md)）。无气动/推力遥测，docs §25
  的完整 Y_E 标签留 WP2B-17。

## 结果总表（300 s，稳态窗 t>30 s / 横侧 t>60 s）

| 科目 | 风场 | 圈数 | IAS RMSE m/s | 高度 RMSE m | 横侧 MAE / P95 m | 累计获能 J/kg | 风场专项 |
|---|---|---:|---:|---:|---|---:|---|
| 8 字 | W0 | 4 | 0.216 | 0.858 | 2.26 / 4.24 | 0.0（严格） | ė_a 均值 +0.003 W/kg |
| 8 字 | W1 | 5 | 0.687 | 4.973 | 2.41 / 4.57 | +2055.4 | 核心区 p_env 均值 +14.47；油门-上升气流相关 −0.69 |
| 8 字 | W2 | 4 | 0.709 | 1.357 | 2.49 / 4.30 | −5.0 | 穿越切变线 9 次；\|p_shear\| 均值 1.045 |
| 盘旋爬升 | W0 | 9 | 0.140 | 0.248 | 2.54 / 2.57 | 0.0（严格） | ė_a 均值 +7.95（爬升储能） |
| 盘旋爬升 | W1 | 9 | 0.118 | 0.851 | 2.53 / 2.56 | +2483.3 | 94.2% tick 在热柱内，p_env 均值 +8.46 |
| 盘旋爬升 | W2 | 9 | 0.390 | 0.540 | 1.58 / 2.42 | −2.2 | 穿越 19 次；\|p_shear\| 均值 1.371 |
| 跑马场 | W0 | 4 | 0.077 | 0.737 | 1.44 / 3.36 | 0.0（严格） | ė_a 均值 −0.002 |
| 跑马场 | W1 | 4 | 0.161 | 1.616 | 1.45 / 3.40 | +904.9 | 核心区 p_env 均值 +8.28；相关 −0.93 |
| 跑马场 | W2 | 4 | 0.482 | 1.034 | 1.22 / 3.13 | −8.5 | 穿越 10 次；\|p_shear\| 均值 0.286 |
| GPS 三角 | W0 | 6 | 0.069 | 0.759 | 1.99 / 4.03 | 0.0（严格） | ė_a 均值 +0.004 |
| GPS 三角 | W1 | 6 | 0.193 | 2.231 | 2.02 / 4.09 | +1269.8 | 核心区 p_env 均值 +8.00；相关 −0.90 |
| GPS 三角 | W2 | 6 | 0.529 | 1.100 | 1.69 / 3.87 | +19.2 | 穿越 13 次；\|p_shear\| 均值 1.502 |

全部 12 条 run：通道完整（32 通道）、全程有限、时间严格 0.02 s 均匀、
相对时间从 0 起、弧长进度单调不倒退。逐 run 完整指标见
[metrics/summary.json](metrics/summary.json) 与各 `data/*.meta.json`。

**能量物理校验**：W0 四条 run 的 wind 与 p_env 逐 tick 严格为 0（E0 类零
获能基线成立；W0 的 ė_a 残差即推力−阻力项，巡航科目均值 ≈±0.004 W/kg，
爬升科目 ≈+7.95 W/kg ≈ g×0.8 m/s 储能率，量级自洽）。W1 四条 run 热柱
核心区 p_env 均为正，8 字穿核峰值 19.2 W/kg ≈ g×2.0 m/s 理论上限；巡航
科目油门与上升气流强相关为负（−0.69 ~ −0.93，TECS 以收油门置换环境
获能）；盘旋爬升因 94% 时间处于近恒定 w_up，该相关不具诊断意义（+0.21，
已在 meta 标注）。W2 四条 run p_shear 逐穿越呈 ± 对称尖峰，300 s 净获能
接近 0（−8.5 ~ +19.2 J/kg）——未主动利用梯度时无净提取，符合预期；GPS
三角两条斜边的 ± 获能条带在热力图上清晰可分（供编码器学习机制方向性）。

**已知限制**：8 字 × W1 中热柱强度（2.0 m/s）超过怠速平飞下沉率，TECS
油门触底比例 17.7%、穿核时高度浮升约 +8 m（高度 RMSE 4.97 m 由此而来），
属对象物理行为而非跟踪失效；W2 下切圆最大滚转 37.0°（指令限幅 35°，与
wp2a8b 一致的内环瞬态超调）。无舵面遥测、无传感噪声/风估计误差（D5 再
引入）、无 counterfactual 分支；npz 为验证级落盘，HDF5 writer 留 WP2B-20。

## 发现并修复的缺陷：PolylinePathFollower 整圈边界死锁

首轮采集中“盘旋爬升 × W0”在第 7 圈边界（progress = 7×周长 =
3518.4833 m）处弧长进度永久冻结，chi 指令固定 89.7°，飞机脱离圆航线
直飞约 1100 m。根因：`_project_local()` 中 `progress % total` 与
`floor(progress / total)` 在整圈倍数处的浮点舍入可各差一圈（前者判为
第 6 圈末、后者判为第 7 圈），组合出超前一整圈的 unwrapped 段索引，
候选投影全部落在前向窗口之外 → `best=None` → 进度不再更新，状态逐
更新自复制形成死锁。修复：lap 与圈内弧长由同一次 floor 一致导出并
夹紧（[controllerAPI.py](../../python/controller/controllerAPI.py)
`_project_local`），新增回归测试
`test_polyline_path_follower_lap_boundary_no_deadlock`（1/7/13 圈边界）。
修复后全部 run 重新采集，盘旋爬升 W0 恢复 9 圈、横侧 MAE 4.97→2.54 m。
wp2a8b 的 8 字结果未触发该边界（其进度未恰好钳位到整圈浮点倍数），
历史结论不受影响。

## 产物清单

```text
data/eso_<subject>_<W>.npz        12 条 avanti-eso 张量 run（32 通道 × 15000 tick）
data/eso_<subject>_<W>.meta.json  冻结参数 + 坐标系约定 + 逐 run 校验结果
data/sample_preview.txt           人读样本（figure8×W1 穿核段）
metrics/summary.json              12 run 指标汇总 + 生成内环增益快照
figures/wind_env_W{0,1,2}.png     二维风场试验环境图（z=100 m 切片 + 航线）
figures/traj_<subject>.png        每科目 W0/W1/W2 轨迹/高度/IAS/横侧/p_env/累计获能
figures/heatmap_W{0,1,2}.png      全局获能热力图（全科目合并 + 分科目 + 风场底图）
dataset_structure.md              数据集结构说明（通道表 + §31 HDF5 映射）
provenance.json                   DLL/源码/配置/数据 SHA256 与运行环境
```

性能参考：batch=3 fleet 串行推进 300 s×3 实例耗时 14.8~17.7 s 墙钟
（约 2900 instance-ticks/s，合 51~61 倍实时），无 Isaac 依赖。

## 复现

```powershell
# 项目根目录；PYTHONUSERBASE 视本机 numpy 安装位置而定
python python/tests/test_controller_interfaces.py
python experiments/wp2a_endpoint/run_endpoint.py --seconds 300
python experiments/wp2a_endpoint/plots.py           # 仅重绘
python experiments/wp2a_endpoint/record_provenance.py
```

运行为确定性（固定初始状态、无随机量、已知风真值），同配置重跑应得到
逐值一致的 npz。当前结论：四科目 × 三风场的编码器输入输出真值流
（风真值 / 动作与控制器中间量 / 状态 / 双通道时间 / 获能标签原料）在
CPU 环境下采集与验证闭环成立，可作为 Phase 2b 风场/场地/录制/任务入口
可复用模块与 HDF5 writer 的行为基准。
