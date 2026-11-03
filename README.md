# TESO-Diff — 固定翼自主获能滑翔 Trajectory-conditioned Energy Space Observer with Diffusion

Phase 2：可规模化生成可信 TESO 数据的仿真基础设施。

- 设计文档：`docs/TESO-Diff_phase2.md`（Phase 2 总计划）、`docs/TESO-Diff_phase1.md`
  （AVANTI2 6DOF 动态库）、`docs/fixedwing_interface.md`（C ABI 实测记录）、
  `docs/TESO-Diff.pdf`（TESO-Diff 总体设计）。
- Phase 1 项目：`~/AVANTI_6DOFmodel`（Simulink/Embedded Coder 生成与 C ABI 冻结），
  其运行时产物（生成源码 + 手写 C ABI bridge + `.so` + Python ctypes 封装）
  已复制进本项目的 `runtime/` 与 `python/bindings/`。

## 目录结构

```text
TESO-Diff/
├── CMakeLists.txt              新旧两个runtime的统一构建入口
├── config/
│   ├── aircraft/avanti2.yaml   机型/plant 冻结参数
│   ├── aircraft/avanti2_att.yaml  内置姿态环对象及编译期默认增益
│   ├── controller/controller.yaml  TECS/L1/协调转弯/PID 参数
│   ├── controller/attitude_runtime.yaml  WP2A-8b验证后的NPFG/TECS外环配置
│   ├── task/                   S1~S4 封闭航迹任务（circle/figure8/racetrack/gps_triangle）
│   ├── environment/            W0~W3 风场矩阵 + wind_smoke
│   └── dataset/smoke.yaml      数据集 smoke 配置
├── docs/                       设计文档 + build_logs/
├── runtime/
│   ├── fixedwing/              AVANTI2 6DOF 运行时
│   │   ├── include/            公共头文件（含 avanti_c_api.h）
│   │   ├── generated/          Embedded Coder 生成源码
│   │   ├── wrapper/            手写 C ABI bridge
│   │   ├── lib/                预构建 libavanti2.so（phase1 Ubuntu 构建）
│   │   └── CMakeLists.txt
│   ├── fixedwing_att/          PX4Controller生成的姿态环+6DOF整体对象
│   │   ├── include/ generated/ wrapper/  自包含C++源码与C ABI
│   │   └── lib/               avanti2_att.dll / libavanti2_att.so
│   └── common/                两对象共享的CMake构建规则
├── python/
│   ├── bindings/               avanti.py / avanti_att.py，按平台选择DLL或SO
│   ├── controller/             Python 张量并行控制器（tecs/l1/coordinated_turn/pid/outer/inner）
│   ├── eso/                    avanti_eso 张量收集（history/tensor_recorder）
│   ├── simulation/             闭环 harness + wind stub
│   └── tests/                  包级单测
├── tests/
│   ├── cpp/                    C++ 20s 回归（CTest）
│   ├── reference/              Simulink 20s 基准 CSV
│   └── regression/             ctypes 20s/并行/生命周期/依赖审计
├── isaac/                      Phase 2b Isaac Lab 并行环境（规划占位）
├── dataset/                    schema/smoke/formal（Phase 2b 输出）
└── scripts/                    build_all / run_phase2a_smoke / run_phase2a_tasks / benchmark_runtime
```

## 快速开始

默认同时构建旧舵面输入对象和新姿态输入对象，无需安装MATLAB。产物先输出到
`build/<platform>/dist`，再复制共享库到各自的 `runtime/*/lib`。

```bash
# 1. 构建 runtime（需要 cmake/g++，无需 MATLAB）
scripts/build_all.sh

# 2. 接口级冒烟（依赖审计 + 20s 回归 + 16 实例并行 + 生命周期）
scripts/run_phase2a_smoke.sh

# 3. 吞吐 benchmark
python3 scripts/benchmark_runtime.py --envs 1 8 32 --seconds 60
```

其中benchmark和原封闭航迹入口仍针对旧舵面输入对象；新对象接口测试使用
`python tests/regression/test_att_runtime.py`，不混用两种输入结构。

Windows（Python命令为`python`，MinGW构建）：

```powershell
./scripts/build_all.ps1 -Compiler 'C:/Program Files/mingw64/bin/g++.exe'
# 仅编译新对象；旧对象可用 -Runtime fixedwing
./scripts/build_all.ps1 -Runtime fixedwing_att
```

Ubuntu原生构建可用 `scripts/build_all.sh build/linux64 fixedwing_att` 选择新对象，
最后一参数也支持 `all` 和 `fixedwing`。CMake对应开关为
`AVANTI_BUILD_FIXEDWING`、`AVANTI_BUILD_FIXEDWING_ATT`，默认均ON；
`AVANTI_STAGE_LIBS=OFF` 可关闭向runtime目录复制。

Windows交叉构建Linux x86_64/glibc2.35产物（需单独提供Zig，不安装WSL）：

```powershell
./scripts/build_linux_cross.ps1 -ZigExe build/toolchains/cross/zig-windows-x86_64-0.13.0/zig.exe `
  -MakeProgram 'C:/Program Files/mingw64/bin/mingw32-make.exe'
```

交叉产物在Ubuntu上直接运行 `build/linux64/dist/avanti_cpp_20s`、
`build/linux64/dist/avanti_att_cpp_smoke` 及
`python3 tests/regression/test_att_runtime.py --library runtime/fixedwing_att/lib/libavanti2_att.so`。
Windows生成的CMake缓存包含主机绝对路径，不能直接迁移后运行CTest；
Ubuntu原生重编译请使用新目录，如 `scripts/build_all.sh build/ubuntu22-native`。
Windows不能直接加载Linux ELF库。工具链来源及跨平台支持见
[Zig官方发布说明](https://ziglang.org/download/0.13.0/release-notes.html)。

## 两种对象的控制边界

| 对象 | 输入 | Python封装 | 控制链 |
|---|---|---|---|
| `fixedwing` / `avanti2` | 三舵面、油门、三维风 | `bindings.avanti` | Python外环与姿态内环→6DOF |
| `fixedwing_att` / `avanti2_att` | phi_sp、theta_sp、油门、三维风 | `bindings.avanti_att` | Python NPFG/L1、TECS→生成的姿态环+6DOF |

新对象接收角度单位为rad，输出姿态仍为deg、位置和垂直速度Up-positive。
底层固定0.02s外部时钟推进。新链路使用 `FlightController` 的NPFG/TECS输出
`phi_cmd/theta_cmd/throttle`，其期望航向输出用于制导诊断；偏航协调已在库内完成。
不要把Python `AvantiController.inner` 的舵面指令送入新对象，也不要重复叠加偏航前馈。

```python
from python.bindings.avanti_att import AvantiAttInput, AvantiAttLibrary
library = AvantiAttLibrary()
with library.model() as aircraft:
    command = AvantiAttInput(phi_sp_rad=0.0, theta_sp_rad=0.05, throttle_01=0.7)
    output = aircraft.tick(command)  # 一次0.02s；输出对应本次major tick
```

14个可调增益是**进程共享**参数，状态和积分器按实例独立；有并发需要时，在创建/调度
环境前冻结同一组增益，不同增益实验使用不同进程。新ABI尚不提供舵面遥测或完整状态
snapshot/clone。WP2A-8b已完成限定工况闭环验证；当前输出仍不能证明所有工况均无舵面饱和。

`config/aircraft/avanti2_att.yaml`保存本次编译实值：角度P均为6.5，
FW_RR=33.0766501559、FW_PR=73.5591994785；它们与上轮IA记录不同，未自动覆盖。
外环以GA结果为种子，已完成TECS兼容性适配和切圆制导修正。新动作接口及多实例调用见
[WP2A-8b报告](experiments/wp2a8b/README.md)。历史wp2a8入口显式使用
`LegacySurfaceFlightController`，新 `FlightController` 输出姿态指令。

## 当前进度（详见 docs/build_logs/phase2_controller_debug.md）

2026-11-02：用户手动测试后暂时接受当前Simulink集成姿态整定。MATLAB工具已迁至 `E:/matlab_Project/AVANTI_4/matlab`；本仓库保留统一调试记录和Python运行时。MATLAB参数尚不代表已同步至Python控制器。

2026-11-03：导入 `portable/avanti2_att` 的生成源码与C ABI，新增
`runtime/fixedwing_att`、Python封装、双对象构建与接口回归；WP2A-8b完成heading/loiter动作接口、
批量外环与独立handle并行推进，四项400秒飞行试验及4实例一致性验证。

```powershell
python experiments/wp2a8b/quality_trials.py --profile adapted --trial all --seconds 400
python experiments/wp2a8b/parallel_check.py
```

无风/3m/s风盘旋稳态径向MAE分别0.00668m/0.173m，8字横侧MAE 2.263m且完成6圈。
4实例串行与线程结果相同，小批量线程实测较慢，默认串行。完整指标、适用范围及图像见
[试验报告](experiments/wp2a8b/README.md)。

- [x] 按 phase2 §6.2 搭建项目目录架构
- [x] 从 phase1 复制 6DOF runtime（源码 + C ABI + `.so` + ctypes 封装 + 回归测试）
- [x] 根 CMakeLists + runtime/fixedwing/CMakeLists（Ubuntu 无 MATLAB 构建）
- [x] 舵面符号复测（delta_e 正→低头 / delta_a 正→左滚 / delta_r 正→右偏航）
- [x] WP2A-8 v2 控制器改造（PX4 架构）：串级姿态内环（角度 P + 角速率 FF/P/PI + 空速缩放 + 协调转弯前馈）、能量 TECS、L1+NPFG 风三角侧向制导、PX4 Circle（loiter）盘旋模式
- [x] 限定记忆 GA 整定：内环姿态跟踪（tuning_02）、外环盘旋闭环（tuning_03）
- [x] WP2A-8 控制品质试验（盘旋/8 字/跑马场轨迹 + 姿态/角速度/速度跟踪图像）
- [ ] WP2A-9 闭环 harness 正式验收、WP2A-10 3 条封闭航迹矩阵 + avanti_eso 张量收集
- [ ] Gate A 验收（docs §6.17）

## 时间契约（phase1 冻结）

- `dt_plant = 0.02 s`（fixed-step ODE4，50 Hz）；第 k 次 tick 只接受
  `t_k = epoch + k * 0.02`；
- 输入 7 通道（归一化舵面 [-1,1]、油门 [0,1]、flat-Earth 风，Ze 向下为正）；
- 输出 NEU（Up-positive）、Euler（deg）、BODY（SI）、alpha/beta（rad）。
