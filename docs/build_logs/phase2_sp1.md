# Phase 2 SP1 — 项目架构搭建与 phase1 运行时移植

**日期：** 2026-08-30
**任务：** 按 `docs/TESO-Diff_phase2.md` 搭建 TESO-Diff 项目架构，从 phase1 项目
`~/AVANTI_6DOFmodel` 复制后续必需的 `.so` 与接口文件（跳过 Simulink 代码生成与编译部分）。

---

## 1. 完成内容

### 1.1 目录架构（phase2 §6.2）

```text
TESO-Diff/
├── CMakeLists.txt                  根构建入口（当前仅 fixedwing runtime）
├── config/
│   ├── aircraft/avanti2.yaml       机型/plant 冻结参数（ABI 0x00010000、dt=0.02s）
│   ├── controller/controller.yaml  TECS/L1/协调转弯/PID 参数（占位初值，WP2A-8 整定）
│   ├── task/                       S1~S4：circle / figure8 / racetrack / gps_triangle
│   ├── environment/                W0 no_wind / W1 thermal_only / W2 shear_only /
│   │                               W3 composite / wind_smoke
│   └── dataset/smoke.yaml          Phase 2b-S0 smoke 配置
├── docs/
│   ├── TESO-Diff_phase2.md（设计总则）、TESO-Diff_phase1.md、TESO-Diff.pdf
│   ├── fixedwing_interface.md      ← phase1 interface.md 副本
│   └── build_logs/phase2_sp1.md    本文档
├── runtime/
│   ├── fixedwing/
│   │   ├── include/                ← portable/avanti2/include（公共头 + avanti_c_api.h）
│   │   ├── generated/              ← portable/avanti2/src（Embedded Coder 生成源码，
│   │   │                           不含 C ABI bridge）
│   │   ├── wrapper/                ← interface/avanti_c_api.{h,cpp}（手写 C ABI bridge）
│   │   ├── lib/libavanti2.so       ← phase1 Ubuntu 构建产物（预构建，无 MATLAB 依赖）
│   │   └── CMakeLists.txt          适配新目录的构建脚本
│   └── common/                     跨后端公共运行时（占位）
├── python/
│   ├── bindings/                   ← phase1 python/avanti.py ctypes 封装（包化 + 路径适配）
│   ├── controller/                 张量并行控制器包：
│   │   ├── types.py                ControlReference / ControlObservation /
│   │   │                           ActuatorCommand（§6.12 冻结数据结构）
│   │   ├── pid.py                  张量化 PID（已实现，含 snapshot/restore）
│   │   ├── tecs.py / l1.py / coordinated_turn.py  外环接口（WP2A-8 实现）
│   │   ├── outer.py / inner.py     外/内环整合接口（WP2A-8 实现）
│   │   └── avanti_controller.py    AvantiController 门面（reset/step/intermediates/snapshot）
│   ├── eso/
│   │   ├── history.py              滚动 history buffer（双时间通道约定，§27）
│   │   └── tensor_recorder.py      avanti_eso 张量记录器（逐 tick 真值流）
│   ├── simulation/
│   │   ├── wind_stub.py            风场查询接口 wind.query(pos, t)（zero/uniform 桩）
│   │   └── harness.py              闭环 harness 接口（§6.13，WP2A-9 实现）
│   └── tests/test_controller_interfaces.py  包级单测
├── tests/
│   ├── cpp/cpp_20s_test.cpp        ← phase1 C++ 20s 回归（CTest）
│   ├── reference/                  ← simulink_20s.csv + input_20s.csv 基准
│   └── regression/                 ← test_ctypes_20s / parallel / lifecycle /
│                                   test_portable_bundle（已适配新路径）
├── isaac/                          Phase 2b 规划（README，§7.3 结构约定）
├── dataset/{schema,smoke,formal}/  Phase 2b 数据集输出（占位）
└── scripts/
    ├── build_all.sh                构建 + CTest + file/ldd/nm 检查
    ├── run_phase2a_smoke.sh        4 项接口级冒烟一键入口
    ├── run_phase2a_tasks.py        配置校验 + trim 冒烟（闭环任务 WP2A-8~10 占位）
    └── benchmark_runtime.py        1/8/32 实例吞吐 benchmark（§6.16）
```

### 1.2 从 phase1 复制的文件

| 来源（~/AVANTI_6DOFmodel） | 目标 | 说明 |
|---|---|---|
| `portable/avanti2/include/*`（11 个文件） | `runtime/fixedwing/include/` | 生成模型头 + solver 头 + `avanti_c_api.h` |
| `portable/avanti2/src/`（5 个生成源码） | `runtime/fixedwing/generated/` | 模型 + rt 运行时（不含 bridge） |
| `interface/avanti_c_api.{h,cpp}` | `runtime/fixedwing/wrapper/` | 手写 C ABI bridge |
| `build/linux64/dist/libavanti2.so` | `runtime/fixedwing/lib/` | phase1 Ubuntu 构建产物 |
| `python/avanti.py` | `python/bindings/` | ctypes 封装（仅改默认库搜索路径） |
| `tests/cpp_20s_test.cpp` | `tests/cpp/` | C++ 回归 |
| `tests/reference/*.csv` | `tests/reference/` | Simulink 20s 基准 |
| `tests/test_ctypes_*.py`、`test_portable_bundle.py` | `tests/regression/` | Python 回归（路径已适配） |
| `interface.md` | `docs/fixedwing_interface.md` | C ABI 实测记录 |

**未复制**（phase2 §6.2 明确不需要）：`.slx`/`.slxc`、`config/*.mat`、
`generated/`、`matlab/`、`slprj/`、`build/`（除 dist 产物）、
`code_build_for_AVANTI.md` 等 Simulink 代码生成与编译相关内容。
模型再生成仍在 phase1 项目（R2021a 机器）完成，产物同步进 `runtime/fixedwing`。

### 1.3 关键适配点

1. **CMake 重组**：根 `CMakeLists.txt`（project + tests）+
   `runtime/fixedwing/CMakeLists.txt`（库目标），输出仍为
   `build/linux64/dist/`，与 phase1 路径风格一致；
2. **`avanti.py` 默认库搜索**：`runtime/fixedwing/lib/` → `build/*/dist/`；
3. **回归测试路径**：`tests/regression/` 下项目根由 `parents[1]` 改为 `parents[2]`；
4. **依赖审计**：`test_portable_bundle.py` 改为审计 `runtime/fixedwing`
   （include + generated + wrapper）与两个 CMakeLists；
5. **控制器接口冻结**：`types.py` 实现 §6.12 的数据结构
   （ControlReference / ControlObservation / ActuatorCommand，batch 化 numpy 张量），
   PID 已实现（含积分限幅与 snapshot），TECS/L1/协调转弯/内外环为接口桩
   （WP2A-8 实现）。

## 2. 验证结果（本机 Ubuntu 22.04）

| 项目 | 结果 |
|---|---|
| CMake 配置 + 构建 `libavanti2.so` | 通过 |
| CTest `avanti_cpp_20s`（1000 tick） | 通过 |
| 依赖审计（无 MATLAB 路径/链接库） | 18 文件，quoted include 0 未解析，通过 |
| Python 20s 逐样本回归（1000 tick） | 通过（输出有限、时间契约正确） |
| 16 实例串行/并行一致性 | max_abs_difference = 0 |
| 生命周期压力（预热 1000 + 5000 周期） | RSS 增长 0 B，fd 增长 0 |
| python 包级单测（PID/数据结构/wind 符号） | 通过 |
| run_phase2a_tasks（12 配置 YAML 校验 + trim 冒烟） | 通过 |
| benchmark（`.so` + Python 封装，20 s） | 1 env ≈ 269k steps/s；8 env ≈ 77k；32 env ≈ 78k（≈1500× realtime） |

环境：Python 3.11.13、cmake 3.22.1、g++ 11.4.0、PyYAML 6.0.2、numpy 1.26.0。

## 3. 冻结的接口约定（本阶段重申）

- `dt_plant = 0.02 s`（fixed-step ODE4）；第 k 次 tick 只接受 `t = epoch + k*0.02`；
- 7 输入：delta_e/a/r 归一化 `[-1,1]`、throttle `[0,1]`、flat-Earth 风（Z 向下为正）；
- 4 组输出：alpha/beta（rad）、NEU（Up-positive）、Euler（deg）、BODY（SI）；
- 控制器张量数据结构（§6.12）已冻结，snapshot 覆盖积分器/滤波器状态；
- 风场查询统一接口 `wind.query(position, time, env_parameters)`（§13）。

## 4. 下一步（WP2A-0 起）

1. WP2A-8：实现 TECS / L1 / 协调转弯 + PID 内环闭环（`python/controller` 桩落位）；
2. WP2A-9：闭环 harness 实现（`python/simulation/harness.py`）；
3. WP2A-10：3 条封闭航迹（盘旋 / 8 字 / 跑马场）闭环 + avanti_eso 张量收集；
4. WP2A-3/6/7：输入通道逐项测试、reset 确定性、snapshot/clone 方案；
5. 长时序（30 min+）与 Gate A 验收（docs §6.17）。
