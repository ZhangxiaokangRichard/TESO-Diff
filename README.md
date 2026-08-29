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
├── CMakeLists.txt              根构建入口（当前仅 fixedwing runtime）
├── config/
│   ├── aircraft/avanti2.yaml   机型/plant 冻结参数
│   ├── controller/controller.yaml  TECS/L1/协调转弯/PID 参数
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
│   └── common/                 跨后端公共运行时（占位）
├── python/
│   ├── bindings/               phase1 python/avanti.py ctypes 封装
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

```bash
# 1. 构建 runtime（需要 cmake/g++，无需 MATLAB）
scripts/build_all.sh

# 2. 接口级冒烟（依赖审计 + 20s 回归 + 16 实例并行 + 生命周期）
scripts/run_phase2a_smoke.sh

# 3. 吞吐 benchmark
python3 scripts/benchmark_runtime.py --envs 1 8 32 --seconds 60
```

## 当前进度（phase2_sp1，详见 docs/build_logs/phase2_sp1.md）

- [x] 按 phase2 §6.2 搭建项目目录架构
- [x] 从 phase1 复制 6DOF runtime（源码 + C ABI + `.so` + ctypes 封装 + 回归测试）
- [x] 根 CMakeLists + runtime/fixedwing/CMakeLists（Ubuntu 无 MATLAB 构建）
- [x] controller/eso/simulation 包骨架与冻结接口（§6.12 数据结构 + PID）
- [x] config YAML（aircraft/controller/task/environment/dataset）
- [ ] WP2A-8 Python 张量控制器闭环（TECS/L1/协调转弯 + PID）
- [ ] WP2A-9 闭环 harness、WP2A-10 3 条封闭航迹闭环 + avanti_eso 张量收集
- [ ] Gate A 验收（docs §6.17）

## 时间契约（phase1 冻结）

- `dt_plant = 0.02 s`（fixed-step ODE4，50 Hz）；第 k 次 tick 只接受
  `t_k = epoch + k * 0.02`；
- 输入 7 通道（归一化舵面 [-1,1]、油门 [0,1]、flat-Earth 风，Ze 向下为正）；
- 输出 NEU（Up-positive）、Euler（deg）、BODY（SI）、alpha/beta（rad）。
