# AVANTI2 Embedded Coder C++ 动态库

本项目把 `AVANTI2_ertmodel0827.slx` 生成的固定步长 6DOF C++ 模型封装为稳定 C ABI。在 Windows 上产出 `avanti2.dll`，在 Ubuntu 上使用同一份 portable 源码产出 `libavanti2.so`。运行动态库不需要 MATLAB、Simulink、MAT 文件或 MathWorks 动态/静态链接库。

当前固定积分步长为 `0.02 s`，solver 为 fixed-step ODE4。动态库不接收 StopTime，也不自行睡眠或自由运行；外部调度器为每架飞机创建一个实例，并按全局时间逐 tick 调用。

## 1. 当前验证状态

验证日期：2026-08-27。

| 阶段 | 状态 | 结果 |
|---|---|---|
| A：Embedded Coder 生成 | 通过 | R2021a、`ert.tlc`、C++、C++ class、ODE4/0.02 s |
| B：稳定接口 | 通过 | 7 输入、23 个模型输出量、独立 handle、多实例冒烟测试 |
| C：Windows DLL | 通过 | CMake/MinGW 生成 64 位 `avanti2.dll`，C++/Python 可加载 |
| D：数值与并行 | 通过 | Simulink/DLL 20 s 逐样本回归、16 实例一致性、生命周期压力测试 |
| E：Ubuntu portable 预检 | 通过 | 源码 include graph 闭合，无 MATLAB 安装路径或 MATLAB 链接库 |
| E：Ubuntu/Isaac 实机 | 待目标机执行 | 当前 Windows 主机没有 Ubuntu 发行版/容器，也没有 Isaac Sim |

已测工具版本：

```text
MATLAB 9.10.0.1602886 (R2021a)
CMake 4.3.3
MinGW-w64 GCC 8.1.0 x86_64-posix-seh
Python 3.10.8
```

Windows DLL 的 SHA-256（每次重新生成代码后会改变）：

```text
B819A16A18E618A8558350B2E6FEAEDFE6E095BDB1793D5F8169A0C5735C4177
```

## 2. 架构与运行边界

```text
config/*.mat + AVANTI2_ertmodel0827.slx
                │  仅在 Windows/R2021a 生成期使用
                ▼
       Embedded Coder C++ class
 generated/codegen/..._ert_rtw
                │  package_avanti_portable.m
                ▼
 portable/avanti2/{include,src}       不含 MATLAB 安装路径
                │
                ├── interface/avanti_c_api.*  稳定 C ABI
                │
                └── 根 CMakeLists.txt
                         ├── Windows → avanti2.dll
                         └── Ubuntu  → libavanti2.so
                                      │
                         C++ / ctypes / Isaac extension
```

每个 `AvantiHandle` 内部拥有一个独立的：

```cpp
AVANTI2_ertmodel0827ModelClass
```

它包含本实例的连续状态、ODE4 solver 数据、block state、输入和输出。不同 handle 可以并行；同一 handle 不得被两个线程同时调用。对外只导出 C ABI，不导出生成 C++ class 或 STL 类型，因此 Windows/Linux 编译器 ABI 差异不会泄漏到调用方。

模型根层有两组显式采样边界：

- 七个 `CodegenInputHold_*` 在 major tick 采样输入，并在整个 ODE4 步长的 minor evaluations 内保持；
- 四个 `CodegenOutputHold_*` 保证 `step()` 返回的 ExtY 是 major tick 快照，不是 RK4 中间值。

这两组保持是 Simulink 与外部调度 DLL 逐样本一致的必要条件。不要删除或改采样时间。

## 3. 目录说明

```text
ert_model/
├─ AVANTI2_ertmodel0827.slx
├─ config/
│  ├─ AVANTI2_model0824.mat
│  └─ initState.mat
├─ matlab/                         R2021a 生成、打包和回归脚本
├─ generated/                      Embedded Coder 输出和 MATLAB 测试报告
├─ interface/avanti_c_api.{h,cpp}  手写稳定 C ABI 源文件
├─ portable/avanti2/
│  ├─ include/                     可移植生成头文件和 solver 头文件
│  └─ src/                         可移植生成 C++ 与 C ABI bridge
├─ python/avanti.py                ctypes 高层封装
├─ tests/                          C++/Python/Simulink 回归测试
├─ CMakeLists.txt                  Windows 和 Linux 共用
├─ interface.md                    生成类和 ABI 的详细实测记录
└─ README.md
```

`config/*.mat` 只在模型更新和代码生成时加载。它们的参数和初值被固化进生成代码；DLL/SO 在运行时不会读取 MAT 文件。修改模型、MAT 参数、初值、端口或步长后，必须从第 4 节重新生成并重新跑所有测试。

## 4. Windows：从 Simulink 手工生成 portable C++

### 4.1 前置条件

- Windows 10；
- MATLAB、Simulink、Simulink Coder、Embedded Coder R2021a；
- `config/AVANTI2_model0824.mat` 和 `config/initState.mat` 存在；
- 独立 64 位 MinGW `g++.exe` 可用。当前脚本会依次寻找 `%MW_MINGW64_LOC%\bin\g++.exe` 和 `C:\Program Files\mingw64\bin\g++.exe`；
- CMake 3.16 或更高版本。

MATLAB 只生成 C++ 源码，不负责最终 DLL 链接。独立 CMake 构建中的所有对象文件统一由同一套 MinGW 编译，避免混用 MATLAB toolchain 对象文件。

### 4.2 MATLAB 命令

打开 MATLAB R2021a，在 Command Window 执行：

```matlab
projectRoot = 'E:\matlab_Project\AVANTI_4\ert_model';
cd(projectRoot);
addpath(fullfile(projectRoot, 'matlab'));

% 载入两个 MAT 文件、保存代码生成配置、slbuild、A/B 检查和双实例冒烟测试
report = run_avanti_stage_ab(projectRoot, 0.02);

% 收集 Ubuntu/Windows 独立构建所需的全部源码和头文件
package_avanti_portable(projectRoot, report.GeneratedBuildDirectory);
```

`run_avanti_stage_ab` 内部依次完成：

1. `load_avanti_workspace` 把两个 MAT 文件中的变量加载到 base workspace；
2. `configure_avanti_codegen` 固化 ERT/C++ class/ODE4/0.02 s 配置；
3. `ensure_avanti_input_holds` 和 `ensure_avanti_output_holds` 幂等地维护根端口保持；
4. `slbuild('AVANTI2_ertmodel0827')` 生成 C++；
5. `test_avanti_stage_ab` 检查设置、端口、生成 class 和入口；
6. `test_avanti_cpp_interface` 编译并运行 2 实例、100 tick 冒烟测试。

关键生成产物：

```text
generated/codegen/AVANTI2_ertmodel0827_ert_rtw/
portable/avanti2/include/
portable/avanti2/src/
generated/stage_ab_test_report.mat
generated/stage_ab_cpp_test.mat
```

`portable/avanti2/include/rtw_continuous.h` 和 `rtw_solver.h` 是打包进项目的 continuous solver 头文件。Ubuntu 编译只从 portable 目录包含它们，不访问 `matlabroot`，也不链接 `libmw*`、`libmat`、`libmex` 或 Simulink 库。`tmwtypes.h` 只位于 `MATLAB_MEX_FILE` 条件分支；standalone CMake 构建不定义该宏，实际使用同目录的 `rtwtypes.h`。

## 5. Windows：CMake 构建 DLL

在普通 PowerShell 中执行：

```powershell
cd E:\matlab_Project\AVANTI_4\ert_model
$env:Path = 'C:\Program Files\mingw64\bin;' + $env:Path

cmake -S . -B build/win64 -G "MinGW Makefiles" `
  -DCMAKE_BUILD_TYPE=Release `
  -DAVANTI_BUILD_TESTS=ON

cmake --build build/win64 --parallel
ctest --test-dir build/win64 --output-on-failure
```

产物：

```text
build/win64/dist/avanti2.dll
build/win64/dist/libavanti2.dll.a
build/win64/dist/avanti_cpp_20s.exe
```

MinGW 链接使用 `-static -static-libgcc -static-libstdc++`。本次 `objdump -p` 实测 DLL 只依赖：

```text
KERNEL32.dll
msvcrt.dll
```

检查导出和依赖：

```powershell
objdump -p build/win64/dist/avanti2.dll | Select-String 'DLL Name'
nm -g --defined-only build/win64/dist/avanti2.dll | Select-String ' avanti_'
```

应看到第 8 节列出的 10 个 `avanti_*` 符号。

## 6. Ubuntu：无 MATLAB 构建 SO

Ubuntu 目标机只需要这些项目内容：

```text
CMakeLists.txt
portable/avanti2/
tests/
python/
```

不需要复制 `.slx`、`config/*.mat`、`generated/` 或 MATLAB 安装目录。推荐直接复制完整项目以保留文档，但 CMake 编译边界只有上述源码。

在与 Isaac Sim 相同的 Ubuntu 版本或构建容器内执行：

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake python3

cd /absolute/path/to/ert_model
python3 tests/test_portable_bundle.py

cmake -S . -B build/linux64 \
  -DCMAKE_BUILD_TYPE=Release \
  -DAVANTI_BUILD_TESTS=ON
cmake --build build/linux64 --parallel
ctest --test-dir build/linux64 --output-on-failure
```

产物应为：

```text
build/linux64/dist/libavanti2.so
build/linux64/dist/avanti_cpp_20s
```

检查 ELF、依赖和导出：

```bash
file build/linux64/dist/libavanti2.so
ldd build/linux64/dist/libavanti2.so
nm -D --defined-only build/linux64/dist/libavanti2.so | grep ' avanti_'
```

通过标准：

- `file` 显示 x86-64 ELF shared object；
- `ldd` 中只能出现 Ubuntu 系统 C/C++ runtime 等依赖，不能出现 MATLAB、Simulink、`libmw*`、`libmat` 或 `libmex`；
- `nm -D` 显示 10 个 C ABI 符号；
- CTest 的 C++ 20 秒测试通过。

然后重跑 Python：

```bash
python3 tests/test_ctypes_20s.py \
  --library build/linux64/dist/libavanti2.so \
  --seconds 20 --print-every 1

python3 tests/test_ctypes_parallel.py \
  --library build/linux64/dist/libavanti2.so \
  --instances 16 --seconds 20 --workers 16

python3 tests/test_ctypes_lifecycle.py \
  --library build/linux64/dist/libavanti2.so \
  --warmup 1000 --cycles 10000
```

为了避免 glibc 向后兼容问题，应在实际 Isaac Sim OS/容器中构建，或在不新于部署环境的 Ubuntu 上构建。不要在 Windows 交叉编译后把未运行的 ELF 当作阶段 E 已通过，也不要把 DLL 改名为 SO。

## 7. 输入/输出数据结构

公开头文件为 `portable/avanti2/include/avanti_c_api.h`。ABI 版本：

```c
#define AVANTI_ABI_VERSION 0x00010000u
```

在当前 x86-64 Windows ABI：

```text
sizeof(AvantiInput)  = 56 bytes
sizeof(AvantiOutput) = 192 bytes
```

调用方必须在加载后比较 `avanti_abi_version()`、`avanti_input_size()` 和 `avanti_output_size()`，不能假定结构体布局永远不变。

### 7.1 输入 `AvantiInput`

| 字段 | 类型 | 含义 | 单位/范围 |
|---|---|---|---|
| `delta_e_norm` | `double` | 升降舵归一化指令 | 建议 `[-1,1]`，映射约 `[-12,+12] deg` |
| `delta_a_norm` | `double` | 副翼归一化指令 | 建议 `[-1,1]`，映射约 `[-10,+10] deg` |
| `delta_r_norm` | `double` | 方向舵归一化指令 | 建议 `[-1,1]`，映射约 `[-22,+22] deg` |
| `throttle_01` | `double` | 油门 | 模型饱和到 `[0,1]` |
| `wind_flat_earth_mps[0]` | `double` | flat-Earth `X_e` 风速 | m/s |
| `wind_flat_earth_mps[1]` | `double` | flat-Earth `Y_e` 风速 | m/s |
| `wind_flat_earth_mps[2]` | `double` | flat-Earth `Z_e` 风速，向下为正 | m/s |

三个舵面输入不是弧度。超出 `[-1,1]` 时 lookup 会端点钳位；调用方仍应主动限幅并记录异常命令。

### 7.2 输出 `AvantiOutput`

| 字段 | 数量 | 含义 | 单位 |
|---|---:|---|---|
| `sample_time_s` | 1 | 本输出对应的外部全局时间 | s |
| `alpha_rad` | 1 | 迎角 | rad |
| `beta_rad` | 1 | 侧滑角 | rad |
| `position_ne_alt_m` | 3 | North/Xe、East/Ye、Altitude=-Ze | m |
| `velocity_ne_up_mps` | 3 | `Ve_x`、`Ve_y`、Up=`-Ve_z` | m/s |
| `euler_rpy_deg` | 3 | roll、pitch、yaw | deg |
| `velocity_body_mps` | 3 | `u,v,w` | m/s |
| `rates_body_radps` | 3 | `p,q,r` | rad/s |
| `rates_body_derivative_radps2` | 3 | `p_dot,q_dot,r_dot` | rad/s² |
| `acceleration_body_mps2` | 3 | `u_dot,v_dot,w_dot` | m/s² |

注意根端口名虽然是 `NED`，但高度和竖直速度已经在模型内取反为 Up-positive；不能把数组第 3、6 个值当作 Down-positive。Euler 输出是度，不是弧度。Isaac 接入必须显式做轴向、手性和角度单位转换。

固化初值：

```text
Xe/Ye/Ze       = [0, 0, -1000] m
u/v/w          = [14.1, 0, 0.0251] m/s
phi/theta/psi  = [0, 0.00178, 0] rad
p/q/r          = [0, 0, 0] rad/s
```

`avanti_reset()` 回到这组编译期初值。当前 ABI 尚未开放任意初始状态或在线气动参数设置；若 Isaac 环境需要 episode 随机化，应在模型和 ABI 中单独设计，不要直接改生成 class 内存。

## 8. C ABI 和时间契约

导出函数：

```c
uint32_t avanti_abi_version(void);
uint32_t avanti_input_size(void);
uint32_t avanti_output_size(void);

int32_t avanti_create(double epoch_s, AvantiHandle* out_handle);
int32_t avanti_destroy(AvantiHandle handle);
int32_t avanti_reset(AvantiHandle handle, double epoch_s);
int32_t avanti_get_fixed_step(AvantiHandle handle, double* out_h_s);
int32_t avanti_get_next_tick_time(AvantiHandle handle, double* out_time_s);
int32_t avanti_tick(AvantiHandle handle, double global_time_s,
                    const AvantiInput* input, AvantiOutput* output);
int32_t avanti_advance_to(AvantiHandle handle, double target_time_s,
                          const AvantiInput* held_input,
                          uint64_t max_substeps, AvantiOutput* output);
```

状态码：

| 值 | 名称 | 含义 |
|---:|---|---|
| 0 | `AVANTI_OK` | 成功 |
| 1 | `AVANTI_ERR_NULL` | 空指针/空 handle |
| 2 | `AVANTI_ERR_ALLOCATION` | 内存分配失败 |
| 3 | `AVANTI_ERR_NONFINITE` | 时间或输入含 NaN/Inf |
| 4 | `AVANTI_ERR_TIME_NOT_ON_GRID` | 时间不在 0.02 s 网格 |
| 5 | `AVANTI_ERR_TIME_BACKWARDS` | 时间倒退 |
| 6 | `AVANTI_ERR_TOO_MANY_SUBSTEPS` | `advance_to` 超过保护上限 |
| 7 | `AVANTI_ERR_MODEL` | 生成模型报告错误 |
| 8 | `AVANTI_ERR_EXCEPTION` | C++ 异常被 ABI 边界捕获 |

令 `h=0.02 s`，创建实例时指定 Isaac epoch：

```text
avanti_create(epoch_s, &handle)
```

第 `k` 次 tick 只接受：

```text
t_k = epoch_s + k*h
```

一次 `avanti_tick(handle, t_k, u_k, &y_k)`：

1. 检查时间网格和输入有限性；
2. 在 major tick 采样 `u_k`，整个 ODE4 步内保持；
3. 调用一次生成类 `step()`；
4. 返回标记为 `t_k` 的 major-step 输出 `y(t_k)`；
5. 内部连续状态已经积分到 `t_k+h`；
6. 下一次只接受 `t_k+h`。

应使用整数 tick 计算时间：`epoch + tick_index*h`，避免通过反复执行 `time += h` 累积浮点误差。接口有小容差，但它不是任意时间插值器。

`avanti_advance_to` 会用同一个 held input 连续调用多个网格 tick，并包含 `target_time_s` 对应的那次调用。只有在所有中间步控制量确实保持不变时才能使用；强化学习策略每步更新控制时应逐次调用 `avanti_tick`。

## 9. C++ 20 秒测试

CTest 已运行 `tests/cpp_20s_test.cpp`：

```powershell
build\win64\dist\avanti_cpp_20s.exe
```

或在 Ubuntu：

```bash
./build/linux64/dist/avanti_cpp_20s
```

用例共 `1000` tick：

| 时间 | 输入 |
|---|---|
| 0–5 s | throttle=0.5，其余为 0 |
| 5–8 s | `delta_e_norm=-0.01` |
| 8–11 s | `delta_a_norm=0.01` |
| 11–14 s | `delta_r_norm=0.01` |
| 14–20 s | `wind_x=1 m/s` |

程序每秒打印一次 alpha/beta、位置、机体系速度和角速度，并验证全部输出有限、最终 next tick 为 `20.0 s`。本次末样本为 `t=19.98 s`，随后内部 next tick 为 `20.0 s`。

这个用例用于接口、调度和数值回归，不代表配平稳定性或飞行品质认证。

## 10. Python 使用和测试

最小使用示例：

```python
from pathlib import Path
from python.avanti import AvantiInput, AvantiLibrary

library = AvantiLibrary(Path("build/win64/dist/avanti2.dll"))
inputs = AvantiInput(throttle_01=0.5)

with library.model(epoch_s=100.0) as aircraft:
    for tick in range(1000):
        global_time_s = 100.0 + tick * aircraft.fixed_step_s
        output = aircraft.tick(inputs, global_time_s)
        print(global_time_s, output.alpha_rad, output.position_ne_alt_m[2])
```

Ubuntu 只需把路径换成 `build/linux64/dist/libavanti2.so`。

### 10.1 20 秒并打印/保存数据

```powershell
python tests/test_ctypes_20s.py `
  --library build/win64/dist/avanti2.dll `
  --seconds 20 --print-every 1
```

输出 CSV：

```text
build/test_output/python_20s.csv
```

实测通过 `1000` tick。Simulink 基准位于 `tests/reference/simulink_20s.csv`；两者 15 列、1000 行逐样本比较结果：

```text
最大时间误差             3.552714e-15 s
最大值绝对误差           4.092726e-12（altitude 列，m）
比较阈值                 abs <= 1e-9 或 rel <= 1e-10
```

重新生成基准和比较：

```matlab
projectRoot = 'E:\matlab_Project\AVANTI_4\ert_model';
addpath(fullfile(projectRoot, 'matlab'));
generate_simulink_reference_20s(projectRoot, 20);
compare_simulink_dll_20s(projectRoot, ...
    fullfile(projectRoot, 'build', 'test_output', 'python_20s.csv'));
```

### 10.2 Python 多实例简易并行

```powershell
python tests/test_ctypes_parallel.py `
  --library build/win64/dist/avanti2.dll `
  --instances 16 --seconds 20 --workers 16
```

实测：16 个实例串行和 ThreadPoolExecutor 并行各执行 `16000` 次调用，最终结果最大绝对差为 `0`。当前小模型单次调用很短，Python 线程版本实测比串行慢（本机 speedup 约 `0.66`），这是 Python 调度/逐调用开销，不是实例互相干扰。该测试证明并行正确性，不证明吞吐加速。

生产环境若有数百/数千实例，应采用 C++ 线程池、原生 batch API 或在一个 native 调用内推进多个 handle，避免 Python 每个实例每 tick 一次跨 ABI 调用。

### 10.3 生命周期压力测试

```powershell
python tests/test_ctypes_lifecycle.py `
  --library build/win64/dist/avanti2.dll `
  --warmup 1000 --cycles 10000
```

每个周期执行 create → tick → reset → tick → destroy。本机测量区间内 RSS 增长 `24576 bytes`，进程 handle 增长 `0`，通过 16 MiB 上限。Ubuntu 脚本会读取 `/proc/self/statm` 和 `/proc/self/fd`，无需 `psutil`。

## 11. Isaac Sim / Isaac Lab 并行接入设计

当前环境没有 Isaac Sim，因此本节是目标实现和验收设计，不是实机通过声明。

### 11.1 实例所有权

- 每个 Isaac environment/aircraft 对应一个 `AvantiHandle`；
- episode reset 调用 `avanti_reset(handle, episode_epoch)`；
- extension/shutdown 时调用 `avanti_destroy`；
- 一个 handle 始终由一个 worker 拥有，同一 tick 内禁止并发访问同一 handle；
- 不同 handle 可按固定分片分配给线程池，保持实例间并行、实例内串行。

推荐所有权关系：

```text
Isaac global clock
      │
      ├─ worker 0: handle[0], handle[4], handle[8], ...
      ├─ worker 1: handle[1], handle[5], handle[9], ...
      ├─ worker 2: handle[2], handle[6], handle[10], ...
      └─ worker 3: handle[3], handle[7], handle[11], ...
```

### 11.2 时钟同步

最简单且最确定的配置是令 Isaac physics step 为 `0.02 s`，每个 physics tick 对每个 active handle 调用一次 `avanti_tick`。

若 Isaac physics dt 不是 `0.02 s`：

1. 为每个环境保存整数 `model_tick_index` 和 accumulator；
2. 当 Isaac 全局时间跨过下一个 AVANTI 网格点时推进一个或多个 0.02 s 子步；
3. 每个子步使用该时刻对应的控制量；若只能保证 held input，才使用 `avanti_advance_to`；
4. 输出应用到 Isaac 前明确选择 hold 或插值策略；
5. 不要把任意 Isaac 时间直接传给 `avanti_tick`，否则会返回 `AVANTI_ERR_TIME_NOT_ON_GRID`。

为可复现性，网格时间由 `epoch + integer_tick*0.02` 计算，并记录 episode epoch、控制输入和 tick index。

### 11.3 坐标与数据流

建议在 adapter 层集中完成：

```text
Isaac world wind/commands
        │  axis + handedness + deg/rad conversion
        ▼
AvantiInput (flat-Earth/body conventions)
        │
        ▼
avanti_tick
        │
        ▼
AvantiOutput
        │  position/orientation conversion
        ▼
Isaac articulation / sensor / observation buffers
```

必须用一个已知姿态和已知风向的单实例测试确认：North/East/Altitude、flat-Earth Z-down、body `u/v/w`、roll/pitch/yaw 顺序、角度单位以及 Isaac stage units。不要只凭端口名假设坐标一致。

### 11.4 大规模吞吐建议

- 预先创建 handle 数组；episode 内只 reset，不频繁 dlopen/create/destroy；
- 输入和输出在 adapter 层使用连续数组，按 worker 分片，减少 Python 对象分配；
- 对每个 worker 批量推进一组 handle，再一次性写回 observation buffer；
- CPU 模型不会自动在 GPU 上执行，注意 GPU/CPU 同步点；
- 如果 Python 成为瓶颈，增加 `avanti_tick_batch(handles, inputs, outputs, count)` 原生 API，或用 pybind11/C++ extension 在一次调用内释放 GIL 并完成整个 batch；
- 锁只保护调度队列，不要在每个 handle 的数值路径上使用全局锁；
- 性能测试至少记录 environments、ticks/s、wall time、CPU utilization、p50/p99 tick latency 和 deterministic replay 差异。

### 11.5 Isaac 目标验收

1. Isaac Python 进程能 `ctypes.CDLL`/原生 extension 加载 `libavanti2.so`；
2. 1 个实例与本项目 20 秒 CSV 在容差内一致；
3. N 个实例串行/并行结果一致；
4. episode reset 后首样本与新建实例一致；
5. 长时间运行无 NaN/Inf、RSS/FD 持续增长或 handle 串扰；
6. physics dt 与 0.02 s 网格的策略已固定并加入测试；
7. 坐标变换用已知姿态/风向案例验证；
8. 在实际部署 Ubuntu/Isaac 容器内保存 `file`、`ldd`、`nm -D`、GCC、CMake 和 ABI 版本记录。

## 12. 何时必须重新生成

以下任一变化都要在 R2021a 机器重新执行第 4 节，然后在 Windows 和 Ubuntu 重跑 C/D/E：

- `.slx` 模型、端口、bus 或 solver 改动；
- `config/*.mat` 参数或初值改动；
- `fixedStep` 改动；
- C ABI 字段、含义或单位改动；
- 生成代码版本或 Embedded Coder release 改动。

只改 Python 测试或 Isaac adapter 不需要重新生成模型，但仍应跑 ABI size/version 和相关回归测试。

## 13. 已知限制

- 固定步长只有 `0.02 s`，未完成 `h/h2/h4` 收敛性论证；
- 当前 20 秒命令序列是数值回归用例，不是配平或飞行品质验证；
- 初始状态和模型参数是编译期常量，尚无 per-instance 参数/任意初值 API；
- `avanti_advance_to` 只支持 held input；
- Python ThreadPoolExecutor 的正确性已验证，但当前粒度没有加速收益；
- Ubuntu `.so` 和 Isaac Sim 实际加载必须在目标环境按第 6、11 节完成，Windows 预检不能替代实机结果。

更底层的生成类名、ExtU/ExtY 数组映射和 A/B 测试证据见 `interface.md`。
