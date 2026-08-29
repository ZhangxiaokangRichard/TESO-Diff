# AVANTI2 生成代码与动态库接口记录

记录时间：2026-08-27  
模型：`AVANTI2_ertmodel0827.slx`（生成后模型版本 1.134）  
环境：MATLAB / Simulink / Embedded Coder R2021a  
阶段 A：通过  
阶段 B：通过  
阶段 C（Windows DLL）：通过  
阶段 D（20 s 数值与多实例）：通过  
阶段 E：portable 无 MATLAB 依赖预检通过；目标 Ubuntu / Isaac Sim 实机验证待执行

## 1. 结论

已用 `ert.tlc + C++ + C++ class` 成功生成代码，通过独立 C ABI bridge 构建 Windows DLL，并完成 Simulink、C++、Python 的 20 秒逐样本数值回归和多实例测试。`portable/avanti2` 的完整 include graph 已闭合，CMake 不使用 MATLAB 安装路径或 MATLAB/Simulink 链接库。

生成目录：

```text
generated/codegen/AVANTI2_ertmodel0827_ert_rtw/
```

生成类：

```cpp
AVANTI2_ertmodel0827ModelClass
```

主要入口：

```cpp
void initialize();
void step();
void terminate();
void setExternalInputs(const ExtU_AVANTI2_ertmodel0827_T*);
const ExtY_AVANTI2_ertmodel0827_T& getExternalOutputs() const;
RT_MODEL_AVANTI2_ertmodel0827_T* getRTM();
```

固定步长为 `0.02 s`，沿用了模型原有的 `ode4 / 0.02 s` 设置。本次验证证明该步长可以生成和运行，不等同于已经完成 `h / h/2 / h/4` 数值收敛性论证。

## 2. 阶段 A 实际配置

| 参数 | 已保存值 |
|---|---|
| System target file | `ert.tlc` |
| Target language | `C++` |
| Code interface packaging | `C++ class` |
| Solver type | `Fixed-step` |
| Solver | `ode4` |
| Fixed step | `0.02` s |
| Solver mode | `SingleTasking` |
| Support continuous time | `on` |
| Suppress error status | `off` |
| Multi-instance diagnostic | `Error` |
| Generate example main | `off` |
| Generate code only | `on` |
| Generate makefile | `off` |

`GenerateMakefile=off` 是有意设置：阶段 A/B 只由 MATLAB 生成源代码，再由项目自己的编译流程处理，避免依赖 MATLAB 当前未登记的 MinGW toolchain。

## 3. 为连续求解器增加的主步输入/输出处理

模型根层的七个输入均增加采样时间为 `0.02 s`、Tag 为 `AVANTI_CODEGEN_INPUT_HOLD` 的 Zero-Order Hold：

```text
CodegenInputHold_delta_e / delta_a / delta_r / delta_t
CodegenInputHold_w_x / w_y / w_z
```

这是为了让桌面 Simulink 与 DLL 具有同一个外部调度契约：在 major tick 采样一次 `u[k]`，随后在整个 ODE4 步长及全部 minor solver evaluations 内保持不变。否则桌面仿真的 RK4 末端可能提前看到下一个时间点的新输入。

模型根层新增了四个采样时间为 `0.02 s` 的 Zero-Order Hold：

```text
CodegenOutputHold_alpha
CodegenOutputHold_beta
CodegenOutputHold_NED
CodegenOutputHold_BODY
```

原因是 C++ class 使用合并的 `step()`。固定步长 ODE4 在一次调用内执行多个 minor step；若直接在 `step()` 返回后读取连续根输出，可能读到 minor-step 残留值。

最终生成源码已确认：四个输出赋值位于 `rtmIsMajorTimeStep(...)` 保护内。因此一次 `step()` 返回的 ExtY 是当前 major tick 的快照。

## 4. 生成类的真实根输入

生成类型：

```cpp
struct ExtU_AVANTI2_ertmodel0827_T {
    real_T delta_e;
    real_T delta_a;
    real_T delta_r;
    real_T delta_t;
    real_T w_x;
    real_T w_y;
    real_T w_z;
};
```

所有成员都是 64 位 `double` 标量，顺序如下。

| 序号 | 生成字段 | C ABI 字段 | 实际含义/单位 |
|---:|---|---|---|
| 1 | `delta_e` | `delta_e_norm` | 升降舵归一化指令；建议范围 `[-1,1]`，对应约 `[-12,+12] deg` |
| 2 | `delta_a` | `delta_a_norm` | 副翼归一化指令；建议范围 `[-1,1]`，对应约 `[-10,+10] deg` |
| 3 | `delta_r` | `delta_r_norm` | 方向舵归一化指令；建议范围 `[-1,1]`，对应约 `[-22,+22] deg` |
| 4 | `delta_t` | `throttle_01` | 油门，模型根层饱和到 `[0,1]` |
| 5 | `w_x` | `wind_flat_earth_mps[0]` | flat-Earth `X_e` 轴风速，m/s |
| 6 | `w_y` | `wind_flat_earth_mps[1]` | flat-Earth `Y_e` 轴风速，m/s |
| 7 | `w_z` | `wind_flat_earth_mps[2]` | flat-Earth `Z_e` 轴风速，m/s；Aerospace Blockset 约定 `Z_e` 向下为正 |

重要：计划文档中的 `delta_*_rad` 只是占位示例，与当前模型不符。当前三个舵面根输入是归一化指令，不是弧度。

风速先通过模型的 `DCM_be` 从 flat-Earth 轴转换到机体系，再从机体系速度中扣除；因此 Isaac 世界坐标接入前必须完成它的世界轴到本模型 flat-Earth 轴的明确映射。

## 5. 生成类的真实根输出

生成类型：

```cpp
struct ExtY_AVANTI2_ertmodel0827_T {
    real_T alpha;
    real_T beta;
    real_T NED[9];
    real_T BODY[12];
};
```

模型内部使用 virtual bus，所以生成接口不是嵌套 C struct，而是两个扁平数组。

### 5.1 标量

| 生成字段 | C ABI 字段 | 单位 |
|---|---|---|
| `alpha` | `alpha_rad` | rad |
| `beta` | `beta_rad` | rad |

### 5.2 `NED[9]`

| 索引 | C ABI 映射 | 实际生成表达式 | 单位 |
|---:|---|---|---|
| 0 | `position_ne_alt_m[0]` | `X_e` | m |
| 1 | `position_ne_alt_m[1]` | `Y_e` | m |
| 2 | `position_ne_alt_m[2]` | `-Z_e`，即高度/向上为正 | m |
| 3 | `velocity_ne_up_mps[0]` | `V_e_x` | m/s |
| 4 | `velocity_ne_up_mps[1]` | `V_e_y` | m/s |
| 5 | `velocity_ne_up_mps[2]` | `-V_e_z`，向上为正 | m/s |
| 6 | `euler_rpy_deg[0]` | roll / `phi` | deg |
| 7 | `euler_rpy_deg[1]` | pitch / `theta` | deg |
| 8 | `euler_rpy_deg[2]` | yaw / `psi` | deg |

根端口虽然名为 `NED`，但索引 2 和 5 已被模型取反，实际提供的是 altitude/up-positive 表达；不能把它们当作标准 NED 的 Down-positive 分量。姿态角也已经由 rad 转换成 deg。

### 5.3 `BODY[12]`

| 索引 | C ABI 映射 | 含义 | 单位 |
|---:|---|---|---|
| 0..2 | `velocity_body_mps[0..2]` | `u, v, w` | m/s |
| 3..5 | `rates_body_radps[0..2]` | `p, q, r` | rad/s |
| 6..8 | `rates_body_derivative_radps2[0..2]` | `p_dot, q_dot, r_dot` | rad/s² |
| 9..11 | `acceleration_body_mps2[0..2]` | `u_dot, v_dot, w_dot` | m/s² |

## 6. 稳定 C ABI

阶段 B 已建立：

```text
interface/avanti_c_api.h
interface/avanti_c_api.cpp
```

ABI 版本：`0x00010000`。

在当前 Windows x86-64 测试环境：

```text
sizeof(AvantiInput)  = 56 bytes
sizeof(AvantiOutput) = 192 bytes
```

入口：

```c
avanti_create
avanti_destroy
avanti_reset
avanti_get_fixed_step
avanti_get_next_tick_time
avanti_tick
avanti_advance_to
```

bridge 对所有 ExtU/ExtY 字段逐项赋值，没有使用 `memcpy` 绑定生成布局。若模型端口变化，编译会在这一个 bridge 处暴露不一致。

每个 `AvantiHandle` 拥有一个独立的 `AVANTI2_ertmodel0827ModelClass`。不同 handle 可由外层进行实例间并行；同一个 handle 不能同时被多个线程调用。

## 7. 时间契约

令 `h = 0.02 s`，创建实例时传入 Isaac 全局 epoch：

```text
avanti_create(epoch_s, &handle)
```

第 `k` 次调用必须满足：

```text
global_time_s = epoch_s + k*h
```

一次 `avanti_tick(handle, t_k, input, output)` 的语义：

1. 检查 `t_k` 是否为该实例的下一个时间网格点；
2. 将输入写入生成对象；
3. 调用一次 `model.step()`；
4. 返回标记为 `t_k` 的 major-step 输出 `y(t_k)`；
5. 模型内部连续状态已经由 ODE4 积分到 `t_k+h`；
6. 下一次只接受 `t_k+h`。

接口不会把 Isaac 时间直接写入模型内部时钟，也不会按 `StopTime` 自由运行。仿真长度完全由外部 tick 次数决定。

`avanti_advance_to()` 只适用于中间子步输入保持不变的追赶；有逐子步控制变化时必须逐次调用 `avanti_tick()`。

## 8. 固化的初始化状态

生成时 `config/initState.mat` 的值已进入 `initialize()`：

```text
X_e/Y_e/Z_e       = [0, 0, -1000] m
u/v/w             = [14.1, 0, 0.0251] m/s
phi/theta/psi     = [0, 0.00178, 0] rad
p/q/r             = [0, 0, 0] rad/s
```

当前 `avanti_reset()` 会回到这组统一初值。MAT 文件不是 C++ 运行时依赖；修改 MAT 文件后必须重新生成代码。

## 9. 已执行测试

一键入口：

```matlab
projectRoot = 'E:\matlab_Project\AVANTI_4\ert_model';
addpath(fullfile(projectRoot, 'matlab'));
report = run_avanti_stage_ab(projectRoot, 0.02);
```

2026-08-27 最终回归结果：

- MATLAB `9.10.0.1602886 (R2021a)`；Embedded Coder 许可证可用；
- `slbuild` 成功；七个输入保持与四个输出保持均通过模型检查；
- 生成类、7 输入、4 根输出、`NED[9]`、`BODY[12]` 和入口函数检查通过；
- 未生成 `ert_main.cpp`；
- 四个根输出均在生成代码的 major-step 分支内；
- CMake 4.3.3 + MinGW-w64 GCC 8.1.0 生成 64 位 `avanti2.dll`；
- DLL 仅依赖 `KERNEL32.dll` 和 `msvcrt.dll`，10 个 `avanti_*` C 符号全部导出；
- C++ 冒烟测试创建两个独立实例并运行 100 tick；
- 相同输入阶段两个实例一致；向第二实例施加副翼指令后状态分离，第一实例不受影响；
- C++ 与 Python 分别完成 20 秒、1000 tick 测试，全部输出有限；
- Simulink 与 DLL 的 1000 个时间点逐列对齐，最大时间误差 `3.553e-15 s`，全部值通过 `1e-9` 绝对或 `1e-10` 相对容差；
- 数值比较中最大的绝对值差为高度列 `4.093e-12 m`；
- Python 16 实例串行/线程执行各 16000 次调用，最终结果最大绝对差为 `0`；
- create/reset/destroy 在 1000 次预热后再执行 10000 个周期，RSS 增长 `24576 bytes`，进程 handle 增长 `0`；
- portable bundle 共 17 个文件，quoted include 缺失数为 `0`，没有 MATLAB 绝对路径或 MATLAB 链接库；
- `fixed_step=0.02 s`、时间倒退错误、next-tick 和 reset 契约检查通过；
- 原生测试输出：`input_size=56 output_size=192 fixed_step_s=0.02`。

测试程序与产物：

```text
tests/stage_ab_cpp_smoke.cpp
tests/cpp_20s_test.cpp
tests/test_ctypes_20s.py
tests/test_ctypes_parallel.py
tests/test_ctypes_lifecycle.py
tests/test_portable_bundle.py
build/stage_ab/avanti_stage_ab_smoke.exe
build/win64/dist/avanti2.dll
build/win64/dist/avanti_cpp_20s.exe
build/test_output/python_20s.csv
tests/reference/simulink_20s.csv
generated/stage_ab_test_report.mat
generated/stage_ab_cpp_test.mat
generated/stage_d_numeric_report.mat
generated/stage_d_numeric_errors.csv
```

说明：GCC 8.1.0 是本次项目自主管理的 DLL 工具链，不参与 MATLAB 内部构建。它不是 R2021a 官方列出的 MinGW 6.3 MATLAB toolchain，因此两者不应混用对象文件；本项目只把 Embedded Coder 产出的源码交给 CMake/GCC 统一编译。

## 10. 仍需目标环境完成的验证

当前 Windows 主机没有 Ubuntu 容器/发行版，也没有 Isaac Sim。以下内容不能由本次本机结果代替：

- `h / h/2 / h/4` 收敛性验证；
- 在目标 Ubuntu 构建并用 `file`、`ldd`、`nm -D` 检查 `libavanti2.so`；
- 在目标 Ubuntu 的 Python 中重跑相同 20 秒、并行和生命周期测试；
- Isaac Sim 实际扩展进程加载、坐标变换、physics step 同步和大规模性能测试；
- 当前 20 秒用例是数值等价/接口稳定性测试，不是飞行品质或配平稳定性认证。

Ubuntu 不需要也不应安装 MATLAB。它只接收 `portable/avanti2`、根 `CMakeLists.txt`、`tests` 和 `python`；完整命令见 `README.md`。
