// AVANTI2 稳定 C ABI 公共头文件（对外唯一接口声明）。
//
// 本头文件把 Embedded Coder 生成的 C++ class 封装为纯 C ABI：
//   - 不向调用方暴露生成的 C++ class、STL 类型或生成结构体布局；
//   - 因此 Windows/Linux 编译器 ABI 差异不会泄漏到调用方；
//   - C/C++ 与 ctypes 均按本头文件声明的布局对接（当前 x86-64 Windows：
//     sizeof(AvantiInput) = 56 字节，sizeof(AvantiOutput) = 192 字节）。
//
// 调用方在加载动态库后必须核对 avanti_abi_version()、avanti_input_size()
// 与 avanti_output_size()，不能假定结构体布局永远不变。
#ifndef AVANTI_C_API_H
#define AVANTI_C_API_H

#include <stdint.h>

// 导出宏：静态链接时为空；Windows DLL 构建时 dllexport、使用时 dllimport；
// 其他平台（Linux GCC）默认可见。
#if defined(AVANTI_STATIC)
#  define AVANTI_API
#elif defined(_WIN32)
#  if defined(AVANTI_BUILDING_DLL)
#    define AVANTI_API __declspec(dllexport)
#  else
#    define AVANTI_API __declspec(dllimport)
#  endif
#else
#  define AVANTI_API __attribute__((visibility("default")))
#endif

// C ABI 以 C 链接规范导出，避免 C++ name mangling
#ifdef __cplusplus
extern "C" {
#endif

// ABI 版本号（0x00010000）。结构体或语义破坏性变更时必须递增。
#define AVANTI_ABI_VERSION 0x00010000u

// 不透明句柄：内部是 AvantiHandleImpl*，对应一个独立的模型实例（一架飞机）。
typedef struct AvantiHandleImpl* AvantiHandle;

// C ABI 状态码
typedef enum AvantiStatus {
    AVANTI_OK = 0,                   // 成功
    AVANTI_ERR_NULL = 1,             // 空指针 / 空 handle
    AVANTI_ERR_ALLOCATION = 2,       // 内存分配失败
    AVANTI_ERR_NONFINITE = 3,        // 时间或输入含 NaN/Inf
    AVANTI_ERR_TIME_NOT_ON_GRID = 4, // 时间不在 0.02 s 网格上
    AVANTI_ERR_TIME_BACKWARDS = 5,   // 时间倒退
    AVANTI_ERR_TOO_MANY_SUBSTEPS = 6,// advance_to 超过保护上限
    AVANTI_ERR_MODEL = 7,            // 生成模型报告错误
    AVANTI_ERR_EXCEPTION = 8         // C++ 异常被 ABI 边界捕获
} AvantiStatus;

// 模型根层 7 个输入（均为 64 位 double）
typedef struct AvantiInput {
    double delta_e_norm;             // 升降舵归一化指令，建议 [-1,1]，映射约 [-12,+12] deg
    double delta_a_norm;             // 副翼归一化指令，建议 [-1,1]，映射约 [-10,+10] deg
    double delta_r_norm;             // 方向舵归一化指令，建议 [-1,1]，映射约 [-22,+22] deg
    double throttle_01;              // 油门，模型根层饱和到 [0,1]
    double wind_flat_earth_mps[3];   // flat-Earth 三轴风速 m/s（Z_e 向下为正）
} AvantiInput;

// 一次 major tick 的输出快照（1 个时间标量 + 2 个标量 + 7 个三维数组，共 23 个量）
typedef struct AvantiOutput {
    double sample_time_s;            // 本输出对应的外部全局时间，s
    double alpha_rad;                // 迎角，rad
    double beta_rad;                 // 侧滑角，rad
    double position_ne_alt_m[3];     // North(Xe)、East(Ye)、Altitude(=-Ze)，m
    double velocity_ne_up_mps[3];    // Ve_x、Ve_y、Up(=-Ve_z)，m/s
    double euler_rpy_deg[3];         // roll、pitch、yaw，deg（模型内已从 rad 转换）
    double velocity_body_mps[3];     // 机体系速度 u、v、w，m/s
    double rates_body_radps[3];      // 机体系角速度 p、q、r，rad/s
    double rates_body_derivative_radps2[3]; // p_dot、q_dot、r_dot，rad/s²
    double acceleration_body_mps2[3];       // 机体系加速度 u_dot、v_dot、w_dot，m/s²
} AvantiOutput;

// ---- 元数据（加载后必须核对的三个值）----
AVANTI_API uint32_t avanti_abi_version(void);
AVANTI_API uint32_t avanti_input_size(void);
AVANTI_API uint32_t avanti_output_size(void);

// ---- 实例生命周期 ----
// 在 epoch_s（Isaac 全局时间零点）创建模型实例，out_handle 输出不透明句柄
AVANTI_API int32_t avanti_create(double epoch_s, AvantiHandle* out_handle);
// 销毁实例并释放模型内存
AVANTI_API int32_t avanti_destroy(AvantiHandle handle);
// 销毁重建模型回到固化的编译期初值，并把时间零点设为新 epoch_s（episode 重置）
AVANTI_API int32_t avanti_reset(AvantiHandle handle, double epoch_s);

// ---- 查询 ----
// 固定积分步长（当前 0.02 s）
AVANTI_API int32_t avanti_get_fixed_step(AvantiHandle handle, double* out_h_s);
// 下一次 tick 期望的全局时间 = epoch_s + 已完成 tick 数 × 步长
AVANTI_API int32_t avanti_get_next_tick_time(AvantiHandle handle, double* out_time_s);

// ---- 推进 ----
// 推进一个时间网格点：校验时间网格与输入有限性 → 写入输入 → 调用一次
// 生成类 step()（内部完成整个 ODE4 步长，状态积到 t+h）→ 返回该 major tick
// 的输出快照 y(t)。下一次只接受 t+h。
AVANTI_API int32_t avanti_tick(AvantiHandle handle,
                               double global_time_s,
                               const AvantiInput* input,
                               AvantiOutput* output);
// 保持同一输入不变连续推进多个网格 tick 直到 target_time_s（含该点），
// 适用于中间子步输入不变的追赶；max_substeps 为失控追赶保护上限。
// 若每个子步控制量都变化，必须逐次调用 avanti_tick。
AVANTI_API int32_t avanti_advance_to(AvantiHandle handle,
                                     double target_time_s,
                                     const AvantiInput* held_input,
                                     uint64_t max_substeps,
                                     AvantiOutput* output);

#ifdef __cplusplus
}
#endif

#endif
