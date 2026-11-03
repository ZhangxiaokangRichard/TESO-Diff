// AVANTI2 姿态内环控制器稳定 C ABI 公共头文件（对外唯一接口声明）。
//
// 本头文件把 Embedded Coder 生成的 C++ class（来自 PX4Controller_ertmodel0827，
// 即 AVANTI2 机体 6DOF Plant 前串联姿态内环控制器后的整体模型）封装为纯 C ABI：
//   - 根输入从“舵面偏转”改为“期望姿态角 + 油门 + 风速”，由模型内的
//     InnerLoopControl（外环 P + 内环 PID + 协调转弯前馈）自动解算舵面；
//   - 根输出与 avanti_c_api.h 的 AvantiOutput 语义完全一致（同一个 Plant）；
//   - 不向调用方暴露生成的 C++ class、STL 类型或生成结构体布局；
//   - 新增 14 个姿态控制器增益的运行时可编辑接口（AvantiAttParams）。
//
// 调用方在加载动态库后必须核对 avanti_att_abi_version()、
// avanti_att_input_size()、avanti_att_output_size() 与
// avanti_att_params_size()，不能假定结构体布局永远不变。
//
// 重要（参数作用域，务必先读）：
//   Embedded Coder 的 "C++ class" 代码接口把可调参数统一放进一个
//   class-static 结构体（generated/codegen_att/.../PX4Controller_ertmodel0827.h
//   中的 PX4Controller_ertmodel0827_P），这是该代码生成目标对独立顶层模型的
//   固有行为，不是本 wrapper 的设计选择：C++ class 封装不支持逐参数设置
//   per-instance 存储类别（这一点与 ExtU/ExtY/DWork/连续状态不同，那些确实是
//   每个对象独立）。因此：
//     - avanti_att_get_params / avanti_att_set_params 不接受 AvantiHandle，
//       因为增益本就与具体实例无关；
//     - 同一进程内所有 AvantiAttHandle 共用同一组增益；修改后立刻对所有
//       实例的下一次 avanti_att_tick 生效；
//     - 需要不同实例使用不同增益时，必须在不同进程中分别加载本动态库。
#ifndef AVANTI_ATT_C_API_H
#define AVANTI_ATT_C_API_H

#include <stdint.h>

#if defined(AVANTI_STATIC)
#  define AVANTI_ATT_API
#elif defined(_WIN32)
#  if defined(AVANTI_ATT_BUILDING_DLL)
#    define AVANTI_ATT_API __declspec(dllexport)
#  else
#    define AVANTI_ATT_API __declspec(dllimport)
#  endif
#else
#  define AVANTI_ATT_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

// ABI 版本号（0x00010000）。结构体或语义破坏性变更时必须递增。
#define AVANTI_ATT_ABI_VERSION 0x00010000u

// 不透明句柄：内部是 AvantiAttHandleImpl*，对应一个独立的模型实例（一架飞机）。
typedef struct AvantiAttHandleImpl* AvantiAttHandle;

// C ABI 状态码（与 avanti_c_api.h 的 AvantiStatus 取值一致，便于共用调用约定）
typedef enum AvantiAttStatus {
    AVANTI_ATT_OK = 0,                   // 成功
    AVANTI_ATT_ERR_NULL = 1,             // 空指针 / 空 handle
    AVANTI_ATT_ERR_ALLOCATION = 2,       // 内存分配失败
    AVANTI_ATT_ERR_NONFINITE = 3,        // 时间或输入含 NaN/Inf
    AVANTI_ATT_ERR_TIME_NOT_ON_GRID = 4, // 时间不在 0.02 s 网格上
    AVANTI_ATT_ERR_TIME_BACKWARDS = 5,   // 时间倒退
    AVANTI_ATT_ERR_TOO_MANY_SUBSTEPS = 6,// advance_to 超过保护上限
    AVANTI_ATT_ERR_MODEL = 7,            // 生成模型报告错误
    AVANTI_ATT_ERR_EXCEPTION = 8         // C++ 异常被 ABI 边界捕获
} AvantiAttStatus;

// 模型根层 4 个输入（均为 64 位 double）。phi_sp/theta_sp 为 rad，
// 模型没有 psi_sp（偏航）外环角度设定点输入——偏航通道只有内部速率阻尼/
// 协调转弯前馈（见 AvantiAttParams.fw_yr 等），详见 interface_att.md。
typedef struct AvantiAttInput {
    double phi_sp_rad;               // 期望滚转角，rad（建议 |value| <= 0.7854 rad ≈ 45°）
    double theta_sp_rad;             // 期望俯仰角，rad（建议 |value| <= 0.7854 rad ≈ 45°）
    double throttle_01;              // 归一化油门，模型根层饱和到 [0,1]
    double wind_flat_earth_mps[3];   // flat-Earth 三轴风速 m/s（Z_e 向下为正，与 avanti_c_api.h 相同约定）
} AvantiAttInput;

// 一次 major tick 的输出快照，字段语义与 avanti_c_api.h 的 AvantiOutput 完全
// 相同（同一个 Plant 子系统），因为本模型只是在 AVANTI2 Plant 前串联了姿态
// 内环控制器。
typedef struct AvantiAttOutput {
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
} AvantiAttOutput;

// 14 个姿态内环控制器增益（ModelWorkspace 原始变量名见注释），进程内所有
// 实例共享（见文件头“参数作用域”说明）。字段顺序与
// matlab/px4att_gain_names.m 一致。
typedef struct AvantiAttParams {
    double p_phi_sp;        // ModelWorkspace: P_phi_sp      外环滚转角 P 增益
    double p_theta_sp;      // ModelWorkspace: P_theta_sp    外环俯仰角 P 增益
    double fw_rr;           // ModelWorkspace: FW_RR         滚转速率前馈增益
    double fw_pr;           // ModelWorkspace: FW_PR         俯仰速率前馈增益
    double fw_yr;           // ModelWorkspace: FW_YR         偏航速率前馈增益（协调转弯）
    double omega_sp_p_phi;  // ModelWorkspace: Omega_sp_PPhi   滚转速率 PID 的 P
    double omega_sp_i_phi;  // ModelWorkspace: Omega_sp_IPhi   滚转速率 PID 的 I
    double omega_sp_d_phi;  // ModelWorkspace: Omega_sp_DPhi   滚转速率 PID 的 D
    double omega_sp_p_theta;// ModelWorkspace: Omega_sp_PTheta 俯仰速率 PID 的 P
    double omega_sp_i_theta;// ModelWorkspace: Omega_sp_ITheta 俯仰速率 PID 的 I
    double omega_sp_d_theta;// ModelWorkspace: Omega_sp_DTheta 俯仰速率 PID 的 D
    double omega_sp_p_psi;  // ModelWorkspace: Omega_sp_PPsi   偏航速率 PID 的 P
    double omega_sp_i_psi;  // ModelWorkspace: Omega_sp_IPsi   偏航速率 PID 的 I
    double omega_sp_d_psi;  // ModelWorkspace: Omega_sp_DPsi   偏航速率 PID 的 D
} AvantiAttParams;

// ---- 元数据（加载后必须核对的四个值）----
AVANTI_ATT_API uint32_t avanti_att_abi_version(void);
AVANTI_ATT_API uint32_t avanti_att_input_size(void);
AVANTI_ATT_API uint32_t avanti_att_output_size(void);
AVANTI_ATT_API uint32_t avanti_att_params_size(void);

// ---- 实例生命周期 ----
// 在 epoch_s（Isaac 全局时间零点）创建模型实例，out_handle 输出不透明句柄
AVANTI_ATT_API int32_t avanti_att_create(double epoch_s, AvantiAttHandle* out_handle);
// 销毁实例并释放模型内存
AVANTI_ATT_API int32_t avanti_att_destroy(AvantiAttHandle handle);
// 销毁重建模型回到固化的编译期初值，并把时间零点设为新 epoch_s（episode 重置）。
// 不影响控制器增益（增益是进程级状态，见 avanti_att_set_params）。
AVANTI_ATT_API int32_t avanti_att_reset(AvantiAttHandle handle, double epoch_s);

// ---- 查询 ----
// 固定积分步长（当前 0.02 s）
AVANTI_ATT_API int32_t avanti_att_get_fixed_step(AvantiAttHandle handle, double* out_h_s);
// 下一次 tick 期望的全局时间 = epoch_s + 已完成 tick 数 × 步长
AVANTI_ATT_API int32_t avanti_att_get_next_tick_time(AvantiAttHandle handle, double* out_time_s);

// ---- 推进 ----
// 推进一个时间网格点：校验时间网格与输入有限性 → 写入输入 → 调用一次
// 生成类 step()（内部完成整个 ODE4 步长，状态积到 t+h）→ 返回该 major tick
// 的输出快照 y(t)。下一次只接受 t+h。
AVANTI_ATT_API int32_t avanti_att_tick(AvantiAttHandle handle,
                                       double global_time_s,
                                       const AvantiAttInput* input,
                                       AvantiAttOutput* output);
// 保持同一输入不变连续推进多个网格 tick 直到 target_time_s（含该点），
// 适用于中间子步输入不变的追赶；max_substeps 为失控追赶保护上限。
// 若每个子步控制量都变化，必须逐次调用 avanti_att_tick。
AVANTI_ATT_API int32_t avanti_att_advance_to(AvantiAttHandle handle,
                                             double target_time_s,
                                             const AvantiAttInput* held_input,
                                             uint64_t max_substeps,
                                             AvantiAttOutput* output);

// ---- 姿态控制器增益（进程级，与具体 handle 无关，见文件头“参数作用域”）----
// 读取当前生效的 14 个增益
AVANTI_ATT_API int32_t avanti_att_get_params(AvantiAttParams* out_params);
// 写入新的 14 个增益；立刻对进程内所有实例的下一次 avanti_att_tick 生效。
// 任一字段非有限数（NaN/Inf）则整体拒绝，不做部分写入。
AVANTI_ATT_API int32_t avanti_att_set_params(const AvantiAttParams* params);
// 读取代码生成时固化的出厂默认增益（库加载时捕获一次，不随
// avanti_att_set_params 改变，可用于恢复出厂值）。
AVANTI_ATT_API int32_t avanti_att_get_default_params(AvantiAttParams* out_params);

#ifdef __cplusplus
}
#endif

#endif
