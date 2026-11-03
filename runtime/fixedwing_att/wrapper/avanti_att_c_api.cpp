// AVANTI2 姿态内环控制器稳定 C ABI bridge 实现。
//
// 职责：把 Embedded Coder 生成的 C++ class（PX4Controller_ertmodel0827ModelClass，
// 即“期望姿态角/油门/风速”为根输入、内部姿态内环控制器 + AVANTI2 Plant 的整体
// 模型）包装为 avanti_att_c_api.h 声明的纯 C 接口：
//   - 每个 AvantiAttHandleImpl 拥有一个独立的 Model 实例（连续状态、ODE4
//     求解器数据、block state、输入输出互不共享）；
//   - 输入写入与输出读取均逐字段显式赋值，不用 memcpy 绑定生成布局；
//   - 14 个控制器增益位于生成类的 class-static 参数结构体
//     （PX4Controller_ertmodel0827_P，Embedded Coder "C++ class" 目标对独立
//     顶层模型的固有限制：参数无法像状态那样逐实例隔离），因此
//     avanti_att_get/set_params 是进程级操作，不针对某个 handle；
//   - 所有 C++ 异常都在 ABI 边界捕获并转换为状态码，不跨边界传播。
#include "avanti_att_c_api.h"
#include "PX4Controller_ertmodel0827.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <new>

using Model = PX4Controller_ertmodel0827ModelClass;

// 单个模型实例的内部实现（对调用方不透明）
struct AvantiAttHandleImpl {
    std::unique_ptr<Model> model;  // 生成的模型对象，独占所有权
    double epoch_s;                // 本实例的时间零点（Isaac 全局 epoch）
    uint64_t ticks_completed;      // 已完成的 tick 数，用于计算 next tick 时间

    explicit AvantiAttHandleImpl(double epoch)
        : model(new Model()), epoch_s(epoch), ticks_completed(0u) {
        model->initialize();  // 初始化固化初值与求解器状态（不影响控制器增益）
    }

    ~AvantiAttHandleImpl() {
        if (model) model->terminate();
    }

    // 销毁重建模型回到固化初值，并重置时间零点与 tick 计数
    void reset(double epoch) {
        if (model) model->terminate();
        model.reset(new Model());
        model->initialize();
        epoch_s = epoch;
        ticks_completed = 0u;
    }
};

// 检查输入 6 个标量分量是否全部有限
static bool finite_input(const AvantiAttInput& input) {
    return std::isfinite(input.phi_sp_rad) &&
           std::isfinite(input.theta_sp_rad) &&
           std::isfinite(input.throttle_01) &&
           std::isfinite(input.wind_flat_earth_mps[0]) &&
           std::isfinite(input.wind_flat_earth_mps[1]) &&
           std::isfinite(input.wind_flat_earth_mps[2]);
}

static bool finite_params(const AvantiAttParams& p) {
    const double* begin = &p.p_phi_sp;
    const double* end = begin + (sizeof(AvantiAttParams) / sizeof(double));
    for (const double* it = begin; it != end; ++it) {
        if (!std::isfinite(*it)) return false;
    }
    return true;
}

// 从生成模型的 RTM 读取固定积分步长（当前 0.02 s）
static double fixed_step(const AvantiAttHandleImpl& context) {
    return context.model->getRTM()->Timing.stepSize0;
}

// 下一次 tick 期望的全局时间 = epoch + 已完成 tick 数 × 步长
static double next_tick_time(const AvantiAttHandleImpl& context) {
    return context.epoch_s +
           static_cast<double>(context.ticks_completed) * fixed_step(context);
}

// 时间网格容差：浮点舍入余量 + 与步长成正比的小量
static double time_tolerance(double time_s, double fixed_step_s) {
    const double scale = std::max(1.0, std::abs(time_s));
    return 64.0 * std::numeric_limits<double>::epsilon() * scale +
           1.0e-9 * fixed_step_s;
}

// 把 ABI 输入逐字段写入生成模型的 ExtU 结构体
static void write_input(Model& model, const AvantiAttInput& input) {
    Model::ExtU_PX4Controller_ertmodel08_T generated_input = {};
    generated_input.phi_sp = input.phi_sp_rad;
    generated_input.theta_sp = input.theta_sp_rad;
    generated_input.T_cmd = input.throttle_01;
    generated_input.w_speed[0] = input.wind_flat_earth_mps[0];
    generated_input.w_speed[1] = input.wind_flat_earth_mps[1];
    generated_input.w_speed[2] = input.wind_flat_earth_mps[2];
    model.setExternalInputs(&generated_input);
}

// 把生成模型的 ExtY（POS_ned[9] + STATE_body[12]，与 avanti_c_api.h 的
// NED[9]/BODY[12] 语义相同，因为共用同一个 Plant）逐字段展开为 ABI 输出。
static void read_output(const Model& model,
                        double sample_time_s,
                        AvantiAttOutput& output) {
    const Model::ExtY_PX4Controller_ertmodel08_T& generated_output =
        model.getExternalOutputs();

    output.sample_time_s = sample_time_s;
    output.alpha_rad = generated_output.alpha;
    output.beta_rad = generated_output.beta;

    output.position_ne_alt_m[0] = generated_output.POS_ned[0];
    output.position_ne_alt_m[1] = generated_output.POS_ned[1];
    output.position_ne_alt_m[2] = generated_output.POS_ned[2];
    output.velocity_ne_up_mps[0] = generated_output.POS_ned[3];
    output.velocity_ne_up_mps[1] = generated_output.POS_ned[4];
    output.velocity_ne_up_mps[2] = generated_output.POS_ned[5];
    output.euler_rpy_deg[0] = generated_output.POS_ned[6];
    output.euler_rpy_deg[1] = generated_output.POS_ned[7];
    output.euler_rpy_deg[2] = generated_output.POS_ned[8];

    output.velocity_body_mps[0] = generated_output.STATE_body[0];
    output.velocity_body_mps[1] = generated_output.STATE_body[1];
    output.velocity_body_mps[2] = generated_output.STATE_body[2];
    output.rates_body_radps[0] = generated_output.STATE_body[3];
    output.rates_body_radps[1] = generated_output.STATE_body[4];
    output.rates_body_radps[2] = generated_output.STATE_body[5];
    output.rates_body_derivative_radps2[0] = generated_output.STATE_body[6];
    output.rates_body_derivative_radps2[1] = generated_output.STATE_body[7];
    output.rates_body_derivative_radps2[2] = generated_output.STATE_body[8];
    output.acceleration_body_mps2[0] = generated_output.STATE_body[9];
    output.acceleration_body_mps2[1] = generated_output.STATE_body[10];
    output.acceleration_body_mps2[2] = generated_output.STATE_body[11];
}

// 逐字段把生成的 class-static 参数结构体映射为 AvantiAttParams（见
// matlab/px4att_param_field_map.m，字段名由 Simulink 自动消歧义生成，
// 重新生成代码后必须重新核对）。
static void read_params(const Model::P_PX4Controller_ertmodel0827_T& p,
                        AvantiAttParams& out) {
    out.p_phi_sp = p.P_phi_sp_Gain;
    out.p_theta_sp = p.P_theta_sp_Gain;
    out.fw_rr = p.FFGain_Gain;
    out.fw_pr = p.FFGain_Gain_d;
    out.fw_yr = p.FFGain_Gain_c;
    out.omega_sp_p_phi = p.DiscretePDController_P;
    out.omega_sp_i_phi = p.DiscretePDController_I;
    out.omega_sp_d_phi = p.DiscretePDController_D;
    out.omega_sp_p_theta = p.DiscretePDController_P_k;
    out.omega_sp_i_theta = p.DiscretePDController_I_c;
    out.omega_sp_d_theta = p.DiscretePDController_D_o;
    out.omega_sp_p_psi = p.DiscretePDController_P_n;
    out.omega_sp_i_psi = p.DiscretePDController_I_a;
    out.omega_sp_d_psi = p.DiscretePDController_D_h;
}

static void write_params(Model::P_PX4Controller_ertmodel0827_T& p,
                         const AvantiAttParams& in) {
    p.P_phi_sp_Gain = in.p_phi_sp;
    p.P_theta_sp_Gain = in.p_theta_sp;
    p.FFGain_Gain = in.fw_rr;
    p.FFGain_Gain_d = in.fw_pr;
    p.FFGain_Gain_c = in.fw_yr;
    p.DiscretePDController_P = in.omega_sp_p_phi;
    p.DiscretePDController_I = in.omega_sp_i_phi;
    p.DiscretePDController_D = in.omega_sp_d_phi;
    p.DiscretePDController_P_k = in.omega_sp_p_theta;
    p.DiscretePDController_I_c = in.omega_sp_i_theta;
    p.DiscretePDController_D_o = in.omega_sp_d_theta;
    p.DiscretePDController_P_n = in.omega_sp_p_psi;
    p.DiscretePDController_I_a = in.omega_sp_i_psi;
    p.DiscretePDController_D_h = in.omega_sp_d_psi;
}

// 出厂默认增益：库加载后首次访问时，从生成代码的编译期初值捕获一次快照
// （PX4Controller_ertmodel0827_P 是 class-static，程序启动时已按生成的
// _data.cpp 初始化，早于任何 avanti_att_set_params 调用）。
static const AvantiAttParams& default_params_snapshot() {
    static const AvantiAttParams snapshot = [] {
        AvantiAttParams p = {};
        read_params(Model::PX4Controller_ertmodel0827_P, p);
        return p;
    }();
    return snapshot;
}

// 保护对 class-static 参数结构体的并发读写（增益是进程级状态，可能被多个
// 线程以不同 handle 同时调用 tick 时读取，也可能被另一线程同时调用
// avanti_att_set_params 写入）。
static std::mutex& params_mutex() {
    static std::mutex m;
    return m;
}

// 强制在动态库加载时（静态初始化阶段，早于任何导出函数可被调用）就捕获
// 出厂默认值，避免调用方在 avanti_att_create 之前就调用
// avanti_att_set_params 时，把已修改的值误当成默认值捕获。
namespace {
struct CaptureDefaultParamsAtLoad {
    CaptureDefaultParamsAtLoad() { (void)default_params_snapshot(); }
} g_capture_default_params_at_load;
}  // namespace

// C ABI 导出区：所有入口一律捕获 C++ 异常，保证不跨 ABI 边界传播
extern "C" {

uint32_t avanti_att_abi_version(void) {
    return AVANTI_ATT_ABI_VERSION;
}

uint32_t avanti_att_input_size(void) {
    return static_cast<uint32_t>(sizeof(AvantiAttInput));
}

uint32_t avanti_att_output_size(void) {
    return static_cast<uint32_t>(sizeof(AvantiAttOutput));
}

uint32_t avanti_att_params_size(void) {
    return static_cast<uint32_t>(sizeof(AvantiAttParams));
}

int32_t avanti_att_create(double epoch_s, AvantiAttHandle* out_handle) {
    if (!out_handle) return AVANTI_ATT_ERR_NULL;
    *out_handle = 0;
    if (!std::isfinite(epoch_s)) return AVANTI_ATT_ERR_NONFINITE;
    try {
        // 确保出厂默认值在首次创建实例之前已经被捕获快照
        (void)default_params_snapshot();
        std::unique_ptr<AvantiAttHandleImpl> context(
            new (std::nothrow) AvantiAttHandleImpl(epoch_s));
        if (!context) return AVANTI_ATT_ERR_ALLOCATION;
        const double step = fixed_step(*context);
        if (!(step > 0.0) || !std::isfinite(step)) return AVANTI_ATT_ERR_MODEL;
        *out_handle = context.release();
        return AVANTI_ATT_OK;
    } catch (...) {
        return AVANTI_ATT_ERR_EXCEPTION;
    }
}

int32_t avanti_att_destroy(AvantiAttHandle handle) {
    if (!handle) return AVANTI_ATT_ERR_NULL;
    try {
        delete handle;
        return AVANTI_ATT_OK;
    } catch (...) {
        return AVANTI_ATT_ERR_EXCEPTION;
    }
}

int32_t avanti_att_reset(AvantiAttHandle handle, double epoch_s) {
    if (!handle) return AVANTI_ATT_ERR_NULL;
    if (!std::isfinite(epoch_s)) return AVANTI_ATT_ERR_NONFINITE;
    try {
        handle->reset(epoch_s);
        return AVANTI_ATT_OK;
    } catch (const std::bad_alloc&) {
        return AVANTI_ATT_ERR_ALLOCATION;
    } catch (...) {
        return AVANTI_ATT_ERR_EXCEPTION;
    }
}

int32_t avanti_att_get_fixed_step(AvantiAttHandle handle, double* out_h_s) {
    if (!handle || !out_h_s) return AVANTI_ATT_ERR_NULL;
    *out_h_s = fixed_step(*handle);
    return AVANTI_ATT_OK;
}

int32_t avanti_att_get_next_tick_time(AvantiAttHandle handle, double* out_time_s) {
    if (!handle || !out_time_s) return AVANTI_ATT_ERR_NULL;
    *out_time_s = next_tick_time(*handle);
    return AVANTI_ATT_OK;
}

// 一次 tick 的语义（与 avanti_c_api.h 的 avanti_tick 相同契约）：
//   1) 校验时间网格（t_k = epoch + k*h，在容差内）与输入有限性；
//   2) 在 major tick 采样输入，整个 ODE4 步内保持；
//   3) 调用一次生成类 step()，内部连续状态积分到 t_k+h；
//   4) 返回标记为 t_k 的 major-step 输出快照 y(t_k)；
//   5) 下一次只接受 t_k+h。
int32_t avanti_att_tick(AvantiAttHandle handle,
                        double global_time_s,
                        const AvantiAttInput* input,
                        AvantiAttOutput* output) {
    if (!handle || !input || !output) return AVANTI_ATT_ERR_NULL;
    if (!std::isfinite(global_time_s) || !finite_input(*input)) {
        return AVANTI_ATT_ERR_NONFINITE;
    }

    try {
        const double step = fixed_step(*handle);
        const double expected = next_tick_time(*handle);
        const double tolerance = time_tolerance(expected, step);
        if (global_time_s < expected - tolerance) return AVANTI_ATT_ERR_TIME_BACKWARDS;
        if (std::abs(global_time_s - expected) > tolerance) {
            return AVANTI_ATT_ERR_TIME_NOT_ON_GRID;
        }

        write_input(*handle->model, *input);
        handle->model->step();
        read_output(*handle->model, expected, *output);
        ++handle->ticks_completed;

        return rtmGetErrorStatus(handle->model->getRTM())
            ? AVANTI_ATT_ERR_MODEL : AVANTI_ATT_OK;
    } catch (...) {
        return AVANTI_ATT_ERR_EXCEPTION;
    }
}

// 用同一个 held_input 连续推进多个网格 tick，直到 target_time_s（含该点）
int32_t avanti_att_advance_to(AvantiAttHandle handle,
                              double target_time_s,
                              const AvantiAttInput* held_input,
                              uint64_t max_substeps,
                              AvantiAttOutput* output) {
    if (!handle || !held_input || !output) return AVANTI_ATT_ERR_NULL;
    if (!std::isfinite(target_time_s) || !finite_input(*held_input)) {
        return AVANTI_ATT_ERR_NONFINITE;
    }

    const double step = fixed_step(*handle);
    const double first = next_tick_time(*handle);
    const double tolerance = time_tolerance(target_time_s, step);
    if (target_time_s < first - tolerance) return AVANTI_ATT_ERR_TIME_BACKWARDS;

    const double intervals = (target_time_s - first) / step;
    const double rounded = std::nearbyint(intervals);
    if (std::abs(intervals - rounded) > tolerance / step) {
        return AVANTI_ATT_ERR_TIME_NOT_ON_GRID;
    }

    const uint64_t calls = static_cast<uint64_t>(rounded) + 1u;
    if (calls > max_substeps) return AVANTI_ATT_ERR_TOO_MANY_SUBSTEPS;
    for (uint64_t call = 0; call < calls; ++call) {
        const int32_t status = avanti_att_tick(
            handle, next_tick_time(*handle), held_input, output);
        if (status != AVANTI_ATT_OK) return status;
    }
    return AVANTI_ATT_OK;
}

int32_t avanti_att_get_params(AvantiAttParams* out_params) {
    if (!out_params) return AVANTI_ATT_ERR_NULL;
    try {
        std::lock_guard<std::mutex> lock(params_mutex());
        read_params(Model::PX4Controller_ertmodel0827_P, *out_params);
        return AVANTI_ATT_OK;
    } catch (...) {
        return AVANTI_ATT_ERR_EXCEPTION;
    }
}

int32_t avanti_att_set_params(const AvantiAttParams* params) {
    if (!params) return AVANTI_ATT_ERR_NULL;
    if (!finite_params(*params)) return AVANTI_ATT_ERR_NONFINITE;
    try {
        std::lock_guard<std::mutex> lock(params_mutex());
        write_params(Model::PX4Controller_ertmodel0827_P, *params);
        return AVANTI_ATT_OK;
    } catch (...) {
        return AVANTI_ATT_ERR_EXCEPTION;
    }
}

int32_t avanti_att_get_default_params(AvantiAttParams* out_params) {
    if (!out_params) return AVANTI_ATT_ERR_NULL;
    try {
        *out_params = default_params_snapshot();
        return AVANTI_ATT_OK;
    } catch (...) {
        return AVANTI_ATT_ERR_EXCEPTION;
    }
}

}  // extern "C"
