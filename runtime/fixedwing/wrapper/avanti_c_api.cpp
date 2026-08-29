// AVANTI2 稳定 C ABI bridge 实现。
//
// 职责：把 Embedded Coder 生成的 C++ class（AVANTI2_ertmodel0827ModelClass）
// 包装为 avanti_c_api.h 声明的纯 C 接口：
//   - 每个 AvantiHandleImpl 拥有一个独立的 Model 实例（连续状态、ODE4
//     求解器数据、block state、输入输出互不共享）；
//   - 输入写入与输出读取均逐字段显式赋值，不用 memcpy 绑定生成布局，
//     模型端口一旦变化会在本文件编译期暴露不一致；
//   - 所有 C++ 异常都在 ABI 边界捕获并转换为状态码，不跨边界传播；
//   - 时间契约见 avanti_tick / avanti_advance_to 内的注释。
#include "avanti_c_api.h"
#include "AVANTI2_ertmodel0827.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <new>

using Model = AVANTI2_ertmodel0827ModelClass;

// 单个模型实例的内部实现（对调用方不透明）
struct AvantiHandleImpl {
    std::unique_ptr<Model> model;  // 生成的模型对象，独占所有权
    double epoch_s;                // 本实例的时间零点（Isaac 全局 epoch）
    uint64_t ticks_completed;      // 已完成的 tick 数，用于计算 next tick 时间

    explicit AvantiHandleImpl(double epoch)
        : model(new Model()), epoch_s(epoch), ticks_completed(0u) {
        model->initialize();  // 初始化固化初值与求解器状态
    }

    ~AvantiHandleImpl() {
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

// 检查输入 7 个分量是否全部有限
static bool finite_input(const AvantiInput& input) {
    return std::isfinite(input.delta_e_norm) &&
           std::isfinite(input.delta_a_norm) &&
           std::isfinite(input.delta_r_norm) &&
           std::isfinite(input.throttle_01) &&
           std::isfinite(input.wind_flat_earth_mps[0]) &&
           std::isfinite(input.wind_flat_earth_mps[1]) &&
           std::isfinite(input.wind_flat_earth_mps[2]);
}

// 从生成模型的 RTM 读取固定积分步长（当前 0.02 s）
static double fixed_step(const AvantiHandleImpl& context) {
    return context.model->getRTM()->Timing.stepSize0;
}

// 下一次 tick 期望的全局时间 = epoch + 已完成 tick 数 × 步长
static double next_tick_time(const AvantiHandleImpl& context) {
    return context.epoch_s +
           static_cast<double>(context.ticks_completed) * fixed_step(context);
}

// 时间网格容差：浮点舍入余量 + 与步长成正比的小量。
// 用于吸收 epoch + k*h 计算的舍入误差，但仍是网格对齐校验而非任意时间插值。
static double time_tolerance(double time_s, double fixed_step_s) {
    const double scale = std::max(1.0, std::abs(time_s));
    return 64.0 * std::numeric_limits<double>::epsilon() * scale +
           1.0e-9 * fixed_step_s;
}

// 把 ABI 输入逐字段写入生成模型的 ExtU 结构体（不依赖生成布局的 memcpy）
static void write_input(Model& model, const AvantiInput& input) {
    Model::ExtU_AVANTI2_ertmodel0827_T generated_input = {};
    generated_input.delta_e = input.delta_e_norm;
    generated_input.delta_a = input.delta_a_norm;
    generated_input.delta_r = input.delta_r_norm;
    generated_input.delta_t = input.throttle_01;
    generated_input.w_x = input.wind_flat_earth_mps[0];
    generated_input.w_y = input.wind_flat_earth_mps[1];
    generated_input.w_z = input.wind_flat_earth_mps[2];
    model.setExternalInputs(&generated_input);
}

// 把生成模型的 ExtY（NED[9] + BODY[12] 两个扁平数组）逐字段展开为 ABI 输出。
// 注意：生成代码中四个输出赋值位于 rtmIsMajorTimeStep 保护内，
// step() 返回的 ExtY 是当前 major tick 的快照，不是 ODE4 minor step 残留值。
static void read_output(const Model& model,
                        double sample_time_s,
                        AvantiOutput& output) {
    const Model::ExtY_AVANTI2_ertmodel0827_T& generated_output =
        model.getExternalOutputs();

    output.sample_time_s = sample_time_s;
    output.alpha_rad = generated_output.alpha;
    output.beta_rad = generated_output.beta;

    // NED[0..8] → 位置(North/East/Alt=-Ze)、速度(Ve_x/Ve_y/Up=-Ve_z)、姿态角(deg)
    output.position_ne_alt_m[0] = generated_output.NED[0];
    output.position_ne_alt_m[1] = generated_output.NED[1];
    output.position_ne_alt_m[2] = generated_output.NED[2];
    output.velocity_ne_up_mps[0] = generated_output.NED[3];
    output.velocity_ne_up_mps[1] = generated_output.NED[4];
    output.velocity_ne_up_mps[2] = generated_output.NED[5];
    output.euler_rpy_deg[0] = generated_output.NED[6];
    output.euler_rpy_deg[1] = generated_output.NED[7];
    output.euler_rpy_deg[2] = generated_output.NED[8];

    // BODY[0..11] → 机体系速度 u,v,w；角速度 p,q,r；角加速度；线加速度
    output.velocity_body_mps[0] = generated_output.BODY[0];
    output.velocity_body_mps[1] = generated_output.BODY[1];
    output.velocity_body_mps[2] = generated_output.BODY[2];
    output.rates_body_radps[0] = generated_output.BODY[3];
    output.rates_body_radps[1] = generated_output.BODY[4];
    output.rates_body_radps[2] = generated_output.BODY[5];
    output.rates_body_derivative_radps2[0] = generated_output.BODY[6];
    output.rates_body_derivative_radps2[1] = generated_output.BODY[7];
    output.rates_body_derivative_radps2[2] = generated_output.BODY[8];
    output.acceleration_body_mps2[0] = generated_output.BODY[9];
    output.acceleration_body_mps2[1] = generated_output.BODY[10];
    output.acceleration_body_mps2[2] = generated_output.BODY[11];
}

// C ABI 导出区：所有入口一律捕获 C++ 异常，保证不跨 ABI 边界传播
extern "C" {

uint32_t avanti_abi_version(void) {
    return AVANTI_ABI_VERSION;
}

uint32_t avanti_input_size(void) {
    return static_cast<uint32_t>(sizeof(AvantiInput));
}

uint32_t avanti_output_size(void) {
    return static_cast<uint32_t>(sizeof(AvantiOutput));
}

int32_t avanti_create(double epoch_s, AvantiHandle* out_handle) {
    if (!out_handle) return AVANTI_ERR_NULL;
    *out_handle = 0;
    if (!std::isfinite(epoch_s)) return AVANTI_ERR_NONFINITE;
    try {
        std::unique_ptr<AvantiHandleImpl> context(
            new (std::nothrow) AvantiHandleImpl(epoch_s));
        if (!context) return AVANTI_ERR_ALLOCATION;
        // 创建后校验模型步长有效，防止配置损坏的模型被使用
        const double step = fixed_step(*context);
        if (!(step > 0.0) || !std::isfinite(step)) return AVANTI_ERR_MODEL;
        *out_handle = context.release();
        return AVANTI_OK;
    } catch (...) {
        return AVANTI_ERR_EXCEPTION;
    }
}

int32_t avanti_destroy(AvantiHandle handle) {
    if (!handle) return AVANTI_ERR_NULL;
    try {
        delete handle;
        return AVANTI_OK;
    } catch (...) {
        return AVANTI_ERR_EXCEPTION;
    }
}

int32_t avanti_reset(AvantiHandle handle, double epoch_s) {
    if (!handle) return AVANTI_ERR_NULL;
    if (!std::isfinite(epoch_s)) return AVANTI_ERR_NONFINITE;
    try {
        handle->reset(epoch_s);
        return AVANTI_OK;
    } catch (const std::bad_alloc&) {
        return AVANTI_ERR_ALLOCATION;
    } catch (...) {
        return AVANTI_ERR_EXCEPTION;
    }
}

int32_t avanti_get_fixed_step(AvantiHandle handle, double* out_h_s) {
    if (!handle || !out_h_s) return AVANTI_ERR_NULL;
    *out_h_s = fixed_step(*handle);
    return AVANTI_OK;
}

int32_t avanti_get_next_tick_time(AvantiHandle handle, double* out_time_s) {
    if (!handle || !out_time_s) return AVANTI_ERR_NULL;
    *out_time_s = next_tick_time(*handle);
    return AVANTI_OK;
}

// 一次 tick 的语义（见 interface.md 第 7/8 节）：
//   1) 校验时间网格（t_k = epoch + k*h，在容差内）与输入有限性；
//   2) 在 major tick 采样输入，整个 ODE4 步内保持；
//   3) 调用一次生成类 step()，内部连续状态积分到 t_k+h；
//   4) 返回标记为 t_k 的 major-step 输出快照 y(t_k)；
//   5) 下一次只接受 t_k+h。
int32_t avanti_tick(AvantiHandle handle,
                    double global_time_s,
                    const AvantiInput* input,
                    AvantiOutput* output) {
    if (!handle || !input || !output) return AVANTI_ERR_NULL;
    if (!std::isfinite(global_time_s) || !finite_input(*input)) {
        return AVANTI_ERR_NONFINITE;
    }

    try {
        const double step = fixed_step(*handle);
        const double expected = next_tick_time(*handle);
        const double tolerance = time_tolerance(expected, step);
        // 早于期望网格点（超过容差）视为时间倒退
        if (global_time_s < expected - tolerance) return AVANTI_ERR_TIME_BACKWARDS;
        // 偏离期望网格点（超过容差）视为不在 0.02 s 网格上
        if (std::abs(global_time_s - expected) > tolerance) {
            return AVANTI_ERR_TIME_NOT_ON_GRID;
        }

        write_input(*handle->model, *input);
        handle->model->step();
        read_output(*handle->model, expected, *output);
        ++handle->ticks_completed;

        // step() 之后检查模型是否置了错误状态
        return rtmGetErrorStatus(handle->model->getRTM())
            ? AVANTI_ERR_MODEL : AVANTI_OK;
    } catch (...) {
        return AVANTI_ERR_EXCEPTION;
    }
}

// 用同一个 held_input 连续推进多个网格 tick，直到 target_time_s（含该点）。
// 仅当所有中间子步的控制量确实保持不变时才能使用；逐子步控制变化时必须
// 逐次调用 avanti_tick。max_substeps 防止目标过远时无界追赶。
int32_t avanti_advance_to(AvantiHandle handle,
                          double target_time_s,
                          const AvantiInput* held_input,
                          uint64_t max_substeps,
                          AvantiOutput* output) {
    if (!handle || !held_input || !output) return AVANTI_ERR_NULL;
    if (!std::isfinite(target_time_s) || !finite_input(*held_input)) {
        return AVANTI_ERR_NONFINITE;
    }

    const double step = fixed_step(*handle);
    const double first = next_tick_time(*handle);
    const double tolerance = time_tolerance(target_time_s, step);
    if (target_time_s < first - tolerance) return AVANTI_ERR_TIME_BACKWARDS;

    // 目标时间必须落在 0.02 s 网格上（按步长取整后残差在容差内）
    const double intervals = (target_time_s - first) / step;
    const double rounded = std::nearbyint(intervals);
    if (std::abs(intervals - rounded) > tolerance / step) {
        return AVANTI_ERR_TIME_NOT_ON_GRID;
    }

    // 需要调用的 tick 数 = 网格间隔数 + 1（包含 target 本身）
    const uint64_t calls = static_cast<uint64_t>(rounded) + 1u;
    if (calls > max_substeps) return AVANTI_ERR_TOO_MANY_SUBSTEPS;
    for (uint64_t call = 0; call < calls; ++call) {
        const int32_t status = avanti_tick(
            handle, next_tick_time(*handle), held_input, output);
        if (status != AVANTI_OK) return status;
    }
    return AVANTI_OK;
}

}  // extern "C"
