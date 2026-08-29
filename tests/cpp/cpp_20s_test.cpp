// C++ 20 秒仿真回归测试（对应 README 第 9 节的 C++ 用例）。
//
// 通过 C ABI 直接驱动生成模型：固定步长 0.02 s 下运行 1000 tick，
// 每秒打印一次 alpha/beta、位置、机体系速度和角速度，并验证：
//   1. 全部输出有限（无 NaN/Inf）；
//   2. 最终内部 next tick 为 20.0 s（时间契约正确）。
// 输入剧本与 Python 版本一致：0-5 s 油门 0.5，5-8 s 升降舵 -0.01，
// 8-11 s 副翼 0.01，11-14 s 方向舵 0.01，14-20 s X 轴风 1 m/s。
// 由 CTest 注册，产物为 build/win64/dist/avanti_cpp_20s.exe。

#include "avanti_c_api.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>

// 检查 ABI 状态码，非 0 则打印错误并退出（退出码 2）
static void require_ok(int32_t status, const char* operation) {
    if (status != AVANTI_OK) {
        std::cerr << operation << " failed with status " << status << '\n';
        std::exit(2);
    }
}

// 检查输出结构体中所有分量是否有限
static bool finite_output(const AvantiOutput& output) {
    if (!std::isfinite(output.sample_time_s) ||
        !std::isfinite(output.alpha_rad) ||
        !std::isfinite(output.beta_rad)) return false;
    for (std::size_t index = 0; index < 3; ++index) {
        if (!std::isfinite(output.position_ne_alt_m[index]) ||
            !std::isfinite(output.velocity_ne_up_mps[index]) ||
            !std::isfinite(output.euler_rpy_deg[index]) ||
            !std::isfinite(output.velocity_body_mps[index]) ||
            !std::isfinite(output.rates_body_radps[index]) ||
            !std::isfinite(output.rates_body_derivative_radps2[index]) ||
            !std::isfinite(output.acceleration_body_mps2[index])) return false;
    }
    return true;
}

// 按经过时间构造固定输入剧本（与 Simulink 基准生成脚本一致）
static AvantiInput test_input(double elapsed_s) {
    AvantiInput input = {};
    input.throttle_01 = 0.5;
    if (elapsed_s >= 5.0 && elapsed_s < 8.0) input.delta_e_norm = -0.01;
    if (elapsed_s >= 8.0 && elapsed_s < 11.0) input.delta_a_norm = 0.01;
    if (elapsed_s >= 11.0 && elapsed_s < 14.0) input.delta_r_norm = 0.01;
    if (elapsed_s >= 14.0) input.wind_flat_earth_mps[0] = 1.0;
    return input;
}

int main() {
    // 在 epoch 0 s 创建模型实例
    AvantiHandle handle = 0;
    require_ok(avanti_create(0.0, &handle), "avanti_create");

    // 读取固定步长，并据此计算总 tick 数与打印间隔
    double fixed_step_s = 0.0;
    require_ok(avanti_get_fixed_step(handle, &fixed_step_s),
               "avanti_get_fixed_step");
    const uint64_t ticks = static_cast<uint64_t>(std::llround(20.0 / fixed_step_s));
    const uint64_t print_stride = static_cast<uint64_t>(std::llround(1.0 / fixed_step_s));

    std::cout << "time_s,alpha_rad,beta_rad,north_m,east_m,alt_m,u_mps,v_mps,w_mps,p_radps,q_radps,r_radps\n";
    AvantiOutput output = {};
    for (uint64_t tick = 0; tick < ticks; ++tick) {
        // 用整数 tick 计算时间，避免反复累加步长造成浮点漂移
        const double time_s = static_cast<double>(tick) * fixed_step_s;
        const AvantiInput input = test_input(time_s);
        require_ok(avanti_tick(handle, time_s, &input, &output), "avanti_tick");
        if (!finite_output(output)) {
            std::cerr << "Non-finite output at tick " << tick << '\n';
            return 3;
        }
        if (tick % print_stride == 0 || tick + 1 == ticks) {
            std::cout << std::setprecision(10)
                      << output.sample_time_s << ',' << output.alpha_rad << ','
                      << output.beta_rad << ',' << output.position_ne_alt_m[0] << ','
                      << output.position_ne_alt_m[1] << ',' << output.position_ne_alt_m[2] << ','
                      << output.velocity_body_mps[0] << ',' << output.velocity_body_mps[1] << ','
                      << output.velocity_body_mps[2] << ',' << output.rates_body_radps[0] << ','
                      << output.rates_body_radps[1] << ',' << output.rates_body_radps[2] << '\n';
        }
    }

    // 验证 20 秒跑完后内部 next tick 恰为 20.0 s
    double next_time_s = 0.0;
    require_ok(avanti_get_next_tick_time(handle, &next_time_s),
               "avanti_get_next_tick_time");
    if (std::abs(next_time_s - 20.0) > 1.0e-12) {
        std::cerr << "Unexpected next tick time " << next_time_s << '\n';
        return 4;
    }

    require_ok(avanti_destroy(handle), "avanti_destroy");
    std::cout << "C++ 20 s simulation passed; ticks=" << ticks << '\n';
    return 0;
}
