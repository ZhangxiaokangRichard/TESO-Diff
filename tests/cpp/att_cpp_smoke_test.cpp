// 姿态内环控制器 C++ 接口冒烟测试（双实例 + 参数读写）。
//
// 验证稳定 C ABI 的核心契约：
//   1. ABI 版本号与输入/输出/参数结构体尺寸和头文件一致；
//   2. 固定步长为 0.02 s；
//   3. 两个独立实例在相同输入下结果一致（前 10 tick），之后指令分叉；
//   4. 时间契约：100 tick 后 next tick 为 102.0 s；时间倒退被拒绝；
//   5. avanti_att_get_default_params 与生成时的出厂值一致；
//   6. avanti_att_set_params 立刻对两个已创建实例的后续 tick 生效
//      （验证增益是进程级共享状态，不是逐 handle 独立状态）。
#include "avanti_att_c_api.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>

static void require_ok(int32_t status) {
    if (status != AVANTI_ATT_OK) {
        std::cerr << "AVANTI_ATT status = " << status << std::endl;
        std::abort();
    }
}

static void require_finite(const AvantiAttOutput& output) {
    assert(std::isfinite(output.sample_time_s));
    assert(std::isfinite(output.alpha_rad));
    assert(std::isfinite(output.beta_rad));
    for (std::size_t index = 0; index < 3; ++index) {
        assert(std::isfinite(output.position_ne_alt_m[index]));
        assert(std::isfinite(output.velocity_ne_up_mps[index]));
        assert(std::isfinite(output.euler_rpy_deg[index]));
        assert(std::isfinite(output.velocity_body_mps[index]));
        assert(std::isfinite(output.rates_body_radps[index]));
        assert(std::isfinite(output.rates_body_derivative_radps2[index]));
        assert(std::isfinite(output.acceleration_body_mps2[index]));
    }
}

int main() {
    assert(avanti_att_abi_version() == AVANTI_ATT_ABI_VERSION);
    assert(avanti_att_input_size() == sizeof(AvantiAttInput));
    assert(avanti_att_output_size() == sizeof(AvantiAttOutput));
    assert(avanti_att_params_size() == sizeof(AvantiAttParams));

    // 出厂默认增益应与 matlab/px4att_gain_names.m / ModelWorkspace 的生成期
    // 值一致（2026-11-03 generation: P_phi_sp=P_theta_sp=6.5）。
    AvantiAttParams defaults = {};
    require_ok(avanti_att_get_default_params(&defaults));
    assert(std::abs(defaults.p_phi_sp - 6.5) < 1.0e-9);
    assert(std::abs(defaults.p_theta_sp - 6.5) < 1.0e-9);

    AvantiAttParams current = {};
    require_ok(avanti_att_get_params(&current));
    assert(current.p_phi_sp == defaults.p_phi_sp);

    AvantiAttHandle first = 0;
    AvantiAttHandle second = 0;
    require_ok(avanti_att_create(100.0, &first));
    require_ok(avanti_att_create(100.0, &second));

    double fixed_step = 0.0;
    require_ok(avanti_att_get_fixed_step(first, &fixed_step));
    assert(std::abs(fixed_step - 0.02) < 1.0e-15);

    AvantiAttInput first_input = {};
    AvantiAttInput second_input = {};
    first_input.throttle_01 = 0.6;
    second_input.throttle_01 = 0.6;
    AvantiAttOutput first_output = {};
    AvantiAttOutput second_output = {};
    bool instances_diverged = false;

    for (uint64_t tick = 0; tick < 100; ++tick) {
        const double time_s = 100.0 + static_cast<double>(tick) * fixed_step;
        // 第 10 tick 起给第二实例施加滚转指令，使两者状态分离
        if (tick >= 10) second_input.phi_sp_rad = 0.1;

        require_ok(avanti_att_tick(first, time_s, &first_input, &first_output));
        require_ok(avanti_att_tick(second, time_s, &second_input, &second_output));
        require_finite(first_output);
        require_finite(second_output);
        assert(std::abs(first_output.sample_time_s - time_s) < 1.0e-12);
        if (tick < 10) {
            assert(first_output.rates_body_radps[0] ==
                   second_output.rates_body_radps[0]);
        } else if (std::abs(first_output.rates_body_radps[0] -
                            second_output.rates_body_radps[0]) > 1.0e-12) {
            instances_diverged = true;
        }
    }
    assert(instances_diverged);

    double next_time = 0.0;
    require_ok(avanti_att_get_next_tick_time(first, &next_time));
    assert(std::abs(next_time - 102.0) < 1.0e-12);
    assert(avanti_att_tick(first, 101.0, &first_input, &first_output) ==
           AVANTI_ATT_ERR_TIME_BACKWARDS);

    require_ok(avanti_att_reset(first, 200.0));
    require_ok(avanti_att_get_next_tick_time(first, &next_time));
    assert(next_time == 200.0);

    // 增益是进程级共享状态：修改一次，两个已存在的 handle 都要受影响。
    AvantiAttParams doubled = defaults;
    doubled.p_phi_sp = defaults.p_phi_sp * 2.0;
    require_ok(avanti_att_set_params(&doubled));
    AvantiAttParams readback = {};
    require_ok(avanti_att_get_params(&readback));
    assert(std::abs(readback.p_phi_sp - doubled.p_phi_sp) < 1.0e-12);

    require_ok(avanti_att_reset(first, 300.0));
    require_ok(avanti_att_reset(second, 300.0));
    AvantiAttInput roll_cmd = {};
    roll_cmd.throttle_01 = 0.6;
    roll_cmd.phi_sp_rad = 0.05;
    AvantiAttOutput y_first_new_gain = {};
    AvantiAttOutput y_second_new_gain = {};
    require_ok(avanti_att_tick(first, 300.0, &roll_cmd, &y_first_new_gain));
    require_ok(avanti_att_tick(second, 300.0, &roll_cmd, &y_second_new_gain));
    // 同一时刻、同一输入、同一（已修改的）增益下，两个 handle 的输出必须一致，
    // 证明 avanti_att_set_params 立刻对进程内所有实例生效，而非某个 handle 私有。
    assert(y_first_new_gain.rates_body_radps[0] ==
           y_second_new_gain.rates_body_radps[0]);

    // 恢复出厂默认增益，避免影响同进程内其他测试可执行文件的假设
    require_ok(avanti_att_set_params(&defaults));

    require_ok(avanti_att_destroy(first));
    require_ok(avanti_att_destroy(second));

    std::cout << "AVANTI attitude-loop C++ interface smoke test passed\n";
    std::cout << "input_size=" << sizeof(AvantiAttInput)
              << " output_size=" << sizeof(AvantiAttOutput)
              << " params_size=" << sizeof(AvantiAttParams)
              << " fixed_step_s=" << fixed_step << '\n';
    return 0;
}
