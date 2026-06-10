#pragma once

#include <stdint.h>

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace learm {

enum class ServoChannel : uint8_t {
    S0 = 0,
    S1,
    S2,
    S3,
    S4,
    S5,
};

inline constexpr uint8_t kServoCount = 6;

inline constexpr uint32_t kServoFrequencyHz = 50;
inline constexpr uint16_t kServoPeriodUs = 20000;

inline constexpr uint16_t kServoMinUs = 500;
inline constexpr uint16_t kServoMaxUs = 2500;

struct ServoPwmConfig {
    ServoChannel channel;
    gpio_num_t gpio;

    uint16_t reset_us;
    uint16_t min_us;
    uint16_t max_us;

    int16_t offset_us;
};

// V5A: MotionProfile 继续放在公共头文件里，供 arm_motion 使用。
// servo_pwm 本身不再执行轨迹插补。
enum class MotionProfile : uint8_t {
    Immediate = 0,
    Linear,
    SmoothStep,
    SmootherStep,
    DoubleSmootherStep,
};

// 仅保留为兼容公共类型；V5A 中 servo_pwm 不消费该结构。
struct ServoMotionOptions {
    uint32_t duration_ms = 700;
    MotionProfile profile = MotionProfile::SmootherStep;
    bool replan_from_current = true;
    uint16_t max_step_us = 0;
    uint16_t min_effective_step_us = 0;
};

struct ServoRuntimeState {
    bool configured;
    bool enabled;
    bool running;
    bool ready;

    uint16_t current_us;
    uint16_t target_us;
    uint16_t output_us;

    uint16_t reset_us;
    uint16_t min_us;
    uint16_t max_us;
    int16_t offset_us;

    uint32_t duration_ms;
    uint32_t total_steps;
    uint32_t elapsed_steps;

    MotionProfile profile;

    uint16_t max_step_us;
    uint16_t min_effective_step_us;
};

class ServoPwm {
public:
    esp_err_t init(const ServoPwmConfig* configs, uint8_t count);
    esp_err_t deinit();

    esp_err_t enable(ServoChannel channel, uint16_t start_us);
    esp_err_t disable(ServoChannel channel);
    esp_err_t disable_all();
    bool is_enabled(ServoChannel channel) const;

    // V5A: 纯底层接口。只负责单通道立即输出 PWM。
    // 不保存 duration/running/elapsed_steps，不执行插补。
    esp_err_t write_us_now(ServoChannel channel, uint16_t pulse_us);

    esp_err_t set_config(ServoChannel channel, const ServoPwmConfig& config);
    ServoPwmConfig get_config(ServoChannel channel) const;

    esp_err_t set_offset(ServoChannel channel, int16_t offset_us);
    int16_t get_offset(ServoChannel channel) const;

    uint16_t read_command_us(ServoChannel channel) const;
    uint16_t read_target_us(ServoChannel channel) const;
    uint16_t read_output_us(ServoChannel channel) const;

    ServoRuntimeState get_state(ServoChannel channel) const;

    bool is_ready(ServoChannel channel) const;
    bool is_all_ready() const;

private:
    struct ServoState {
        bool configured = false;
        bool enabled = false;

        ServoPwmConfig config = {};

        // current_us / target_us 是软件命令脉宽，不含 offset。
        // V5A 中二者始终相同；轨迹 target 由 arm_motion 保存。
        uint16_t current_us = 1500;
        uint16_t target_us = 1500;

        // output_us 是实际写入 PWM 后端的脉宽，包含 offset 后再限幅。
        uint16_t output_us = 1500;
    };

private:
    class LockGuard {
    public:
        explicit LockGuard(const ServoPwm* owner, TickType_t timeout_ticks = portMAX_DELAY);
        ~LockGuard();

        bool locked() const;

    private:
        const ServoPwm* owner_ = nullptr;
        bool locked_ = false;
    };

    bool take_lock(TickType_t timeout_ticks = portMAX_DELAY) const;
    void give_lock() const;

    esp_err_t setup_pwm_backend();
    void teardown_pwm_backend();
    esp_err_t create_channel_resources(uint8_t index);
    void destroy_channel_resources(uint8_t index);
    esp_err_t configure_channel(uint8_t index, uint16_t output_us);
    esp_err_t write_output_us(uint8_t index, uint16_t output_us);

    uint16_t clamp_command_us(uint8_t index, uint16_t pulse_us) const;
    uint16_t apply_offset_and_clamp(uint8_t index, uint16_t command_us) const;

    static int channel_to_index(ServoChannel channel);

private:
    bool initialized_ = false;
    SemaphoreHandle_t lock_ = nullptr;

    ServoState states_[kServoCount] = {};

    // ESP32 MCPWM: one group has 3 operators, each operator provides two
    // generator/comparator pairs. This maps exactly to the six servo outputs.
    static constexpr uint8_t kMcpwmOperatorCount = 3;
    mcpwm_timer_handle_t mcpwm_timer_ = nullptr;
    mcpwm_oper_handle_t mcpwm_operators_[kMcpwmOperatorCount] = {};
    mcpwm_cmpr_handle_t mcpwm_comparators_[kServoCount] = {};
    mcpwm_gen_handle_t mcpwm_generators_[kServoCount] = {};
};

}  // namespace learm
