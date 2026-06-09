#pragma once

#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

enum class MotionProfile : uint8_t {
    Immediate = 0,
    Linear,
    SmoothStep,
    SmootherStep,
    DoubleSmootherStep,
};

struct ServoMotionOptions {
    uint32_t duration_ms = 700;
    MotionProfile profile = MotionProfile::SmootherStep;

    // 运动中途收到新目标时，是否从当前 command_us 重新规划。
    bool replan_from_current = true;

    // 每个 20ms 更新周期允许的最大脉宽变化量。
    // 0 表示不限制。
    uint16_t max_step_us = 0;

    // 最小有效步进。
    // 0 表示不启用。
    // 这个参数以后可用于规避某些舵机慢速下 1us/2us 死区抖动。
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

    // 立即写入脉宽，不做插补。
    // 仍然经过 min_us / max_us / offset_us 安全处理。
    esp_err_t write_us_now(ServoChannel channel, uint16_t pulse_us);

    // 简化接口：只指定目标脉宽和总时长。
    esp_err_t move_us(
        ServoChannel channel,
        uint16_t target_us,
        uint32_t duration_ms
    );

    // 完整接口：指定运动曲线和运动参数。
    esp_err_t move_us(
        ServoChannel channel,
        uint16_t target_us,
        const ServoMotionOptions& options
    );

    esp_err_t reset(ServoChannel channel, uint32_t duration_ms);
    esp_err_t reset_all(uint32_t duration_ms);

    esp_err_t stop(ServoChannel channel);
    esp_err_t stop_all();

    esp_err_t set_config(ServoChannel channel, const ServoPwmConfig& config);
    ServoPwmConfig get_config(ServoChannel channel) const;

    esp_err_t set_default_motion_options(
        ServoChannel channel,
        const ServoMotionOptions& options
    );

    ServoMotionOptions get_default_motion_options(ServoChannel channel) const;

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

        // current_us 是当前软件命令脉宽，不含 offset。
        uint16_t current_us = 1500;

        // target_us 是目标命令脉宽，不含 offset。
        uint16_t target_us = 1500;

        // output_us 是真正写到 LEDC 的脉宽，包含 offset 后再限幅。
        uint16_t output_us = 1500;

        bool pulse_changed = false;
        bool running = false;

        uint16_t start_us = 1500;

        uint32_t duration_ms = 0;
        uint32_t total_steps = 0;
        uint32_t elapsed_steps = 0;

        ServoMotionOptions active_options = {};
        ServoMotionOptions default_options = {};
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

    static void task_entry(void* arg);

    bool take_lock(TickType_t timeout_ticks = portMAX_DELAY) const;
    void give_lock() const;

    void task_loop();
    void update_all();
    void update_one(uint8_t index);

    esp_err_t configure_channel(uint8_t index, uint16_t output_us);
    esp_err_t write_output_us(uint8_t index, uint16_t output_us);

    uint16_t clamp_command_us(uint8_t index, uint16_t pulse_us) const;
    uint16_t apply_offset_and_clamp(uint8_t index, uint16_t command_us) const;

    static int channel_to_index(ServoChannel channel);

    static float evaluate_profile(MotionProfile profile, float t);
    static float smoothstep(float t);
    static float smootherstep(float t);

    uint32_t pulse_us_to_duty(uint16_t pulse_us) const;

private:
    bool initialized_ = false;
    TaskHandle_t task_handle_ = nullptr;
    volatile bool task_stop_requested_ = false;
    SemaphoreHandle_t lock_ = nullptr;

    ServoState states_[kServoCount] = {};
};

}  // namespace learm