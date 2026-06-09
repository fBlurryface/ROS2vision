#include "servo_pwm.hpp"

#include <algorithm>
#include <cmath>

#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

namespace learm {

static const char* TAG = "servo_pwm";

static constexpr ledc_mode_t kLedcSpeedMode = LEDC_LOW_SPEED_MODE;
static constexpr ledc_timer_t kLedcTimer = LEDC_TIMER_0;
static constexpr ledc_timer_bit_t kLedcResolution = LEDC_TIMER_16_BIT;
static constexpr uint32_t kLedcMaxDuty = (1UL << 16) - 1;

static constexpr uint32_t kServoUpdatePeriodMs = 20;

static constexpr uint32_t kMinDurationMs = kServoUpdatePeriodMs;
static constexpr uint32_t kMaxDurationMs = 30000;

static constexpr ledc_channel_t kLedcChannels[kServoCount] = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3,
    LEDC_CHANNEL_4,
    LEDC_CHANNEL_5,
};

int ServoPwm::channel_to_index(ServoChannel channel)
{
    const uint8_t index = static_cast<uint8_t>(channel);

    if (index >= kServoCount) {
        return -1;
    }

    return static_cast<int>(index);
}

float ServoPwm::smoothstep(float t)
{
    t = std::clamp<float>(t, 0.0f, 1.0f);

    // Cubic smoothstep:
    // p = 3t^2 - 2t^3
    return t * t * (3.0f - 2.0f * t);
}

float ServoPwm::smootherstep(float t)
{
    t = std::clamp<float>(t, 0.0f, 1.0f);

    // Quintic smootherstep:
    // p = 6t^5 - 15t^4 + 10t^3
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

float ServoPwm::evaluate_profile(MotionProfile profile, float t)
{
    t = std::clamp<float>(t, 0.0f, 1.0f);

    switch (profile) {
        case MotionProfile::Immediate:
            return 1.0f;

        case MotionProfile::Linear:
            return t;

        case MotionProfile::SmoothStep:
            return smoothstep(t);

        case MotionProfile::SmootherStep:
            return smootherstep(t);

        case MotionProfile::DoubleSmootherStep:
            return smootherstep(smootherstep(t));

        default:
            return smootherstep(t);
    }
}

uint32_t ServoPwm::pulse_us_to_duty(uint16_t pulse_us) const
{
    pulse_us = std::clamp<uint16_t>(
        pulse_us,
        kServoMinUs,
        kServoMaxUs
    );

    return static_cast<uint32_t>(
        static_cast<uint64_t>(pulse_us) * kLedcMaxDuty / kServoPeriodUs
    );
}

uint16_t ServoPwm::clamp_command_us(uint8_t index, uint16_t pulse_us) const
{
    if (index >= kServoCount || !states_[index].configured) {
        return pulse_us;
    }

    const ServoPwmConfig& cfg = states_[index].config;

    return std::clamp<uint16_t>(
        pulse_us,
        cfg.min_us,
        cfg.max_us
    );
}

uint16_t ServoPwm::apply_offset_and_clamp(uint8_t index, uint16_t command_us) const
{
    if (index >= kServoCount || !states_[index].configured) {
        return command_us;
    }

    const ServoPwmConfig& cfg = states_[index].config;

    const int output =
        static_cast<int>(command_us) + static_cast<int>(cfg.offset_us);

    return static_cast<uint16_t>(
        std::clamp<int>(
            output,
            cfg.min_us,
            cfg.max_us
        )
    );
}

esp_err_t ServoPwm::configure_channel(uint8_t index, uint16_t output_us)
{
    if (index >= kServoCount || !states_[index].configured) {
        return ESP_ERR_INVALID_ARG;
    }

    const ServoPwmConfig& cfg = states_[index].config;

    output_us = std::clamp<uint16_t>(
        output_us,
        cfg.min_us,
        cfg.max_us
    );

    ledc_channel_config_t channel_config = {};
    channel_config.gpio_num = cfg.gpio;
    channel_config.speed_mode = kLedcSpeedMode;
    channel_config.channel = kLedcChannels[index];
    channel_config.intr_type = LEDC_INTR_DISABLE;
    channel_config.timer_sel = kLedcTimer;
    channel_config.duty = pulse_us_to_duty(output_us);
    channel_config.hpoint = 0;

    esp_err_t ret = ledc_channel_config(&channel_config);
    if (ret != ESP_OK) {
        ESP_LOGE(
            TAG,
            "ledc_channel_config failed index=%u gpio=%d: %s",
            static_cast<unsigned>(index),
            static_cast<int>(cfg.gpio),
            esp_err_to_name(ret)
        );
        return ret;
    }

    return ESP_OK;
}

esp_err_t ServoPwm::write_output_us(uint8_t index, uint16_t output_us)
{
    if (index >= kServoCount || !states_[index].configured) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!states_[index].enabled) {
        return ESP_ERR_INVALID_STATE;
    }

    const ServoPwmConfig& cfg = states_[index].config;

    output_us = std::clamp<uint16_t>(
        output_us,
        cfg.min_us,
        cfg.max_us
    );

    const uint32_t duty = pulse_us_to_duty(output_us);

    esp_err_t ret = ledc_set_duty(
        kLedcSpeedMode,
        kLedcChannels[index],
        duty
    );

    if (ret != ESP_OK) {
        return ret;
    }

    return ledc_update_duty(
        kLedcSpeedMode,
        kLedcChannels[index]
    );
}

esp_err_t ServoPwm::init(const ServoPwmConfig* configs, uint8_t count)
{
    if (initialized_) {
        return ESP_OK;
    }

    if (configs == nullptr || count == 0 || count > kServoCount) {
        return ESP_ERR_INVALID_ARG;
    }

    for (uint8_t i = 0; i < kServoCount; ++i) {
        states_[i] = ServoState{};
    }

    for (uint8_t i = 0; i < count; ++i) {
        ServoPwmConfig cfg = configs[i];

        const int index = channel_to_index(cfg.channel);
        if (index < 0) {
            ESP_LOGE(TAG, "invalid servo channel in config[%u]", static_cast<unsigned>(i));
            return ESP_ERR_INVALID_ARG;
        }

        if (states_[index].configured) {
            ESP_LOGE(TAG, "duplicate servo channel index=%d", index);
            return ESP_ERR_INVALID_ARG;
        }

        cfg.min_us = std::clamp<uint16_t>(cfg.min_us, kServoMinUs, kServoMaxUs);
        cfg.max_us = std::clamp<uint16_t>(cfg.max_us, kServoMinUs, kServoMaxUs);

        if (cfg.min_us > cfg.max_us) {
            ESP_LOGE(TAG, "invalid pulse range index=%d", index);
            return ESP_ERR_INVALID_ARG;
        }

        cfg.reset_us = std::clamp<uint16_t>(
            cfg.reset_us,
            cfg.min_us,
            cfg.max_us
        );

        ServoState& s = states_[index];

        s.configured = true;
        s.enabled = false;
        s.config = cfg;

        s.current_us = cfg.reset_us;
        s.target_us = cfg.reset_us;
        s.output_us = apply_offset_and_clamp(static_cast<uint8_t>(index), cfg.reset_us);

        s.pulse_changed = false;
        s.running = false;
        s.start_us = cfg.reset_us;

        s.duration_ms = 0;
        s.total_steps = 0;
        s.elapsed_steps = 0;

        s.active_options = ServoMotionOptions{};
        s.default_options = ServoMotionOptions{};

        gpio_reset_pin(cfg.gpio);
        gpio_set_direction(cfg.gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(cfg.gpio, 0);

        ESP_LOGI(
            TAG,
            "configured S%u gpio=%d reset=%uus range=[%u,%u] offset=%d disabled",
            static_cast<unsigned>(index),
            static_cast<int>(cfg.gpio),
            cfg.reset_us,
            cfg.min_us,
            cfg.max_us,
            static_cast<int>(cfg.offset_us)
        );
    }

    ledc_timer_config_t timer_config = {};
    timer_config.speed_mode = kLedcSpeedMode;
    timer_config.timer_num = kLedcTimer;
    timer_config.duty_resolution = kLedcResolution;
    timer_config.freq_hz = kServoFrequencyHz;
    timer_config.clk_cfg = LEDC_AUTO_CLK;

    esp_err_t ret = ledc_timer_config(&timer_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    lock_ = xSemaphoreCreateRecursiveMutex();
    if (lock_ == nullptr) {
        ESP_LOGE(TAG, "xSemaphoreCreateRecursiveMutex failed");
        return ESP_ERR_NO_MEM;
    }

    task_stop_requested_ = false;
    initialized_ = true;

    const BaseType_t task_ret = xTaskCreatePinnedToCore(
        &ServoPwm::task_entry,
        "servo_pwm",
        4096,
        this,
        5,
        &task_handle_,
        1
    );

    if (task_ret != pdPASS) {
        initialized_ = false;
        task_handle_ = nullptr;
        vSemaphoreDelete(lock_);
        lock_ = nullptr;
        ESP_LOGE(TAG, "xTaskCreatePinnedToCore failed");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(
        TAG,
        "servo_pwm initialized, update_period=%ums",
        static_cast<unsigned>(kServoUpdatePeriodMs)
    );

    return ESP_OK;
}

esp_err_t ServoPwm::deinit()
{
    if (!initialized_) {
        return ESP_OK;
    }

    task_stop_requested_ = true;

    for (int i = 0; i < 50 && task_handle_ != nullptr; ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (task_handle_ != nullptr) {
        vTaskDelete(task_handle_);
        task_handle_ = nullptr;
    }

    esp_err_t ret = ESP_OK;

    {
        LockGuard guard(this);
        if (!guard.locked()) {
            ret = ESP_ERR_TIMEOUT;
        } else {
            ret = disable_all();
            initialized_ = false;
        }
    }

    if (lock_ != nullptr) {
        vSemaphoreDelete(lock_);
        lock_ = nullptr;
    }

    ESP_LOGI(TAG, "servo_pwm deinitialized");

    return ret;
}

esp_err_t ServoPwm::enable(ServoChannel channel, uint16_t start_us)
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }

    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    const int index = channel_to_index(channel);
    if (index < 0 || !states_[index].configured) {
        return ESP_ERR_INVALID_ARG;
    }

    ServoState& s = states_[index];

    start_us = clamp_command_us(static_cast<uint8_t>(index), start_us);

    s.current_us = start_us;
    s.target_us = start_us;
    s.output_us = apply_offset_and_clamp(static_cast<uint8_t>(index), start_us);

    s.pulse_changed = false;
    s.running = false;
    s.start_us = start_us;

    s.duration_ms = 0;
    s.total_steps = 0;
    s.elapsed_steps = 0;

    esp_err_t ret = configure_channel(
        static_cast<uint8_t>(index),
        s.output_us
    );

    if (ret != ESP_OK) {
        return ret;
    }

    s.enabled = true;

    ESP_LOGW(
        TAG,
        "enabled S%u gpio=%d command=%uus output=%uus",
        static_cast<unsigned>(index),
        static_cast<int>(s.config.gpio),
        s.current_us,
        s.output_us
    );

    return ESP_OK;
}

esp_err_t ServoPwm::disable(ServoChannel channel)
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }

    const int index = channel_to_index(channel);
    if (index < 0 || !states_[index].configured) {
        return ESP_ERR_INVALID_ARG;
    }

    ServoState& s = states_[index];

    s.pulse_changed = false;
    s.running = false;
    s.total_steps = 0;
    s.elapsed_steps = 0;

    if (s.enabled) {
        ledc_stop(kLedcSpeedMode, kLedcChannels[index], 0);

        gpio_reset_pin(s.config.gpio);
        gpio_set_direction(s.config.gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(s.config.gpio, 0);

        ESP_LOGW(
            TAG,
            "disabled S%u gpio=%d",
            static_cast<unsigned>(index),
            static_cast<int>(s.config.gpio)
        );
    }

    s.enabled = false;

    return ESP_OK;
}

esp_err_t ServoPwm::disable_all()
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }

    for (uint8_t i = 0; i < kServoCount; ++i) {
        if (!states_[i].configured) {
            continue;
        }

        esp_err_t ret = disable(static_cast<ServoChannel>(i));
        if (ret != ESP_OK) {
            return ret;
        }
    }

    return ESP_OK;
}

bool ServoPwm::is_enabled(ServoChannel channel) const
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return false;
    }

    const int index = channel_to_index(channel);
    if (index < 0 || !states_[index].configured) {
        return false;
    }

    return states_[index].enabled;
}

esp_err_t ServoPwm::write_us_now(ServoChannel channel, uint16_t pulse_us)
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }

    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    const int index = channel_to_index(channel);
    if (index < 0 || !states_[index].configured) {
        return ESP_ERR_INVALID_ARG;
    }

    ServoState& s = states_[index];

    if (!s.enabled) {
        ESP_LOGE(TAG, "write_us_now rejected: S%u not enabled", static_cast<unsigned>(index));
        return ESP_ERR_INVALID_STATE;
    }

    pulse_us = clamp_command_us(static_cast<uint8_t>(index), pulse_us);

    s.current_us = pulse_us;
    s.target_us = pulse_us;
    s.output_us = apply_offset_and_clamp(static_cast<uint8_t>(index), pulse_us);

    s.pulse_changed = false;
    s.running = false;
    s.start_us = pulse_us;
    s.duration_ms = 0;
    s.total_steps = 0;
    s.elapsed_steps = 0;

    return write_output_us(static_cast<uint8_t>(index), s.output_us);
}

esp_err_t ServoPwm::move_us(
    ServoChannel channel,
    uint16_t target_us,
    uint32_t duration_ms
)
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }

    const int index = channel_to_index(channel);
    if (index < 0 || !states_[index].configured) {
        return ESP_ERR_INVALID_ARG;
    }

    ServoMotionOptions options = states_[index].default_options;
    options.duration_ms = duration_ms;

    return move_us(channel, target_us, options);
}

esp_err_t ServoPwm::move_us(
    ServoChannel channel,
    uint16_t target_us,
    const ServoMotionOptions& options
)
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }

    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    const int index = channel_to_index(channel);
    if (index < 0 || !states_[index].configured) {
        return ESP_ERR_INVALID_ARG;
    }

    ServoState& s = states_[index];

    if (!s.enabled) {
        ESP_LOGE(TAG, "move_us rejected: S%u not enabled", static_cast<unsigned>(index));
        return ESP_ERR_INVALID_STATE;
    }

    target_us = clamp_command_us(static_cast<uint8_t>(index), target_us);

    ServoMotionOptions normalized = options;

    normalized.duration_ms = std::clamp<uint32_t>(
        normalized.duration_ms,
        kMinDurationMs,
        kMaxDurationMs
    );

    if (normalized.profile == MotionProfile::Immediate) {
        return write_us_now(channel, target_us);
    }

    s.target_us = target_us;
    s.active_options = normalized;
    s.duration_ms = normalized.duration_ms;
    s.pulse_changed = true;

    ESP_LOGI(
        TAG,
        "move S%u target=%uus duration=%ums profile=%u",
        static_cast<unsigned>(index),
        target_us,
        static_cast<unsigned>(normalized.duration_ms),
        static_cast<unsigned>(normalized.profile)
    );

    return ESP_OK;
}

esp_err_t ServoPwm::reset(ServoChannel channel, uint32_t duration_ms)
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }

    const int index = channel_to_index(channel);
    if (index < 0 || !states_[index].configured) {
        return ESP_ERR_INVALID_ARG;
    }

    return move_us(
        channel,
        states_[index].config.reset_us,
        duration_ms
    );
}

esp_err_t ServoPwm::reset_all(uint32_t duration_ms)
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }

    for (uint8_t i = 0; i < kServoCount; ++i) {
        if (!states_[i].configured || !states_[i].enabled) {
            continue;
        }

        esp_err_t ret = reset(static_cast<ServoChannel>(i), duration_ms);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    return ESP_OK;
}

esp_err_t ServoPwm::stop(ServoChannel channel)
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }

    const int index = channel_to_index(channel);
    if (index < 0 || !states_[index].configured) {
        return ESP_ERR_INVALID_ARG;
    }

    ServoState& s = states_[index];

    s.target_us = s.current_us;
    s.pulse_changed = false;
    s.running = false;
    s.total_steps = 0;
    s.elapsed_steps = 0;

    return ESP_OK;
}

esp_err_t ServoPwm::stop_all()
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }

    for (uint8_t i = 0; i < kServoCount; ++i) {
        if (!states_[i].configured) {
            continue;
        }

        esp_err_t ret = stop(static_cast<ServoChannel>(i));
        if (ret != ESP_OK) {
            return ret;
        }
    }

    return ESP_OK;
}

esp_err_t ServoPwm::set_config(ServoChannel channel, const ServoPwmConfig& config)
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }

    const int index = channel_to_index(channel);
    if (index < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (states_[index].enabled) {
        ESP_LOGE(TAG, "set_config rejected: S%u is enabled", static_cast<unsigned>(index));
        return ESP_ERR_INVALID_STATE;
    }

    ServoPwmConfig cfg = config;
    cfg.channel = channel;

    cfg.min_us = std::clamp<uint16_t>(cfg.min_us, kServoMinUs, kServoMaxUs);
    cfg.max_us = std::clamp<uint16_t>(cfg.max_us, kServoMinUs, kServoMaxUs);

    if (cfg.min_us > cfg.max_us) {
        return ESP_ERR_INVALID_ARG;
    }

    cfg.reset_us = std::clamp<uint16_t>(
        cfg.reset_us,
        cfg.min_us,
        cfg.max_us
    );

    ServoState& s = states_[index];

    s.configured = true;
    s.config = cfg;

    s.current_us = cfg.reset_us;
    s.target_us = cfg.reset_us;
    s.output_us = apply_offset_and_clamp(static_cast<uint8_t>(index), cfg.reset_us);

    s.pulse_changed = false;
    s.running = false;
    s.start_us = cfg.reset_us;
    s.duration_ms = 0;
    s.total_steps = 0;
    s.elapsed_steps = 0;

    gpio_reset_pin(cfg.gpio);
    gpio_set_direction(cfg.gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(cfg.gpio, 0);

    return ESP_OK;
}

ServoPwmConfig ServoPwm::get_config(ServoChannel channel) const
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return {};
    }

    const int index = channel_to_index(channel);
    if (index < 0 || !states_[index].configured) {
        return {};
    }

    return states_[index].config;
}

esp_err_t ServoPwm::set_default_motion_options(
    ServoChannel channel,
    const ServoMotionOptions& options
)
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }

    const int index = channel_to_index(channel);
    if (index < 0 || !states_[index].configured) {
        return ESP_ERR_INVALID_ARG;
    }

    states_[index].default_options = options;

    return ESP_OK;
}

ServoMotionOptions ServoPwm::get_default_motion_options(ServoChannel channel) const
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return {};
    }

    const int index = channel_to_index(channel);
    if (index < 0 || !states_[index].configured) {
        return {};
    }

    return states_[index].default_options;
}

esp_err_t ServoPwm::set_offset(ServoChannel channel, int16_t offset_us)
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }

    const int index = channel_to_index(channel);
    if (index < 0 || !states_[index].configured) {
        return ESP_ERR_INVALID_ARG;
    }

    ServoState& s = states_[index];

    s.config.offset_us = offset_us;
    s.output_us = apply_offset_and_clamp(
        static_cast<uint8_t>(index),
        s.current_us
    );

    if (!s.enabled) {
        return ESP_OK;
    }

    return write_output_us(static_cast<uint8_t>(index), s.output_us);
}

int16_t ServoPwm::get_offset(ServoChannel channel) const
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return 0;
    }

    const int index = channel_to_index(channel);
    if (index < 0 || !states_[index].configured) {
        return 0;
    }

    return states_[index].config.offset_us;
}

uint16_t ServoPwm::read_command_us(ServoChannel channel) const
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return 0;
    }

    const int index = channel_to_index(channel);
    if (index < 0 || !states_[index].configured) {
        return 0;
    }

    return states_[index].current_us;
}

uint16_t ServoPwm::read_target_us(ServoChannel channel) const
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return 0;
    }

    const int index = channel_to_index(channel);
    if (index < 0 || !states_[index].configured) {
        return 0;
    }

    return states_[index].target_us;
}

uint16_t ServoPwm::read_output_us(ServoChannel channel) const
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return 0;
    }

    const int index = channel_to_index(channel);
    if (index < 0 || !states_[index].configured) {
        return 0;
    }

    return states_[index].output_us;
}

ServoRuntimeState ServoPwm::get_state(ServoChannel channel) const
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return {};
    }

    const int index = channel_to_index(channel);
    if (index < 0 || !states_[index].configured) {
        return {};
    }

    const ServoState& s = states_[index];

    ServoRuntimeState state = {};

    state.configured = s.configured;
    state.enabled = s.enabled;
    state.running = s.running;
    state.ready = is_ready(channel);

    state.current_us = s.current_us;
    state.target_us = s.target_us;
    state.output_us = s.output_us;

    state.reset_us = s.config.reset_us;
    state.min_us = s.config.min_us;
    state.max_us = s.config.max_us;
    state.offset_us = s.config.offset_us;

    state.duration_ms = s.duration_ms;
    state.total_steps = s.total_steps;
    state.elapsed_steps = s.elapsed_steps;

    state.profile = s.active_options.profile;

    state.max_step_us = s.active_options.max_step_us;
    state.min_effective_step_us = s.active_options.min_effective_step_us;

    return state;
}

bool ServoPwm::is_ready(ServoChannel channel) const
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return false;
    }

    const int index = channel_to_index(channel);
    if (index < 0 || !states_[index].configured) {
        return false;
    }

    const ServoState& s = states_[index];

    if (!s.enabled) {
        return true;
    }

    return !s.running &&
           !s.pulse_changed &&
           s.current_us == s.target_us;
}

bool ServoPwm::is_all_ready() const
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return false;
    }

    for (uint8_t i = 0; i < kServoCount; ++i) {
        const ServoState& s = states_[i];

        if (!s.configured || !s.enabled) {
            continue;
        }

        if (s.running || s.pulse_changed || s.current_us != s.target_us) {
            return false;
        }
    }

    return true;
}


bool ServoPwm::take_lock(TickType_t timeout_ticks) const
{
    if (lock_ == nullptr) {
        return true;
    }

    return xSemaphoreTakeRecursive(lock_, timeout_ticks) == pdTRUE;
}

void ServoPwm::give_lock() const
{
    if (lock_ != nullptr) {
        xSemaphoreGiveRecursive(lock_);
    }
}

ServoPwm::LockGuard::LockGuard(
    const ServoPwm* owner,
    TickType_t timeout_ticks
) : owner_(owner)
{
    locked_ = (owner_ != nullptr) && owner_->take_lock(timeout_ticks);
}

ServoPwm::LockGuard::~LockGuard()
{
    if (locked_ && owner_ != nullptr) {
        owner_->give_lock();
    }
}

bool ServoPwm::LockGuard::locked() const
{
    return locked_;
}

void ServoPwm::task_entry(void* arg)
{
    auto* self = static_cast<ServoPwm*>(arg);

    if (self != nullptr) {
        self->task_loop();
    }

    vTaskDelete(nullptr);
}

void ServoPwm::task_loop()
{
    const TickType_t period_ticks = pdMS_TO_TICKS(kServoUpdatePeriodMs);
    TickType_t last_wake = xTaskGetTickCount();

    while (!task_stop_requested_) {
        {
            LockGuard guard(this, pdMS_TO_TICKS(5));
            if (guard.locked()) {
                update_all();
            }
        }

        const TickType_t now = xTaskGetTickCount();
        if ((now - last_wake) >= period_ticks) {
            last_wake = now;
            vTaskDelay(1);
        } else {
            vTaskDelayUntil(&last_wake, period_ticks);
        }
    }

    task_handle_ = nullptr;
}

void ServoPwm::update_all()
{
    for (uint8_t i = 0; i < kServoCount; ++i) {
        update_one(i);
    }
}

void ServoPwm::update_one(uint8_t index)
{
    if (index >= kServoCount) {
        return;
    }

    ServoState& s = states_[index];

    if (!s.configured || !s.enabled) {
        return;
    }

    if (s.pulse_changed) {
        s.pulse_changed = false;

        if (s.active_options.replan_from_current || !s.running) {
            s.start_us = s.current_us;
            s.elapsed_steps = 0;
        }

        s.total_steps = std::max<uint32_t>(
            1,
            (s.duration_ms + kServoUpdatePeriodMs - 1) / kServoUpdatePeriodMs
        );

        s.running = (s.start_us != s.target_us);

        if (!s.running) {
            return;
        }
    }

    if (!s.running) {
        return;
    }

    const uint16_t previous_us = s.current_us;

    s.elapsed_steps++;

    if (s.elapsed_steps >= s.total_steps) {
        s.current_us = s.target_us;
        s.running = false;
    } else {
        const float t =
            static_cast<float>(s.elapsed_steps) /
            static_cast<float>(s.total_steps);

        const float shaped = evaluate_profile(
            s.active_options.profile,
            t
        );

        const float delta =
            static_cast<float>(
                static_cast<int>(s.target_us) -
                static_cast<int>(s.start_us)
            );

        const float interpolated =
            static_cast<float>(s.start_us) + delta * shaped;

        int proposed = static_cast<int>(std::lround(interpolated));

        const int remaining =
            static_cast<int>(s.target_us) - static_cast<int>(previous_us);

        int step =
            proposed - static_cast<int>(previous_us);

        if (s.active_options.max_step_us > 0) {
            const int max_step = static_cast<int>(s.active_options.max_step_us);

            if (step > max_step) {
                step = max_step;
            } else if (step < -max_step) {
                step = -max_step;
            }

            proposed = static_cast<int>(previous_us) + step;
        }

        if (s.active_options.min_effective_step_us > 0 && remaining != 0) {
            const int min_step = static_cast<int>(s.active_options.min_effective_step_us);
            step = proposed - static_cast<int>(previous_us);

            if (std::abs(step) < min_step) {
                const int direction = (remaining > 0) ? 1 : -1;

                proposed =
                    static_cast<int>(previous_us) +
                    direction * min_step;

                if (direction > 0) {
                    proposed = std::min<int>(
                        proposed,
                        static_cast<int>(s.target_us)
                    );
                } else {
                    proposed = std::max<int>(
                        proposed,
                        static_cast<int>(s.target_us)
                    );
                }
            }
        }

        proposed = std::clamp<int>(
            proposed,
            s.config.min_us,
            s.config.max_us
        );

        s.current_us = static_cast<uint16_t>(proposed);
    }

    const uint16_t next_output_us =
        apply_offset_and_clamp(index, s.current_us);

    if (next_output_us != s.output_us || !s.running) {
        s.output_us = next_output_us;
        write_output_us(index, s.output_us);
    }
}

}  // namespace learm