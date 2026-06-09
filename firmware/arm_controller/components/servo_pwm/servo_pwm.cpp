#include "servo_pwm.hpp"

#include <algorithm>

#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace learm {

static const char* TAG = "servo_pwm";

static constexpr ledc_mode_t kLedcSpeedMode = LEDC_LOW_SPEED_MODE;
static constexpr ledc_timer_t kLedcTimer = LEDC_TIMER_0;
static constexpr ledc_timer_bit_t kLedcResolution = LEDC_TIMER_16_BIT;
static constexpr uint32_t kLedcMaxDuty = (1UL << 16) - 1;

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

    for (uint8_t i = 0; i < count; ++i) {
        ServoPwmConfig cfg = configs[i];

        const int index = channel_to_index(cfg.channel);
        if (index < 0) {
            ESP_LOGE(TAG, "invalid servo channel in config[%u]", static_cast<unsigned>(i));
            vSemaphoreDelete(lock_);
            lock_ = nullptr;
            return ESP_ERR_INVALID_ARG;
        }

        if (states_[index].configured) {
            ESP_LOGE(TAG, "duplicate servo channel index=%d", index);
            vSemaphoreDelete(lock_);
            lock_ = nullptr;
            return ESP_ERR_INVALID_ARG;
        }

        cfg.min_us = std::clamp<uint16_t>(cfg.min_us, kServoMinUs, kServoMaxUs);
        cfg.max_us = std::clamp<uint16_t>(cfg.max_us, kServoMinUs, kServoMaxUs);

        if (cfg.min_us > cfg.max_us) {
            ESP_LOGE(TAG, "invalid pulse range index=%d", index);
            vSemaphoreDelete(lock_);
            lock_ = nullptr;
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

    initialized_ = true;

    ESP_LOGI(TAG, "servo_pwm initialized as pure PWM output layer");

    return ESP_OK;
}

esp_err_t ServoPwm::deinit()
{
    if (!initialized_) {
        return ESP_OK;
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

    return write_output_us(static_cast<uint8_t>(index), s.output_us);
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
    state.running = false;
    state.ready = true;

    state.current_us = s.current_us;
    state.target_us = s.target_us;
    state.output_us = s.output_us;

    state.reset_us = s.config.reset_us;
    state.min_us = s.config.min_us;
    state.max_us = s.config.max_us;
    state.offset_us = s.config.offset_us;

    state.duration_ms = 0;
    state.total_steps = 0;
    state.elapsed_steps = 0;

    state.profile = MotionProfile::Immediate;

    state.max_step_us = 0;
    state.min_effective_step_us = 0;

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

    return true;
}

bool ServoPwm::is_all_ready() const
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return false;
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

}  // namespace learm
