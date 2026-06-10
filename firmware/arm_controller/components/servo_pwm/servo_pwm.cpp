#include "servo_pwm.hpp"

#include <algorithm>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace learm {

static const char* TAG = "servo_pwm";

static constexpr uint32_t kMcpwmResolutionHz = 1000000;  // 1 tick = 1us.
static constexpr int kMcpwmGroupId = 0;

static uint8_t channel_to_operator_index(uint8_t index)
{
    return static_cast<uint8_t>(index / 2);
}

int ServoPwm::channel_to_index(ServoChannel channel)
{
    const uint8_t index = static_cast<uint8_t>(channel);

    if (index >= kServoCount) {
        return -1;
    }

    return static_cast<int>(index);
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

esp_err_t ServoPwm::setup_pwm_backend()
{
    if (mcpwm_timer_ != nullptr) {
        return ESP_OK;
    }

    mcpwm_timer_config_t timer_config = {};
    timer_config.group_id = kMcpwmGroupId;
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = kMcpwmResolutionHz;
    timer_config.period_ticks = kServoPeriodUs;
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;

    esp_err_t ret = mcpwm_new_timer(&timer_config, &mcpwm_timer_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_new_timer failed: %s", esp_err_to_name(ret));
        return ret;
    }

    for (uint8_t i = 0; i < kMcpwmOperatorCount; ++i) {
        mcpwm_operator_config_t operator_config = {};
        operator_config.group_id = kMcpwmGroupId;

        ret = mcpwm_new_operator(&operator_config, &mcpwm_operators_[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(
                TAG,
                "mcpwm_new_operator failed op=%u: %s",
                static_cast<unsigned>(i),
                esp_err_to_name(ret)
            );
            teardown_pwm_backend();
            return ret;
        }

        ret = mcpwm_operator_connect_timer(mcpwm_operators_[i], mcpwm_timer_);
        if (ret != ESP_OK) {
            ESP_LOGE(
                TAG,
                "mcpwm_operator_connect_timer failed op=%u: %s",
                static_cast<unsigned>(i),
                esp_err_to_name(ret)
            );
            teardown_pwm_backend();
            return ret;
        }
    }

    // Do not create generator/comparator resources for disabled channels here.
    // A configured-but-disabled servo must not emit any PWM pulse.  Channel
    // resources are created lazily in enable() and destroyed again in disable().

    ret = mcpwm_timer_enable(mcpwm_timer_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_timer_enable failed: %s", esp_err_to_name(ret));
        teardown_pwm_backend();
        return ret;
    }

    ret = mcpwm_timer_start_stop(mcpwm_timer_, MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcpwm_timer_start_stop failed: %s", esp_err_to_name(ret));
        teardown_pwm_backend();
        return ret;
    }

    ESP_LOGI(TAG, "mcpwm servo backend started: group=%d freq=%uHz period=%uus resolution=%uHz",
             kMcpwmGroupId, kServoFrequencyHz, kServoPeriodUs, kMcpwmResolutionHz);

    return ESP_OK;
}

void ServoPwm::teardown_pwm_backend()
{
    if (mcpwm_timer_ != nullptr) {
        // Best effort: stop the timer before deleting downstream resources.
        (void)mcpwm_timer_start_stop(mcpwm_timer_, MCPWM_TIMER_STOP_EMPTY);
    }

    for (uint8_t i = 0; i < kServoCount; ++i) {
        destroy_channel_resources(i);
    }

    for (uint8_t i = 0; i < kMcpwmOperatorCount; ++i) {
        if (mcpwm_operators_[i] != nullptr) {
            (void)mcpwm_del_operator(mcpwm_operators_[i]);
            mcpwm_operators_[i] = nullptr;
        }
    }

    if (mcpwm_timer_ != nullptr) {
        (void)mcpwm_timer_disable(mcpwm_timer_);
        (void)mcpwm_del_timer(mcpwm_timer_);
        mcpwm_timer_ = nullptr;
    }
}

esp_err_t ServoPwm::create_channel_resources(uint8_t index)
{
    if (index >= kServoCount || !states_[index].configured) {
        return ESP_ERR_INVALID_ARG;
    }

    if (mcpwm_comparators_[index] != nullptr || mcpwm_generators_[index] != nullptr) {
        return ESP_OK;
    }

    const uint8_t op_index = channel_to_operator_index(index);
    if (op_index >= kMcpwmOperatorCount || mcpwm_operators_[op_index] == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    const ServoPwmConfig& cfg = states_[index].config;

    mcpwm_comparator_config_t comparator_config = {};
    comparator_config.flags.update_cmp_on_tez = true;

    esp_err_t ret = mcpwm_new_comparator(
        mcpwm_operators_[op_index],
        &comparator_config,
        &mcpwm_comparators_[index]
    );
    if (ret != ESP_OK) {
        ESP_LOGE(
            TAG,
            "mcpwm_new_comparator failed S%u: %s",
            static_cast<unsigned>(index),
            esp_err_to_name(ret)
        );
        return ret;
    }

    mcpwm_generator_config_t generator_config = {};
    generator_config.gen_gpio_num = cfg.gpio;

    ret = mcpwm_new_generator(
        mcpwm_operators_[op_index],
        &generator_config,
        &mcpwm_generators_[index]
    );
    if (ret != ESP_OK) {
        ESP_LOGE(
            TAG,
            "mcpwm_new_generator failed S%u gpio=%d: %s",
            static_cast<unsigned>(index),
            static_cast<int>(cfg.gpio),
            esp_err_to_name(ret)
        );
        destroy_channel_resources(index);
        return ret;
    }

    ret = mcpwm_comparator_set_compare_value(
        mcpwm_comparators_[index],
        states_[index].output_us
    );
    if (ret != ESP_OK) {
        ESP_LOGE(
            TAG,
            "mcpwm_comparator_set_compare_value failed S%u: %s",
            static_cast<unsigned>(index),
            esp_err_to_name(ret)
        );
        destroy_channel_resources(index);
        return ret;
    }

    ret = mcpwm_generator_set_action_on_timer_event(
        mcpwm_generators_[index],
        MCPWM_GEN_TIMER_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            MCPWM_TIMER_EVENT_EMPTY,
            MCPWM_GEN_ACTION_HIGH
        )
    );
    if (ret != ESP_OK) {
        ESP_LOGE(
            TAG,
            "mcpwm_generator_set_action_on_timer_event failed S%u: %s",
            static_cast<unsigned>(index),
            esp_err_to_name(ret)
        );
        destroy_channel_resources(index);
        return ret;
    }

    ret = mcpwm_generator_set_action_on_compare_event(
        mcpwm_generators_[index],
        MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            mcpwm_comparators_[index],
            MCPWM_GEN_ACTION_LOW
        )
    );
    if (ret != ESP_OK) {
        ESP_LOGE(
            TAG,
            "mcpwm_generator_set_action_on_compare_event failed S%u: %s",
            static_cast<unsigned>(index),
            esp_err_to_name(ret)
        );
        destroy_channel_resources(index);
        return ret;
    }

    // Keep the generator under normal timer/compare event control.
    // Do not hold it force-low during init: on ESP32-WROOM this prevented
    // enabled channels from producing usable servo pulses.
    ESP_LOGI(
        TAG,
        "mcpwm channel S%u gpio=%d op=%u compare=%uus ready",
        static_cast<unsigned>(index),
        static_cast<int>(cfg.gpio),
        static_cast<unsigned>(op_index),
        states_[index].output_us
    );

    return ESP_OK;
}

void ServoPwm::destroy_channel_resources(uint8_t index)
{
    if (index >= kServoCount) {
        return;
    }

    if (mcpwm_generators_[index] != nullptr) {
        (void)mcpwm_generator_set_force_level(mcpwm_generators_[index], 0, true);
        (void)mcpwm_del_generator(mcpwm_generators_[index]);
        mcpwm_generators_[index] = nullptr;
    }

    if (mcpwm_comparators_[index] != nullptr) {
        (void)mcpwm_del_comparator(mcpwm_comparators_[index]);
        mcpwm_comparators_[index] = nullptr;
    }
}

esp_err_t ServoPwm::configure_channel(uint8_t index, uint16_t output_us)
{
    if (index >= kServoCount || !states_[index].configured) {
        return ESP_ERR_INVALID_ARG;
    }

    if (mcpwm_comparators_[index] == nullptr || mcpwm_generators_[index] == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    const ServoPwmConfig& cfg = states_[index].config;

    output_us = std::clamp<uint16_t>(
        output_us,
        cfg.min_us,
        cfg.max_us
    );

    return mcpwm_comparator_set_compare_value(
        mcpwm_comparators_[index],
        output_us
    );
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

    return mcpwm_comparator_set_compare_value(
        mcpwm_comparators_[index],
        output_us
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

    esp_err_t ret = setup_pwm_backend();
    if (ret != ESP_OK) {
        vSemaphoreDelete(lock_);
        lock_ = nullptr;
        return ret;
    }

    initialized_ = true;

    ESP_LOGI(TAG, "servo_pwm initialized with MCPWM backend, 50Hz, 1us resolution");

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
            teardown_pwm_backend();
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

    esp_err_t ret = create_channel_resources(static_cast<uint8_t>(index));
    if (ret != ESP_OK) {
        return ret;
    }

    ret = configure_channel(
        static_cast<uint8_t>(index),
        s.output_us
    );

    if (ret != ESP_OK) {
        destroy_channel_resources(static_cast<uint8_t>(index));
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

    const bool had_pwm_resource =
        (mcpwm_generators_[index] != nullptr) ||
        (mcpwm_comparators_[index] != nullptr);

    // Disable must mean "no valid RC-servo pulse on the signal wire".
    // Never encode disable as an ultra-short pulse: some servos interpret that
    // as an extreme position command and can slam into mechanical limits.
    destroy_channel_resources(static_cast<uint8_t>(index));

    const gpio_num_t gpio = s.config.gpio;
    esp_err_t ret = gpio_reset_pin(gpio);
    if (ret != ESP_OK) {
        ESP_LOGE(
            TAG,
            "gpio_reset_pin failed while disabling S%u gpio=%d: %s",
            static_cast<unsigned>(index),
            static_cast<int>(gpio),
            esp_err_to_name(ret)
        );
        return ret;
    }

    // Preload the output latch low before switching the pin back to GPIO
    // output mode, so the signal wire does not briefly produce a high level.
    ret = gpio_set_level(gpio, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(
            TAG,
            "gpio_set_level failed while disabling S%u gpio=%d: %s",
            static_cast<unsigned>(index),
            static_cast<int>(gpio),
            esp_err_to_name(ret)
        );
        return ret;
    }

    ret = gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        ESP_LOGE(
            TAG,
            "gpio_set_direction failed while disabling S%u gpio=%d: %s",
            static_cast<unsigned>(index),
            static_cast<int>(gpio),
            esp_err_to_name(ret)
        );
        return ret;
    }

    s.enabled = false;

    if (had_pwm_resource) {
        ESP_LOGW(
            TAG,
            "disabled S%u gpio=%d pwm=off gpio=low",
            static_cast<unsigned>(index),
            static_cast<int>(gpio)
        );
    }

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

    const bool need_recreate = initialized_ &&
        (mcpwm_generators_[index] != nullptr || mcpwm_comparators_[index] != nullptr);

    if (need_recreate) {
        destroy_channel_resources(static_cast<uint8_t>(index));
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
    s.enabled = false;
    s.config = cfg;

    s.current_us = cfg.reset_us;
    s.target_us = cfg.reset_us;
    s.output_us = apply_offset_and_clamp(static_cast<uint8_t>(index), cfg.reset_us);

    // Keep configured-but-disabled channels silent.  MCPWM resources are
    // created lazily by enable(), not by set_config().
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
