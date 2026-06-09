#include "arm_motion.hpp"

#include <algorithm>
#include <cmath>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

namespace learm {

static const char* TAG = "arm_motion";

static constexpr uint32_t kMotionUpdatePeriodMs = 20;
static constexpr uint32_t kMinDurationMs = kMotionUpdatePeriodMs;
static constexpr uint32_t kMaxDurationMs = 30000;

static constexpr ArmJoint kMotionOrder[kArmJointCount] = {
    ArmJoint::Base,
    ArmJoint::Shoulder,
    ArmJoint::Elbow,
    ArmJoint::WristPitch,
    ArmJoint::WristRoll,
    ArmJoint::Claw,
};

int ArmMotion::joint_to_index(ArmJoint joint)
{
    const uint8_t index = static_cast<uint8_t>(joint);

    if (index >= kArmJointCount) {
        return -1;
    }

    return static_cast<int>(index);
}

uint32_t ArmMotion::clamp_duration_ms(uint32_t duration_ms)
{
    return std::clamp<uint32_t>(
        duration_ms,
        kMinDurationMs,
        kMaxDurationMs
    );
}

float ArmMotion::smoothstep(float t)
{
    t = std::clamp<float>(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

float ArmMotion::smootherstep(float t)
{
    t = std::clamp<float>(t, 0.0f, 1.0f);
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

float ArmMotion::evaluate_profile(MotionProfile profile, float t)
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

MotionProfile ArmMotion::style_to_profile(JointMotionStyle style)
{
    switch (style) {
        case JointMotionStyle::Linear:
            return MotionProfile::Linear;

        case JointMotionStyle::Smooth:
            return MotionProfile::SmoothStep;

        case JointMotionStyle::Soft:
        default:
            return MotionProfile::SmootherStep;
    }
}

JointMotionOptions ArmMotion::make_options(
    uint32_t duration_ms,
    JointMotionStyle style
)
{
    JointMotionOptions options;

    options.timing_mode = JointTimingMode::Duration;
    options.duration_ms = duration_ms;
    options.speed_deg_per_s = 60.0f;
    options.profile = style_to_profile(style);
    options.replan_from_current = true;
    options.max_step_us = 0;
    options.min_effective_step_us = 0;

    return options;
}

esp_err_t ArmMotion::init(ArmJointController* joints)
{
    if (joints == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!joints->is_initialized()) {
        ESP_LOGE(TAG, "init rejected: arm_joint is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (initialized_) {
        return ESP_OK;
    }

    lock_ = xSemaphoreCreateRecursiveMutex();
    if (lock_ == nullptr) {
        ESP_LOGE(TAG, "xSemaphoreCreateRecursiveMutex failed");
        return ESP_ERR_NO_MEM;
    }

    joints_ = joints;

    for (uint8_t i = 0; i < kArmJointCount; ++i) {
        states_[i] = MotionJointState{};
    }

    for (ArmJoint joint : kMotionOrder) {
        const int index = joint_to_index(joint);
        if (index < 0) {
            continue;
        }

        if (!joints_->is_configured(joint)) {
            continue;
        }

        const JointCalibration cal = joints_->get_calibration(joint);
        MotionJointState& s = states_[index];

        s.configured = true;
        s.enabled = false;
        s.running = false;
        s.joint = joint;
        s.channel = cal.channel;
        s.start_us = cal.zero_us;
        s.current_us = cal.zero_us;
        s.target_us = cal.zero_us;
        s.output_us = cal.zero_us;
        s.duration_ms = 0;
        s.total_steps = 0;
        s.elapsed_steps = 0;
        s.profile = MotionProfile::SmootherStep;
        s.max_step_us = 0;
        s.min_effective_step_us = 0;
    }

    task_stop_requested_ = false;
    initialized_ = true;

    const BaseType_t task_ret = xTaskCreatePinnedToCore(
        &ArmMotion::task_entry,
        "arm_motion",
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
        "arm_motion initialized, update_period=%ums",
        static_cast<unsigned>(kMotionUpdatePeriodMs)
    );

    return ESP_OK;
}

bool ArmMotion::is_initialized() const
{
    return initialized_;
}

esp_err_t ArmMotion::enable_all()
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }

    if (!initialized_ || joints_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    for (ArmJoint joint : kMotionOrder) {
        const int index = joint_to_index(joint);
        if (index < 0 || !states_[index].configured) {
            continue;
        }

        const esp_err_t ret = joints_->enable_joint(joint);
        if (ret != ESP_OK) {
            ESP_LOGE(
                TAG,
                "enable_all failed joint=%u ret=0x%x",
                static_cast<unsigned>(joint),
                static_cast<unsigned>(ret)
            );
            return ret;
        }

        const esp_err_t sync_ret = sync_state_from_hardware_locked(joint);
        if (sync_ret != ESP_OK) {
            return sync_ret;
        }
    }

    ESP_LOGI(TAG, "all joints enabled");
    return ESP_OK;
}

esp_err_t ArmMotion::disable_all()
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }

    if (!initialized_ || joints_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    for (ArmJoint joint : kMotionOrder) {
        const int index = joint_to_index(joint);
        if (index < 0 || !states_[index].configured) {
            continue;
        }

        MotionJointState& s = states_[index];
        s.running = false;
        s.target_us = s.current_us;
        s.start_us = s.current_us;
        s.duration_ms = 0;
        s.total_steps = 0;
        s.elapsed_steps = 0;

        const esp_err_t ret = joints_->disable_joint(joint);
        if (ret != ESP_OK) {
            return ret;
        }

        s.enabled = false;
    }

    ESP_LOGI(TAG, "all joints disabled");
    return ESP_OK;
}

esp_err_t ArmMotion::sync_state_from_hardware_locked(ArmJoint joint)
{
    if (joints_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    const int index = joint_to_index(joint);
    if (index < 0 || !states_[index].configured) {
        return ESP_ERR_INVALID_ARG;
    }

    MotionJointState& s = states_[index];

    const uint16_t current_us = joints_->read_command_us(joint);
    const uint16_t output_us = joints_->read_output_us(joint);

    s.enabled = true;
    s.running = false;
    s.start_us = current_us;
    s.current_us = current_us;
    s.target_us = current_us;
    s.output_us = output_us;
    s.duration_ms = 0;
    s.total_steps = 0;
    s.elapsed_steps = 0;
    s.profile = MotionProfile::Immediate;
    s.max_step_us = 0;
    s.min_effective_step_us = 0;

    return ESP_OK;
}

esp_err_t ArmMotion::set_durations_ms(
    uint32_t base_ms,
    uint32_t shoulder_ms,
    uint32_t elbow_ms,
    uint32_t wrist_pitch_ms,
    uint32_t wrist_roll_ms,
    uint32_t claw_ms
)
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }

    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    if (base_ms == 0 ||
        shoulder_ms == 0 ||
        elbow_ms == 0 ||
        wrist_pitch_ms == 0 ||
        wrist_roll_ms == 0 ||
        claw_ms == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    durations_ms_ = {
        base_ms,
        shoulder_ms,
        elbow_ms,
        wrist_pitch_ms,
        wrist_roll_ms,
        claw_ms,
    };

    ESP_LOGI(
        TAG,
        "durations updated base=%u shoulder=%u elbow=%u wrist_pitch=%u wrist_roll=%u claw=%u",
        static_cast<unsigned>(durations_ms_.base_ms),
        static_cast<unsigned>(durations_ms_.shoulder_ms),
        static_cast<unsigned>(durations_ms_.elbow_ms),
        static_cast<unsigned>(durations_ms_.wrist_pitch_ms),
        static_cast<unsigned>(durations_ms_.wrist_roll_ms),
        static_cast<unsigned>(durations_ms_.claw_ms)
    );

    return ESP_OK;
}

esp_err_t ArmMotion::set_strategies(
    JointMotionStyle base,
    JointMotionStyle shoulder,
    JointMotionStyle elbow,
    JointMotionStyle wrist_pitch,
    JointMotionStyle wrist_roll,
    JointMotionStyle claw
)
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }

    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    strategies_ = {
        base,
        shoulder,
        elbow,
        wrist_pitch,
        wrist_roll,
        claw,
    };

    ESP_LOGI(
        TAG,
        "strategies updated base=%u shoulder=%u elbow=%u wrist_pitch=%u wrist_roll=%u claw=%u",
        static_cast<unsigned>(strategies_.base),
        static_cast<unsigned>(strategies_.shoulder),
        static_cast<unsigned>(strategies_.elbow),
        static_cast<unsigned>(strategies_.wrist_pitch),
        static_cast<unsigned>(strategies_.wrist_roll),
        static_cast<unsigned>(strategies_.claw)
    );

    return ESP_OK;
}

ArmMotionDurationsMs ArmMotion::get_durations_ms() const
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return {};
    }

    return durations_ms_;
}

ArmMotionStrategies ArmMotion::get_strategies() const
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return {};
    }

    return strategies_;
}

esp_err_t ArmMotion::move_to(
    float base_deg,
    float shoulder_deg,
    float elbow_deg,
    float wrist_pitch_deg,
    float wrist_roll_deg,
    float claw_gap_cm
)
{
    const ArmMotionTarget target = {
        base_deg,
        shoulder_deg,
        elbow_deg,
        wrist_pitch_deg,
        wrist_roll_deg,
        claw_gap_cm,
    };

    return move_to(target);
}

esp_err_t ArmMotion::commit_joint_target_locked(
    ArmJoint joint,
    uint16_t target_us,
    const JointMotionOptions& options
)
{
    const int index = joint_to_index(joint);
    if (index < 0 || !states_[index].configured) {
        return ESP_ERR_INVALID_ARG;
    }

    MotionJointState& s = states_[index];

    if (!s.enabled) {
        ESP_LOGE(
            TAG,
            "move_to rejected: joint=%u not enabled",
            static_cast<unsigned>(joint)
        );
        return ESP_ERR_INVALID_STATE;
    }

    MotionProfile profile = options.profile;
    uint32_t duration_ms = clamp_duration_ms(options.duration_ms);

    switch (options.timing_mode) {
        case JointTimingMode::Immediate:
            profile = MotionProfile::Immediate;
            duration_ms = kMinDurationMs;
            break;

        case JointTimingMode::Duration:
            break;

        case JointTimingMode::Speed:
            // V5A 的 group move 以同步姿态事务为主；Speed mode 暂不用于多轴同步。
            return ESP_ERR_INVALID_ARG;

        default:
            return ESP_ERR_INVALID_ARG;
    }

    if (profile == MotionProfile::Immediate) {
        s.start_us = target_us;
        s.current_us = target_us;
        s.target_us = target_us;
        s.duration_ms = 0;
        s.total_steps = 0;
        s.elapsed_steps = 0;
        s.profile = profile;
        s.max_step_us = options.max_step_us;
        s.min_effective_step_us = options.min_effective_step_us;
        s.running = false;

        const esp_err_t ret = joints_->write_us_now(joint, target_us);
        if (ret != ESP_OK) {
            return ret;
        }

        s.output_us = joints_->read_output_us(joint);
        return ESP_OK;
    }

    s.start_us = (options.replan_from_current || !s.running) ?
        s.current_us :
        s.start_us;

    s.target_us = target_us;
    s.duration_ms = duration_ms;
    s.total_steps = std::max<uint32_t>(
        1,
        (duration_ms + kMotionUpdatePeriodMs - 1) / kMotionUpdatePeriodMs
    );
    s.elapsed_steps = 0;
    s.profile = profile;
    s.max_step_us = options.max_step_us;
    s.min_effective_step_us = options.min_effective_step_us;
    s.running = (s.start_us != s.target_us);

    if (!s.running) {
        s.current_us = s.target_us;
        s.output_us = joints_->read_output_us(joint);
    }

    return ESP_OK;
}

esp_err_t ArmMotion::move_to(const ArmMotionTarget& target)
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }

    if (!initialized_ || joints_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    struct PreparedTarget {
        ArmJoint joint;
        uint16_t target_us;
        JointMotionOptions options;
    };

    PreparedTarget prepared[kArmJointCount] = {
        {
            ArmJoint::Base,
            joints_->deg_to_us(ArmJoint::Base, target.base_deg),
            make_options(durations_ms_.base_ms, strategies_.base)
        },
        {
            ArmJoint::Shoulder,
            joints_->deg_to_us(ArmJoint::Shoulder, target.shoulder_deg),
            make_options(durations_ms_.shoulder_ms, strategies_.shoulder)
        },
        {
            ArmJoint::Elbow,
            joints_->deg_to_us(ArmJoint::Elbow, target.elbow_deg),
            make_options(durations_ms_.elbow_ms, strategies_.elbow)
        },
        {
            ArmJoint::WristPitch,
            joints_->deg_to_us(ArmJoint::WristPitch, target.wrist_pitch_deg),
            make_options(durations_ms_.wrist_pitch_ms, strategies_.wrist_pitch)
        },
        {
            ArmJoint::WristRoll,
            joints_->deg_to_us(ArmJoint::WristRoll, target.wrist_roll_deg),
            make_options(durations_ms_.wrist_roll_ms, strategies_.wrist_roll)
        },
        {
            ArmJoint::Claw,
            joints_->claw_gap_cm_to_us(target.claw_gap_cm),
            make_options(durations_ms_.claw_ms, strategies_.claw)
        },
    };

    // 先完整校验，避免部分提交。
    for (const PreparedTarget& item : prepared) {
        const int index = joint_to_index(item.joint);
        if (index < 0 || !states_[index].configured) {
            return ESP_ERR_INVALID_ARG;
        }

        if (!states_[index].enabled) {
            ESP_LOGE(
                TAG,
                "move_to rejected: joint=%u not enabled",
                static_cast<unsigned>(item.joint)
            );
            return ESP_ERR_INVALID_STATE;
        }

        if (item.target_us == 0) {
            return ESP_ERR_INVALID_ARG;
        }
    }

    for (const PreparedTarget& item : prepared) {
        const esp_err_t ret = commit_joint_target_locked(
            item.joint,
            item.target_us,
            item.options
        );

        if (ret != ESP_OK) {
            return ret;
        }
    }

    ESP_LOGI(
        TAG,
        "move_to motion-owned base=%.2f shoulder=%.2f elbow=%.2f wrist_pitch=%.2f wrist_roll=%.2f claw_gap=%.2fcm",
        static_cast<double>(target.base_deg),
        static_cast<double>(target.shoulder_deg),
        static_cast<double>(target.elbow_deg),
        static_cast<double>(target.wrist_pitch_deg),
        static_cast<double>(target.wrist_roll_deg),
        static_cast<double>(target.claw_gap_cm)
    );

    return ESP_OK;
}

esp_err_t ArmMotion::stop_all()
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }

    if (!initialized_ || joints_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    for (uint8_t i = 0; i < kArmJointCount; ++i) {
        MotionJointState& s = states_[i];
        if (!s.configured || !s.enabled) {
            continue;
        }

        s.target_us = s.current_us;
        s.start_us = s.current_us;
        s.running = false;
        s.duration_ms = 0;
        s.total_steps = 0;
        s.elapsed_steps = 0;
    }

    return ESP_OK;
}

bool ArmMotion::is_ready() const
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return false;
    }

    if (!initialized_ || joints_ == nullptr) {
        return false;
    }

    for (uint8_t i = 0; i < kArmJointCount; ++i) {
        const MotionJointState& s = states_[i];
        if (!s.configured || !s.enabled) {
            continue;
        }

        if (s.running || s.current_us != s.target_us) {
            return false;
        }
    }

    return true;
}

bool ArmMotion::is_moving() const
{
    return !is_ready();
}

JointRuntimeState ArmMotion::make_runtime_state_locked(ArmJoint joint) const
{
    const int index = joint_to_index(joint);
    if (index < 0 || !states_[index].configured || joints_ == nullptr) {
        return {};
    }

    const MotionJointState& s = states_[index];
    const JointCalibration cal = joints_->get_calibration(joint);

    JointRuntimeState state = {};

    state.configured = s.configured;
    state.enabled = s.enabled;
    state.running = s.running;
    state.ready = !s.running && (s.current_us == s.target_us);

    state.joint = joint;
    state.channel = cal.channel;

    state.current_us = s.current_us;
    state.target_us = s.target_us;
    state.output_us = s.output_us;

    state.current_deg = joints_->us_to_deg(joint, s.current_us);
    state.target_deg = joints_->us_to_deg(joint, s.target_us);

    state.zero_deg = cal.zero_deg;
    state.zero_us = cal.zero_us;
    state.us_per_degree = cal.us_per_degree;
    state.direction = cal.direction;

    state.min_deg = cal.min_deg;
    state.max_deg = cal.max_deg;

    state.duration_ms = s.duration_ms;
    state.total_steps = s.total_steps;
    state.elapsed_steps = s.elapsed_steps;

    state.profile = s.profile;

    return state;
}

ArmMotionState ArmMotion::get_state() const
{
    const LockGuard guard(this);

    ArmMotionState state = {};

    if (!guard.locked()) {
        return state;
    }

    state.initialized = initialized_;
    state.durations_ms = durations_ms_;
    state.strategies = strategies_;

    if (!initialized_ || joints_ == nullptr) {
        state.ready = false;
        state.moving = false;
        return state;
    }

    state.base = make_runtime_state_locked(ArmJoint::Base);
    state.shoulder = make_runtime_state_locked(ArmJoint::Shoulder);
    state.elbow = make_runtime_state_locked(ArmJoint::Elbow);
    state.wrist_pitch = make_runtime_state_locked(ArmJoint::WristPitch);
    state.wrist_roll = make_runtime_state_locked(ArmJoint::WristRoll);
    state.claw = make_runtime_state_locked(ArmJoint::Claw);

    const int claw_index = joint_to_index(ArmJoint::Claw);
    if (claw_index >= 0 && states_[claw_index].configured) {
        state.claw_gap_cm = joints_->claw_us_to_gap_cm(
            states_[claw_index].current_us
        );
        state.claw_target_gap_cm = joints_->claw_us_to_gap_cm(
            states_[claw_index].target_us
        );
    }

    state.ready = true;
    for (uint8_t i = 0; i < kArmJointCount; ++i) {
        const MotionJointState& s = states_[i];
        if (!s.configured || !s.enabled) {
            continue;
        }

        if (s.running || s.current_us != s.target_us) {
            state.ready = false;
            break;
        }
    }

    state.moving = !state.ready;

    return state;
}

void ArmMotion::task_entry(void* arg)
{
    auto* self = static_cast<ArmMotion*>(arg);

    if (self != nullptr) {
        self->task_loop();
    }

    vTaskDelete(nullptr);
}

void ArmMotion::task_loop()
{
    const TickType_t period_ticks = pdMS_TO_TICKS(kMotionUpdatePeriodMs);
    TickType_t last_wake = xTaskGetTickCount();

    while (!task_stop_requested_) {
        {
            LockGuard guard(this, pdMS_TO_TICKS(5));
            if (guard.locked()) {
                update_all_locked();
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

void ArmMotion::update_all_locked()
{
    if (!initialized_ || joints_ == nullptr) {
        return;
    }

    for (ArmJoint joint : kMotionOrder) {
        const int index = joint_to_index(joint);
        if (index < 0) {
            continue;
        }

        const esp_err_t ret = update_one_locked(static_cast<uint8_t>(index));
        if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(
                TAG,
                "update_one failed joint=%u ret=0x%x",
                static_cast<unsigned>(joint),
                static_cast<unsigned>(ret)
            );
        }
    }
}

esp_err_t ArmMotion::update_one_locked(uint8_t index)
{
    if (index >= kArmJointCount || joints_ == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    MotionJointState& s = states_[index];

    if (!s.configured || !s.enabled || !s.running) {
        return ESP_OK;
    }

    const uint16_t previous_us = s.current_us;
    bool reached_final = false;

    s.elapsed_steps++;

    if (s.elapsed_steps >= s.total_steps) {
        s.current_us = s.target_us;
        s.running = false;
        reached_final = true;
    } else {
        const float t =
            static_cast<float>(s.elapsed_steps) /
            static_cast<float>(s.total_steps);

        const float shaped = evaluate_profile(
            s.profile,
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

        int step = proposed - static_cast<int>(previous_us);

        if (s.max_step_us > 0) {
            const int max_step = static_cast<int>(s.max_step_us);

            if (step > max_step) {
                step = max_step;
            } else if (step < -max_step) {
                step = -max_step;
            }

            proposed = static_cast<int>(previous_us) + step;
        }

        if (s.min_effective_step_us > 0 && remaining != 0) {
            const int min_step = static_cast<int>(s.min_effective_step_us);
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
            static_cast<int>(kServoMinUs),
            static_cast<int>(kServoMaxUs)
        );

        s.current_us = static_cast<uint16_t>(proposed);
    }

    if (s.current_us == previous_us && !reached_final) {
        return ESP_OK;
    }

    const esp_err_t ret = joints_->write_us_now(
        s.joint,
        s.current_us
    );

    if (ret != ESP_OK) {
        return ret;
    }

    s.output_us = joints_->read_output_us(s.joint);

    return ESP_OK;
}

bool ArmMotion::take_lock(TickType_t timeout_ticks) const
{
    if (lock_ == nullptr) {
        return true;
    }

    return xSemaphoreTakeRecursive(lock_, timeout_ticks) == pdTRUE;
}

void ArmMotion::give_lock() const
{
    if (lock_ != nullptr) {
        xSemaphoreGiveRecursive(lock_);
    }
}

ArmMotion::LockGuard::LockGuard(
    const ArmMotion* owner,
    TickType_t timeout_ticks
) : owner_(owner)
{
    locked_ = (owner_ != nullptr) && owner_->take_lock(timeout_ticks);
}

ArmMotion::LockGuard::~LockGuard()
{
    if (locked_ && owner_ != nullptr) {
        owner_->give_lock();
    }
}

bool ArmMotion::LockGuard::locked() const
{
    return locked_;
}

}  // namespace learm
