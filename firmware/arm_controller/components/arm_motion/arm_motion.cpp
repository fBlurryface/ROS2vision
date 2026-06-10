#include "arm_motion.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

namespace learm {

static const char* TAG = "arm_motion";

static constexpr uint32_t kMotionUpdatePeriodMs = 20;
static constexpr float kMotionUpdatePeriodS =
    static_cast<float>(kMotionUpdatePeriodMs) / 1000.0f;
static constexpr float kMaxMotionDtS = kMotionUpdatePeriodS * 5.0f;
static constexpr uint32_t kMinDurationMs = kMotionUpdatePeriodMs;
static constexpr uint32_t kMaxDurationMs = 30000;

static constexpr float kMinSpeedUnitsPerS = 0.001f;
static constexpr float kPositionEpsilon = 0.0005f;

// Speed line is direct linear speed: no ease-in, no brake curve.
// Use measured dt for integration. kMotionUpdatePeriodMs is only the target
// scheduler period, not a source of truth for elapsed time.

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

bool ArmMotion::is_valid_joint_speed_strategy(
    const JointSpeedStrategy& strategy
)
{
    return std::isfinite(strategy.max_speed_deg_per_s) &&
           strategy.max_speed_deg_per_s > 0.0f;
}

bool ArmMotion::is_valid_claw_speed_strategy(
    const ClawSpeedStrategy& strategy
)
{
    return std::isfinite(strategy.max_speed_cm_per_s) &&
           strategy.max_speed_cm_per_s > 0.0f;
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

JointMotionOptions ArmMotion::make_duration_options(
    uint32_t duration_ms,
    JointMotionStyle style
)
{
    JointMotionOptions options;

    options.timing_mode = JointTimingMode::Duration;
    options.duration_ms = duration_ms;
    options.speed_deg_per_s = 60.0f;
    options.accel_deg_per_s2 = 0.0f;
    options.decel_deg_per_s2 = 0.0f;
    options.profile = style_to_profile(style);
    options.replan_from_current = true;
    options.max_step_us = 0;
    options.min_effective_step_us = 0;

    return options;
}

JointMotionOptions ArmMotion::make_options(
    uint32_t duration_ms,
    JointMotionStyle style
)
{
    return make_duration_options(duration_ms, style);
}

JointMotionOptions ArmMotion::make_speed_options(
    float max_speed_units_per_s
)
{
    JointMotionOptions options;

    options.timing_mode = JointTimingMode::Speed;
    options.duration_ms = kMinDurationMs;
    options.speed_deg_per_s = max_speed_units_per_s;
    // Direct linear speed: no acceleration/deceleration curve.
    options.accel_deg_per_s2 = 0.0f;
    options.decel_deg_per_s2 = 0.0f;
    options.profile = MotionProfile::Linear;
    options.replan_from_current = true;
    options.max_step_us = 0;
    options.min_effective_step_us = 0;

    return options;
}

JointMotionOptions ArmMotion::make_immediate_options()
{
    JointMotionOptions options;

    options.timing_mode = JointTimingMode::Immediate;
    options.duration_ms = kMinDurationMs;
    options.speed_deg_per_s = 1.0f;
    options.accel_deg_per_s2 = 0.0f;
    options.decel_deg_per_s2 = 0.0f;
    options.profile = MotionProfile::Immediate;
    options.replan_from_current = true;
    options.max_step_us = 0;
    options.min_effective_step_us = 0;

    return options;
}

void ArmMotion::clear_motion_progress_locked(MotionJointState& s)
{
    s.duration_ms = 0;
    s.total_steps = 0;
    s.elapsed_steps = 0;
    s.elapsed_time_ms = 0.0f;
    s.current_speed_units_per_s = 0.0f;
    s.max_speed_units_per_s = 0.0f;
    s.accel_units_per_s2 = 0.0f;
    s.decel_units_per_s2 = 0.0f;
}

float ArmMotion::motion_position_from_us_locked(
    ArmJoint joint,
    uint16_t pulse_us
) const
{
    if (joints_ == nullptr) {
        return 0.0f;
    }

    if (joint == ArmJoint::Claw) {
        return joints_->claw_us_to_gap_cm(pulse_us);
    }

    return joints_->us_to_deg(joint, pulse_us);
}

uint16_t ArmMotion::motion_us_from_position_locked(
    ArmJoint joint,
    float position
) const
{
    if (joints_ == nullptr) {
        return 0;
    }

    if (joint == ArmJoint::Claw) {
        return joints_->claw_gap_cm_to_us(position);
    }

    return joints_->deg_to_us(joint, position);
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
        s.timing_mode = JointTimingMode::Duration;
        s.start_us = cal.zero_us;
        s.current_us = cal.zero_us;
        s.target_us = cal.zero_us;
        s.output_us = cal.zero_us;
        s.start_us_f = static_cast<float>(cal.zero_us);
        s.current_us_f = static_cast<float>(cal.zero_us);
        s.target_us_f = static_cast<float>(cal.zero_us);
        s.duration_ms = 0;
        s.total_steps = 0;
        s.elapsed_steps = 0;
        s.profile = MotionProfile::SmootherStep;
        s.max_step_us = 0;
        s.min_effective_step_us = 0;
        s.current_position = motion_position_from_us_locked(joint, cal.zero_us);
        s.target_position = s.current_position;
        s.current_speed_units_per_s = 0.0f;
        s.max_speed_units_per_s = 0.0f;
        s.accel_units_per_s2 = 0.0f;
        s.decel_units_per_s2 = 0.0f;
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
        s.start_us_f = s.current_us_f;
        s.target_us_f = s.current_us_f;
        s.current_position = motion_position_from_us_locked(joint, s.current_us);
        s.target_position = s.current_position;
        clear_motion_progress_locked(s);

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
    const float current_position = motion_position_from_us_locked(
        joint,
        current_us
    );

    s.enabled = true;
    s.running = false;
    s.timing_mode = JointTimingMode::Duration;
    s.start_us = current_us;
    s.current_us = current_us;
    s.target_us = current_us;
    s.output_us = output_us;
    s.start_us_f = static_cast<float>(current_us);
    s.current_us_f = static_cast<float>(current_us);
    s.target_us_f = static_cast<float>(current_us);
    s.current_position = current_position;
    s.target_position = current_position;
    s.profile = MotionProfile::Immediate;
    s.max_step_us = 0;
    s.min_effective_step_us = 0;
    clear_motion_progress_locked(s);

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

esp_err_t ArmMotion::set_duration_strategies(
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

    duration_strategies_ = {
        base,
        shoulder,
        elbow,
        wrist_pitch,
        wrist_roll,
        claw,
    };

    ESP_LOGI(
        TAG,
        "duration strategies updated base=%u shoulder=%u elbow=%u wrist_pitch=%u wrist_roll=%u claw=%u",
        static_cast<unsigned>(duration_strategies_.base),
        static_cast<unsigned>(duration_strategies_.shoulder),
        static_cast<unsigned>(duration_strategies_.elbow),
        static_cast<unsigned>(duration_strategies_.wrist_pitch),
        static_cast<unsigned>(duration_strategies_.wrist_roll),
        static_cast<unsigned>(duration_strategies_.claw)
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
    return set_duration_strategies(
        base,
        shoulder,
        elbow,
        wrist_pitch,
        wrist_roll,
        claw
    );
}

esp_err_t ArmMotion::set_speed_strategies(
    const JointSpeedStrategy& base,
    const JointSpeedStrategy& shoulder,
    const JointSpeedStrategy& elbow,
    const JointSpeedStrategy& wrist_pitch,
    const JointSpeedStrategy& wrist_roll,
    const ClawSpeedStrategy& claw
)
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }

    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!is_valid_joint_speed_strategy(base) ||
        !is_valid_joint_speed_strategy(shoulder) ||
        !is_valid_joint_speed_strategy(elbow) ||
        !is_valid_joint_speed_strategy(wrist_pitch) ||
        !is_valid_joint_speed_strategy(wrist_roll) ||
        !is_valid_claw_speed_strategy(claw)) {
        return ESP_ERR_INVALID_ARG;
    }

    speed_strategies_ = {
        base,
        shoulder,
        elbow,
        wrist_pitch,
        wrist_roll,
        claw,
    };

    ESP_LOGI(
        TAG,
        "speed strategies updated linear max: base=%.2f shoulder=%.2f elbow=%.2f wrist_pitch=%.2f wrist_roll=%.2f claw=%.2f",
        static_cast<double>(speed_strategies_.base.max_speed_deg_per_s),
        static_cast<double>(speed_strategies_.shoulder.max_speed_deg_per_s),
        static_cast<double>(speed_strategies_.elbow.max_speed_deg_per_s),
        static_cast<double>(speed_strategies_.wrist_pitch.max_speed_deg_per_s),
        static_cast<double>(speed_strategies_.wrist_roll.max_speed_deg_per_s),
        static_cast<double>(speed_strategies_.claw.max_speed_cm_per_s)
    );

    return ESP_OK;
}

esp_err_t ArmMotion::set_speed_limits(
    float base_deg_per_s,
    float shoulder_deg_per_s,
    float elbow_deg_per_s,
    float wrist_pitch_deg_per_s,
    float wrist_roll_deg_per_s,
    float claw_cm_per_s
)
{
    ArmMotionSpeedStrategies next = get_speed_strategies();

    next.base.max_speed_deg_per_s = base_deg_per_s;
    next.shoulder.max_speed_deg_per_s = shoulder_deg_per_s;
    next.elbow.max_speed_deg_per_s = elbow_deg_per_s;
    next.wrist_pitch.max_speed_deg_per_s = wrist_pitch_deg_per_s;
    next.wrist_roll.max_speed_deg_per_s = wrist_roll_deg_per_s;
    next.claw.max_speed_cm_per_s = claw_cm_per_s;

    return set_speed_strategies(
        next.base,
        next.shoulder,
        next.elbow,
        next.wrist_pitch,
        next.wrist_roll,
        next.claw
    );
}

ArmMotionDurationsMs ArmMotion::get_durations_ms() const
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return {};
    }

    return durations_ms_;
}

ArmMotionDurationStrategies ArmMotion::get_duration_strategies() const
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return {};
    }

    return duration_strategies_;
}

ArmMotionStrategies ArmMotion::get_strategies() const
{
    return get_duration_strategies();
}

ArmMotionSpeedStrategies ArmMotion::get_speed_strategies() const
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return {};
    }

    return speed_strategies_;
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

esp_err_t ArmMotion::move_to_speed(
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

    return move_to_speed(target);
}

esp_err_t ArmMotion::move_to_immediate(
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

    return move_to_immediate(target);
}

esp_err_t ArmMotion::move_delta_immediate(
    float base_delta_deg,
    float shoulder_delta_deg,
    float elbow_delta_deg,
    float wrist_pitch_delta_deg,
    float wrist_roll_delta_deg,
    float claw_gap_delta_cm
)
{
    const ArmMotionDelta delta = {
        base_delta_deg,
        shoulder_delta_deg,
        elbow_delta_deg,
        wrist_pitch_delta_deg,
        wrist_roll_delta_deg,
        claw_gap_delta_cm,
    };

    return move_delta_immediate(delta);
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
            "move rejected: joint=%u not enabled",
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
            if (!std::isfinite(options.speed_deg_per_s) ||
                !std::isfinite(options.accel_deg_per_s2) ||
                !std::isfinite(options.decel_deg_per_s2) ||
                options.speed_deg_per_s <= 0.0f ||
                options.accel_deg_per_s2 < 0.0f ||
                options.decel_deg_per_s2 < 0.0f) {
                return ESP_ERR_INVALID_ARG;
            }
            break;

        default:
            return ESP_ERR_INVALID_ARG;
    }

    if (profile == MotionProfile::Immediate ||
        options.timing_mode == JointTimingMode::Immediate) {
        s.timing_mode = JointTimingMode::Immediate;
        s.start_us = target_us;
        s.current_us = target_us;
        s.target_us = target_us;
        s.start_us_f = static_cast<float>(target_us);
        s.current_us_f = static_cast<float>(target_us);
        s.target_us_f = static_cast<float>(target_us);
        s.current_position = motion_position_from_us_locked(joint, target_us);
        s.target_position = s.current_position;
        s.profile = profile;
        s.max_step_us = options.max_step_us;
        s.min_effective_step_us = options.min_effective_step_us;
        s.running = false;
        clear_motion_progress_locked(s);

        const esp_err_t ret = joints_->write_us_now(joint, target_us);
        if (ret != ESP_OK) {
            return ret;
        }

        s.output_us = joints_->read_output_us(joint);
        return ESP_OK;
    }

    const uint16_t effective_start_us =
        (options.replan_from_current || !s.running) ?
            s.current_us :
            s.start_us;

    s.start_us = effective_start_us;
    s.current_us = effective_start_us;
    s.target_us = target_us;
    s.start_us_f = static_cast<float>(effective_start_us);
    s.current_us_f = static_cast<float>(effective_start_us);
    s.target_us_f = static_cast<float>(target_us);
    s.profile = profile;
    s.max_step_us = options.max_step_us;
    s.min_effective_step_us = options.min_effective_step_us;

    if (options.timing_mode == JointTimingMode::Duration) {
        s.timing_mode = JointTimingMode::Duration;
        s.duration_ms = duration_ms;
        s.total_steps = std::max<uint32_t>(
            1,
            (duration_ms + kMotionUpdatePeriodMs - 1) / kMotionUpdatePeriodMs
        );
        s.elapsed_steps = 0;
        s.elapsed_time_ms = 0.0f;
        s.current_position = motion_position_from_us_locked(joint, s.current_us);
        s.target_position = motion_position_from_us_locked(joint, s.target_us);
        s.current_speed_units_per_s = 0.0f;
        s.max_speed_units_per_s = 0.0f;
        s.accel_units_per_s2 = 0.0f;
        s.decel_units_per_s2 = 0.0f;
        s.running = (s.start_us != s.target_us);
    } else if (options.timing_mode == JointTimingMode::Speed) {
        s.timing_mode = JointTimingMode::Speed;
        s.duration_ms = 0;
        s.total_steps = 0;
        s.elapsed_steps = 0;
        s.elapsed_time_ms = 0.0f;
        s.current_position = motion_position_from_us_locked(joint, s.current_us);
        s.target_position = motion_position_from_us_locked(joint, s.target_us);
        const float distance_units = std::fabs(s.target_position - s.current_position);

        s.current_speed_units_per_s = options.speed_deg_per_s;
        s.max_speed_units_per_s = options.speed_deg_per_s;
        s.accel_units_per_s2 = 0.0f;
        s.decel_units_per_s2 = 0.0f;
        s.running = (distance_units > kPositionEpsilon) &&
                    (s.current_us != s.target_us);
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s.running) {
        s.current_us = s.target_us;
        s.current_us_f = s.target_us_f;
        s.current_position = s.target_position;
        s.current_speed_units_per_s = 0.0f;
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
            make_duration_options(durations_ms_.base_ms, duration_strategies_.base)
        },
        {
            ArmJoint::Shoulder,
            joints_->deg_to_us(ArmJoint::Shoulder, target.shoulder_deg),
            make_duration_options(durations_ms_.shoulder_ms, duration_strategies_.shoulder)
        },
        {
            ArmJoint::Elbow,
            joints_->deg_to_us(ArmJoint::Elbow, target.elbow_deg),
            make_duration_options(durations_ms_.elbow_ms, duration_strategies_.elbow)
        },
        {
            ArmJoint::WristPitch,
            joints_->deg_to_us(ArmJoint::WristPitch, target.wrist_pitch_deg),
            make_duration_options(durations_ms_.wrist_pitch_ms, duration_strategies_.wrist_pitch)
        },
        {
            ArmJoint::WristRoll,
            joints_->deg_to_us(ArmJoint::WristRoll, target.wrist_roll_deg),
            make_duration_options(durations_ms_.wrist_roll_ms, duration_strategies_.wrist_roll)
        },
        {
            ArmJoint::Claw,
            joints_->claw_gap_cm_to_us(target.claw_gap_cm),
            make_duration_options(durations_ms_.claw_ms, duration_strategies_.claw)
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
        "move_to duration base=%.2f shoulder=%.2f elbow=%.2f wrist_pitch=%.2f wrist_roll=%.2f claw_gap=%.2fcm",
        static_cast<double>(target.base_deg),
        static_cast<double>(target.shoulder_deg),
        static_cast<double>(target.elbow_deg),
        static_cast<double>(target.wrist_pitch_deg),
        static_cast<double>(target.wrist_roll_deg),
        static_cast<double>(target.claw_gap_cm)
    );

    return ESP_OK;
}

esp_err_t ArmMotion::move_to_speed(const ArmMotionTarget& target)
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
            make_speed_options(
                speed_strategies_.base.max_speed_deg_per_s
            )
        },
        {
            ArmJoint::Shoulder,
            joints_->deg_to_us(ArmJoint::Shoulder, target.shoulder_deg),
            make_speed_options(
                speed_strategies_.shoulder.max_speed_deg_per_s
            )
        },
        {
            ArmJoint::Elbow,
            joints_->deg_to_us(ArmJoint::Elbow, target.elbow_deg),
            make_speed_options(
                speed_strategies_.elbow.max_speed_deg_per_s
            )
        },
        {
            ArmJoint::WristPitch,
            joints_->deg_to_us(ArmJoint::WristPitch, target.wrist_pitch_deg),
            make_speed_options(
                speed_strategies_.wrist_pitch.max_speed_deg_per_s
            )
        },
        {
            ArmJoint::WristRoll,
            joints_->deg_to_us(ArmJoint::WristRoll, target.wrist_roll_deg),
            make_speed_options(
                speed_strategies_.wrist_roll.max_speed_deg_per_s
            )
        },
        {
            ArmJoint::Claw,
            joints_->claw_gap_cm_to_us(target.claw_gap_cm),
            make_speed_options(
                speed_strategies_.claw.max_speed_cm_per_s
            )
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
                "move_to_speed rejected: joint=%u not enabled",
                static_cast<unsigned>(item.joint)
            );
            return ESP_ERR_INVALID_STATE;
        }

        if (item.target_us == 0 ||
            item.options.speed_deg_per_s <= 0.0f) {
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
        "move_to speed base=%.2f shoulder=%.2f elbow=%.2f wrist_pitch=%.2f wrist_roll=%.2f claw_gap=%.2fcm",
        static_cast<double>(target.base_deg),
        static_cast<double>(target.shoulder_deg),
        static_cast<double>(target.elbow_deg),
        static_cast<double>(target.wrist_pitch_deg),
        static_cast<double>(target.wrist_roll_deg),
        static_cast<double>(target.claw_gap_cm)
    );

    return ESP_OK;
}

esp_err_t ArmMotion::move_to_immediate(const ArmMotionTarget& target)
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }

    if (!initialized_ || joints_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    const JointMotionOptions immediate_options = make_immediate_options();

    struct PreparedTarget {
        ArmJoint joint;
        uint16_t target_us;
        JointMotionOptions options;
    };

    PreparedTarget prepared[kArmJointCount] = {
        {
            ArmJoint::Base,
            joints_->deg_to_us(ArmJoint::Base, target.base_deg),
            immediate_options
        },
        {
            ArmJoint::Shoulder,
            joints_->deg_to_us(ArmJoint::Shoulder, target.shoulder_deg),
            immediate_options
        },
        {
            ArmJoint::Elbow,
            joints_->deg_to_us(ArmJoint::Elbow, target.elbow_deg),
            immediate_options
        },
        {
            ArmJoint::WristPitch,
            joints_->deg_to_us(ArmJoint::WristPitch, target.wrist_pitch_deg),
            immediate_options
        },
        {
            ArmJoint::WristRoll,
            joints_->deg_to_us(ArmJoint::WristRoll, target.wrist_roll_deg),
            immediate_options
        },
        {
            ArmJoint::Claw,
            joints_->claw_gap_cm_to_us(target.claw_gap_cm),
            immediate_options
        },
    };

    // Validate the complete group first, so an invalid command does not
    // partially update some joints.
    for (const PreparedTarget& item : prepared) {
        const int index = joint_to_index(item.joint);
        if (index < 0 || !states_[index].configured) {
            return ESP_ERR_INVALID_ARG;
        }

        if (!states_[index].enabled) {
            ESP_LOGE(
                TAG,
                "move_to_immediate rejected: joint=%u not enabled",
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

    // Intentionally no INFO log here: immediate mode may be called at high rate
    // by an external controller such as an incremental Jacobian loop.
    return ESP_OK;
}

esp_err_t ArmMotion::move_delta_immediate(const ArmMotionDelta& delta)
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }

    if (!initialized_ || joints_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    const JointMotionOptions immediate_options = make_immediate_options();

    struct PreparedTarget {
        ArmJoint joint;
        uint16_t target_us;
        JointMotionOptions options;
    };

    const int base_index = joint_to_index(ArmJoint::Base);
    const int shoulder_index = joint_to_index(ArmJoint::Shoulder);
    const int elbow_index = joint_to_index(ArmJoint::Elbow);
    const int wrist_pitch_index = joint_to_index(ArmJoint::WristPitch);
    const int wrist_roll_index = joint_to_index(ArmJoint::WristRoll);
    const int claw_index = joint_to_index(ArmJoint::Claw);

    if (base_index < 0 || shoulder_index < 0 || elbow_index < 0 ||
        wrist_pitch_index < 0 || wrist_roll_index < 0 || claw_index < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!std::isfinite(delta.base_deg) ||
        !std::isfinite(delta.shoulder_deg) ||
        !std::isfinite(delta.elbow_deg) ||
        !std::isfinite(delta.wrist_pitch_deg) ||
        !std::isfinite(delta.wrist_roll_deg) ||
        !std::isfinite(delta.claw_gap_cm)) {
        return ESP_ERR_INVALID_ARG;
    }

    const float next_base =
        states_[base_index].current_position + delta.base_deg;
    const float next_shoulder =
        states_[shoulder_index].current_position + delta.shoulder_deg;
    const float next_elbow =
        states_[elbow_index].current_position + delta.elbow_deg;
    const float next_wrist_pitch =
        states_[wrist_pitch_index].current_position + delta.wrist_pitch_deg;
    const float next_wrist_roll =
        states_[wrist_roll_index].current_position + delta.wrist_roll_deg;
    const float next_claw_gap =
        states_[claw_index].current_position + delta.claw_gap_cm;

    PreparedTarget prepared[kArmJointCount] = {
        {
            ArmJoint::Base,
            joints_->deg_to_us(ArmJoint::Base, next_base),
            immediate_options
        },
        {
            ArmJoint::Shoulder,
            joints_->deg_to_us(ArmJoint::Shoulder, next_shoulder),
            immediate_options
        },
        {
            ArmJoint::Elbow,
            joints_->deg_to_us(ArmJoint::Elbow, next_elbow),
            immediate_options
        },
        {
            ArmJoint::WristPitch,
            joints_->deg_to_us(ArmJoint::WristPitch, next_wrist_pitch),
            immediate_options
        },
        {
            ArmJoint::WristRoll,
            joints_->deg_to_us(ArmJoint::WristRoll, next_wrist_roll),
            immediate_options
        },
        {
            ArmJoint::Claw,
            joints_->claw_gap_cm_to_us(next_claw_gap),
            immediate_options
        },
    };

    // Validate the whole group before writing anything. This keeps a bad delta
    // command from partially updating the arm.
    for (const PreparedTarget& item : prepared) {
        const int index = joint_to_index(item.joint);
        if (index < 0 || !states_[index].configured) {
            return ESP_ERR_INVALID_ARG;
        }

        if (!states_[index].enabled) {
            ESP_LOGE(
                TAG,
                "move_delta_immediate rejected: joint=%u not enabled",
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

    // Intentionally no INFO log here: delta-immediate mode is intended for
    // high-rate external controllers.
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

        // STOP / HOLD semantics:
        //   - abort the active trajectory immediately;
        //   - keep PWM enabled so the servo holds the last commanded pose;
        //   - collapse the target onto the current commanded position so the
        //     next move/movespeed replans from here instead of continuing the
        //     old target.
        s.running = false;
        s.target_us = s.current_us;
        s.start_us = s.current_us;
        s.start_us_f = s.current_us_f;
        s.target_us_f = s.current_us_f;
        s.current_position = motion_position_from_us_locked(s.joint, s.current_us);
        s.target_position = s.current_position;
        s.current_speed_units_per_s = 0.0f;
        clear_motion_progress_locked(s);

        const esp_err_t ret = joints_->write_us_now(s.joint, s.current_us);
        if (ret != ESP_OK) {
            ESP_LOGE(
                TAG,
                "stop hold write failed joint=%u ret=0x%x",
                static_cast<unsigned>(s.joint),
                static_cast<unsigned>(ret)
            );
            return ret;
        }

        s.output_us = joints_->read_output_us(s.joint);
    }

    ESP_LOGI(TAG, "all joints stopped and holding current commanded pose");
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
    state.duration_strategies = duration_strategies_;
    state.strategies = duration_strategies_;
    state.speed_strategies = speed_strategies_;

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

    int64_t last_loop_us = esp_timer_get_time();
    bool motion_active = false;
    int64_t motion_start_us = 0;

    const auto any_running_locked = [this]() -> bool {
        for (uint8_t i = 0; i < kArmJointCount; ++i) {
            const MotionJointState& s = states_[i];
            if (s.configured && s.enabled && s.running) {
                return true;
            }
        }
        return false;
    };

    ESP_LOGI(
        TAG,
        "motion task started: target_period_ms=%lu period_ticks=%lu dt_fix=1",
        static_cast<unsigned long>(kMotionUpdatePeriodMs),
        static_cast<unsigned long>(period_ticks)
    );

    while (!task_stop_requested_) {
        const int64_t now_us = esp_timer_get_time();
        const int64_t raw_loop_dt_us = now_us - last_loop_us;
        last_loop_us = now_us;

        const float raw_dt_s = std::max(
            0.0f,
            static_cast<float>(raw_loop_dt_us) / 1000000.0f
        );
        const float dt_s = std::min(raw_dt_s, kMaxMotionDtS);

        {
            LockGuard guard(this, pdMS_TO_TICKS(5));
            if (guard.locked()) {
                const bool moving_before_update = any_running_locked();
                const int64_t update_start_us = esp_timer_get_time();

                update_all_locked(dt_s);

                const bool moving_after_update = any_running_locked();

                if (!motion_active && (moving_before_update || moving_after_update)) {
                    motion_active = true;
                    motion_start_us = update_start_us;
                    ESP_LOGI(TAG, "motion active window started");
                }

                if (motion_active && !moving_after_update) {
                    const int64_t motion_wall_us =
                        esp_timer_get_time() - motion_start_us;
                    ESP_LOGI(
                        TAG,
                        "motion active window finished: wall=%lldus",
                        static_cast<long long>(motion_wall_us)
                    );
                    motion_active = false;
                }
            }
        }

        vTaskDelayUntil(&last_wake, period_ticks);
    }

    task_handle_ = nullptr;
}

void ArmMotion::update_all_locked(float dt_s)
{
    if (!initialized_ || joints_ == nullptr) {
        return;
    }

    for (ArmJoint joint : kMotionOrder) {
        const int index = joint_to_index(joint);
        if (index < 0) {
            continue;
        }

        const esp_err_t ret = update_one_locked(static_cast<uint8_t>(index), dt_s);
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

esp_err_t ArmMotion::update_one_locked(uint8_t index, float dt_s)
{
    if (index >= kArmJointCount || joints_ == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    MotionJointState& s = states_[index];

    if (!s.configured || !s.enabled || !s.running) {
        return ESP_OK;
    }

    switch (s.timing_mode) {
        case JointTimingMode::Duration:
            return update_duration_locked(s, dt_s);

        case JointTimingMode::Speed:
            return update_speed_locked(s, dt_s);

        case JointTimingMode::Immediate:
            s.running = false;
            return ESP_OK;

        default:
            return ESP_ERR_INVALID_STATE;
    }
}

esp_err_t ArmMotion::update_duration_locked(MotionJointState& s, float dt_s)
{
    const uint16_t previous_us = s.current_us;
    bool reached_final = false;

    s.elapsed_steps++;
    s.elapsed_time_ms += dt_s * 1000.0f;

    const float t = std::clamp(
        s.duration_ms > 0 ?
            s.elapsed_time_ms / static_cast<float>(s.duration_ms) :
            1.0f,
        0.0f,
        1.0f
    );

    if (t >= 1.0f) {
        s.current_us = s.target_us;
        s.current_us_f = s.target_us_f;
        s.running = false;
        reached_final = true;
    } else {
        const float shaped = evaluate_profile(
            s.profile,
            t
        );

        const float delta = s.target_us_f - s.start_us_f;
        const float interpolated = s.start_us_f + delta * shaped;

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
        s.current_us_f = static_cast<float>(s.current_us);
    }

    s.current_position = motion_position_from_us_locked(s.joint, s.current_us);

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

esp_err_t ArmMotion::update_speed_locked(MotionJointState& s, float dt_s)
{
    if (s.max_speed_units_per_s <= kMinSpeedUnitsPerS) {
        return ESP_ERR_INVALID_STATE;
    }

    const uint16_t previous_us = s.current_us;

    const float remaining_signed = s.target_position - s.current_position;
    const float remaining = std::fabs(remaining_signed);

    if (remaining <= kPositionEpsilon) {
        s.current_position = s.target_position;
        s.current_us = s.target_us;
        s.current_us_f = s.target_us_f;
        s.current_speed_units_per_s = 0.0f;
        s.running = false;
    } else {
        const float direction = (remaining_signed >= 0.0f) ? 1.0f : -1.0f;
        const float speed = s.max_speed_units_per_s;
        const float step = speed * dt_s;

        if (step >= remaining) {
            s.current_position = s.target_position;
            s.current_us = s.target_us;
            s.current_us_f = s.target_us_f;
            s.current_speed_units_per_s = 0.0f;
            s.running = false;
        } else {
            s.current_position += direction * step;
            s.current_speed_units_per_s = speed;

            const uint16_t next_us = motion_us_from_position_locked(
                s.joint,
                s.current_position
            );

            s.current_us = next_us;
            s.current_us_f = static_cast<float>(next_us);
        }
    }

    if (s.current_us == previous_us && s.running) {
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
