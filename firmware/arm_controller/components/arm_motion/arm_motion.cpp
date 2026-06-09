#include "arm_motion.hpp"

#include "esp_log.h"

namespace learm {

static const char* TAG = "arm_motion";

esp_err_t ArmMotion::init(ArmJointController* joints)
{
    if (joints == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!joints->is_initialized()) {
        ESP_LOGE(TAG, "init rejected: arm_joint is not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    joints_ = joints;
    initialized_ = true;

    ESP_LOGI(TAG, "arm_motion initialized");

    return ESP_OK;
}

bool ArmMotion::is_initialized() const
{
    return initialized_;
}

esp_err_t ArmMotion::enable_all()
{
    if (!initialized_ || joints_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    // 固定顺序：根部到末端。
    // enable_joint() 是初始化附着，不经过插补。
    const ArmJoint joints[] = {
        ArmJoint::Base,
        ArmJoint::Shoulder,
        ArmJoint::Elbow,
        ArmJoint::WristPitch,
        ArmJoint::WristRoll,
        ArmJoint::Claw,
    };

    for (ArmJoint joint : joints) {
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
    }

    ESP_LOGI(TAG, "all joints enabled");
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
    return durations_ms_;
}

ArmMotionStrategies ArmMotion::get_strategies() const
{
    return strategies_;
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

    // 第一版 arm_motion 不向上层暴露底层 PWM 步进细节。
    options.max_step_us = 0;
    options.min_effective_step_us = 0;

    return options;
}

esp_err_t ArmMotion::move_one_deg(
    ArmJoint joint,
    float target_deg,
    uint32_t duration_ms,
    JointMotionStyle style
)
{
    if (!initialized_ || joints_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    const JointMotionOptions options =
        make_options(duration_ms, style);

    return joints_->move_deg(
        joint,
        target_deg,
        options
    );
}

esp_err_t ArmMotion::move_claw_gap(
    float target_gap_cm,
    uint32_t duration_ms,
    JointMotionStyle style
)
{
    if (!initialized_ || joints_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    const JointMotionOptions options =
        make_options(duration_ms, style);

    return joints_->move_claw_gap_cm(
        target_gap_cm,
        options
    );
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

esp_err_t ArmMotion::move_to(const ArmMotionTarget& target)
{
    if (!initialized_ || joints_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    // 固定提交顺序：根部到末端。
    // 这些调用只提交目标，实际插补由 servo_pwm 的 20ms 定时器执行。
    esp_err_t ret = ESP_OK;

    ret = move_one_deg(
        ArmJoint::Base,
        target.base_deg,
        durations_ms_.base_ms,
        strategies_.base
    );
    if (ret != ESP_OK) {
        return ret;
    }

    ret = move_one_deg(
        ArmJoint::Shoulder,
        target.shoulder_deg,
        durations_ms_.shoulder_ms,
        strategies_.shoulder
    );
    if (ret != ESP_OK) {
        return ret;
    }

    ret = move_one_deg(
        ArmJoint::Elbow,
        target.elbow_deg,
        durations_ms_.elbow_ms,
        strategies_.elbow
    );
    if (ret != ESP_OK) {
        return ret;
    }

    ret = move_one_deg(
        ArmJoint::WristPitch,
        target.wrist_pitch_deg,
        durations_ms_.wrist_pitch_ms,
        strategies_.wrist_pitch
    );
    if (ret != ESP_OK) {
        return ret;
    }

    ret = move_one_deg(
        ArmJoint::WristRoll,
        target.wrist_roll_deg,
        durations_ms_.wrist_roll_ms,
        strategies_.wrist_roll
    );
    if (ret != ESP_OK) {
        return ret;
    }

    ret = move_claw_gap(
        target.claw_gap_cm,
        durations_ms_.claw_ms,
        strategies_.claw
    );
    if (ret != ESP_OK) {
        return ret;
    }

    ESP_LOGI(
        TAG,
        "move_to base=%.2f shoulder=%.2f elbow=%.2f wrist_pitch=%.2f wrist_roll=%.2f claw_gap=%.2fcm",
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
    if (!initialized_ || joints_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    return joints_->stop_all();
}

bool ArmMotion::is_ready() const
{
    if (!initialized_ || joints_ == nullptr) {
        return false;
    }

    return joints_->is_all_ready();
}

bool ArmMotion::is_moving() const
{
    if (!initialized_ || joints_ == nullptr) {
        return false;
    }

    return !joints_->is_all_ready();
}

ArmMotionState ArmMotion::get_state() const
{
    ArmMotionState state = {};

    state.initialized = initialized_;
    state.durations_ms = durations_ms_;
    state.strategies = strategies_;

    if (!initialized_ || joints_ == nullptr) {
        state.ready = false;
        state.moving = false;
        return state;
    }

    state.base = joints_->get_state(ArmJoint::Base);
    state.shoulder = joints_->get_state(ArmJoint::Shoulder);
    state.elbow = joints_->get_state(ArmJoint::Elbow);
    state.wrist_pitch = joints_->get_state(ArmJoint::WristPitch);
    state.wrist_roll = joints_->get_state(ArmJoint::WristRoll);
    state.claw = joints_->get_state(ArmJoint::Claw);
    state.claw_gap_cm = joints_->read_claw_gap_cm();
    state.claw_target_gap_cm = joints_->read_claw_target_gap_cm();

    state.ready = joints_->is_all_ready();
    state.moving = !state.ready;

    return state;
}

}  // namespace learm
