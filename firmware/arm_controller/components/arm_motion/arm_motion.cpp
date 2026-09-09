#include "arm_motion.hpp"

#include <algorithm>
#include <cmath>

#include "esp_log.h"
#include "esp_timer.h"

namespace learm {

static const char* TAG = "arm_motion_pos_vel";
static constexpr float kMaxDtS = 0.100f;
static constexpr float kMinDtS = 0.001f;
static constexpr float kVelocityEpsilon = 0.001f;
static constexpr float kLimitEpsilonDeg = 0.001f;
static constexpr float kMinClawGapCm = 0.0f;
static constexpr float kMaxClawGapCm = 5.8f;
static constexpr uint32_t kTaskStackSize = 4096;
static constexpr UBaseType_t kTaskPriority = 5;
static constexpr BaseType_t kTaskCore = 1;

static constexpr ArmJoint kStreamJoints[5] = {
    ArmJoint::Base,
    ArmJoint::Shoulder,
    ArmJoint::Elbow,
    ArmJoint::WristPitch,
    ArmJoint::WristRoll,
};

ArmMotion::LockGuard::LockGuard(const ArmMotion* owner, TickType_t timeout_ticks)
    : owner_(owner)
{
    if (owner_ != nullptr && owner_->mutex_ != nullptr) {
        locked_ = xSemaphoreTakeRecursive(owner_->mutex_, timeout_ticks) == pdTRUE;
    }
}

ArmMotion::LockGuard::~LockGuard()
{
    if (locked_ && owner_ != nullptr && owner_->mutex_ != nullptr) {
        xSemaphoreGiveRecursive(owner_->mutex_);
    }
}

bool ArmMotion::LockGuard::locked() const
{
    return locked_;
}

int ArmMotion::stream_index(ArmJoint joint)
{
    switch (joint) {
        case ArmJoint::Base:       return 0;
        case ArmJoint::Shoulder:   return 1;
        case ArmJoint::Elbow:      return 2;
        case ArmJoint::WristPitch: return 3;
        case ArmJoint::WristRoll:  return 4;
        default:                    return -1;
    }
}

ArmMotion::JointStreamState* ArmMotion::stream_for_joint_locked(ArmJoint joint)
{
    const int index = stream_index(joint);
    return index >= 0 ? &streams_[index] : nullptr;
}

const ArmMotion::JointStreamState* ArmMotion::stream_for_joint_locked(ArmJoint joint) const
{
    const int index = stream_index(joint);
    return index >= 0 ? &streams_[index] : nullptr;
}

const ArmMotionJointParameters& ArmMotion::parameters_for_joint(ArmJoint joint) const
{
    switch (joint) {
        case ArmJoint::Base:       return parameters_.base;
        case ArmJoint::Shoulder:   return parameters_.shoulder;
        case ArmJoint::Elbow:      return parameters_.elbow;
        case ArmJoint::WristPitch: return parameters_.wrist_pitch;
        case ArmJoint::WristRoll:  return parameters_.wrist_roll;
        default:                    return parameters_.base;
    }
}

bool ArmMotion::finite_position(const ArmMotionJointPosition& position)
{
    return std::isfinite(position.base_deg) &&
           std::isfinite(position.shoulder_deg) &&
           std::isfinite(position.elbow_deg) &&
           std::isfinite(position.wrist_pitch_deg) &&
           std::isfinite(position.wrist_roll_deg);
}

bool ArmMotion::finite_velocity(const ArmMotionJointVelocity& velocity)
{
    return std::isfinite(velocity.base_deg_per_s) &&
           std::isfinite(velocity.shoulder_deg_per_s) &&
           std::isfinite(velocity.elbow_deg_per_s) &&
           std::isfinite(velocity.wrist_pitch_deg_per_s) &&
           std::isfinite(velocity.wrist_roll_deg_per_s);
}

float ArmMotion::approach(float current, float target, float max_change)
{
    if (max_change <= 0.0f) {
        return target;
    }
    return current + std::clamp(target - current, -max_change, max_change);
}

esp_err_t ArmMotion::init(ArmJointController* joints)
{
    if (joints == nullptr || !joints->is_initialized()) {
        return ESP_ERR_INVALID_ARG;
    }
    if (initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    mutex_ = xSemaphoreCreateRecursiveMutex();
    if (mutex_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    joints_ = joints;
    for (int i = 0; i < 5; ++i) {
        streams_[i].joint = kStreamJoints[i];
        streams_[i].configured = joints_->is_configured(kStreamJoints[i]);
        if (!streams_[i].configured) {
            ESP_LOGE(TAG, "joint not configured index=%d", i);
            vSemaphoreDelete(mutex_);
            mutex_ = nullptr;
            joints_ = nullptr;
            return ESP_ERR_INVALID_STATE;
        }
    }

    claw_reference_gap_cm_ = joints_->read_claw_gap_cm();
    claw_target_gap_cm_ = claw_reference_gap_cm_;
    latest_position_command_.claw_gap_cm = claw_target_gap_cm_;
    latest_velocity_command_.claw_gap_cm = claw_target_gap_cm_;
    output_state_ = ArmOutputState::Disabled;
    clear_driver_error_locked();
    initialized_ = true;

    const BaseType_t created = xTaskCreatePinnedToCore(
        task_entry,
        "arm_pos_vel",
        kTaskStackSize,
        this,
        kTaskPriority,
        &task_handle_,
        kTaskCore
    );

    if (created != pdPASS) {
        initialized_ = false;
        joints_ = nullptr;
        task_handle_ = nullptr;
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "position + streaming velocity initialized period=%lums",
             static_cast<unsigned long>(parameters_.update_period_ms));
    return ESP_OK;
}

bool ArmMotion::is_initialized() const
{
    return initialized_;
}

esp_err_t ArmMotion::sync_streams_from_joint_layer_locked()
{
    if (joints_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    for (JointStreamState& stream : streams_) {
        const JointRuntimeState state = joints_->get_state(stream.joint);
        if (!state.configured || !state.enabled) {
            return ESP_ERR_INVALID_STATE;
        }

        stream.reference_deg = state.command_deg;
        stream.target_position_deg = state.command_deg;
        stream.target_velocity_deg_per_s = 0.0f;
        stream.applied_velocity_deg_per_s = 0.0f;
        stream.enabled = true;
        stream.reference_reached = true;
        stream.limit_blocked = false;
    }

    claw_reference_gap_cm_ = joints_->read_claw_gap_cm();
    claw_target_gap_cm_ = claw_reference_gap_cm_;
    latest_position_command_ = {};
    latest_velocity_command_ = {};
    latest_position_command_.joint_position.base_deg = streams_[0].reference_deg;
    latest_position_command_.joint_position.shoulder_deg = streams_[1].reference_deg;
    latest_position_command_.joint_position.elbow_deg = streams_[2].reference_deg;
    latest_position_command_.joint_position.wrist_pitch_deg = streams_[3].reference_deg;
    latest_position_command_.joint_position.wrist_roll_deg = streams_[4].reference_deg;
    latest_position_command_.claw_gap_cm = claw_target_gap_cm_;
    latest_velocity_command_.claw_gap_cm = claw_target_gap_cm_;
    mode_ = ArmMotionMode::Hold;
    last_command_us_ = 0;
    command_received_ = false;
    command_timed_out_ = false;
    return ESP_OK;
}

esp_err_t ArmMotion::enable_all_at_zero_closed()
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }
    if (!initialized_ || joints_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    ArmMotionPositionCommand initial_pose;
    initial_pose.joint_position.base_deg =
        joints_->get_calibration(ArmJoint::Base).zero_deg;
    initial_pose.joint_position.shoulder_deg =
        joints_->get_calibration(ArmJoint::Shoulder).zero_deg;
    initial_pose.joint_position.elbow_deg =
        joints_->get_calibration(ArmJoint::Elbow).zero_deg;
    initial_pose.joint_position.wrist_pitch_deg =
        joints_->get_calibration(ArmJoint::WristPitch).zero_deg;
    initial_pose.joint_position.wrist_roll_deg =
        joints_->get_calibration(ArmJoint::WristRoll).zero_deg;
    initial_pose.claw_gap_cm = 0.0f;

    return enable_all_at_locked(initial_pose);
}

esp_err_t ArmMotion::enable_all_at(
    const ArmMotionPositionCommand& initial_pose
)
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }

    return enable_all_at_locked(initial_pose);
}

esp_err_t ArmMotion::enable_all_at_locked(
    const ArmMotionPositionCommand& initial_pose
)
{
    if (!initialized_ || joints_ == nullptr ||
        output_state_ != ArmOutputState::Disabled) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!finite_position(initial_pose.joint_position) ||
        !std::isfinite(initial_pose.claw_gap_cm) ||
        initial_pose.claw_gap_cm < kMinClawGapCm ||
        initial_pose.claw_gap_cm > kMaxClawGapCm) {
        return ESP_ERR_INVALID_ARG;
    }

    const float start_deg[5] = {
        initial_pose.joint_position.base_deg,
        initial_pose.joint_position.shoulder_deg,
        initial_pose.joint_position.elbow_deg,
        initial_pose.joint_position.wrist_pitch_deg,
        initial_pose.joint_position.wrist_roll_deg,
    };

    for (int i = 0; i < 5; ++i) {
        const JointCalibration calibration =
            joints_->get_calibration(kStreamJoints[i]);
        if (start_deg[i] < calibration.min_deg ||
            start_deg[i] > calibration.max_deg) {
            return ESP_ERR_INVALID_ARG;
        }
    }

    auto cleanup_after_failure = [this](
        esp_err_t cause,
        ArmJoint failed_joint,
        bool joint_valid
    ) {
        const esp_err_t cleanup_ret = joints_->disable_all();
        for (JointStreamState& stream : streams_) {
            stream.enabled = joints_->get_state(stream.joint).enabled;
        }

        last_driver_error_ = cleanup_ret != ESP_OK ? cleanup_ret : cause;
        driver_error_joint_valid_ = cleanup_ret == ESP_OK && joint_valid;
        driver_error_joint_ = failed_joint;
        output_state_ = cleanup_ret == ESP_OK
            ? ArmOutputState::Disabled
            : ArmOutputState::DriverError;

        if (cleanup_ret != ESP_OK) {
            ESP_LOGE(
                TAG,
                "enable cleanup failed cause=0x%x cleanup=0x%x",
                static_cast<unsigned>(cause),
                static_cast<unsigned>(cleanup_ret)
            );
        }
        return cause;
    };

    for (int i = 0; i < 5; ++i) {
        const esp_err_t ret =
            joints_->enable_joint(kStreamJoints[i], start_deg[i]);
        if (ret != ESP_OK) {
            return cleanup_after_failure(ret, kStreamJoints[i], true);
        }
    }

    const esp_err_t claw_ret =
        joints_->enable_claw_at_gap(initial_pose.claw_gap_cm);
    if (claw_ret != ESP_OK) {
        return cleanup_after_failure(claw_ret, ArmJoint::Claw, true);
    }

    const esp_err_t sync_ret = sync_streams_from_joint_layer_locked();
    if (sync_ret != ESP_OK) {
        return cleanup_after_failure(sync_ret, ArmJoint::Base, false);
    }

    output_state_ = ArmOutputState::Enabled;
    clear_driver_error_locked();
    ESP_LOGI(TAG, "all actuators enabled at explicit initial pose; entering Hold");
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

    enter_hold_locked();
    const esp_err_t ret = joints_->disable_all();
    if (ret != ESP_OK) {
        output_state_ = ArmOutputState::DriverError;
        last_driver_error_ = ret;
        driver_error_joint_valid_ = false;
        for (JointStreamState& stream : streams_) {
            stream.enabled = joints_->get_state(stream.joint).enabled;
        }
        return ret;
    }

    output_state_ = ArmOutputState::Disabled;
    clear_driver_error_locked();
    command_received_ = false;
    command_timed_out_ = false;
    last_command_us_ = 0;
    for (JointStreamState& stream : streams_) {
        stream.enabled = false;
    }
    ESP_LOGI(TAG, "all joints disabled");
    return ESP_OK;
}

void ArmMotion::set_position_target_locked(const ArmMotionJointPosition& position)
{
    const float values[5] = {
        position.base_deg,
        position.shoulder_deg,
        position.elbow_deg,
        position.wrist_pitch_deg,
        position.wrist_roll_deg,
    };

    for (int i = 0; i < 5; ++i) {
        JointStreamState& stream = streams_[i];
        const JointCalibration calibration = joints_->get_calibration(stream.joint);
        stream.target_position_deg = std::clamp(
            values[i],
            calibration.min_deg,
            calibration.max_deg
        );
        stream.reference_reached =
            std::fabs(stream.target_position_deg - stream.reference_deg) <=
            parameters_.position_arrival_epsilon_deg;
        stream.limit_blocked = false;
    }
}

void ArmMotion::set_target_velocity_locked(const ArmMotionJointVelocity& velocity)
{
    const float values[5] = {
        velocity.base_deg_per_s,
        velocity.shoulder_deg_per_s,
        velocity.elbow_deg_per_s,
        velocity.wrist_pitch_deg_per_s,
        velocity.wrist_roll_deg_per_s,
    };

    for (int i = 0; i < 5; ++i) {
        JointStreamState& stream = streams_[i];
        const ArmMotionJointParameters& p = parameters_for_joint(stream.joint);
        stream.target_velocity_deg_per_s = std::clamp(
            values[i],
            -std::fabs(p.max_velocity_deg_per_s),
             std::fabs(p.max_velocity_deg_per_s)
        );
        stream.target_position_deg = stream.reference_deg;
        stream.reference_reached = true;
        stream.limit_blocked = false;
    }
}

void ArmMotion::clear_velocity_locked()
{
    latest_velocity_command_.joint_velocity = {};
    for (JointStreamState& stream : streams_) {
        stream.target_velocity_deg_per_s = 0.0f;
        stream.limit_blocked = false;
    }
}

void ArmMotion::start_stopping_locked()
{
    clear_velocity_locked();

    bool reference_moving = false;
    for (JointStreamState& stream : streams_) {
        stream.target_position_deg = stream.reference_deg;
        stream.reference_reached =
            std::fabs(stream.applied_velocity_deg_per_s) <= kVelocityEpsilon;
        reference_moving |= !stream.reference_reached;
    }

    if (reference_moving) {
        mode_ = ArmMotionMode::Stopping;
    } else {
        enter_hold_locked();
    }
}

void ArmMotion::enter_hold_locked()
{
    clear_velocity_locked();
    mode_ = ArmMotionMode::Hold;
    for (JointStreamState& stream : streams_) {
        stream.target_position_deg = stream.reference_deg;
        stream.applied_velocity_deg_per_s = 0.0f;
        stream.reference_reached = true;
    }
}

void ArmMotion::record_driver_error_locked(
    esp_err_t error,
    ArmJoint joint,
    bool joint_valid
)
{
    enter_hold_locked();
    output_state_ = ArmOutputState::DriverError;
    last_driver_error_ = error;
    driver_error_joint_valid_ = joint_valid;
    driver_error_joint_ = joint;
    command_received_ = false;
}

void ArmMotion::clear_driver_error_locked()
{
    last_driver_error_ = ESP_OK;
    driver_error_joint_valid_ = false;
    driver_error_joint_ = ArmJoint::Base;
}

esp_err_t ArmMotion::move_to(const ArmMotionPositionCommand& command_value)
{
    if (!finite_position(command_value.joint_position) ||
        !std::isfinite(command_value.claw_gap_cm)) {
        return ESP_ERR_INVALID_ARG;
    }

    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }
    if (!initialized_ || output_state_ != ArmOutputState::Enabled ||
        joints_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    set_position_target_locked(command_value.joint_position);
    latest_position_command_ = command_value;
    const float requested_gap = std::clamp(
        command_value.claw_gap_cm,
        kMinClawGapCm,
        kMaxClawGapCm
    );
    latest_position_command_.claw_gap_cm = requested_gap;
    claw_target_gap_cm_ = requested_gap;

    mode_ = ArmMotionMode::Position;
    command_received_ = false;
    command_timed_out_ = false;
    last_command_us_ = 0;
    return ESP_OK;
}

esp_err_t ArmMotion::command(const ArmMotionCommand& command_value)
{
    if (!finite_velocity(command_value.joint_velocity) ||
        !std::isfinite(command_value.claw_gap_cm)) {
        return ESP_ERR_INVALID_ARG;
    }

    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }
    if (!initialized_ || output_state_ != ArmOutputState::Enabled ||
        joints_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    set_target_velocity_locked(command_value.joint_velocity);
    latest_velocity_command_ = command_value;
    last_command_us_ = esp_timer_get_time();
    command_received_ = true;
    command_timed_out_ = false;
    mode_ = ArmMotionMode::Velocity;

    const float requested_gap = std::clamp(
        command_value.claw_gap_cm,
        kMinClawGapCm,
        kMaxClawGapCm
    );
    latest_velocity_command_.claw_gap_cm = requested_gap;
    claw_target_gap_cm_ = requested_gap;

    return ESP_OK;
}

esp_err_t ArmMotion::stop()
{
    const LockGuard guard(this);
    if (!guard.locked()) {
        return ESP_ERR_TIMEOUT;
    }
    if (!initialized_ || output_state_ != ArmOutputState::Enabled) {
        return ESP_ERR_INVALID_STATE;
    }

    start_stopping_locked();
    command_received_ = false;
    command_timed_out_ = false;
    last_command_us_ = 0;
    return ESP_OK;
}

void ArmMotion::prepare_position_velocity_locked(JointStreamState& state)
{
    const float error = state.target_position_deg - state.reference_deg;
    const float abs_error = std::fabs(error);

    if (abs_error <= parameters_.position_arrival_epsilon_deg &&
        std::fabs(state.applied_velocity_deg_per_s) <= kVelocityEpsilon) {
        state.reference_deg = state.target_position_deg;
        state.target_velocity_deg_per_s = 0.0f;
        state.applied_velocity_deg_per_s = 0.0f;
        state.reference_reached = true;
        return;
    }

    const ArmMotionJointParameters& p = parameters_for_joint(state.joint);
    const float max_velocity = std::fabs(p.max_velocity_deg_per_s);
    const float acceleration = std::fabs(p.max_acceleration_deg_per_s2);

    // Velocity envelope required to stop at the target under the configured
    // reference acceleration. This yields a compact trapezoidal/triangular
    // setup move without introducing another profile subsystem.
    const float stopping_velocity = acceleration > 0.0f
        ? std::sqrt(std::max(0.0f, 2.0f * acceleration * abs_error))
        : max_velocity;
    const float desired_speed = std::min(max_velocity, stopping_velocity);

    state.target_velocity_deg_per_s = std::copysign(desired_speed, error);
    state.reference_reached = false;
}

esp_err_t ArmMotion::write_stream_joint_locked(
    JointStreamState& state,
    float dt_s,
    bool clamp_to_position_target
)
{
    if (joints_ == nullptr || !state.enabled) {
        return ESP_ERR_INVALID_STATE;
    }

    JointStreamState next = state;
    const ArmMotionJointParameters& p = parameters_for_joint(next.joint);
    const JointCalibration calibration = joints_->get_calibration(next.joint);

    const float previous_reference = next.reference_deg;
    const float accel_step = std::fabs(p.max_acceleration_deg_per_s2) * dt_s;
    next.applied_velocity_deg_per_s = approach(
        next.applied_velocity_deg_per_s,
        next.target_velocity_deg_per_s,
        accel_step
    );

    const bool at_min = next.reference_deg <= calibration.min_deg + kLimitEpsilonDeg;
    const bool at_max = next.reference_deg >= calibration.max_deg - kLimitEpsilonDeg;

    if ((at_min && next.applied_velocity_deg_per_s < 0.0f) ||
        (at_max && next.applied_velocity_deg_per_s > 0.0f)) {
        next.applied_velocity_deg_per_s = 0.0f;
        next.target_velocity_deg_per_s = 0.0f;
        next.limit_blocked = true;
    }

    float next_reference = next.reference_deg +
                           next.applied_velocity_deg_per_s * dt_s;

    if (clamp_to_position_target) {
        const float error_before = next.target_position_deg - previous_reference;
        const float error_after = next.target_position_deg - next_reference;
        const bool crossed_target =
            (error_before > 0.0f && error_after <= 0.0f) ||
            (error_before < 0.0f && error_after >= 0.0f);
        if (crossed_target ||
            std::fabs(error_after) <= parameters_.position_arrival_epsilon_deg) {
            next_reference = next.target_position_deg;
            next.target_velocity_deg_per_s = 0.0f;
            next.applied_velocity_deg_per_s = 0.0f;
            next.reference_reached = true;
        }
    }

    if (next_reference < calibration.min_deg) {
        next_reference = calibration.min_deg;
        next.applied_velocity_deg_per_s = 0.0f;
        next.target_velocity_deg_per_s = 0.0f;
        next.limit_blocked = true;
        next.reference_reached = clamp_to_position_target;
    } else if (next_reference > calibration.max_deg) {
        next_reference = calibration.max_deg;
        next.applied_velocity_deg_per_s = 0.0f;
        next.target_velocity_deg_per_s = 0.0f;
        next.limit_blocked = true;
        next.reference_reached = clamp_to_position_target;
    }

    next.reference_deg = next_reference;
    if (!clamp_to_position_target) {
        next.reference_reached =
            std::fabs(next.applied_velocity_deg_per_s) <= kVelocityEpsilon;
    }

    const esp_err_t ret =
        joints_->write_deg_now(next.joint, next.reference_deg);
    if (ret != ESP_OK) {
        return ret;
    }

    state = next;
    return ESP_OK;
}

void ArmMotion::update_locked(float dt_s)
{
    if (!initialized_ || output_state_ != ArmOutputState::Enabled ||
        joints_ == nullptr) {
        return;
    }

    if (mode_ == ArmMotionMode::Velocity && command_received_ && !command_timed_out_) {
        const int64_t age_us = esp_timer_get_time() - last_command_us_;
        const int64_t timeout_us =
            static_cast<int64_t>(parameters_.command_timeout_ms) * 1000;
        if (age_us > timeout_us) {
            start_stopping_locked();
            command_timed_out_ = true;
            ESP_LOGW(TAG, "velocity command timeout; entering Stopping");
        }
    }

    if (mode_ == ArmMotionMode::Position) {
        for (JointStreamState& stream : streams_) {
            prepare_position_velocity_locked(stream);
        }
    }

    bool all_position_reached = mode_ == ArmMotionMode::Position;
    bool all_references_stopped = mode_ == ArmMotionMode::Stopping;

    if (mode_ == ArmMotionMode::Velocity) {
        // A Cartesian Jacobian solution is one coordinated joint-velocity
        // vector. If one joint reaches a limit, zeroing only that component
        // changes the vector and can reverse the actual tool motion. Preserve
        // the vector direction by applying one common scale to every joint.
        float candidate_velocity[5] = {};
        bool constrains_scale[5] = {};
        float group_scale = 1.0f;

        for (int i = 0; i < 5; ++i) {
            JointStreamState& stream = streams_[i];
            stream.limit_blocked = false;

            const ArmMotionJointParameters& p =
                parameters_for_joint(stream.joint);
            const float accel_step =
                std::fabs(p.max_acceleration_deg_per_s2) * dt_s;
            candidate_velocity[i] = approach(
                stream.applied_velocity_deg_per_s,
                stream.target_velocity_deg_per_s,
                accel_step
            );

            const JointCalibration calibration =
                joints_->get_calibration(stream.joint);
            const float requested_step = candidate_velocity[i] * dt_s;
            float joint_scale = 1.0f;

            if (requested_step > 0.0f) {
                const float remaining =
                    calibration.max_deg - stream.reference_deg;
                if (remaining <= kLimitEpsilonDeg) {
                    joint_scale = 0.0f;
                } else if (requested_step > remaining) {
                    joint_scale = remaining / requested_step;
                }
            } else if (requested_step < 0.0f) {
                const float remaining =
                    stream.reference_deg - calibration.min_deg;
                if (remaining <= kLimitEpsilonDeg) {
                    joint_scale = 0.0f;
                } else if (-requested_step > remaining) {
                    joint_scale = remaining / -requested_step;
                }
            }

            joint_scale = std::clamp(joint_scale, 0.0f, 1.0f);
            constrains_scale[i] = joint_scale < 1.0f;
            group_scale = std::min(group_scale, joint_scale);
        }

        group_scale = std::clamp(group_scale, 0.0f, 1.0f);

        for (int i = 0; i < 5; ++i) {
            JointStreamState next = streams_[i];
            const JointCalibration calibration =
                joints_->get_calibration(next.joint);

            next.applied_velocity_deg_per_s =
                candidate_velocity[i] * group_scale;
            next.limit_blocked = constrains_scale[i];

            next.reference_deg = std::clamp(
                next.reference_deg +
                    next.applied_velocity_deg_per_s * dt_s,
                calibration.min_deg,
                calibration.max_deg
            );
            next.target_position_deg = next.reference_deg;
            next.reference_reached = true;

            const esp_err_t ret =
                joints_->write_deg_now(next.joint, next.reference_deg);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "stream write failed joint=%u ret=0x%x",
                         static_cast<unsigned>(next.joint),
                         static_cast<unsigned>(ret));
                record_driver_error_locked(ret, next.joint, true);
                return;
            }

            streams_[i] = next;
        }
    } else if (mode_ == ArmMotionMode::Position ||
               mode_ == ArmMotionMode::Stopping) {
        for (JointStreamState& stream : streams_) {
            const esp_err_t ret = write_stream_joint_locked(
                stream,
                dt_s,
                mode_ == ArmMotionMode::Position
            );
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "stream write failed joint=%u ret=0x%x",
                         static_cast<unsigned>(stream.joint),
                         static_cast<unsigned>(ret));
                record_driver_error_locked(ret, stream.joint, true);
                return;
            }
            if (mode_ == ArmMotionMode::Position &&
                !stream.reference_reached) {
                all_position_reached = false;
            }
            if (mode_ == ArmMotionMode::Stopping &&
                !stream.reference_reached) {
                all_references_stopped = false;
            }
        }
    }

    if (all_position_reached) {
        enter_hold_locked();
        ESP_LOGI(TAG, "position target reached; entering Hold");
    } else if (all_references_stopped) {
        enter_hold_locked();
        ESP_LOGI(TAG, "reference velocities stopped; entering Hold");
    }

    const float claw_error = claw_target_gap_cm_ - claw_reference_gap_cm_;
    float next_claw_reference_gap_cm = claw_reference_gap_cm_;
    if (std::fabs(claw_error) > parameters_.claw_arrival_epsilon_cm) {
        const float max_step = parameters_.claw_max_speed_cm_per_s * dt_s;
        next_claw_reference_gap_cm +=
            std::clamp(claw_error, -max_step, max_step);
    } else {
        next_claw_reference_gap_cm = claw_target_gap_cm_;
    }

    if (std::fabs(next_claw_reference_gap_cm - claw_reference_gap_cm_) >
        0.000001f) {
        const uint16_t claw_us =
            joints_->claw_gap_cm_to_us(next_claw_reference_gap_cm);
        const esp_err_t claw_ret =
            joints_->write_us_now(ArmJoint::Claw, claw_us);
        if (claw_ret != ESP_OK) {
            ESP_LOGE(TAG, "claw write failed ret=0x%x",
                     static_cast<unsigned>(claw_ret));
            record_driver_error_locked(claw_ret, ArmJoint::Claw, true);
            return;
        }
        claw_reference_gap_cm_ = next_claw_reference_gap_cm;
    }
}

void ArmMotion::task_entry(void* arg)
{
    ArmMotion* self = static_cast<ArmMotion*>(arg);
    if (self != nullptr) {
        self->task_loop();
    }
    vTaskDelete(nullptr);
}

void ArmMotion::task_loop()
{
    TickType_t last_wake = xTaskGetTickCount();
    int64_t previous_us = esp_timer_get_time();

    while (!task_stop_requested_) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(parameters_.update_period_ms));

        const int64_t now_us = esp_timer_get_time();
        float dt_s = static_cast<float>(now_us - previous_us) * 1.0e-6f;
        previous_us = now_us;
        dt_s = std::clamp(dt_s, kMinDtS, kMaxDtS);

        const LockGuard guard(this, pdMS_TO_TICKS(5));
        if (guard.locked()) {
            update_locked(dt_s);
        }
    }

    task_handle_ = nullptr;
}

ArmMotionState ArmMotion::get_state() const
{
    ArmMotionState result = {};
    const LockGuard guard(this);
    if (!guard.locked()) {
        return result;
    }

    result.initialized = initialized_;
    result.output_state = output_state_;
    result.mode = mode_;
    result.last_driver_error = last_driver_error_;
    result.driver_error_joint_valid = driver_error_joint_valid_;
    result.driver_error_joint = driver_error_joint_;
    result.latest_position_command = latest_position_command_;
    result.latest_velocity_command = latest_velocity_command_;
    result.parameters = parameters_;
    result.command_timed_out = command_timed_out_;
    result.command_fresh =
        mode_ == ArmMotionMode::Velocity && command_received_ && !command_timed_out_;
    if (command_received_ && last_command_us_ > 0) {
        const int64_t age_us = std::max<int64_t>(
            0,
            esp_timer_get_time() - last_command_us_
        );
        result.command_age_ms = static_cast<uint32_t>(age_us / 1000);
    }

    auto copy_state = [](const JointStreamState& source) {
        ArmMotionReferenceState out;
        out.reference_deg = source.reference_deg;
        out.target_position_deg = source.target_position_deg;
        out.target_velocity_deg_per_s = source.target_velocity_deg_per_s;
        out.applied_velocity_deg_per_s = source.applied_velocity_deg_per_s;
        out.reference_reached = source.reference_reached;
        out.limit_blocked = source.limit_blocked;
        return out;
    };

    result.base = copy_state(streams_[0]);
    result.shoulder = copy_state(streams_[1]);
    result.elbow = copy_state(streams_[2]);
    result.wrist_pitch = copy_state(streams_[3]);
    result.wrist_roll = copy_state(streams_[4]);

    result.claw_reference_gap_cm = claw_reference_gap_cm_;
    result.claw_target_gap_cm = claw_target_gap_cm_;

    result.reference_moving = false;
    if (output_state_ == ArmOutputState::Enabled) {
        for (const JointStreamState& stream : streams_) {
            if (std::fabs(stream.target_velocity_deg_per_s) > kVelocityEpsilon ||
                std::fabs(stream.applied_velocity_deg_per_s) > kVelocityEpsilon ||
                (mode_ == ArmMotionMode::Position &&
                 !stream.reference_reached)) {
                result.reference_moving = true;
                break;
            }
        }
        if (std::fabs(claw_target_gap_cm_ - claw_reference_gap_cm_) >
            parameters_.claw_arrival_epsilon_cm) {
            result.reference_moving = true;
        }
    }

    return result;
}

ArmMotionParameters ArmMotion::get_parameters() const
{
    const LockGuard guard(this);
    return guard.locked() ? parameters_ : ArmMotionParameters{};
}

}  // namespace learm
