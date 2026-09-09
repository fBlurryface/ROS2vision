#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "arm_joint.hpp"

namespace learm {

// Rotary-reference generator mode. Claw gap follows its own finite target.
enum class ArmMotionMode : uint8_t {
    Hold = 0,
    Position,
    Velocity,
    Stopping,
};

// State of the software-controlled PWM output path. The arm has no actuator
// feedback, so none of these values confirms servo power, motion, or position.
enum class ArmOutputState : uint8_t {
    Disabled = 0,
    Enabled,
    DriverError,
};

// Five rotary-joint absolute position references.
struct ArmMotionJointPosition {
    float base_deg = 0.0f;
    float shoulder_deg = 0.0f;
    float elbow_deg = 0.0f;
    float wrist_pitch_deg = 0.0f;
    float wrist_roll_deg = 0.0f;
};

// Five rotary-joint reference velocities. These are software reference rates,
// not measured servo shaft velocities.
struct ArmMotionJointVelocity {
    float base_deg_per_s = 0.0f;
    float shoulder_deg_per_s = 0.0f;
    float elbow_deg_per_s = 0.0f;
    float wrist_pitch_deg_per_s = 0.0f;
    float wrist_roll_deg_per_s = 0.0f;
};

// Basic finite position command used to place the arm before an experiment.
// It shares the same velocity and acceleration limits as velocity mode.
struct ArmMotionPositionCommand {
    ArmMotionJointPosition joint_position = {};
    float claw_gap_cm = 0.0f;
};

// Streaming velocity command used by the visual-servo experiment. The claw
// keeps the finite target-gap behavior.
struct ArmMotionCommand {
    ArmMotionJointVelocity joint_velocity = {};
    float claw_gap_cm = 0.0f;
};

struct ArmMotionJointParameters {
    float max_velocity_deg_per_s = 30.0f;
    float max_acceleration_deg_per_s2 = 120.0f;
};

// One fixed parameter group for basic positioning and streaming velocity.
// There are deliberately no runtime tuning setters in this experiment.
struct ArmMotionParameters {
    ArmMotionJointParameters base = {75.0f, 420.0f};
    ArmMotionJointParameters shoulder = {70.0f, 360.0f};
    ArmMotionJointParameters elbow = {70.0f, 360.0f};
    ArmMotionJointParameters wrist_pitch = {85.0f, 450.0f};
    ArmMotionJointParameters wrist_roll = {60.0f, 300.0f};

    uint32_t update_period_ms = 20;

    // Applies only to streaming velocity mode.
    uint32_t command_timeout_ms = 300;

    // Position mode stops when the software reference reaches this tolerance.
    float position_arrival_epsilon_deg = 0.05f;

    // Legacy claw behavior retained as a target-gap speed line.
    float claw_max_speed_cm_per_s = 2.5f;
    float claw_arrival_epsilon_cm = 0.01f;
};

struct ArmMotionReferenceState {
    float reference_deg = 0.0f;
    float target_position_deg = 0.0f;
    float target_velocity_deg_per_s = 0.0f;
    float applied_velocity_deg_per_s = 0.0f;
    bool reference_reached = true;
    bool limit_blocked = false;
};

struct ArmMotionState {
    bool initialized = false;
    ArmOutputState output_state = ArmOutputState::Disabled;
    bool reference_moving = false;
    ArmMotionMode mode = ArmMotionMode::Hold;

    // Last MCPWM/GPIO driver error. A non-OK value does not indicate a servo
    // fault; it only describes the software output path.
    esp_err_t last_driver_error = ESP_OK;
    bool driver_error_joint_valid = false;
    ArmJoint driver_error_joint = ArmJoint::Base;

    bool command_fresh = false;
    bool command_timed_out = false;
    uint32_t command_age_ms = 0;

    ArmMotionReferenceState base = {};
    ArmMotionReferenceState shoulder = {};
    ArmMotionReferenceState elbow = {};
    ArmMotionReferenceState wrist_pitch = {};
    ArmMotionReferenceState wrist_roll = {};

    float claw_reference_gap_cm = 0.0f;
    float claw_target_gap_cm = 0.0f;

    ArmMotionPositionCommand latest_position_command = {};
    ArmMotionCommand latest_velocity_command = {};
    ArmMotionParameters parameters = {};
};

class ArmMotion {
public:
    esp_err_t init(ArmJointController* joints);

    bool is_initialized() const;

    // Enable all rotary joints at their calibration zero and close the claw.
    esp_err_t enable_all_at_zero_closed();

    // Enable all actuators using one explicit initial command. The resulting
    // PWM commands become the initial software references.
    esp_err_t enable_all_at(const ArmMotionPositionCommand& initial_pose);

    esp_err_t disable_all();

    // Basic absolute-position mode for setup and pose adjustment. The command
    // automatically completes and then transitions to Hold.
    esp_err_t move_to(const ArmMotionPositionCommand& command);

    // Streaming velocity mode. The velocity remains active until replaced,
    // stop() is called, a joint limit blocks it, timeout occurs, or positioning
    // mode explicitly takes control.
    esp_err_t command(const ArmMotionCommand& command);

    // Enter Stopping, ramp rotary reference velocity to zero, then enter Hold.
    esp_err_t stop();

    ArmMotionState get_state() const;
    ArmMotionParameters get_parameters() const;

private:
    struct JointStreamState {
        ArmJoint joint = ArmJoint::Base;
        float reference_deg = 0.0f;
        float target_position_deg = 0.0f;
        float target_velocity_deg_per_s = 0.0f;
        float applied_velocity_deg_per_s = 0.0f;
        bool configured = false;
        bool enabled = false;
        bool reference_reached = true;
        bool limit_blocked = false;
    };

    class LockGuard {
    public:
        explicit LockGuard(const ArmMotion* owner, TickType_t timeout_ticks = portMAX_DELAY);
        ~LockGuard();
        bool locked() const;

    private:
        const ArmMotion* owner_ = nullptr;
        bool locked_ = false;
    };

    static void task_entry(void* arg);
    void task_loop();
    void update_locked(float dt_s);

    static bool finite_position(const ArmMotionJointPosition& position);
    static bool finite_velocity(const ArmMotionJointVelocity& velocity);
    static float approach(float current, float target, float max_change);
    static int stream_index(ArmJoint joint);

    JointStreamState* stream_for_joint_locked(ArmJoint joint);
    const JointStreamState* stream_for_joint_locked(ArmJoint joint) const;
    const ArmMotionJointParameters& parameters_for_joint(ArmJoint joint) const;

    esp_err_t sync_streams_from_joint_layer_locked();
    esp_err_t enable_all_at_locked(
        const ArmMotionPositionCommand& initial_pose
    );
    esp_err_t write_stream_joint_locked(
        JointStreamState& state,
        float dt_s,
        bool clamp_to_position_target
    );
    void prepare_position_velocity_locked(JointStreamState& state);
    void set_position_target_locked(const ArmMotionJointPosition& position);
    void set_target_velocity_locked(const ArmMotionJointVelocity& velocity);
    void clear_velocity_locked();
    void start_stopping_locked();
    void enter_hold_locked();
    void record_driver_error_locked(
        esp_err_t error,
        ArmJoint joint,
        bool joint_valid
    );
    void clear_driver_error_locked();

private:
    bool initialized_ = false;
    ArmOutputState output_state_ = ArmOutputState::Disabled;
    ArmMotionMode mode_ = ArmMotionMode::Hold;
    esp_err_t last_driver_error_ = ESP_OK;
    bool driver_error_joint_valid_ = false;
    ArmJoint driver_error_joint_ = ArmJoint::Base;
    ArmJointController* joints_ = nullptr;

    ArmMotionParameters parameters_ = {};
    ArmMotionPositionCommand latest_position_command_ = {};
    ArmMotionCommand latest_velocity_command_ = {};

    JointStreamState streams_[5] = {};

    float claw_reference_gap_cm_ = 0.0f;
    float claw_target_gap_cm_ = 0.0f;

    int64_t last_command_us_ = 0;
    bool command_received_ = false;
    bool command_timed_out_ = false;

    mutable SemaphoreHandle_t mutex_ = nullptr;
    TaskHandle_t task_handle_ = nullptr;
    volatile bool task_stop_requested_ = false;
};

}  // namespace learm
