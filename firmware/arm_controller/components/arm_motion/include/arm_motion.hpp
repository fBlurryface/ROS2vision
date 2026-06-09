#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "arm_joint.hpp"

namespace learm {

enum class JointMotionStyle : uint8_t {
    Linear = 0,
    Smooth,
    Soft,
};

struct ArmMotionTarget {
    float base_deg;
    float shoulder_deg;
    float elbow_deg;
    float wrist_pitch_deg;
    float wrist_roll_deg;

    // 夹爪使用真实控制目标：爪距，单位 cm。
    float claw_gap_cm;
};

struct ArmMotionDurationsMs {
    uint32_t base_ms;
    uint32_t shoulder_ms;
    uint32_t elbow_ms;
    uint32_t wrist_pitch_ms;
    uint32_t wrist_roll_ms;
    uint32_t claw_ms;
};

struct ArmMotionStrategies {
    JointMotionStyle base;
    JointMotionStyle shoulder;
    JointMotionStyle elbow;
    JointMotionStyle wrist_pitch;
    JointMotionStyle wrist_roll;
    JointMotionStyle claw;
};

struct ArmMotionState {
    bool initialized;
    bool ready;
    bool moving;

    ArmMotionDurationsMs durations_ms;
    ArmMotionStrategies strategies;

    JointRuntimeState base;
    JointRuntimeState shoulder;
    JointRuntimeState elbow;
    JointRuntimeState wrist_pitch;
    JointRuntimeState wrist_roll;
    JointRuntimeState claw;

    float claw_gap_cm;
    float claw_target_gap_cm;
};

class ArmMotion {
public:
    esp_err_t init(ArmJointController* joints);

    bool is_initialized() const;

    esp_err_t enable_all();
    esp_err_t disable_all();

    esp_err_t set_durations_ms(
        uint32_t base_ms,
        uint32_t shoulder_ms,
        uint32_t elbow_ms,
        uint32_t wrist_pitch_ms,
        uint32_t wrist_roll_ms,
        uint32_t claw_ms
    );

    esp_err_t set_strategies(
        JointMotionStyle base,
        JointMotionStyle shoulder,
        JointMotionStyle elbow,
        JointMotionStyle wrist_pitch,
        JointMotionStyle wrist_roll,
        JointMotionStyle claw
    );

    ArmMotionDurationsMs get_durations_ms() const;
    ArmMotionStrategies get_strategies() const;

    esp_err_t move_to(
        float base_deg,
        float shoulder_deg,
        float elbow_deg,
        float wrist_pitch_deg,
        float wrist_roll_deg,
        float claw_gap_cm
    );

    esp_err_t move_to(const ArmMotionTarget& target);

    esp_err_t stop_all();

    bool is_ready() const;
    bool is_moving() const;

    ArmMotionState get_state() const;

private:
    struct MotionJointState {
        bool configured = false;
        bool enabled = false;
        bool running = false;

        ArmJoint joint = ArmJoint::Claw;
        ServoChannel channel = ServoChannel::S0;

        uint16_t start_us = 1500;
        uint16_t current_us = 1500;
        uint16_t target_us = 1500;
        uint16_t output_us = 1500;

        uint32_t duration_ms = 0;
        uint32_t total_steps = 0;
        uint32_t elapsed_steps = 0;

        MotionProfile profile = MotionProfile::SmootherStep;
        uint16_t max_step_us = 0;
        uint16_t min_effective_step_us = 0;
    };

private:
    class LockGuard {
    public:
        explicit LockGuard(const ArmMotion* owner, TickType_t timeout_ticks = portMAX_DELAY);
        ~LockGuard();

        bool locked() const;

    private:
        const ArmMotion* owner_ = nullptr;
        bool locked_ = false;
    };

    static JointMotionOptions make_options(
        uint32_t duration_ms,
        JointMotionStyle style
    );

    static MotionProfile style_to_profile(JointMotionStyle style);

    static float evaluate_profile(MotionProfile profile, float t);
    static float smoothstep(float t);
    static float smootherstep(float t);

    static int joint_to_index(ArmJoint joint);
    static uint32_t clamp_duration_ms(uint32_t duration_ms);

    bool take_lock(TickType_t timeout_ticks = portMAX_DELAY) const;
    void give_lock() const;

    static void task_entry(void* arg);
    void task_loop();

    void update_all_locked();
    esp_err_t update_one_locked(uint8_t index);

    esp_err_t sync_state_from_hardware_locked(ArmJoint joint);

    esp_err_t commit_joint_target_locked(
        ArmJoint joint,
        uint16_t target_us,
        const JointMotionOptions& options
    );

    JointRuntimeState make_runtime_state_locked(ArmJoint joint) const;

private:
    bool initialized_ = false;
    ArmJointController* joints_ = nullptr;

    TaskHandle_t task_handle_ = nullptr;
    volatile bool task_stop_requested_ = false;
    SemaphoreHandle_t lock_ = nullptr;

    MotionJointState states_[kArmJointCount] = {};

    ArmMotionDurationsMs durations_ms_ = {
        700,
        700,
        700,
        700,
        700,
        700,
    };

    ArmMotionStrategies strategies_ = {
        JointMotionStyle::Soft,
        JointMotionStyle::Soft,
        JointMotionStyle::Soft,
        JointMotionStyle::Soft,
        JointMotionStyle::Soft,
        JointMotionStyle::Soft,
    };
};

}  // namespace learm
