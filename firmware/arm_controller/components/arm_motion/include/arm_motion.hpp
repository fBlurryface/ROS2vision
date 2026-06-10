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

struct JointSpeedStrategy {
    // 普通关节：deg/s。
    // 这是命令轨迹最大速度，不是舵机真实闭环速度。
    // Speed line is direct linear command speed: no ease-in, no brake curve.
    float max_speed_deg_per_s;
};

struct ClawSpeedStrategy {
    // 夹爪：cm/s。
    // 使用当前的 gap_cm <-> pulse_us 标定表生成命令轨迹。
    // Speed line is direct linear claw-gap speed: no ease-in, no brake curve.
    float max_speed_cm_per_s;
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

struct ArmMotionDelta {
    // First five joints: semantic joint-angle deltas in degrees.
    float base_deg;
    float shoulder_deg;
    float elbow_deg;
    float wrist_pitch_deg;
    float wrist_roll_deg;

    // Claw: gap delta in cm. Positive opens the claw, negative closes it.
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

// 旧的策略集现在明确归属于 Duration 控制线。
// 它描述的是 position progress 的曲线形状，而不是速度曲线。
struct ArmMotionDurationStrategies {
    JointMotionStyle base;
    JointMotionStyle shoulder;
    JointMotionStyle elbow;
    JointMotionStyle wrist_pitch;
    JointMotionStyle wrist_roll;
    JointMotionStyle claw;
};

// 兼容旧代码里的 ArmMotionStrategies 命名。
using ArmMotionStrategies = ArmMotionDurationStrategies;

// Speed 控制线专属策略集。
// 普通关节用 deg/s；夹爪用 cm/s。
// Speed 控制线不做曲线整形；每个 tick 直接按设定速度推进，到目标时钳住。
struct ArmMotionSpeedStrategies {
    JointSpeedStrategy base;
    JointSpeedStrategy shoulder;
    JointSpeedStrategy elbow;
    JointSpeedStrategy wrist_pitch;
    JointSpeedStrategy wrist_roll;
    ClawSpeedStrategy claw;
};

struct ArmMotionState {
    bool initialized;
    bool ready;
    bool moving;

    ArmMotionDurationsMs durations_ms;
    ArmMotionDurationStrategies duration_strategies;
    ArmMotionSpeedStrategies speed_strategies;

    // 兼容旧字段名。它等同于 duration_strategies。
    ArmMotionDurationStrategies strategies;

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

    // Duration 控制线的策略配置。
    esp_err_t set_duration_strategies(
        JointMotionStyle base,
        JointMotionStyle shoulder,
        JointMotionStyle elbow,
        JointMotionStyle wrist_pitch,
        JointMotionStyle wrist_roll,
        JointMotionStyle claw
    );

    // 兼容旧接口名：等同于 set_duration_strategies。
    esp_err_t set_strategies(
        JointMotionStyle base,
        JointMotionStyle shoulder,
        JointMotionStyle elbow,
        JointMotionStyle wrist_pitch,
        JointMotionStyle wrist_roll,
        JointMotionStyle claw
    );

    // Speed 控制线的策略配置。
    esp_err_t set_speed_strategies(
        const JointSpeedStrategy& base,
        const JointSpeedStrategy& shoulder,
        const JointSpeedStrategy& elbow,
        const JointSpeedStrategy& wrist_pitch,
        const JointSpeedStrategy& wrist_roll,
        const ClawSpeedStrategy& claw
    );

    // 便捷接口：只改最大速度。
    esp_err_t set_speed_limits(
        float base_deg_per_s,
        float shoulder_deg_per_s,
        float elbow_deg_per_s,
        float wrist_pitch_deg_per_s,
        float wrist_roll_deg_per_s,
        float claw_cm_per_s
    );


    ArmMotionDurationsMs get_durations_ms() const;
    ArmMotionDurationStrategies get_duration_strategies() const;
    ArmMotionStrategies get_strategies() const;
    ArmMotionSpeedStrategies get_speed_strategies() const;

    // Duration 控制线：按总时长和 duration_strategies 运动。
    esp_err_t move_to(
        float base_deg,
        float shoulder_deg,
        float elbow_deg,
        float wrist_pitch_deg,
        float wrist_roll_deg,
        float claw_gap_cm
    );

    esp_err_t move_to(const ArmMotionTarget& target);

    // Speed 控制线：按 speed_strategies 的最大速度直线匀速运动，动作耗时由距离自然决定。
    esp_err_t move_to_speed(
        float base_deg,
        float shoulder_deg,
        float elbow_deg,
        float wrist_pitch_deg,
        float wrist_roll_deg,
        float claw_gap_cm
    );

    esp_err_t move_to_speed(const ArmMotionTarget& target);

    // Immediate control line: write all joints to the target pose now.
    // No interpolation, duration, or speed limiting is applied by arm_motion.
    // This is intended for high-level controllers that own timing externally
    // (for example, incremental Jacobian / resolved-rate control loops).
    esp_err_t move_to_immediate(
        float base_deg,
        float shoulder_deg,
        float elbow_deg,
        float wrist_pitch_deg,
        float wrist_roll_deg,
        float claw_gap_cm
    );

    esp_err_t move_to_immediate(const ArmMotionTarget& target);

    // Incremental immediate control line: add deltas to the current software
    // commanded pose, then write the resulting absolute pose immediately.
    // First five deltas are degrees; claw delta is gap cm.
    // This is intended for controllers that output delta-q, such as an
    // incremental Jacobian controller.
    esp_err_t move_delta_immediate(
        float base_delta_deg,
        float shoulder_delta_deg,
        float elbow_delta_deg,
        float wrist_pitch_delta_deg,
        float wrist_roll_delta_deg,
        float claw_gap_delta_cm
    );

    esp_err_t move_delta_immediate(const ArmMotionDelta& delta);

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

        JointTimingMode timing_mode = JointTimingMode::Duration;

        uint16_t start_us = 1500;
        uint16_t current_us = 1500;
        uint16_t target_us = 1500;
        uint16_t output_us = 1500;

        // 浮点累计值避免速度模式下 6.67us/tick 这类小数步长反复丢精度。
        float start_us_f = 1500.0f;
        float current_us_f = 1500.0f;
        float target_us_f = 1500.0f;

        // Duration strategy state.
        uint32_t duration_ms = 0;
        uint32_t total_steps = 0;      // Debug/compatibility only: nominal 20ms step count.
        uint32_t elapsed_steps = 0;    // Debug counter; duration progress uses elapsed_time_ms.
        float elapsed_time_ms = 0.0f;

        MotionProfile profile = MotionProfile::SmootherStep;
        uint16_t max_step_us = 0;
        uint16_t min_effective_step_us = 0;

        // Speed strategy state.
        // 普通关节单位是 deg / deg/s / deg/s^2。
        // 夹爪单位是 cm / cm/s / cm/s^2。
        float current_position = 0.0f;
        float target_position = 0.0f;
        float current_speed_units_per_s = 0.0f;
        float max_speed_units_per_s = 0.0f;
        float accel_units_per_s2 = 0.0f;
        float decel_units_per_s2 = 0.0f;
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

    static JointMotionOptions make_duration_options(
        uint32_t duration_ms,
        JointMotionStyle style
    );

    // 兼容旧私有函数名。
    static JointMotionOptions make_options(
        uint32_t duration_ms,
        JointMotionStyle style
    );

    static JointMotionOptions make_speed_options(
        float max_speed_units_per_s
    );

    static JointMotionOptions make_immediate_options();

    static MotionProfile style_to_profile(JointMotionStyle style);

    static float evaluate_profile(MotionProfile profile, float t);
    static float smoothstep(float t);
    static float smootherstep(float t);

    static int joint_to_index(ArmJoint joint);
    static uint32_t clamp_duration_ms(uint32_t duration_ms);
    static bool is_valid_joint_speed_strategy(const JointSpeedStrategy& strategy);
    static bool is_valid_claw_speed_strategy(const ClawSpeedStrategy& strategy);

    bool take_lock(TickType_t timeout_ticks = portMAX_DELAY) const;
    void give_lock() const;

    static void task_entry(void* arg);
    void task_loop();

    void update_all_locked(float dt_s);
    esp_err_t update_one_locked(uint8_t index, float dt_s);
    esp_err_t update_duration_locked(MotionJointState& s, float dt_s);
    esp_err_t update_speed_locked(MotionJointState& s, float dt_s);

    esp_err_t sync_state_from_hardware_locked(ArmJoint joint);

    esp_err_t commit_joint_target_locked(
        ArmJoint joint,
        uint16_t target_us,
        const JointMotionOptions& options
    );

    float motion_position_from_us_locked(ArmJoint joint, uint16_t pulse_us) const;
    uint16_t motion_us_from_position_locked(ArmJoint joint, float position) const;

    void clear_motion_progress_locked(MotionJointState& s);

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

    ArmMotionDurationStrategies duration_strategies_ = {
        JointMotionStyle::Soft,
        JointMotionStyle::Soft,
        JointMotionStyle::Soft,
        JointMotionStyle::Soft,
        JointMotionStyle::Soft,
        JointMotionStyle::Soft,
    };

    ArmMotionSpeedStrategies speed_strategies_ = {
        {45.0f},  // base deg/s
        {35.0f},  // shoulder deg/s
        {35.0f},  // elbow deg/s
        {50.0f},  // wrist_pitch deg/s
        {70.0f},  // wrist_roll deg/s
        {2.5f},   // claw cm/s
    };
};

}  // namespace learm
