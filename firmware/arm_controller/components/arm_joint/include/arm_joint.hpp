#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "servo_pwm.hpp"

namespace learm {

enum class ArmJoint : uint8_t {
    Claw = 0,
    WristRoll,
    WristPitch,
    Elbow,
    Shoulder,
    Base,
};

inline constexpr uint8_t kArmJointCount = 6;

enum class JointTimingMode : uint8_t {
    Duration = 0,

    // 这里的 Speed 是“命令轨迹速度”，不是舵机真实闭环速度。
    // 普通 PWM 舵机没有实际角度/速度反馈。
    Speed,

    Immediate,
};

struct JointCalibration {
    ArmJoint joint;
    ServoChannel channel;

    // 语义角度零点。
    float zero_deg;

    // zero_deg 对应的 PWM 脉宽。
    uint16_t zero_us;

    // 每 1° 对应多少 us。
    float us_per_degree;

    // +1 表示语义角度增加时，pulse_us 增加。
    // -1 表示语义角度增加时，pulse_us 减少。
    int8_t direction;

    // 该关节在语义坐标系下允许的角度范围。
    float min_deg;
    float max_deg;
};

struct JointMotionOptions {
    JointTimingMode timing_mode = JointTimingMode::Duration;

    // timing_mode = Duration 时使用。
    uint32_t duration_ms = 700;

    // timing_mode = Speed 时使用。
    // 普通关节单位：deg/s。
    // 在 arm_motion 的夹爪速度链路中，该字段复用为 cm/s。
    // 注意：这是命令轨迹速度，不是舵机真实速度。
    float speed_deg_per_s = 60.0f;

    // timing_mode = Speed 时使用。
    // 普通关节单位：deg/s^2。
    // 在 arm_motion 的夹爪速度链路中，这两个字段复用为 cm/s^2。
    // 0 表示不做加减速斜坡，直接按 speed_deg_per_s 匀速推进。
    float accel_deg_per_s2 = 0.0f;
    float decel_deg_per_s2 = 0.0f;

    MotionProfile profile = MotionProfile::SmootherStep;

    bool replan_from_current = true;

    // 透传到底层 servo_pwm。
    // 0 表示不限制。
    uint16_t max_step_us = 0;

    // 透传到底层 servo_pwm。
    // 0 表示不启用。
    uint16_t min_effective_step_us = 0;
};


struct ArmJointGroupTarget {
    float base_deg;
    float shoulder_deg;
    float elbow_deg;
    float wrist_pitch_deg;
    float wrist_roll_deg;
    float claw_gap_cm;
};

struct ArmJointGroupMotionOptions {
    JointMotionOptions base;
    JointMotionOptions shoulder;
    JointMotionOptions elbow;
    JointMotionOptions wrist_pitch;
    JointMotionOptions wrist_roll;
    JointMotionOptions claw;
};

struct JointRuntimeState {
    bool configured;
    bool enabled;
    bool running;
    bool ready;

    ArmJoint joint;
    ServoChannel channel;

    // 注意：这里的 current_deg / target_deg 都是根据软件命令脉宽反算的。
    // 它们不是舵机真实角度反馈。
    float current_deg;
    float target_deg;

    uint16_t current_us;
    uint16_t target_us;
    uint16_t output_us;

    float zero_deg;
    uint16_t zero_us;
    float us_per_degree;
    int8_t direction;

    float min_deg;
    float max_deg;

    uint32_t duration_ms;
    uint32_t total_steps;
    uint32_t elapsed_steps;

    MotionProfile profile;
};

struct ClawGapPoint {
    uint16_t pulse_us;
    float gap_cm;
};

class ArmJointController {
public:
    esp_err_t init(
        ServoPwm* servo_pwm,
        const JointCalibration* calibrations,
        uint8_t count
    );

    bool is_initialized() const;
    bool is_configured(ArmJoint joint) const;

    esp_err_t enable_joint(ArmJoint joint);
    esp_err_t enable_joint(ArmJoint joint, float start_deg);
    esp_err_t disable_joint(ArmJoint joint);
    esp_err_t disable_all();

    esp_err_t move_deg(
        ArmJoint joint,
        float target_deg,
        uint32_t duration_ms
    );

    esp_err_t move_deg(
        ArmJoint joint,
        float target_deg,
        const JointMotionOptions& options
    );

    esp_err_t move_speed(
        ArmJoint joint,
        float target_deg,
        float speed_deg_per_s,
        MotionProfile profile = MotionProfile::SmootherStep
    );

    esp_err_t write_deg_now(
        ArmJoint joint,
        float target_deg
    );

    // V5A: 中层只做单关节抽象和单位转换。
    // 该接口按关节写入原始 pulse_us，不做插补。
    esp_err_t write_us_now(
        ArmJoint joint,
        uint16_t pulse_us
    );

    // 按夹爪目标爪距控制，单位 cm。
    // 内部使用夹爪爪距标定表进行分段线性插值。
    esp_err_t move_claw_gap_cm(
        float target_gap_cm,
        uint32_t duration_ms
    );

    esp_err_t move_claw_gap_cm(
        float target_gap_cm,
        const JointMotionOptions& options
    );

    // 多关节同步提交。
    // 这是独立 group 链路，不循环调用 move_deg() / move_claw_gap_cm()。
    esp_err_t move_group(
        const ArmJointGroupTarget& target,
        const ArmJointGroupMotionOptions& options
    );

    uint16_t claw_gap_cm_to_us(float gap_cm) const;
    float claw_us_to_gap_cm(uint16_t pulse_us) const;

    float read_claw_gap_cm() const;
    float read_claw_target_gap_cm() const;

    esp_err_t reset_joint(
        ArmJoint joint,
        uint32_t duration_ms
    );

    esp_err_t reset_all(uint32_t duration_ms);

    esp_err_t stop(ArmJoint joint);
    esp_err_t stop_all();

    float read_command_deg(ArmJoint joint) const;
    float read_target_deg(ArmJoint joint) const;

    uint16_t read_command_us(ArmJoint joint) const;
    uint16_t read_target_us(ArmJoint joint) const;
    uint16_t read_output_us(ArmJoint joint) const;

    uint16_t deg_to_us(ArmJoint joint, float deg) const;
    float us_to_deg(ArmJoint joint, uint16_t pulse_us) const;

    JointCalibration get_calibration(ArmJoint joint) const;

    // 允许运行时改标定，便于以后上位机调参。
    // 该函数只改软件映射，不会直接移动舵机。
    esp_err_t set_calibration(
        ArmJoint joint,
        const JointCalibration& calibration
    );

    esp_err_t set_default_motion_options(
        ArmJoint joint,
        const JointMotionOptions& options
    );

    JointMotionOptions get_default_motion_options(ArmJoint joint) const;

    JointRuntimeState get_state(ArmJoint joint) const;

    bool is_ready(ArmJoint joint) const;
    bool is_all_ready() const;

private:
    struct JointState {
        bool configured = false;
        JointCalibration calibration = {};
        JointMotionOptions default_options = {};
    };

private:
    static int joint_to_index(ArmJoint joint);

    bool is_valid_joint_index(int index) const;

    esp_err_t normalize_calibration(
        JointCalibration* calibration
    ) const;

    float clamp_deg_by_index(uint8_t index, float deg) const;

    uint16_t deg_to_us_by_index(uint8_t index, float deg) const;
    float us_to_deg_by_index(uint8_t index, uint16_t pulse_us) const;

    uint16_t claw_gap_cm_to_us_from_table(float gap_cm) const;
    float claw_us_to_gap_cm_from_table(uint16_t pulse_us) const;

private:
    bool initialized_ = false;
    ServoPwm* servo_ = nullptr;

    JointState joints_[kArmJointCount] = {};
};

// 当前这台机械臂的固化默认参数。
// 后续基础标定结果就改这里对应的数组内容。
// main 不再保存这些参数。
namespace arm_joint_defaults {

extern const ServoPwmConfig kServoPwmConfigs[];
extern const uint8_t kServoPwmConfigCount;

extern const JointCalibration kJointCalibrations[];
extern const uint8_t kJointCalibrationCount;

extern const ClawGapPoint kClawGapTable[];
extern const uint8_t kClawGapTableCount;

}  // namespace arm_joint_defaults

}  // namespace learm