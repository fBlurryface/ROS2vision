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

struct JointRuntimeState {
    bool configured;
    bool enabled;

    ArmJoint joint;
    ServoChannel channel;

    // Software command inferred from the commanded pulse width. This is not
    // measured servo feedback.
    float command_deg;

    uint16_t command_us;
    uint16_t output_us;

    float zero_deg;
    uint16_t zero_us;
    float us_per_degree;
    int8_t direction;

    float min_deg;
    float max_deg;
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

    esp_err_t enable_joint(ArmJoint joint, float start_deg);
    esp_err_t enable_claw_at_gap(float start_gap_cm);
    esp_err_t disable_joint(ArmJoint joint);
    esp_err_t disable_all();

    esp_err_t write_deg_now(
        ArmJoint joint,
        float target_deg
    );

    // Write a raw pulse through the joint-to-channel mapping. Trajectory
    // interpolation belongs to the motion layer.
    esp_err_t write_us_now(
        ArmJoint joint,
        uint16_t pulse_us
    );

    uint16_t claw_gap_cm_to_us(float gap_cm) const;
    float claw_us_to_gap_cm(uint16_t pulse_us) const;

    float read_claw_gap_cm() const;

    float read_command_deg(ArmJoint joint) const;

    uint16_t read_command_us(ArmJoint joint) const;
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

    JointRuntimeState get_state(ArmJoint joint) const;

private:
    struct JointState {
        bool configured = false;
        JointCalibration calibration = {};
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
