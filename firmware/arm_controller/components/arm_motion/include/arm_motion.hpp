#pragma once

#include <stdint.h>

#include "esp_err.h"
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
    // 内部调用 arm_joint 的 move_claw_gap_cm()，使用夹爪标定表插值。
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

    // 夹爪最重要的状态是爪距，而不是抽象角度。
    float claw_gap_cm;
    float claw_target_gap_cm;
};

class ArmMotion {
public:
    esp_err_t init(ArmJointController* joints);

    bool is_initialized() const;

    // 批量使能全部关节/舵机。
    // 这是初始化附着操作，不是平滑运动。
    // 每个关节会使用 arm_joint 层的默认 enable_joint(joint)，
    // 即附着到各自标定的零位/默认位。
    esp_err_t enable_all();

    // 配置类函数：只更新内部配置，不触发运动。
    // 修改后的配置只影响下一次 move_to()。
    esp_err_t set_durations_ms(
        uint32_t base_ms,
        uint32_t shoulder_ms,
        uint32_t elbow_ms,
        uint32_t wrist_pitch_ms,
        uint32_t wrist_roll_ms,
        uint32_t claw_ms
    );

    // 配置类函数：只更新内部配置，不触发运动。
    // 修改后的配置只影响下一次 move_to()。
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

    // 运动类函数：读取当前 durations + strategies，
    // 一次性提交 6 个关节目标并触发运动。
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
    static JointMotionOptions make_options(
        uint32_t duration_ms,
        JointMotionStyle style
    );

    static MotionProfile style_to_profile(JointMotionStyle style);

    esp_err_t move_one_deg(
        ArmJoint joint,
        float target_deg,
        uint32_t duration_ms,
        JointMotionStyle style
    );

    esp_err_t move_claw_gap(
        float target_gap_cm,
        uint32_t duration_ms,
        JointMotionStyle style
    );

private:
    bool initialized_ = false;
    ArmJointController* joints_ = nullptr;

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
