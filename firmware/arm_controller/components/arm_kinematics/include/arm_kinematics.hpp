#pragma once

#include "esp_err.h"

namespace learm {

struct ArmKinematicsParameters {
    // Geometry in millimeters.
    float base_to_shoulder_z_mm = 28.9f;
    float upper_arm_mm = 104.3f;
    float forearm_mm = 89.0f;
    float wrist_to_tool_mm = 177.0f;

    // model_angle = semantic_angle * sign + offset
    float base_sign = 1.0f;
    float shoulder_sign = 1.0f;
    float elbow_sign = 1.0f;
    float wrist_pitch_sign = 1.0f;
    float wrist_roll_sign = 1.0f;

    float base_offset_deg = 0.0f;
    float shoulder_offset_deg = 0.0f;
    float elbow_offset_deg = 0.0f;
    float wrist_pitch_offset_deg = 0.0f;
    float wrist_roll_offset_deg = 0.0f;

    // Damped least-squares parameters. Angular task rows are converted to an
    // equivalent linear scale using orientation_weight_mm.
    float damping_mm = 10.0f;
    float orientation_weight_mm = 150.0f;

    // Output reference-velocity clamps.
    float max_base_deg_per_s = 75.0f;
    float max_shoulder_deg_per_s = 70.0f;
    float max_elbow_deg_per_s = 70.0f;
    float max_wrist_pitch_deg_per_s = 85.0f;
    float max_wrist_roll_deg_per_s = 60.0f;
};

struct ArmKinematicsJointState {
    float base_deg = 0.0f;
    float shoulder_deg = 0.0f;
    float elbow_deg = 0.0f;
    float wrist_pitch_deg = 0.0f;
    float wrist_roll_deg = 0.0f;
};

struct ArmCartesianPose {
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float z_mm = 0.0f;
    float tool_pitch_deg = 0.0f;
    float tool_roll_deg = 0.0f;
};

// Desired instantaneous tool twist expressed in the current tool frame.
struct ArmToolVelocity {
    float forward_mm_per_s = 0.0f;
    float left_mm_per_s = 0.0f;
    float up_mm_per_s = 0.0f;
    float pitch_deg_per_s = 0.0f;
    float roll_deg_per_s = 0.0f;
};

struct ArmKinematicsJointVelocity {
    float base_deg_per_s = 0.0f;
    float shoulder_deg_per_s = 0.0f;
    float elbow_deg_per_s = 0.0f;
    float wrist_pitch_deg_per_s = 0.0f;
    float wrist_roll_deg_per_s = 0.0f;
};

// Selects which rotary joints may participate in the resolved-rate solution.
// Disabled joints are constrained to zero reference velocity while the same
// damped least-squares solver redistributes the task among the remaining
// joints.
struct ArmKinematicsJointMask {
    bool base = true;
    bool shoulder = true;
    bool elbow = true;
    bool wrist_pitch = true;
    bool wrist_roll = true;
};

class ArmKinematics {
public:
    // Uses the single fixed parameter group above.
    esp_err_t init();

    bool is_initialized() const;
    ArmKinematicsParameters get_parameters() const;

    esp_err_t forward_pose(
        const ArmKinematicsJointState& joints,
        ArmCartesianPose* out_pose
    ) const;

    // Traditional resolved-rate relation:
    //   q_dot = J^T (J J^T + lambda^2 I)^-1 x_dot
    // q_dot is a commanded/reference velocity, not measured servo velocity.
    esp_err_t solve_tool_velocity(
        const ArmKinematicsJointState& joints,
        const ArmToolVelocity& tool_velocity,
        ArmKinematicsJointVelocity* out_velocity
    ) const;

    // Same resolved-rate calculation with selected joints constrained to zero.
    // The original overload above remains unchanged for callers that use all
    // joints.
    esp_err_t solve_tool_velocity(
        const ArmKinematicsJointState& joints,
        const ArmToolVelocity& tool_velocity,
        const ArmKinematicsJointMask& enabled_joints,
        ArmKinematicsJointVelocity* out_velocity
    ) const;

private:
    static bool finite(float value);
    static float deg_to_rad(float value);
    static float rad_to_deg(float value);
    static float dot3(const float a[3], const float b[3]);
    static bool solve_5x5(const float a[5][5], const float b[5], float x[5]);

    void compute_axes(
        const ArmKinematicsJointState& joints,
        float forward[3],
        float left[3],
        float up[3]
    ) const;

    void compute_position_jacobian(
        const ArmKinematicsJointState& joints,
        ArmCartesianPose* out_pose,
        float out_position_jacobian[3][5]
    ) const;

private:
    bool initialized_ = false;
    ArmKinematicsParameters parameters_ = {};
};

}  // namespace learm
