#pragma once

#include "esp_err.h"

namespace learm {

struct ArmKinematicsConfig {
    // Geometry, in millimeters.
    // Model: base yaw + three pitch joints in one vertical plane.
    // Pitch convention:
    //   0 deg means the link is vertical / coaxial with the previous link;
    //   positive deg tilts forward; negative deg tilts backward.
    float base_to_shoulder_z_mm = 28.9f;
    float upper_arm_mm = 104.3f;
    float forearm_mm = 89.0f;
    float wrist_to_tool_mm = 177.0f;

    // Semantic joint angle -> kinematic model angle mapping.
    // model_angle_deg = semantic_deg * sign + offset_deg.
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

    // Task solver settings.
    // The main solver is solve_tool_target_delta(). It assumes the target point
    // is expressed in the current tool/f-link frame:
    //   forward: along the final link f;
    //   left:    horizontal left/right around the base yaw axis;
    //   up:      pitch-plane perpendicular to f.
    // It solves:
    //   left        -> base yaw only;
    //   forward/up  -> C/D/E pitch-plane translation, first in the null space
    //                  that preserves f pitch, then relaxed if that translation
    //                  is ineffective near workspace limits/singularities.
    float lateral_gain = 0.45f;
    float longitudinal_gain = 0.35f;
    float up_gain = 0.30f;

    // Damping terms. Units are mm because the task rows are mm/rad.
    float lateral_damping_mm = 35.0f;
    float longitudinal_damping_mm = 35.0f;
    float up_damping_mm = 35.0f;

    // If the posture-preserving 2D planar translation Jacobian has less than
    // this minimum squared singular value, the solver falls back to a relaxed
    // planar solve with a soft penalty on changing f pitch.
    float planar_hold_min_effect_mm2 = 1.0f;

    // Soft orientation weight used only in the relaxed planar fallback. Larger
    // values resist changing f pitch; smaller values allow C/D/E to rotate f
    // to keep making progress near workspace limits/singularities.
    float relaxed_orientation_weight_mm = 10.0f;

    // Safety clamps for one solver call.
    float max_error_mm = 20.0f;
    float max_delta_deg_base = 1.0f;
    float max_delta_deg_shoulder = 1.0f;
    float max_delta_deg_elbow = 1.0f;
    float max_delta_deg_wrist_pitch = 1.0f;
    float max_delta_deg_wrist_roll = 1.0f;

    // Semantic joint hard ranges. Keep these in sync with arm_joint calibration.
    // The solver uses them before returning deltas so a lower layer clamp does
    // not silently break posture-hold constraints.
    float min_deg_base = -85.0f;
    float max_deg_base = 85.0f;
    float min_deg_shoulder = -85.0f;
    float max_deg_shoulder = 85.0f;
    float min_deg_elbow = -85.0f;
    float max_deg_elbow = 85.0f;
    float min_deg_wrist_pitch = -85.0f;
    float max_deg_wrist_pitch = 85.0f;
};

struct ArmKinematicsJointState {
    float base_deg = 0.0f;
    float shoulder_deg = 0.0f;
    float elbow_deg = 0.0f;
    float wrist_pitch_deg = 0.0f;
    float wrist_roll_deg = 0.0f;
};

struct ArmCartesianPosition {
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float z_mm = 0.0f;
};

struct ArmToolTargetError {
    // Target point relative to the current tool/f-link frame, in millimeters.
    // forward_mm is the current signed distance along f from the tool reference
    // point to the target point.
    float forward_mm = 0.0f;

    // Positive left means the target point is to the left of the current base
    // yaw plane. This error is solved by base yaw.
    float left_mm = 0.0f;

    // Positive up means the target point is above the current f axis in the
    // pitch plane. This is treated as a pitch-plane translation error: the
    // solver first tries to remove it while preserving f pitch.
    float up_mm = 0.0f;

    // Desired signed distance from the tool reference point to the target along
    // f. For example, 0 means drive the tool reference point onto the target;
    // a positive value keeps the target that many mm in front of the tool.
    float desired_forward_mm = 0.0f;
};

struct ArmKinematicsDelta {
    float base_delta_deg = 0.0f;
    float shoulder_delta_deg = 0.0f;
    float elbow_delta_deg = 0.0f;
    float wrist_pitch_delta_deg = 0.0f;
    float wrist_roll_delta_deg = 0.0f;
};

class ArmKinematics {
public:
    esp_err_t init(const ArmKinematicsConfig& config);

    bool is_initialized() const;
    ArmKinematicsConfig get_config() const;

    // Runtime parameter update hooks. These are intended for upper layers
    // such as a future binary serial/WiFi protocol: parse the packet there,
    // then call these functions directly instead of going through ASCII shell
    // commands. All updates are validated before being applied.
    esp_err_t set_config(const ArmKinematicsConfig& config);
    esp_err_t set_uniform_delta_limit_deg(float limit_deg);
    esp_err_t set_delta_limits_deg(
        float base_deg,
        float shoulder_deg,
        float elbow_deg,
        float wrist_pitch_deg,
        float wrist_roll_deg
    );

    esp_err_t forward_position(
        const ArmKinematicsJointState& joints,
        ArmCartesianPosition* out_position
    ) const;

    esp_err_t solve_tool_target_delta(
        const ArmKinematicsJointState& joints,
        const ArmToolTargetError& target,
        ArmKinematicsDelta* out_delta
    ) const;

private:
    static bool is_finite(float value);
    static float deg_to_rad(float deg);
    static float rad_to_deg(float rad);
    static float clamp_abs(float value, float limit_abs);
    static float clamp_error(float value, float limit_abs);
    static float dot3(const float a[3], const float b[3]);
    static bool solve_3x3(const float a[3][3], const float b[3], float x[3]);
    static esp_err_t validate_config(const ArmKinematicsConfig& config);
    static void log_config(const ArmKinematicsConfig& config, const char* action);

    float scale_cde_to_limits(
        const ArmKinematicsJointState& joints,
        float dq_deg[3],
        bool* out_step_limited,
        bool* out_range_limited
    ) const;

    float clamp_delta_to_joint_range(
        float current_deg,
        float delta_deg,
        float min_deg,
        float max_deg
    ) const;

    void compute_tool_axes(
        const ArmKinematicsJointState& joints,
        float forward[3],
        float left[3],
        float up[3],
        float pitch_axis[3]
    ) const;

    void compute_position_and_jacobian(
        const ArmKinematicsJointState& joints,
        ArmCartesianPosition* out_position,
        float out_jacobian[3][4]
    ) const;

private:
    bool initialized_ = false;
    ArmKinematicsConfig config_ = {};
};

}  // namespace learm
