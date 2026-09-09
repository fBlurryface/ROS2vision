#include "arm_kinematics.hpp"

#include <algorithm>
#include <cmath>

#include "esp_log.h"

namespace learm {

static const char* TAG = "arm_velocity_kinematics";
static constexpr float kPi = 3.14159265358979323846f;

bool ArmKinematics::finite(float value)
{
    return std::isfinite(value);
}

float ArmKinematics::deg_to_rad(float value)
{
    return value * (kPi / 180.0f);
}

float ArmKinematics::rad_to_deg(float value)
{
    return value * (180.0f / kPi);
}

float ArmKinematics::dot3(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

bool ArmKinematics::solve_5x5(const float a[5][5], const float b[5], float x[5])
{
    float m[5][6] = {};
    for (int row = 0; row < 5; ++row) {
        for (int col = 0; col < 5; ++col) {
            m[row][col] = a[row][col];
        }
        m[row][5] = b[row];
    }

    for (int col = 0; col < 5; ++col) {
        int pivot = col;
        float pivot_abs = std::fabs(m[col][col]);
        for (int row = col + 1; row < 5; ++row) {
            const float candidate = std::fabs(m[row][col]);
            if (candidate > pivot_abs) {
                pivot = row;
                pivot_abs = candidate;
            }
        }

        if (pivot_abs < 1.0e-9f) {
            return false;
        }

        if (pivot != col) {
            for (int k = col; k < 6; ++k) {
                std::swap(m[col][k], m[pivot][k]);
            }
        }

        const float divisor = m[col][col];
        for (int k = col; k < 6; ++k) {
            m[col][k] /= divisor;
        }

        for (int row = 0; row < 5; ++row) {
            if (row == col) {
                continue;
            }
            const float factor = m[row][col];
            for (int k = col; k < 6; ++k) {
                m[row][k] -= factor * m[col][k];
            }
        }
    }

    for (int i = 0; i < 5; ++i) {
        x[i] = m[i][5];
        if (!finite(x[i])) {
            return false;
        }
    }
    return true;
}

esp_err_t ArmKinematics::init()
{
    if (parameters_.upper_arm_mm <= 0.0f ||
        parameters_.forearm_mm <= 0.0f ||
        parameters_.wrist_to_tool_mm < 0.0f ||
        parameters_.damping_mm <= 0.0f ||
        parameters_.orientation_weight_mm <= 0.0f) {
        return ESP_ERR_INVALID_STATE;
    }

    initialized_ = true;
    ESP_LOGI(TAG,
             "velocity Jacobian initialized links=[%.1f %.1f %.1f] damping=%.1f orientation_weight=%.1f",
             static_cast<double>(parameters_.upper_arm_mm),
             static_cast<double>(parameters_.forearm_mm),
             static_cast<double>(parameters_.wrist_to_tool_mm),
             static_cast<double>(parameters_.damping_mm),
             static_cast<double>(parameters_.orientation_weight_mm));
    return ESP_OK;
}

bool ArmKinematics::is_initialized() const
{
    return initialized_;
}

ArmKinematicsParameters ArmKinematics::get_parameters() const
{
    return parameters_;
}

void ArmKinematics::compute_axes(
    const ArmKinematicsJointState& joints,
    float forward[3],
    float left[3],
    float up[3]
) const
{
    const float base = deg_to_rad(
        joints.base_deg * parameters_.base_sign + parameters_.base_offset_deg
    );
    const float shoulder = deg_to_rad(
        joints.shoulder_deg * parameters_.shoulder_sign + parameters_.shoulder_offset_deg
    );
    const float elbow = deg_to_rad(
        joints.elbow_deg * parameters_.elbow_sign + parameters_.elbow_offset_deg
    );
    const float wrist = deg_to_rad(
        joints.wrist_pitch_deg * parameters_.wrist_pitch_sign + parameters_.wrist_pitch_offset_deg
    );

    const float pitch = shoulder + elbow + wrist;
    const float cb = std::cos(base);
    const float sb = std::sin(base);
    const float cp = std::cos(pitch);
    const float sp = std::sin(pitch);

    forward[0] = cb * sp;
    forward[1] = sb * sp;
    forward[2] = cp;

    left[0] = -sb;
    left[1] = cb;
    left[2] = 0.0f;

    up[0] = -cb * cp;
    up[1] = -sb * cp;
    up[2] = sp;
}

void ArmKinematics::compute_position_jacobian(
    const ArmKinematicsJointState& joints,
    ArmCartesianPose* out_pose,
    float out_position_jacobian[3][5]
) const
{
    const float base = deg_to_rad(
        joints.base_deg * parameters_.base_sign + parameters_.base_offset_deg
    );
    const float shoulder = deg_to_rad(
        joints.shoulder_deg * parameters_.shoulder_sign + parameters_.shoulder_offset_deg
    );
    const float elbow = deg_to_rad(
        joints.elbow_deg * parameters_.elbow_sign + parameters_.elbow_offset_deg
    );
    const float wrist = deg_to_rad(
        joints.wrist_pitch_deg * parameters_.wrist_pitch_sign + parameters_.wrist_pitch_offset_deg
    );

    const float a1 = shoulder;
    const float a2 = shoulder + elbow;
    const float a3 = shoulder + elbow + wrist;

    const float cb = std::cos(base);
    const float sb = std::sin(base);
    const float c1 = std::cos(a1);
    const float s1 = std::sin(a1);
    const float c2 = std::cos(a2);
    const float s2 = std::sin(a2);
    const float c3 = std::cos(a3);
    const float s3 = std::sin(a3);

    const float l1 = parameters_.upper_arm_mm;
    const float l2 = parameters_.forearm_mm;
    const float l3 = parameters_.wrist_to_tool_mm;

    const float radius = l1 * s1 + l2 * s2 + l3 * s3;
    const float z = parameters_.base_to_shoulder_z_mm +
                    l1 * c1 + l2 * c2 + l3 * c3;

    if (out_pose != nullptr) {
        out_pose->x_mm = radius * cb;
        out_pose->y_mm = radius * sb;
        out_pose->z_mm = z;
        out_pose->tool_pitch_deg = rad_to_deg(a3);
        out_pose->tool_roll_deg =
            joints.wrist_roll_deg * parameters_.wrist_roll_sign +
            parameters_.wrist_roll_offset_deg;
    }

    if (out_position_jacobian == nullptr) {
        return;
    }

    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 5; ++col) {
            out_position_jacobian[row][col] = 0.0f;
        }
    }

    const float dr_ds = l1 * c1 + l2 * c2 + l3 * c3;
    const float dz_ds = -l1 * s1 - l2 * s2 - l3 * s3;
    const float dr_de = l2 * c2 + l3 * c3;
    const float dz_de = -l2 * s2 - l3 * s3;
    const float dr_dw = l3 * c3;
    const float dz_dw = -l3 * s3;

    out_position_jacobian[0][0] = -radius * sb * parameters_.base_sign;
    out_position_jacobian[1][0] =  radius * cb * parameters_.base_sign;

    out_position_jacobian[0][1] = dr_ds * cb * parameters_.shoulder_sign;
    out_position_jacobian[1][1] = dr_ds * sb * parameters_.shoulder_sign;
    out_position_jacobian[2][1] = dz_ds * parameters_.shoulder_sign;

    out_position_jacobian[0][2] = dr_de * cb * parameters_.elbow_sign;
    out_position_jacobian[1][2] = dr_de * sb * parameters_.elbow_sign;
    out_position_jacobian[2][2] = dz_de * parameters_.elbow_sign;

    out_position_jacobian[0][3] = dr_dw * cb * parameters_.wrist_pitch_sign;
    out_position_jacobian[1][3] = dr_dw * sb * parameters_.wrist_pitch_sign;
    out_position_jacobian[2][3] = dz_dw * parameters_.wrist_pitch_sign;
}

esp_err_t ArmKinematics::forward_pose(
    const ArmKinematicsJointState& joints,
    ArmCartesianPose* out_pose
) const
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (out_pose == nullptr ||
        !finite(joints.base_deg) ||
        !finite(joints.shoulder_deg) ||
        !finite(joints.elbow_deg) ||
        !finite(joints.wrist_pitch_deg) ||
        !finite(joints.wrist_roll_deg)) {
        return ESP_ERR_INVALID_ARG;
    }

    compute_position_jacobian(joints, out_pose, nullptr);
    return ESP_OK;
}

esp_err_t ArmKinematics::solve_tool_velocity(
    const ArmKinematicsJointState& joints,
    const ArmToolVelocity& tool_velocity,
    ArmKinematicsJointVelocity* out_velocity
) const
{
    return solve_tool_velocity(
        joints,
        tool_velocity,
        ArmKinematicsJointMask{},
        out_velocity
    );
}

esp_err_t ArmKinematics::solve_tool_velocity(
    const ArmKinematicsJointState& joints,
    const ArmToolVelocity& tool_velocity,
    const ArmKinematicsJointMask& enabled_joints,
    ArmKinematicsJointVelocity* out_velocity
) const
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (out_velocity == nullptr ||
        !finite(joints.base_deg) ||
        !finite(joints.shoulder_deg) ||
        !finite(joints.elbow_deg) ||
        !finite(joints.wrist_pitch_deg) ||
        !finite(joints.wrist_roll_deg) ||
        !finite(tool_velocity.forward_mm_per_s) ||
        !finite(tool_velocity.left_mm_per_s) ||
        !finite(tool_velocity.up_mm_per_s) ||
        !finite(tool_velocity.pitch_deg_per_s) ||
        !finite(tool_velocity.roll_deg_per_s)) {
        return ESP_ERR_INVALID_ARG;
    }

    float j_position[3][5] = {};
    compute_position_jacobian(joints, nullptr, j_position);

    float forward[3] = {};
    float left[3] = {};
    float up[3] = {};
    compute_axes(joints, forward, left, up);

    float j[5][5] = {};
    for (int col = 0; col < 5; ++col) {
        const float column[3] = {
            j_position[0][col],
            j_position[1][col],
            j_position[2][col],
        };
        j[0][col] = dot3(forward, column);
        j[1][col] = dot3(left, column);
        j[2][col] = dot3(up, column);
    }

    const float orientation_weight = parameters_.orientation_weight_mm;
    j[3][1] = orientation_weight * parameters_.shoulder_sign;
    j[3][2] = orientation_weight * parameters_.elbow_sign;
    j[3][3] = orientation_weight * parameters_.wrist_pitch_sign;
    j[4][4] = orientation_weight * parameters_.wrist_roll_sign;

    const bool joint_enabled[5] = {
        enabled_joints.base,
        enabled_joints.shoulder,
        enabled_joints.elbow,
        enabled_joints.wrist_pitch,
        enabled_joints.wrist_roll,
    };
    for (int joint = 0; joint < 5; ++joint) {
        if (joint_enabled[joint]) {
            continue;
        }
        for (int task_row = 0; task_row < 5; ++task_row) {
            j[task_row][joint] = 0.0f;
        }
    }

    const float task[5] = {
        tool_velocity.forward_mm_per_s,
        tool_velocity.left_mm_per_s,
        tool_velocity.up_mm_per_s,
        orientation_weight * deg_to_rad(tool_velocity.pitch_deg_per_s),
        orientation_weight * deg_to_rad(tool_velocity.roll_deg_per_s),
    };

    float jj_t[5][5] = {};
    for (int row = 0; row < 5; ++row) {
        for (int col = 0; col < 5; ++col) {
            for (int k = 0; k < 5; ++k) {
                jj_t[row][col] += j[row][k] * j[col][k];
            }
        }
    }

    const float lambda2 = parameters_.damping_mm * parameters_.damping_mm;
    for (int i = 0; i < 5; ++i) {
        jj_t[i][i] += lambda2;
    }

    float intermediate[5] = {};
    if (!solve_5x5(jj_t, task, intermediate)) {
        *out_velocity = {};
        return ESP_ERR_INVALID_STATE;
    }

    float qdot_rad_per_s[5] = {};
    for (int joint = 0; joint < 5; ++joint) {
        for (int task_row = 0; task_row < 5; ++task_row) {
            qdot_rad_per_s[joint] += j[task_row][joint] * intermediate[task_row];
        }
    }

    const float limits[5] = {
        parameters_.max_base_deg_per_s,
        parameters_.max_shoulder_deg_per_s,
        parameters_.max_elbow_deg_per_s,
        parameters_.max_wrist_pitch_deg_per_s,
        parameters_.max_wrist_roll_deg_per_s,
    };

    float qdot_deg_per_s[5] = {};
    float velocity_scale = 1.0f;
    for (int i = 0; i < 5; ++i) {
        qdot_deg_per_s[i] = rad_to_deg(qdot_rad_per_s[i]);
        if (!finite(qdot_deg_per_s[i])) {
            *out_velocity = {};
            return ESP_ERR_INVALID_STATE;
        }

        const float magnitude = std::fabs(qdot_deg_per_s[i]);
        const float limit = std::fabs(limits[i]);
        if (magnitude > limit && magnitude > 0.0f) {
            velocity_scale = std::min(velocity_scale, limit / magnitude);
        }
    }

    // Preserve the resolved-rate joint vector direction when any joint reaches
    // its reference-velocity limit. Independent clipping would alter the task
    // direction and can create surprising tool motion.
    for (float& value : qdot_deg_per_s) {
        value *= velocity_scale;
    }

    out_velocity->base_deg_per_s = qdot_deg_per_s[0];
    out_velocity->shoulder_deg_per_s = qdot_deg_per_s[1];
    out_velocity->elbow_deg_per_s = qdot_deg_per_s[2];
    out_velocity->wrist_pitch_deg_per_s = qdot_deg_per_s[3];
    out_velocity->wrist_roll_deg_per_s = qdot_deg_per_s[4];
    return ESP_OK;
}

}  // namespace learm
