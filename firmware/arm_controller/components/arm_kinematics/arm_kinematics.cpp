#include "arm_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "esp_log.h"

namespace learm {

static const char* TAG = "arm_kinematics";
static constexpr float kPi = 3.14159265358979323846f;
static constexpr float kMinLinkMm = 1.0f;
static constexpr float kMinDampingMm = 0.001f;
static constexpr float kMinGain = 0.0f;
static constexpr float kMaxGain = 5.0f;
static constexpr float kMinMaxErrorMm = 0.1f;
static constexpr float kMinMaxDeltaDeg = 0.01f;

bool ArmKinematics::is_finite(float value)
{
    return std::isfinite(value);
}

float ArmKinematics::deg_to_rad(float deg)
{
    return deg * (kPi / 180.0f);
}

float ArmKinematics::rad_to_deg(float rad)
{
    return rad * (180.0f / kPi);
}

float ArmKinematics::clamp_abs(float value, float limit_abs)
{
    const float limit = std::fabs(limit_abs);
    return std::clamp<float>(value, -limit, limit);
}

float ArmKinematics::clamp_error(float value, float limit_abs)
{
    return clamp_abs(value, limit_abs);
}

float ArmKinematics::dot3(const float a[3], const float b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

bool ArmKinematics::solve_3x3(const float a[3][3], const float b[3], float x[3])
{
    if (a == nullptr || b == nullptr || x == nullptr) {
        return false;
    }

    float m[3][4] = {
        {a[0][0], a[0][1], a[0][2], b[0]},
        {a[1][0], a[1][1], a[1][2], b[1]},
        {a[2][0], a[2][1], a[2][2], b[2]},
    };

    for (int col = 0; col < 3; ++col) {
        int pivot = col;
        float pivot_abs = std::fabs(m[col][col]);

        for (int row = col + 1; row < 3; ++row) {
            const float candidate = std::fabs(m[row][col]);
            if (candidate > pivot_abs) {
                pivot_abs = candidate;
                pivot = row;
            }
        }

        if (pivot_abs < 1.0e-9f) {
            return false;
        }

        if (pivot != col) {
            for (int k = col; k < 4; ++k) {
                std::swap(m[col][k], m[pivot][k]);
            }
        }

        const float div = m[col][col];
        for (int k = col; k < 4; ++k) {
            m[col][k] /= div;
        }

        for (int row = 0; row < 3; ++row) {
            if (row == col) {
                continue;
            }

            const float factor = m[row][col];
            for (int k = col; k < 4; ++k) {
                m[row][k] -= factor * m[col][k];
            }
        }
    }

    x[0] = m[0][3];
    x[1] = m[1][3];
    x[2] = m[2][3];
    return is_finite(x[0]) && is_finite(x[1]) && is_finite(x[2]);
}

float ArmKinematics::clamp_delta_to_joint_range(
    float current_deg,
    float delta_deg,
    float min_deg,
    float max_deg
) const
{
    if (!is_finite(current_deg) || !is_finite(delta_deg) ||
        !is_finite(min_deg) || !is_finite(max_deg) ||
        min_deg > max_deg) {
        return 0.0f;
    }

    const float target = current_deg + delta_deg;
    if (target < min_deg) {
        return min_deg - current_deg;
    }
    if (target > max_deg) {
        return max_deg - current_deg;
    }
    return delta_deg;
}

float ArmKinematics::scale_cde_to_limits(
    const ArmKinematicsJointState& joints,
    float dq_deg[3],
    bool* out_step_limited,
    bool* out_range_limited
) const
{
    if (out_step_limited != nullptr) {
        *out_step_limited = false;
    }
    if (out_range_limited != nullptr) {
        *out_range_limited = false;
    }

    const float step_limits[3] = {
        config_.max_delta_deg_shoulder,
        config_.max_delta_deg_elbow,
        config_.max_delta_deg_wrist_pitch,
    };
    const float currents[3] = {
        joints.shoulder_deg,
        joints.elbow_deg,
        joints.wrist_pitch_deg,
    };
    const float mins[3] = {
        config_.min_deg_shoulder,
        config_.min_deg_elbow,
        config_.min_deg_wrist_pitch,
    };
    const float maxs[3] = {
        config_.max_deg_shoulder,
        config_.max_deg_elbow,
        config_.max_deg_wrist_pitch,
    };

    float scale = 1.0f;

    for (int i = 0; i < 3; ++i) {
        const float abs_step = std::fabs(dq_deg[i]);
        const float step_limit = std::fabs(step_limits[i]);
        if (abs_step > step_limit && abs_step > 0.0f) {
            scale = std::min(scale, step_limit / abs_step);
            if (out_step_limited != nullptr) {
                *out_step_limited = true;
            }
        }
    }

    // Apply semantic hard ranges with the same vector scale. This is important:
    // independent clipping would break delta_shoulder + delta_elbow +
    // delta_wrist_pitch == 0, so posture hold would appear correct in the
    // solver output but fail after arm_motion clamps a saturated joint.
    for (int i = 0; i < 3; ++i) {
        const float d = dq_deg[i];
        if (d > 0.0f) {
            const float room = maxs[i] - currents[i];
            if (room <= 0.0f) {
                scale = 0.0f;
                if (out_range_limited != nullptr) {
                    *out_range_limited = true;
                }
            } else if (d * scale > room) {
                scale = std::min(scale, room / d);
                if (out_range_limited != nullptr) {
                    *out_range_limited = true;
                }
            }
        } else if (d < 0.0f) {
            const float room = currents[i] - mins[i];
            if (room <= 0.0f) {
                scale = 0.0f;
                if (out_range_limited != nullptr) {
                    *out_range_limited = true;
                }
            } else if ((-d) * scale > room) {
                scale = std::min(scale, room / (-d));
                if (out_range_limited != nullptr) {
                    *out_range_limited = true;
                }
            }
        }
    }

    scale = std::clamp<float>(scale, 0.0f, 1.0f);
    for (int i = 0; i < 3; ++i) {
        dq_deg[i] *= scale;
    }
    return scale;
}

esp_err_t ArmKinematics::validate_config(const ArmKinematicsConfig& config)
{
    if (!is_finite(config.base_to_shoulder_z_mm) ||
        !is_finite(config.upper_arm_mm) ||
        !is_finite(config.forearm_mm) ||
        !is_finite(config.wrist_to_tool_mm) ||
        config.upper_arm_mm < kMinLinkMm ||
        config.forearm_mm < kMinLinkMm ||
        config.wrist_to_tool_mm < 0.0f) {
        ESP_LOGE(TAG, "invalid link geometry");
        return ESP_ERR_INVALID_ARG;
    }

    if (!is_finite(config.base_sign) ||
        !is_finite(config.shoulder_sign) ||
        !is_finite(config.elbow_sign) ||
        !is_finite(config.wrist_pitch_sign) ||
        !is_finite(config.wrist_roll_sign) ||
        std::fabs(config.base_sign) < 0.001f ||
        std::fabs(config.shoulder_sign) < 0.001f ||
        std::fabs(config.elbow_sign) < 0.001f ||
        std::fabs(config.wrist_pitch_sign) < 0.001f ||
        std::fabs(config.wrist_roll_sign) < 0.001f) {
        ESP_LOGE(TAG, "invalid joint sign mapping");
        return ESP_ERR_INVALID_ARG;
    }

    if (!is_finite(config.base_offset_deg) ||
        !is_finite(config.shoulder_offset_deg) ||
        !is_finite(config.elbow_offset_deg) ||
        !is_finite(config.wrist_pitch_offset_deg) ||
        !is_finite(config.wrist_roll_offset_deg)) {
        ESP_LOGE(TAG, "invalid joint angle offsets");
        return ESP_ERR_INVALID_ARG;
    }

    if (!is_finite(config.min_deg_base) ||
        !is_finite(config.max_deg_base) ||
        !is_finite(config.min_deg_shoulder) ||
        !is_finite(config.max_deg_shoulder) ||
        !is_finite(config.min_deg_elbow) ||
        !is_finite(config.max_deg_elbow) ||
        !is_finite(config.min_deg_wrist_pitch) ||
        !is_finite(config.max_deg_wrist_pitch) ||
        config.min_deg_base > config.max_deg_base ||
        config.min_deg_shoulder > config.max_deg_shoulder ||
        config.min_deg_elbow > config.max_deg_elbow ||
        config.min_deg_wrist_pitch > config.max_deg_wrist_pitch) {
        ESP_LOGE(TAG, "invalid joint hard ranges");
        return ESP_ERR_INVALID_ARG;
    }

    if (!is_finite(config.lateral_gain) ||
        !is_finite(config.longitudinal_gain) ||
        !is_finite(config.up_gain) ||
        !is_finite(config.lateral_damping_mm) ||
        !is_finite(config.longitudinal_damping_mm) ||
        !is_finite(config.up_damping_mm) ||
        !is_finite(config.planar_hold_min_effect_mm2) ||
        !is_finite(config.relaxed_orientation_weight_mm) ||
        !is_finite(config.max_error_mm) ||
        !is_finite(config.max_delta_deg_base) ||
        !is_finite(config.max_delta_deg_shoulder) ||
        !is_finite(config.max_delta_deg_elbow) ||
        !is_finite(config.max_delta_deg_wrist_pitch) ||
        !is_finite(config.max_delta_deg_wrist_roll) ||
        config.lateral_gain < kMinGain ||
        config.lateral_gain > kMaxGain ||
        config.longitudinal_gain < kMinGain ||
        config.longitudinal_gain > kMaxGain ||
        config.up_gain < kMinGain ||
        config.up_gain > kMaxGain ||
        config.lateral_damping_mm < kMinDampingMm ||
        config.longitudinal_damping_mm < kMinDampingMm ||
        config.up_damping_mm < kMinDampingMm ||
        config.planar_hold_min_effect_mm2 < 0.0f ||
        config.relaxed_orientation_weight_mm < 0.0f ||
        config.max_error_mm < kMinMaxErrorMm ||
        config.max_delta_deg_base < kMinMaxDeltaDeg ||
        config.max_delta_deg_shoulder < kMinMaxDeltaDeg ||
        config.max_delta_deg_elbow < kMinMaxDeltaDeg ||
        config.max_delta_deg_wrist_pitch < kMinMaxDeltaDeg ||
        config.max_delta_deg_wrist_roll < kMinMaxDeltaDeg) {
        ESP_LOGE(TAG, "invalid solver configuration");
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

void ArmKinematics::log_config(const ArmKinematicsConfig& config, const char* action)
{
    const char* verb = (action != nullptr) ? action : "configured";
    ESP_LOGI(
        TAG,
        "%s: L=[%.1f %.1f %.1f]mm gains=[lat %.2f long %.2f up %.2f] damp=[%.1f %.1f %.1f] max_delta=[%.2f %.2f %.2f %.2f %.2f]deg",
        verb,
        static_cast<double>(config.upper_arm_mm),
        static_cast<double>(config.forearm_mm),
        static_cast<double>(config.wrist_to_tool_mm),
        static_cast<double>(config.lateral_gain),
        static_cast<double>(config.longitudinal_gain),
        static_cast<double>(config.up_gain),
        static_cast<double>(config.lateral_damping_mm),
        static_cast<double>(config.longitudinal_damping_mm),
        static_cast<double>(config.up_damping_mm),
        static_cast<double>(config.max_delta_deg_base),
        static_cast<double>(config.max_delta_deg_shoulder),
        static_cast<double>(config.max_delta_deg_elbow),
        static_cast<double>(config.max_delta_deg_wrist_pitch),
        static_cast<double>(config.max_delta_deg_wrist_roll)
    );


}

esp_err_t ArmKinematics::init(const ArmKinematicsConfig& config)
{
    const esp_err_t ret = validate_config(config);
    if (ret != ESP_OK) {
        return ret;
    }

    config_ = config;
    initialized_ = true;
    log_config(config_, "initialized");
    return ESP_OK;
}

esp_err_t ArmKinematics::set_config(const ArmKinematicsConfig& config)
{
    if (!initialized_) {
        ESP_LOGE(TAG, "set_config called before init");
        return ESP_ERR_INVALID_STATE;
    }

    const esp_err_t ret = validate_config(config);
    if (ret != ESP_OK) {
        return ret;
    }

    config_ = config;
    log_config(config_, "config updated");
    return ESP_OK;
}

esp_err_t ArmKinematics::set_uniform_delta_limit_deg(float limit_deg)
{
    return set_delta_limits_deg(limit_deg, limit_deg, limit_deg, limit_deg, limit_deg);
}

esp_err_t ArmKinematics::set_delta_limits_deg(
    float base_deg,
    float shoulder_deg,
    float elbow_deg,
    float wrist_pitch_deg,
    float wrist_roll_deg
)
{
    if (!initialized_) {
        ESP_LOGE(TAG, "set_delta_limits_deg called before init");
        return ESP_ERR_INVALID_STATE;
    }

    ArmKinematicsConfig next = config_;
    next.max_delta_deg_base = base_deg;
    next.max_delta_deg_shoulder = shoulder_deg;
    next.max_delta_deg_elbow = elbow_deg;
    next.max_delta_deg_wrist_pitch = wrist_pitch_deg;
    next.max_delta_deg_wrist_roll = wrist_roll_deg;
    return set_config(next);
}

bool ArmKinematics::is_initialized() const
{
    return initialized_;
}

ArmKinematicsConfig ArmKinematics::get_config() const
{
    return config_;
}

void ArmKinematics::compute_tool_axes(
    const ArmKinematicsJointState& joints,
    float forward[3],
    float left[3],
    float up[3],
    float pitch_axis[3]
) const
{
    const float base = deg_to_rad(
        joints.base_deg * config_.base_sign + config_.base_offset_deg
    );
    const float shoulder = deg_to_rad(
        joints.shoulder_deg * config_.shoulder_sign + config_.shoulder_offset_deg
    );
    const float elbow = deg_to_rad(
        joints.elbow_deg * config_.elbow_sign + config_.elbow_offset_deg
    );
    const float wrist_pitch = deg_to_rad(
        joints.wrist_pitch_deg * config_.wrist_pitch_sign + config_.wrist_pitch_offset_deg
    );

    const float pitch_sum = shoulder + elbow + wrist_pitch;

    const float cb = std::cos(base);
    const float sb = std::sin(base);
    const float cv = std::cos(pitch_sum);
    const float sv = std::sin(pitch_sum);

    // Tool/f-link frame axes expressed in robot base frame.
    // This intentionally ignores wrist_roll because the task is defined on the
    // final link f, not on a rolled camera image frame.
    if (forward != nullptr) {
        forward[0] = cb * sv;
        forward[1] = sb * sv;
        forward[2] = cv;
    }

    if (left != nullptr) {
        left[0] = -sb;
        left[1] =  cb;
        left[2] =  0.0f;
    }

    if (up != nullptr) {
        up[0] = -cb * cv;
        up[1] = -sb * cv;
        up[2] =  sv;
    }

    if (pitch_axis != nullptr) {
        pitch_axis[0] = -sb;
        pitch_axis[1] =  cb;
        pitch_axis[2] =  0.0f;
    }
}

void ArmKinematics::compute_position_and_jacobian(
    const ArmKinematicsJointState& joints,
    ArmCartesianPosition* out_position,
    float out_jacobian[3][4]
) const
{
    const float base = deg_to_rad(
        joints.base_deg * config_.base_sign + config_.base_offset_deg
    );
    const float shoulder = deg_to_rad(
        joints.shoulder_deg * config_.shoulder_sign + config_.shoulder_offset_deg
    );
    const float elbow = deg_to_rad(
        joints.elbow_deg * config_.elbow_sign + config_.elbow_offset_deg
    );
    const float wrist_pitch = deg_to_rad(
        joints.wrist_pitch_deg * config_.wrist_pitch_sign + config_.wrist_pitch_offset_deg
    );

    const float l1 = config_.upper_arm_mm;
    const float l2 = config_.forearm_mm;
    const float l3 = config_.wrist_to_tool_mm;

    const float a1 = shoulder;
    const float a2 = shoulder + elbow;
    const float a3 = shoulder + elbow + wrist_pitch;

    const float c0 = std::cos(base);
    const float s0 = std::sin(base);
    const float c1 = std::cos(a1);
    const float s1 = std::sin(a1);
    const float c2 = std::cos(a2);
    const float s2 = std::sin(a2);
    const float c3 = std::cos(a3);
    const float s3 = std::sin(a3);

    const float r = l1 * s1 + l2 * s2 + l3 * s3;
    const float z = config_.base_to_shoulder_z_mm + l1 * c1 + l2 * c2 + l3 * c3;

    if (out_position != nullptr) {
        out_position->x_mm = r * c0;
        out_position->y_mm = r * s0;
        out_position->z_mm = z;
    }

    if (out_jacobian == nullptr) {
        return;
    }

    // Jacobian columns are derivatives with respect to semantic joint radians.
    const float base_sign = config_.base_sign;
    const float shoulder_sign = config_.shoulder_sign;
    const float elbow_sign = config_.elbow_sign;
    const float wrist_sign = config_.wrist_pitch_sign;

    out_jacobian[0][0] = -r * s0 * base_sign;
    out_jacobian[1][0] =  r * c0 * base_sign;
    out_jacobian[2][0] =  0.0f;

    const float dr_dshoulder =  l1 * c1 + l2 * c2 + l3 * c3;
    const float dz_dshoulder = -l1 * s1 - l2 * s2 - l3 * s3;

    const float dr_delbow =  l2 * c2 + l3 * c3;
    const float dz_delbow = -l2 * s2 - l3 * s3;

    const float dr_dwrist =  l3 * c3;
    const float dz_dwrist = -l3 * s3;

    out_jacobian[0][1] = dr_dshoulder * c0 * shoulder_sign;
    out_jacobian[1][1] = dr_dshoulder * s0 * shoulder_sign;
    out_jacobian[2][1] = dz_dshoulder * shoulder_sign;

    out_jacobian[0][2] = dr_delbow * c0 * elbow_sign;
    out_jacobian[1][2] = dr_delbow * s0 * elbow_sign;
    out_jacobian[2][2] = dz_delbow * elbow_sign;

    out_jacobian[0][3] = dr_dwrist * c0 * wrist_sign;
    out_jacobian[1][3] = dr_dwrist * s0 * wrist_sign;
    out_jacobian[2][3] = dz_dwrist * wrist_sign;
}

esp_err_t ArmKinematics::forward_position(
    const ArmKinematicsJointState& joints,
    ArmCartesianPosition* out_position
) const
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    if (out_position == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!is_finite(joints.base_deg) ||
        !is_finite(joints.shoulder_deg) ||
        !is_finite(joints.elbow_deg) ||
        !is_finite(joints.wrist_pitch_deg)) {
        return ESP_ERR_INVALID_ARG;
    }

    compute_position_and_jacobian(joints, out_position, nullptr);
    return ESP_OK;
}

esp_err_t ArmKinematics::solve_tool_target_delta(
    const ArmKinematicsJointState& joints,
    const ArmToolTargetError& target,
    ArmKinematicsDelta* out_delta
) const
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    if (out_delta == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!is_finite(joints.base_deg) ||
        !is_finite(joints.shoulder_deg) ||
        !is_finite(joints.elbow_deg) ||
        !is_finite(joints.wrist_pitch_deg) ||
        !is_finite(joints.wrist_roll_deg) ||
        !is_finite(target.forward_mm) ||
        !is_finite(target.left_mm) ||
        !is_finite(target.up_mm) ||
        !is_finite(target.desired_forward_mm)) {
        return ESP_ERR_INVALID_ARG;
    }

    ArmCartesianPosition tool_pos;
    float jpos[3][4] = {};
    compute_position_and_jacobian(joints, &tool_pos, jpos);

    float forward[3] = {};
    float left[3] = {};
    float up[3] = {};
    compute_tool_axes(joints, forward, left, up, nullptr);

    const float target_forward = clamp_error(target.forward_mm, config_.max_error_mm);
    const float target_left = clamp_error(target.left_mm, config_.max_error_mm);
    const float target_up = clamp_error(target.up_mm, config_.max_error_mm);
    const float desired_forward = target.desired_forward_mm;

    const float target_base[3] = {
        tool_pos.x_mm + target_forward * forward[0] + target_left * left[0] + target_up * up[0],
        tool_pos.y_mm + target_forward * forward[1] + target_left * left[1] + target_up * up[1],
        tool_pos.z_mm + target_forward * forward[2] + target_left * left[2] + target_up * up[2],
    };

    const float base_rad = deg_to_rad(
        joints.base_deg * config_.base_sign + config_.base_offset_deg
    );
    const float cb = std::cos(base_rad);
    const float sb = std::sin(base_rad);
    const float plane_forward[3] = {cb, sb, 0.0f};

    // Task 1: horizontal lateral error. Only base yaw can remove this error.
    const float e_left = dot3(left, target_base);
    const float x_plane = dot3(plane_forward, target_base);
    const float j_left_base = -x_plane * config_.base_sign;
    const float b_left = -config_.lateral_gain * e_left;
    const float lambda_left2 = config_.lateral_damping_mm * config_.lateral_damping_mm;
    float dq_base_rad = 0.0f;
    const float denom_left = j_left_base * j_left_base + lambda_left2;
    if (denom_left > 0.0f) {
        dq_base_rad = (j_left_base * b_left) / denom_left;
    }

    // Tool-frame residuals in the current pitch plane. These are equivalent to
    // target_up and target_forward but recomputed from the base target point to
    // keep the Jacobian signs clear.
    const float r_tool_to_target[3] = {
        target_base[0] - tool_pos.x_mm,
        target_base[1] - tool_pos.y_mm,
        target_base[2] - tool_pos.z_mm,
    };
    const float distance_along_f = dot3(forward, r_tool_to_target);
    const float e_up = dot3(up, r_tool_to_target);
    const float e_forward = distance_along_f - desired_forward;

    const float theta_sign[3] = {
        config_.shoulder_sign,
        config_.elbow_sign,
        config_.wrist_pitch_sign,
    };

    // Derivative rows for C/D/E semantic radians.
    // e_up      = up^T      (p_target - p_tool)
    // e_forward = forward^T (p_target - p_tool) - desired
    // p_target is fixed during this incremental step.
    float j_up[3] = {};
    float j_forward[3] = {};
    for (int k = 0; k < 3; ++k) {
        const int joint_col = k + 1;
        const float jcol[3] = {
            jpos[0][joint_col],
            jpos[1][joint_col],
            jpos[2][joint_col],
        };

        j_up[k] = distance_along_f * theta_sign[k] - dot3(up, jcol);
        j_forward[k] = -e_up * theta_sign[k] - dot3(forward, jcol);
    }

    const float b_forward = -config_.longitudinal_gain * e_forward;
    const float b_up = -config_.up_gain * e_up;
    const float lambda_forward2 = config_.longitudinal_damping_mm * config_.longitudinal_damping_mm;
    const float lambda_up2 = config_.up_damping_mm * config_.up_damping_mm;

    const float step_limits_deg[3] = {
        config_.max_delta_deg_shoulder,
        config_.max_delta_deg_elbow,
        config_.max_delta_deg_wrist_pitch,
    };
    const float currents_deg[3] = {
        joints.shoulder_deg,
        joints.elbow_deg,
        joints.wrist_pitch_deg,
    };
    const float mins_deg[3] = {
        config_.min_deg_shoulder,
        config_.min_deg_elbow,
        config_.min_deg_wrist_pitch,
    };
    const float maxs_deg[3] = {
        config_.max_deg_shoulder,
        config_.max_deg_elbow,
        config_.max_deg_wrist_pitch,
    };

    auto solve_relaxed_active_set = [&](float out_rad[3], bool* out_bound_limited) -> bool {
        if (out_rad == nullptr) {
            return false;
        }
        if (out_bound_limited != nullptr) {
            *out_bound_limited = false;
        }

        const float w = config_.relaxed_orientation_weight_mm;
        const float rows[3][3] = {
            {j_forward[0], j_forward[1], j_forward[2]},
            {j_up[0], j_up[1], j_up[2]},
            {w * theta_sign[0], w * theta_sign[1], w * theta_sign[2]},
        };
        const float rhs_rows[3] = {b_forward, b_up, 0.0f};
        const float lambda_relaxed2 = 0.5f * (lambda_forward2 + lambda_up2);

        float lo[3] = {};
        float hi[3] = {};
        for (int i = 0; i < 3; ++i) {
            const float lower_deg = std::max(
                mins_deg[i] - currents_deg[i],
                -std::fabs(step_limits_deg[i])
            );
            const float upper_deg = std::min(
                maxs_deg[i] - currents_deg[i],
                std::fabs(step_limits_deg[i])
            );
            if (lower_deg <= upper_deg) {
                lo[i] = deg_to_rad(lower_deg);
                hi[i] = deg_to_rad(upper_deg);
            } else {
                // Current state is outside the configured hard range. Do not
                // make it worse from this solver call.
                lo[i] = 0.0f;
                hi[i] = 0.0f;
            }
        }

        bool fixed[3] = {false, false, false};
        float fixed_value[3] = {0.0f, 0.0f, 0.0f};
        float solution[3] = {0.0f, 0.0f, 0.0f};

        for (int iter = 0; iter < 4; ++iter) {
            int free_idx[3] = {-1, -1, -1};
            int n_free = 0;
            for (int i = 0; i < 3; ++i) {
                if (!fixed[i]) {
                    free_idx[n_free++] = i;
                }
            }

            for (int i = 0; i < 3; ++i) {
                solution[i] = fixed[i] ? fixed_value[i] : 0.0f;
            }

            if (n_free > 0) {
                float residual[3] = {rhs_rows[0], rhs_rows[1], rhs_rows[2]};
                for (int row = 0; row < 3; ++row) {
                    for (int i = 0; i < 3; ++i) {
                        if (fixed[i]) {
                            residual[row] -= rows[row][i] * fixed_value[i];
                        }
                    }
                }

                float a[3][3] = {};
                float b[3] = {};
                for (int r = 0; r < n_free; ++r) {
                    const int ir = free_idx[r];
                    for (int row = 0; row < 3; ++row) {
                        b[r] += rows[row][ir] * residual[row];
                    }
                    for (int c = 0; c < n_free; ++c) {
                        const int ic = free_idx[c];
                        for (int row = 0; row < 3; ++row) {
                            a[r][c] += rows[row][ir] * rows[row][ic];
                        }
                    }
                    a[r][r] += lambda_relaxed2;
                }

                float x[3] = {};
                bool ok = false;
                if (n_free == 1) {
                    if (std::fabs(a[0][0]) > 1.0e-9f) {
                        x[0] = b[0] / a[0][0];
                        ok = is_finite(x[0]);
                    }
                } else if (n_free == 2) {
                    const float det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
                    if (std::fabs(det) > 1.0e-9f) {
                        x[0] = ( b[0] * a[1][1] - a[0][1] * b[1]) / det;
                        x[1] = (-a[1][0] * b[0] + a[0][0] * b[1]) / det;
                        ok = is_finite(x[0]) && is_finite(x[1]);
                    }
                } else {
                    ok = solve_3x3(a, b, x);
                }

                if (!ok) {
                    return false;
                }

                for (int r = 0; r < n_free; ++r) {
                    solution[free_idx[r]] = x[r];
                }
            }

            int fix_index = -1;
            float fix_value = 0.0f;
            float max_violation = 0.0f;
            constexpr float kBoundTol = 1.0e-6f;
            for (int i = 0; i < 3; ++i) {
                if (fixed[i]) {
                    continue;
                }
                if (solution[i] < lo[i] - kBoundTol) {
                    const float violation = lo[i] - solution[i];
                    if (violation > max_violation) {
                        max_violation = violation;
                        fix_index = i;
                        fix_value = lo[i];
                    }
                } else if (solution[i] > hi[i] + kBoundTol) {
                    const float violation = solution[i] - hi[i];
                    if (violation > max_violation) {
                        max_violation = violation;
                        fix_index = i;
                        fix_value = hi[i];
                    }
                }
            }

            if (fix_index < 0) {
                for (int i = 0; i < 3; ++i) {
                    out_rad[i] = std::clamp<float>(solution[i], lo[i], hi[i]);
                }
                return is_finite(out_rad[0]) && is_finite(out_rad[1]) && is_finite(out_rad[2]);
            }

            fixed[fix_index] = true;
            fixed_value[fix_index] = fix_value;
            if (out_bound_limited != nullptr) {
                *out_bound_limited = true;
            }
        }

        for (int i = 0; i < 3; ++i) {
            out_rad[i] = fixed[i]
                ? fixed_value[i]
                : std::clamp<float>(solution[i], lo[i], hi[i]);
        }
        return is_finite(out_rad[0]) && is_finite(out_rad[1]) && is_finite(out_rad[2]);
    };

    // Task 2: pitch-plane translation. Forward and up errors are first solved
    // inside the null space of theta_f = shoulder + elbow + wrist_pitch. If
    // that posture-preserving step would run into a semantic hard joint range,
    // switch to a relaxed active-set solve instead of letting arm_motion clip a
    // single joint and silently break posture hold.
    const float s_norm2 = theta_sign[0] * theta_sign[0] +
                          theta_sign[1] * theta_sign[1] +
                          theta_sign[2] * theta_sign[2];

    float hold_rad[3] = {};
    bool hold_valid = false;
    float min_effect = 0.0f;

    if (s_norm2 > 0.0f) {
        float v_forward_hold[3] = {};
        float v_up_hold[3] = {};

        const float dot_forward_s = j_forward[0] * theta_sign[0] +
                                    j_forward[1] * theta_sign[1] +
                                    j_forward[2] * theta_sign[2];
        const float dot_up_s = j_up[0] * theta_sign[0] +
                               j_up[1] * theta_sign[1] +
                               j_up[2] * theta_sign[2];
        for (int k = 0; k < 3; ++k) {
            v_forward_hold[k] = j_forward[k] - theta_sign[k] * (dot_forward_s / s_norm2);
            v_up_hold[k] = j_up[k] - theta_sign[k] * (dot_up_s / s_norm2);
        }

        const float s00 = j_forward[0] * v_forward_hold[0] +
                          j_forward[1] * v_forward_hold[1] +
                          j_forward[2] * v_forward_hold[2];
        const float s01 = j_forward[0] * v_up_hold[0] +
                          j_forward[1] * v_up_hold[1] +
                          j_forward[2] * v_up_hold[2];
        const float s11 = j_up[0] * v_up_hold[0] +
                          j_up[1] * v_up_hold[1] +
                          j_up[2] * v_up_hold[2];

        const float trace = s00 + s11;
        const float eig_disc = std::max(
            0.0f,
            (s00 - s11) * (s00 - s11) + 4.0f * s01 * s01
        );
        min_effect = 0.5f * (trace - std::sqrt(eig_disc));

        if (min_effect >= config_.planar_hold_min_effect_mm2) {
            const float a00 = s00 + lambda_forward2;
            const float a01 = s01;
            const float a11 = s11 + lambda_up2;
            const float det = a00 * a11 - a01 * a01;

            if (std::fabs(det) > 1.0e-9f) {
                const float y_forward = ( a11 * b_forward - a01 * b_up) / det;
                const float y_up      = (-a01 * b_forward + a00 * b_up) / det;

                for (int k = 0; k < 3; ++k) {
                    hold_rad[k] = v_forward_hold[k] * y_forward + v_up_hold[k] * y_up;
                }
                hold_valid = is_finite(hold_rad[0]) && is_finite(hold_rad[1]) && is_finite(hold_rad[2]);
            }
        }
    }

    float dq_cde_deg[3] = {};
    bool used_relaxed = false;
    const char* relax_reason = "none";

    if (hold_valid) {
        float hold_deg[3] = {
            rad_to_deg(hold_rad[0]),
            rad_to_deg(hold_rad[1]),
            rad_to_deg(hold_rad[2]),
        };

        bool hold_step_limited = false;
        bool hold_range_limited = false;
        const float hold_scale = scale_cde_to_limits(
            joints,
            hold_deg,
            &hold_step_limited,
            &hold_range_limited
        );

        if (!hold_range_limited) {
            dq_cde_deg[0] = hold_deg[0];
            dq_cde_deg[1] = hold_deg[1];
            dq_cde_deg[2] = hold_deg[2];
        } else {
            used_relaxed = true;
            relax_reason = "hold_range";
            ESP_LOGI(
                TAG,
                "jtool switch relaxed: reason=%s hold_scale=%.3f q=[%.2f %.2f %.2f] hold_dq=[%.3f %.3f %.3f]",
                relax_reason,
                static_cast<double>(hold_scale),
                static_cast<double>(joints.shoulder_deg),
                static_cast<double>(joints.elbow_deg),
                static_cast<double>(joints.wrist_pitch_deg),
                static_cast<double>(hold_deg[0]),
                static_cast<double>(hold_deg[1]),
                static_cast<double>(hold_deg[2])
            );
        }
    } else {
        used_relaxed = true;
        relax_reason = (min_effect < config_.planar_hold_min_effect_mm2) ? "hold_weak" : "hold_invalid";
    }

    if (used_relaxed) {
        float relaxed_rad[3] = {};
        bool relaxed_bound_limited = false;
        if (solve_relaxed_active_set(relaxed_rad, &relaxed_bound_limited)) {
            dq_cde_deg[0] = rad_to_deg(relaxed_rad[0]);
            dq_cde_deg[1] = rad_to_deg(relaxed_rad[1]);
            dq_cde_deg[2] = rad_to_deg(relaxed_rad[2]);
            if (relaxed_bound_limited) {
                ESP_LOGI(
                    TAG,
                    "jtool relaxed bound-limited: reason=%s dq=[%.3f %.3f %.3f] q=[%.2f %.2f %.2f]",
                    relax_reason,
                    static_cast<double>(dq_cde_deg[0]),
                    static_cast<double>(dq_cde_deg[1]),
                    static_cast<double>(dq_cde_deg[2]),
                    static_cast<double>(joints.shoulder_deg),
                    static_cast<double>(joints.elbow_deg),
                    static_cast<double>(joints.wrist_pitch_deg)
                );
            }
        } else {
            dq_cde_deg[0] = 0.0f;
            dq_cde_deg[1] = 0.0f;
            dq_cde_deg[2] = 0.0f;
            ESP_LOGI(TAG, "jtool relaxed solve failed: reason=%s", relax_reason);
        }
    }

    float dq_base_deg = clamp_abs(rad_to_deg(dq_base_rad), config_.max_delta_deg_base);
    dq_base_deg = clamp_delta_to_joint_range(
        joints.base_deg,
        dq_base_deg,
        config_.min_deg_base,
        config_.max_deg_base
    );

    out_delta->base_delta_deg = dq_base_deg;
    out_delta->shoulder_delta_deg = dq_cde_deg[0];
    out_delta->elbow_delta_deg = dq_cde_deg[1];
    out_delta->wrist_pitch_delta_deg = dq_cde_deg[2];
    out_delta->wrist_roll_delta_deg = 0.0f;

    if (!is_finite(out_delta->base_delta_deg) ||
        !is_finite(out_delta->shoulder_delta_deg) ||
        !is_finite(out_delta->elbow_delta_deg) ||
        !is_finite(out_delta->wrist_pitch_delta_deg) ||
        !is_finite(out_delta->wrist_roll_delta_deg)) {
        *out_delta = {};
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

}  // namespace learm
