#include "arm_joint.hpp"

#include <algorithm>
#include <cmath>

#include "driver/gpio.h"
#include "esp_log.h"

namespace learm {

static const char* TAG = "arm_joint";

namespace arm_joint_defaults {

static constexpr float kDefaultUsPerDegree = 2000.0f / 180.0f;

const ServoPwmConfig kServoPwmConfigs[] = {
    // S0: Claw
    //
    // 实测夹爪爪距标定：
    //   500us  = 最大张开，爪距约 5.8cm
    //   700us  = 爪距约 5.6cm
    //   900us  = 爪距约 5.0cm
    //   1050us = 爪距约 4.4cm
    //   1150us = 爪距约 3.6cm
    //   1200us = 爪距约 2.9cm
    //   1250us = 爪距约 2.8cm
    //   1300us = 爪距约 2.4cm
    //   1400us = 爪距约 1.6cm
    //   1500us = 爪距约 0.6cm
    //   1600us = 安全闭合，爪距约 0cm
    //
    // 夹爪不使用完整 500~2500us 范围，避免过度闭合堵转。
    {ServoChannel::S0, GPIO_NUM_19,   500,   500, 1600, 0},

    // S1: WristRoll
    {ServoChannel::S1, GPIO_NUM_18,  1480,   500, 2500, 0},

    // S2: WristPitch
    {ServoChannel::S2, GPIO_NUM_5,   1580,   500, 2500, 0},

    // S3: Elbow
    {ServoChannel::S3, GPIO_NUM_4,   1490,  500, 2500, 0},

    // S4: Shoulder
    {ServoChannel::S4, GPIO_NUM_0,   1530,  500, 2500, 0},

    // S5: Base
    //
    // 基座已重新机械装配并实测：
    //   1450us = 正前方 0deg
    //
    // 使用普通 PWM 舵机完整范围：
    //   500us ~ 2500us
    {ServoChannel::S5, GPIO_NUM_15,  1450,   500, 2500, 0},
};

const uint8_t kServoPwmConfigCount =
    sizeof(kServoPwmConfigs) / sizeof(kServoPwmConfigs[0]);

const JointCalibration kJointCalibrations[] = {
    // Claw:
    //   坐标规范：
    //     0    = 最大张开，爪距约 5.8cm
    //     100  = 安全闭合，爪距约 0cm
    //
    //   注意：这里的 deg 对夹爪不是严格物理角度，
    //   而是 0~100 的抽象闭合量。
    //
    // 实测：
    //   0      = 500us
    //   50     = 1050us
    //   100    = 1600us
    //
    // us_per_unit = (1600 - 500) / 100 = 11.0
    {
        ArmJoint::Claw,
        ServoChannel::S0,
        0.0f,
        500,
        11.0f,
        +1,
        0.0f,
        100.0f
    },

    // WristRoll:
    //   当前沿用旧标定。
    //   +deg 的顺/逆方向后续再实测确认。
    {
        ArmJoint::WristRoll,
        ServoChannel::S1,
        0.0f,
        1480,
        kDefaultUsPerDegree,
        -1,
        -85.0f,
        85.0f
    },

    // WristPitch:
    //   坐标规范：
    //     +deg = 腕部前倾 / 下压
    //     -deg = 腕部后仰 / 上抬
    //
    //   当前仍是旧基础标定。
    //   之前实测接近 +90deg 时容易抖，所以 max_deg 暂收为 +85deg。
    {
        ArmJoint::WristPitch,
        ServoChannel::S2,
        0.0f,
        1580,
        kDefaultUsPerDegree,
        -1,
        -85.0f,
        85.0f
    },

    // Elbow:
    //   坐标规范：
    //     +deg = 小臂前倾 / 收肘
    //     -deg = 小臂后伸
    //
    //   当前仍是旧基础标定。
    {
        ArmJoint::Elbow,
        ServoChannel::S3,
        0.0f,
        1490,
        kDefaultUsPerDegree,
        -1,
        -85.0f,
        85.0f
    },

    // Shoulder:
    //   坐标规范：
    //     +deg = 大臂前倾
    //     -deg = 大臂后仰
    //
    //   当前仍是旧基础标定。
    {
        ArmJoint::Shoulder,
        ServoChannel::S4,
        0.0f,
        1530,
        kDefaultUsPerDegree,
        -1,
        -85.0f,
        85.0f
    },

    // Base:
    //   坐标规范：
    //     0deg  = 正前方
    //     +deg  = 左转
    //     -deg  = 右转
    //
    //   实测标定：
    //     0deg    = 1450us
    //     +45deg  ≈ 1940us
    //     +90deg  ≈ 2430us
    //     -45deg  ≈  960us
    //
    //   us_per_degree = (2430 - 1450) / 90 ≈ 10.89
    //
    //   理论 -90deg ≈ 470us，略低于 500us。
    //   所以软件范围先收为 -85deg ~ +85deg。
    {
        ArmJoint::Base,
        ServoChannel::S5,
        0.0f,
        1450,
        10.89f,
        +1,
        -85.0f,
        85.0f
    },
};

const uint8_t kJointCalibrationCount =
    sizeof(kJointCalibrations) / sizeof(kJointCalibrations[0]);

const ClawGapPoint kClawGapTable[] = {
    {500,  5.8f},
    {700,  5.6f},
    {900,  5.0f},
    {1050, 4.4f},
    {1150, 3.6f},
    {1200, 2.9f},
    {1250, 2.8f},
    {1300, 2.4f},
    {1400, 1.6f},
    {1500, 0.6f},
    {1600, 0.0f},
};

const uint8_t kClawGapTableCount =
    sizeof(kClawGapTable) / sizeof(kClawGapTable[0]);

}  // namespace arm_joint_defaults

int ArmJointController::joint_to_index(ArmJoint joint)
{
    const uint8_t index = static_cast<uint8_t>(joint);

    if (index >= kArmJointCount) {
        return -1;
    }

    return static_cast<int>(index);
}

bool ArmJointController::is_valid_joint_index(int index) const
{
    return index >= 0 &&
           index < static_cast<int>(kArmJointCount) &&
           joints_[index].configured;
}

esp_err_t ArmJointController::normalize_calibration(
    JointCalibration* calibration
) const
{
    if (calibration == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    const int joint_index = joint_to_index(calibration->joint);
    if (joint_index < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t channel_index =
        static_cast<uint8_t>(calibration->channel);

    if (channel_index >= kServoCount) {
        return ESP_ERR_INVALID_ARG;
    }

    if (calibration->us_per_degree <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    if (calibration->min_deg > calibration->max_deg) {
        return ESP_ERR_INVALID_ARG;
    }

    calibration->direction =
        (calibration->direction >= 0) ? 1 : -1;

    calibration->zero_deg = std::clamp<float>(
        calibration->zero_deg,
        calibration->min_deg,
        calibration->max_deg
    );

    if (servo_ != nullptr) {
        const ServoRuntimeState servo_state =
            servo_->get_state(calibration->channel);

        if (!servo_state.configured) {
            ESP_LOGE(
                TAG,
                "calibration rejected: servo channel S%u is not configured",
                static_cast<unsigned>(channel_index)
            );
            return ESP_ERR_INVALID_ARG;
        }

        if (calibration->zero_us < servo_state.min_us ||
            calibration->zero_us > servo_state.max_us) {
            ESP_LOGE(
                TAG,
                "calibration rejected: zero_us=%u outside servo range [%u,%u]",
                calibration->zero_us,
                servo_state.min_us,
                servo_state.max_us
            );
            return ESP_ERR_INVALID_ARG;
        }
    }

    return ESP_OK;
}

esp_err_t ArmJointController::init(
    ServoPwm* servo_pwm,
    const JointCalibration* calibrations,
    uint8_t count
)
{
    if (servo_pwm == nullptr ||
        calibrations == nullptr ||
        count == 0 ||
        count > kArmJointCount) {
        return ESP_ERR_INVALID_ARG;
    }

    servo_ = servo_pwm;

    for (uint8_t i = 0; i < kArmJointCount; ++i) {
        joints_[i] = JointState{};
    }

    for (uint8_t i = 0; i < count; ++i) {
        JointCalibration cal = calibrations[i];

        esp_err_t ret = normalize_calibration(&cal);
        if (ret != ESP_OK) {
            ESP_LOGE(
                TAG,
                "invalid calibration at index=%u",
                static_cast<unsigned>(i)
            );
            return ret;
        }

        const int joint_index = joint_to_index(cal.joint);
        if (joint_index < 0) {
            return ESP_ERR_INVALID_ARG;
        }

        if (joints_[joint_index].configured) {
            ESP_LOGE(
                TAG,
                "duplicate joint calibration index=%d",
                joint_index
            );
            return ESP_ERR_INVALID_ARG;
        }

        joints_[joint_index].configured = true;
        joints_[joint_index].calibration = cal;
        joints_[joint_index].default_options = JointMotionOptions{};

        ESP_LOGI(
            TAG,
            "configured joint=%u -> S%u zero_deg=%.2f zero_us=%u us_per_deg=%.4f dir=%d range=[%.2f,%.2f]",
            static_cast<unsigned>(cal.joint),
            static_cast<unsigned>(cal.channel),
            static_cast<double>(cal.zero_deg),
            cal.zero_us,
            static_cast<double>(cal.us_per_degree),
            static_cast<int>(cal.direction),
            static_cast<double>(cal.min_deg),
            static_cast<double>(cal.max_deg)
        );
    }

    initialized_ = true;

    ESP_LOGI(TAG, "arm_joint initialized");

    return ESP_OK;
}

bool ArmJointController::is_initialized() const
{
    return initialized_;
}

bool ArmJointController::is_configured(ArmJoint joint) const
{
    const int index = joint_to_index(joint);

    return is_valid_joint_index(index);
}

float ArmJointController::clamp_deg_by_index(uint8_t index, float deg) const
{
    if (index >= kArmJointCount || !joints_[index].configured) {
        return deg;
    }

    const JointCalibration& cal = joints_[index].calibration;

    return std::clamp<float>(
        deg,
        cal.min_deg,
        cal.max_deg
    );
}

uint16_t ArmJointController::deg_to_us_by_index(uint8_t index, float deg) const
{
    if (index >= kArmJointCount || !joints_[index].configured) {
        return 0;
    }

    const JointCalibration& cal = joints_[index].calibration;

    deg = clamp_deg_by_index(index, deg);

    const float delta_deg = deg - cal.zero_deg;

    const float pulse =
        static_cast<float>(cal.zero_us) +
        static_cast<float>(cal.direction) *
            delta_deg *
            cal.us_per_degree;

    int pulse_i = static_cast<int>(std::lround(pulse));

    if (servo_ != nullptr) {
        const ServoRuntimeState servo_state =
            servo_->get_state(cal.channel);

        if (servo_state.configured) {
            pulse_i = std::clamp<int>(
                pulse_i,
                servo_state.min_us,
                servo_state.max_us
            );
        }
    }

    pulse_i = std::clamp<int>(
        pulse_i,
        kServoMinUs,
        kServoMaxUs
    );

    return static_cast<uint16_t>(pulse_i);
}

float ArmJointController::us_to_deg_by_index(
    uint8_t index,
    uint16_t pulse_us
) const
{
    if (index >= kArmJointCount || !joints_[index].configured) {
        return 0.0f;
    }

    const JointCalibration& cal = joints_[index].calibration;

    const float denom =
        static_cast<float>(cal.direction) *
        cal.us_per_degree;

    if (std::fabs(denom) < 0.0001f) {
        return cal.zero_deg;
    }

    const float deg =
        cal.zero_deg +
        (static_cast<float>(pulse_us) - static_cast<float>(cal.zero_us)) /
            denom;

    return std::clamp<float>(
        deg,
        cal.min_deg,
        cal.max_deg
    );
}

uint16_t ArmJointController::deg_to_us(ArmJoint joint, float deg) const
{
    const int index = joint_to_index(joint);
    if (!is_valid_joint_index(index)) {
        return 0;
    }

    return deg_to_us_by_index(static_cast<uint8_t>(index), deg);
}

float ArmJointController::us_to_deg(
    ArmJoint joint,
    uint16_t pulse_us
) const
{
    const int index = joint_to_index(joint);
    if (!is_valid_joint_index(index)) {
        return 0.0f;
    }

    return us_to_deg_by_index(
        static_cast<uint8_t>(index),
        pulse_us
    );
}

uint16_t ArmJointController::claw_gap_cm_to_us(float gap_cm) const
{
    return claw_gap_cm_to_us_from_table(gap_cm);
}

float ArmJointController::claw_us_to_gap_cm(uint16_t pulse_us) const
{
    return claw_us_to_gap_cm_from_table(pulse_us);
}

uint16_t ArmJointController::claw_gap_cm_to_us_from_table(float gap_cm) const
{
    const ClawGapPoint* table = arm_joint_defaults::kClawGapTable;
    const uint8_t count = arm_joint_defaults::kClawGapTableCount;

    if (table == nullptr || count == 0) {
        return 0;
    }

    if (count == 1) {
        return table[0].pulse_us;
    }

    const float max_gap = table[0].gap_cm;
    const float min_gap = table[count - 1].gap_cm;

    gap_cm = std::clamp<float>(gap_cm, min_gap, max_gap);

    if (gap_cm >= table[0].gap_cm) {
        return table[0].pulse_us;
    }

    if (gap_cm <= table[count - 1].gap_cm) {
        return table[count - 1].pulse_us;
    }

    for (uint8_t i = 0; i + 1 < count; ++i) {
        const ClawGapPoint& a = table[i];
        const ClawGapPoint& b = table[i + 1];

        // 表格按 pulse_us 递增、gap_cm 递减排列。
        if (gap_cm <= a.gap_cm && gap_cm >= b.gap_cm) {
            const float denom = b.gap_cm - a.gap_cm;

            if (std::fabs(denom) < 0.0001f) {
                return a.pulse_us;
            }

            const float ratio =
                (gap_cm - a.gap_cm) / denom;

            const float pulse =
                static_cast<float>(a.pulse_us) +
                ratio *
                    static_cast<float>(
                        static_cast<int>(b.pulse_us) -
                        static_cast<int>(a.pulse_us)
                    );

            int pulse_i = static_cast<int>(std::lround(pulse));

            pulse_i = std::clamp<int>(
                pulse_i,
                table[0].pulse_us,
                table[count - 1].pulse_us
            );

            return static_cast<uint16_t>(pulse_i);
        }
    }

    return table[count - 1].pulse_us;
}

float ArmJointController::claw_us_to_gap_cm_from_table(uint16_t pulse_us) const
{
    const ClawGapPoint* table = arm_joint_defaults::kClawGapTable;
    const uint8_t count = arm_joint_defaults::kClawGapTableCount;

    if (table == nullptr || count == 0) {
        return 0.0f;
    }

    if (count == 1) {
        return table[0].gap_cm;
    }

    pulse_us = std::clamp<uint16_t>(
        pulse_us,
        table[0].pulse_us,
        table[count - 1].pulse_us
    );

    if (pulse_us <= table[0].pulse_us) {
        return table[0].gap_cm;
    }

    if (pulse_us >= table[count - 1].pulse_us) {
        return table[count - 1].gap_cm;
    }

    for (uint8_t i = 0; i + 1 < count; ++i) {
        const ClawGapPoint& a = table[i];
        const ClawGapPoint& b = table[i + 1];

        if (pulse_us >= a.pulse_us && pulse_us <= b.pulse_us) {
            const float denom =
                static_cast<float>(
                    static_cast<int>(b.pulse_us) -
                    static_cast<int>(a.pulse_us)
                );

            if (std::fabs(denom) < 0.0001f) {
                return a.gap_cm;
            }

            const float ratio =
                static_cast<float>(
                    static_cast<int>(pulse_us) -
                    static_cast<int>(a.pulse_us)
                ) / denom;

            return a.gap_cm + ratio * (b.gap_cm - a.gap_cm);
        }
    }

    return table[count - 1].gap_cm;
}

float ArmJointController::read_claw_gap_cm() const
{
    return claw_us_to_gap_cm(
        read_command_us(ArmJoint::Claw)
    );
}

float ArmJointController::read_claw_target_gap_cm() const
{
    return claw_us_to_gap_cm(
        read_target_us(ArmJoint::Claw)
    );
}

esp_err_t ArmJointController::enable_joint(ArmJoint joint)
{
    const int index = joint_to_index(joint);
    if (!initialized_ || !is_valid_joint_index(index)) {
        return ESP_ERR_INVALID_ARG;
    }

    const JointCalibration& cal = joints_[index].calibration;

    return servo_->enable(
        cal.channel,
        cal.zero_us
    );
}

esp_err_t ArmJointController::enable_joint(
    ArmJoint joint,
    float start_deg
)
{
    const int index = joint_to_index(joint);
    if (!initialized_ || !is_valid_joint_index(index)) {
        return ESP_ERR_INVALID_ARG;
    }

    const JointCalibration& cal = joints_[index].calibration;

    const uint16_t start_us =
        deg_to_us_by_index(static_cast<uint8_t>(index), start_deg);

    return servo_->enable(
        cal.channel,
        start_us
    );
}

esp_err_t ArmJointController::disable_joint(ArmJoint joint)
{
    const int index = joint_to_index(joint);
    if (!initialized_ || !is_valid_joint_index(index)) {
        return ESP_ERR_INVALID_ARG;
    }

    const JointCalibration& cal = joints_[index].calibration;

    return servo_->disable(cal.channel);
}

esp_err_t ArmJointController::disable_all()
{
    if (!initialized_ || servo_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    for (uint8_t i = 0; i < kArmJointCount; ++i) {
        if (!joints_[i].configured) {
            continue;
        }

        esp_err_t ret = servo_->disable(
            joints_[i].calibration.channel
        );

        if (ret != ESP_OK) {
            return ret;
        }
    }

    return ESP_OK;
}

esp_err_t ArmJointController::make_servo_motion_options(
    uint8_t index,
    float target_deg,
    const JointMotionOptions& joint_options,
    ServoMotionOptions* servo_options
) const
{
    if (servo_options == nullptr ||
        index >= kArmJointCount ||
        !joints_[index].configured) {
        return ESP_ERR_INVALID_ARG;
    }

    ServoMotionOptions opt = {};

    opt.profile = joint_options.profile;
    opt.replan_from_current = joint_options.replan_from_current;
    opt.max_step_us = joint_options.max_step_us;
    opt.min_effective_step_us = joint_options.min_effective_step_us;

    switch (joint_options.timing_mode) {
        case JointTimingMode::Immediate:
            opt.profile = MotionProfile::Immediate;
            opt.duration_ms = 20;
            break;

        case JointTimingMode::Duration:
            opt.duration_ms = joint_options.duration_ms;
            break;

        case JointTimingMode::Speed: {
            if (joint_options.speed_deg_per_s <= 0.0f) {
                return ESP_ERR_INVALID_ARG;
            }

            const ArmJoint joint =
                joints_[index].calibration.joint;

            const float current_deg =
                read_command_deg(joint);

            const float delta_deg =
                std::fabs(target_deg - current_deg);

            const float duration_ms_f =
                delta_deg / joint_options.speed_deg_per_s * 1000.0f;

            uint32_t duration_ms =
                static_cast<uint32_t>(std::lround(duration_ms_f));

            if (duration_ms < 20) {
                duration_ms = 20;
            }

            opt.duration_ms = duration_ms;
            break;
        }

        default:
            return ESP_ERR_INVALID_ARG;
    }

    *servo_options = opt;

    return ESP_OK;
}

esp_err_t ArmJointController::move_deg(
    ArmJoint joint,
    float target_deg,
    uint32_t duration_ms
)
{
    JointMotionOptions options;
    options.timing_mode = JointTimingMode::Duration;
    options.duration_ms = duration_ms;
    options.profile = MotionProfile::SmootherStep;
    options.replan_from_current = true;

    return move_deg(
        joint,
        target_deg,
        options
    );
}

esp_err_t ArmJointController::move_deg(
    ArmJoint joint,
    float target_deg,
    const JointMotionOptions& options
)
{
    const int index = joint_to_index(joint);
    if (!initialized_ || !is_valid_joint_index(index)) {
        return ESP_ERR_INVALID_ARG;
    }

    const JointCalibration& cal = joints_[index].calibration;

    target_deg = clamp_deg_by_index(
        static_cast<uint8_t>(index),
        target_deg
    );

    const uint16_t target_us =
        deg_to_us_by_index(
            static_cast<uint8_t>(index),
            target_deg
        );

    ServoMotionOptions servo_options = {};

    esp_err_t ret = make_servo_motion_options(
        static_cast<uint8_t>(index),
        target_deg,
        options,
        &servo_options
    );

    if (ret != ESP_OK) {
        return ret;
    }

    if (servo_options.profile == MotionProfile::Immediate) {
        return servo_->write_us_now(
            cal.channel,
            target_us
        );
    }

    return servo_->move_us(
        cal.channel,
        target_us,
        servo_options
    );
}

esp_err_t ArmJointController::move_claw_gap_cm(
    float target_gap_cm,
    uint32_t duration_ms
)
{
    JointMotionOptions options;
    options.timing_mode = JointTimingMode::Duration;
    options.duration_ms = duration_ms;
    options.profile = MotionProfile::SmootherStep;
    options.replan_from_current = true;

    return move_claw_gap_cm(
        target_gap_cm,
        options
    );
}

esp_err_t ArmJointController::move_claw_gap_cm(
    float target_gap_cm,
    const JointMotionOptions& options
)
{
    const int index = joint_to_index(ArmJoint::Claw);
    if (!initialized_ || !is_valid_joint_index(index)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (servo_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    if (options.timing_mode == JointTimingMode::Speed) {
        ESP_LOGE(
            TAG,
            "move_claw_gap_cm rejected: JointTimingMode::Speed is not defined for gap_cm yet"
        );
        return ESP_ERR_INVALID_ARG;
    }

    const JointCalibration& cal = joints_[index].calibration;

    const uint16_t target_us =
        claw_gap_cm_to_us(target_gap_cm);

    ServoMotionOptions servo_options = {};
    servo_options.profile = options.profile;
    servo_options.replan_from_current = options.replan_from_current;
    servo_options.max_step_us = options.max_step_us;
    servo_options.min_effective_step_us = options.min_effective_step_us;

    switch (options.timing_mode) {
        case JointTimingMode::Immediate:
            servo_options.profile = MotionProfile::Immediate;
            servo_options.duration_ms = 20;
            break;

        case JointTimingMode::Duration:
            servo_options.duration_ms = options.duration_ms;
            break;

        default:
            return ESP_ERR_INVALID_ARG;
    }

    if (servo_options.profile == MotionProfile::Immediate) {
        return servo_->write_us_now(
            cal.channel,
            target_us
        );
    }

    return servo_->move_us(
        cal.channel,
        target_us,
        servo_options
    );
}

esp_err_t ArmJointController::move_speed(
    ArmJoint joint,
    float target_deg,
    float speed_deg_per_s,
    MotionProfile profile
)
{
    JointMotionOptions options;

    options.timing_mode = JointTimingMode::Speed;
    options.speed_deg_per_s = speed_deg_per_s;
    options.profile = profile;
    options.replan_from_current = true;

    return move_deg(
        joint,
        target_deg,
        options
    );
}

esp_err_t ArmJointController::write_deg_now(
    ArmJoint joint,
    float target_deg
)
{
    JointMotionOptions options;

    options.timing_mode = JointTimingMode::Immediate;
    options.profile = MotionProfile::Immediate;

    return move_deg(
        joint,
        target_deg,
        options
    );
}

esp_err_t ArmJointController::reset_joint(
    ArmJoint joint,
    uint32_t duration_ms
)
{
    const int index = joint_to_index(joint);
    if (!initialized_ || !is_valid_joint_index(index)) {
        return ESP_ERR_INVALID_ARG;
    }

    const JointCalibration& cal = joints_[index].calibration;

    return move_deg(
        joint,
        cal.zero_deg,
        duration_ms
    );
}

esp_err_t ArmJointController::reset_all(uint32_t duration_ms)
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    for (uint8_t i = 0; i < kArmJointCount; ++i) {
        if (!joints_[i].configured) {
            continue;
        }

        esp_err_t ret = reset_joint(
            joints_[i].calibration.joint,
            duration_ms
        );

        if (ret != ESP_OK) {
            return ret;
        }
    }

    return ESP_OK;
}

esp_err_t ArmJointController::stop(ArmJoint joint)
{
    const int index = joint_to_index(joint);
    if (!initialized_ || !is_valid_joint_index(index)) {
        return ESP_ERR_INVALID_ARG;
    }

    return servo_->stop(
        joints_[index].calibration.channel
    );
}

esp_err_t ArmJointController::stop_all()
{
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    for (uint8_t i = 0; i < kArmJointCount; ++i) {
        if (!joints_[i].configured) {
            continue;
        }

        esp_err_t ret = stop(joints_[i].calibration.joint);

        if (ret != ESP_OK) {
            return ret;
        }
    }

    return ESP_OK;
}

float ArmJointController::read_command_deg(ArmJoint joint) const
{
    const int index = joint_to_index(joint);
    if (!initialized_ || !is_valid_joint_index(index)) {
        return 0.0f;
    }

    const JointCalibration& cal = joints_[index].calibration;

    const uint16_t pulse_us =
        servo_->read_command_us(cal.channel);

    return us_to_deg_by_index(
        static_cast<uint8_t>(index),
        pulse_us
    );
}

float ArmJointController::read_target_deg(ArmJoint joint) const
{
    const int index = joint_to_index(joint);
    if (!initialized_ || !is_valid_joint_index(index)) {
        return 0.0f;
    }

    const JointCalibration& cal = joints_[index].calibration;

    const uint16_t pulse_us =
        servo_->read_target_us(cal.channel);

    return us_to_deg_by_index(
        static_cast<uint8_t>(index),
        pulse_us
    );
}

uint16_t ArmJointController::read_command_us(ArmJoint joint) const
{
    const int index = joint_to_index(joint);
    if (!initialized_ || !is_valid_joint_index(index)) {
        return 0;
    }

    return servo_->read_command_us(
        joints_[index].calibration.channel
    );
}

uint16_t ArmJointController::read_target_us(ArmJoint joint) const
{
    const int index = joint_to_index(joint);
    if (!initialized_ || !is_valid_joint_index(index)) {
        return 0;
    }

    return servo_->read_target_us(
        joints_[index].calibration.channel
    );
}

uint16_t ArmJointController::read_output_us(ArmJoint joint) const
{
    const int index = joint_to_index(joint);
    if (!initialized_ || !is_valid_joint_index(index)) {
        return 0;
    }

    return servo_->read_output_us(
        joints_[index].calibration.channel
    );
}

JointCalibration ArmJointController::get_calibration(ArmJoint joint) const
{
    const int index = joint_to_index(joint);
    if (!is_valid_joint_index(index)) {
        return {};
    }

    return joints_[index].calibration;
}

esp_err_t ArmJointController::set_calibration(
    ArmJoint joint,
    const JointCalibration& calibration
)
{
    const int index = joint_to_index(joint);
    if (index < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    JointCalibration cal = calibration;
    cal.joint = joint;

    esp_err_t ret = normalize_calibration(&cal);
    if (ret != ESP_OK) {
        return ret;
    }

    joints_[index].configured = true;
    joints_[index].calibration = cal;

    ESP_LOGW(
        TAG,
        "calibration updated joint=%u -> S%u zero_deg=%.2f zero_us=%u us_per_deg=%.4f dir=%d range=[%.2f,%.2f]",
        static_cast<unsigned>(cal.joint),
        static_cast<unsigned>(cal.channel),
        static_cast<double>(cal.zero_deg),
        cal.zero_us,
        static_cast<double>(cal.us_per_degree),
        static_cast<int>(cal.direction),
        static_cast<double>(cal.min_deg),
        static_cast<double>(cal.max_deg)
    );

    return ESP_OK;
}

esp_err_t ArmJointController::set_default_motion_options(
    ArmJoint joint,
    const JointMotionOptions& options
)
{
    const int index = joint_to_index(joint);
    if (!is_valid_joint_index(index)) {
        return ESP_ERR_INVALID_ARG;
    }

    joints_[index].default_options = options;

    return ESP_OK;
}

JointMotionOptions ArmJointController::get_default_motion_options(
    ArmJoint joint
) const
{
    const int index = joint_to_index(joint);
    if (!is_valid_joint_index(index)) {
        return {};
    }

    return joints_[index].default_options;
}

JointRuntimeState ArmJointController::get_state(ArmJoint joint) const
{
    const int index = joint_to_index(joint);
    if (!initialized_ || !is_valid_joint_index(index)) {
        return {};
    }

    const JointCalibration& cal = joints_[index].calibration;

    const ServoRuntimeState servo_state =
        servo_->get_state(cal.channel);

    JointRuntimeState state = {};

    state.configured = joints_[index].configured &&
                       servo_state.configured;

    state.enabled = servo_state.enabled;
    state.running = servo_state.running;
    state.ready = servo_state.ready;

    state.joint = joint;
    state.channel = cal.channel;

    state.current_us = servo_state.current_us;
    state.target_us = servo_state.target_us;
    state.output_us = servo_state.output_us;

    state.current_deg = us_to_deg_by_index(
        static_cast<uint8_t>(index),
        servo_state.current_us
    );

    state.target_deg = us_to_deg_by_index(
        static_cast<uint8_t>(index),
        servo_state.target_us
    );

    state.zero_deg = cal.zero_deg;
    state.zero_us = cal.zero_us;
    state.us_per_degree = cal.us_per_degree;
    state.direction = cal.direction;

    state.min_deg = cal.min_deg;
    state.max_deg = cal.max_deg;

    state.duration_ms = servo_state.duration_ms;
    state.total_steps = servo_state.total_steps;
    state.elapsed_steps = servo_state.elapsed_steps;

    state.profile = servo_state.profile;

    return state;
}

bool ArmJointController::is_ready(ArmJoint joint) const
{
    const int index = joint_to_index(joint);
    if (!initialized_ || !is_valid_joint_index(index)) {
        return false;
    }

    return servo_->is_ready(
        joints_[index].calibration.channel
    );
}

bool ArmJointController::is_all_ready() const
{
    if (!initialized_) {
        return false;
    }

    for (uint8_t i = 0; i < kArmJointCount; ++i) {
        if (!joints_[i].configured) {
            continue;
        }

        if (!servo_->is_ready(joints_[i].calibration.channel)) {
            return false;
        }
    }

    return true;
}

}  // namespace learm