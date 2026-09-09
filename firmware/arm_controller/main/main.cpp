#include <algorithm>
#include <cmath>
#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "arm_joint.hpp"
#include "arm_kinematics.hpp"
#include "arm_motion.hpp"
#include "servo_pwm.hpp"

using learm::ArmCartesianPose;
using learm::ArmJoint;
using learm::ArmJointController;
using learm::ArmKinematics;
using learm::ArmKinematicsJointMask;
using learm::ArmKinematicsJointState;
using learm::ArmKinematicsJointVelocity;
using learm::ArmMotion;
using learm::ArmMotionCommand;
using learm::ArmMotionJointVelocity;
using learm::ArmMotionMode;
using learm::ArmMotionPositionCommand;
using learm::ArmMotionState;
using learm::ArmOutputState;
using learm::ArmToolVelocity;
using learm::JointCalibration;
using learm::ServoPwm;

static const char* TAG = "jerr_pos_vel_experiment";

static ServoPwm g_servo;
static ArmJointController g_joints;
static ArmMotion g_motion;
static ArmKinematics g_kinematics;

// Application-level command ownership. Track is the default source. A valid
// pos command temporarily owns ArmMotion until an explicit stop returns
// control to Track. ArmMotionMode still describes command execution itself.
enum class AppControlMode : uint8_t {
    Track = 0,
    Pos,
};

static AppControlMode g_app_control_mode = AppControlMode::Track;
static bool g_jerr_ignore_reported = false;

// One fixed visual-error-to-tool-velocity parameter group.
// jerr inputs are dimensionless normalized errors in [-1, 1].
struct VisualVelocityParameters {
    float forward_gain_mm_per_s = 30.0f;
    float left_gain_mm_per_s = 220.0f;
    float up_gain_mm_per_s = 300.0f;
    float pitch_gain_deg_per_s = 20.0f;

    float max_forward_mm_per_s = 30.0f;
    float max_left_mm_per_s = 150.0f;
    float max_up_mm_per_s = 200.0f;
    float max_pitch_deg_per_s = 20.0f;
};

static constexpr VisualVelocityParameters kVisualVelocity = {};

// The physical arm must be placed near this pose before power-on. This first
// command becomes the motion layer's initial software reference.
static constexpr ArmMotionPositionCommand kInitialPose = {
    {0.0f, -60.0f, 70.0f, 70.0f, 0.0f},
    0.0f,
};

static void delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static void trim_line(char* line)
{
    if (line == nullptr) {
        return;
    }

    size_t len = strlen(line);
    while (len > 0 && isspace(static_cast<unsigned char>(line[len - 1]))) {
        line[--len] = '\0';
    }

    char* first = line;
    while (*first != '\0' && isspace(static_cast<unsigned char>(*first))) {
        ++first;
    }
    if (first != line) {
        memmove(line, first, strlen(first) + 1);
    }
}

static bool has_command_prefix(const char* line, const char* command)
{
    if (line == nullptr || command == nullptr) {
        return false;
    }

    const size_t length = strlen(command);
    return strncmp(line, command, length) == 0 &&
           (line[length] == '\0' ||
            isspace(static_cast<unsigned char>(line[length])));
}

static const char* app_control_mode_name(AppControlMode mode)
{
    switch (mode) {
        case AppControlMode::Track: return "track";
        case AppControlMode::Pos:   return "pos";
        default:                    return "unknown";
    }
}

static void enter_track_mode()
{
    g_app_control_mode = AppControlMode::Track;
    g_jerr_ignore_reported = false;
}

static void enter_pos_mode()
{
    g_app_control_mode = AppControlMode::Pos;
    g_jerr_ignore_reported = false;
}

static bool parse_position_command(
    const char* line,
    ArmMotionPositionCommand* command
)
{
    if (line == nullptr || command == nullptr) {
        return false;
    }

    int consumed = -1;
    const int matched = sscanf(
        line,
        "pos %f %f %f %f %f %f%n",
        &command->joint_position.base_deg,
        &command->joint_position.shoulder_deg,
        &command->joint_position.elbow_deg,
        &command->joint_position.wrist_pitch_deg,
        &command->joint_position.wrist_roll_deg,
        &command->claw_gap_cm,
        &consumed
    );

    return matched == 6 && consumed >= 0 && line[consumed] == '\0';
}

static bool parse_normalized_error_command(
    const char* line,
    float* forward_error,
    float* left_error,
    float* up_error,
    float* pitch_error
)
{
    if (line == nullptr || forward_error == nullptr || left_error == nullptr ||
        up_error == nullptr || pitch_error == nullptr) {
        return false;
    }

    int consumed = -1;
    const int matched = sscanf(
        line,
        "jerr %f %f %f %f%n",
        forward_error,
        left_error,
        up_error,
        pitch_error,
        &consumed
    );

    return matched == 4 && consumed >= 0 && line[consumed] == '\0' &&
           std::isfinite(*forward_error) &&
           std::isfinite(*left_error) &&
           std::isfinite(*up_error) &&
           std::isfinite(*pitch_error);
}

static const char* mode_name(ArmMotionMode mode)
{
    switch (mode) {
        case ArmMotionMode::Hold:     return "hold";
        case ArmMotionMode::Position: return "position";
        case ArmMotionMode::Velocity: return "velocity";
        case ArmMotionMode::Stopping: return "stopping";
        default:                      return "unknown";
    }
}

static const char* output_state_name(ArmOutputState state)
{
    switch (state) {
        case ArmOutputState::Disabled:    return "disabled";
        case ArmOutputState::Enabled:     return "enabled";
        case ArmOutputState::DriverError: return "driver_error";
        default:                          return "unknown";
    }
}

static const char* joint_name(ArmJoint joint)
{
    switch (joint) {
        case ArmJoint::Claw:       return "claw";
        case ArmJoint::WristRoll:  return "wrist_roll";
        case ArmJoint::WristPitch: return "wrist_pitch";
        case ArmJoint::Elbow:      return "elbow";
        case ArmJoint::Shoulder:   return "shoulder";
        case ArmJoint::Base:       return "base";
        default:                   return "unknown";
    }
}

static ArmKinematicsJointState kinematics_state_from_motion(const ArmMotionState& state)
{
    ArmKinematicsJointState result;
    result.base_deg = state.base.reference_deg;
    result.shoulder_deg = state.shoulder.reference_deg;
    result.elbow_deg = state.elbow.reference_deg;
    result.wrist_pitch_deg = state.wrist_pitch.reference_deg;
    result.wrist_roll_deg = state.wrist_roll.reference_deg;
    return result;
}

static ArmMotionJointVelocity motion_velocity_from_kinematics(
    const ArmKinematicsJointVelocity& velocity
)
{
    ArmMotionJointVelocity result;
    result.base_deg_per_s = velocity.base_deg_per_s;
    result.shoulder_deg_per_s = velocity.shoulder_deg_per_s;
    result.elbow_deg_per_s = velocity.elbow_deg_per_s;
    result.wrist_pitch_deg_per_s = velocity.wrist_pitch_deg_per_s;
    result.wrist_roll_deg_per_s = velocity.wrist_roll_deg_per_s;
    return result;
}

static float normalized_error(float value)
{
    if (!std::isfinite(value)) {
        return 0.0f;
    }
    return std::clamp(value, -1.0f, 1.0f);
}

static ArmToolVelocity tool_velocity_from_error(
    float forward_error,
    float left_error,
    float up_error,
    float pitch_error
)
{
    const float ef = normalized_error(forward_error);
    const float el = normalized_error(left_error);
    const float eu = normalized_error(up_error);
    const float ep = normalized_error(pitch_error);

    ArmToolVelocity velocity;
    velocity.forward_mm_per_s = std::clamp(
        kVisualVelocity.forward_gain_mm_per_s * ef,
        -kVisualVelocity.max_forward_mm_per_s,
         kVisualVelocity.max_forward_mm_per_s
    );
    velocity.left_mm_per_s = std::clamp(
        kVisualVelocity.left_gain_mm_per_s * el,
        -kVisualVelocity.max_left_mm_per_s,
         kVisualVelocity.max_left_mm_per_s
    );
    velocity.up_mm_per_s = std::clamp(
        kVisualVelocity.up_gain_mm_per_s * eu,
        -kVisualVelocity.max_up_mm_per_s,
         kVisualVelocity.max_up_mm_per_s
    );
    velocity.pitch_deg_per_s = std::clamp(
        kVisualVelocity.pitch_gain_deg_per_s * ep,
        -kVisualVelocity.max_pitch_deg_per_s,
         kVisualVelocity.max_pitch_deg_per_s
    );
    velocity.roll_deg_per_s = 0.0f;
    return velocity;
}

static constexpr float kActiveSetLimitEpsilonDeg = 0.001f;
static constexpr float kActiveSetVelocityEpsilonDegPerS = 0.001f;

static bool disable_if_moving_outward_at_limit(
    float reference_deg,
    float velocity_deg_per_s,
    learm::ArmJoint joint,
    bool* enabled
)
{
    if (enabled == nullptr || !*enabled) {
        return false;
    }

    const JointCalibration calibration = g_joints.get_calibration(joint);
    const bool outward_at_min =
        reference_deg <= calibration.min_deg + kActiveSetLimitEpsilonDeg &&
        velocity_deg_per_s < -kActiveSetVelocityEpsilonDegPerS;
    const bool outward_at_max =
        reference_deg >= calibration.max_deg - kActiveSetLimitEpsilonDeg &&
        velocity_deg_per_s > kActiveSetVelocityEpsilonDegPerS;

    if (!outward_at_min && !outward_at_max) {
        return false;
    }

    *enabled = false;
    return true;
}

static bool remove_outward_limited_joints(
    const ArmMotionState& state,
    const ArmKinematicsJointVelocity& velocity,
    ArmKinematicsJointMask* enabled_joints
)
{
    if (enabled_joints == nullptr) {
        return false;
    }

    bool changed = false;
    changed |= disable_if_moving_outward_at_limit(
        state.base.reference_deg,
        velocity.base_deg_per_s,
        learm::ArmJoint::Base,
        &enabled_joints->base
    );
    changed |= disable_if_moving_outward_at_limit(
        state.shoulder.reference_deg,
        velocity.shoulder_deg_per_s,
        learm::ArmJoint::Shoulder,
        &enabled_joints->shoulder
    );
    changed |= disable_if_moving_outward_at_limit(
        state.elbow.reference_deg,
        velocity.elbow_deg_per_s,
        learm::ArmJoint::Elbow,
        &enabled_joints->elbow
    );
    changed |= disable_if_moving_outward_at_limit(
        state.wrist_pitch.reference_deg,
        velocity.wrist_pitch_deg_per_s,
        learm::ArmJoint::WristPitch,
        &enabled_joints->wrist_pitch
    );
    changed |= disable_if_moving_outward_at_limit(
        state.wrist_roll.reference_deg,
        velocity.wrist_roll_deg_per_s,
        learm::ArmJoint::WristRoll,
        &enabled_joints->wrist_roll
    );
    return changed;
}

static esp_err_t apply_normalized_error(
    float forward_error,
    float left_error,
    float up_error,
    float pitch_error
)
{
    const ArmMotionState motion_state = g_motion.get_state();
    if (!motion_state.initialized ||
        motion_state.output_state != ArmOutputState::Enabled) {
        return ESP_ERR_INVALID_STATE;
    }

    const ArmKinematicsJointState joints =
        kinematics_state_from_motion(motion_state);
    const ArmToolVelocity tool_velocity = tool_velocity_from_error(
        forward_error,
        left_error,
        up_error,
        pitch_error
    );

    ArmKinematicsJointMask enabled_joints;
    ArmKinematicsJointVelocity solved_velocity;
    bool solved = false;

    // Small active-set loop: if the current solution asks a joint already at
    // its limit to move farther outward, constrain that joint to zero and run
    // the same Jacobian solver again with the remaining joints. Multiple
    // joints may be removed in one pass. Six solves are sufficient for five
    // rotary joints, including the final all-constrained solution.
    for (int pass = 0; pass < 6; ++pass) {
        const esp_err_t solve_ret = g_kinematics.solve_tool_velocity(
            joints,
            tool_velocity,
            enabled_joints,
            &solved_velocity
        );
        if (solve_ret != ESP_OK) {
            g_motion.stop();
            return solve_ret;
        }

        if (!remove_outward_limited_joints(
                motion_state,
                solved_velocity,
                &enabled_joints)) {
            solved = true;
            break;
        }
    }

    if (!solved) {
        g_motion.stop();
        return ESP_ERR_INVALID_STATE;
    }

    ArmMotionCommand motion_command;
    motion_command.joint_velocity =
        motion_velocity_from_kinematics(solved_velocity);
    motion_command.claw_gap_cm = motion_state.claw_target_gap_cm;
    return g_motion.command(motion_command);
}

static void print_help()
{
    printf("\n=== normalized-error position + velocity experiment ===\n");
    printf("Default tracking protocol:\n");
    printf("  jerr F L U P\n");
    printf("    F/L/U/P are dimensionless normalized errors in [-1, 1].\n");
    printf("    The hand tracker sends only L/U: jerr 0 L U 0.\n");
    printf("    Accepted in Track mode; refreshes the velocity watchdog.\n\n");
    printf("Position override:\n");
    printf("  pos B S E WP WR GAP\n");
    printf("    Move to absolute rotary joint angles in degrees and claw gap in cm.\n");
    printf("    A successful pos enters Pos mode; jerr is ignored until stop.\n");
    printf("  stop       stop current motion and return to default Track mode\n");
    printf("  state      print PWM state, references, targets and watchdog state\n");
    printf("  fk         print forward pose from reference positions\n");
    printf("  enable 0   when disabled, enable at calibration zero with claw closed\n");
    printf("  disable    immediately disable PWM outputs\n");
    printf("  help       print this help\n\n");
    printf("Startup application mode: Track. No mode-selection command is needed.\n\n");
    printf("Example position override:\n");
    printf("  pos 0 -25 45 15 0 0\n\n");
    printf("Fixed task-speed limits: F=%.1f L=%.1f U=%.1f mm/s, P=%.1f deg/s.\n",
           static_cast<double>(kVisualVelocity.max_forward_mm_per_s),
           static_cast<double>(kVisualVelocity.max_left_mm_per_s),
           static_cast<double>(kVisualVelocity.max_up_mm_per_s),
           static_cast<double>(kVisualVelocity.max_pitch_deg_per_s));
}

static void print_state()
{
    const ArmMotionState s = g_motion.get_state();
    const char* error_joint = s.driver_error_joint_valid
        ? joint_name(s.driver_error_joint)
        : "none";
    printf("initialized=%d app=%s pwm=%s mode=%s reference_moving=%d fresh=%d timeout=%d age=%lums claw_ref=%.2f claw_target=%.2f driver_error=0x%x error_joint=%s\n",
           s.initialized,
           app_control_mode_name(g_app_control_mode),
           output_state_name(s.output_state),
           mode_name(s.mode),
           s.reference_moving,
           s.command_fresh,
           s.command_timed_out,
           static_cast<unsigned long>(s.command_age_ms),
           static_cast<double>(s.claw_reference_gap_cm),
           static_cast<double>(s.claw_target_gap_cm),
           static_cast<unsigned>(s.last_driver_error),
           error_joint);

    auto print_joint = [](const char* name, const learm::ArmMotionReferenceState& joint) {
        printf("  %-12s q_ref=%8.3f  q_target=%8.3f  v_target=%8.3f  v_applied=%8.3f ref_reached=%d limit=%d\n",
               name,
               static_cast<double>(joint.reference_deg),
               static_cast<double>(joint.target_position_deg),
               static_cast<double>(joint.target_velocity_deg_per_s),
               static_cast<double>(joint.applied_velocity_deg_per_s),
               joint.reference_reached,
               joint.limit_blocked);
    };

    print_joint("base", s.base);
    print_joint("shoulder", s.shoulder);
    print_joint("elbow", s.elbow);
    print_joint("wrist_pitch", s.wrist_pitch);
    print_joint("wrist_roll", s.wrist_roll);
}

static void print_fk()
{
    const ArmMotionState motion_state = g_motion.get_state();
    const ArmKinematicsJointState joints =
        kinematics_state_from_motion(motion_state);

    ArmCartesianPose pose;
    const esp_err_t ret = g_kinematics.forward_pose(joints, &pose);
    if (ret != ESP_OK) {
        printf("forward_pose ret=0x%x\n", static_cast<unsigned>(ret));
        return;
    }

    printf("pose x=%.2f y=%.2f z=%.2f pitch=%.2f roll=%.2f\n",
           static_cast<double>(pose.x_mm),
           static_cast<double>(pose.y_mm),
           static_cast<double>(pose.z_mm),
           static_cast<double>(pose.tool_pitch_deg),
           static_cast<double>(pose.tool_roll_deg));
}

static void process_command(char* line)
{
    trim_line(line);
    if (line[0] == '\0') {
        return;
    }

    if (strcmp(line, "help") == 0) {
        print_help();
        return;
    }
    if (strcmp(line, "state") == 0) {
        print_state();
        return;
    }
    if (strcmp(line, "fk") == 0) {
        print_fk();
        return;
    }
    if (strcmp(line, "stop") == 0) {
        const esp_err_t ret = g_motion.stop();
        enter_track_mode();
        printf("stop ret=0x%x app=%s\n",
               static_cast<unsigned>(ret),
               app_control_mode_name(g_app_control_mode));
        return;
    }
    if (strcmp(line, "enable 0") == 0) {
        const esp_err_t ret = g_motion.enable_all_at_zero_closed();
        if (ret == ESP_OK) {
            enter_track_mode();
        }
        printf(
            "enable 0 ret=0x%x app=%s\n",
            static_cast<unsigned>(ret),
            app_control_mode_name(g_app_control_mode)
        );
        return;
    }
    if (strcmp(line, "disable") == 0) {
        const esp_err_t ret = g_motion.disable_all();
        enter_track_mode();
        printf("disable ret=0x%x app=%s\n",
               static_cast<unsigned>(ret),
               app_control_mode_name(g_app_control_mode));
        return;
    }

    ArmMotionPositionCommand position_command;
    if (parse_position_command(line, &position_command)) {
        const esp_err_t ret = g_motion.move_to(position_command);
        if (ret == ESP_OK) {
            enter_pos_mode();
        }
        printf("pos ret=0x%x app=%s\n",
               static_cast<unsigned>(ret),
               app_control_mode_name(g_app_control_mode));
        return;
    }
    if (has_command_prefix(line, "pos")) {
        printf("invalid pos; expected: pos B S E WP WR GAP\n");
        return;
    }

    float forward_error = 0.0f;
    float left_error = 0.0f;
    float up_error = 0.0f;
    float pitch_error = 0.0f;
    if (parse_normalized_error_command(
            line,
            &forward_error,
            &left_error,
            &up_error,
            &pitch_error)) {
        if (g_app_control_mode != AppControlMode::Track) {
            if (!g_jerr_ignore_reported) {
                printf("jerr ignored: app=pos; send stop to resume tracking\n");
                g_jerr_ignore_reported = true;
            }
            return;
        }

        const esp_err_t ret = apply_normalized_error(
            forward_error,
            left_error,
            up_error,
            pitch_error
        );
        if (ret != ESP_OK) {
            printf("jerr failed ret=0x%x; app=%s\n",
                   static_cast<unsigned>(ret),
                   app_control_mode_name(g_app_control_mode));
        }
        return;
    }
    if (has_command_prefix(line, "jerr")) {
        printf("invalid jerr; expected: jerr F L U P\n");
        return;
    }

    printf("unknown command; type help\n");
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "starting normalized-error position + streaming-velocity experiment");

    ESP_ERROR_CHECK(g_servo.init(
        learm::arm_joint_defaults::kServoPwmConfigs,
        learm::arm_joint_defaults::kServoPwmConfigCount
    ));

    ESP_ERROR_CHECK(g_joints.init(
        &g_servo,
        learm::arm_joint_defaults::kJointCalibrations,
        learm::arm_joint_defaults::kJointCalibrationCount
    ));

    ESP_ERROR_CHECK(g_motion.init(&g_joints));
    ESP_ERROR_CHECK(g_kinematics.init());

    ESP_LOGW(TAG, "servo outputs remain disabled for 3 seconds");
    delay_ms(3000);
    ESP_ERROR_CHECK(g_motion.enable_all_at(kInitialPose));
    enter_track_mode();

    print_help();
    print_state();

    char line[160];
    while (true) {
        if (fgets(line, sizeof(line), stdin) == nullptr) {
            delay_ms(20);
            continue;
        }
        process_command(line);
    }
}
