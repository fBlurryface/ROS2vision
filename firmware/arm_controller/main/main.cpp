#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "servo_pwm.hpp"
#include "arm_joint.hpp"
#include "arm_motion.hpp"
#include "arm_kinematics.hpp"

using learm::ArmJoint;
using learm::ArmJointController;
using learm::ArmMotion;
using learm::ArmMotionDurationsMs;
using learm::ArmMotionState;
using learm::ArmMotionStrategies;
using learm::ArmMotionSpeedStrategies;
using learm::ArmToolTargetError;
using learm::ArmKinematics;
using learm::ArmKinematicsConfig;
using learm::ArmKinematicsDelta;
using learm::ArmKinematicsJointState;
using learm::JointCalibration;
using learm::JointMotionStyle;
using learm::JointRuntimeState;
using learm::ServoPwm;

static const char* TAG = "arm_motion_v5a";

static ServoPwm g_servo;
static ArmJointController g_joints;
static ArmMotion g_motion;
static ArmKinematics g_kinematics;

static void delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static ArmKinematicsConfig make_default_kinematics_config()
{
    ArmKinematicsConfig config;

    // Measured linkage geometry for incremental Jacobian testing.
    // LINKAGE_1: base -> shoulder plane vertical offset.
    // LINKAGE_2: shoulder -> elbow.
    // LINKAGE_3: elbow -> wrist_pitch.
    // LINKAGE_4: wrist_pitch -> TCP / claw reference point.
    config.base_to_shoulder_z_mm = 28.9f;
    config.upper_arm_mm = 104.3f;
    config.forearm_mm = 89.0f;
    config.wrist_to_tool_mm = 177.0f;

    // Semantic joint angle -> kinematic model angle mapping.
    // If jtool direction is inverted for a joint, tune the corresponding sign.
    config.base_sign = 1.0f;
    config.shoulder_sign = 1.0f;
    config.elbow_sign = 1.0f;
    config.wrist_pitch_sign = 1.0f;
    config.wrist_roll_sign = 1.0f;

    config.base_offset_deg = 0.0f;
    config.shoulder_offset_deg = 0.0f;
    config.elbow_offset_deg = 0.0f;
    config.wrist_pitch_offset_deg = 0.0f;
    config.wrist_roll_offset_deg = 0.0f;

    // Purpose-built tool target solver.
    config.lateral_gain = 0.45f;
    config.longitudinal_gain = 0.35f;
    config.up_gain = 0.30f;

    config.lateral_damping_mm = 35.0f;
    config.longitudinal_damping_mm = 35.0f;
    config.up_damping_mm = 35.0f;

    config.planar_hold_min_effect_mm2 = 1.0f;
    config.relaxed_orientation_weight_mm = 10.0f;

    config.max_error_mm = 20.0f;
    config.max_delta_deg_base = 1.0f;
    config.max_delta_deg_shoulder = 1.0f;
    config.max_delta_deg_elbow = 1.0f;
    config.max_delta_deg_wrist_pitch = 1.0f;
    config.max_delta_deg_wrist_roll = 1.0f;

    // Keep these synchronized with arm_joint_defaults::kJointCalibrations.
    config.min_deg_base = -85.0f;
    config.max_deg_base = 85.0f;
    config.min_deg_shoulder = -85.0f;
    config.max_deg_shoulder = 85.0f;
    config.min_deg_elbow = -85.0f;
    config.max_deg_elbow = 85.0f;
    config.min_deg_wrist_pitch = -85.0f;
    config.max_deg_wrist_pitch = 85.0f;

    return config;
}

static void trim_line(char* line)
{
    if (line == nullptr) {
        return;
    }

    size_t len = strlen(line);

    while (len > 0 &&
           (line[len - 1] == '\n' ||
            line[len - 1] == '\r' ||
            isspace(static_cast<unsigned char>(line[len - 1])))) {
        line[len - 1] = '\0';
        len--;
    }

    char* start = line;

    while (*start != '\0' &&
           isspace(static_cast<unsigned char>(*start))) {
        start++;
    }

    if (start != line) {
        memmove(line, start, strlen(start) + 1);
    }
}

static bool starts_with(const char* s, const char* prefix)
{
    if (s == nullptr || prefix == nullptr) {
        return false;
    }

    return strncmp(s, prefix, strlen(prefix)) == 0;
}

static bool equals_ignore_case(const char* a, const char* b)
{
    if (a == nullptr || b == nullptr) {
        return false;
    }

    while (*a != '\0' && *b != '\0') {
        const int ca = tolower(static_cast<unsigned char>(*a));
        const int cb = tolower(static_cast<unsigned char>(*b));

        if (ca != cb) {
            return false;
        }

        a++;
        b++;
    }

    return *a == '\0' && *b == '\0';
}

static const char* style_to_name(JointMotionStyle style)
{
    switch (style) {
        case JointMotionStyle::Linear:
            return "linear";

        case JointMotionStyle::Smooth:
            return "smooth";

        case JointMotionStyle::Soft:
        default:
            return "soft";
    }
}

static bool parse_style_token(const char* token, JointMotionStyle* style)
{
    if (token == nullptr || style == nullptr) {
        return false;
    }

    if (equals_ignore_case(token, "linear") || strcmp(token, "0") == 0) {
        *style = JointMotionStyle::Linear;
        return true;
    }

    if (equals_ignore_case(token, "smooth") || strcmp(token, "1") == 0) {
        *style = JointMotionStyle::Smooth;
        return true;
    }

    if (equals_ignore_case(token, "soft") || strcmp(token, "2") == 0) {
        *style = JointMotionStyle::Soft;
        return true;
    }

    return false;
}

static const char* joint_to_name(ArmJoint joint)
{
    switch (joint) {
        case ArmJoint::Base:
            return "Base";

        case ArmJoint::Shoulder:
            return "Shoulder";

        case ArmJoint::Elbow:
            return "Elbow";

        case ArmJoint::WristPitch:
            return "WristPitch";

        case ArmJoint::WristRoll:
            return "WristRoll";

        case ArmJoint::Claw:
            return "Claw";

        default:
            return "Unknown";
    }
}

static ArmKinematicsJointState current_kinematics_joints()
{
    const ArmMotionState state = g_motion.get_state();

    ArmKinematicsJointState joints;
    joints.base_deg = state.base.current_deg;
    joints.shoulder_deg = state.shoulder.current_deg;
    joints.elbow_deg = state.elbow.current_deg;
    joints.wrist_pitch_deg = state.wrist_pitch.current_deg;
    joints.wrist_roll_deg = state.wrist_roll.current_deg;

    return joints;
}

static void print_help()
{
    printf("\n");
    printf("========== arm_motion V5A motion-owned test ==========\n");
    printf("\n");
    printf("This test uses:\n");
    printf("  g_motion.enable_all()\n");
    printf("  g_motion.set_durations_ms(...)\n");
    printf("  g_motion.set_strategies(...)\n");
    printf("  g_motion.move_to(...)\n");
    printf("  g_motion.move_to_speed(...)\n");
    printf("  g_motion.move_to_immediate(...)\n");
    printf("  g_motion.move_delta_immediate(...)\n");
    printf("\n");
    printf("Axis order:\n");
    printf("  base shoulder elbow wrist_pitch wrist_roll claw\n");
    printf("\n");
    printf("Commands:\n");
    printf("  help        print this help\n");
    printf("  enable      initialize/enable all joints at default calibration pose\n");
    printf("  stop        stop current motion and hold current commanded pose\n");
    printf("  disable     stop motion and turn PWM output off; torque release is not guaranteed\n");
    printf("  default     print default enable pulse / calibration\n");
    printf("  config      print current durations and strategies\n");
    printf("  state       print arm_motion state\n");
    printf("  ready       print ready / moving state\n");
    printf("\n");
    printf("  dur B S E WP WR C\n");
    printf("    Set durations in ms. Example:\n");
    printf("    dur 1000 1200 1200 800 600 500\n");
    printf("\n");
    printf("  style B S E WP WR C\n");
    printf("    Set duration-line styles. Use linear/smooth/soft or 0/1/2. Example:\n");
    printf("    style smooth soft soft smooth smooth smooth\n");
    printf("\n");
    printf("  speed B S E WP WR C\n");
    printf("    Set speed-line max speeds. First 5 are deg/s, last is claw cm/s. Example:\n");
    printf("    speed 45 35 35 50 70 2.5\n");
    printf("\n");
    printf("  move B S E WP WR GAP\n");
    printf("    Duration-line move. First 5 are degrees, last is claw gap cm. Example:\n");
    printf("    move 0 -30 45 60 0 5.8\n");
    printf("\n");
    printf("  movespeed B S E WP WR GAP\n");
    printf("    Speed-line move using configured max speeds, direct linear speed. Example:\n");
    printf("    movespeed 0 -30 45 60 0 5.8\n");
    printf("\n");
    printf("  movenow B S E WP WR GAP\n");
    printf("    Absolute immediate move: write the target pose now, no timing/speed planning. Example:\n");
    printf("    movenow 0 -30 45 60 0 5.8\n");
    printf("\n");
    printf("  deltanow dB dS dE dWP dWR dGAP\n");
    printf("    Incremental immediate move from current commanded pose. First 5 are deg deltas, last is claw gap cm delta. Example:\n");
    printf("    deltanow 1 0 0 0 0 0\n");
    printf("\n");
    printf("  jtool F L U [D]\n");
    printf("    Purpose-built f-link target solver, mm. F/L/U are target point coordinates\n");
    printf("    in the current f-link frame; optional D is desired forward distance.\n");
    printf("    left -> base yaw; forward/up -> C/D/E planar translation\n");
    printf("    with f-pitch hold first, relaxed only when ineffective. Example:\n");
    printf("    jtool 10 0 0 0\n");
    printf("\n");
    printf("  zero\n");
    printf("    Move to 0 0 0 0 0 5.8 using current config.\n");
    printf("\n");
    printf("  demo\n");
    printf("    Run a short group-motion sequence.\n");
    printf("\n");
    printf("==================================================\n");
    printf("\n");
}

static void print_joint_state(const char* label, const JointRuntimeState& s)
{
    printf(
        "  %-10s enabled=%d running=%d ready=%d current=%7.2fdeg target=%7.2fdeg us=%4u->%4u out=%4u\n",
        label,
        static_cast<int>(s.enabled),
        static_cast<int>(s.running),
        static_cast<int>(s.ready),
        static_cast<double>(s.current_deg),
        static_cast<double>(s.target_deg),
        static_cast<unsigned>(s.current_us),
        static_cast<unsigned>(s.target_us),
        static_cast<unsigned>(s.output_us)
    );
}

static void print_config()
{
    const ArmMotionDurationsMs d = g_motion.get_durations_ms();
    const ArmMotionStrategies s = g_motion.get_strategies();
    const ArmMotionSpeedStrategies v = g_motion.get_speed_strategies();

    printf("\n");
    printf("Current arm_motion config:\n");
    printf("  order: base shoulder elbow wrist_pitch wrist_roll claw\n");
    printf(
        "  durations_ms: %u %u %u %u %u %u\n",
        static_cast<unsigned>(d.base_ms),
        static_cast<unsigned>(d.shoulder_ms),
        static_cast<unsigned>(d.elbow_ms),
        static_cast<unsigned>(d.wrist_pitch_ms),
        static_cast<unsigned>(d.wrist_roll_ms),
        static_cast<unsigned>(d.claw_ms)
    );
    printf(
        "  duration_style: %s %s %s %s %s %s\n",
        style_to_name(s.base),
        style_to_name(s.shoulder),
        style_to_name(s.elbow),
        style_to_name(s.wrist_pitch),
        style_to_name(s.wrist_roll),
        style_to_name(s.claw)
    );
    printf(
        "  speed_max:      %.2f %.2f %.2f %.2f %.2f %.2f  (deg/s x5, claw cm/s)\n",
        static_cast<double>(v.base.max_speed_deg_per_s),
        static_cast<double>(v.shoulder.max_speed_deg_per_s),
        static_cast<double>(v.elbow.max_speed_deg_per_s),
        static_cast<double>(v.wrist_pitch.max_speed_deg_per_s),
        static_cast<double>(v.wrist_roll.max_speed_deg_per_s),
        static_cast<double>(v.claw.max_speed_cm_per_s)
    );
    printf("  speed_mode:     direct linear speed, no ease-in/ease-out curve\n");
    printf("\n");
}

static void print_default_enable_pulses()
{
    printf("\n");
    printf("Default enable positions from arm_joint calibration:\n");

    const ArmJoint order[] = {
        ArmJoint::Base,
        ArmJoint::Shoulder,
        ArmJoint::Elbow,
        ArmJoint::WristPitch,
        ArmJoint::WristRoll,
        ArmJoint::Claw,
    };

    for (ArmJoint joint : order) {
        const JointCalibration cal = g_joints.get_calibration(joint);

        printf(
            "  %-10s zero_deg=%7.2f zero_us=%4u range=[%7.2f,%7.2f]\n",
            joint_to_name(joint),
            static_cast<double>(cal.zero_deg),
            static_cast<unsigned>(cal.zero_us),
            static_cast<double>(cal.min_deg),
            static_cast<double>(cal.max_deg)
        );
    }

    printf("  Claw zero_us=500us is max open, about 5.8cm gap.\n");
    printf("\n");
}

static void print_state()
{
    const ArmMotionState s = g_motion.get_state();

    printf("\n");
    printf("Arm motion state:\n");
    printf("  initialized:        %d\n", static_cast<int>(s.initialized));
    printf("  ready:              %d\n", static_cast<int>(s.ready));
    printf("  moving:             %d\n", static_cast<int>(s.moving));
    printf("  claw_gap_cm:        %.2f\n", static_cast<double>(s.claw_gap_cm));
    printf("  claw_target_gap_cm: %.2f\n", static_cast<double>(s.claw_target_gap_cm));
    printf("\n");

    print_joint_state("Base", s.base);
    print_joint_state("Shoulder", s.shoulder);
    print_joint_state("Elbow", s.elbow);
    print_joint_state("WristPitch", s.wrist_pitch);
    print_joint_state("WristRoll", s.wrist_roll);
    print_joint_state("Claw", s.claw);
    printf("\n");
}

static bool parse_six_u32(char* args, uint32_t out[6])
{
    if (args == nullptr || out == nullptr) {
        return false;
    }

    for (int i = 0; i < 6; ++i) {
        char* token = strtok(i == 0 ? args : nullptr, " \t");
        if (token == nullptr) {
            return false;
        }

        char* end = nullptr;
        const unsigned long value = strtoul(token, &end, 10);

        if (end == token || *end != '\0' || value == 0 || value > UINT32_MAX) {
            return false;
        }

        out[i] = static_cast<uint32_t>(value);
    }

    return strtok(nullptr, " \t") == nullptr;
}

static bool parse_six_float(char* args, float out[6])
{
    if (args == nullptr || out == nullptr) {
        return false;
    }

    for (int i = 0; i < 6; ++i) {
        char* token = strtok(i == 0 ? args : nullptr, " \t");
        if (token == nullptr) {
            return false;
        }

        char* end = nullptr;
        const float value = strtof(token, &end);

        if (end == token || *end != '\0') {
            return false;
        }

        out[i] = value;
    }

    return strtok(nullptr, " \t") == nullptr;
}

static bool parse_three_float(char* args, float out[3])
{
    if (args == nullptr || out == nullptr) {
        return false;
    }

    for (int i = 0; i < 3; ++i) {
        char* token = strtok(i == 0 ? args : nullptr, " \t");
        if (token == nullptr) {
            return false;
        }

        char* end = nullptr;
        const float value = strtof(token, &end);

        if (end == token || *end != '\0') {
            return false;
        }

        out[i] = value;
    }

    return strtok(nullptr, " \t") == nullptr;
}

static bool parse_six_styles(char* args, JointMotionStyle out[6])
{
    if (args == nullptr || out == nullptr) {
        return false;
    }

    for (int i = 0; i < 6; ++i) {
        char* token = strtok(i == 0 ? args : nullptr, " \t");
        if (token == nullptr) {
            return false;
        }

        if (!parse_style_token(token, &out[i])) {
            return false;
        }
    }

    return strtok(nullptr, " \t") == nullptr;
}

static void run_demo_sequence()
{
    printf("\n");
    printf("Running arm_motion demo sequence...\n");
    printf("\n");

    ESP_ERROR_CHECK(g_motion.set_durations_ms(
        900,
        1100,
        1100,
        800,
        600,
        500
    ));

    ESP_ERROR_CHECK(g_motion.set_strategies(
        JointMotionStyle::Smooth,
        JointMotionStyle::Soft,
        JointMotionStyle::Soft,
        JointMotionStyle::Smooth,
        JointMotionStyle::Smooth,
        JointMotionStyle::Smooth
    ));

    printf("Demo step 1: ready-ish pose, claw open.\n");
    ESP_ERROR_CHECK(g_motion.move_to(
        0.0f,
        -25.0f,
        35.0f,
        45.0f,
        0.0f,
        5.8f
    ));

    delay_ms(1600);
    print_state();

    printf("Demo step 2: another pose, claw gap 3.0cm.\n");
    ESP_ERROR_CHECK(g_motion.set_durations_ms(
        800,
        1000,
        1000,
        700,
        500,
        500
    ));

    ESP_ERROR_CHECK(g_motion.move_to(
        25.0f,
        -15.0f,
        50.0f,
        55.0f,
        20.0f,
        3.0f
    ));

    delay_ms(1500);
    print_state();

    printf("Demo step 3: back to zero/open.\n");
    ESP_ERROR_CHECK(g_motion.set_strategies(
        JointMotionStyle::Smooth,
        JointMotionStyle::Smooth,
        JointMotionStyle::Smooth,
        JointMotionStyle::Smooth,
        JointMotionStyle::Smooth,
        JointMotionStyle::Smooth
    ));

    ESP_ERROR_CHECK(g_motion.move_to(
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        5.8f
    ));

    delay_ms(1500);
    print_state();

    printf("Demo finished.\n\n");
}

static void process_command(char* line)
{
    trim_line(line);

    if (line[0] == '\0') {
        return;
    }

    if (strcmp(line, "help") == 0 ||
        strcmp(line, "h") == 0 ||
        strcmp(line, "?") == 0) {
        print_help();
        return;
    }

    if (strcmp(line, "enable") == 0) {
        ESP_LOGW(TAG, "Batch enable all joints via arm_motion.enable_all()");
        const esp_err_t ret = g_motion.enable_all();
        printf("enable_all ret=0x%x\n", static_cast<unsigned>(ret));
        print_state();
        return;
    }

    if (strcmp(line, "disable") == 0 ||
        strcmp(line, "detach") == 0) {
        ESP_LOGW(TAG, "Disable all joints via arm_motion.disable_all()");
        const esp_err_t ret = g_motion.disable_all();
        printf("disable_all ret=0x%x\n", static_cast<unsigned>(ret));
        print_state();
        return;
    }

    if (strcmp(line, "default") == 0) {
        print_default_enable_pulses();
        return;
    }

    if (strcmp(line, "config") == 0) {
        print_config();
        return;
    }

    if (strcmp(line, "state") == 0 ||
        strcmp(line, "s") == 0) {
        print_state();
        return;
    }

    if (strcmp(line, "ready") == 0) {
        printf(
            "ready=%d moving=%d\n",
            static_cast<int>(g_motion.is_ready()),
            static_cast<int>(g_motion.is_moving())
        );
        return;
    }

    if (strcmp(line, "stop") == 0) {
        ESP_LOGW(TAG, "Stop motion and hold current commanded pose via arm_motion.stop_all()");
        const esp_err_t ret = g_motion.stop_all();
        printf("stop_all ret=0x%x\n", static_cast<unsigned>(ret));
        print_state();
        return;
    }

    if (starts_with(line, "dur ")) {
        uint32_t values[6] = {};
        char* args = line + strlen("dur ");

        if (!parse_six_u32(args, values)) {
            printf("Invalid durations. Usage: dur 1000 1200 1200 800 600 500\n");
            return;
        }

        const esp_err_t ret = g_motion.set_durations_ms(
            values[0],
            values[1],
            values[2],
            values[3],
            values[4],
            values[5]
        );

        printf("set_durations_ms ret=0x%x\n", static_cast<unsigned>(ret));
        print_config();
        return;
    }

    if (starts_with(line, "duration ")) {
        uint32_t values[6] = {};
        char* args = line + strlen("duration ");

        if (!parse_six_u32(args, values)) {
            printf("Invalid durations. Usage: duration 1000 1200 1200 800 600 500\n");
            return;
        }

        const esp_err_t ret = g_motion.set_durations_ms(
            values[0],
            values[1],
            values[2],
            values[3],
            values[4],
            values[5]
        );

        printf("set_durations_ms ret=0x%x\n", static_cast<unsigned>(ret));
        print_config();
        return;
    }

    if (starts_with(line, "style ")) {
        JointMotionStyle values[6] = {};
        char* args = line + strlen("style ");

        if (!parse_six_styles(args, values)) {
            printf("Invalid styles. Use linear/smooth/soft or 0/1/2.\n");
            return;
        }

        const esp_err_t ret = g_motion.set_strategies(
            values[0],
            values[1],
            values[2],
            values[3],
            values[4],
            values[5]
        );

        printf("set_strategies ret=0x%x\n", static_cast<unsigned>(ret));
        print_config();
        return;
    }

    if (starts_with(line, "styles ")) {
        JointMotionStyle values[6] = {};
        char* args = line + strlen("styles ");

        if (!parse_six_styles(args, values)) {
            printf("Invalid styles. Use linear/smooth/soft or 0/1/2.\n");
            return;
        }

        const esp_err_t ret = g_motion.set_strategies(
            values[0],
            values[1],
            values[2],
            values[3],
            values[4],
            values[5]
        );

        printf("set_strategies ret=0x%x\n", static_cast<unsigned>(ret));
        print_config();
        return;
    }

    if (starts_with(line, "speed ")) {
        float values[6] = {};
        char* args = line + strlen("speed ");

        if (!parse_six_float(args, values)) {
            printf("Invalid speeds. Usage: speed 45 35 35 50 70 2.5\n");
            return;
        }

        const esp_err_t ret = g_motion.set_speed_limits(
            values[0],
            values[1],
            values[2],
            values[3],
            values[4],
            values[5]
        );

        printf("set_speed_limits ret=0x%x\n", static_cast<unsigned>(ret));
        print_config();
        return;
    }

    if (starts_with(line, "movenow ")) {
        float values[6] = {};
        char* args = line + strlen("movenow ");

        if (!parse_six_float(args, values)) {
            printf("Invalid immediate move target. Usage: movenow 0 -30 45 60 0 5.8\n");
            return;
        }

        const esp_err_t ret = g_motion.move_to_immediate(
            values[0],
            values[1],
            values[2],
            values[3],
            values[4],
            values[5]
        );

        printf("move_to_immediate ret=0x%x\n", static_cast<unsigned>(ret));
        print_state();
        return;
    }

    if (starts_with(line, "deltanow ")) {
        float values[6] = {};
        char* args = line + strlen("deltanow ");

        if (!parse_six_float(args, values)) {
            printf("Invalid delta-immediate command. Usage: deltanow 1 0 0 0 0 0\n");
            return;
        }

        const esp_err_t ret = g_motion.move_delta_immediate(
            values[0],
            values[1],
            values[2],
            values[3],
            values[4],
            values[5]
        );

        printf("move_delta_immediate ret=0x%x\n", static_cast<unsigned>(ret));
        print_state();
        return;
    }

    if (starts_with(line, "jtool ") || starts_with(line, "jcam ")) {
        float values[4] = {};
        char* args = starts_with(line, "jtool ")
            ? line + strlen("jtool ")
            : line + strlen("jcam ");

        const int parsed = sscanf(
            args,
            "%f %f %f %f",
            &values[0],
            &values[1],
            &values[2],
            &values[3]
        );
        if (parsed < 3) {
            printf("Invalid tool target error. Usage: jtool F L U [desired_forward]\n");
            return;
        }

        const ArmKinematicsJointState joints = current_kinematics_joints();

        ArmToolTargetError error;
        error.forward_mm = values[0];
        error.left_mm = values[1];
        error.up_mm = values[2];
        error.desired_forward_mm = (parsed >= 4) ? values[3] : 0.0f;

        ArmKinematicsDelta delta;
        const esp_err_t solve_ret = g_kinematics.solve_tool_target_delta(
            joints,
            error,
            &delta
        );

        printf("solve_tool_target_delta ret=0x%x\n", static_cast<unsigned>(solve_ret));
        if (solve_ret != ESP_OK) {
            return;
        }

        printf(
            "tool delta deg: base=%+.3f shoulder=%+.3f elbow=%+.3f wrist_pitch=%+.3f wrist_roll=%+.3f\n",
            static_cast<double>(delta.base_delta_deg),
            static_cast<double>(delta.shoulder_delta_deg),
            static_cast<double>(delta.elbow_delta_deg),
            static_cast<double>(delta.wrist_pitch_delta_deg),
            static_cast<double>(delta.wrist_roll_delta_deg)
        );

        const esp_err_t move_ret = g_motion.move_delta_immediate(
            delta.base_delta_deg,
            delta.shoulder_delta_deg,
            delta.elbow_delta_deg,
            delta.wrist_pitch_delta_deg,
            delta.wrist_roll_delta_deg,
            0.0f
        );

        printf("move_delta_immediate ret=0x%x\n", static_cast<unsigned>(move_ret));
        print_state();
        return;
    }

    if (starts_with(line, "movespeed ")) {
        float values[6] = {};
        char* args = line + strlen("movespeed ");

        if (!parse_six_float(args, values)) {
            printf("Invalid speed move target. Usage: movespeed 0 -30 45 60 0 5.8\n");
            return;
        }

        const esp_err_t ret = g_motion.move_to_speed(
            values[0],
            values[1],
            values[2],
            values[3],
            values[4],
            values[5]
        );

        printf("move_to_speed ret=0x%x\n", static_cast<unsigned>(ret));
        print_state();
        return;
    }

    if (starts_with(line, "movev ")) {
        float values[6] = {};
        char* args = line + strlen("movev ");

        if (!parse_six_float(args, values)) {
            printf("Invalid speed move target. Usage: movev 0 -30 45 60 0 5.8\n");
            return;
        }

        const esp_err_t ret = g_motion.move_to_speed(
            values[0],
            values[1],
            values[2],
            values[3],
            values[4],
            values[5]
        );

        printf("move_to_speed ret=0x%x\n", static_cast<unsigned>(ret));
        print_state();
        return;
    }

    if (starts_with(line, "move ")) {
        float values[6] = {};
        char* args = line + strlen("move ");

        if (!parse_six_float(args, values)) {
            printf("Invalid move target. Usage: move 0 -30 45 60 0 5.8\n");
            return;
        }

        const esp_err_t ret = g_motion.move_to(
            values[0],
            values[1],
            values[2],
            values[3],
            values[4],
            values[5]
        );

        printf("move_to ret=0x%x\n", static_cast<unsigned>(ret));
        print_state();
        return;
    }

    if (strcmp(line, "zero") == 0 ||
        strcmp(line, "home") == 0) {
        const esp_err_t ret = g_motion.move_to(
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            5.8f
        );

        printf("move_to zero/open ret=0x%x\n", static_cast<unsigned>(ret));
        print_state();
        return;
    }

    if (strcmp(line, "demo") == 0) {
        run_demo_sequence();
        return;
    }

    printf("Unknown command: %s\n", line);
    printf("Type 'help' for commands.\n");
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "arm_motion V5A motion-owned test starting");

    ESP_ERROR_CHECK(
        g_servo.init(
            learm::arm_joint_defaults::kServoPwmConfigs,
            learm::arm_joint_defaults::kServoPwmConfigCount
        )
    );

    ESP_ERROR_CHECK(
        g_joints.init(
            &g_servo,
            learm::arm_joint_defaults::kJointCalibrations,
            learm::arm_joint_defaults::kJointCalibrationCount
        )
    );

    ESP_ERROR_CHECK(g_motion.init(&g_joints));
    ESP_ERROR_CHECK(g_kinematics.init(make_default_kinematics_config()));

    ESP_LOGW(TAG, "SAFE MODE: servo_pwm initialized, joints still disabled");
    delay_ms(3000);

    ESP_LOGW(TAG, "Batch enable all joints through g_motion.enable_all()");
    ESP_ERROR_CHECK(g_motion.enable_all());

    delay_ms(1000);

    print_help();
    print_default_enable_pulses();
    print_config();
    print_state();

    char line[160];

    printf("arm_motion_v5a> ");
    fflush(stdout);

    while (true) {
        if (fgets(line, sizeof(line), stdin) == nullptr) {
            delay_ms(100);
            continue;
        }

        process_command(line);

        printf("arm_motion_v5a> ");
        fflush(stdout);
    }
}