# Hardware

[README](../README.md) / Hardware

This page connects manufacturer hardware references with the configuration used by ROS2vision. Pin assignments, calibration, geometry, and startup pose describe the current firmware. Product specifications and standard assembly instructions are linked in the relevant section.

## Hardware inventory

| Item | Project configuration | Reference or setup record |
| --- | --- | --- |
| Linux host | ROS 2 Jazzy workspace and industrial-PC deployment notes | Host environment and USB setup in the [bringup record](bringup.md#deployment-notes-to-complete) |
| USB camera | V4L2 / `usb_cam` acquisition | Device identity, mounting, and calibration in [Camera setup](#camera-setup) |
| UNO stepper assembly | Arduino UNO, ULN2003 driver, and 28BYJ-48 half-step sequence | Project wiring and rotation scale in [UNO stepper wiring](#uno-stepper-wiring) |
| Arm controller | ESP32 target with USB serial command access | Controller and core-board details in [Hardware reference](#hardware-reference) |
| Arm actuators | Five rotary PWM servos and one claw servo | Manufacturer references plus the [project configuration and calibration](#project-configuration-and-calibration) |

Record project-specific hardware substitutions or wiring changes alongside the relevant section. Standard arm parts, specifications, and assembly procedures are maintained in the manufacturer documentation.

## Camera setup

The configured acquisition modes are:

| Mode | Image size |
| --- | --- |
| `vga` | 640 × 480 |
| `wide` | 800 × 480 |
| `hd` | 1280 × 720 |

All three configurations request 20 Hz and use `mjpeg2rgb` with `mmap`. Earlier testing reported approximately 16.6 Hz; see the dated [validation record](progress.md#validation-record).

The preferred device path is `/dev/ros2vision_camera`. The original host used a custom udev alias. Same-port reconnect was reported working; cross-port recovery remains unverified.

> **TODO — Device identity and calibration:** add the exact udev rule, a camera calibration file, and a recorded capture format. The current `camera_info_url` is empty.

> **TODO — Camera mount:** add a photograph or drawing showing the camera relative to the arm, its image orientation, and the frame relationship used for visual control.

Source: [camera configuration](../ros2_ws/src/camera_vision_pkg/config/).

## UNO stepper wiring

The firmware constructs the stepper as `StepperDevice(8, 9, 10, 11)`:

| UNO pin | ULN2003 input |
| --- | --- |
| D8 | IN1 |
| D9 | IN2 |
| D10 | IN3 |
| D11 | IN4 |

The driver uses an eight-state half-step sequence, a default of 4096 logical steps per revolution, and a default 2 ms step interval. These are configurable through the [UNO protocol](interfaces.md#uno-serial-protocol).

The initial logical position is zero. There is no homing sensor or encoder feedback in this implementation. `ZERO` changes the logical origin while idle.

> **TODO — Wiring diagram:** document the motor supply, signal reference/ground connections, connector orientation, and host-to-UNO cable. Add the mechanical sign check and measured rotation scale.

Sources: [UNO sketch](../firmware/uno_controller/uno_controller.ino), [stepper driver](../firmware/uno_controller/stepper_device.h).

## Arm hardware and calibration

### Hardware reference

The firmware configuration uses an ESP32 controller and six PWM servos. The following LeArm AI manufacturer references provide the hardware documentation entry points:

| Official reference | Relevant content |
| --- | --- |
| [LeArm AI documentation](https://docs.hiwonder.com/projects/LeArm_AI/en/latest/) | Manual index and supporting resources |
| [Getting Ready](https://docs.hiwonder.com/projects/LeArm_AI/en/latest/docs/1.Geting_Ready.html) | Sections 1.3.3–1.3.4: controller and core boards, including ESP32; section 1.4: assembly |
| [Basic Development Course](https://docs.hiwonder.com/projects/LeArm_AI/en/latest/docs/5.Basic_Development_Course.html) | Section 5.5: PWM servo connections and control examples |

These pages cover several controller and servo variants. Use the material corresponding to the ESP32/PWM assembly. Manufacturer software examples describe the factory software; this project's build and operating procedure is in [Bringup](bringup.md#esp32-arm-controller).

> **TODO — Reference match:** identify the kit/controller revision covered by the linked manual and note any project-specific changes to the standard assembly. Add a more specific manufacturer link where needed.

### Project configuration and calibration

The table below records how the firmware maps joints to PWM outputs. Its zero points, direction conventions, and limits belong to this project configuration.

| Joint | Channel | ESP32 GPIO | Pulse at semantic zero | Pulse direction for increasing command | Command range |
| --- | --- | --- | --- | --- | --- |
| Claw | S0 | 19 | 500 µs | Increasing closure increases pulse | Closure scale 0–100 |
| Wrist roll | S1 | 18 | 1480 µs | Decreases | −85° to +85° |
| Wrist pitch | S2 | 5 | 1580 µs | Decreases | −85° to +85° |
| Elbow | S3 | 4 | 1490 µs | Decreases | −85° to +85° |
| Shoulder | S4 | 0 | 1530 µs | Decreases | −85° to +85° |
| Base | S5 | 15 | 1450 µs | Increases | −85° to +85° |

The five rotary ranges are software limits from the current calibration. The base uses 10.89 µs per degree; the other rotary joints use 2000/180 µs per degree. PWM frequency is 50 Hz.

The claw's internal closure scale is not a physical angle. Application commands use gap in centimeters. The firmware maps gap to pulse width through a measured, piecewise table:

- 5.8 cm open corresponds to 500 µs.
- 0 cm closed corresponds to 1600 µs.
- Intermediate values use the calibration table rather than one linear conversion.

The claw output is limited to 500–1600 µs. Other channels are configured for 500–2500 µs, further constrained by their joint mappings.

The manufacturer's PWM example describes 500 µs as closing the gripper; this project's calibration maps 500 µs to a 5.8 cm opening. Follow the project mapping for this firmware and verify it when changing the assembly. See the [manufacturer example, section 5.5](https://docs.hiwonder.com/projects/LeArm_AI/en/latest/docs/5.Basic_Development_Course.html).

The authoritative arrays and claw table are in [arm_joint.cpp](../firmware/arm_controller/components/arm_joint/arm_joint.cpp).

> **TODO — Calibration procedure:** document joint-zero alignment, pulse measurements, allowed travel checks, claw-gap measurement, and the process for updating source defaults.

### Geometry

| Model quantity | Value |
| --- | --- |
| Base-to-shoulder vertical offset | 28.9 mm |
| Upper arm | 104.3 mm |
| Forearm | 89.0 mm |
| Wrist-to-tool reference point | 177.0 mm |

These values come from [ArmKinematicsParameters](../firmware/arm_controller/components/arm_kinematics/include/arm_kinematics.hpp).

> **TODO — Geometry drawing:** identify each joint axis, the tool reference point, link measurement endpoints, and the positive rotation directions. Confirm the dimensions against the assembly revision.

### Startup pose

After initializing the software, the application waits three seconds and automatically enables outputs at:

| Base | Shoulder | Elbow | Wrist pitch | Wrist roll | Claw gap |
| --- | --- | --- | --- | --- | --- |
| 0° | −60° | +70° | +70° | 0° | 0 cm |

The physical arm must be placed near this pose before power-on. This command initializes the software reference; it does not measure or discover the physical pose.

After outputs have been disabled, `enable 0` uses calibration-zero rotary angles and a closed claw. That is a different pose from the boot pose.

Source: [application startup](../firmware/arm_controller/main/main.cpp).
