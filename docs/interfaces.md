# Interfaces

[README](../README.md) / Interfaces

This page defines the current ROS messages and the two USB serial command interfaces. UNO commands and ESP32 commands are separate protocols; the existing UNO bridge cannot directly control the arm. Future communication choices are described in [Architecture](architecture.md#host-to-mcu-communication).

## ROS topics

The table lists default topic names. Source paths and YAML files may override them.

| Topic | Type | Producer → consumer |
| --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | `usb_cam` → preprocessor |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | `usb_cam` → optional consumers |
| `/recognition/preprocessed/image` | `sensor_msgs/msg/Image` | Preprocessor → detector |
| `/recognition/preprocessed/mask` | `sensor_msgs/msg/Image` | Color preprocessor → color detector |
| `/recognition/preprocessed/debug_image` | `sensor_msgs/msg/Image` | Preprocessor → optional viewer |
| `/recognition/detection/debug_image` | `sensor_msgs/msg/Image` | Detector → debug viewer |
| `/recognition/target` | `ros2vision_interfaces/msg/Target` | Detector → UNO follower |
| `/control/angle_command` | `std_msgs/msg/Float32` | Follower → UNO bridge |
| `/control/raw_serial_command` | `std_msgs/msg/String` | Optional command publisher → UNO bridge |

The raw serial topic is enabled in the supplied control profile. It sends text through the UNO bridge; it is not a structured arm interface.

Most internal publishers/subscribers use queue depth 10. The debug viewer uses Best Effort with Keep Last depth 1. Its image subscription policy does not change the rest of the pipeline.

### Target message

| Field group | Meaning |
| --- | --- |
| `header` | Copied from the source image |
| `mode`, `label`, `detected` | Recognition mode, target label, and detection validity |
| `image_width`, `image_height` | Dimensions of the image used by the detector |
| `center_x`, `center_y` | Target center in image pixels |
| `center_x_norm`, `center_y_norm` | Normalized center coordinates |
| `bbox_x`, `bbox_y`, `bbox_w`, `bbox_h` | Bounding rectangle in pixels |
| `area`, `score` | Target area and an area-derived score |

Normalized coordinates are computed as:

```text
center_x_norm = 2 * center_x / (image_width  - 1) - 1
center_y_norm = 2 * center_y / (image_height - 1) - 1
```

The origin is the image center; positive X points right and positive Y points down. A dimension of one or less produces zero for that normalized coordinate.

Coordinates refer to the preprocessed image, including any aspect-ratio padding. They are not 3D positions. On detection loss, `detected=false` and the numeric target fields are zeroed.

The current score is `min(1, area / (image_area * 0.25))`. It is not a model confidence probability. Face area is rectangle area; color area is contour area.

Sources: [Target.msg](../ros2_ws/src/ros2vision_interfaces/msg/Target.msg), [target publisher](../ros2_ws/src/recognition_pkg/recognition_pkg/target_detector_node.py), [detectors](../ros2_ws/src/recognition_pkg/recognition_pkg/detectors/).

### UNO follower profile

The supplied [control_params.yaml](../ros2_ws/src/control_pkg/config/control_params.yaml) uses:

| Parameter | Profile value |
| --- | --- |
| Smoothing alpha | 0.35 |
| Horizontal deadband / resume threshold | 0.08 / 0.12 |
| Target reception timeout | 0.5 s |
| Command cooldown | 0.08 s |
| Mapping | Stepwise corrections of 1°, 2°, 4°, or 6° |
| Direction inversion | Enabled |
| Bridge send interval / busy hold | 0.05 s / 0.05 s |

The default mapping switches at error magnitudes 0.15, 0.30, and 0.50. A proportional mapping is also implemented. Only horizontal error is consumed.

Directly running a node without its YAML can use different fallback defaults. Treat the profile above as the configured launch behavior.

## UNO serial protocol

The project's UNO path uses USB serial at 115200 baud with line-delimited text. Command matching is case-insensitive. The input buffer is 64 bytes including the string terminator.

| Command | Meaning |
| --- | --- |
| `HELP`, `PING` | Print commands; respond to ping with `OK PONG` |
| `STEP n` | Move by signed relative logical steps |
| `ANG deg` | Move by signed relative degrees |
| `REV r` | Move by signed relative revolutions |
| `STOP` | Stop stepping at the current logical count |
| `ZERO` | Set the logical origin while idle |
| `POS?`, `STATE?` | Print logical position or controller state |
| `SPR value` | Set steps per revolution while idle |
| `DELAY ms` | Set step interval while idle; minimum 1 ms |
| `HOLD ON` / `HOLD OFF` | Configure coil holding after a move/stop while idle |
| `DIRINV ON` / `DIRINV OFF` | Configure logical-to-electrical direction while idle |
| `RELEASE` | Stop and de-energize the coils |

A nonzero motion command replaces the previous target relative to the current logical position. Commands are not queued.

Responses include `OK ...`, `ERR ...`, `DONE STEPS ... ANGLE ...`, and `STOPPED STEPS ... ANGLE ...`. Position reports count issued logical steps, without an encoder measurement.

The ROS bridge uses `PING` / `OK PONG` for its handshake. It can clear busy state on completion feedback or when the configured busy hold expires; it does not always wait for physical move completion before accepting another correction.

The follower's lost-target handler resets its control state and stops issuing corrections. It does not emit `STOP`. The UNO has no streaming-command watchdog in this implementation.

Sources: [UNO firmware](../firmware/uno_controller/uno_controller.ino), [serial bridge](../ros2_ws/src/control_pkg/control_pkg/bridges/uno_serial_bridge_node.py).

## ESP32 arm serial protocol

The current application is accessed over USB serial using a 115200-baud console and case-sensitive, line-delimited commands. Its line buffer is 160 bytes. Console output mixes logs and diagnostic text; there is no request-ID or structured acknowledgement protocol.

| Command | Meaning |
| --- | --- |
| `help` | Print current command help |
| `state` | Print ownership, motion/output state, references, and watchdog/driver diagnostics |
| `fk` | Compute forward pose from software reference angles |
| `jerr F L U P` | Submit normalized visual error in Track mode |
| `pos B S E WP WR GAP` | Set absolute rotary angles in degrees and claw gap in centimeters |
| `stop` | Stop rotary reference motion and restore Track ownership |
| `disable` | Disable PWM outputs and restore Track ownership |
| `enable 0` | While disabled, enable at calibration-zero rotary angles with the claw closed |

### Visual-error fields

`jerr` takes four finite numbers. The application clamps each to `[-1, 1]` before applying its fixed gains.

| Field | Meaning | Gain | Maximum requested task speed |
| --- | --- | --- | --- |
| F | Forward error along the solver's forward axis | 30 mm/s per unit | 30 mm/s |
| L | Left error along the solver's left axis | 220 mm/s per unit | 150 mm/s |
| U | Up error along the solver's up axis | 300 mm/s per unit | 200 mm/s |
| P | Pitch error | 20°/s per unit | 20°/s |

The command values are dimensionless errors, not millimeters, angles, or joint velocities. The application fixes roll velocity to zero and retains the existing claw target.

The solver constructs its forward/left/up axes from base yaw and the combined pitch angles. Wrist roll is a separate orientation channel and does not rotate those axes in the current implementation. The image-to-solver frame mapping must be defined during ROS 2 integration.

> **Planned integration:** a direct copy of existing `Target` normalized coordinates would have the wrong sign for left/up image error. Define sign, reference center, padding, and camera orientation explicitly.

### Command ownership and stopping

| Event | Application ownership | Rotary reference behavior |
| --- | --- | --- |
| Startup | Track | Outputs are enabled after three seconds at the configured initial pose |
| Accepted `jerr` | Track | Enters/refreshes Velocity mode |
| Successful `pos` | Pos | Enters Position mode; finite targets are clamped to configured ranges |
| Position reference reached | Remains Pos | Enters Hold; subsequent `jerr` is still ignored |
| `stop` | Track | Decelerates through Stopping, or holds immediately if already stationary |
| Velocity command older than 300 ms | Remains Track | Decelerates through Stopping |
| `disable` | Track | PWM outputs are disabled |

The nominal motion update period is 20 ms; integration uses measured elapsed time. The 300 ms watchdog applies to streaming velocity, not finite position moves.

`stop` leaves PWM enabled and does not cancel the claw's independent gap target. Once Track is restored, a new `jerr` can command motion again, including during deceleration. Stop the sending source when requesting a lasting pause.

Software reference arrival, output enable state, and `fk` do not prove physical actuator position.

Sources: [application and gains](../firmware/arm_controller/main/main.cpp), [motion parameters](../firmware/arm_controller/components/arm_motion/include/arm_motion.hpp), [kinematics](../firmware/arm_controller/components/arm_kinematics/arm_kinematics.cpp).

## Interface work to complete

The tables above specify the implemented console protocol. For ROS 2 integration, define control meaning separately from its wire representation. A host bridge using the current protocol would encode visual error as `jerr`; a micro-ROS implementation could carry the same agreed fields in a structured message. Future Wi-Fi or Bluetooth framing is undecided.

> **TODO — ROS 2 arm messages:** define target validity, normalized error, timestamps, ownership requests, and connection diagnostics. Decide whether to extend `ros2vision_interfaces` or reuse standard messages.

> **TODO — Connection and command contract:** specify startup synchronization, reconnect behavior, stale-command rejection, error reporting, and protocol versioning for the selected integration method. Define how target loss or connection loss requests a pause, and how control resumes. If multiple command inputs are retained, define which input owns the arm.

> **TODO — Configuration reference:** add a complete parameter reference when the ROS 2 arm interfaces stabilize. For current defaults, follow the linked source headers and YAML profiles.
