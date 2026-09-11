# Bringup

[README](../README.md) / Bringup

Use this guide for the paths already implemented in the repository. Both MCU procedures use USB serial. The ROS 2 system launch targets UNO; the ESP32 arm is built and operated separately. Wi-Fi, Bluetooth, and micro-ROS are [candidates for evaluation](progress.md#communication-options-to-evaluate).

## Before starting

| Path | Required environment |
| --- | --- |
| ROS 2 host | Linux with ROS 2 Jazzy, `rosdep`, `colcon`, and a usable USB camera |
| UNO | Arduino CLI with the AVR core, plus USB serial access to the UNO |
| ESP32 arm | ESP-IDF environment matching the project configuration, plus USB serial access to the board |

ROS workspace CI uses Ubuntu 24.04 (Noble). The committed arm `sdkconfig` records ESP32 and ESP-IDF 6.0.1. Use the [manufacturer references](hardware.md#hardware-reference) to identify the matching controller hardware; record the tested board revision and toolchain in the [validation record](progress.md#validation-record).

Commands below use Bash. Install the toolchains before following the project-specific steps. The account running the tools must be able to access the selected camera and serial devices.

## ROS 2 host

### Build the workspace

Clone the repository, then work from `ROS2vision/ros2_ws`:

```bash
git clone https://github.com/fBlurryface/ROS2vision.git
cd ROS2vision/ros2_ws
source /opt/ros/jazzy/setup.bash

rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

If this machine has never initialized rosdep, run `sudo rosdep init` once before `rosdep update`.

The camera runner also requires `v4l2-ctl`. On Ubuntu:

```bash
sudo apt install v4l-utils
v4l2-ctl --list-devices
```

In each new ROS terminal, return to this workspace and source both the Jazzy environment and `install/setup.bash`.

### Prepare the camera

The default device is `/dev/ros2vision_camera`. The earlier deployment used a local udev rule to create that alias; the rule is not in the repository yet.

Use `v4l2-ctl --list-devices` to identify the image-capture node. If the alias is unavailable, pass the actual node as `video_device`. The following uses `/dev/video0` as an example; replace it with the node you identified.

### Run camera and recognition

```bash
ros2 launch ros2vision_bringup full_system.launch.py \
  enable_control:=false \
  video_device:=/dev/video0
```

With the persistent alias configured, omit `video_device`. The launch defaults to VGA acquisition and face-mode recognition. Other camera modes are `wide` and `hd`.

In a second sourced terminal, inspect the outputs:

```bash
ros2 topic hz /camera/image_raw
ros2 topic echo /recognition/target --once
```

Run the optional viewer in another sourced terminal with a graphical session:

```bash
ros2 launch recognition_pkg tracking_debug_viewer.launch.py
```

The viewer has its own launch file and is not included in `full_system.launch.py`.

### Select color mode

Switch both preprocessing and detection configuration:

```bash
RECOGNITION_SHARE="$(ros2 pkg prefix --share recognition_pkg)"
ros2 launch ros2vision_bringup full_system.launch.py \
  enable_control:=false \
  recognition_params_file:="$RECOGNITION_SHARE/config/recognition_params.color.yaml" \
  detector_params_file:="$RECOGNITION_SHARE/config/target_detector.color.yaml"
```

Add `video_device:=...` when the persistent alias is unavailable. Color thresholds and minimum area are set in those YAML files.

### Add UNO control

First flash the UNO firmware using the instructions below. Check the wiring, actual serial port, and logical direction against [Hardware](hardware.md#uno-stepper-wiring).

Edit `src/control_pkg/config/control_params.yaml` for the local machine, especially `serial_port` and `invert_direction`. Then launch using that file explicitly:

```bash
ros2 launch ros2vision_bringup full_system.launch.py \
  control_params_file:="$PWD/src/control_pkg/config/control_params.yaml"
```

The system launch enables control by default. For a camera/perception session, continue using `enable_control:=false`.

Expected connection messages include a successful `PING` / `OK PONG` handshake. Inspect the generated correction topic with:

```bash
ros2 topic echo /control/angle_command
```

The supplied launch profile differs from some direct-node fallback defaults. Use the YAML profile when reproducing the documented behavior.

## UNO firmware

From the repository root, with Arduino CLI installed:

```bash
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli compile --fqbn arduino:avr:uno firmware/uno_controller
arduino-cli board list
```

Select the actual UNO port before uploading:

```bash
UNO_PORT=/dev/ttyACM0
arduino-cli upload --port "$UNO_PORT" --fqbn arduino:avr:uno firmware/uno_controller
```

The port above is an example. USB serial communication uses 115200 baud. A serial terminal can send `PING`, `HELP`, and `STATE?` to inspect the controller. Close the terminal before letting the ROS bridge open the same port.

## ESP32 arm controller

For board, connector, and assembly information, follow the [official hardware references](hardware.md#hardware-reference). The steps below build and run this repository's ESP-IDF firmware.

### Build and flash

Activate the matching ESP-IDF environment, then work from `firmware/arm_controller`:

```bash
cd firmware/arm_controller
idf.py build
```

Read the [startup pose and calibration notes](hardware.md#startup-pose) before flashing or resetting a connected arm.

**On boot, the application waits three seconds and then automatically enables outputs at its configured initial pose.** It has no sensor-based homing. Place the physical arm near that pose before power-on.

Select the actual ESP32 port before flashing:

```bash
ARM_PORT=/dev/ttyUSB0
idf.py -p "$ARM_PORT" flash monitor
```

The port above is an example. The console is configured for 115200 baud. Exit IDF Monitor with `Ctrl+]`.

### Inspect the controller

At the firmware console:

```text
help
state
fk
```

Use [Interfaces](interfaces.md#esp32-arm-serial-protocol) when sending position or tracking commands. `stop` decelerates rotary references and restores Track ownership; `disable` disables PWM outputs. The claw's finite gap target is independent of rotary stopping.

USB serial command access is available independently of ROS 2. The current workspace has no ROS 2 arm integration or arm launch entry.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| Camera runner keeps waiting | Device alias, selected capture node, access permissions, and availability of `v4l2-ctl` |
| Face detector cannot load its model | OpenCV Haar cascade data; set `face_cascade_path` in the face detector YAML if needed |
| Color detector publishes no target | Both nodes must use color configuration; check mask output and area/HSV thresholds |
| UNO handshake fails | Port ownership, baud, matching UNO firmware, and startup/reset waiting |
| ESP32 ignores `jerr` after positioning | Check `app=pos`; an explicit `stop` restores Track ownership |
| ESP32 stops responding to a velocity stream | Check command freshness and the 300 ms watchdog in `state` |
| Viewer does not open | A graphical session is required when `show_window=true` |

## Deployment notes to complete

Earlier industrial-PC work used `~/deployments/ros2vision/repo` for a sparse checkout and `~/deployments/ros2vision/current` as the active workspace entrypoint. Pull, build, launch, and hardware checks were manual.

> **TODO — Reproducible host setup:** add the tested OS/tool versions, required device permissions, exact udev rule, and the relationship between the checkout and `current`.

> **TODO — Deployment lifecycle:** document start/stop, restart, logging, and rollback procedures. Automated CD remains a future direction.

Validation history and remaining checks are maintained in [Project Progress](progress.md#validation-record).
