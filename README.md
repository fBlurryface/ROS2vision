# ROS2vision

ROS 2 Jazzy visual closed-loop project with an industrial PC camera pipeline and Arduino UNO actuator control.

## Repository structure

- `.github/workflows/`
  - CI workflows for ROS 2 workspace and Arduino firmware
- `docs/`
  - System architecture, hardware notes, protocol notes, software notes, deployment notes
- `firmware/uno_controller/`
  - Arduino UNO firmware for actuator-side control
- `ros2_ws/src/camera_vision_pkg/`
  - Camera acquisition package
- `ros2_ws/src/control_pkg/`
  - Motion/control-side ROS 2 package scaffold
- `ros2_ws/src/recognition_pkg/`
  - Vision recognition package with preprocessing and target detection nodes
- `ros2_ws/src/ros2vision_interfaces/`
  - Custom ROS 2 interfaces for structured target output

## Current project status

The repository has completed its initial engineering scaffold and now has its first usable recognition-side detection path:

- ROS 2 workspace skeleton is in place
- GitHub Actions CI exists for ROS 2 Jazzy workspace
- GitHub Actions CI exists for Arduino firmware compilation
- UNO controller firmware has an initial implementation
- `camera_vision_pkg` is now a real ROS 2 package and has become the first package with a validated bringup path
- `recognition_pkg` now has two validated nodes:
  - `image_preprocessor_node`
  - `target_detector_node`
- `ros2vision_interfaces` now provides the first custom target message used by the recognition layer:
  - `Target.msg`

## Camera subsystem status

The first usable version of the camera acquisition path is now available.

Current characteristics:

- Package: `ros2_ws/src/camera_vision_pkg`
- Output role: raw image source only
- Image processing such as grayscale conversion, undistortion, rectification, and recognition pre-processing is intentionally left to downstream nodes
- Supported startup modes:
  - `vga`
  - `wide`
  - `hd`
- Device path strategy:
  - do not rely on drifting `/dev/videoN`
  - use persistent alias `/dev/ros2vision_camera`
- Recovery behavior:
  - same-port USB disconnect/reconnect has been validated
  - reconnect through a different physical USB port is not yet guaranteed
- Observed runtime:
  - all three validated startup modes are currently around ~16.6 Hz in practical testing

## Recognition subsystem status

The recognition package has now moved beyond pure preprocessing and has reached its first structured target-output milestone.

Current characteristics:

- Package: `ros2_ws/src/recognition_pkg`
- Current node chain:
  - `image_preprocessor_node`
  - `target_detector_node`
- Supported current modes:
  - `face`
  - `color`
- Current recognition outputs:
  - `/recognition/preprocessed/image`
  - `/recognition/preprocessed/debug_image`
  - `/recognition/preprocessed/mask`
  - `/recognition/detection/debug_image`
  - `/recognition/target`
- Current practical status:
  - `face` mode is currently the more stable validated path
  - `color` mode works, but still requires threshold and robustness tuning

See detailed notes here:

- `docs/software/camera.md`
- `docs/software/recognition.md`
- `docs/deployment/industrial_pc_setup.md`
- `ros2_ws/src/camera_vision_pkg/README.md`
- `ros2_ws/src/recognition_pkg/README.md`
- `ros2_ws/src/ros2vision_interfaces/README.md`

## Recommended reading order

If you are new to the project:

1. Read `docs/architecture.md`
2. Read `docs/software/camera.md`
3. Read `docs/software/recognition.md`
4. Read `docs/deployment/industrial_pc_setup.md`
5. Read package-level README files as needed

## Development note

At the current stage, the camera package is still the most mature source-layer package in the repository.

Recognition has now moved beyond scaffold status and has reached its first structured target-output stage, while `control_pkg` remains the next major implementation target for completing the end-to-end closed-loop path.