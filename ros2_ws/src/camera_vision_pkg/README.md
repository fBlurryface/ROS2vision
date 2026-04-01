# camera_vision_pkg

## Purpose

`camera_vision_pkg` is the current camera acquisition package for ROS2vision.

Its role is intentionally narrow:

- start the camera
- publish raw image output
- publish `camera_info`
- support startup mode selection
- tolerate validated same-port disconnect/reconnect behavior

It is not the package for downstream image processing.

## Current output role

This package is the raw image source layer.

Expected downstream consumers may later perform:

- grayscale conversion
- undistortion
- rectify
- recognition preprocessing
- detector/tracker input preparation

## Current device path policy

Do not rely on drifting `/dev/videoN`.

Current persistent alias:

- `/dev/ros2vision_camera`

## Current startup modes

Supported modes:

- `vga`
  - 640 x 480
- `wide`
  - 800 x 480
- `hd`
  - 1280 x 720

## Current format policy

Current implementation keeps format policy fixed:

- pixel format: `mjpeg2rgb`
- I/O method: `mmap`

## Launch usage

Typical usage:

```bash
ros2 launch camera_vision_pkg camera_source.launch.py
```

Explicit mode selection:

```bash
ros2 launch camera_vision_pkg camera_source.launch.py mode:=vga
ros2 launch camera_vision_pkg camera_source.launch.py mode:=wide
ros2 launch camera_vision_pkg camera_source.launch.py mode:=hd
```

Explicit device override:

```bash
ros2 launch camera_vision_pkg camera_source.launch.py mode:=vga video_device:=/dev/ros2vision_camera
```

## Expected topics

Current primary outputs:

- `/camera/image_raw`
- `/camera/camera_info`

## Current validated behavior

Validated so far:

- package builds successfully as a ROS 2 package
- camera stream starts successfully
- `vga`, `wide`, and `hd` all launch successfully
- all three modes currently observe practical frame rates around ~16.6 Hz
- same-port disconnect/reconnect recovery works in current testing

## Known limitations

Known limitations at the current stage:

- reconnect through a different physical USB port is not yet guaranteed
- calibration file is not yet provided
- some unsupported V4L2 control warnings still appear
- practical frame rate is below the configured target of 20 Hz

## Practical debug commands

Check package discovery:

```bash
colcon list
```

Build only this package:

```bash
colcon build --packages-select camera_vision_pkg --symlink-install
```

Inspect running topic rate:

```bash
ros2 topic hz /camera/image_raw -w 100
```

Inspect topic presence:

```bash
ros2 topic list | grep camera
```

Inspect camera info once:

```bash
ros2 topic echo /camera/camera_info --once
```

Check available V4L2 formats:

```bash
v4l2-ctl --list-formats-ext --device=/dev/ros2vision_camera
```
