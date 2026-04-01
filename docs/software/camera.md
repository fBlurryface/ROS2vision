# Camera subsystem notes

## Purpose

This document records the current camera acquisition design decisions for ROS2vision.

The camera subsystem is currently designed as a source-layer component. Its responsibility is limited to starting the camera, maintaining acquisition, handling same-port reconnect behavior, and publishing raw image output.

## Current package

- Package: `ros2_ws/src/camera_vision_pkg`

This package is currently the first ROS 2 package in the repository with a validated bringup path.

## Current node responsibility

The current camera entrypoint is responsible for:

- selecting startup mode by resolution
- opening the camera through a persistent device alias
- publishing raw image stream
- publishing `camera_info`
- surviving same-port disconnect/reconnect through a recovery loop

The following are intentionally not part of this node:

- grayscale conversion
- undistortion
- rectify / calibration output transforms
- recognition-specific preprocessing
- target detection / tracking logic

Those belong to downstream nodes.

## Device naming strategy

The repository no longer treats `/dev/videoN` as a reliable long-term device path.

Current strategy:

- use a persistent alias: `/dev/ros2vision_camera`
- avoid direct dependency on dynamic V4L2 node numbering
- rely on udev naming to reduce device index drift after reconnect

This solves the device-number drift problem for the validated same-port reconnect case.

## Recovery strategy

The current camera entrypoint has been upgraded beyond a simple static launch wrapper.

Behavior:

- wait for the persistent device alias to become available
- start the underlying `usb_cam` process
- if the underlying process exits, wait for the device again
- restart acquisition when the device becomes available again

This is sufficient for same-port reconnect recovery in current testing.

## Startup modes

Current startup modes:

- `vga`
  - 640 x 480
- `wide`
  - 800 x 480
- `hd`
  - 1280 x 720

Current design intent:

- resolution is the only user-facing mode distinction
- pixel format is fixed for now
- mode switching is done at startup, not dynamically at runtime

## Image format policy

The current implementation intentionally keeps image format policy simple.

Current choice:

- `pixel_format: mjpeg2rgb`
- `io_method: mmap`

This keeps the package focused on practical bringup and recovery instead of exposing too many low-level toggles too early.

## Current observed runtime behavior

Based on current validation:

- `vga`, `wide`, and `hd` all start successfully
- all three currently run at roughly ~16.6 Hz in observed testing
- same-port disconnect/reconnect can recover
- moving the cable to a different physical USB port is not yet guaranteed to recover automatically

## Known limitations

Current limitations include:

- reconnect through a different physical USB port is not yet guaranteed
- camera calibration file is not yet provided
- some V4L2 control warnings still appear for unsupported controls on this camera
- actual measured frame rate is below the configured 20 Hz target
- the package still relies on `usb_cam` rather than a fully custom capture backend

## Next likely steps

Reasonable next steps after the current milestone:

- document and stabilize deployment on the industrial PC
- evaluate why measured runtime remains around ~16.6 Hz
- decide whether cross-port reconnect should be supported
- begin downstream image-processing nodes that consume raw output
