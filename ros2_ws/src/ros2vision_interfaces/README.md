# ros2vision_interfaces

## Purpose

`ros2vision_interfaces` contains custom ROS 2 interfaces used by ROS2vision.

At the current stage, this package is no longer only a placeholder. It now provides the first custom message used by the recognition layer.

## Current provided interface

### `msg/Target.msg`

Current intent:

- publish whether a target is currently detected
- publish which recognition mode produced the target
- publish target label information
- publish image size
- publish target center in pixel coordinates
- publish target center in normalized coordinates
- publish target bounding box
- publish basic area / score metadata

## Current usage in the repository

Current producer:

- `recognition_pkg/target_detector_node`

Current intended consumer:

- future `control_pkg`

## Current status

At the current stage, `Target.msg` is the first structured recognition-to-control contract in the repository.