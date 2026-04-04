# Recognition subsystem notes

## Purpose

This document records the current recognition-side design decisions for ROS2vision.

The recognition subsystem has now moved beyond pure preprocessing. It currently consists of a preprocessing node followed by a target detection node that publishes structured target output for downstream control work.

## Current package

- Package: `ros2_ws/src/recognition_pkg`

This package is now the second ROS 2 package in the repository to move beyond scaffold state.

Its currently validated nodes are:

- `image_preprocessor_node`
- `target_detector_node`

## Current subsystem responsibility

The recognition subsystem is intended to sit between the camera source layer and the control layer.

Current design direction:

- `camera_vision_pkg`
  - provide stable image source
- `recognition_pkg`
  - prepare task-oriented image data
  - detect a current primary target
  - publish structured target output
- `control_pkg`
  - consume target information and generate motion decisions

## Current implemented nodes

### `image_preprocessor_node`

Current role:

- subscribe to raw image input from camera layer
- prepare task-oriented preprocessed image output
- publish preprocessed outputs for downstream recognition nodes
- support mode switching based on downstream task type

This node is intentionally not the detector itself.

### `target_detector_node`

Current role:

- consume the outputs of `image_preprocessor_node`
- switch detector behavior by mode
- publish a structured current primary target
- publish a detector-side debug image with target overlay

This node is the first recognition-side node that produces semantic target output for future control logic.

## Current node chain

Current intended chain:

- `/camera/image_raw`
  - published by camera layer
- `image_preprocessor_node`
  - publishes recognition-oriented image outputs
- `target_detector_node`
  - publishes structured target information
- future `control_pkg`
  - consumes target information for motion decisions

## Current input / output contract

### Inputs to `image_preprocessor_node`

Current expected input:

- `/camera/image_raw`

### Outputs from `image_preprocessor_node`

Current outputs:

- `/recognition/preprocessed/image`
- `/recognition/preprocessed/debug_image`

Additional output in color mode:

- `/recognition/preprocessed/mask`

### Inputs to `target_detector_node`

Current expected inputs:

- `/recognition/preprocessed/image`

Additional input in color mode:

- `/recognition/preprocessed/mask`

### Outputs from `target_detector_node`

Current outputs:

- `/recognition/target`
- `/recognition/detection/debug_image`

## Current structured target output

The first custom target message is now available through:

- package: `ros2_ws/src/ros2vision_interfaces`
- message: `Target.msg`

Current intent of this message:

- report whether a target is currently detected
- report which mode produced the target
- report target center position
- report normalized target center position
- report bounding box information
- report basic area / score metadata

This is the current contract intended for the future control layer.

## Current supported modes

### `face`

This mode is intended for face-related tasks.

Current preprocessing characteristics:

- resize to configured output size
- optional aspect-ratio preservation
- current default output encoding is `rgb8`
- optional grayscale / equalization / blur-related options exist in parameters

Current target detection characteristics:

- consume preprocessed image input
- use a Haar Cascade based face detector in the current implementation
- select one current primary face target
- publish face bounding box and center information
- publish detector overlay image for debugging

Current practical status:

- currently more stable than the color path in validation
- suitable as the current preferred path for downstream control integration work

### `color`

This mode is intended for color-target tasks, with red as the current primary development case.

Current preprocessing characteristics:

- convert to HSV internally
- apply threshold segmentation
- support dual-range red segmentation
- apply simple morphology
- publish binary mask output as `mono8`

Current target detection characteristics:

- consume preprocessed image and mask
- select one current primary color target
- publish color target bounding box and center information
- publish detector overlay image for debugging

Current practical status:

- functional, but still less stable than face mode
- still requires threshold and robustness tuning to reduce false positives and improve consistency under lighting variation

## Current validation status

Industrial PC validation has already confirmed:

- package build success
- executable discovery success
- successful node startup for `image_preprocessor_node`
- successful node startup for `target_detector_node`
- successful publication of `/recognition/preprocessed/image`
- successful publication of `/recognition/preprocessed/debug_image`
- successful publication of `/recognition/preprocessed/mask` in color mode
- successful publication of `/recognition/detection/debug_image`
- successful publication of `/recognition/target`
- stable practical face-mode operation
- functional color-mode operation with remaining tuning work

## What is not implemented yet

The following are intentionally not implemented yet:

- target tracking logic
- target selection among multiple competing semantic targets
- detector / recognizer model integration beyond the current first-stage implementation
- final control-side consumption and closed-loop motion execution
- richer structured target output such as multi-target arrays
- recognition-side state estimation across frames

## Current known limitations

Current limitations include:

- `face` mode currently relies on a Haar Cascade based detector rather than a stronger modern detector
- `color` mode is still sensitive to lighting, hue variation, and scene-dependent false positives
- current output is a single current primary target rather than a multi-target result
- no dedicated target tracking stage exists yet
- no final control-layer integration exists yet

## Next likely step

The next likely step after the current milestone is:

- implement `control_pkg` so that it consumes `/recognition/target`

Reasonable first control-side direction:

- consume `Target.msg`
- use `center_x_norm` and `center_y_norm` as the first control error signals
- generate rotation / step decisions for the actuator side
- validate the first end-to-end closed-loop behavior using the more stable `face` mode first