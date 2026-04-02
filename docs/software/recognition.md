# Recognition subsystem notes

## Purpose

This document records the current recognition-side design decisions for ROS2vision.

At the current stage, the recognition subsystem has only started from the preprocessing layer. The actual detector / recognizer node is not yet implemented.

## Current package

- Package: `ros2_ws/src/recognition_pkg`

This package is now the second ROS 2 package in the repository to move beyond scaffold state.

Its first validated node is:

- `image_preprocessor_node`

## Current subsystem responsibility

The recognition subsystem is intended to sit between the camera source layer and the control layer.

Current design direction:

- `camera_vision_pkg`
  - provide stable image source
- `recognition_pkg`
  - prepare recognition-oriented image data
  - later perform detector / recognizer inference
- `control_pkg`
  - consume target information and generate motion decisions

## Current implemented node

### `image_preprocessor_node`

Current role:

- subscribe to raw image input from camera layer
- prepare task-oriented preprocessed image output
- publish preprocessed output for a future inference node
- support mode switching based on downstream task type

This node is intentionally not the detector itself.

## Current input / output contract

Current expected input:

- `/camera/image_raw`

Current outputs:

- `/recognition/preprocessed/image`
- `/recognition/preprocessed/debug_image`

Additional output in color mode:

- `/recognition/preprocessed/mask`

## Current supported modes

### `face`

This mode is intended for future face-related tasks.

Current characteristics:

- resize to configured output size
- optional aspect-ratio preservation
- current default output encoding is `rgb8`
- optional grayscale/equalization-related options exist in parameters

Current design intent:

- prepare the whole frame for future face detector / recognizer input
- do not perform face detection here

### `color`

This mode is intended for future color-target tasks.

Current characteristics:

- convert to HSV internally
- apply threshold segmentation
- apply simple morphology
- publish binary mask output as `mono8`

Current design intent:

- provide a simple and explicit preprocessing path for color-target work
- allow the next node to consume either processed image or mask

## Current validation status

Industrial PC validation has already confirmed:

- package build success
- executable discovery success
- successful node startup in `face` mode
- successful node startup in `color` mode
- stable publication of `/recognition/preprocessed/image`
- stable publication of `/recognition/preprocessed/debug_image`
- stable publication of `/recognition/preprocessed/mask` in `color` mode
- practical output rates around ~20 Hz in both validated modes

## What is not implemented yet

The following are intentionally not implemented yet:

- detector / recognizer inference node
- integration of external pretrained models
- integration of self-trained lightweight models
- structured target output for control layer
- target selection logic
- target tracking logic

## Current known limitations

Current limitations include:

- preprocessing exists, but semantic recognition output does not yet exist
- no final message contract for target information has been implemented yet
- color thresholds are still static and manually configured
- current face-mode output is still generic preprocessing rather than task-optimized face inference input tuning
- mask topic publisher currently exists even when the node is running in face mode, although no live mask data is produced in that mode

## Next likely step

The next likely step after the current milestone is:

- add the first real inference node after preprocessing

Reasonable initial implementation direction:

- consume `/recognition/preprocessed/image` or `/recognition/preprocessed/mask`
- use either a library-based detector or a simple non-model color-target extractor first
- publish target center and related data needed by `control_pkg`