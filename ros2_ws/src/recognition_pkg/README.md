# recognition_pkg

## Purpose

`recognition_pkg` is the current recognition-side package for ROS2vision.

At the current stage, the package has started from the preprocessing layer rather than from model inference.

Its role is to prepare camera output for downstream recognition tasks and later host the actual detector / recognizer node.

## Current package status

The first real node in this package is now available:

- `image_preprocessor_node`

This node is intended to sit directly after the camera source node and directly before the future inference node.

## Current node responsibility

The current preprocessing node is responsible for:

- subscribing to raw camera image input
- preparing task-oriented preprocessed output
- publishing preprocessed image topics for downstream recognition nodes
- supporting multiple preprocessing modes

It is not responsible for:

- target detection
- face recognition
- object classification
- target selection
- tracking logic
- control decision output

Those belong to later nodes.

## Current input topic

Current expected input:

- `/camera/image_raw`

## Current output topics

Current outputs:

- `/recognition/preprocessed/image`
- `/recognition/preprocessed/debug_image`

Additional output in color mode:

- `/recognition/preprocessed/mask`

## Current preprocessing modes

### `face`

Purpose:

- prepare a full-frame image for future face-related inference

Current behavior:

- resize to configured output size
- optional aspect-ratio preservation
- output encoding can be selected
- current default output is `rgb8`

### `color`

Purpose:

- prepare color-segmentation-oriented output for future color-target tasks

Current behavior:

- convert image to HSV internally
- apply configured threshold range
- apply simple morphology
- publish a binary mask as `mono8`

## Launch usage

Default launch:

    ros2 launch recognition_pkg image_preprocessor.launch.py

Explicit color-mode launch:

    ros2 launch recognition_pkg image_preprocessor.launch.py `
      params_file:=/absolute/path/to/recognition_params.color.yaml

## Current configuration files

Current provided parameter files:

- `config/recognition_params.face.yaml`
- `config/recognition_params.color.yaml`

## Current validated behavior

Validated so far on the industrial PC:

- package builds successfully as a ROS 2 Python package
- node executable is discovered correctly
- `face` mode publishes preprocessed image output
- `face` mode publishes debug image output
- `face` mode does not publish live mask data
- `color` mode publishes preprocessed image output
- `color` mode publishes debug image output
- `color` mode publishes live binary mask output
- practical observed output rates are around ~20 Hz in both validated modes

## Known limitations

Current limitations at this stage:

- no detector / recognizer node exists yet
- no trained model is integrated yet
- no structured target message is published yet
- mask topic publisher currently exists even when face mode is used, although no live mask data is published in that mode
- preprocessing is still limited to the initial `face` and `color` paths

## Practical debug commands

Check package discovery:

    ros2 pkg list | grep recognition_pkg

Check executable discovery:

    ros2 pkg executables recognition_pkg

Run node directly:

    ros2 run recognition_pkg image_preprocessor_node

Inspect published topics:

    ros2 topic list | grep recognition

Inspect output rate:

    ros2 topic hz /recognition/preprocessed/image

Inspect mask rate:

    ros2 topic hz /recognition/preprocessed/mask

Read one output message:

    ros2 topic echo /recognition/preprocessed/image --once