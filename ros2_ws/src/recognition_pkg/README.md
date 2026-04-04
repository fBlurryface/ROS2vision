# recognition_pkg

## Purpose

`recognition_pkg` is the current recognition-side package for ROS2vision.

At the current stage, the package no longer stops at preprocessing. It now contains a preprocessing node and a target detection node.

Its role is to prepare camera output for downstream recognition tasks, detect a current primary target, and publish structured target output for future control logic.

## Current package status

The current validated nodes in this package are:

- `image_preprocessor_node`
- `target_detector_node`

Current intended placement in the system:

- `image_preprocessor_node`
  - sits directly after the camera source node
- `target_detector_node`
  - sits directly after the preprocessing node
  - publishes the current primary target for future control use

## Current node responsibilities

### `image_preprocessor_node`

Current responsibility:

- subscribe to raw camera image input
- prepare task-oriented preprocessed output
- publish preprocessed image topics for downstream recognition nodes
- support multiple preprocessing modes

It is not responsible for:

- final target output for control
- target tracking
- control decision output

### `target_detector_node`

Current responsibility:

- consume preprocessed recognition outputs
- detect the current primary target based on mode
- publish structured target information
- publish a detector-side debug image with target overlay

It is not responsible for:

- target tracking across frames
- target prioritization among multiple semantic targets
- final motor command generation

Those belong to later stages.

## Current input topics

Current expected detector-side inputs:

- `/recognition/preprocessed/image`

Additional detector input in color mode:

- `/recognition/preprocessed/mask`

Current package root input from camera layer:

- `/camera/image_raw`

## Current output topics

Current package outputs:

- `/recognition/preprocessed/image`
- `/recognition/preprocessed/debug_image`
- `/recognition/preprocessed/mask`
- `/recognition/detection/debug_image`
- `/recognition/target`

## Current supported modes

### `face`

Purpose:

- prepare and detect a face target for future closed-loop target following

Current behavior:

- preprocess a full-frame image
- detect a current primary face
- publish target center and bounding box
- publish a detector overlay debug image

Current practical status:

- currently the more stable validated path

### `color`

Purpose:

- prepare and detect a color target, with red as the current primary development case

Current behavior:

- preprocess color-target-oriented image data
- publish a segmentation mask
- detect a current primary color target
- publish target center and bounding box
- publish a detector overlay debug image

Current practical status:

- functional, but still requires threshold and robustness tuning

## Current structured target output

Current target output is published through:

- topic: `/recognition/target`
- message type: `ros2vision_interfaces/msg/Target`

Current message intent includes:

- detection state
- mode
- label
- image size
- target center
- normalized target center
- bounding box
- area
- score

This is the current recognition-to-control contract.

## Launch usage

Preprocessing node launch:

    ros2 launch recognition_pkg image_preprocessor.launch.py

Target detector node launch:

    ros2 launch recognition_pkg target_detector.launch.py

Explicit color-mode detector launch:

    ros2 launch recognition_pkg target_detector.launch.py `
      params_file:=/absolute/path/to/target_detector.color.yaml

Explicit face-mode detector launch:

    ros2 launch recognition_pkg target_detector.launch.py `
      params_file:=/absolute/path/to/target_detector.face.yaml

## Current configuration files

Current provided parameter files:

- `config/recognition_params.face.yaml`
- `config/recognition_params.color.yaml`
- `config/target_detector.face.yaml`
- `config/target_detector.color.yaml`

## Current validated behavior

Validated so far on the industrial PC:

- package builds successfully as a ROS 2 Python package
- both node executables are discovered correctly
- `image_preprocessor_node` works in `face` mode
- `image_preprocessor_node` works in `color` mode
- `target_detector_node` works in `face` mode
- `target_detector_node` works in `color` mode
- `/recognition/target` is published successfully
- `/recognition/detection/debug_image` is published successfully
- face mode currently behaves more stably than color mode

## Known limitations

Current limitations at this stage:

- `face` mode currently uses a Haar Cascade based detector
- `color` mode still requires further tuning against false positives and scene variation
- only one current primary target is published
- no multi-target output exists yet
- no target tracking exists yet
- no control integration exists yet

## Practical debug commands

Check package discovery:

    ros2 pkg list | grep recognition_pkg

Check executable discovery:

    ros2 pkg executables recognition_pkg

Run preprocessing node directly:

    ros2 run recognition_pkg image_preprocessor_node

Run target detector node directly:

    ros2 run recognition_pkg target_detector_node

Inspect published topics:

    ros2 topic list | grep recognition

Inspect target output:

    ros2 topic echo /recognition/target

Inspect detector debug image rate:

    ros2 topic hz /recognition/detection/debug_image

Read one target message:

    ros2 topic echo /recognition/target --once