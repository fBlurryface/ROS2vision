# control_pkg

## Purpose

`control_pkg` is the current control-side package for ROS2vision.

At the current stage, this package provides the first minimal control path between the structured recognition output and the Arduino UNO actuator side.

## Current validated node design

The current first-stage node split is:

- `target_follower_node`
- `uno_serial_bridge_node`

This split is intentional:

- `target_follower_node`
  - consumes structured target output
  - computes a small angle correction command
- `uno_serial_bridge_node`
  - converts the control command into UNO serial protocol messages
  - sends actuator-side commands such as `ANG <deg>` or `STEP <n>`

## Current intended system placement

Current intended chain:

- `/recognition/target`
  - published by `recognition_pkg/target_detector_node`
- `target_follower_node`
  - computes control correction
- `/control/angle_command`
  - current internal control topic
- `uno_serial_bridge_node`
  - sends serial commands to the UNO
- UNO firmware
  - drives the stepper actuator

## Current node responsibilities

### `target_follower_node`

Current responsibility:

- subscribe to `/recognition/target`
- smooth the horizontal target error
- apply deadband and hysteresis behavior
- apply command cooldown
- map target error to a small angle correction
- publish `/control/angle_command`

Current first-stage design intent:

- favor stability over aggressive correction
- keep mechanisms parameterized for future tuning
- preserve room for future online learning / search work

### `uno_serial_bridge_node`

Current responsibility:

- subscribe to `/control/angle_command`
- open and maintain the UNO serial connection
- send commands in either:
  - `ANG <deg>` mode
  - `STEP <n>` mode
- optionally use simple busy gating and UNO feedback lines such as:
  - `DONE ...`
  - `STOPPED ...`

Current default behavior:

- use `ANG <deg>` as the default command mode
- keep `STEP <n>` available as a lower-level fallback mode

## Current topics

Current input topics:

- `/recognition/target`

Current internal control topic:

- `/control/angle_command`

## Current parameters

The first version intentionally exposes many control parameters so that later tuning and learning-oriented work has room to expand.

Current parameter groups include:

- target acceptance thresholds
- smoothing
- deadband and hysteresis
- angle mapping strategy
- command pacing
- serial bridge behavior
- command mode selection

## Launch usage

Launch both current control nodes:

    ros2 launch control_pkg control_follow.launch.py

## Current configuration file

Current provided parameter file:

- `config/control_params.yaml`

## Current limitations

Current limitations at this stage:

- only horizontal control is implemented
- no final closed-loop validation is documented yet
- no dedicated search behavior exists when the target is lost
- no richer control debug topics exist yet
- no control-specific custom ROS interface exists yet

## Practical debug commands

Check package discovery:

    ros2 pkg list | grep control_pkg

Check executable discovery:

    ros2 pkg executables control_pkg

Run control follower directly:

    ros2 run control_pkg target_follower_node

Run UNO serial bridge directly:

    ros2 run control_pkg uno_serial_bridge_node

Inspect current control topic:

    ros2 topic echo /control/angle_command