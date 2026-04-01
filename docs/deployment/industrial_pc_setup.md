# Industrial PC setup notes

## Purpose

This document records the current industrial PC environment arrangement used for manual pull, build, and hardware validation.

The goal is to keep the target machine ready for future CD integration without enabling automated deployment yet.

## Current deployment layout

The industrial PC currently uses a dedicated deployment root:

- `~/deployments/ros2vision/`

Key paths:

- `~/deployments/ros2vision/repo`
  - sparse checkout repository copy
- `~/deployments/ros2vision/current`
  - active ROS 2 workspace entrypoint
- `~/deployments/ros2vision/scripts`
  - reserved for future deployment scripts
- `~/deployments/ros2vision/logs`
  - reserved for deployment/runtime notes
- `~/deployments/ros2vision/releases`
  - reserved for future release-oriented deployment
- `~/deployments/ros2vision/env`
  - reserved for environment-specific files

## Repository fetch strategy

The industrial PC does not use the old generic workspace as the main development target.

Instead, it pulls the project repository into a dedicated deployment location.

The repository is currently fetched by:

- SSH authentication to GitHub
- sparse checkout
- project-specific working path

This keeps the target machine prepared for future controlled deployment.

## ROS environment status

The machine already has ROS 2 Jazzy available and can successfully:

- source `/opt/ros/jazzy/setup.bash`
- run `rosdep update`
- build the workspace using `colcon`

## Camera device strategy on industrial PC

For camera stability, the deployment no longer relies on drifting `/dev/videoN`.

Current validated alias:

- `/dev/ros2vision_camera`

The alias is produced through a custom udev rule based on the detected USB camera identity.

## Camera package validation status on industrial PC

The industrial PC has already validated:

- repository pull
- package build
- camera bringup
- raw stream publication
- same-port USB reconnect recovery

Current observed frame rate across tested startup modes is roughly ~16.6 Hz.

## What is not enabled yet

The following are intentionally not enabled yet:

- automatic CD from GitHub Actions
- self-hosted GitHub Actions runner on the industrial PC
- unattended deployment writes or service restarts
- production-style deployment promotion flow

## Manual workflow currently used

Typical current workflow:

1. pull latest repository changes
2. build the selected workspace/package
3. source the install environment
4. manually run launch files
5. perform hardware validation on the industrial PC

## Future CD direction

The current directory layout was chosen so that CD can later be added without redesigning the machine layout.

Likely future direction:

- GitHub-hosted workflow
- controlled SSH-based deployment trigger
- optional environment gating
- reuse of the same `~/deployments/ros2vision/current` entrypoint pattern

## Current limitation

The currently validated reconnect path is same-port reconnect. Moving the camera to a different physical USB port is not yet guaranteed to restore acquisition automatically.
