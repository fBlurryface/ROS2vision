# Project Progress

[README](../README.md) / Project Progress

This page owns the detailed implementation status, validation record, and roadmap. The README contains only a summary.

**Documentation baseline:** 2026-09-11, main commit [d5cb681](https://github.com/fBlurryface/ROS2vision/commit/d5cb6815d1a210e46e3fc3f8da70db12a9efea63). Dates below follow repository commit timestamps in UTC.

## Status vocabulary

- **Implemented:** the capability exists in the referenced repository version.
- **Reported validation:** earlier project documentation or calibration notes record an observation; the underlying setup/results may still need fuller records.
- **Planned:** future work, with no claim of an available implementation.
- **Candidate:** an option to evaluate; adoption and implementation are undecided.
- **TODO:** missing documentation or an unresolved decision.

Implementation, CI success, and physical system validation are different forms of evidence.

## Current implementation

| Area | Repository state | Remaining boundary |
| --- | --- | --- |
| Camera | Mode-based acquisition, persistent-device override, and process recovery | Camera calibration and exact host device rule are not checked in |
| Recognition | Face/color preprocessing and single-target detection | Hand perception and persistent target selection are planned |
| UNO path | Follower, USB serial bridge, stepper firmware, and system launch | Final closed-loop acceptance results are not recorded |
| Debugging | Optional detection-image display and recording | Quantitative latency and end-to-end measurements remain to be recorded |
| ESP32 arm | USB serial commands, position/velocity reference control, kinematics, calibration, and MCPWM output | No actuator feedback; ROS 2 host integration is planned |
| Automation | ROS workspace CI and UNO compilation CI | ESP32 build coverage and hardware validation are not included |

## Milestones

| Period | Change |
| --- | --- |
| 2026-03-18–19 | Established the UNO stepper protocol, ROS 2 package scaffold, and CI workflows |
| 2026-04-01 | Implemented camera modes, a persistent-device strategy, and reconnect handling |
| 2026-04-02–04 | Added preprocessing, face/color detection, `Target.msg`, and the first UNO control path |
| 2026-04-12 | Improved serial connection handling and timing; added system bringup and the debug viewer |
| 2026-06-09–11 | Added ESP-IDF arm firmware, moved motion generation into `arm_motion`, adopted MCPWM, and introduced incremental tool-target IK |
| 2026-09-09 | Replaced the older motion/IK APIs with position and continuous resolved-rate velocity control |

The September change is a breaking firmware API redesign. June-era `jtool`, incremental-delta APIs, and older duration/speed commands are historical interfaces. Use the current [command reference](interfaces.md#esp32-arm-serial-protocol).

Details remain available in [commit history](https://github.com/fBlurryface/ROS2vision/commits/main/).

## Validation record

| Evidence | Recorded result | Limits of the record |
| --- | --- | --- |
| Earlier camera notes, April 2026 | VGA/wide/HD startup and same-port reconnect; approximately 16.6 Hz observed | Camera model, full setup, and repeatable measurements still need to be recorded; cross-port recovery is unverified |
| Earlier recognition notes, April 2026 | Successful node startup and target/debug publication; face mode more stable than color | No quantitative accuracy or latency benchmark |
| Arm calibration comments | Joint pulse mappings, claw-gap measurements, and geometry defaults recorded in source | No complete assembly/calibration procedure or acceptance report |
| ROS CI at [7236c10](https://github.com/fBlurryface/ROS2vision/actions/runs/24317261408) | Workspace workflow completed successfully | Builds and invokes `colcon test`; no hardware execution or substantive behavior-test suite is present |
| Firmware CI at [d5cb681](https://github.com/fBlurryface/ROS2vision/actions/runs/34408765092) | UNO firmware compiled successfully | The workflow compiles UNO even when an ESP32 file triggered it; it does not compile the ESP32 project |

Historical source records: [camera notes](https://github.com/fBlurryface/ROS2vision/blob/f4e4959acfcaebf3531f40b25e2e2e321b1f4bdb/docs/software/camera.md), [recognition notes](https://github.com/fBlurryface/ROS2vision/blob/f4e4959acfcaebf3531f40b25e2e2e321b1f4bdb/docs/software/recognition.md).

Workflow definitions: [ROS workspace](../.github/workflows/ci.yml), [UNO firmware](../.github/workflows/firmware-ci.yml).

### Record the next validation run

> **TODO — Validation entry:** record date and commit, host/toolchain versions, hardware and wiring revision, configuration files, procedure, observed result, and a link to logs or video. For tracking, include loss/reacquisition, latency, and limit behavior.

## Next milestone: ROS 2 arm integration

Linux and ROS 2 remain the formal host direction. The next milestone is to connect perception to the ESP32 arm. USB serial command access is already available; the ROS 2 integration method is still to be selected.

| Work item | Proposed scope | Completion evidence |
| --- | --- | --- |
| Hand-target perception | Extend `recognition_pkg` with hand landmarks and a configurable target point; MediaPipe Hand Landmarker is a candidate backend | Target output and debug visualization run in the Linux ROS 2 workspace |
| Visual-error processing | Define desired image center, sign conventions, normalization, deadband, smoothing, and lost-target handling | Published error follows the agreed coordinate/validity contract |
| ESP32 ROS 2 integration | Connect the agreed visual-error and command interface to the firmware; implement stop, startup/reconnect handling, and diagnostics | Host and firmware state transitions are reproducible, including disconnect and command timeout |
| System integration | Add arm-oriented launch/configuration and debug entry points | A documented camera-to-arm run with recorded results |

These are planned changes. The current workspace provides neither hand recognition nor ROS 2 arm integration. The existing USB serial protocol is an available basis for a custom host bridge; micro-ROS is another integration option to evaluate.

### Decisions before implementation

- Choose the ROS message contract and node split.
- Select the MCU integration method and link; define control semantics independently of transport framing.
- Document camera placement and image-to-solver axes.
- Decide how target selection and reacquisition should behave.
- Define command ownership and operator stop/resume behavior.
- Decide how UNO and arm launch paths should be presented and maintained.

### Communication options to evaluate

- **Wi-Fi with a custom protocol:** evaluate a host bridge using TCP or UDP, including message framing, latency, command freshness, and reconnection.
- **Bluetooth:** verify the relevant board interface, Bluetooth protocol, and Linux host support before defining a development scope. A manufacturer's Bluetooth feature does not establish support in this project's firmware.
- **micro-ROS:** evaluate an ESP32 client and host-side Agent over serial or Wi-Fi/UDP. Check the Jazzy/ESP-IDF build, resource use, control-task timing, and connection behavior while assessing reuse of the existing motion and kinematics components.

These are candidates with no committed wireless route or micro-ROS migration. The current USB serial path remains available for initial integration. [Architecture](architecture.md#host-to-mcu-communication) explains the link and integration choices; [Interfaces](interfaces.md#interface-work-to-complete) owns the remaining message and command decisions.

## Follow-up work

- Add ESP32 build validation and targeted checks for motion/protocol behavior.
- Link the matching manufacturer hardware references; document project-specific calibration, camera mounting, and deployment reproduction.
- Measure camera throughput, recognition latency, and closed-loop behavior.
- Revisit deployment automation after the manual operating procedure is reproducible.

These items have no scheduled release dates. Earlier binary-transport and CD ideas remain candidates for later decisions, rather than current interface commitments.

## Maintaining this documentation

| Change | Update |
| --- | --- |
| Responsibility or dataflow changes | [Architecture](architecture.md) |
| A runnable command or environment requirement changes | [Bringup](bringup.md) |
| Wiring, geometry, or calibration changes | [Hardware](hardware.md) |
| A topic, field, command, unit, or timeout changes | [Interfaces](interfaces.md) |
| A milestone is completed or an experiment is recorded | This page, then the README summary if needed |

Keep TODOs specific: identify the missing decision or evidence and what will fill the section. Replace completed TODOs with the result and its source. Add a new page only when a topic has a distinct reader task and enough content to maintain independently.
