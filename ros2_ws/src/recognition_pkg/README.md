# recognition_pkg

Image preprocessing, single-target detection, and debug visualization for ROS2vision.

Current nodes are `image_preprocessor_node`, `target_detector_node`, and `tracking_debug_viewer_node`. Recognition supports face and color modes. The viewer displays/records images; persistent target tracking and hand perception are future work.

- Launch entries: [launch/](launch/)
- Mode and viewer profiles: [config/](config/)
- [Build and run](../../../docs/bringup.md#ros-2-host)
- [Module responsibilities](../../../docs/architecture.md#ros-2-host)
- [Target message semantics](../../../docs/interfaces.md#target-message)
- [Validation and planned work](../../../docs/progress.md)
