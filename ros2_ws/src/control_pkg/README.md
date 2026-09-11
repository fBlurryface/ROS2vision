# control_pkg

Host-side target following and actuator transport for ROS2vision.

The current implementation controls UNO: `target_follower_node` publishes horizontal angle corrections, and `uno_serial_bridge_node` transports them over USB serial. ROS 2 integration with the ESP32 arm is planned; its integration method remains open.

- Launch entry: [control_follow.launch.py](launch/control_follow.launch.py)
- Configured launch behavior: [control_params.yaml](config/control_params.yaml)
- [Build and run](../../../docs/bringup.md#ros-2-host)
- [Follower and serial contracts](../../../docs/interfaces.md#uno-follower-profile)
- [Planned arm integration](../../../docs/progress.md#next-milestone-ros-2-arm-integration)
