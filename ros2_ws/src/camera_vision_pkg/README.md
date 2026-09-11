# camera_vision_pkg

Camera acquisition and process recovery for ROS2vision.

The package starts `usb_cam` through a device-readiness loop and publishes raw images and camera information. Image processing belongs to `recognition_pkg`.

- Launch entry: [camera_source.launch.py](launch/camera_source.launch.py)
- Acquisition profiles: [config/](config/)
- [Build and run](../../../docs/bringup.md#ros-2-host)
- [Camera hardware and device setup](../../../docs/hardware.md#camera-setup)
- [ROS topic reference](../../../docs/interfaces.md#ros-topics)
- [Validation and limitations](../../../docs/progress.md#validation-record)
