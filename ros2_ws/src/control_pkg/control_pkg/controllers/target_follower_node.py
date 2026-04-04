from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from ros2vision_interfaces.msg import Target
from std_msgs.msg import Float32


class TargetFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__("target_follower_node")

        self.declare_parameter("target_topic", "/recognition/target")
        self.declare_parameter("angle_command_topic", "/control/angle_command")

        self.declare_parameter("min_target_score", 0.0)
        self.declare_parameter("min_target_area", 0.0)

        self.declare_parameter("smoothing_alpha", 0.35)
        self.declare_parameter("deadband_x", 0.08)
        self.declare_parameter("resume_threshold_x", 0.12)
        self.declare_parameter("reset_filter_on_lost", True)
        self.declare_parameter("target_timeout_sec", 0.5)
        self.declare_parameter("command_cooldown_sec", 0.25)

        self.declare_parameter("mapping_mode", "stepwise")
        self.declare_parameter("angle_gain", 10.0)

        self.declare_parameter("small_error_threshold", 0.15)
        self.declare_parameter("medium_error_threshold", 0.30)
        self.declare_parameter("large_error_threshold", 0.50)

        self.declare_parameter("min_angle_deg", 1.0)
        self.declare_parameter("medium_angle_deg", 2.0)
        self.declare_parameter("large_angle_deg", 4.0)
        self.declare_parameter("max_angle_deg", 6.0)

        self.declare_parameter("invert_direction", False)
        self.declare_parameter("publish_debug_logs", True)

        self._load_params()

        self._filtered_error_x: Optional[float] = None
        self._centered_latched = False
        self._last_target_rx_time = None
        self._last_command_time = None

        self._angle_publisher = self.create_publisher(Float32, self.angle_command_topic, 10)
        self._target_subscription = self.create_subscription(
            Target,
            self.target_topic,
            self._target_callback,
            10,
        )
        self._watchdog_timer = self.create_timer(0.05, self._watchdog_callback)

        self.get_logger().info(
            f"Started target_follower_node | target_topic={self.target_topic} | "
            f"angle_command_topic={self.angle_command_topic}"
        )

    def _load_params(self) -> None:
        self.target_topic = str(self.get_parameter("target_topic").value)
        self.angle_command_topic = str(self.get_parameter("angle_command_topic").value)

        self.min_target_score = float(self.get_parameter("min_target_score").value)
        self.min_target_area = float(self.get_parameter("min_target_area").value)

        self.smoothing_alpha = float(self.get_parameter("smoothing_alpha").value)
        self.deadband_x = float(self.get_parameter("deadband_x").value)
        self.resume_threshold_x = float(self.get_parameter("resume_threshold_x").value)
        self.reset_filter_on_lost = bool(self.get_parameter("reset_filter_on_lost").value)
        self.target_timeout_sec = float(self.get_parameter("target_timeout_sec").value)
        self.command_cooldown_sec = float(self.get_parameter("command_cooldown_sec").value)

        self.mapping_mode = str(self.get_parameter("mapping_mode").value).strip().lower()
        self.angle_gain = float(self.get_parameter("angle_gain").value)

        self.small_error_threshold = float(self.get_parameter("small_error_threshold").value)
        self.medium_error_threshold = float(self.get_parameter("medium_error_threshold").value)
        self.large_error_threshold = float(self.get_parameter("large_error_threshold").value)

        self.min_angle_deg = float(self.get_parameter("min_angle_deg").value)
        self.medium_angle_deg = float(self.get_parameter("medium_angle_deg").value)
        self.large_angle_deg = float(self.get_parameter("large_angle_deg").value)
        self.max_angle_deg = float(self.get_parameter("max_angle_deg").value)

        self.invert_direction = bool(self.get_parameter("invert_direction").value)
        self.publish_debug_logs = bool(self.get_parameter("publish_debug_logs").value)

    def _target_callback(self, msg: Target) -> None:
        self._last_target_rx_time = self.get_clock().now()

        if not msg.detected:
            self._handle_target_lost("target message says detected=false")
            return

        if float(msg.score) < self.min_target_score:
            self._handle_target_lost("target score below threshold")
            return

        if float(msg.area) < self.min_target_area:
            self._handle_target_lost("target area below threshold")
            return

        raw_error_x = float(msg.center_x_norm)
        filtered_error_x = self._apply_smoothing(raw_error_x)
        abs_error_x = abs(filtered_error_x)

        if self._centered_latched:
            if abs_error_x <= self.resume_threshold_x:
                return
            self._centered_latched = False

        if abs_error_x <= self.deadband_x:
            self._centered_latched = True
            return

        if self._in_cooldown():
            return

        angle_delta_deg = self._map_error_to_angle(abs_error_x)
        if filtered_error_x < 0.0:
            angle_delta_deg = -angle_delta_deg

        if self.invert_direction:
            angle_delta_deg = -angle_delta_deg

        self._publish_angle_command(angle_delta_deg)

        if self.publish_debug_logs:
            self.get_logger().info(
                "target_follower | "
                f"raw_x={raw_error_x:.3f} filtered_x={filtered_error_x:.3f} "
                f"angle_delta_deg={angle_delta_deg:.3f}"
            )

    def _apply_smoothing(self, raw_error_x: float) -> float:
        alpha = min(max(self.smoothing_alpha, 0.0), 1.0)
        if self._filtered_error_x is None:
            self._filtered_error_x = raw_error_x
        else:
            self._filtered_error_x = ((1.0 - alpha) * self._filtered_error_x) + (alpha * raw_error_x)
        return self._filtered_error_x

    def _map_error_to_angle(self, abs_error_x: float) -> float:
        if self.mapping_mode == "proportional":
            angle = abs_error_x * self.angle_gain
            return max(self.min_angle_deg, min(self.max_angle_deg, angle))

        if abs_error_x >= self.large_error_threshold:
            return self.max_angle_deg
        if abs_error_x >= self.medium_error_threshold:
            return self.large_angle_deg
        if abs_error_x >= self.small_error_threshold:
            return self.medium_angle_deg
        return self.min_angle_deg

    def _publish_angle_command(self, angle_delta_deg: float) -> None:
        msg = Float32()
        msg.data = float(angle_delta_deg)
        self._angle_publisher.publish(msg)
        self._last_command_time = self.get_clock().now()

    def _in_cooldown(self) -> bool:
        if self._last_command_time is None:
            return False
        elapsed = self.get_clock().now() - self._last_command_time
        return elapsed < Duration(seconds=self.command_cooldown_sec)

    def _handle_target_lost(self, reason: str) -> None:
        self._centered_latched = False
        if self.reset_filter_on_lost:
            self._filtered_error_x = None
        if self.publish_debug_logs:
            self.get_logger().debug(f"target_follower lost target: {reason}")

    def _watchdog_callback(self) -> None:
        if self._last_target_rx_time is None:
            return
        elapsed = self.get_clock().now() - self._last_target_rx_time
        if elapsed > Duration(seconds=self.target_timeout_sec):
            self._handle_target_lost("target timeout")
            self._last_target_rx_time = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()