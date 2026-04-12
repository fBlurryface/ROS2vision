from __future__ import annotations

import datetime as dt
import os
from pathlib import Path
from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


class TrackingDebugViewerNode(Node):
    def __init__(self) -> None:
        super().__init__("tracking_debug_viewer_node")
        self._bridge = CvBridge()

        self.declare_parameter("input_topic", "/recognition/detection/debug_image")
        self.declare_parameter("window_name", "ROS2vision Tracking Debug")
        self.declare_parameter("window_width", 1280)
        self.declare_parameter("window_height", 720)
        self.declare_parameter("show_window", True)
        self.declare_parameter("start_fullscreen", False)
        self.declare_parameter("display_fps_overlay", True)

        self.declare_parameter("record_enabled", False)
        self.declare_parameter("recording_dir", "~/ros2vision_debug_recordings")
        self.declare_parameter("recording_prefix", "tracking_debug")
        self.declare_parameter("recording_fps", 20.0)
        self.declare_parameter("recording_codec", "MJPG")

        self._load_params()

        self._writer: Optional[cv2.VideoWriter] = None
        self._recording_path: Optional[Path] = None
        self._window_initialized = False
        self._last_frame_time_ns: Optional[int] = None
        self._display_fps: Optional[float] = None

        image_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self._subscription = self.create_subscription(
            Image,
            self.input_topic,
            self._image_callback,
            image_qos,
        )

        self.get_logger().info(
            "Started tracking_debug_viewer_node | "
            f"input_topic={self.input_topic} | "
            f"show_window={self.show_window} | "
            f"record_enabled={self.record_enabled}"
        )
        if self.record_enabled:
            self.get_logger().info(
                f"Recordings will be stored under {self.recording_dir}"
            )

    def _load_params(self) -> None:
        self.input_topic = str(self.get_parameter("input_topic").value)
        self.window_name = str(self.get_parameter("window_name").value)
        self.window_width = max(320, int(self.get_parameter("window_width").value))
        self.window_height = max(240, int(self.get_parameter("window_height").value))
        self.show_window = bool(self.get_parameter("show_window").value)
        self.start_fullscreen = bool(self.get_parameter("start_fullscreen").value)
        self.display_fps_overlay = bool(self.get_parameter("display_fps_overlay").value)

        self.record_enabled = bool(self.get_parameter("record_enabled").value)
        self.recording_dir = Path(
            os.path.expanduser(str(self.get_parameter("recording_dir").value))
        ).resolve()
        self.recording_prefix = (
            str(self.get_parameter("recording_prefix").value).strip()
            or "tracking_debug"
        )
        self.recording_fps = max(1.0, float(self.get_parameter("recording_fps").value))
        self.recording_codec = (
            str(self.get_parameter("recording_codec").value).strip()
            or "MJPG"
        )

    def _image_callback(self, msg: Image) -> None:
        try:
            frame_bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"Failed to convert debug image: {exc}")
            return

        frame_to_show = frame_bgr.copy()
        self._update_display_fps(msg)
        self._draw_overlay(frame_to_show)

        if self.record_enabled:
            self._write_recording_frame(frame_to_show)

        if self.show_window:
            self._show_frame(frame_to_show)

    def _update_display_fps(self, msg: Image) -> None:
        stamp_ns = (int(msg.header.stamp.sec) * 1_000_000_000) + int(msg.header.stamp.nanosec)
        if self._last_frame_time_ns is None:
            self._last_frame_time_ns = stamp_ns
            return

        delta_ns = stamp_ns - self._last_frame_time_ns
        self._last_frame_time_ns = stamp_ns
        if delta_ns <= 0:
            return

        fps = 1_000_000_000.0 / float(delta_ns)
        if self._display_fps is None:
            self._display_fps = fps
        else:
            self._display_fps = (0.8 * self._display_fps) + (0.2 * fps)

    def _draw_overlay(self, frame_bgr) -> None:
        h, w = frame_bgr.shape[:2]
        bottom_y = max(28, h - 18)

        if self.display_fps_overlay and self._display_fps is not None:
            fps_text = f"view_fps={self._display_fps:.1f}"
            cv2.putText(
                frame_bgr,
                fps_text,
                (10, bottom_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        if self.record_enabled:
            rec_text = "REC"
            if self._recording_path is not None:
                rec_text += f" {self._recording_path.name}"
            cv2.putText(
                frame_bgr,
                rec_text,
                (max(10, w - 420), 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.circle(frame_bgr, (w - 18, 22), 7, (0, 0, 255), -1)

    def _show_frame(self, frame_bgr) -> None:
        if not self._window_initialized:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, self.window_width, self.window_height)
            if self.start_fullscreen:
                cv2.setWindowProperty(
                    self.window_name,
                    cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN,
                )
            self._window_initialized = True

        cv2.imshow(self.window_name, frame_bgr)
        cv2.waitKey(1)

    def _write_recording_frame(self, frame_bgr) -> None:
        if self._writer is None:
            self._open_writer(frame_bgr.shape[1], frame_bgr.shape[0])
            if self._writer is None:
                return

        self._writer.write(frame_bgr)

    def _open_writer(self, frame_width: int, frame_height: int) -> None:
        try:
            self.recording_dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            self.get_logger().error(f"Failed to create recording directory {self.recording_dir}: {exc}")
            return

        timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = self.recording_dir / f"{self.recording_prefix}_{timestamp}.avi"
        fourcc = cv2.VideoWriter_fourcc(*self.recording_codec[:4].ljust(4))
        writer = cv2.VideoWriter(
            str(output_path),
            fourcc,
            self.recording_fps,
            (frame_width, frame_height),
        )
        if not writer.isOpened():
            self.get_logger().error(f"Failed to open video writer for {output_path}")
            writer.release()
            return

        self._writer = writer
        self._recording_path = output_path
        self.get_logger().info(f"Recording debug video to {output_path}")

    def destroy_node(self):
        try:
            if self._writer is not None:
                self._writer.release()
                self._writer = None
            if self._window_initialized:
                cv2.destroyWindow(self.window_name)
        finally:
            super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackingDebugViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()