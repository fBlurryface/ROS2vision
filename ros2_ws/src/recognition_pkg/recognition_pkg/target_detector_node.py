from __future__ import annotations

from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image

from ros2vision_interfaces.msg import Target
from recognition_pkg.detectors.base_detector import DetectionResult
from recognition_pkg.detectors.color_detector import ColorDetector
from recognition_pkg.detectors.face_detector import FaceDetector


class TargetDetectorNode(Node):
    VALID_MODES = {"color", "face"}

    def __init__(self) -> None:
        super().__init__("target_detector_node")
        self._bridge = CvBridge()

        self.declare_parameter("mode", "color")

        self.declare_parameter("input_image_topic", "/recognition/preprocessed/image")
        self.declare_parameter("input_mask_topic", "/recognition/preprocessed/mask")

        self.declare_parameter("target_topic", "/recognition/target")
        self.declare_parameter("debug_image_topic", "/recognition/detection/debug_image")
        self.declare_parameter("publish_debug_image", True)

        self.declare_parameter("mask_max_age_sec", 0.3)

        self.declare_parameter("color_use_mask_topic", True)
        self.declare_parameter("color_fallback_to_hsv", True)
        self.declare_parameter("color_min_area", 300.0)
        self.declare_parameter("color_lower_h_1", 0)
        self.declare_parameter("color_upper_h_1", 10)
        self.declare_parameter("color_lower_h_2", 170)
        self.declare_parameter("color_upper_h_2", 180)
        self.declare_parameter("color_lower_s", 120)
        self.declare_parameter("color_lower_v", 70)
        self.declare_parameter("color_upper_s", 255)
        self.declare_parameter("color_upper_v", 255)

        self.declare_parameter("face_cascade_path", "")
        self.declare_parameter("face_min_size", 40)
        self.declare_parameter("face_scale_factor", 1.1)
        self.declare_parameter("face_min_neighbors", 5)

        self._load_params()
        self._build_detector()

        self._latest_mask: Optional[np.ndarray] = None
        self._latest_mask_stamp_ns: Optional[int] = None

        self._target_publisher = self.create_publisher(Target, self.target_topic, 10)
        self._debug_publisher = self.create_publisher(Image, self.debug_image_topic, 10)

        self._image_subscription = self.create_subscription(
            Image,
            self.input_image_topic,
            self._image_callback,
            10,
        )

        self._mask_subscription = self.create_subscription(
            Image,
            self.input_mask_topic,
            self._mask_callback,
            10,
        )

        self.get_logger().info(
            f"Started target_detector_node | mode={self.mode} | "
            f"image={self.input_image_topic} | target={self.target_topic}"
        )

    def _load_params(self) -> None:
        self.mode = str(self.get_parameter("mode").value).strip().lower()
        if self.mode not in self.VALID_MODES:
            self.get_logger().warn(f"Invalid mode '{self.mode}', falling back to 'color'.")
            self.mode = "color"

        self.input_image_topic = str(self.get_parameter("input_image_topic").value)
        self.input_mask_topic = str(self.get_parameter("input_mask_topic").value)

        self.target_topic = str(self.get_parameter("target_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)

        self.mask_max_age_sec = float(self.get_parameter("mask_max_age_sec").value)

        self.color_use_mask_topic = bool(self.get_parameter("color_use_mask_topic").value)
        self.color_fallback_to_hsv = bool(self.get_parameter("color_fallback_to_hsv").value)
        self.color_min_area = float(self.get_parameter("color_min_area").value)
        self.color_lower_h_1 = int(self.get_parameter("color_lower_h_1").value)
        self.color_upper_h_1 = int(self.get_parameter("color_upper_h_1").value)
        self.color_lower_h_2 = int(self.get_parameter("color_lower_h_2").value)
        self.color_upper_h_2 = int(self.get_parameter("color_upper_h_2").value)
        self.color_lower_s = int(self.get_parameter("color_lower_s").value)
        self.color_lower_v = int(self.get_parameter("color_lower_v").value)
        self.color_upper_s = int(self.get_parameter("color_upper_s").value)
        self.color_upper_v = int(self.get_parameter("color_upper_v").value)

        self.face_cascade_path = str(self.get_parameter("face_cascade_path").value)
        self.face_min_size = int(self.get_parameter("face_min_size").value)
        self.face_scale_factor = float(self.get_parameter("face_scale_factor").value)
        self.face_min_neighbors = int(self.get_parameter("face_min_neighbors").value)

    def _build_detector(self) -> None:
        if self.mode == "color":
            self._detector = ColorDetector(
                use_mask_input=self.color_use_mask_topic,
                fallback_to_hsv=self.color_fallback_to_hsv,
                min_area=self.color_min_area,
                lower_h_1=self.color_lower_h_1,
                upper_h_1=self.color_upper_h_1,
                lower_h_2=self.color_lower_h_2,
                upper_h_2=self.color_upper_h_2,
                lower_s=self.color_lower_s,
                lower_v=self.color_lower_v,
                upper_s=self.color_upper_s,
                upper_v=self.color_upper_v,
            )
            return

        self._detector = FaceDetector(
            cascade_path=self.face_cascade_path,
            min_size=self.face_min_size,
            scale_factor=self.face_scale_factor,
            min_neighbors=self.face_min_neighbors,
        )

    def _mask_callback(self, msg: Image) -> None:
        try:
            mask = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except CvBridgeError as exc:
            self.get_logger().error(f"Failed to convert mask image: {exc}")
            return

        self._latest_mask = mask
        self._latest_mask_stamp_ns = self._stamp_to_ns(msg.header.stamp)

    def _image_callback(self, msg: Image) -> None:
        try:
            frame_bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"Failed to convert input image: {exc}")
            return

        mask = self._get_valid_mask_for(msg, frame_bgr.shape[:2])
        result = self._detector.detect(frame_bgr, mask=mask)

        self._publish_target(msg, frame_bgr, result)

        if self.publish_debug_image:
            self._publish_debug_image(msg, frame_bgr, result)

    def _get_valid_mask_for(self, image_msg: Image, image_shape: tuple[int, int]) -> Optional[np.ndarray]:
        if self.mode != "color" or not self.color_use_mask_topic:
            return None

        if self._latest_mask is None or self._latest_mask_stamp_ns is None:
            return None

        if self._latest_mask.shape[:2] != image_shape:
            return None

        image_stamp_ns = self._stamp_to_ns(image_msg.header.stamp)
        age_sec = abs(image_stamp_ns - self._latest_mask_stamp_ns) / 1e9
        if age_sec > self.mask_max_age_sec:
            return None

        return self._latest_mask

    def _publish_target(self, source_msg: Image, frame_bgr: np.ndarray, result: DetectionResult) -> None:
        target_msg = Target()
        target_msg.header = source_msg.header
        target_msg.mode = result.mode
        target_msg.label = result.label
        target_msg.detected = bool(result.detected)

        image_height, image_width = frame_bgr.shape[:2]
        target_msg.image_width = int(image_width)
        target_msg.image_height = int(image_height)

        if result.detected:
            target_msg.center_x = int(result.center_x)
            target_msg.center_y = int(result.center_y)
            target_msg.center_x_norm = float(self._normalize_coordinate(result.center_x, image_width))
            target_msg.center_y_norm = float(self._normalize_coordinate(result.center_y, image_height))

            target_msg.bbox_x = int(result.bbox_x)
            target_msg.bbox_y = int(result.bbox_y)
            target_msg.bbox_w = int(result.bbox_w)
            target_msg.bbox_h = int(result.bbox_h)

            target_msg.area = float(result.area)
            target_msg.score = float(result.score)
        else:
            target_msg.center_x = 0
            target_msg.center_y = 0
            target_msg.center_x_norm = 0.0
            target_msg.center_y_norm = 0.0
            target_msg.bbox_x = 0
            target_msg.bbox_y = 0
            target_msg.bbox_w = 0
            target_msg.bbox_h = 0
            target_msg.area = 0.0
            target_msg.score = 0.0

        self._target_publisher.publish(target_msg)

    def _publish_debug_image(self, source_msg: Image, frame_bgr: np.ndarray, result: DetectionResult) -> None:
        debug_bgr = frame_bgr.copy()

        status_text = f"mode={self.mode} status={'DETECTED' if result.detected else 'LOST'}"
        cv2.putText(
            debug_bgr,
            status_text,
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

        if result.detected:
            x = int(result.bbox_x)
            y = int(result.bbox_y)
            w = int(result.bbox_w)
            h = int(result.bbox_h)
            cx = int(result.center_x)
            cy = int(result.center_y)

            cv2.rectangle(debug_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(debug_bgr, (cx, cy), 4, (0, 0, 255), -1)

            info_text = (
                f"{result.label} "
                f"cx={cx} cy={cy} "
                f"score={result.score:.2f} area={result.area:.1f}"
            )
            cv2.putText(
                debug_bgr,
                info_text,
                (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        try:
            debug_msg = self._bridge.cv2_to_imgmsg(debug_bgr, encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"Failed to convert debug image: {exc}")
            return

        debug_msg.header = source_msg.header
        self._debug_publisher.publish(debug_msg)

    @staticmethod
    def _normalize_coordinate(value: int, size: int) -> float:
        if size <= 1:
            return 0.0
        return ((float(value) / float(size - 1)) * 2.0) - 1.0

    @staticmethod
    def _stamp_to_ns(stamp) -> int:
        return Time.from_msg(stamp).nanoseconds


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()