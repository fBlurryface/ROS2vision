#!/usr/bin/env python3

from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImagePreprocessorNode(Node):
    """
    Preprocess camera images for downstream recognition tasks.

    Supported modes:
      - face: prepare full-frame image for face-related inference
      - color: prepare color-segmented mask/image for color-object tasks

    Inputs:
      - /camera/image_raw (configurable)

    Outputs:
      - /recognition/preprocessed/image (configurable)
      - /recognition/preprocessed/debug_image (optional)
      - /recognition/preprocessed/mask (color mode only, optional)
    """

    VALID_MODES = {"face", "color"}

    def __init__(self) -> None:
        super().__init__("image_preprocessor_node")
        self._bridge = CvBridge()

        self.declare_parameter("input_topic", "/camera/image_raw")
        self.declare_parameter("output_topic", "/recognition/preprocessed/image")
        self.declare_parameter("debug_topic", "/recognition/preprocessed/debug_image")
        self.declare_parameter("mask_topic", "/recognition/preprocessed/mask")

        self.declare_parameter("mode", "face")

        self.declare_parameter("output_width", 320)
        self.declare_parameter("output_height", 320)
        self.declare_parameter("keep_aspect_ratio", True)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("publish_mask", True)

        self.declare_parameter("face_output_encoding", "rgb8")
        self.declare_parameter("face_equalize_hist", False)
        self.declare_parameter("face_gaussian_blur", False)
        self.declare_parameter("face_blur_kernel", 3)

        self.declare_parameter("color_space", "hsv")
        self.declare_parameter("color_lower_h", 0)
        self.declare_parameter("color_lower_s", 120)
        self.declare_parameter("color_lower_v", 70)
        self.declare_parameter("color_upper_h", 10)
        self.declare_parameter("color_upper_s", 255)
        self.declare_parameter("color_upper_v", 255)
        self.declare_parameter("color_morph_open", 3)
        self.declare_parameter("color_morph_close", 5)
        self.declare_parameter("color_apply_mask_to_output", False)

        self._load_params()
        self.add_on_set_parameters_callback(self._on_parameter_change)

        self._publisher = self.create_publisher(Image, self.output_topic, 10)
        self._debug_publisher = self.create_publisher(Image, self.debug_topic, 10)
        self._mask_publisher = self.create_publisher(Image, self.mask_topic, 10)

        self._subscription = self.create_subscription(
            Image,
            self.input_topic,
            self._image_callback,
            10,
        )

        self.get_logger().info(
            f"Started image_preprocessor_node | mode={self.mode} | "
            f"input={self.input_topic} | output={self.output_topic}"
        )

    def _load_params(self) -> None:
        self.input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.debug_topic = self.get_parameter("debug_topic").get_parameter_value().string_value
        self.mask_topic = self.get_parameter("mask_topic").get_parameter_value().string_value

        self.mode = self.get_parameter("mode").get_parameter_value().string_value.strip().lower()
        if self.mode not in self.VALID_MODES:
            self.get_logger().warn(f"Invalid mode '{self.mode}', falling back to 'face'.")
            self.mode = "face"

        self.output_width = int(self.get_parameter("output_width").value)
        self.output_height = int(self.get_parameter("output_height").value)
        self.keep_aspect_ratio = bool(self.get_parameter("keep_aspect_ratio").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.publish_mask = bool(self.get_parameter("publish_mask").value)

        self.face_output_encoding = self.get_parameter("face_output_encoding").value
        self.face_equalize_hist = bool(self.get_parameter("face_equalize_hist").value)
        self.face_gaussian_blur = bool(self.get_parameter("face_gaussian_blur").value)
        self.face_blur_kernel = int(self.get_parameter("face_blur_kernel").value)

        self.color_space = self.get_parameter("color_space").value.strip().lower()
        self.color_lower_h = int(self.get_parameter("color_lower_h").value)
        self.color_lower_s = int(self.get_parameter("color_lower_s").value)
        self.color_lower_v = int(self.get_parameter("color_lower_v").value)
        self.color_upper_h = int(self.get_parameter("color_upper_h").value)
        self.color_upper_s = int(self.get_parameter("color_upper_s").value)
        self.color_upper_v = int(self.get_parameter("color_upper_v").value)
        self.color_morph_open = int(self.get_parameter("color_morph_open").value)
        self.color_morph_close = int(self.get_parameter("color_morph_close").value)
        self.color_apply_mask_to_output = bool(self.get_parameter("color_apply_mask_to_output").value)

    def _on_parameter_change(self, params) -> SetParametersResult:
        try:
            for param in params:
                if param.name == "mode":
                    new_mode = str(param.value).strip().lower()
                    if new_mode not in self.VALID_MODES:
                        return SetParametersResult(
                            successful=False,
                            reason=f"Unsupported mode: {new_mode}",
                        )
                if param.name == "face_output_encoding":
                    if str(param.value) not in {"rgb8", "bgr8", "mono8"}:
                        return SetParametersResult(
                            successful=False,
                            reason="face_output_encoding must be one of: rgb8, bgr8, mono8",
                        )
                if param.name == "color_space":
                    if str(param.value).strip().lower() != "hsv":
                        return SetParametersResult(
                            successful=False,
                            reason="Only hsv color_space is currently supported.",
                        )
            self._load_params()
            self.get_logger().info("Parameters updated successfully.")
            return SetParametersResult(successful=True)
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))

    def _image_callback(self, msg: Image) -> None:
        try:
            frame_bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"Failed to convert incoming image: {exc}")
            return

        processed: Optional[np.ndarray] = None
        processed_encoding: str = "bgr8"
        debug_image: Optional[np.ndarray] = None
        debug_encoding: str = "bgr8"
        mask: Optional[np.ndarray] = None

        if self.mode == "face":
            processed, processed_encoding, debug_image, debug_encoding = self._process_face(frame_bgr)
        elif self.mode == "color":
            processed, processed_encoding, debug_image, debug_encoding, mask = self._process_color(frame_bgr)
        else:
            self.get_logger().error(f"Unexpected mode '{self.mode}'.")
            return

        if processed is not None:
            self._publish_image(self._publisher, processed, msg, processed_encoding)

        if self.publish_debug_image and debug_image is not None:
            self._publish_image(self._debug_publisher, debug_image, msg, debug_encoding)

        if self.mode == "color" and self.publish_mask and mask is not None:
            self._publish_image(self._mask_publisher, mask, msg, "mono8")

    def _process_face(
        self,
        frame_bgr: np.ndarray,
    ) -> Tuple[np.ndarray, str, np.ndarray, str]:
        resized_bgr = self._resize_for_output(frame_bgr)
        work_bgr = resized_bgr.copy()

        if self.face_gaussian_blur:
            k = self._ensure_odd_kernel(self.face_blur_kernel)
            work_bgr = cv2.GaussianBlur(work_bgr, (k, k), 0)

        debug_bgr = work_bgr.copy()

        if self.face_output_encoding == "mono8":
            gray = cv2.cvtColor(work_bgr, cv2.COLOR_BGR2GRAY)
            if self.face_equalize_hist:
                gray = cv2.equalizeHist(gray)
            return gray, "mono8", debug_bgr, "bgr8"

        if self.face_equalize_hist:
            gray = cv2.cvtColor(work_bgr, cv2.COLOR_BGR2GRAY)
            eq = cv2.equalizeHist(gray)
            work_bgr = cv2.cvtColor(eq, cv2.COLOR_GRAY2BGR)
            debug_bgr = work_bgr.copy()

        if self.face_output_encoding == "rgb8":
            rgb = cv2.cvtColor(work_bgr, cv2.COLOR_BGR2RGB)
            return rgb, "rgb8", debug_bgr, "bgr8"

        return work_bgr, "bgr8", debug_bgr, "bgr8"

    def _process_color(
        self,
        frame_bgr: np.ndarray,
    ) -> Tuple[np.ndarray, str, np.ndarray, str, np.ndarray]:
        resized_bgr = self._resize_for_output(frame_bgr)
        hsv = cv2.cvtColor(resized_bgr, cv2.COLOR_BGR2HSV)

        lower = np.array([self.color_lower_h, self.color_lower_s, self.color_lower_v], dtype=np.uint8)
        upper = np.array([self.color_upper_h, self.color_upper_s, self.color_upper_v], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        if self.color_morph_open > 1:
            k_open = self._make_kernel(self.color_morph_open)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k_open)

        if self.color_morph_close > 1:
            k_close = self._make_kernel(self.color_morph_close)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_close)

        if self.color_apply_mask_to_output:
            processed_bgr = cv2.bitwise_and(resized_bgr, resized_bgr, mask=mask)
        else:
            processed_bgr = resized_bgr

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        debug_bgr = processed_bgr.copy()
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 0:
                x, y, w, h = cv2.boundingRect(largest)
                cv2.rectangle(debug_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cx = x + w // 2
                cy = y + h // 2
                cv2.circle(debug_bgr, (cx, cy), 4, (0, 0, 255), -1)

        return processed_bgr, "bgr8", debug_bgr, "bgr8", mask

    def _resize_for_output(self, image: np.ndarray) -> np.ndarray:
        target_size = (self.output_width, self.output_height)
        if self.output_width <= 0 or self.output_height <= 0:
            return image

        if not self.keep_aspect_ratio:
            return cv2.resize(image, target_size, interpolation=cv2.INTER_LINEAR)

        h, w = image.shape[:2]
        if h == 0 or w == 0:
            return image

        scale = min(self.output_width / w, self.output_height / h)
        new_w = max(1, int(round(w * scale)))
        new_h = max(1, int(round(h * scale)))

        resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        canvas = np.zeros((self.output_height, self.output_width, 3), dtype=np.uint8)

        x_offset = (self.output_width - new_w) // 2
        y_offset = (self.output_height - new_h) // 2
        canvas[y_offset:y_offset + new_h, x_offset:x_offset + new_w] = resized
        return canvas

    def _publish_image(
        self,
        publisher,
        image: np.ndarray,
        source_msg: Image,
        encoding: str,
    ) -> None:
        try:
            out_msg = self._bridge.cv2_to_imgmsg(image, encoding=encoding)
        except CvBridgeError as exc:
            self.get_logger().error(f"Failed to convert OpenCV image to ROS message: {exc}")
            return

        out_msg.header = source_msg.header
        publisher.publish(out_msg)

    @staticmethod
    def _ensure_odd_kernel(kernel_size: int) -> int:
        k = max(1, int(kernel_size))
        if k % 2 == 0:
            k += 1
        return k

    @staticmethod
    def _make_kernel(kernel_size: int) -> np.ndarray:
        k = max(1, int(kernel_size))
        return np.ones((k, k), dtype=np.uint8)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImagePreprocessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()