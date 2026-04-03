from __future__ import annotations

from typing import Optional

import cv2
import numpy as np

from recognition_pkg.detectors.base_detector import BaseDetector, DetectionResult


class ColorDetector(BaseDetector):
    def __init__(
        self,
        use_mask_input: bool = True,
        fallback_to_hsv: bool = True,
        min_area: float = 300.0,
        lower_h_1: int = 0,
        upper_h_1: int = 10,
        lower_h_2: int = 170,
        upper_h_2: int = 180,
        lower_s: int = 120,
        lower_v: int = 70,
        upper_s: int = 255,
        upper_v: int = 255,
    ) -> None:
        self.use_mask_input = bool(use_mask_input)
        self.fallback_to_hsv = bool(fallback_to_hsv)
        self.min_area = float(min_area)

        self.lower_h_1 = int(lower_h_1)
        self.upper_h_1 = int(upper_h_1)
        self.lower_h_2 = int(lower_h_2)
        self.upper_h_2 = int(upper_h_2)
        self.lower_s = int(lower_s)
        self.lower_v = int(lower_v)
        self.upper_s = int(upper_s)
        self.upper_v = int(upper_v)

    def detect(self, image_bgr: np.ndarray, mask: Optional[np.ndarray] = None) -> DetectionResult:
        working_mask: Optional[np.ndarray] = None

        if self.use_mask_input and mask is not None:
            working_mask = self._sanitize_mask(mask)
        elif self.fallback_to_hsv:
            working_mask = self._build_red_mask(image_bgr)

        if working_mask is None:
            return DetectionResult(mode="color", label="red", detected=False)

        contours, _ = cv2.findContours(working_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) >= self.min_area]

        if not contours:
            return DetectionResult(mode="color", label="red", detected=False)

        largest = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(largest))
        x, y, w, h = cv2.boundingRect(largest)
        cx = x + (w // 2)
        cy = y + (h // 2)

        frame_area = float(max(1, image_bgr.shape[0] * image_bgr.shape[1]))
        score = min(1.0, area / max(1.0, frame_area * 0.25))

        return DetectionResult(
            mode="color",
            label="red",
            detected=True,
            center_x=int(cx),
            center_y=int(cy),
            bbox_x=int(x),
            bbox_y=int(y),
            bbox_w=int(w),
            bbox_h=int(h),
            area=area,
            score=float(score),
        )

    @staticmethod
    def _sanitize_mask(mask: np.ndarray) -> np.ndarray:
        if mask.ndim == 3:
            mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)
        return binary

    def _build_red_mask(self, image_bgr: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)

        lower1 = np.array([self.lower_h_1, self.lower_s, self.lower_v], dtype=np.uint8)
        upper1 = np.array([self.upper_h_1, self.upper_s, self.upper_v], dtype=np.uint8)
        lower2 = np.array([self.lower_h_2, self.lower_s, self.lower_v], dtype=np.uint8)
        upper2 = np.array([self.upper_h_2, self.upper_s, self.upper_v], dtype=np.uint8)

        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)

        return cv2.bitwise_or(mask1, mask2)