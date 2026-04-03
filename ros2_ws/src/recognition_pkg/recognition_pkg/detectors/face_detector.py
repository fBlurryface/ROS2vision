from __future__ import annotations

from pathlib import Path

import cv2

from recognition_pkg.detectors.base_detector import BaseDetector, DetectionResult


class FaceDetector(BaseDetector):
    def __init__(
        self,
        cascade_path: str = "",
        min_size: int = 40,
        scale_factor: float = 1.1,
        min_neighbors: int = 5,
    ) -> None:
        if cascade_path:
            resolved_path = Path(cascade_path)
        else:
            resolved_path = Path(cv2.data.haarcascades) / "haarcascade_frontalface_default.xml"

        self.cascade_path = str(resolved_path)
        self.min_size = int(min_size)
        self.scale_factor = float(scale_factor)
        self.min_neighbors = int(min_neighbors)

        self._classifier = cv2.CascadeClassifier(self.cascade_path)
        if self._classifier.empty():
            raise ValueError(f"Failed to load face cascade: {self.cascade_path}")

    def detect(self, image_bgr, mask=None) -> DetectionResult:
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)

        faces = self._classifier.detectMultiScale(
            gray,
            scaleFactor=self.scale_factor,
            minNeighbors=self.min_neighbors,
            minSize=(self.min_size, self.min_size),
        )

        if len(faces) == 0:
            return DetectionResult(mode="face", label="face", detected=False)

        x, y, w, h = max(faces, key=lambda rect: rect[2] * rect[3])
        cx = x + (w // 2)
        cy = y + (h // 2)
        area = float(w * h)

        frame_area = float(max(1, image_bgr.shape[0] * image_bgr.shape[1]))
        score = min(1.0, area / max(1.0, frame_area * 0.25))

        return DetectionResult(
            mode="face",
            label="face",
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