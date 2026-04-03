from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class DetectionResult:
    mode: str
    label: str
    detected: bool = False
    center_x: int = 0
    center_y: int = 0
    bbox_x: int = 0
    bbox_y: int = 0
    bbox_w: int = 0
    bbox_h: int = 0
    area: float = 0.0
    score: float = 0.0


class BaseDetector(ABC):
    @abstractmethod
    def detect(self, image_bgr: np.ndarray, mask: Optional[np.ndarray] = None) -> DetectionResult:
        raise NotImplementedError