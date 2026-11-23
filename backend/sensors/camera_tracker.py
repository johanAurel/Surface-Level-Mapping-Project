"""Camera-based reference tracking.

PSEUDOCODE:
    CLASS CameraTracker(config):
        connect(): initialize Picamera2 lazily
        read(): capture frame -> process_frame() -> dict
        process_frame(frame):
            CONVERT BGR->HSV
            MASK by HSV range
            FIND contours
            SELECT largest contour as reference
            COMPUTE center and pixel error from image center
            RETURN CameraTrackingReading
        close(): stop/close camera if owned
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Optional, Tuple

import numpy as np

try:
    import cv2
except Exception:  # pragma: no cover - hardware dependency
    cv2 = None

try:
    from picamera2 import Picamera2
except Exception:  # pragma: no cover - hardware dependency
    Picamera2 = None


@dataclass
class CameraTrackingReading:
    reference_detected: bool
    reference_x_px: Optional[float] = None
    reference_y_px: Optional[float] = None
    x_error_px: Optional[float] = None
    y_error_px: Optional[float] = None
    bbox: Optional[Tuple[int, int, int, int]] = None
    frame_width: Optional[int] = None
    frame_height: Optional[int] = None
    area_px: Optional[float] = None


class CameraTracker:
    """Detect a colored reference target and report pixel offset from frame center."""

    def __init__(self, width: int = 720, height: int = 360, lower_hsv=(80, 40, 0), upper_hsv=(140, 200, 69), camera=None):
        # PSEUDOCODE: store detection config and optional injected camera (for testing).
        self.width = width
        self.height = height
        self.lower_hsv = np.array(lower_hsv, dtype=np.uint8)
        self.upper_hsv = np.array(upper_hsv, dtype=np.uint8)
        self.camera = camera
        self._owns_camera = camera is None

    def connect(self) -> None:
        # PSEUDOCODE: create/start camera only when needed.
        if self.camera is not None:
            return
        if Picamera2 is None:
            raise RuntimeError("Picamera2 is required for camera tracking")
        cam = Picamera2()
        cam.preview_configuration.main.size = (self.width, self.height)
        cam.preview_configuration.main.format = "RGB888"
        cam.preview_configuration.align()
        cam.configure("preview")
        cam.start()
        self.camera = cam

    def read(self) -> dict:
        # PSEUDOCODE: capture one frame and convert detection result to dict for CSV fusion.
        self.connect()
        if cv2 is None:
            raise RuntimeError("opencv-python is required for camera tracking")
        frame = self.camera.capture_array()
        return asdict(self.process_frame(frame))

    def process_frame(self, frame) -> CameraTrackingReading:
        # PSEUDOCODE: perform HSV masking and contour-based reference localization.
        if cv2 is None:
            raise RuntimeError("opencv-python is required for camera tracking")
        height, width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return CameraTrackingReading(reference_detected=False, frame_width=width, frame_height=height)

        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        if area <= 0:
            return CameraTrackingReading(reference_detected=False, frame_width=width, frame_height=height)

        x, y, w, h = cv2.boundingRect(contour)
        cx = x + w / 2.0
        cy = y + h / 2.0
        return CameraTrackingReading(
            reference_detected=True,
            reference_x_px=cx,
            reference_y_px=cy,
            x_error_px=cx - (width / 2.0),
            y_error_px=cy - (height / 2.0),
            bbox=(int(x), int(y), int(w), int(h)),
            frame_width=width,
            frame_height=height,
            area_px=area,
        )

    def close(self) -> None:
        # PSEUDOCODE: release camera resources only if this object created them.
        if self._owns_camera and self.camera is not None:
            stop = getattr(self.camera, "stop", None)
            if callable(stop):
                stop()
            close = getattr(self.camera, "close", None)
            if callable(close):
                close()
        if self._owns_camera:
            self.camera = None
