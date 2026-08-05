"""OpenCV camera that does NOT force width/height/fps/fourcc on the device.

The Intel D435i UVC color node rejects manual resolution settings, which
makes LeRobot's stock ``opencv`` camera fail at ``_configure_capture_settings``
and prevents the ``realsense`` type from registering without pyrealsense2.
This camera opens the node and reads frames at the device defaults (the
D435i color node yields 640x480@30 out of the box), while still exposing the
configured width/height so LeRobot's robot validation is satisfied.
"""

from __future__ import annotations

from dataclasses import dataclass, kw_only
from typing import Any

import cv2
import numpy as np

from lerobot.cameras import Camera, CameraConfig
from lerobot.utils.errors import DeviceNotConnectedError


@CameraConfig.register_subclass("uf::opencv_default")
@dataclass(kw_only=True)
class OpenCVDefaultCameraConfig(CameraConfig):
    index_or_path: str
    width: int = 640
    height: int = 480
    fps: int = 30


class OpenCVDefaultCamera(Camera):
    config_class = OpenCVDefaultCameraConfig
    name = "opencv_default"

    def __init__(self, config: OpenCVDefaultCameraConfig) -> None:
        super().__init__(config)
        self.config = config
        self.cap: cv2.VideoCapture | None = None

    @property
    def height(self) -> int:
        return self.config.height

    @property
    def width(self) -> int:
        return self.config.width

    def connect(self) -> None:
        if self.cap is not None:
            return
        cap = cv2.VideoCapture(self.config.index_or_path, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise ConnectionError(
                f"Failed to open OpenCVDefaultCamera({self.config.index_or_path})"
            )
        self.cap = cap

    def read(self) -> np.ndarray:
        if self.cap is None:
            raise DeviceNotConnectedError(f"{self} is not connected")
        ok, frame = self.cap.read()
        if not ok or frame is None:
            raise RuntimeError(
                f"Failed to read frame from {self.config.index_or_path}"
            )
        return frame

    def async_read(self) -> np.ndarray:
        return self.read()

    def disconnect(self) -> None:
        if self.cap is not None:
            self.cap.release()
            self.cap = None


__all__ = ["OpenCVDefaultCamera", "OpenCVDefaultCameraConfig"]
