"""Lazy USB/UVC RGB camera adapter backed by OpenCV."""

from __future__ import annotations

import importlib
import threading
from typing import Any

import numpy as np

from .base import CameraFrame

__all__ = ["USBDriver"]


def _vendor_module():
    try:
        return importlib.import_module("cv2")
    except ModuleNotFoundError as exc:
        if exc.name != "cv2":
            raise
        raise RuntimeError(
            "USB camera capture needs OpenCV: pip install 'waddle-sdk[usb]'"
        ) from exc


class USBDriver:
    """One RGB-only UVC stream satisfying :class:`CameraDriver`."""

    def __init__(
        self,
        *,
        device: int | str = 0,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
    ) -> None:
        cv2 = _vendor_module()
        capture = cv2.VideoCapture(device)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f"USB camera failed to open device {device!r}")
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
        capture.set(cv2.CAP_PROP_FPS, int(fps))
        self._cv2: Any = cv2
        self._capture: Any | None = capture
        self._lock = threading.Lock()

    def capture(self) -> CameraFrame:
        with self._lock:
            capture = self._capture
            if capture is None:
                raise RuntimeError("USB camera is closed")
            ok, bgr = capture.read()
        if not ok or bgr is None:
            raise RuntimeError("USB camera returned no frame")
        rgb = self._cv2.cvtColor(bgr, self._cv2.COLOR_BGR2RGB)
        return CameraFrame(rgb=np.ascontiguousarray(rgb))

    def close(self) -> None:
        with self._lock:
            capture, self._capture = self._capture, None
        if capture is not None:
            capture.release()
