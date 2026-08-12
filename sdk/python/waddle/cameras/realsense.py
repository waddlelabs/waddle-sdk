"""Lazy Intel RealSense RGB-D adapter.

The module itself has no vendor dependency.  Constructing
:class:`RealSenseDriver` loads ``pyrealsense2`` and names the install extra if
it is unavailable.
"""

from __future__ import annotations

import importlib
import threading

import numpy as np

from .base import CameraFrame

__all__ = ["RealSenseDriver"]


def _vendor_module():
    try:
        return importlib.import_module("pyrealsense2")
    except ModuleNotFoundError as exc:
        if exc.name != "pyrealsense2":
            raise
        raise RuntimeError(
            "RealSense capture needs the optional camera package: "
            "pip install 'waddle-sdk[realsense]'"
        ) from exc


class RealSenseDriver:
    """One aligned RealSense RGB-D stream satisfying ``CameraDriver``."""

    def __init__(
        self,
        *,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        serial: str | None = None,
    ) -> None:
        rs = _vendor_module()
        pipeline = rs.pipeline()
        config = rs.config()
        if serial:
            config.enable_device(serial)
        config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        profile = pipeline.start(config)
        self._pipeline = pipeline
        self._align = rs.align(rs.stream.color)
        self._closed = False
        self._close_lock = threading.Lock()
        self.depth_scale_mm = (
            float(profile.get_device().first_depth_sensor().get_depth_scale()) * 1000.0
        )

    def capture(self) -> CameraFrame:
        frames = self._align.process(self._pipeline.wait_for_frames())
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        if not color or not depth:
            raise RuntimeError("RealSense returned an incomplete RGB-D frameset")
        return CameraFrame(
            rgb=np.asanyarray(color.get_data()),
            depth=np.asanyarray(depth.get_data()),
        )

    def close(self) -> None:
        with self._close_lock:
            if self._closed:
                return
            self._closed = True
            self._pipeline.stop()
