"""Lazy Orbbec RGB-D adapter.

The module itself has no vendor dependency.  Constructing
:class:`OrbbecDriver` loads ``pyorbbecsdk`` and names the install extra if it
is unavailable.
"""

from __future__ import annotations

import importlib
import threading

import numpy as np

from .base import CameraFrame

__all__ = ["OrbbecDriver"]


def _vendor_module():
    try:
        return importlib.import_module("pyorbbecsdk")
    except ModuleNotFoundError as exc:
        if exc.name != "pyorbbecsdk":
            raise
        raise RuntimeError(
            "Orbbec capture needs the optional camera package: "
            "pip install 'waddle-sdk[orbbec]'"
        ) from exc


def _profile(pipeline, sensor, width: int, height: int, fmt, fps: int):
    profiles = pipeline.get_stream_profile_list(sensor)
    return profiles.get_video_stream_profile(width, height, fmt, fps)


class OrbbecDriver:
    """One pixel-aligned Orbbec RGB-D stream satisfying ``CameraDriver``."""

    def __init__(
        self,
        *,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
    ) -> None:
        sdk = _vendor_module()
        pipeline = sdk.Pipeline()
        config = sdk.Config()
        color = _profile(
            pipeline,
            sdk.OBSensorType.COLOR_SENSOR,
            width,
            height,
            sdk.OBFormat.RGB,
            fps,
        )
        depth_format = getattr(
            sdk.OBFormat, "Y16", getattr(sdk.OBFormat, "Z16", None)
        )
        if depth_format is None:
            raise RuntimeError(
                "this pyorbbecsdk build exposes neither Y16 nor Z16 depth"
            )
        depth = _profile(
            pipeline,
            sdk.OBSensorType.DEPTH_SENSOR,
            width,
            height,
            depth_format,
            fps,
        )
        config.enable_stream(color)
        config.enable_stream(depth)
        pipeline.start(config)
        self._pipeline = pipeline
        self._align = sdk.AlignFilter(align_to_stream=sdk.OBStreamType.COLOR_STREAM)
        self.depth_scale_mm: float | None = None
        self._closed = False
        self._close_lock = threading.Lock()

    def capture(self) -> CameraFrame:
        frames = self._pipeline.wait_for_frames(1000)
        if frames is None:
            raise RuntimeError("Orbbec timed out waiting for an RGB-D frameset")
        frames = self._align.process(frames)
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        if color is None or depth is None:
            raise RuntimeError("Orbbec returned an incomplete RGB-D frameset")
        width, height = int(color.get_width()), int(color.get_height())
        if (int(depth.get_width()), int(depth.get_height())) != (width, height):
            raise RuntimeError("Orbbec did not return pixel-aligned RGB-D frames")
        self.depth_scale_mm = float(depth.get_depth_scale())
        rgb = np.frombuffer(color.get_data(), dtype=np.uint8).reshape(height, width, 3)
        z16 = np.frombuffer(depth.get_data(), dtype=np.uint16).reshape(height, width)
        return CameraFrame(rgb=rgb, depth=z16)

    def close(self) -> None:
        with self._close_lock:
            if self._closed:
                return
            self._closed = True
            self._pipeline.stop()
