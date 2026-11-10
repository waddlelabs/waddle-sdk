"""Lazy Orbbec RGB-D adapter.

The module itself has no vendor dependency.  Constructing
:class:`OrbbecDriver` loads ``pyorbbecsdk`` and names the install extra if it
is unavailable.
"""

from __future__ import annotations

import importlib
import math
import threading

import numpy as np

from ..descriptors import Intrinsics
from .base import CameraFrame, _integral_fps

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
        serial: str | None = None,
    ) -> None:
        fps = _integral_fps(fps)
        sdk = _vendor_module()
        self._context = sdk.Context() if serial is not None else None
        pipeline = (
            sdk.Pipeline(
                self._context.query_devices().get_device_by_serial_number(serial)
            )
            if self._context is not None
            else sdk.Pipeline()
        )
        config = sdk.Config()
        color = _profile(
            pipeline,
            sdk.OBSensorType.COLOR_SENSOR,
            width,
            height,
            sdk.OBFormat.RGB,
            fps,
        )
        depth_format = getattr(sdk.OBFormat, "Y16", getattr(sdk.OBFormat, "Z16", None))
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
        config.set_frame_aggregate_output_mode(
            sdk.OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE
        )
        self._pipeline = pipeline
        self._closed = False
        self._close_lock = threading.Lock()
        self.depth_scale_mm: float | None = None
        self._intrinsics: Intrinsics | None = None
        try:
            pipeline.start(config)
            self._align = sdk.AlignFilter(align_to_stream=sdk.OBStreamType.COLOR_STREAM)
            self._color_profile = color
            # The scale belongs to an actual depth frame. Acquire it before the
            # rig snapshots calibration and opens actuators; never guess units.
            self.capture()
        except Exception:
            self.close()
            raise

    def intrinsics(self) -> Intrinsics:
        """Calibration of the active color grid to which depth is aligned."""
        if self._intrinsics is None:
            raise RuntimeError("Orbbec has no acquired RGB-D calibration")
        return self._intrinsics

    def capture(self) -> CameraFrame:
        if self._closed:
            raise RuntimeError("Orbbec camera is closed")
        frames = self._pipeline.wait_for_frames(1000)
        if frames is None:
            raise RuntimeError("Orbbec timed out waiting for an RGB-D frameset")
        frames = self._align.process(frames)
        if frames is None:
            raise RuntimeError("Orbbec alignment returned no RGB-D frameset")
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        if color is None or depth is None:
            raise RuntimeError("Orbbec returned an incomplete RGB-D frameset")
        width, height = int(color.get_width()), int(color.get_height())
        if (int(depth.get_width()), int(depth.get_height())) != (width, height):
            raise RuntimeError("Orbbec did not return pixel-aligned RGB-D frames")
        scale = float(depth.get_depth_scale())
        if not math.isfinite(scale) or scale <= 0:
            raise ValueError(
                "Orbbec depth frame must declare a finite positive depth scale"
            )
        if self.depth_scale_mm is not None and scale != self.depth_scale_mm:
            raise RuntimeError("Orbbec depth scale changed during capture")
        if self._intrinsics is None:
            raw = self._color_profile.get_intrinsic()
            distortion = self._color_profile.get_distortion()
            self._intrinsics = Intrinsics(
                fx=float(raw.fx),
                fy=float(raw.fy),
                cx=float(raw.cx),
                cy=float(raw.cy),
                distortion=tuple(
                    float(getattr(distortion, key))
                    for key in ("k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6")
                ),
                depth_scale_mm=scale,
            )
        self.depth_scale_mm = scale
        rgb = np.frombuffer(color.get_data(), dtype=np.uint8).reshape(height, width, 3)
        z16 = np.frombuffer(depth.get_data(), dtype=np.uint16).reshape(height, width)
        return CameraFrame(rgb=rgb, depth=z16)

    def close(self) -> None:
        with self._close_lock:
            if self._closed:
                return
            self._closed = True
            self._pipeline.stop()
