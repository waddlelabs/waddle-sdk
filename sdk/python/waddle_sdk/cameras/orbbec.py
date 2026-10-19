"""Lazy Orbbec RGB-D adapter.

The module itself has no vendor dependency.  Constructing
:class:`OrbbecDriver` loads ``pyorbbecsdk`` and names the install extra if it
is unavailable.
"""

from __future__ import annotations

import importlib
import logging
import math
import threading
import time

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
        if serial is not None and (
            not isinstance(serial, str) or not serial or serial.strip() != serial
        ):
            raise ValueError("Orbbec serial must be a nonempty string")
        sdk = _vendor_module()
        self._context = sdk.Context() if serial is not None else None
        if self._context is not None:
            device = self._context.query_devices().get_device_by_serial_number(serial)
            if device is None or device.get_device_info().get_serial_number() != serial:
                raise RuntimeError(
                    f"Orbbec selected serial {serial!r} is not connected"
                )
            pipeline = sdk.Pipeline(device)
        else:
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
        pipeline.enable_frame_sync()
        self._pipeline = pipeline
        self._align = sdk.AlignFilter(align_to_stream=sdk.OBStreamType.COLOR_STREAM)
        self._formats = (sdk.OBFormat.RGB, depth_format)
        self._size = (width, height)
        self.depth_scale_mm: float | None = None
        self._intrinsics: Intrinsics | None = None
        self._initial_frame: CameraFrame | None = None
        self._closed = False
        self._close_lock = threading.Lock()
        self._capture_lock = threading.Lock()
        try:
            pipeline.start(config)
            self._initial_frame = self._capture()
        except BaseException:
            try:
                self.close()
            except Exception:
                logging.getLogger(__name__).exception("Orbbec startup cleanup failed")
            raise

    def capture(self) -> CameraFrame:
        with self._capture_lock:
            if self._closed:
                raise RuntimeError("Orbbec camera is closed")
            if self._initial_frame is not None:
                frame, self._initial_frame = self._initial_frame, None
                return frame
            return self._capture()

    def _capture(self) -> CameraFrame:
        deadline = time.monotonic() + 3.0
        while True:
            if self._closed:
                raise RuntimeError("Orbbec camera is closed")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError(
                    "Orbbec timed out waiting for an aligned RGB-D frameset"
                )
            frames = self._pipeline.wait_for_frames(
                min(1000, max(1, int(remaining * 1000)))
            )
            if frames is None:
                continue
            aligned = self._align.process(frames)
            if aligned is None:
                continue
            frames = aligned.as_frame_set()
            if frames is None:
                continue
            color, depth = frames.get_color_frame(), frames.get_depth_frame()
            if color is not None and depth is not None:
                break
        if self._closed:
            raise RuntimeError("Orbbec camera is closed")
        width, height = int(color.get_width()), int(color.get_height())
        if (width, height) != self._size or (
            int(depth.get_width()),
            int(depth.get_height()),
        ) != (width, height):
            raise RuntimeError("Orbbec did not return pixel-aligned RGB-D frames")
        if (color.get_format(), depth.get_format()) != self._formats:
            raise RuntimeError("Orbbec returned unexpected RGB-D pixel formats")
        scale = float(depth.get_depth_scale())
        if not math.isfinite(scale) or scale <= 0:
            raise RuntimeError("Orbbec returned an invalid depth scale")
        if self.depth_scale_mm is not None and self.depth_scale_mm != scale:
            raise RuntimeError("Orbbec depth scale changed while streaming")
        self.depth_scale_mm = scale
        intrinsics = self._color_intrinsics(color, width, height)
        if self._intrinsics is not None and self._intrinsics != intrinsics:
            raise RuntimeError("Orbbec color calibration changed while streaming")
        self._intrinsics = intrinsics
        # Own the pixels even when the vendor exposes a read-only view of a
        # reusable buffer. CameraFrame then freezes these owned copies.
        rgb = (
            np.frombuffer(color.get_data(), dtype=np.uint8)
            .reshape(height, width, 3)
            .copy()
        )
        z16 = (
            np.frombuffer(depth.get_data(), dtype=np.uint16)
            .reshape(height, width)
            .copy()
        )
        return CameraFrame(rgb=rgb, depth=z16)

    def _color_intrinsics(self, color, width: int, height: int) -> Intrinsics:
        profile = color.get_stream_profile().as_video_stream_profile()
        raw, distortion = profile.get_intrinsic(), profile.get_distortion()
        if (raw.width, raw.height) != (width, height):
            raise RuntimeError(
                "Orbbec color calibration dimensions differ from the aligned frame"
            )
        model = distortion.model.name
        if model == "NONE":
            name, coefficients = "plumb_bob", ()
        elif model == "KANNALA_BRANDT4":
            name = "kannala_brandt"
            coefficients = tuple(
                float(getattr(distortion, f"k{i}")) for i in range(1, 5)
            )
        elif model in {
            "BROWN_CONRADY",
            "BROWN_CONRADY_K6",
            "MODIFIED_BROWN_CONRADY",
            "INVERSE_BROWN_CONRADY",
        }:
            name = {
                "BROWN_CONRADY": "plumb_bob",
                "BROWN_CONRADY_K6": "rational_polynomial",
                "MODIFIED_BROWN_CONRADY": "modified_brown_conrady",
                "INVERSE_BROWN_CONRADY": "inverse_brown_conrady",
            }[model]
            keys = ("k1", "k2", "p1", "p2", "k3")
            if model == "BROWN_CONRADY_K6":
                keys += ("k4", "k5", "k6")
            coefficients = tuple(float(getattr(distortion, key)) for key in keys)
        else:
            raise RuntimeError(f"Orbbec color distortion model is unsupported: {model}")
        values = tuple(float(value) for value in (raw.fx, raw.fy, raw.cx, raw.cy))
        if (
            not all(math.isfinite(value) for value in (*values, *coefficients))
            or min(values[:2]) <= 0
        ):
            raise RuntimeError("Orbbec returned invalid color calibration")
        return Intrinsics(
            *values,
            distortion_model=name,
            distortion=coefficients,
            depth_scale_mm=self.depth_scale_mm,
        )

    def intrinsics(self) -> Intrinsics:
        assert self._intrinsics is not None
        return self._intrinsics

    def close(self) -> None:
        with self._close_lock:
            if self._closed:
                return
            self._closed = True
            self._pipeline.stop()
