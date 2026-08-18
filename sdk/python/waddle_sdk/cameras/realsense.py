"""Lazy Intel RealSense RGB-D adapter.

The module itself has no vendor dependency. Constructing
:class:`RealSenseDriver` loads ``pyrealsense2`` and names the install extra if
it is unavailable.

RealSense devices can remain enumerated while their pipeline is wedged and
delivers no frames. Opening therefore proves that frames flow, hardware-resets
once when they do not, and retries. A later capture timeout similarly rebuilds
the pipeline once instead of permanently killing the owning camera pump.
"""

from __future__ import annotations

import importlib
import threading
import time
from collections.abc import Callable
from typing import Any

import numpy as np

from ..descriptors import Intrinsics
from .base import CameraFrame

__all__ = ["RealSenseDriver"]


_CONTEXT_LOCK = threading.Lock()
_RS_CONTEXT: Any = None
_RS_MODULE: Any = None


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


def _shared_context(rs: Any) -> Any:
    """Keep librealsense's USB watcher context alive for the process.

    The module identity check keeps fake-vendor tests isolated while real
    processes still construct exactly one context.
    """

    global _RS_CONTEXT, _RS_MODULE
    with _CONTEXT_LOCK:
        if _RS_CONTEXT is None or _RS_MODULE is not rs:
            _RS_MODULE = rs
            _RS_CONTEXT = rs.context()
        return _RS_CONTEXT


class RealSenseDriver:
    """One aligned, self-recovering RealSense RGB-D stream."""

    _ENUMERATION_TIMEOUT_S = 6.0
    _STREAM_CONFIRM_TIMEOUT_S = 3.0
    _RESET_SETTLE_S = 3.0

    def __init__(
        self,
        *,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        serial: str | None = None,
    ) -> None:
        self._rs = _vendor_module()
        self._width = width
        self._height = height
        self._fps = fps
        self._serial = serial
        self._pipeline: Any = None
        self._align: Any = None
        self._recover_lock = threading.Lock()
        self._closed = False
        self.depth_scale_mm = 0.0
        self._intrinsics: Intrinsics | None = None
        self._active_intrinsics: Any = None
        with self._recover_lock:
            self._open_with_recovery()

    def _select_device(self) -> Any:
        context = _shared_context(self._rs)
        deadline = time.monotonic() + self._ENUMERATION_TIMEOUT_S
        while True:
            for device in context.query_devices():
                serial = device.get_info(self._rs.camera_info.serial_number)
                if self._serial is None or serial == self._serial:
                    return device
            if time.monotonic() >= deadline:
                wanted = self._serial or "any"
                raise RuntimeError(f"RealSense device {wanted!r} did not enumerate")
            time.sleep(0.25)

    def _stop_pipeline(self) -> None:
        pipeline, self._pipeline = self._pipeline, None
        self._align = None
        self._active_intrinsics = None
        if pipeline is not None:
            try:
                pipeline.stop()
            except RuntimeError:
                pass

    def _start_pipeline(self, device: Any) -> None:
        context = _shared_context(self._rs)
        pipeline = self._rs.pipeline(context)
        config = self._rs.config()
        serial = device.get_info(self._rs.camera_info.serial_number)
        config.enable_device(serial)
        config.enable_stream(
            self._rs.stream.color,
            self._width,
            self._height,
            self._rs.format.rgb8,
            self._fps,
        )
        config.enable_stream(
            self._rs.stream.depth,
            self._width,
            self._height,
            self._rs.format.z16,
            self._fps,
        )
        try:
            profile = pipeline.start(config)
        except Exception:
            try:
                pipeline.stop()
            except RuntimeError:
                pass
            raise
        self._pipeline = pipeline
        self._align = self._rs.align(self._rs.stream.color)
        self.depth_scale_mm = (
            float(profile.get_device().first_depth_sensor().get_depth_scale()) * 1000.0
        )
        try:
            active = profile.get_stream(self._rs.stream.color)
            video = active.as_video_stream_profile()
            raw = video.get_intrinsics()
            self._active_intrinsics = raw
            coefficients = tuple(float(value) for value in getattr(raw, "coeffs", ()))
            self._intrinsics = Intrinsics(
                fx=float(raw.fx),
                fy=float(raw.fy),
                cx=float(raw.ppx),
                cy=float(raw.ppy),
                distortion=coefficients,
                depth_scale_mm=self.depth_scale_mm,
            )
        except (AttributeError, TypeError, ValueError):
            # Older/fake vendor profiles may not expose calibration.  The
            # optional extension then remains unavailable and site.yaml is
            # still the authoritative fallback.
            self._intrinsics = None

    def _point_resolver(
        self,
    ) -> Callable[[int, int, float], tuple[float, float, float]]:
        intrinsics = self._active_intrinsics
        if intrinsics is None:
            raise RuntimeError("the active RealSense profile exposes no intrinsics")

        def resolve(x: int, y: int, depth_m: float) -> tuple[float, float, float]:
            point = self._rs.rs2_deproject_pixel_to_point(
                intrinsics,
                [float(x), float(y)],
                float(depth_m),
            )
            return float(point[0]), float(point[1]), float(point[2])

        return resolve

    def intrinsics(self) -> Intrinsics:
        """Return intrinsics for the aligned RGB/depth grid in active use."""

        if self._intrinsics is None:
            raise RuntimeError("the active RealSense profile exposes no intrinsics")
        return self._intrinsics

    def _confirm_streaming(self) -> bool:
        if self._pipeline is None:
            return False
        deadline = time.monotonic() + self._STREAM_CONFIRM_TIMEOUT_S
        while time.monotonic() < deadline:
            try:
                ok, _frames = self._pipeline.try_wait_for_frames(250)
            except RuntimeError:
                ok = False
            if ok:
                return True
        return False

    def _open_with_recovery(self) -> None:
        """Open and prove frame flow, resetting one wedged device once."""

        last_error: Exception | None = None
        for attempt in range(2):
            if self._closed:
                raise RuntimeError("RealSense camera is closed")
            device = self._select_device()
            try:
                self._start_pipeline(device)
                if not self._confirm_streaming():
                    raise RuntimeError("pipeline started but no frames arrived")
                return
            except RuntimeError as exc:
                last_error = exc
                self._stop_pipeline()
                if attempt == 0:
                    try:
                        device.hardware_reset()
                    except RuntimeError:
                        pass
                    time.sleep(self._RESET_SETTLE_S)
        assert last_error is not None
        raise RuntimeError(
            "RealSense stream did not recover after one hardware reset"
        ) from last_error

    def _capture_once(self) -> CameraFrame:
        pipeline = self._pipeline
        align = self._align
        if pipeline is None or align is None:
            raise RuntimeError("RealSense camera is not open")
        frames = align.process(pipeline.wait_for_frames())
        color = frames.get_color_frame()
        depth = frames.get_depth_frame()
        if not color or not depth:
            raise RuntimeError("RealSense returned an incomplete RGB-D frameset")
        return CameraFrame(
            rgb=np.asanyarray(color.get_data()),
            depth=np.asanyarray(depth.get_data()),
            point_resolver=self._point_resolver(),
        )

    def capture(self) -> CameraFrame:
        if self._closed:
            raise RuntimeError("RealSense camera is closed")
        try:
            return self._capture_once()
        except RuntimeError as first_error:
            # Only one thread calls capture in the SDK, but close may race it.
            # Serialize the rebuild and check closure again before reopening.
            with self._recover_lock:
                if self._closed:
                    raise RuntimeError("RealSense camera is closed") from first_error
                self._stop_pipeline()
                self._open_with_recovery()
                return self._capture_once()

    def close(self) -> None:
        with self._recover_lock:
            if self._closed:
                return
            self._closed = True
            self._stop_pipeline()
