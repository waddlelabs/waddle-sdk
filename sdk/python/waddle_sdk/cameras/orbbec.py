"""Lazy Orbbec RGB-D adapter.

The module itself has no vendor dependency.  Constructing
:class:`OrbbecDriver` loads ``pyorbbecsdk`` and names the install extra if it
is unavailable.
"""

from __future__ import annotations

import importlib
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


def _distortion(distortion):
    """Translate the vendor-declared model, not just its coefficient storage."""
    name = getattr(getattr(distortion, "model", None), "name", None)
    layouts = {
        "NONE": ("plumb_bob", ()),
        "BROWN_CONRADY": ("plumb_bob", ("k1", "k2", "p1", "p2", "k3")),
        "BROWN_CONRADY_K6": (
            "rational_polynomial",
            ("k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6"),
        ),
        "MODIFIED_BROWN_CONRADY": (
            "modified_brown_conrady",
            ("k1", "k2", "p1", "p2", "k3"),
        ),
        "INVERSE_BROWN_CONRADY": (
            "inverse_brown_conrady",
            ("k1", "k2", "p1", "p2", "k3"),
        ),
        "KANNALA_BRANDT4": ("kannala_brandt", ("k1", "k2", "k3", "k4")),
    }
    if name not in layouts:
        raise ValueError(f"Unsupported Orbbec distortion model: {name!r}")
    model, keys = layouts[name]
    return model, tuple(float(getattr(distortion, key)) for key in keys)


class _FrameTimeout(RuntimeError):
    """Only the adapter's bounded acquisition timeout permits startup recovery."""


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
        self._sdk = sdk
        self._serial = serial
        self._pipeline = None
        self._closed = False
        self._close_lock = threading.Lock()
        self.depth_scale_mm: float | None = None
        self._intrinsics: Intrinsics | None = None
        self._context = sdk.Context() if serial is not None else None
        try:
            try:
                self._start(width, height, fps)
            except _FrameTimeout:
                if self._context is None:
                    raise
                # Only a timed-out owned stream may be rebooted, before arms open.
                # A failed stop is not permission to reset a possibly busy device.
                self._stop_pipeline()
                self._reboot_and_wait()
                self._start(width, height, fps)
        except BaseException as error:
            try:
                self.close()
            except Exception as cleanup:
                raise RuntimeError(
                    f"Orbbec startup failed: {error}; pipeline cleanup failed: {cleanup}"
                ) from error
            raise

    def _start(self, width: int, height: int, fps: int) -> None:
        sdk = self._sdk
        pipeline = (
            sdk.Pipeline(
                self._context.query_devices().get_device_by_serial_number(self._serial)
            )
            if self._context is not None
            else sdk.Pipeline()
        )
        self._pipeline = pipeline
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
        pipeline.start(config)
        self._align = sdk.AlignFilter(align_to_stream=sdk.OBStreamType.COLOR_STREAM)
        self._color_profile = color
        self._capture(attempts=10)

    def _stop_pipeline(self) -> None:
        if self._pipeline is not None:
            self._pipeline.stop()
            self._pipeline = None

    def _reboot_and_wait(self, timeout_s: float = 30.0) -> None:
        def present() -> bool:
            devices = self._context.query_devices()
            return any(
                devices.get_device_serial_number_by_index(i) == self._serial
                for i in range(devices.get_count())
            )

        device = self._context.query_devices().get_device_by_serial_number(self._serial)
        device.reboot()
        del device
        deadline = time.monotonic() + timeout_s
        for expected in (False, True):
            while present() is not expected:
                if time.monotonic() >= deadline:
                    phase = "return" if expected else "disconnect"
                    raise RuntimeError(
                        f"Orbbec {self._serial} did not {phase} after reboot"
                    )
                time.sleep(0.2)
        time.sleep(1.0)

    def intrinsics(self) -> Intrinsics:
        """Calibration of the active color grid to which depth is aligned."""
        if self._intrinsics is None:
            raise RuntimeError("Orbbec has no acquired RGB-D calibration")
        return self._intrinsics

    def capture(self) -> CameraFrame:
        return self._capture(attempts=1)

    def _capture(self, *, attempts: int) -> CameraFrame:
        if self._closed:
            raise RuntimeError("Orbbec camera is closed")
        # Startup may initially return empty frames. Retry only acquisition,
        # before actuators open; ordinary capture retains its one-second bound.
        failure = "no RGB-D frameset"
        for _ in range(attempts):
            frames = self._pipeline.wait_for_frames(1000)
            if frames is None:
                failure = "no RGB-D frameset"
                continue
            aligned = self._align.process(frames)
            if aligned is None:
                failure = "alignment returned no RGB-D frameset"
                continue
            frames = aligned.as_frame_set()
            color = frames.get_color_frame()
            depth = frames.get_depth_frame()
            if color is not None and depth is not None:
                break
            failure = f"incomplete RGB-D frameset (color={color is not None}, depth={depth is not None})"
        else:
            raise _FrameTimeout(
                f"Orbbec timed out after {attempts} frame wait(s): {failure}"
            )
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
            model, coefficients = _distortion(distortion)
            self._intrinsics = Intrinsics(
                fx=float(raw.fx),
                fy=float(raw.fy),
                cx=float(raw.cx),
                cy=float(raw.cy),
                distortion_model=model,
                distortion=coefficients,
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
            self._stop_pipeline()
            self._closed = True
