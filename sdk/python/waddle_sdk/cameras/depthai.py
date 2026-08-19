"""Lazy DepthAI 3 RGB-D adapter for fixed OAK cameras."""

from __future__ import annotations

import importlib
import threading
from datetime import timedelta
from typing import Any

import numpy as np

from ..descriptors import Intrinsics
from .base import CameraFrame

__all__ = ["DepthaiDriver", "available_devices"]


def _vendor_module():
    try:
        return importlib.import_module("depthai")
    except ModuleNotFoundError as exc:
        if exc.name != "depthai":
            raise
        raise RuntimeError(
            "DepthAI capture needs the optional camera package: "
            "pip install 'waddle-sdk[depthai]'"
        ) from exc


def available_devices() -> tuple[str, ...]:
    """Return stable DepthAI device IDs without booting a camera pipeline."""
    sdk = _vendor_module()
    return tuple(
        sorted(str(info.getDeviceId()) for info in sdk.Device.getAllAvailableDevices())
    )


def _device_info(sdk: Any, mxid: str) -> Any:
    devices = list(sdk.Device.getAllAvailableDevices())
    for info in devices:
        if str(info.getDeviceId()) == mxid:
            return info
    found = sorted(str(info.getDeviceId()) for info in devices)
    raise RuntimeError(f"DepthAI device {mxid!r} not found; available={found!r}")


class DepthaiDriver:
    """One synchronized RGB/depth stream aligned on an undistorted RGB grid.

    A stable mxid is required so two attached cameras cannot exchange logical
    names after USB enumeration order changes.
    """

    _CAPTURE_TIMEOUT_S = 1.0
    _WARMUP_FRAME_COUNT = 2

    def __init__(
        self,
        *,
        mxid: str,
        width: int = 640,
        height: int = 400,
        fps: int = 30,
        stereo_preset: str = "ROBOTICS",
    ) -> None:
        if not mxid:
            raise ValueError("DepthAI mxid is required when selecting a camera")
        if width <= 0 or height <= 0 or fps <= 0:
            raise ValueError("DepthAI width, height, and fps must be > 0")
        sdk = _vendor_module()
        self._sdk = sdk
        self._width = int(width)
        self._height = int(height)
        self._fps = int(fps)
        self._mxid = str(mxid)
        self._close_lock = threading.Lock()
        self._closed = False
        self._device: Any = None
        self._pipeline: Any = None
        self._queue: Any = None
        self._intrinsics: Intrinsics | None = None
        self._warmup_frames_remaining = self._WARMUP_FRAME_COUNT
        try:
            info = _device_info(sdk, self._mxid)
            self._device = sdk.Device(info)
            self._pipeline = sdk.Pipeline(self._device)
            try:
                preset = getattr(sdk.node.StereoDepth.PresetMode, stereo_preset.upper())
            except AttributeError as exc:
                choices = sorted(sdk.node.StereoDepth.PresetMode.__members__)
                raise ValueError(
                    f"unknown DepthAI stereo_preset={stereo_preset!r}; "
                    f"expected one of {choices}"
                ) from exc
            connected_fn = getattr(self._device, "getConnectedCameras", None)
            connected = None if connected_fn is None else tuple(connected_fn())
            cam_a = sdk.CameraBoardSocket.CAM_A
            if connected is None or cam_a in connected:
                self._rgb_socket = cam_a
                camera = self._pipeline.create(sdk.node.Camera).build(cam_a)
                rgb = camera.requestOutput(
                    (self._width, self._height),
                    sdk.ImgFrame.Type.RGB888i,
                    fps=float(self._fps),
                    enableUndistortion=True,
                )
                stereo = self._pipeline.create(sdk.node.StereoDepth).build(
                    True,
                    preset,
                    (self._width, self._height),
                    float(self._fps),
                )
            else:
                cam_b = sdk.CameraBoardSocket.CAM_B
                cam_c = sdk.CameraBoardSocket.CAM_C
                if cam_b not in connected or cam_c not in connected:
                    raise RuntimeError(
                        f"DepthAI {self._mxid} needs CAM_A or a CAM_B/C stereo pair; "
                        f"connected={connected!r}"
                    )
                self._rgb_socket = cam_b
                left = self._pipeline.create(sdk.node.Camera).build(cam_b)
                right = self._pipeline.create(sdk.node.Camera).build(cam_c)
                rgb = left.requestOutput(
                    (self._width, self._height),
                    sdk.ImgFrame.Type.RGB888i,
                    fps=float(self._fps),
                    enableUndistortion=True,
                )
                left_stereo = left.requestOutput(
                    (self._width, self._height),
                    sdk.ImgFrame.Type.GRAY8,
                    fps=float(self._fps),
                )
                right_stereo = right.requestOutput(
                    (self._width, self._height),
                    sdk.ImgFrame.Type.GRAY8,
                    fps=float(self._fps),
                )
                stereo = self._pipeline.create(sdk.node.StereoDepth).build(
                    left_stereo, right_stereo, preset
                )
            stereo.setDepthAlign(self._rgb_socket)
            stereo.setOutputSize(self._width, self._height)
            stereo.setLeftRightCheck(True)
            stereo.setSubpixel(True)
            sync = self._pipeline.create(sdk.node.Sync)
            sync.setSyncThreshold(timedelta(seconds=1.0 / self._fps))
            sync.setSyncAttempts(-1)
            rgb.link(sync.inputs["rgb"])
            stereo.depth.link(sync.inputs["depth"])
            self._queue = sync.out.createOutputQueue(maxSize=4, blocking=True)
            self._read_intrinsics()
            self._pipeline.start()
        except BaseException:
            self.close()
            raise

    def _read_intrinsics(self) -> None:
        calibration = self._device.readCalibration()
        matrix = calibration.getCameraIntrinsics(
            self._rgb_socket,
            self._width,
            self._height,
        )
        if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
            raise RuntimeError("DepthAI returned a malformed RGB intrinsic matrix")
        values = np.asarray(matrix, dtype=float)
        if not np.all(np.isfinite(values)):
            raise RuntimeError("DepthAI returned non-finite RGB intrinsics")
        self._intrinsics = Intrinsics(
            fx=float(values[0, 0]),
            fy=float(values[1, 1]),
            cx=float(values[0, 2]),
            cy=float(values[1, 2]),
            distortion=(),
            depth_scale_mm=1.0,
        )

    def intrinsics(self) -> Intrinsics:
        if self._intrinsics is None:
            raise RuntimeError("DepthAI camera has no active intrinsics")
        return self._intrinsics

    def capture(self) -> CameraFrame:
        if self._closed:
            raise RuntimeError("DepthAI camera is closed")
        warmup = self._warmup_frames_remaining
        self._warmup_frames_remaining = 0
        group = None
        for _ in range(warmup + 1):
            group = self._queue.get(timedelta(seconds=self._CAPTURE_TIMEOUT_S))
        if group is None:
            raise RuntimeError(
                f"DepthAI {self._mxid} timed out waiting for synchronized RGB-D"
            )
        rgb_frame = group["rgb"]
        depth_frame = group["depth"]
        if rgb_frame is None or depth_frame is None:
            raise RuntimeError(
                f"DepthAI {self._mxid} returned an incomplete RGB-D group"
            )
        rgb = np.asarray(rgb_frame.getFrame())
        depth = np.asarray(depth_frame.getFrame())
        if rgb.dtype != np.uint8:
            rgb = rgb.astype(np.uint8, copy=False)
        if depth.dtype != np.uint16:
            depth = depth.astype(np.uint16, copy=False)
        if rgb.shape != (self._height, self._width, 3):
            raise RuntimeError(
                f"DepthAI RGB shape {rgb.shape!r} does not match "
                f"{self._height}x{self._width}"
            )
        if depth.shape != (self._height, self._width):
            raise RuntimeError(
                f"DepthAI depth shape {depth.shape!r} does not match "
                f"{self._height}x{self._width}"
            )
        return CameraFrame(rgb=rgb, depth=depth)

    def close(self) -> None:
        with self._close_lock:
            if self._closed:
                return
            self._closed = True
            pipeline, self._pipeline = self._pipeline, None
            device, self._device = self._device, None
            self._queue = None
            if pipeline is not None:
                try:
                    pipeline.stop()
                except RuntimeError:
                    pass
            if device is not None:
                device.close()
