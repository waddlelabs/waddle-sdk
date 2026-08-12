"""Vendor-neutral camera capture and timestamped sample contracts."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Protocol, runtime_checkable

import numpy as np

if TYPE_CHECKING:
    from .. import SessionStamp
    from ..descriptors import Intrinsics

__all__ = ["CameraDriver", "CameraFrame", "CameraSample"]


def _frozen_rgb(value: object) -> np.ndarray:
    array = np.asarray(value)
    if array.dtype != np.uint8 or array.ndim != 3 or array.shape[2] != 3:
        raise TypeError("camera RGB must be a uint8 array shaped (height, width, 3)")
    if not array.flags.c_contiguous or array.flags.writeable:
        array = np.array(array, dtype=np.uint8, order="C", copy=True)
        array.setflags(write=False)
    return array


def _frozen_depth(
    value: object | None, rgb_shape: tuple[int, ...]
) -> np.ndarray | None:
    if value is None:
        return None
    array = np.asarray(value)
    if array.dtype != np.uint16 or array.ndim != 2:
        raise TypeError("camera depth must be a uint16 array shaped (height, width)")
    if array.shape != rgb_shape[:2]:
        raise ValueError(
            "camera RGB and depth must be pixel-aligned with the same height and width"
        )
    if not array.flags.c_contiguous or array.flags.writeable:
        array = np.array(array, dtype=np.uint16, order="C", copy=True)
        array.setflags(write=False)
    return array


@dataclass(frozen=True, eq=False)
class CameraFrame:
    """One RGB or pixel-aligned RGB-D capture before session timestamping.

    Drivers return this from :meth:`CameraDriver.capture`.  Arrays are copied
    when needed and made read-only so a vendor buffer reused for the next
    capture cannot rewrite a sample already handed to the rig.
    """

    rgb: np.ndarray
    depth: np.ndarray | None = None

    def __post_init__(self) -> None:
        rgb = _frozen_rgb(self.rgb)
        object.__setattr__(self, "rgb", rgb)
        object.__setattr__(self, "depth", _frozen_depth(self.depth, rgb.shape))


@runtime_checkable
class CameraDriver(Protocol):
    """Structural camera driver: capture one aligned frame and close.

    ``capture`` may block until the device has a frame.  ``close`` must be
    idempotent and unblock a pending capture so rig shutdown can join its
    capture pump deterministically.
    """

    def capture(self) -> CameraFrame: ...

    def close(self) -> None: ...


@dataclass(frozen=True, eq=False)
class CameraSample:
    """An immutable RGB or RGB-D sample on the session's paired clocks.

    ``stamp`` is minted once by ``Session.stamp()`` immediately after capture;
    its session-monotonic and Unix twins therefore remain an atomic pair.
    Depth is aligned to RGB and remains local to the owning rig.
    """

    stamp: SessionStamp
    rgb: np.ndarray
    depth: np.ndarray | None = None
    frame_sequence: int = 0

    def __post_init__(self) -> None:
        session_ns = getattr(self.stamp, "session_ns", None)
        unix_ns = getattr(self.stamp, "unix_ns", None)
        if (
            isinstance(session_ns, bool)
            or not isinstance(session_ns, int)
            or session_ns < 0
            or isinstance(unix_ns, bool)
            or not isinstance(unix_ns, int)
            or unix_ns <= 0
        ):
            raise TypeError("CameraSample.stamp must be a paired waddle.SessionStamp")
        if (
            isinstance(self.frame_sequence, bool)
            or not isinstance(self.frame_sequence, int)
            or self.frame_sequence < 0
        ):
            raise TypeError(
                "CameraSample.frame_sequence must be a non-negative integer"
            )
        rgb = _frozen_rgb(self.rgb)
        object.__setattr__(self, "rgb", rgb)
        object.__setattr__(self, "depth", _frozen_depth(self.depth, rgb.shape))

    @property
    def session_ns(self) -> int:
        return self.stamp.session_ns

    @property
    def unix_ns(self) -> int:
        return self.stamp.unix_ns

    def point_at(
        self, x: int, y: int, intrinsics: Intrinsics
    ) -> tuple[float, float, float]:
        """Resolve one RGB pixel against this sample's aligned local depth.

        Returns ``(x, y, z)`` metres in the camera frame.  No image or depth
        bytes leave this process.  The compact result is suitable for a later
        bounded calibration-measurement message, but this method itself sends
        nothing.

        The local resolver implements the pinhole model.  Non-zero distortion
        requires a vendor-specific deprojection and is refused rather than
        silently treated as rectified.
        """

        if self.depth is None:
            raise ValueError("this camera sample has no aligned depth")
        if isinstance(x, bool) or not isinstance(x, int):
            raise TypeError("pixel x must be an integer")
        if isinstance(y, bool) or not isinstance(y, int):
            raise TypeError("pixel y must be an integer")
        height, width = self.depth.shape
        if x < 0 or x >= width or y < 0 or y >= height:
            raise ValueError(f"pixel ({x}, {y}) is outside the {width}x{height} frame")

        try:
            fx = float(intrinsics.fx)
            fy = float(intrinsics.fy)
            cx = float(intrinsics.cx)
            cy = float(intrinsics.cy)
            scale_mm = float(intrinsics.depth_scale_mm)
            distortion = tuple(float(value) for value in intrinsics.distortion)
        except (AttributeError, TypeError, ValueError) as exc:
            raise TypeError(
                "intrinsics must declare finite fx/fy/cx/cy and depth_scale_mm"
            ) from exc
        if not all(math.isfinite(value) for value in (fx, fy, cx, cy, scale_mm)):
            raise ValueError("camera intrinsics and depth scale must be finite")
        if fx <= 0.0 or fy <= 0.0 or scale_mm <= 0.0:
            raise ValueError("camera fx, fy, and depth_scale_mm must be > 0")
        if any(value != 0.0 for value in distortion):
            raise ValueError(
                "local pixel resolution requires rectified depth; non-zero distortion "
                "needs the camera vendor's deprojection"
            )

        raw_depth = int(self.depth[y, x])
        if raw_depth == 0:
            raise ValueError(f"pixel ({x}, {y}) has no valid depth")
        z = raw_depth * scale_mm / 1000.0
        return ((x - cx) * z / fx, (y - cy) * z / fy, z)
