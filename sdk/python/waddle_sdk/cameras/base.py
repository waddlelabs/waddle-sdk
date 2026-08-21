"""Vendor-neutral camera capture and timestamped sample contracts."""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, field
from functools import lru_cache
from typing import TYPE_CHECKING, Protocol, runtime_checkable

import numpy as np

if TYPE_CHECKING:
    from .. import SessionStamp
    from ..descriptors import Intrinsics

__all__ = ["CameraCalibrationDriver", "CameraDriver", "CameraFrame", "CameraSample"]

_DEPTH_PREVIEW_NEAR_M = 0.15
_DEPTH_PREVIEW_FAR_M = 3.0


def _depth_color_ramp(normalized: np.ndarray) -> np.ndarray:
    x = np.clip(normalized, 0.0, 1.0)
    return np.stack(
        (
            np.clip(1.5 - np.abs(4.0 * x - 3.0), 0.0, 1.0),
            np.clip(1.5 - np.abs(4.0 * x - 2.0), 0.0, 1.0),
            np.clip(1.5 - np.abs(4.0 * x - 1.0), 0.0, 1.0),
        ),
        axis=-1,
    )


@lru_cache(maxsize=16)
def _metric_depth_preview_lut(depth_scale_mm: float) -> np.ndarray:
    """Build one small immutable Z16→RGB lookup table per camera scale."""

    raw = np.arange(1 << 16, dtype=np.float32)
    metres = raw * np.float32(depth_scale_mm / 1000.0)
    normalized = (metres - _DEPTH_PREVIEW_NEAR_M) / (
        _DEPTH_PREVIEW_FAR_M - _DEPTH_PREVIEW_NEAR_M
    )
    table = np.ascontiguousarray(
        np.rint(_depth_color_ramp(normalized) * 255.0),
        dtype=np.uint8,
    )
    table[0] = 0
    table.setflags(write=False)
    return table


def _integral_fps(value: object) -> int:
    """Return the whole-frame rate required by built-in camera vendor APIs."""

    if isinstance(value, bool):
        raise TypeError("camera fps must be a positive whole number")
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise TypeError("camera fps must be a positive whole number") from error
    if not math.isfinite(number) or number <= 0.0 or not number.is_integer():
        raise ValueError("camera fps must be a positive whole number")
    return int(number)


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


def _depth_preview_rgb(
    depth: np.ndarray,
    depth_scale_mm: float | None,
) -> np.ndarray:
    """Colorize one metric Z16 plane for an ordinary browser video track.

    This never replaces or mutates the raw aligned depth sample. When the
    active camera declares its unit scale, colors use a stable 0.15–3.0 m
    range (near blue, far red) so successive frames remain comparable. A
    custom RGB-D driver without scale metadata degrades to robust per-frame
    percentiles; zero/no-return pixels are always black.
    """

    valid = depth != 0
    if not bool(np.any(valid)):
        return np.zeros((*depth.shape, 3), dtype=np.uint8)

    scale = None
    if depth_scale_mm is not None:
        try:
            candidate = float(depth_scale_mm)
        except (TypeError, ValueError):
            candidate = math.nan
        if math.isfinite(candidate) and candidate > 0.0:
            scale = candidate

    values = depth.astype(np.float32, copy=False)
    if scale is not None:
        # Advanced indexing allocates only the final H×W×3 preview. The
        # expensive color-ramp arithmetic is paid once per camera unit scale,
        # not once per captured frame on the camera thread.
        return np.ascontiguousarray(_metric_depth_preview_lut(scale)[depth])
    else:
        present = values[valid]
        near, far = np.percentile(present, (2.0, 98.0))
        if not math.isfinite(float(near)) or not math.isfinite(float(far)):
            near, far = float(np.min(present)), float(np.max(present))
        if far <= near:
            normalized = np.full(depth.shape, 0.5, dtype=np.float32)
        else:
            normalized = (values - np.float32(near)) / np.float32(far - near)

    preview = _depth_color_ramp(normalized)
    colored = np.ascontiguousarray(np.rint(preview * 255.0), dtype=np.uint8)
    colored[~valid] = 0
    return colored


@dataclass(frozen=True, eq=False)
class CameraFrame:
    """One RGB or pixel-aligned RGB-D capture before session timestamping.

    Drivers return this from :meth:`CameraDriver.capture`.  Arrays are copied
    when needed and made read-only so a vendor buffer reused for the next
    capture cannot rewrite a sample already handed to the rig.
    """

    rgb: np.ndarray
    depth: np.ndarray | None = None
    point_resolver: Callable[[int, int, float], tuple[float, float, float]] | None = (
        field(
            default=None,
            repr=False,
            compare=False,
        )
    )

    def __post_init__(self) -> None:
        rgb = _frozen_rgb(self.rgb)
        object.__setattr__(self, "rgb", rgb)
        object.__setattr__(self, "depth", _frozen_depth(self.depth, rgb.shape))
        if self.point_resolver is not None and not callable(self.point_resolver):
            raise TypeError("camera point_resolver must be callable")


@runtime_checkable
class CameraDriver(Protocol):
    """Structural camera driver: capture one aligned frame and close.

    ``capture`` may block until the device has a frame.  ``close`` must be
    idempotent and unblock a pending capture so rig shutdown can join its
    capture pump deterministically.
    """

    def capture(self) -> CameraFrame: ...

    def close(self) -> None: ...


@runtime_checkable
class CameraCalibrationDriver(Protocol):
    """Optional live calibration extension for camera drivers.

    A driver that can read the active stream profile exposes its aligned color
    intrinsics here.  The SDK folds those facts into the registered
    camera declaration before transport starts.  Drivers without this
    extension remain valid and use the explicit ``site.yaml`` intrinsics.
    """

    def intrinsics(self) -> Intrinsics: ...


@dataclass(frozen=True, eq=False)
class CameraSample:
    """An immutable RGB or RGB-D sample on the session's paired clocks.

    ``stamp`` is minted once by ``Session.stamp()`` immediately after capture;
    its session-monotonic and Unix twins therefore remain an atomic pair.
    Raw metric depth is aligned to RGB and remains local to the owning rig.
    A derived RGB8 visualization may independently ride the media plane.
    """

    stamp: SessionStamp
    rgb: np.ndarray
    depth: np.ndarray | None = None
    frame_sequence: int = 0
    point_resolver: Callable[[int, int, float], tuple[float, float, float]] | None = (
        field(
            default=None,
            repr=False,
            compare=False,
        )
    )

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
            raise TypeError(
                "CameraSample.stamp must be a paired waddle_sdk.SessionStamp"
            )
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
        if self.point_resolver is not None and not callable(self.point_resolver):
            raise TypeError("camera point_resolver must be callable")

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

        The local fallback implements the pinhole model. A driver may attach a
        vendor-owned resolver to the captured frame for non-zero distortion;
        otherwise distorted samples are refused rather than treated as rectified.
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
        if self.point_resolver is None and any(value != 0.0 for value in distortion):
            raise ValueError(
                "local pixel resolution requires rectified depth; non-zero distortion "
                "needs the camera vendor's deprojection"
            )
        raw_depth = int(self.depth[y, x])
        if raw_depth == 0:
            raise ValueError(f"pixel ({x}, {y}) has no valid depth")
        z = raw_depth * scale_mm / 1000.0
        if self.point_resolver is not None:
            point = tuple(float(value) for value in self.point_resolver(x, y, z))
            if len(point) != 3 or not all(math.isfinite(value) for value in point):
                raise ValueError("camera point resolver returned a malformed xyz point")
            if point[2] <= 0.0 or not math.isclose(
                point[2], z, rel_tol=1e-3, abs_tol=1e-6
            ):
                raise ValueError("camera point resolver returned inconsistent depth")
            return point
        return ((x - cx) * z / fx, (y - cy) * z / fy, z)
