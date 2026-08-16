"""Deterministic RGB/RGB-D camera for SDK simulation and tests."""

from __future__ import annotations

import threading

import numpy as np

from .base import CameraFrame

__all__ = ["MockDriver"]


class MockDriver:
    """A mutable synthetic scene with one bright-green circular object."""

    def __init__(
        self,
        *,
        width: int = 640,
        height: int = 480,
        fps: float = 30,
        has_depth: bool = True,
        depth_scale_mm: float = 1.0,
        background_depth_m: float = 1.0,
        object_depth_m: float = 0.4,
        object_u: int | None = None,
        object_v: int | None = None,
        object_radius_px: int = 30,
    ) -> None:
        del fps
        if width <= 0 or height <= 0:
            raise ValueError("mock camera width and height must be positive")
        if depth_scale_mm <= 0.0:
            raise ValueError("mock camera depth_scale_mm must be positive")
        self.width = int(width)
        self.height = int(height)
        self.has_depth = bool(has_depth)
        self.depth_scale_mm = float(depth_scale_mm)
        self.background_depth_m = float(background_depth_m)
        self.object_depth_m = float(object_depth_m)
        self.object_u = self.width // 2 if object_u is None else int(object_u)
        self.object_v = self.height // 2 if object_v is None else int(object_v)
        self.object_radius_px = int(object_radius_px)
        self._lock = threading.Lock()
        self._closed = False

    def set_object(
        self,
        *,
        u: int,
        v: int,
        depth_m: float | None = None,
        radius_px: int | None = None,
    ) -> None:
        with self._lock:
            self.object_u = int(u)
            self.object_v = int(v)
            if depth_m is not None:
                self.object_depth_m = float(depth_m)
            if radius_px is not None:
                self.object_radius_px = int(radius_px)

    def capture(self) -> CameraFrame:
        with self._lock:
            if self._closed:
                raise RuntimeError("mock camera is closed")
            u = self.object_u
            v = self.object_v
            radius = self.object_radius_px
            object_depth_m = self.object_depth_m
            background_depth_m = self.background_depth_m
        yy, xx = np.ogrid[: self.height, : self.width]
        mask = (xx - u) ** 2 + (yy - v) ** 2 <= radius**2
        rgb = np.full((self.height, self.width, 3), 60, dtype=np.uint8)
        rgb[mask] = np.array([0, 220, 0], dtype=np.uint8)
        if not self.has_depth:
            return CameraFrame(rgb=rgb)
        depth = np.full(
            (self.height, self.width),
            round(background_depth_m * 1000.0 / self.depth_scale_mm),
            dtype=np.uint16,
        )
        depth[mask] = round(object_depth_m * 1000.0 / self.depth_scale_mm)
        return CameraFrame(rgb=rgb, depth=depth)

    def close(self) -> None:
        with self._lock:
            self._closed = True
