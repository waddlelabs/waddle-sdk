"""Native RGB appearance plus depth on calibrated pixel-centre rays.

Construct and capture on the owning render thread, with the caller's model/state
lock held. Reference worker engines already serialize these operations.
"""

from __future__ import annotations

import time

import numpy as np

from .timing import CameraContentTiming


class RGBDRenderer:
    """Keep configured RGB multisampling out of the metric depth resolve."""

    def __init__(self, mj, model, *, height: int, width: int, depth: bool):
        self.rgb = mj.Renderer(model, height=height, width=width)
        self.depth = self.rgb if depth else None
        self.closed = False
        samples = model.vis.quality.offsamples
        if depth and samples:
            # MuJoCo fixes the framebuffer sample count when constructing its
            # rendering context. Restore the model even if context creation fails.
            # The caller holds its world lock, so other cameras cannot inherit it.
            try:
                model.vis.quality.offsamples = 0
                self.depth = mj.Renderer(model, height=height, width=width)
            except BaseException:
                self.close()
                raise
            finally:
                model.vis.quality.offsamples = samples

    def capture(self, data, *, camera, scene_option=None):
        if self.closed:
            raise RuntimeError("MuJoCo RGB-D renderer is closed")
        options = {} if scene_option is None else {"scene_option": scene_option}
        began = time.monotonic_ns()
        self.rgb.update_scene(data, camera=camera, **options)
        if self.depth is not None and self.depth is not self.rgb:
            self.depth.update_scene(data, camera=camera, **options)
        ended = time.monotonic_ns()
        self.rgb.disable_depth_rendering()
        rgb = np.array(self.rgb.render(), dtype=np.uint8, order="C", copy=True)
        depth = None
        if self.depth is not None:
            self.depth.enable_depth_rendering()
            try:
                depth = self.depth.render().copy()
            finally:
                self.depth.disable_depth_rendering()
        return (
            rgb,
            depth,
            CameraContentTiming(
                kind="simulated_state",
                clock_revision="local-host-monotonic/v1",
                rgb_monotonic_ns=(began, ended),
                depth_monotonic_ns=(began, ended) if depth is not None else None,
            ),
        )

    def rebind_model(self, model):
        """Rebind on the owning thread after verifying identical GL resources.

        The next capture rebuilds each visual scene from the new model/data;
        only uploaded assets and framebuffers survive the revision.
        """
        if self.closed:
            raise RuntimeError("MuJoCo RGB-D renderer is closed")
        self.rgb._model = model
        if self.depth is not None:
            self.depth._model = model

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            if self.depth is not None and self.depth is not self.rgb:
                self.depth.close()
        finally:
            self.rgb.close()


def same_render_resources(first, second):
    """Conservatively compare native resources baked into a rendering context."""
    sizes = ("nmesh", "ntex", "nhfield", "nskin", "nflex")
    if any(getattr(first, name) != getattr(second, name) for name in sizes):
        return False
    if (
        _settings(first.vis) != _settings(second.vis)
        or first.stat.extent != second.stat.extent
    ):
        return False
    # Planes use display lists tied to their order, size and material. Other
    # primitive geometry is rebuilt by update_scene on every capture.
    planes = [np.flatnonzero(model.geom_type == 0) for model in (first, second)]
    if not np.array_equal(*planes):
        return False
    for name in ("geom_size", "geom_matid"):
        if not np.array_equal(
            getattr(first, name)[planes[0]], getattr(second, name)[planes[1]]
        ):
            return False
    for name in dir(first):
        if name.startswith(("mesh_", "tex_", "hfield_", "skin_", "flex_", "mat_")):
            if name in {"mesh_bvhadr", "mesh_bvhnum"}:
                continue  # Collision BVH offsets are not uploaded render assets.
            a, b = getattr(first, name), getattr(second, name)
            if isinstance(a, np.ndarray) and not np.array_equal(a, b):
                return False
    return True


def _settings(value):
    if isinstance(value, np.ndarray):
        return value.dtype.str, value.shape, value.tobytes()
    if isinstance(value, (int, float, str, np.integer, np.floating)):
        return value
    return tuple(
        (name, _settings(getattr(value, name)))
        for name in dir(value)
        if not name.startswith("_") and not callable(getattr(value, name))
    )
