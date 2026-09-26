"""Metric depth follows declared pixel-centre rays, independently of RGB AA."""

from __future__ import annotations

from dataclasses import asdict

import numpy as np
import pytest
from waddle_sdk.cameras.site import CameraConfig
from waddle_sdk.robots.mujoco import MujocoBackend
from waddle_sdk.simulators.mujoco import Engine


@pytest.mark.parametrize("samples", [0, 4])
@pytest.mark.parametrize("adapter", ["modular", "reference"])
def test_native_depth_matches_analytic_plane_without_changing_rgb(
    tmp_path, samples, adapter
):
    mj = pytest.importorskip("mujoco")
    height, width = 120, 160
    normal = np.array([0.35, 0.25, 1.0])
    normal /= np.linalg.norm(normal)
    w = np.sqrt((1 + normal[2]) / 2)
    quaternion = [w, -normal[1] / (2 * w), normal[0] / (2 * w), 0]
    path = tmp_path / "camera.xml"
    path.write_text(
        '<mujoco><statistic extent="1"/>'
        f'<visual><quality offsamples="{samples}"/><map znear=".01" zfar="3"/></visual>'
        '<worldbody><light pos="0 0 3"/><camera name="view" pos="0 0 1" fovy="50"/>'
        f'<geom type="plane" size="5 5 .1" quat="{" ".join(map(str, quaternion))}"/>'
        '<geom type="box" pos=".18 0 .2" size=".055 .065 .025" rgba="1 0 0 1"/>'
        "</worldbody></mujoco>"
    )
    world = MujocoBackend(model_path=path)
    world.open()
    driver = world.camera(
        config=CameraConfig(
            name="view",
            connection={"camera": "view"},
            stream={"width": width, "height": height},
            frame_id="view",
            intrinsics=None,
            options={"depth_scale_mm": 0.1},
        )
    )
    # Exercise the reference engine's actual capture implementation against the
    # same tiny native model, without unrelated robot assets or task semantics.
    engine = Engine.__new__(Engine)
    engine.mj, engine.model, engine.data = mj, world.model, world.data
    engine.renderers = {}
    engine.config = {
        "cameras": {
            "view": {
                "stream": {"width": width, "height": height},
                "options": {"depth": True},
                "intrinsics": asdict(driver.intrinsics()),
            }
        }
    }
    initial = world.data.qpos.copy()
    try:
        with mj.Renderer(world.model, height=height, width=width) as reference:
            reference.update_scene(world.data, camera="view")
            expected_rgb = reference.render().copy()
        if adapter == "modular":
            frame = driver.capture()
            rgb, raw, timing = frame.rgb, frame.depth, asdict(frame.content_timing)
        else:
            rgb, raw, timing = engine.capture_timed("view")
        np.testing.assert_array_equal(rgb, expected_rgb)
        assert world.model.vis.quality.offsamples == samples
        np.testing.assert_array_equal(world.data.qpos, initial)
        intrinsics = driver.intrinsics()
        yy, xx = np.mgrid[10 : height - 10, 10 : width // 3]
        # Camera looks down -Z with +Y image coordinates pointing down.
        rays = np.stack(
            (
                (xx - intrinsics.cx) / intrinsics.fx,
                -(yy - intrinsics.cy) / intrinsics.fy,
                -np.ones_like(xx),
            ),
            axis=-1,
        )
        expected_depth = -normal[2] / (rays @ normal)
        measured = raw[yy, xx].astype(float) * 0.0001
        error = abs(measured - expected_depth)
        # 0.1-mm quantization plus native depth-buffer arithmetic.
        assert float(error.max()) < 0.00008, {
            "maximum_error_m": float(error.max()),
            "samples": samples,
            "adapter": adapter,
        }
        assert timing["kind"] == "simulated_state"
        assert timing["rgb_monotonic_ns"] == timing["depth_monotonic_ns"]
    finally:
        engine.close()
        driver.close()
        world.close()
