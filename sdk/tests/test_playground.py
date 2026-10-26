"""Authored scenes retain reference feedback, camera and reset behavior."""

from copy import deepcopy

import numpy as np
import pytest
from waddle_sdk.simulators.playground import CATALOG, compile_scene, validate_scene
from waddle_sdk.simulators.scene import load_scene, make_site


def scene():
    return {
        "id": "test",
        "name": "Test scene",
        "objects": [
            {
                "id": kind,
                "kind": kind,
                "position": [0.3, -0.2 + 0.09 * i, row["size"][2] / 2 + 0.001],
                "rotation": [0, 0, 0],
                "size": row["size"],
                "color": [0.3, 0.6, 0.8],
            }
            for i, (kind, row) in enumerate(CATALOG.items())
        ],
    }


@pytest.mark.parametrize("robot", ["yam", "so101", "xarm7"])
@pytest.mark.parametrize("arms", [1, 2])
def test_compiled_scene_preserves_robot_camera_and_canonical_reset(
    tmp_path, robot, arms
):
    pytest.importorskip("mujoco")
    from waddle_sdk.simulators.mujoco import Engine

    _, config = make_site(
        "editable",
        backend="mujoco",
        robot=robot,
        environment="two_cubes",
        arms=arms,
        width=64,
        height=48,
    )
    output = tmp_path / "world"
    compile_scene(config, scene(), output)
    _, compiled = load_scene(output, "simulation.json")
    engine = Engine(compiled, output)
    try:
        original = engine.data.qpos.copy()
        for part in engine.parts:
            np.testing.assert_allclose(
                engine.read(part)[0], engine.profile.home, atol=1e-8
            )
        for _ in range(100):
            engine.step()
        for camera in config["cameras"]:
            rgb, depth = engine.capture(camera)
            assert rgb.shape == (48, 64, 3) and np.ptp(rgb) > 20
            if config["cameras"][camera]["options"]["depth"]:
                assert depth.shape == (48, 64) and (depth > 0).any()
            else:
                assert depth is None
        engine.reset()
        np.testing.assert_array_equal(engine.data.qpos, original)
        assert not engine._force_modes
    finally:
        engine.close()


def test_editing_creates_new_bytes_and_preserves_original_scene(tmp_path):
    pytest.importorskip("mujoco")
    _, config = make_site(
        "editable", backend="mujoco", robot="yam", environment="two_cubes"
    )
    first = compile_scene(config, scene(), tmp_path / "first")
    original = (tmp_path / "first/world.xml").read_bytes()
    changed = scene()
    changed["objects"][0]["position"][0] += 0.04
    second = compile_scene(config, changed, tmp_path / "second", revision=2)
    assert (
        second["native_scene"]["files"]["world.xml"]
        != first["native_scene"]["files"]["world.xml"]
    )
    assert (tmp_path / "first/world.xml").read_bytes() == original
    with pytest.raises(ValueError, match="already exists"):
        compile_scene(config, changed, tmp_path / "first")


@pytest.mark.parametrize(
    "mutation",
    [
        lambda s: s["objects"][0].update(id="../escape"),
        lambda s: s["objects"][0].update(position=[float("nan"), 0, 0]),
        lambda s: s["objects"][0].update(size=[-1, 1, 1]),
        lambda s: s["objects"][0].update(kind="executable.py"),
        lambda s: s["objects"].append(deepcopy(s["objects"][0])),
    ],
)
def test_scene_rejects_invalid_geometry_before_compiling(mutation):
    value = scene()
    mutation(value)
    with pytest.raises(ValueError, match="playground"):
        validate_scene(value)
