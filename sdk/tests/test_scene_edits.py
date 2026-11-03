"""Native edits preserve the actual running world, not authored initial state."""

from copy import deepcopy

import numpy as np
import pytest
from waddle_sdk.simulators.mujoco import Engine
from waddle_sdk.simulators.playground import compile_scene
from waddle_sdk.simulators.scene import load_scene, make_site


def test_live_contract_remains_bound_to_the_loaded_source_snapshot(tmp_path):
    """Retiring old authoring files cannot change an already opened robot."""
    pytest.importorskip("mujoco")
    _, config = make_site(
        "snapshot", backend="mujoco", robot="yam", environment="two_cubes"
    )
    scene = {"id": "task", "name": "Task", "objects": []}
    first, second = tmp_path / "first", tmp_path / "second"
    for folder in (first, second):
        compile_scene(config, scene, folder)
    old = load_scene(first, "simulation.json")[1]
    new = load_scene(second, "simulation.json")[1]
    engine = Engine(old, first)
    replacement = None
    try:
        engine.data.time = 42
        # The open world owns the original validated bytes, independently of
        # later changes to the old source. New target bytes are still validated.
        (first / "world.xml").unlink()
        replacement = engine.replacement(new, second, preserve_state=True)
        assert replacement.data.time == 42
        # A real robot-contract change in a newly validated publication refuses.
        import hashlib

        path = second / "world.xml"
        # Change a drive rather than relying on a particular XML attribute order.
        import xml.etree.ElementTree as ET

        tree = ET.fromstring(path.read_bytes())
        motor = tree.find("actuator")[0]
        motor.set("forcerange", "-1 1")
        data = ET.tostring(tree)
        path.write_bytes(data)
        changed = deepcopy(new)
        changed["native_scene"]["files"]["world.xml"] = hashlib.sha256(data).hexdigest()
        with pytest.raises(ValueError, match="scene_contract"):
            replacement.replacement(changed, second, preserve_state=True)
        # A digest mismatch must still fail before a new scene is accepted.
        with pytest.raises(ValueError, match="digest differs"):
            replacement.replacement(new, second, preserve_state=True)
    finally:
        if replacement is not None:
            replacement.close()
        engine.close()


def test_add_resize_move_rotate_delete_preserve_unedited_live_state(tmp_path):
    pytest.importorskip("mujoco")
    _, config = make_site(
        "editing", backend="mujoco", robot="yam", environment="two_cubes"
    )
    a = {
        "id": "a",
        "kind": "block",
        "position": [0.3, -0.1, 0.03],
        "rotation": [0, 0, 0],
        "size": [0.04] * 3,
        "color": [1, 0, 0],
    }
    b = dict(deepcopy(a), id="b", position=[0.4, 0.1, 0.03])
    scene = {"id": "task", "name": "Task", "objects": [a]}

    def engine_for(index):
        root = tmp_path / str(index)
        compile_scene(config, scene, root, revision=index)
        return load_scene(root, "simulation.json")[1], root

    definition, root = engine_for(1)
    engine = Engine(definition, root)
    try:
        robot = engine.read("arm")[0].copy()
        robot[0] += 0.2
        engine.home("arm", robot)
        engine.data.joint("prop_a_free").qpos[:] = [
            0.5,
            -0.2,
            0.2,
            0.9238795325,
            0,
            0,
            0.3826834324,
        ]
        engine.data.joint("prop_a_free").qvel[:] = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        engine.data.time = 12.5
        expected_pose = engine.data.joint("prop_a_free").qpos.copy()
        expected_velocity = engine.data.joint("prop_a_free").qvel.copy()
        controls = engine.data.ctrl.copy()
        for index, edit in enumerate(("add", "resize", "move", "rotate", "delete"), 2):
            if edit == "add":
                scene["objects"].append(b)
            elif edit == "resize":
                a["size"] = [0.06] * 3
                a["color"] = [0, 1, 0]
            elif edit == "move":
                a["position"] = [0.2, -0.3, 0.15]
                expected_pose[:3] = a["position"]
                expected_velocity[:] = 0
            elif edit == "rotate":
                a["rotation"] = [0, 0, 90]
                expected_pose[3:] = [2**-0.5, 0, 0, 2**-0.5]
            elif edit == "delete":
                scene["objects"].remove(b)
            definition, root = engine_for(index)
            replacement = engine.replacement(definition, root, preserve_state=True)
            engine.close()
            engine = replacement
            np.testing.assert_allclose(engine.read("arm")[0], robot, atol=1e-12)
            np.testing.assert_array_equal(engine.data.ctrl, controls)
            np.testing.assert_allclose(
                engine.data.joint("prop_a_free").qpos, expected_pose, atol=1e-9
            )
            np.testing.assert_allclose(
                engine.data.joint("prop_a_free").qvel, expected_velocity
            )
            assert engine.data.time == 12.5
            assert ("prop_b" in engine.evaluation_snapshot()["bodies"]) == (
                edit != "delete"
            )
        engine.reset()
        np.testing.assert_allclose(
            engine.data.joint("prop_a_free").qpos[:3], a["position"]
        )
        assert engine.data.time == 0
    finally:
        engine.close()
