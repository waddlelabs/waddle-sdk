"""Native edits preserve the actual running world, not authored initial state."""

from copy import deepcopy

import numpy as np
import pytest
from waddle_sdk.simulators.mujoco import Engine
from waddle_sdk.simulators.playground import compile_scene
from waddle_sdk.simulators.scene import load_scene, make_site


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
