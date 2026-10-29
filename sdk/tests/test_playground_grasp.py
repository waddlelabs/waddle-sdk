"""Easy contacts retain sound grasps but cannot glue marginal or missed ones."""

import math

import numpy as np
import pytest
from waddle_sdk.simulators.playground import compile_scene
from waddle_sdk.simulators.scene import load_scene, make_site


@pytest.mark.parametrize(
    "offset,force,retained",
    [(0, 34.21875, True), (0.055, 20, False), (0, 10, False), (0.10, 34.21875, False)],
    ids=["centered", "edge", "weak", "miss"],
)
def test_supported_grasp_survives_only_with_sufficient_contact_and_releases(
    tmp_path, offset, force, retained
):
    mj = pytest.importorskip("mujoco")
    from waddle_sdk.simulators.mujoco import Engine

    _, config = make_site(
        "grasp",
        backend="mujoco",
        robot="yam",
        environment="two_cubes",
        width=32,
        height=32,
    )
    folder = tmp_path / "scene"
    compile_scene(
        config,
        {
            "id": "grasp",
            "name": "Supported grasp",
            "objects": [
                {
                    "id": "item",
                    "kind": "bottle",
                    "position": [0.3, -0.12, 0.0455],
                    "rotation": [0, 0, 0],
                    "size": [0.04, 0.04, 0.09],
                    "color": [0.3, 0.5, 0.8],
                }
            ],
        },
        folder,
    )
    _, compiled = load_scene(folder, "simulation.json")
    engine = Engine(compiled, folder)

    def advance(seconds):
        for _ in range(round(seconds / engine.model.opt.timestep)):
            engine.step()

    def fingers():
        touched = set()
        effort = np.zeros(6)
        for index, contact in enumerate(engine.data.contact):
            names = {
                engine.model.body(int(engine.model.geom_bodyid[g])).name
                for g in contact.geom
            }
            if "prop_item" in names:
                mj.mj_contactForce(engine.model, engine.data, index, effort)
                if effort[0] > 0.01:
                    touched.update(names & {"arm__tip_left", "arm__tip_right"})
        return touched

    try:
        # Fixed raised, downward-facing hand. Fixture support balances gravity
        # only while closing; every retained/drop measurement is unsupported.
        engine.home(
            np.array(
                [
                    -0.3805207603,
                    1.7719534517,
                    1.656210454,
                    -1.4550611243,
                    -0.00000747,
                    -0.380505675,
                    0.4,
                ]
            )
        )
        advance(0.1)
        body = engine.data.body("prop_item")
        address = int(engine.model.joint("prop_item_free").qposadr[0])
        engine.data.qpos[address : address + 3] = engine.data.site(
            "arm__tcp_site"
        ).xpos + [offset, 0, 0.015]
        engine.data.qpos[address + 3 : address + 7] = [
            math.sqrt(0.5),
            0,
            math.sqrt(0.5),
            0,
        ]
        mj.mj_forward(engine.model, engine.data)
        engine.data.xfrc_applied[body.id, 2] = engine.model.body_mass[body.id] * 9.81
        target = engine.read()[0]
        target[-1] = 0
        engine.write_force(target, force)
        advance(1)
        supported = body.xpos.copy()
        assert supported[2] > 0.14
        engine.data.xfrc_applied[body.id] = 0
        advance(5)
        if retained:
            assert fingers() == {"arm__tip_left", "arm__tip_right"}
            assert np.linalg.norm(body.xpos - supported) < 0.005
            assert engine.gripper_force() == pytest.approx(force, abs=0.5)
        else:
            assert body.xpos[2] < supported[2] - 0.10
            assert not fingers()
        target = engine.read()[0]
        target[-1] = 1
        engine.write_force(target, 0)
        advance(1)
        assert not engine._force_modes
        assert engine.read()[0][-1] * engine.profile.opening > 0.09
        assert body.xpos[2] < supported[2] - 0.10
        assert not fingers()
    finally:
        engine.close()
