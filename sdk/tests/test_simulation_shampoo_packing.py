"""Native pile stability and initial robot clearance for every reference family."""

import numpy as np
import pytest
from waddle_sdk.simulators.scene import make_site


@pytest.mark.parametrize("robot", ["yam", "so101", "xarm7"])
def test_gravity_pile_is_stable_varied_and_inside_box(tmp_path, robot):
    pytest.importorskip("mujoco")
    from waddle_sdk.simulators.mujoco import Engine

    _, config = make_site(
        "shampoo-pile",
        backend="mujoco",
        robot=robot,
        environment="shampoo-packing",
        width=320,
        height=240,
    )
    engine = Engine(config, tmp_path)
    try:
        names = [f"shampoo_{i:02d}" for i in range(1, 21)]
        initial = np.array([engine.data.body(n).xpos.copy() for n in names])
        tilts = [np.arccos(abs(engine.data.body(n).xmat[8])) for n in names]
        assert sum(t > 1.3 for t in tilts) >= 4
        assert sum(0.3 < t < 1.3 for t in tilts) >= 4
        for _ in range(round(3 / engine.model.opt.timestep)):
            engine.step()
            for c in engine.data.contact:
                bodies = [
                    engine.model.body(int(engine.model.geom_bodyid[g])).name
                    for g in c.geom
                ]
                robot_contact = any(
                    n not in engine._prop_links and n not in ("table", "world")
                    for n in bodies
                )
                assert not (
                    robot_contact
                    and any(n in engine._prop_links or n == "table" for n in bodies)
                    and c.dist < -0.0001
                ), bodies
        positions = np.array([engine.data.body(n).xpos for n in names])
        assert np.max(np.linalg.norm(positions - initial, axis=1)) < 0.003
        assert np.all(np.abs(positions[:, :2] - [0.29, -0.20]) < [0.124, 0.114])
        assert np.all((positions[:, 2] > 0.02) & (positions[:, 2] < 0.15))
        for n in names:
            j = engine.model.joint(int(engine.model.body(n).jntadr[0]))
            v = int(j.dofadr[0])
            assert np.linalg.norm(engine.data.qvel[v : v + 3]) < 0.002
    finally:
        engine.close()


@pytest.mark.parametrize("slot_index", range(1, 7))
def test_tilted_bottle_is_retained_rotated_carried_and_seated(tmp_path, slot_index):
    import json
    from pathlib import Path

    mujoco = pytest.importorskip("mujoco")
    from waddle_sdk.simulators.mujoco import Engine

    fixture_name = (
        "shampoo_transport.json"
        if slot_index == 1
        else f"shampoo_transport_slot{slot_index}.json"
    )
    fixture = json.loads((Path(__file__).parent / "fixtures" / fixture_name).read_text())
    _, config = make_site(
        "shampoo-transport",
        backend="mujoco",
        robot="yam",
        environment="shampoo-packing",
        width=320,
        height=240,
    )
    engine = Engine(config, tmp_path)
    target = fixture["target"]
    force = np.zeros(6)
    fingers = {"tip_left", "tip_right"}
    relative_reference = None
    try:
        start = np.array(fixture["initial"])
        initial_z = float(engine.data.body(target).xpos[2])
        # Only initialize the open robot approach. Bottle state is never written.
        engine.home(start)
        for motion in fixture["motions"]:
            end = np.array(motion["end"])
            steps = round(motion["seconds"] / engine.model.opt.timestep)
            for step in range(steps):
                t = (step + 1) / steps
                smooth = t**3 * (10 - 15 * t + 6 * t * t)
                engine.write(start + smooth * (end - start))
                engine.step()
                if not motion["check"] or step % 10:
                    continue
                touched = set()
                for i, c in enumerate(engine.data.contact):
                    names = {
                        engine.model.body(int(engine.model.geom_bodyid[g])).name
                        for g in c.geom
                    }
                    if target in names:
                        mujoco.mj_contactForce(engine.model, engine.data, i, force)
                        if force[0] > 0.01:
                            touched.update(names & fingers)
                assert touched == fingers
                bottle = engine.data.body(target)
                assert bottle.xpos[2] > initial_z + 0.03
                tcp = engine.data.site("tcp_site")
                relative = tcp.xmat.reshape(3, 3).T @ (bottle.xpos - tcp.xpos)
                if relative_reference is None:
                    relative_reference = relative.copy()
                assert np.linalg.norm(relative - relative_reference) < 0.003
            start = end
        bottle = engine.data.body(target)
        slot = engine.data.body(f"shampoo_slot_{slot_index}")
        assert np.linalg.norm(bottle.xpos[:2] - slot.xpos[:2]) < 0.0035
        assert 0.050 < bottle.xpos[2] - slot.xpos[2] < 0.057
        assert np.arccos(abs(bottle.xmat[8])) < 0.2
        support = 0.0
        for i, c in enumerate(engine.data.contact):
            names = {
                engine.model.body(int(engine.model.geom_bodyid[g])).name for g in c.geom
            }
            if target not in names:
                continue
            mujoco.mj_contactForce(engine.model, engine.data, i, force)
            assert not (names & fingers and force[0] > 0.01)
            if "shampoo_packing_box" in names:
                support += force[0]
        assert support > 0.2
    finally:
        engine.close()


@pytest.mark.parametrize("offset", [(0.008, 0), (-0.008, 0), (0, 0.008), (0, -0.008)])
def test_tapered_pockets_physically_center_released_bottles(tmp_path, offset):
    pytest.importorskip("mujoco")
    from waddle_sdk.simulators.mujoco import Engine

    _, config = make_site(
        "shampoo-drop",
        backend="mujoco",
        robot="yam",
        environment="shampoo-packing",
        width=320,
        height=240,
    )
    engine = Engine(config, tmp_path)
    try:
        for index in range(1, 7):
            body = engine.model.body(f"shampoo_{index:02d}")
            slot = engine.data.body(f"shampoo_slot_{index}").xpos
            joint = engine.model.joint(int(body.jntadr[0]))
            q = int(joint.qposadr[0])
            v = int(joint.dofadr[0])
            # A dropped fixture tests physical guidance; it is not robot evidence.
            engine.data.qpos[q : q + 7] = [
                slot[0] + offset[0],
                slot[1] + offset[1],
                slot[2] + 0.14,
                1,
                0,
                0,
                0,
            ]
            engine.data.qvel[v : v + 6] = 0
        engine.mj.mj_forward(engine.model, engine.data)
        for _ in range(round(2 / engine.model.opt.timestep)):
            engine.step()
        for index in range(1, 7):
            body = engine.data.body(f"shampoo_{index:02d}")
            slot = engine.data.body(f"shampoo_slot_{index}")
            assert np.linalg.norm(body.xpos[:2] - slot.xpos[:2]) < 0.0035
            assert 0.050 < body.xpos[2] - slot.xpos[2] < 0.057
            assert np.arccos(abs(body.xmat[8])) < 0.2
            joint = engine.model.joint(int(engine.model.body(body.name).jntadr[0]))
            v = int(joint.dofadr[0])
            assert np.linalg.norm(engine.data.qvel[v : v + 3]) < 0.015
            force, support = np.zeros(6), 0.0
            for i, contact in enumerate(engine.data.contact):
                names = {
                    engine.model.body(int(engine.model.geom_bodyid[g])).name
                    for g in contact.geom
                }
                if names == {body.name, "shampoo_packing_box"}:
                    engine.mj.mj_contactForce(engine.model, engine.data, i, force)
                    support += force[0]
            assert support > 0.2
    finally:
        engine.close()
