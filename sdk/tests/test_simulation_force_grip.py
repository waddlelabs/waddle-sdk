"""Native actuator feedback and retained force mode across contact and release."""

import json
from pathlib import Path

import numpy as np
import pytest
from waddle_sdk.simulators.scene import make_site


@pytest.mark.parametrize("empty", [False, True])
@pytest.mark.parametrize("release", ["zero", "hold"])
def test_native_force_closure_reports_actual_effort_and_retains_force(
    tmp_path, empty, release
):
    pytest.importorskip("mujoco")
    from waddle_sdk.simulators.mujoco import Engine

    fixture = next(
        row
        for row in json.loads(
            (Path(__file__).parent / "fixtures/tube_transport.json").read_text()
        )
        if row["robot"] == "yam"
    )
    _, config = make_site(
        "force",
        backend="mujoco",
        robot="yam",
        environment="load-clear-test-tubes",
        width=32,
        height=32,
    )
    engine = Engine(config, tmp_path)
    try:
        target = np.array(fixture["approach" if empty else "grasp"])
        target[-1] = 1.0
        engine.home(target)
        for _ in range(500):
            engine.step()
        target[-1] = 0.0
        engine.write_force(target, 10)
        for _ in range(2500):
            engine.step()
        opening = engine.read()[0][-1] * engine.profile.opening
        effort = engine.gripper_force()
        if empty:
            assert opening < 0.0005
        else:
            assert opening > 0.001, (opening, effort)
            assert effort == pytest.approx(10, abs=1), (opening, effort)
            # An unrelated arm write cannot turn a grasp back into a fixed jaw.
            target[0] += 0.002
            target[-1] = 1.0
            engine.write(target)
            for _ in range(1000):
                engine.step()
            assert engine.gripper_force() == pytest.approx(10, abs=1)
        target[-1] = 1.0
        if release == "zero":
            engine.write_force(target, 0)
        else:
            engine.hold()
            engine.write(target)
        assert not engine._force_modes
        for _ in range(1000):
            engine.step()
        assert engine.read()[0][-1] * engine.profile.opening > 0.09
    finally:
        engine.close()


def test_native_force_hold_tracks_request_under_unequal_finger_load(tmp_path):
    pytest.importorskip("mujoco")
    from waddle_sdk.simulators.mujoco import Engine

    fixture = next(
        row
        for row in json.loads(
            (Path(__file__).parent / "fixtures/tube_transport.json").read_text()
        )
        if row["robot"] == "yam"
    )
    _, config = make_site(
        "loaded-force",
        backend="mujoco",
        robot="yam",
        environment="load-clear-test-tubes",
        width=32,
        height=32,
    )
    engine = Engine(config, tmp_path)
    try:
        target = np.array(fixture["grasp"])
        target[-1] = 1.0
        engine.home(target)
        for _ in range(500):
            engine.step()
        # Unequal external load on one finger exercises the compliant coupling;
        # the native solver still owns both jaw positions and object contacts.
        follower = engine.model.joint(engine.description.hand_names[1])
        engine.data.qfrc_applied[follower.dofadr[0]] = -40
        force = engine.gripper_force_limits()[1]
        measured = []
        for tick in range(150):
            target[-1] = max(0, engine.read()[0][-1] - 0.05)
            engine.write_force(target, force)
            for _ in range(round(0.04 / engine.model.opt.timestep)):
                engine.step()
            q, dq = engine.read()
            if tick >= 125:
                measured.append(
                    (q[-1] * engine.profile.opening, dq[-1], engine.gripper_force())
                )
        assert all(opening > 0.001 for opening, _, _ in measured)
        assert all(
            abs(speed) * engine.profile.opening < 0.001 for _, speed, _ in measured
        )
        assert all(abs(effort - force) <= 0.5 for _, _, effort in measured), measured
        # Force hold must remain an effort request while the load changes, too.
        # A position-servo approximation can read correctly only after settling.
        moving_effort = []
        for tick in range(500):
            engine.data.qfrc_applied[follower.dofadr[0]] = -40 + 20 * np.sin(
                tick * engine.model.opt.timestep * 2 * np.pi * 5
            )
            engine.step()
            moving_effort.append(engine.gripper_force())
        assert min(moving_effort) >= force - 0.5, min(moving_effort)
        assert max(moving_effort) <= force + 1e-6
        engine.data.qfrc_applied[follower.dofadr[0]] = 0
        target[-1] = 1.0
        engine.write_force(target, 0)
        for _ in range(1000):
            engine.step()
        assert engine.read()[0][-1] * engine.profile.opening > 0.09
        assert engine.gripper_force() < 0.5
    finally:
        engine.close()


def test_native_force_grasp_retains_a_bottle_during_lift_and_releases(tmp_path):
    mujoco = pytest.importorskip("mujoco")
    from waddle_sdk.simulators.mujoco import Engine

    fixture = json.loads(
        (Path(__file__).parent / "fixtures/shampoo_transport.json").read_text()
    )
    _, config = make_site(
        "force-transport",
        backend="mujoco",
        robot="yam",
        environment="shampoo-packing",
        width=32,
        height=32,
    )
    engine = Engine(config, tmp_path)
    try:
        target = np.array(fixture["initial"])
        engine.home(target)
        # Initialize only the arm; the ordinary native pile and contacts remain.
        for motion in fixture["motions"][:2]:
            start, target = target, np.array(motion["end"])
            steps = round(motion["seconds"] / engine.model.opt.timestep)
            for step in range(steps):
                t = (step + 1) / steps
                blend = t**3 * (10 - 15 * t + 6 * t * t)
                engine.write(start + blend * (target - start))
                engine.step()
        force = engine.gripper_force_limits()[1]
        consecutive = 0
        for _ in range(125):
            target[-1] = max(0, engine.read()[0][-1] - 0.05)
            engine.write_force(target, force)
            for _ in range(round(0.04 / engine.model.opt.timestep)):
                engine.step()
            q, dq = engine.read()
            qualified = (
                q[-1] * engine.profile.opening > 0.003
                and abs(dq[-1]) * engine.profile.opening < 0.001
                and engine.gripper_force() >= force - 0.5
            )
            consecutive = consecutive + 1 if qualified else 0
            if consecutive == 3:
                break
        assert consecutive == 3
        bottle = engine.data.body(fixture["target"])
        initial_z = float(bottle.xpos[2])
        start = target.copy()
        lift = next(m for m in fixture["motions"] if m["phase"] == "lift")
        target = np.array(lift["end"])
        steps = round(lift["seconds"] / engine.model.opt.timestep)
        for step in range(steps):
            t = (step + 1) / steps
            blend = t**3 * (10 - 15 * t + 6 * t * t)
            engine.write(start + blend * (target - start))
            engine.step()
        assert bottle.xpos[2] > initial_z + 0.03
        touched = set()
        contact_force = np.zeros(6)
        for index, contact in enumerate(engine.data.contact):
            names = {
                engine.model.body(int(engine.model.geom_bodyid[g])).name
                for g in contact.geom
            }
            if fixture["target"] in names:
                mujoco.mj_contactForce(engine.model, engine.data, index, contact_force)
                if contact_force[0] > 0.01:
                    touched.update(names & {"tip_left", "tip_right"})
        assert touched == {"tip_left", "tip_right"}
        assert engine.gripper_force() == pytest.approx(force, abs=0.5)
        lifted_z = float(bottle.xpos[2])
        target[-1] = 1.0
        engine.write_force(target, 0)
        for _ in range(1000):
            engine.step()
        assert engine.read()[0][-1] * engine.profile.opening > 0.09
        assert bottle.xpos[2] < lifted_z - 0.02
    finally:
        engine.close()


@pytest.mark.parametrize("arms", [1, 2])
def test_public_named_sdk_force_command_reaches_native_worker(tmp_path, arms):
    pytest.importorskip("mujoco")
    import yaml
    from waddle_sdk import load_site
    from waddle_sdk.runtime import JointPositionCommand, SupportFact

    site, config = make_site(
        "force-port",
        backend="mujoco",
        robot="yam",
        environment="two_cubes",
        arms=arms,
        width=32,
        height=32,
    )
    part = next(iter(site["parts"]))
    site["cameras"] = {}
    (tmp_path / "simulation.json").write_text(json.dumps(config))
    (tmp_path / "site.yaml").write_text(yaml.safe_dump(site))
    with load_site(tmp_path / "site.yaml").open(console=False, _testing=True) as owner:
        assert SupportFact.GRIPPER_FORCE in owner.support().rows[0].facts
        assert owner.describe()["command_limits"][part]["gripper_force_n"][0] == 1
        with owner.run(task="force contract", actor="test") as run:
            observation = owner.observe_parts([part])
            force = observation.parts[part].gripper_force
            assert (
                force is not None
                and force.estimated
                and force.source == "mujoco.yam.actuator_force"
            )
            target = observation.parts[part].joint_position.copy()
            target[-1] = max(0, target[-1] - 0.01)
            receipt = run.step_parts(
                {part: JointPositionCommand(target, gripper_force_n=10)}, observation
            )[part]
            assert receipt.dispatched and receipt.gate == "pass"
            receipt = run.step_parts(
                {part: JointPositionCommand(target, gripper_force_n=0)},
                owner.observe_parts([part]),
            )[part]
            assert receipt.dispatched
