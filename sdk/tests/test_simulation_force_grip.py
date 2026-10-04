"""Native actuator feedback and retained force mode across contact and release."""

import json
from pathlib import Path

import numpy as np
import pytest
from waddle_sdk.simulators.scene import make_site


@pytest.mark.parametrize("empty", [False, True])
def test_native_force_closure_reports_actual_effort_and_retains_force(tmp_path, empty):
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
        engine.write_force(target, 0)
        assert not engine._force_modes
        engine.hold()
        assert not engine._force_modes
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
