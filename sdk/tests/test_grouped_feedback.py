"""Fresh shared-backend observations retain subsets, faults and legacy drivers."""

from types import SimpleNamespace

import numpy as np
import pytest
from test_named_parts import DRIVERS
from test_named_parts import site_path as named_site_path
from waddle_sdk import load_site
from waddle_sdk.runtime import FaultCode, PartObservation, RuntimeFault

site_path = named_site_path


@pytest.mark.parametrize(
    "case", ["fresh", "subset", "fault", "malformed", "legacy_peer"]
)
def test_shared_observation_preserves_independent_samples_and_faults(site_path, case):
    calls = []
    generation = [0]
    original = RuntimeFault(
        FaultCode.MOTOR_FAILURE, "right encoder failed", context={"motor": 2}
    )

    def read_parts(parts):
        calls.append(tuple(parts))
        result = {
            part: PartObservation(np.full(2, 0.2 + generation[0] / 100), np.zeros(2))
            for part in parts
        }
        if case == "fault":
            result["right"] = original
        if case == "malformed":
            result.pop("right")
        return result

    source = SimpleNamespace(read_parts=read_parts)
    with load_site(site_path).open() as sdk:
        for name, driver in DRIVERS.items():
            driver.feedback_group = source
            driver.feedback_key = name
        if case == "legacy_peer":
            DRIVERS["right"].feedback_group = None
        selected = ["left"] if case == "subset" else ["left", "right"]
        observed = sdk.observe_parts(selected)
        assert calls == [
            ("left",) if case in {"subset", "legacy_peer"} else ("left", "right")
        ]
        if case == "malformed":
            assert not observed.parts and set(observed.faults) == {"left", "right"}
            assert all(f.code == FaultCode.INTERNAL for f in observed.faults.values())
            return
        np.testing.assert_array_equal(observed.parts["left"].joint_position, [0.2, 0.2])
        if case == "fault":
            assert observed.faults["right"] is original
            assert "right" not in observed.parts
        elif case == "legacy_peer":
            np.testing.assert_array_equal(
                observed.parts["right"].joint_position, [0, 0]
            )
        elif case == "subset":
            assert set(observed.parts) == {"left"}
        else:
            assert not observed.faults
            generation[0] = 1
            fresh = sdk.observe_parts(selected)
            np.testing.assert_allclose(
                fresh.parts["left"].joint_position, [0.21, 0.21], atol=1e-12, rtol=0
            )
            assert len(calls) == 2
        rows = sdk.support().as_dict()["rows"]
        assert "observation.grouped_parts" in next(
            r["facts"] for r in rows if r["scope"] == "robot:left"
        )


@pytest.mark.parametrize("arms", [1, 2])
def test_reference_observation_acquires_joint_and_force_feedback_in_one_transaction(
    tmp_path, monkeypatch, arms
):
    import json

    import yaml

    pytest.importorskip("mujoco")
    from waddle_sdk.simulators.adapters import World
    from waddle_sdk.simulators.scene import make_site

    site, simulation = make_site(
        "grouped-feedback-native",
        backend="mujoco",
        robot="yam",
        environment="two_cubes",
        arms=arms,
    )
    for document in [site, simulation]:
        for camera in document["cameras"].values():
            camera["stream"].update(width=32, height=32, fps=1)
            for key in ["fx", "fy", "cx", "cy"]:
                camera["intrinsics"][key] = 16 if key in {"fx", "fy"} else 15.5
    (tmp_path / "simulation.json").write_text(json.dumps(simulation))
    path = tmp_path / "site.yaml"
    path.write_text(yaml.safe_dump(site))
    requests = []
    original = World.call

    def call(world, operation, *args):
        if operation == "read_feedback":
            requests.append(args)
        return original(world, operation, *args)

    monkeypatch.setattr(World, "call", call)
    with load_site(path).open(console=False) as sdk:
        for _ in range(3):
            before = len(requests)
            observed = sdk.observe_parts()
            assert len(requests) == before + 1
            assert not observed.faults and set(observed.parts) == set(site["parts"])
            for part in observed.parts.values():
                assert part.joint_position.shape == part.joint_velocity.shape == (7,)
                assert part.gripper_force is not None
                assert part.gripper_force.source == "mujoco.yam.actuator_force"
                assert part.gripper_force.estimated is True
