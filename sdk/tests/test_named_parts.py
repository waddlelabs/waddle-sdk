"""Independent addressed commands retain native supervision and scoped failures."""

from dataclasses import replace
from threading import Event

import numpy as np
import pytest
import yaml

from waddle_sdk import load_site
from waddle_sdk.robots import mock
from waddle_sdk.runtime import FaultCode, JointPositionCommand, RuntimeFault

from test_site_api import _write_site

DRIVERS = {}


def arm(*, config):
    rig = mock.arm(config=config)
    build = rig.build_arms

    def opened():
        arms = build()
        DRIVERS[config.name] = next(iter(arms.values())).driver
        return arms

    return replace(rig, build_arms=opened, report=lambda _: None)


@pytest.fixture
def site_path(tmp_path):
    DRIVERS.clear()
    path = _write_site(tmp_path)
    document = yaml.safe_load(path.read_text())
    declaration = document["parts"]["arm"]
    declaration["driver"] = "test_named_parts:arm"
    declaration["options"] = {"joint_count": 2}
    document["parts"] = {name: dict(declaration) for name in ("left", "right")}
    document["cameras"] = {}
    path.write_text(yaml.safe_dump(document, sort_keys=False))
    return path


def test_named_dispatch_does_not_overwrite_neighbor_and_observes_arrival(site_path):
    with load_site(site_path).open() as sdk, sdk.run(task="parts", actor="test") as run:
        left = JointPositionCommand([0.01, 0.02])
        right = JointPositionCommand([-0.02, 0.01])
        receipts = run.step_parts({"left": left, "right": right}, sdk.observe_parts())
        assert all(
            result.dispatched and result.gate == "pass" for result in receipts.values()
        )
        np.testing.assert_allclose(DRIVERS["left"]._target, left.positions)
        np.testing.assert_allclose(DRIVERS["right"]._target, right.positions)
        assert run.step_parts({"left": JointPositionCommand([0.0, 0.0])})[
            "left"
        ].dispatched
        np.testing.assert_allclose(DRIVERS["right"]._target, right.positions)


@pytest.mark.parametrize("method", ["read", "write"])
def test_local_fault_retains_neighbor_receipt_and_original_error(
    site_path, monkeypatch, method
):
    with (
        load_site(site_path).open() as sdk,
        sdk.run(task="faults", actor="test") as run,
    ):
        failure = RuntimeFault(
            FaultCode.MOTOR_FAILURE, "left motor 2 failed", context={"motor": 2}
        )

        def fail(*_):
            raise failure

        with monkeypatch.context() as patch:
            patch.setattr(DRIVERS["left"], method, fail)
            observation = sdk.observe_parts()
            assert "right" in observation.parts
            if method == "read":
                assert observation.faults["left"] is failure
                with pytest.raises(RuntimeFault) as raised:
                    sdk.observe()
                assert raised.value is failure
            result = run.step_parts(
                {p: JointPositionCommand([0.01, 0.0]) for p in ("left", "right")},
                observation,
            )
            assert result["left"].fault is failure
            assert result["right"].dispatched
            np.testing.assert_allclose(DRIVERS["right"]._target, [0.01, 0.0])


def test_named_commands_keep_global_hold_and_reject_unknown_parts(site_path, monkeypatch):
    with load_site(site_path).open() as sdk, sdk.run(task="hold", actor="test") as run:
        with pytest.raises(RuntimeFault):
            run.step_parts({"absent": JointPositionCommand([0.0, 0.0])})
        stopped = []
        for driver in DRIVERS.values():
            event = Event()
            original = driver.estop

            def stop(original=original, event=event):
                original()
                event.set()

            monkeypatch.setattr(driver, "estop", stop)
            stopped.append(event)
        sdk.estop("explicit stop")
        assert all(event.wait(2) for event in stopped)
        receipts = run.step_parts(
            {p: JointPositionCommand([0.01, 0.0]) for p in ("left", "right")}
        )
        assert all(not result.dispatched for result in receipts.values())
        for driver in DRIVERS.values():
            np.testing.assert_allclose(driver._target, [0.0, 0.0])
