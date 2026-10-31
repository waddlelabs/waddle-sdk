"""Distinct named buffers share native ownership without sharing completion."""

from dataclasses import replace
from threading import Event

import numpy as np
import pytest
from test_coordinated_playback import request as group_request
from test_named_parts import DRIVERS
from test_named_parts import site_path as site_path  # noqa: PLC0414 -- fixture
from waddle_sdk import load_site
from waddle_sdk.runtime import (
    FaultCode,
    FiniteJointMotion,
    JointPositionCommand,
    JointReference,
    RuntimeFault,
    SupportFact,
)


def request(part, target):
    group = group_request()
    return FiniteJointMotion(
        part,
        (
            JointReference(0, JointPositionCommand((0, 0))),
            JointReference(200_000_000, JointPositionCommand(target)),
        ),
        replace(
            group.policy,
            indices=(0, 1),
            start_tolerance=(0.01,) * 2,
            position_tolerance=(0.001,) * 2,
            stationary_velocity=(0.01,) * 2,
            timeout_ns=120_000_000_000,
        ),
    )


def feedback(monkeypatch):
    moving = {part: Event() for part in DRIVERS}
    dispatched = {part: Event() for part in DRIVERS}
    for part, driver in DRIVERS.items():

        def read(driver=driver, part=part):
            return driver._target.copy(), np.full(2, float(moving[part].is_set()))

        def write(target, original=driver.write, part=part):
            original(target)
            dispatched[part].set()

        monkeypatch.setattr(driver, "read", read)
        monkeypatch.setattr(driver, "write", write)
    return moving, dispatched


def test_independent_completion_and_named_direct_scope_conflicts(
    site_path, monkeypatch
):
    with (
        load_site(site_path).open(console=False, _testing=True) as sdk,
        sdk.run(task="independent", actor="test") as run,
    ):
        moving, _ = feedback(monkeypatch)
        moving["left"].set()  # Explicit feedback condition prevents early settling.
        assert all(
            SupportFact.PARALLEL_JOINT_PLAYBACK in row.facts
            for row in sdk.support().rows
        )
        left = run.start_playback(request("left", (0.04, -0.02)))
        try:
            for motion in (request("left", (0.03, 0)), group_request()):
                with pytest.raises(RuntimeFault) as caught:
                    run.start_playback(motion)
                assert caught.value.code == FaultCode.BUSY
            with pytest.raises(RuntimeFault) as caught:
                run.step_parts({"left": JointPositionCommand((0, 0))})
            assert caught.value.code == FaultCode.BUSY
            # A disjoint ordinary command retains its own gate and owner checks.
            assert run.step_parts({"right": JointPositionCommand((0, 0))})[
                "right"
            ].dispatched
            right = run.start_playback(request("right", (-0.03, 0.01)))
            completed = right.wait()
            assert completed["status"] == "arrived", completed
            assert completed["cleanup"] is None
            assert left.snapshot()["status"] == "running"
            assert completed["feedback"]["positions"] == pytest.approx((-0.03, 0.01))
        finally:
            moving["left"].clear()
        result = left.wait()
        assert result["status"] == "arrived", result
        assert result["cleanup"] is None
        assert result["feedback"]["positions"] == pytest.approx((0.04, -0.02))


@pytest.mark.parametrize("stop", ["cancel", "hold", "estop"])
def test_site_wide_stop_drains_every_independent_buffer(site_path, monkeypatch, stop):
    with (
        load_site(site_path).open(console=False, _testing=True) as sdk,
        sdk.run(task="stop independent", actor="test") as run,
    ):
        moving, dispatched = feedback(monkeypatch)
        for flag in moving.values():
            flag.set()
        left = run.start_playback(request("left", (0.04, -0.02)))
        right = run.start_playback(request("right", (-0.03, 0.01)))
        assert all(event.wait(5) for event in dispatched.values())
        # Both buffers have entered owner dispatch. The cleanup receipt below
        # remains distinct from measured physical stopping.
        if stop == "cancel":
            left.cancel("end local buffer")
        else:
            getattr(sdk, stop)("end local buffers")
        for handle in (left, right):
            result = handle.wait()
            assert result["failure"]["code"] == "cancelled", result
            assert result["cleanup"]["outcome"] == "acknowledged", result
            assert result["cleanup"]["request"] == (
                "estop" if stop == "estop" else "hold"
            )
        assert not run._episode.playback_active
