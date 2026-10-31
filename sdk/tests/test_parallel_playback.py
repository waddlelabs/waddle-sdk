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


def scoped_request(part, target):
    motion = request(part, target)
    return replace(motion, policy=replace(motion.policy, part_hold=True))


@pytest.mark.parametrize("fail_motion", [False, True])
@pytest.mark.parametrize("fail_cleanup", [False, True])
def test_part_hold_keeps_peer_running_and_retains_original_faults(
    site_path, monkeypatch, fail_motion, fail_cleanup
):
    with (
        load_site(site_path).open(console=False, _testing=True) as sdk,
        sdk.run(task="scoped cleanup", actor="test") as run,
    ):
        moving, dispatched = feedback(monkeypatch)
        for event in moving.values():
            event.set()
        fault = RuntimeFault(
            FaultCode.MOTOR_FAILURE, "left encoder lost", context={"motor": 2}
        )
        cleanup_fault = RuntimeFault(
            FaultCode.TRANSPORT_LOST, "left Hold reply lost", context={"part": "left"}
        )
        fail_read = Event()
        original_read = DRIVERS["left"].read
        holds = []

        def read():
            if fail_read.is_set():
                raise fault
            return original_read()

        def hold():
            holds.append("left")
            if fail_cleanup:
                raise cleanup_fault

        monkeypatch.setattr(DRIVERS["left"], "read", read)
        left = run.start_playback(scoped_request("left", (0.04, -0.02)))
        right = run.start_playback(scoped_request("right", (-0.03, 0.01)))
        assert all(event.wait(5) for event in dispatched.values())
        # Restrict fault injection to this cleanup; normal session drain still
        # uses the original drivers after both buffers are terminal.
        with monkeypatch.context() as patch:
            patch.setattr(DRIVERS["left"], "hold", hold)
            patch.setattr(DRIVERS["right"], "hold", lambda: holds.append("right"))
            if fail_motion:
                fail_read.set()
            else:
                left.cancel("left only")
            result = left.wait()
            fail_read.clear()
            assert holds == ["left"]
            if fail_motion:
                assert result["failure"]["evidence"] == fault.as_dict()
            else:
                assert result["failure"]["code"] == "cancelled"
            cleanup = result["cleanup"]
            assert cleanup["part"] == "left"
            assert cleanup["request"] == "hold"
            assert cleanup["outcome"] == ("unknown" if fail_cleanup else "acknowledged")
            assert cleanup["measured_stationary"] is False
            if fail_cleanup:
                assert cleanup["evidence"] == cleanup_fault.as_dict()
            assert right.snapshot()["status"] == "running"
            moving["right"].clear()
            completed = right.wait()
            assert completed["status"] == "arrived", completed
            assert completed["cleanup"] is None
            assert completed["feedback"]["positions"] == pytest.approx((-0.03, 0.01))
            assert left.snapshot() == result
        moving["left"].clear()


@pytest.mark.parametrize("stop", ["hold", "estop"])
def test_explicit_site_stop_overrides_scoped_policy(site_path, monkeypatch, stop):
    with (
        load_site(site_path).open(console=False, _testing=True) as sdk,
        sdk.run(task="global stop", actor="test") as run,
    ):
        moving, dispatched = feedback(monkeypatch)
        for event in moving.values():
            event.set()
        left = run.start_playback(scoped_request("left", (0.04, -0.02)))
        right = run.start_playback(scoped_request("right", (-0.03, 0.01)))
        assert all(event.wait(5) for event in dispatched.values())
        getattr(sdk, stop)("stop all")
        for handle in (left, right):
            result = handle.wait()
            assert result["failure"]["code"] == "cancelled", result
            assert result["cleanup"]["request"] == stop
            assert "part" not in result["cleanup"]
            assert result["cleanup"]["outcome"] == "acknowledged"


def test_group_playback_rejects_part_hold_before_writing(site_path, monkeypatch):
    with (
        load_site(site_path).open(console=False, _testing=True) as sdk,
        sdk.run(task="group scope", actor="test") as run,
    ):
        _, dispatched = feedback(monkeypatch)
        motion = group_request()
        with pytest.raises(RuntimeFault) as caught:
            run.start_playback(
                replace(motion, policy=replace(motion.policy, part_hold=True))
            )
        assert caught.value.code == FaultCode.UNSUPPORTED
        assert not any(event.is_set() for event in dispatched.values())
