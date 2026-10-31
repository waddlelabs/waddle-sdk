"""Locally enforced playback envelopes survive absent cloud polling."""

from threading import Event

import numpy as np
import pytest
from test_coordinated_playback import _engage, _push_due_chunk
from test_coordinated_playback import request as group_request
from test_named_parts import DRIVERS
from test_named_parts import site_path as site_path  # noqa: PLC0414
from test_parallel_playback import feedback, request, scoped_request
from waddle_sdk import load_site
from waddle_sdk.runtime import (
    FaultCode,
    JointPositionBounds,
    JointPositionCommand,
    RuntimeFault,
    SupportFact,
)


def limits(radius=0.1):
    return JointPositionBounds((-radius, -radius), (radius, radius))


def assert_shared_stop(handles):
    results = [handle.wait() for handle in handles]
    for row in results:
        assert row["status"] == "failed", row
        assert row["cleanup"]["request"] == "hold"
        assert row["cleanup"]["outcome"] == "acknowledged"
        assert "part" not in row["cleanup"]
    return next(
        row["failure"]["evidence"] for row in results if row["failure"]["evidence"]
    )


@pytest.mark.parametrize("failure", ["outside", "read_fault"])
def test_peer_feedback_invalidates_shared_bounds_locally(
    site_path, monkeypatch, failure
):
    with (
        load_site(site_path).open(console=False, _testing=True) as sdk,
        sdk.run(task="shared bounds", actor="test") as run,
    ):
        moving, dispatched = feedback(monkeypatch)
        for flag in moving.values():
            flag.set()
        bad = Event()
        original = DRIVERS["right"].read
        fault = RuntimeFault(
            FaultCode.MOTOR_FAILURE, "right encoder lost", context={"motor": 2}
        )

        def read():
            if bad.is_set():
                if failure == "read_fault":
                    raise fault
                return np.array([0.2, 0]), np.ones(2)
            return original()

        with monkeypatch.context() as patch:
            patch.setattr(DRIVERS["right"], "read", read)
            assert all(
                SupportFact.JOINT_PLAYBACK_BOUNDS in row.facts
                for row in sdk.support().rows
            )
            run.set_playback_bounds({part: limits() for part in DRIVERS})
            handles = [
                run.start_playback(scoped_request(part, (0.04, 0))) for part in DRIVERS
            ]
            assert all(event.wait(5) for event in dispatched.values())
            bad.set()
            evidence = assert_shared_stop(handles)
            bad.clear()
            if failure == "read_fault":
                assert evidence == fault.as_dict()
            else:
                assert evidence["detail"] == "playback_joint_bounds"
                assert evidence["context"]["part"] == "right"
                assert evidence["context"]["source"] == "feedback"
                assert evidence["context"]["positions"] == [0.2, 0]
        for flag in moving.values():
            flag.clear()


def test_reference_outside_bounds_never_reaches_owner(site_path, monkeypatch):
    with (
        load_site(site_path).open(console=False, _testing=True) as sdk,
        sdk.run(task="bounded targets", actor="test") as run,
    ):
        feedback(monkeypatch)
        commands = []
        original = DRIVERS["left"].write

        def write(target):
            commands.append(tuple(target))
            original(target)

        monkeypatch.setattr(DRIVERS["left"], "write", write)
        run.set_playback_bounds({"left": limits(0.05)})
        result = run.start_playback(scoped_request("left", (0.06, 0))).wait()
        assert result["failure"]["evidence"]["context"]["source"] == "reference", result
        assert result["cleanup"]["outcome"] == "acknowledged"
        assert "part" not in result["cleanup"]
        assert commands and all(abs(row[0]) <= 0.05 for row in commands)


@pytest.mark.parametrize(
    "bad",
    [
        {"absent": limits()},
        {"right": JointPositionBounds((0,), (1,))},
        {"right": JointPositionBounds((0, 0), (float("nan"), 1))},
        {"right": JointPositionBounds((1, 0), (0, 1))},
    ],
)
def test_invalid_replacement_preserves_complete_old_bounds(site_path, monkeypatch, bad):
    with (
        load_site(site_path).open(console=False, _testing=True) as sdk,
        sdk.run(task="atomic bounds", actor="test") as run,
    ):
        _, dispatched = feedback(monkeypatch)
        run.set_playback_bounds({"left": limits()})
        with pytest.raises(RuntimeFault) as caught:
            run.set_playback_bounds({"left": limits(0.01), **bad})
        assert caught.value.code == FaultCode.INVALID_REQUEST
        assert not any(event.is_set() for event in dispatched.values())
        result = run.start_playback(request("left", (0.04, 0))).wait()
        assert result["status"] == "arrived", result


def test_replacement_is_visible_to_active_buffers_and_clear_removes_it(
    site_path, monkeypatch
):
    with load_site(site_path).open(console=False, _testing=True) as sdk:
        with sdk.run(task="replace bounds", actor="test") as run:
            moving, dispatched = feedback(monkeypatch)
            moving["left"].set()
            run.set_playback_bounds({"left": limits()})
            handle = run.start_playback(scoped_request("left", (0.04, 0)))
            assert dispatched["left"].wait(5)
            # The next local reference is refused under the replacement even
            # though the cloud does not poll this buffer while it advances.
            run.set_playback_bounds({"left": limits(0.001)})
            result = handle.wait()
            assert result["failure"]["evidence"]["detail"] == "playback_joint_bounds"
            moving["left"].clear()
            run.set_playback_bounds({})
            DRIVERS["left"]._target[:] = 0
            result = run.start_playback(request("left", (0.04, 0))).wait()
            assert result["status"] == "arrived", result
            run.set_playback_bounds({"left": limits(0.001)})
        # Envelopes are run-scoped. No configuration leaks to a later run.
        with sdk.run(task="next run", actor="test") as later:
            DRIVERS["left"]._target[:] = 0
            result = later.start_playback(request("left", (0.04, 0))).wait()
            assert result["status"] == "arrived", result


@pytest.mark.parametrize("source", ["direct", "supervision"])
def test_final_selected_command_cannot_escape_bounds(site_path, monkeypatch, source):
    with (
        load_site(site_path).open(console=False, _testing=True) as sdk,
        sdk.run(task="bounded gate output", actor="test") as run,
    ):
        feedback(monkeypatch)
        run.step((0, 0, 0, 0))
        run.set_playback_bounds({part: limits() for part in DRIVERS})
        if source == "direct":
            result = run.step_parts({"right": JointPositionCommand((0.2, 0))})["right"]
            assert not result.dispatched
            assert result.gate == "playback_bounds_refusal"
            fault = result.fault.as_dict()
        else:
            native = _engage(sdk)
            _push_due_chunk(native, [0.2, 0], part="right")
            result = run.start_playback(group_request()).wait()
            assert result["status"] == "failed", result
            assert result["failure"]["outcome_unknown"] is False
            fault = result["failure"]["evidence"]
            assert result["cleanup"]["outcome"] == "acknowledged"
        assert fault["detail"] == "playback_joint_bounds"
        assert fault["context"]["source"] == "reference"
        assert fault["context"]["part"] == "right"
        assert DRIVERS["right"]._target == pytest.approx((0, 0))
