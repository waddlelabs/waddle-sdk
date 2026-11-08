"""Native queue handoff through the public Run and the authenticated relay adapter."""

from dataclasses import replace
from threading import Event

import numpy as np
import pytest
from test_coordinated_playback import request as group_request
from test_finite_playback import motion
from test_finite_playback import opened as opened  # noqa: PLC0414
from test_named_parts import site_path as site_path  # noqa: PLC0414
from test_parallel_playback import feedback
from test_parallel_playback import request as named_request
from waddle_sdk import load_site
from waddle_sdk.runtime import (
    FaultCode,
    JointPositionCommand,
    JointReference,
    PlaybackDependency,
    RuntimeFault,
    SupportFact,
)
from waddle_sdk_relay.device import Device
from waddle_sdk_relay.errors import RelayError
from waddle_sdk_relay.wire import decode, encode


def moving_owner(session, monkeypatch):
    driver = session._managed.arms["arm"].driver
    moving, wrote = Event(), Event()
    moving.set()
    writes = []
    write = driver.write

    def record(q):
        write(q)
        writes.append(tuple(q))
        wrote.set()

    monkeypatch.setattr(driver, "write", record)
    monkeypatch.setattr(
        driver,
        "read",
        lambda: (driver._target.copy(), np.full(2, float(moving.is_set()))),
    )
    return moving, wrote, writes


def request(start, end):
    value = motion(start, end)
    return replace(value, policy=replace(value.policy, timeout_ns=120_000_000_000))


def test_sdk_runs_ordered_queue_without_polling_between_commands(opened, monkeypatch):
    session, run = opened
    moving, wrote, writes = moving_owner(session, monkeypatch)
    first = run.start_playback(request((0, 0), (0.02, 0)))
    try:
        assert wrote.wait(5)
        second = run.start_playback(
            request((0.02, 0), (0.04, 0)),
            after=[PlaybackDependency(first, ("arrived",))],
        )
        third = run.start_playback(
            request((0.04, 0), (0.06, 0)),
            after=[PlaybackDependency(second, ("arrived",))],
        )
        assert third.snapshot()["waiting_for"] == [second.snapshot()["id"]]
        with pytest.raises(RuntimeFault) as conflict:
            run.start_playback(
                request((0.02, 0), (0.08, 0)),
                after=[PlaybackDependency(first, ("arrived",))],
            )
        assert conflict.value.code == FaultCode.BUSY
    finally:
        moving.clear()
    result = (
        third.wait()
    )  # This waits for the retained result, without driving handoff.
    assert result["status"] == "arrived", result
    assert result["waiting_for"] == []
    assert first.snapshot()["status"] == second.snapshot()["status"] == "arrived"
    assert writes == [(0, 0), (0.02, 0), (0.02, 0), (0.04, 0), (0.04, 0), (0.06, 0)]


@pytest.mark.parametrize("end", ["cancel", "expire"])
def test_ending_queued_work_does_not_stop_its_running_parent(opened, monkeypatch, end):
    session, run = opened
    moving, wrote, writes = moving_owner(session, monkeypatch)
    first = run.start_playback(request((0, 0), (0.02, 0)))
    try:
        assert wrote.wait(5)
        second = run.start_playback(
            request((0.02, 0), (0.04, 0)),
            after=[PlaybackDependency(first, ("arrived",))],
            start_before_ns=1 if end == "expire" else None,
        )
        if end == "cancel":
            second.cancel("remove queued motion")
        result = second.wait()
        assert result["failure"]["code"] == (
            "cancelled" if end == "cancel" else "start_expired"
        )
        assert result["dispatched"] == 0 and result["cleanup"] is None
        assert first.snapshot()["status"] == "running"
    finally:
        moving.clear()
    assert first.wait()["status"] == "arrived"
    assert writes == [(0, 0), (0.02, 0)]


def test_queue_support_and_outcome_validation_precede_dispatch(opened, monkeypatch):
    session, run = opened
    first = run.start_playback(motion())
    assert first.wait()["status"] == "arrived"
    for outcomes in [(), ("failed",), ("anything",)]:
        with pytest.raises(RuntimeFault) as invalid:
            run.start_playback(motion(), after=[PlaybackDependency(first, outcomes)])
        assert invalid.value.code == FaultCode.INVALID_REQUEST
    support = session.support()
    monkeypatch.setattr(
        session,
        "support",
        lambda: replace(
            support,
            rows=tuple(
                replace(
                    row,
                    facts=tuple(
                        f for f in row.facts if f != SupportFact.DEPENDENT_PLAYBACK
                    ),
                )
                for row in support.rows
            ),
        ),
    )
    with pytest.raises(RuntimeFault) as unsupported:
        run.start_playback(motion(), after=[PlaybackDependency(first, ("arrived",))])
    assert unsupported.value.code == FaultCode.UNSUPPORTED


def test_relay_resolves_only_retained_dependencies_in_this_run(
    opened, monkeypatch, tmp_path
):
    session, run = opened
    moving, wrote, writes = moving_owner(session, monkeypatch)
    device = Device(None, tmp_path)
    device.sdk, device.run = session, run
    first = device.dispatch(
        "playback_start", {"run_id": run.id, "motion": request((0, 0), (0.02, 0))}
    )
    try:
        assert wrote.wait(5)
        second = device.dispatch(
            "playback_start",
            decode(
                encode(
                    {
                        "run_id": run.id,
                        "motion": request((0.02, 0), (0.04, 0)),
                        "after": [{"id": first["id"], "outcomes": ["arrived"]}],
                    }
                )
            ),
        )
        with pytest.raises(RelayError) as unknown:
            device.dispatch(
                "playback_start",
                {
                    "run_id": run.id,
                    "motion": motion(),
                    "after": [{"id": "foreign", "outcomes": ["arrived"]}],
                },
            )
        assert unknown.value.code == "sdk_playback_expired"
    finally:
        moving.clear()
    result = device._playbacks[(run.id, second["id"])].wait()
    assert result["status"] == "arrived", result
    assert (
        device.dispatch("playback_status", {"run_id": run.id, "id": second["id"]})
        == result
    )
    assert writes == [(0, 0), (0.02, 0), (0.02, 0), (0.04, 0)]


def test_coordinated_queue_waits_for_every_selected_predecessor(site_path, monkeypatch):
    with (
        load_site(site_path).open(console=False, _testing=True) as sdk,
        sdk.run(task="joined queue", actor="test") as run,
    ):
        moving, _ = feedback(monkeypatch)
        for flag in moving.values():
            flag.set()
        left = run.start_playback(named_request("left", (0.04, 0)))
        right = run.start_playback(named_request("right", (-0.03, 0)))
        value = group_request()
        value = replace(
            value,
            references=(
                JointReference(0, JointPositionCommand((0.04, 0, -0.03, 0))),
                JointReference(200_000_000, JointPositionCommand((0.02, 0, -0.01, 0))),
            ),
        )
        try:
            with pytest.raises(RuntimeFault) as incomplete:
                run.start_playback(
                    value, after=[PlaybackDependency(left, ("arrived",))]
                )
            assert incomplete.value.code == FaultCode.BUSY
            joined = run.start_playback(
                value,
                after=[
                    PlaybackDependency(left, ("arrived",)),
                    PlaybackDependency(right, ("arrived",)),
                ],
            )
            moving["left"].clear()
            assert left.wait()["status"] == "arrived"
            assert right.snapshot()["status"] == "running"
            assert joined.snapshot()["dispatched"] == 0
        finally:
            for flag in moving.values():
                flag.clear()
        result = joined.wait()
        assert result["status"] == "arrived", result
        assert result["feedback"]["positions"] == pytest.approx((0.02, 0, -0.01, 0))


def test_elapsed_time_begins_on_activation_and_survives_wire_roundtrip(
    opened, monkeypatch
):
    session, run = opened
    moving, wrote, _ = moving_owner(session, monkeypatch)
    first = run.start_playback(request((0, 0), (0.02, 0)))
    try:
        assert wrote.wait(5)
        second = run.start_playback(
            request((0.02, 0), (0.04, 0)),
            after=[PlaybackDependency(first, ("arrived",))],
        )
        assert second.snapshot()["elapsed_ns"] is None
    finally:
        moving.clear()
    child = decode(encode(second.wait()))
    parent = first.wait()
    assert SupportFact.PLAYBACK_TIMING in session.support().rows[0].facts
    assert child["status"] == "arrived"
    assert child["elapsed_ns"] >= 0
    # Activation follows the predecessor's final acquired feedback, even if the
    # caller receives both results only afterward. No clock synchronization.
    assert (
        child["feedback"]["observed_ns"] - child["elapsed_ns"]
        >= parent["feedback"]["observed_ns"]
    )
    assert second.snapshot()["elapsed_ns"] == child["elapsed_ns"]
