"""Checked polynomial playback through the public Run and real native worker."""

import threading
from dataclasses import replace

import pytest
from test_finite_playback import opened as opened  # noqa: PLC0414 -- pytest fixture
from waddle_sdk.robots.base import SimDriver
from waddle_sdk.runtime import (
    FaultCode,
    FiniteJointMotion,
    JointPositionCommand,
    JointReference,
    PacedJointCurve,
    PlaybackCurveUpdate,
    PlaybackPolicy,
    RuntimeFault,
    SupportFact,
)


def request(*, grid=False):
    target = (0.1, -0.05)
    curve = PacedJointCurve(
        (0.0, 0.2),
        ((0.0, 0.0), target),
        (tuple(tuple(scale * q for q in target) for scale in (0, 0, 0, 10, -15, 6)),),
        50_000_000,
        (0.2, 0.2),
        (0.2, 0.2),
        (0.0, 0.05, 0.1, 0.15, 0.2) if grid else None,
    )
    return FiniteJointMotion(
        "arm",
        (
            JointReference(0, JointPositionCommand((0, 0), (0, 0))),
            JointReference(200_000_000, JointPositionCommand(target, (0, 0))),
        ),
        PlaybackPolicy(
            (0, 1),
            (0.01, 0.01),
            (0.001, 0.001),
            (0.01, 0.01),
            40_000_000,
            1,
            2_000_000_000,
            paced_curve=curve,
        ),
    )


@pytest.mark.parametrize("grid", [False, True])
def test_paced_curve_uses_native_owner_and_retains_acknowledged_progress(opened, grid):
    session, run = opened
    assert SupportFact.PACED_JOINT_PLAYBACK in session.support().rows[0].facts
    result = run.start_playback(request(grid=grid)).wait()
    assert result["status"] == "arrived", result
    assert result["dispatched"] == 5 and result["references"] == 2
    assert result["feedback"]["positions"] == pytest.approx([0.1, -0.05])
    assert result["last_reference"]["positions"] == [0.1, -0.05]
    assert result["last_reference"]["velocity_feedforward"] == [0.0, 0.0]
    assert result["last_reference"]["offset_ns"] == 200_000_000
    assert result["curve_progress"] == {
        "revision": 0,
        "pending_revision": None,
        "acknowledged_curve_time_s": 0.2,
        "next_curve_time_s": None,
        "endpoint_acknowledged": True,
        "limited_ticks": 0,
        "partial_ticks": 0,
        "held_ticks": 0,
    }


@pytest.mark.parametrize("invalid", ["cadence", "shape", "curve", "hint", "grid"])
def test_paced_curve_admission_never_opens_an_unchecked_reference(opened, invalid):
    session, run = opened
    motion = request()
    curve = motion.policy.paced_curve
    if invalid == "cadence":
        curve = replace(curve, period_ns=1_000_000)
    elif invalid == "shape":
        curve = replace(curve, coefficients=())
    elif invalid == "curve":
        curve = replace(curve, positions=((0, 0), (0.2, -0.05)))
    elif invalid == "hint":
        motion = replace(
            motion,
            references=(
                replace(
                    motion.references[0], command=JointPositionCommand((0, 0), (0.1, 0))
                ),
                motion.references[1],
            ),
        )
    else:
        curve = replace(curve, sample_times_s=(0.0, 0.2))
    motion = replace(motion, policy=replace(motion.policy, paced_curve=curve))
    with pytest.raises(RuntimeFault) as caught:
        run.start_playback(motion)
    assert caught.value.code == FaultCode.INVALID_REQUEST
    assert session._managed.accepted == 0


def test_paced_cancel_drains_entered_write_and_preserves_original_failure(
    opened, monkeypatch
):
    _, run = opened
    entered, release = threading.Event(), threading.Event()
    original = RuntimeFault(
        FaultCode.MOTOR_FAILURE, "simulated write lost", context={"motor": 2}
    )

    def write(_driver, _target):
        entered.set()
        assert release.wait(5)
        raise original

    monkeypatch.setattr(SimDriver, "write", write)
    handle = run.start_playback(request())
    try:
        assert entered.wait(3)
        assert handle.snapshot()["feedback_reference"] is None
        handle.cancel("stop during write")
        assert handle.snapshot()["status"] == "running"
    finally:
        release.set()
    result = handle.wait()
    assert result["dispatched"] == 0 and result["last_reference"] is None
    assert result["failure"]["code"] == "dispatch"
    assert result["failure"]["outcome_unknown"] is True
    assert result["failure"]["evidence"] == original.as_dict()


def future_update():
    # Join at the original resting endpoint, then move to a different endpoint.
    original = request().policy.paced_curve
    start = original.positions[-1]
    end = tuple(2 * q for q in start)
    curve = replace(
        original,
        positions=(start, end),
        coefficients=(
            (start, (0, 0), (0, 0))
            + tuple(tuple(scale * q for q in start) for scale in (10, -15, 6)),
        ),
    )
    return PlaybackCurveUpdate(0, 200_000_000, curve)


@pytest.mark.parametrize("outcome", ["updated", "stale", "cancelled"])
def test_update_admission_does_not_wait_for_device_io_or_replay_on_refusal(
    opened, monkeypatch, outcome
):
    session, run = opened
    assert SupportFact.REVISABLE_JOINT_PLAYBACK in session.support().rows[0].facts
    assert SupportFact.PLAYBACK_FEEDBACK_REFERENCE in session.support().rows[0].facts
    write = SimDriver.write
    entered, release = threading.Event(), threading.Event()
    writes = []

    def paused(driver, target):
        writes.append(tuple(target))
        if len(writes) == 1:
            entered.set()
            assert release.wait(5)
        write(driver, target)

    monkeypatch.setattr(SimDriver, "write", paused)
    motion = request()
    motion = replace(
        motion,
        policy=replace(motion.policy, reference_only=True, allow_curve_updates=True),
    )
    handle = run.start_playback(motion)
    update = future_update()
    if outcome == "stale":
        update = replace(update, expected_revision=1)
    try:
        assert entered.wait(3)
        assert handle.update_curve(update) == 1
        assert handle.snapshot()["curve_update"] == {
            "request_id": 1,
            "status": "pending",
            "error": None,
        }
        with pytest.raises(RuntimeFault) as caught:
            handle.update_curve(update)
        assert caught.value.code == FaultCode.BUSY
        if outcome == "cancelled":
            handle.cancel("cancel queued update")
    finally:
        release.set()
    result = handle.wait()
    if outcome == "cancelled":
        assert result["failure"]["code"] == "cancelled", result
        assert result["curve_update"]["status"] == "rejected"
        assert len(writes) == 1
    else:
        assert result["status"] == "references_dispatched", result
        changed = outcome == "updated"
        assert result["curve_update"]["status"] == (
            "accepted" if changed else "rejected"
        )
        assert result["curve_progress"]["revision"] == int(changed)
        assert result["last_reference"]["offset_ns"] == (
            400_000_000 if changed else 200_000_000
        )
        assert result["last_reference"]["positions"] == pytest.approx(
            [0.2, -0.1] if changed else [0.1, -0.05]
        )
        assert writes[-1] == tuple(result["last_reference"]["positions"])
        assert result["feedback"]["observed_ns"] > result["last_acknowledged_ns"]
        assert result["feedback_reference"] == result["last_reference"]
    with pytest.raises(RuntimeFault):
        handle.update_curve(update)


def test_revisable_admission_requires_declared_support(opened, monkeypatch):
    session, run = opened
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
                        f
                        for f in row.facts
                        if f != SupportFact.REVISABLE_JOINT_PLAYBACK
                    ),
                )
                for row in support.rows
            ),
        ),
    )
    motion = request()
    motion = replace(
        motion,
        policy=replace(motion.policy, reference_only=True, allow_curve_updates=True),
    )
    with pytest.raises(RuntimeFault) as caught:
        run.start_playback(motion)
    assert caught.value.code == FaultCode.UNSUPPORTED
    assert session._managed.accepted == 0
