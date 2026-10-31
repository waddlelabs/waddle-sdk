"""A native reference stream waits at rest until explicit continuation or stop."""

import time
from dataclasses import replace

import pytest
from test_finite_playback import opened as opened  # noqa: PLC0414 -- pytest fixture
from test_paced_playback import request
from waddle_sdk.runtime import (
    FaultCode,
    PlaybackCurveUpdate,
    PlaybackWatchdog,
    RuntimeFault,
    SupportFact,
)


def stream(session):
    motion = request()
    return replace(
        motion,
        policy=replace(
            motion.policy,
            timeout_ns=None,
            reference_only=True,
            allow_curve_updates=True,
            watchdog=PlaybackWatchdog(session.observe().session_ns, 30_000_000_000),
        ),
    )


def resting(handle, revision):
    # Completion follows acknowledged native evidence, never a chosen sleep.
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        value = handle.snapshot()
        assert value["failure"] is None, value
        progress = value["curve_progress"]
        if (
            progress
            and progress["revision"] == revision
            and progress["endpoint_acknowledged"]
        ):
            return value
        time.sleep(0.001)
    pytest.fail(f"stream did not reach its resting tail: {handle.snapshot()}")


def test_native_stream_rests_resumes_and_cancels_with_completed_hold(opened):
    session, run = opened
    assert SupportFact.STREAMED_JOINT_PLAYBACK in session.support().rows[0].facts
    motion = stream(session)
    handle = run.start_playback(motion)
    try:
        first = resting(handle, 0)
        assert first["status"] == "running"
        curve = motion.policy.paced_curve
        goal = curve.positions[-1]
        continuation = replace(
            curve,
            positions=(goal, (0.0, 0.0)),
            coefficients=(
                tuple(
                    tuple(scale * q for q in goal) for scale in (1, 0, 0, -10, 15, -6)
                ),
            ),
        )
        handle.update_curve(PlaybackCurveUpdate(0, 200_000_000, continuation))
        second = resting(handle, 1)
        assert second["status"] == "running"
        assert second["curve_update"]["status"] == "accepted"
        assert second["last_reference"]["positions"] == [0.0, 0.0]
        assert second["dispatched"] > first["dispatched"]
    finally:
        handle.cancel("released")
        stopped = handle.wait()
    assert stopped["status"] == "failed"
    assert stopped["failure"]["code"] == "cancelled"
    assert stopped["failure"]["outcome_unknown"] is False
    assert stopped["cleanup"]["error"] is None


def test_stream_requires_watchdog_and_separate_support(opened, monkeypatch):
    session, run = opened
    motion = stream(session)
    with pytest.raises(RuntimeFault) as caught:
        run.start_playback(
            replace(motion, policy=replace(motion.policy, watchdog=None))
        )
    assert caught.value.code == FaultCode.INVALID_REQUEST
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
                        f for f in row.facts if f != SupportFact.STREAMED_JOINT_PLAYBACK
                    ),
                )
                for row in support.rows
            ),
        ),
    )
    with pytest.raises(RuntimeFault) as caught:
        run.start_playback(motion)
    assert caught.value.code == FaultCode.UNSUPPORTED
    assert session._managed.accepted == 0
