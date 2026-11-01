"""Independent goal dwell runs on the existing worker and survives wire transport."""

from dataclasses import replace

import pytest
from test_finite_playback import opened as opened  # noqa: PLC0414
from test_resting_goal_playback import resting_request
from waddle_sdk.runtime import (
    FaultCode,
    PlaybackGoalDwell,
    RuntimeFault,
    SupportFact,
)
from waddle_sdk_relay.wire import decode, encode


def request(window):
    motion = resting_request(window)
    return replace(
        motion,
        policy=replace(
            motion.policy,
            goal=replace(motion.policy.goal, dwell=PlaybackGoalDwell(300_000_000)),
            settle_ns=motion.policy.settle_ns if window else 0,
            minimum_settle_samples=1,
        ),
    )


@pytest.mark.parametrize("window", [False, True])
def test_goal_dwell_finishes_locally_with_retained_feedback_clock(opened, window):
    _, run = opened
    handle = run.start_playback(decode(encode(request(window))))
    result = handle.wait()
    assert result["status"] == "arrived", result
    dwell = result["goal_dwell"]
    assert dwell["observed_ns"] == result["feedback"]["observed_ns"]
    assert dwell["observed_ns"] - dwell["since_ns"] >= 300_000_000
    assert dwell["restarts"] == 0
    assert not result["curve_progress"]["endpoint_acknowledged"]
    handle.cancel("late cancellation must preserve retained local completion")
    assert handle.snapshot() == result


@pytest.mark.parametrize("missing", ["support", "duration", "multiplier"])
def test_goal_dwell_admission_refuses_unsupported_or_invalid_rules(
    opened, monkeypatch, missing
):
    session, run = opened
    value = request(False)
    if missing == "support":
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
                            f for f in row.facts if f != SupportFact.DWELL_GOAL_PLAYBACK
                        ),
                    )
                    for row in support.rows
                ),
            ),
        )
    else:
        dwell = (
            PlaybackGoalDwell(0)
            if missing == "duration"
            else PlaybackGoalDwell(300_000_000, 0.5)
        )
        value = replace(
            value,
            policy=replace(value.policy, goal=replace(value.policy.goal, dwell=dwell)),
        )
    with pytest.raises(RuntimeFault) as error:
        run.start_playback(value)
    assert error.value.code == (
        FaultCode.UNSUPPORTED if missing == "support" else FaultCode.INVALID_REQUEST
    )
    assert session._managed.accepted == 0


def test_default_goal_wire_omits_independent_dwell():
    motion = resting_request(False)
    assert b"dwell" not in encode(motion.policy.goal)
    assert decode(encode(motion)) == motion
