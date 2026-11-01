"""Completion uses local measurements, even when the remote caller is silent."""

import math
import time
from dataclasses import replace

import numpy as np
import pytest
from test_finite_playback import motion
from test_finite_playback import opened as opened  # noqa: PLC0414
from waddle_sdk.runtime import (
    FaultCode,
    PlaybackFeedbackPolicy,
    PlaybackGoal,
    Pose,
    PositionWindow,
    RuntimeFault,
)
from waddle_sdk_relay.wire import decode, encode


def with_goal(request, goal, **policy):
    return replace(
        request,
        policy=replace(
            request.policy,
            goal=goal,
            settle_timeout_ns=300_000_000,
            feedback=PlaybackFeedbackPolicy(
                position_window=PositionWindow(
                    (0.0005, 0.0005),
                    max_gap_ns=150_000_000,
                )
            ),
            **policy,
        ),
    )


def test_original_joint_goal_finishes_without_remote_observation(opened, monkeypatch):
    session, run = opened
    driver = session._managed.arms["arm"].driver
    write = driver.write
    monkeypatch.setattr(driver, "write", lambda q: write(np.asarray(q) * 0.5))
    request = with_goal(
        motion((0, 0), (0.06, 0.02)), PlaybackGoal(joint_positions=(0.03, 0.01))
    )
    handle = run.start_playback(decode(encode(request)))
    # No polling/renewals/finish RPCs while execution and its settling complete.
    time.sleep(0.35)
    result = handle.snapshot()
    assert result["status"] == "arrived", result
    assert result["feedback"]["positions"] == pytest.approx([0.03, 0.01])
    assert result["settling"]["span_ns"] >= request.policy.settle_ns
    assert result["dispatched"] == 2
    assert handle.snapshot() == result


def install_fk(session, monkeypatch):
    def fk(q):
        c, s = math.cos(q[1]), math.sin(q[1])
        return np.array([q[0], q[1], 0]), np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    monkeypatch.setattr(session._managed.arms["arm"], "fk", fk)


@pytest.mark.parametrize("position_only", [False, True])
def test_cartesian_completion_uses_local_tcp_and_requested_orientation(
    opened, monkeypatch, position_only
):
    session, run = opened
    install_fk(session, monkeypatch)
    goal = PlaybackGoal(
        pose=Pose((0.04, 0.03, 0), (1, 0, 0, 0), "cell"),
        orientation_tolerance_rad=None if position_only else 0.001,
    )
    request = with_goal(motion((0, 0), (0.04, 0.03)), goal, allow_settled=True)
    result = run.start_playback(decode(encode(request))).wait()
    assert result["status"] == ("arrived" if position_only else "settled"), result
    assert result["feedback"]["pose"]["position_m"] == pytest.approx([0.04, 0.03, 0])
    assert result["feedback"]["pose"]["frame_id"] == "cell"


def test_cartesian_goal_refuses_wrong_frame_before_dispatch(opened, monkeypatch):
    session, run = opened
    install_fk(session, monkeypatch)
    request = with_goal(
        motion(), PlaybackGoal(pose=Pose((0, 0, 0), (1, 0, 0, 0), "other"))
    )
    with pytest.raises(RuntimeFault) as error:
        run.start_playback(request)
    assert error.value.code == FaultCode.INVALID_REQUEST
    assert session._managed.accepted == 0


def test_joint_endpoint_is_not_cartesian_arrival(opened, monkeypatch):
    session, run = opened
    install_fk(session, monkeypatch)
    request = with_goal(
        motion((0, 0), (0.04, 0)),
        PlaybackGoal(
            pose=Pose((0.1, 0, 0), (1, 0, 0, 0), "cell"),
        ),
    )
    result = run.start_playback(request).wait()
    assert result["status"] == "failed", result
    assert result["failure"]["code"] == "deadline"
    assert result["cleanup"] is not None
    assert (
        result["feedback"]["observed_ns"] - result["last_acknowledged_ns"]
        < request.policy.timeout_ns
    )


def test_absent_goal_preserves_old_wire_policy():
    encoded = encode(motion())
    assert b"settle_timeout_ns" not in encoded
    assert b"PlaybackGoal" not in encoded
    assert decode(encoded) == motion()
