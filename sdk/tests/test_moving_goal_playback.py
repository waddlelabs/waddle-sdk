"""Accepted targets and completion remain local across target-message delays."""

from dataclasses import replace
from threading import Event
from time import monotonic
from types import SimpleNamespace

import pytest
from test_finite_playback import motion
from test_finite_playback import opened as opened  # noqa: PLC0414
from test_goal_dwell_playback import request
from test_goal_playback import install_fk
from waddle_sdk.runtime import (
    FaultCode,
    PlaybackDependency,
    PlaybackGoal,
    PlaybackGoalDwell,
    PlaybackGoalUpdate,
    Pose,
    RuntimeFault,
    SupportFact,
)
from waddle_sdk_relay.device import Device
from waddle_sdk_relay.errors import RelayError
from waddle_sdk_relay.wire import decode, encode


def target(x):
    return Pose((x, -0.05, 0.0), (1.0, 0.0, 0.0, 0.0), "cell")


def moving_request(window=False):
    value = request(window)
    return replace(
        value,
        policy=replace(
            value.policy,
            goal=PlaybackGoal(
                pose=target(0.15),
                orientation_tolerance_rad=None,
                dwell=PlaybackGoalDwell(300_000_000),
                allow_updates=True,
            ),
        ),
    )


def until(handle, predicate):
    deadline = monotonic() + 5
    while not predicate(value := handle.snapshot()):
        assert value["status"] == "running", value
        assert monotonic() < deadline, value
        Event().wait(0.005)
    return value


@pytest.mark.parametrize("relay", [False, True])
@pytest.mark.parametrize("window", [False, True])
def test_accepted_target_completes_and_releases_queue_without_cloud_finish(
    opened, monkeypatch, tmp_path, relay, window
):
    session, run = opened
    install_fk(session, monkeypatch)
    value = moving_request(window)
    device = Device(SimpleNamespace(), tmp_path)
    device.sdk, device.run = session, run
    if relay:
        initial = device.dispatch(
            "playback_start", {"run_id": run.id, "motion": decode(encode(value))}
        )
        first = device._playbacks[(run.id, initial["id"])]
    else:
        first = run.start_playback(value)
    progress = until(first, lambda v: v["settled_miss"])
    assert progress["goal_target"]["revision"] == 0
    assert progress["goal_target"]["pose"]["position_m"] == pytest.approx(
        target(0.15).position_m
    )
    second = run.start_playback(
        motion((0.1, -0.05), (0.12, -0.05)),
        after=(PlaybackDependency(first, ("arrived",)),),
    )
    update = decode(encode(PlaybackGoalUpdate(0, target(0.1))))
    if relay:
        args = {"run_id": run.id, "id": progress["id"], "update": update}
        with pytest.raises(RelayError, match="retained run"):
            device.dispatch("playback_goal_update", {**args, "run_id": "foreign"})
        reply = device.dispatch("playback_goal_update", args)
        assert reply["request_id"] == 1
    else:
        assert first.update_goal(update) == 1
    result = second.wait()
    assert result["status"] == "arrived", result
    parent = decode(encode(first.snapshot()))
    assert parent["status"] == "arrived", parent
    assert parent["goal_update"]["status"] == "accepted"
    assert parent["goal_update"]["revision"] == parent["goal_target"]["revision"] == 1
    assert parent["goal_target"]["pose"]["position_m"] == pytest.approx(
        target(0.1).position_m
    )
    assert parent["goal_dwell"]["observed_ns"] == parent["feedback"]["observed_ns"]
    assert (
        parent["goal_dwell"]["observed_ns"] - parent["goal_dwell"]["since_ns"]
        >= 300_000_000
    )
    assert not parent["curve_progress"]["endpoint_acknowledged"]
    with pytest.raises(RuntimeFault):
        first.update_goal(PlaybackGoalUpdate(1, target(0.2)))
    first.cancel("late cancellation")
    assert decode(encode(first.snapshot())) == parent


@pytest.mark.parametrize("invalid", ["revision", "frame", "fixed"])
def test_refused_goal_update_preserves_target_and_normal_cancel(
    opened, monkeypatch, invalid
):
    session, run = opened
    install_fk(session, monkeypatch)
    value = moving_request()
    if invalid == "fixed":
        value = replace(
            value,
            policy=replace(
                value.policy, goal=replace(value.policy.goal, allow_updates=False)
            ),
        )
    handle = run.start_playback(value)
    progress = until(handle, lambda v: v["settled_miss"])
    pose = target(0.1)
    if invalid == "frame":
        pose = replace(pose, frame_id="foreign")
    handle.update_goal(PlaybackGoalUpdate(1 if invalid == "revision" else 0, pose))
    refused = until(handle, lambda v: v["goal_update"]["status"] == "rejected")
    assert refused["goal_target"] == progress["goal_target"]
    handle.cancel("stop after refused update")
    result = handle.wait()
    assert result["failure"]["code"] == "cancelled", result
    assert result["cleanup"]["outcome"] == "acknowledged"
    assert result["cleanup"]["completed_ns"] >= result["feedback"]["observed_ns"]
    assert result["goal_target"] == progress["goal_target"]


@pytest.mark.parametrize("missing", ["support", "dwell", "pose"])
def test_mutable_goal_requires_its_explicit_contract(opened, monkeypatch, missing):
    session, run = opened
    install_fk(session, monkeypatch)
    value = moving_request()
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
                            f
                            for f in row.facts
                            if f != SupportFact.UPDATABLE_GOAL_PLAYBACK
                        ),
                    )
                    for row in support.rows
                ),
            ),
        )
    else:
        goal = (
            replace(value.policy.goal, dwell=None)
            if missing == "dwell"
            else replace(value.policy.goal, pose=None, joint_positions=(0.1, -0.05))
        )
        value = replace(value, policy=replace(value.policy, goal=goal))
    with pytest.raises(RuntimeFault) as error:
        run.start_playback(value)
    assert error.value.code == (
        FaultCode.UNSUPPORTED if missing == "support" else FaultCode.INVALID_REQUEST
    )
    assert session._managed.accepted == 0


def test_default_goal_wire_omits_target_mutability():
    goal = PlaybackGoal(joint_positions=(0.1,))
    assert b"allow_updates" not in encode(goal)
    assert decode(encode(goal)) == goal
