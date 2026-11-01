"""Selected TCP data is evaluated with local feedback, without a planner."""

import math
from dataclasses import replace

import pytest
from test_finite_playback import motion
from test_finite_playback import opened as opened  # noqa: PLC0414
from test_goal_playback import with_goal
from waddle_sdk.runtime import (
    FaultCode,
    PlaybackGoal,
    PlaybackKinematicJoint,
    PlaybackKinematics,
    Pose,
    RuntimeFault,
    SupportFact,
)
from waddle_sdk_relay.wire import decode, encode


def request():
    angle = 0.03
    goal = PlaybackGoal(
        pose=Pose(
            (0.04 + 0.2 * math.cos(angle), 0.2 * math.sin(angle), 0),
            (math.cos(angle / 2), 0, 0, math.sin(angle / 2)),
            "cell",
        ),
        kinematics=PlaybackKinematics(
            Pose((0.2, 0, 0), (1, 0, 0, 0), "cell"),
            (
                PlaybackKinematicJoint(0, "slide", (1, 0, 0)),
                PlaybackKinematicJoint(1, "hinge", (0, 0, 1)),
            ),
        ),
    )
    return with_goal(motion((0, 0), (0.04, angle)), goal)


def test_selected_tool_completes_without_owner_fk_or_remote_polling(opened):
    session, run = opened
    session._managed.arms["arm"].fk = None
    value = decode(encode(request()))
    handle = run.start_playback(value)
    result = handle.wait()
    assert result["status"] == "arrived", result
    assert result["feedback"]["pose"]["position_m"] == pytest.approx(
        value.policy.goal.pose.position_m
    )
    assert result["feedback"]["pose"]["quaternion_wxyz"] == pytest.approx(
        value.policy.goal.pose.quaternion_wxyz
    )
    assert result["settling"]["span_ns"] >= value.policy.settle_ns
    assert handle.snapshot() == result


@pytest.mark.parametrize(
    "change",
    [
        {"index": 2},
        {"index": True},
        {"axis": (0, 0, 0)},
        {"axis": (0, 0, 2)},
        {"axis": (0, float("nan"), 1)},
        {"origin_m": (0, 0)},
        {"kind": "callback"},
        {"scale": float("inf")},
    ],
)
def test_invalid_chain_is_refused_before_dispatch(opened, change):
    session, run = opened
    value = request()
    goal = value.policy.goal
    chain = goal.kinematics
    bad = replace(chain.joints[0], **change)
    value = replace(
        value,
        policy=replace(
            value.policy,
            goal=replace(
                goal, kinematics=replace(chain, joints=(bad, *chain.joints[1:]))
            ),
        ),
    )
    with pytest.raises(RuntimeFault) as error:
        run.start_playback(value)
    assert error.value.code == FaultCode.INVALID_REQUEST
    assert session._managed.accepted == 0


def test_selected_tool_requires_its_support_fact(opened, monkeypatch):
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
                        f for f in row.facts if f != SupportFact.KINEMATIC_GOAL_PLAYBACK
                    ),
                )
                for row in support.rows
            ),
        ),
    )
    with pytest.raises(RuntimeFault) as error:
        run.start_playback(request())
    assert error.value.code == FaultCode.UNSUPPORTED
    assert session._managed.accepted == 0


def test_default_tool_omits_new_wire_field():
    value = PlaybackGoal(pose=Pose((0, 0, 0), (1, 0, 0, 0), "cell"))
    assert b"kinematics" not in encode(value)
    assert decode(encode(value)) == value
