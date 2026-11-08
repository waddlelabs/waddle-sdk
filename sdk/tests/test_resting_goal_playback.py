"""Native goal completion replaces a caller's constant-tail finish decision."""

from dataclasses import replace

import pytest
from test_finite_playback import opened as opened  # noqa: PLC0414
from test_revisable_goal_playback import goal_request
from waddle_sdk.runtime import (
    FaultCode,
    PlaybackDependency,
    PlaybackFeedbackPolicy,
    RuntimeFault,
    SupportFact,
)
from waddle_sdk_relay.wire import decode, encode


def resting_request(window=True):
    value = goal_request()
    curve = value.policy.paced_curve
    target = curve.positions[-1]
    curve = replace(
        curve,
        times_s=(*curve.times_s, 10.0),
        positions=(*curve.positions, target),
        coefficients=(*curve.coefficients, (target, *((0.0, 0.0),) * 5)),
    )
    return replace(
        value,
        references=(
            value.references[0],
            replace(value.references[-1], offset_ns=10_000_000_000),
        ),
        policy=replace(
            value.policy,
            paced_curve=curve,
            complete_in_resting_tail=True,
            minimum_settle_samples=3,
            feedback=value.policy.feedback if window else PlaybackFeedbackPolicy(),
        ),
    )


@pytest.mark.parametrize("window", [False, True])
def test_local_resting_goal_releases_successor_without_finish_calls(opened, window):
    from test_finite_playback import motion

    _, run = opened
    first = run.start_playback(decode(encode(resting_request(window))))
    second = run.start_playback(
        motion((0.1, -0.05), (0.12, -0.05)),
        after=(PlaybackDependency(first, ("arrived",)),),
    )
    result = second.wait()
    assert result["status"] == "arrived", result
    original = first.snapshot()
    assert original["status"] == "arrived", original
    assert original["curve_progress"]["acknowledged_curve_time_s"] == pytest.approx(0.2)
    assert not original["curve_progress"]["endpoint_acknowledged"]
    first.cancel("late request cannot change local completion or its successor")
    assert first.snapshot() == original
    assert second.snapshot() == result


@pytest.mark.parametrize("missing", ["goal", "support"])
def test_resting_completion_requires_explicit_supported_goal(
    opened, monkeypatch, missing
):
    session, run = opened
    value = resting_request()
    if missing == "goal":
        value = replace(value, policy=replace(value.policy, goal=None))
    else:
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
                            if f != SupportFact.RESTING_GOAL_PLAYBACK
                        ),
                    )
                    for row in support.rows
                ),
            ),
        )
    with pytest.raises(RuntimeFault) as error:
        run.start_playback(value)
    assert error.value.code == (
        FaultCode.INVALID_REQUEST if missing == "goal" else FaultCode.UNSUPPORTED
    )
    assert session._managed.accepted == 0


def test_default_wire_policy_keeps_rest_duration():
    value = goal_request()
    assert b"complete_in_resting_tail" not in encode(value)
    assert decode(encode(value)) == value
