"""One SDK execution retains its goal through corrections and local queue handoff."""

from dataclasses import replace
from math import cos, sin
from threading import Event

import numpy as np
import pytest
from test_finite_playback import motion
from test_finite_playback import opened as opened  # noqa: PLC0414
from test_goal_playback import install_fk
from test_paced_playback import request
from waddle_sdk.runtime import (
    FaultCode,
    PlaybackCurveUpdate,
    PlaybackDependency,
    PlaybackFeedbackPolicy,
    PlaybackGoal,
    Pose,
    PositionWindow,
    RuntimeFault,
    SupportFact,
)
from waddle_sdk_relay.wire import decode, encode


def goal_request(cartesian=False):
    value = request()
    goal = (
        PlaybackGoal(
            pose=Pose((0.1, -0.05, 0), (cos(-0.025), 0, 0, sin(-0.025)), "cell")
        )
        if cartesian
        else PlaybackGoal(joint_positions=(0.1, -0.05))
    )
    return replace(
        value,
        policy=replace(
            value.policy,
            goal=goal,
            allow_curve_updates=True,
            timeout_ns=30_000_000_000,
            feedback=PlaybackFeedbackPolicy(
                position_window=PositionWindow((0.0005, 0.0005), max_gap_ns=500_000_000)
            ),
        ),
    )


def correction(value):
    curve = value.policy.paced_curve
    start, delta = curve.positions[-1], (0.02, 0.0)
    return PlaybackCurveUpdate(
        0,
        200_000_000,
        replace(
            curve,
            positions=(start, tuple(a + b for a, b in zip(start, delta))),
            coefficients=(
                (start, (0, 0), (0, 0))
                + tuple(tuple(scale * q for q in delta) for scale in (10, -15, 6)),
            ),
        ),
    )


@pytest.mark.parametrize("cartesian", [False, True])
def test_correction_keeps_original_goal_then_sdk_starts_queued_successor(
    opened, monkeypatch, cartesian
):
    session, run = opened
    install_fk(session, monkeypatch)
    driver = session._managed.arms["arm"].driver
    write, read = driver.write, driver.read
    endpoint = Event()
    writes = []

    def biased_write(q):
        writes.append(tuple(q))
        write(np.asarray(q) - (0.02, 0))

    def biased_feedforward(q, _velocity):
        biased_write(q)
        return True

    def observed_read():
        if np.allclose(driver._target, (0.08, -0.05)) and not endpoint.is_set():
            endpoint.set()
        return read()

    monkeypatch.setattr(driver, "write", biased_write)
    monkeypatch.setattr(driver, "write_position_velocity", biased_feedforward)
    monkeypatch.setattr(driver, "read", observed_read)
    value = goal_request(cartesian)
    first = run.start_playback(decode(encode(value)))
    # Fixed command bias keeps the goal unsatisfied until a checked correction.
    assert endpoint.wait(5), (first.snapshot(), writes, driver._target)
    child = motion((0.12, -0.05), (0.14, -0.05))
    child = replace(
        child,
        policy=replace(
            child.policy,
            start_tolerance=(0.03, 0.03),
            goal=PlaybackGoal(joint_positions=(0.12, -0.05)),
        ),
    )
    second = run.start_playback(child, after=[PlaybackDependency(first, ("arrived",))])
    assert second.snapshot()["waiting_for"] == [first.snapshot()["id"]]
    assert first.update_curve(decode(encode(correction(value)))) == 1
    # No parent polling/finish call drives correction, settling or the handoff.
    result = second.wait()
    assert result["status"] == "arrived", result
    parent = first.snapshot()
    assert parent["status"] == "arrived", parent
    assert parent["curve_progress"]["revision"] == 1
    assert parent["curve_update"]["status"] == "accepted"
    assert parent["feedback"]["positions"] == pytest.approx((0.1, -0.05))
    assert parent["last_reference"]["positions"] == pytest.approx((0.12, -0.05))
    assert parent["settling"]["span_ns"] >= value.policy.settle_ns
    assert parent["cleanup"] is None
    assert writes[-2:] == [(0.12, -0.05), (0.14, -0.05)]
    assert first.snapshot() == parent


def test_revisable_goal_requires_its_own_support_fact_before_dispatch(
    opened, monkeypatch
):
    session, run = opened
    support = session.support()
    assert SupportFact.REVISABLE_GOAL_PLAYBACK in support.rows[0].facts
    monkeypatch.setattr(
        session,
        "support",
        lambda: replace(
            support,
            rows=tuple(
                replace(
                    row,
                    facts=tuple(
                        f for f in row.facts if f != SupportFact.REVISABLE_GOAL_PLAYBACK
                    ),
                )
                for row in support.rows
            ),
        ),
    )
    with pytest.raises(RuntimeFault) as error:
        run.start_playback(goal_request())
    assert error.value.code == FaultCode.UNSUPPORTED
    assert session._managed.accepted == 0
