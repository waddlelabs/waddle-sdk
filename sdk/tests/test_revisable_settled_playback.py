"""Revised report curves complete on the existing local worker, including misses."""

from dataclasses import replace
from threading import Event

import numpy as np
import pytest
from test_finite_playback import opened as opened  # noqa: PLC0414
from test_paced_playback import request
from test_revisable_goal_playback import correction
from waddle_sdk.runtime import (
    FaultCode,
    PlaybackFeedbackPolicy,
    PositionWindow,
    RuntimeFault,
    SupportFact,
)


def report(window=False):
    value = request()
    return replace(
        value,
        policy=replace(
            value.policy,
            allow_settled=True,
            allow_curve_updates=True,
            credit_pacing_delay=True,
            settle_ns=100_000_000 if window else 0,
            minimum_settle_samples=3 if window else 1,
            feedback=PlaybackFeedbackPolicy(
                position_window=PositionWindow((0.0005, 0.0005)) if window else None
            ),
        ),
    )


@pytest.mark.parametrize("window", [False, True])
@pytest.mark.parametrize("miss", [0.0, 0.01])
def test_revised_report_settles_locally_without_a_pose_goal(
    opened, monkeypatch, window, miss
):
    session, run = opened
    driver = session._managed.arms["arm"].driver
    entered, release = Event(), Event()

    def write(q, velocity):
        driver.write(np.asarray(q) - (miss, 0))
        if not entered.is_set():
            entered.set()
            assert release.wait(5)
        return True

    monkeypatch.setattr(driver, "write_position_velocity", write)
    value = report(window)
    handle = run.start_playback(value)
    try:
        assert entered.wait(3)
        request_id = handle.update_curve(correction(value))
    finally:
        release.set()
    result = handle.wait()
    assert result["status"] == ("settled" if miss else "arrived"), result
    assert result["feedback"]["positions"] == pytest.approx((0.12 - miss, -0.05))
    assert result["goal_target"] is None
    assert result["curve_progress"]["revision"] == 1
    assert result["curve_progress"]["endpoint_acknowledged"]
    assert result["curve_update"]["request_id"] == request_id
    assert result["curve_update"]["status"] == "accepted"
    handle.cancel("late cancellation")
    assert handle.wait() == result


def test_revised_settling_requires_its_advertised_capability(opened, monkeypatch):
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
                        fact
                        for fact in row.facts
                        if fact != SupportFact.REVISABLE_SETTLED_PLAYBACK
                    ),
                )
                for row in support.rows
            ),
        ),
    )
    with pytest.raises(RuntimeFault) as error:
        run.start_playback(report())
    assert error.value.code == FaultCode.UNSUPPORTED
