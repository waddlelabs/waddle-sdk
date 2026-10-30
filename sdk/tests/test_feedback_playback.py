"""Measured execution policies through the actual native worker and owner."""

from dataclasses import replace

import pytest
import site_fixtures
from test_finite_playback import motion
from test_finite_playback import opened as opened  # noqa: PLC0414 -- pytest fixture
from waddle_sdk.robots.base import SimDriver
from waddle_sdk.runtime import (
    FaultCode,
    HeldJointGroup,
    PlaybackFeedbackPolicy,
    PositionWindow,
    RuntimeFault,
    SupportFact,
)


def selected_motion(*, measured=False):
    request = motion((0, 0.5), (0.04, 0.5), (0.08, 0.5), (0.12, 0.5), feedforward=True)
    feedback = (
        PlaybackFeedbackPolicy(measured_indices=(1,))
        if measured
        else PlaybackFeedbackPolicy(
            held_groups=(HeldJointGroup((1,), (0.2,), 50_000_000),)
        )
    )
    return replace(
        request,
        policy=replace(
            request.policy,
            indices=(0,),
            start_tolerance=(0.01,),
            position_tolerance=(0.001,),
            stationary_velocity=(0.01,),
            feedback=feedback,
        ),
    )


def test_held_coordinate_compensation_passes_ordinary_owner_limits(opened):
    session, run = opened
    assert SupportFact.FEEDBACK_JOINT_PLAYBACK in session.support().rows[0].facts
    result = run.start_playback(selected_motion()).wait()
    assert result["status"] == "arrived", result
    targets = [q.tolist() for q, _ in site_fixtures.velocity_commands]
    assert targets[0] == pytest.approx([0.0, 0.18])
    assert targets[-1] == [0.12, 0.5]
    assert session._managed.rejected == 0


def test_measured_coordinate_keeps_current_position_instead_of_old_target(opened):
    _, run = opened
    result = run.start_playback(selected_motion(measured=True)).wait()
    assert result["status"] == "arrived", result
    assert all(q[1] == 0 for q, _ in site_fixtures.velocity_commands)


def test_position_window_uses_encoder_stability_and_exposes_raw_velocity(
    opened, monkeypatch
):
    _, run = opened
    original = SimDriver.read

    def read(driver):
        q, velocity = original(driver)
        velocity[:] = 0.5
        return q, velocity

    monkeypatch.setattr(SimDriver, "read", read)
    request = motion((0, 0), (0.1, -0.05))
    request = replace(
        request,
        policy=replace(
            request.policy,
            feedback=PlaybackFeedbackPolicy(
                position_window=PositionWindow((0.0005, 0.0005), consecutive_samples=3),
            ),
        ),
    )
    result = run.start_playback(request).wait()
    assert result["status"] == "arrived", result
    assert result["settling"]["raw_velocity_peak"] == 0.5
    assert result["settling"]["consecutive_samples"] == 3
    assert result["settling"]["span_ns"] >= request.policy.settle_ns


@pytest.mark.parametrize(
    "feedback",
    [
        PlaybackFeedbackPolicy(measured_indices=(0,)),
        PlaybackFeedbackPolicy(velocity_hints="unknown"),
        PlaybackFeedbackPolicy(
            held_groups=(HeldJointGroup((1,), (float("nan"),), 50_000_000),)
        ),
        PlaybackFeedbackPolicy(position_window=PositionWindow(())),
        PlaybackFeedbackPolicy(position_window=PositionWindow((0.001,), max_gap_ns=-1)),
        PlaybackFeedbackPolicy(
            position_window=PositionWindow((0.001,), consecutive_samples=0)
        ),
    ],
)
def test_feedback_admission_rejects_invalid_policy_before_io(opened, feedback):
    session, run = opened
    request = selected_motion()
    request = replace(request, policy=replace(request.policy, feedback=feedback))
    with pytest.raises(RuntimeFault) as error:
        run.start_playback(request)
    assert error.value.code == FaultCode.INVALID_REQUEST
    assert session._managed.accepted == 0


def test_feedback_policy_requires_its_separate_support_fact(opened, monkeypatch):
    session, run = opened
    matrix = session.support()
    matrix = replace(
        matrix,
        rows=tuple(
            replace(
                row,
                facts=tuple(
                    f for f in row.facts if f != SupportFact.FEEDBACK_JOINT_PLAYBACK
                ),
            )
            for row in matrix.rows
        ),
    )
    monkeypatch.setattr(session, "support", lambda: matrix)
    with pytest.raises(RuntimeFault) as error:
        run.start_playback(selected_motion())
    assert error.value.code == FaultCode.UNSUPPORTED
    assert session._managed.accepted == 0
