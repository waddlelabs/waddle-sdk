"""Measured start admission is independent of the first commanded reference."""

from dataclasses import replace

import numpy as np
import pytest
from test_finite_playback import motion
from test_finite_playback import opened as opened  # noqa: PLC0414


@pytest.mark.parametrize("drift", [0.0, 0.0015, -0.0015])
def test_distinct_expected_start_keeps_tolerance_and_command_limits(
    opened, monkeypatch, drift
):
    session, run = opened
    request = motion((-1.0, 0.0), (-0.995, 0.0))
    request = replace(
        request,
        policy=replace(
            request.policy,
            expected_start=(-1.002, 0.0),
            start_tolerance=(0.001, 0.001),
        ),
    )
    driver = session._managed.arms["arm"].driver
    measured, writes = np.array([-1.002 + drift, 0.0]), []

    def write(target):
        writes.append(tuple(target))
        measured[:] = target

    monkeypatch.setattr(driver, "read", lambda: (measured.copy(), np.zeros(2)))
    monkeypatch.setattr(driver, "write", write)
    result = run.start_playback(request).wait()
    if drift:
        assert result["failure"]["code"] == "stale_start", result
        assert not writes
        assert result["cleanup"]["outcome"] == "acknowledged"
    else:
        assert result["status"] == "arrived", result
        assert writes == [(-1.0, 0.0), (-0.995, 0.0)]
        assert all(-1 <= q <= 1 for target in writes for q in target)


@pytest.mark.parametrize("anchor", [(), (0,), (0, 0, 0), (float("nan"), 0)])
def test_expected_start_requires_the_complete_finite_layout(opened, anchor):
    from waddle_sdk.runtime import FaultCode, RuntimeFault

    _, run = opened
    request = motion()
    with pytest.raises(RuntimeFault) as caught:
        run.start_playback(
            replace(request, policy=replace(request.policy, expected_start=anchor))
        )
    assert caught.value.code == FaultCode.INVALID_REQUEST


def test_expected_start_preserves_wire_shape_and_requires_its_own_capability(
    opened, monkeypatch
):
    from waddle_sdk.runtime import FaultCode, RuntimeFault, SupportFact
    from waddle_sdk_relay.wire import decode, encode

    session, run = opened
    request = motion()
    assert b"expected_start" not in encode(request)
    request = replace(request, policy=replace(request.policy, expected_start=(0, 0)))
    assert decode(encode(request)).policy.expected_start == (0, 0)
    support = session.support()
    assert SupportFact.EXPECTED_START_PLAYBACK in support.rows[0].facts
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
                        if fact != SupportFact.EXPECTED_START_PLAYBACK
                    ),
                )
                for row in support.rows
            ),
        ),
    )
    with pytest.raises(RuntimeFault) as caught:
        run.start_playback(request)
    assert caught.value.code == FaultCode.UNSUPPORTED


def test_measured_start_anchor_never_waives_owner_target_limits(opened, monkeypatch):
    session, run = opened
    request = motion((-1.01, 0.0), (-0.995, 0.0))
    request = replace(
        request, policy=replace(request.policy, expected_start=(-1.002, 0.0))
    )
    driver = session._managed.arms["arm"].driver
    writes = []
    monkeypatch.setattr(driver, "read", lambda: (np.array([-1.002, 0.0]), np.zeros(2)))
    monkeypatch.setattr(driver, "write", lambda target: writes.append(tuple(target)))
    result = run.start_playback(request).wait()
    assert result["status"] == "failed", result
    assert result["dispatched"] == 0
    assert not writes
    assert result["cleanup"]["outcome"] == "acknowledged"
