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
