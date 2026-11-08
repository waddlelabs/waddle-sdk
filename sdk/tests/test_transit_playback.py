"""An admitted successor can waive intermediate settling on the same worker."""

from dataclasses import replace

import pytest
from test_dependent_playback import moving_owner
from test_finite_playback import motion
from test_finite_playback import opened as opened  # noqa: PLC0414
from waddle_sdk.runtime import PlaybackDependency, SupportFact


@pytest.mark.parametrize("mode", ["transit", "cancelled", "arrival_only"])
def test_only_an_admitted_uncancelled_successor_can_waive_settling(
    opened, monkeypatch, mode
):
    session, run = opened
    assert SupportFact.TRANSIT_PLAYBACK in session.support().rows[0].facts
    moving, wrote, _writes = moving_owner(session, monkeypatch)
    value = motion((0.0, 0.0), (0.04, 0.02))
    value = replace(
        value,
        references=(
            value.references[0],
            replace(value.references[1], offset_ns=500_000_000),
        ),
        policy=replace(value.policy, settle_ns=300_000_000),
    )
    parent = run.start_playback(value)
    try:
        assert wrote.wait(3)
        outcomes = (
            ("arrived",)
            if mode == "arrival_only"
            else ("arrived", "references_dispatched")
        )
        child = run.start_playback(
            motion((0.04, 0.02), (0.06, 0.02)),
            after=(PlaybackDependency(parent, outcomes),),
        )
        if mode == "cancelled":
            child.cancel("cancel prepared continuation")
        if mode == "transit":
            assert parent.wait()["status"] == "references_dispatched"
    finally:
        moving.clear()
    parent_result, child_result = parent.wait(), child.wait()
    assert parent_result["status"] == (
        "references_dispatched" if mode == "transit" else "arrived"
    )
    assert parent_result["transit_successor"] == (
        child_result["id"] if mode == "transit" else None
    )
    assert parent_result["cleanup"] is None
    if mode == "cancelled":
        assert child_result["failure"]["code"] == "cancelled"
        assert child_result["dispatched"] == 0
        assert child_result["cleanup"] is None
    else:
        assert child_result["status"] == "arrived", child_result
        assert child_result["feedback"]["positions"] == pytest.approx((0.06, 0.02))
