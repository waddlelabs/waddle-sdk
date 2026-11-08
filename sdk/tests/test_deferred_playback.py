"""Finite references retain their native sampler until explicit normal finish."""

import threading
import time
from dataclasses import replace

import pytest
from test_finite_playback import opened as opened  # noqa: PLC0414
from test_paced_playback import request
from waddle_sdk.runtime import (
    DeferredPlaybackHandlePort,
    FaultCode,
    PlaybackFeedbackPolicy,
    PositionWindow,
    RuntimeFault,
    SupportFact,
)


def motion():
    original = request()
    return replace(
        original,
        policy=replace(
            original.policy,
            reference_only=True,
            defer_reference_completion=True,
            allow_settled=True,
            settle_ns=100_000_000,
            feedback=PlaybackFeedbackPolicy(
                position_window=PositionWindow(
                    (0.001, 0.001), max_gap_ns=150_000_000, during_motion=True
                )
            ),
        ),
    )


def endpoint(handle):
    until = time.monotonic() + 2
    while time.monotonic() < until:
        row = handle.snapshot()
        assert row["status"] == "running", row
        if row["curve_progress"] and row["curve_progress"]["endpoint_acknowledged"]:
            return row
        time.sleep(0.01)
    pytest.fail("endpoint not acknowledged")


def test_deferred_sampler_remains_reserved_without_replaying_motion(opened):
    session, run = opened
    assert SupportFact.DEFERRED_REFERENCE_COMPLETION in session.support().rows[0].facts
    handle = run.start_playback(motion())
    assert isinstance(handle, DeferredPlaybackHandlePort)
    ready = endpoint(handle)
    with pytest.raises(RuntimeFault) as caught:
        run.start_playback(motion())
    assert caught.value.code == FaultCode.BUSY
    until = time.monotonic() + 2
    while True:
        local = handle.snapshot()  # Only reads retained diagnostics, never the owner.
        window = local["settling"]
        if window and window["samples"] >= 3 and window["span_ns"] >= 100_000_000:
            break
        assert time.monotonic() < until, local
        time.sleep(0.01)
    assert local["status"] == "running"
    assert local["dispatched"] == ready["dispatched"]
    assert local["last_acknowledged_ns"] == ready["last_acknowledged_ns"]
    assert local["settling"]["samples"] >= 3
    assert local["settling"]["span_ns"] >= 100_000_000
    handle.finish_references()
    terminal = handle.wait()
    assert terminal["status"] == "references_dispatched"
    assert terminal["cleanup"] is None
    assert terminal["settling"]["observed_ns"] == terminal["feedback"]["observed_ns"]
    assert terminal["feedback"]["observed_ns"] > local["feedback"]["observed_ns"]
    assert terminal["dispatched"] == ready["dispatched"]
    handle.finish_references()  # A terminal finish cannot replay any work.
    assert handle.snapshot() == terminal


@pytest.mark.parametrize("stop", ["cancel", "deadline"])
def test_deferred_reference_keeps_original_stop_and_deadline(opened, stop):
    _, run = opened
    source = motion()
    source = replace(source, policy=replace(source.policy, timeout_ns=700_000_000))
    handle = run.start_playback(source)
    endpoint(handle)
    if stop == "cancel":
        handle.cancel("local test stop")
    terminal = handle.wait()
    assert terminal["status"] == "failed"
    assert terminal["failure"]["code"] == (
        "cancelled" if stop == "cancel" else "deadline"
    )
    assert terminal["cleanup"]["outcome"] == "acknowledged"
    with pytest.raises(RuntimeFault):
        handle.finish_references()
    assert handle.snapshot() == terminal


def test_finish_before_endpoint_is_refused_without_advancing_motion(
    opened, monkeypatch
):
    from waddle_sdk.robots import base

    _, run = opened
    entered, release = threading.Event(), threading.Event()
    original = base.apply_decision

    def blocked(*args, **kwargs):
        entered.set()
        assert release.wait(5)
        return original(*args, **kwargs)

    monkeypatch.setattr(base, "apply_decision", blocked)
    handle = run.start_playback(motion())
    try:
        assert entered.wait(3)
        with pytest.raises(RuntimeFault) as caught:
            handle.finish_references()
        assert caught.value.code == FaultCode.INVALID_REQUEST
        assert handle.snapshot()["dispatched"] == 0
        handle.cancel("finish cannot skip unacknowledged references")
    finally:
        release.set()
    terminal = handle.wait()
    assert terminal["failure"]["code"] == "cancelled"
    assert terminal["cleanup"]["outcome"] == "acknowledged"
    assert terminal["dispatched"] == 1


@pytest.mark.parametrize("cancel", [False, True])
def test_finish_waits_for_inflight_feedback_and_preserves_cancellation(
    opened, monkeypatch, cancel
):
    session, run = opened
    handle = run.start_playback(motion())
    ready = endpoint(handle)
    entered, release = threading.Event(), threading.Event()
    original = session.observe_parts

    def blocked(*args, **kwargs):
        entered.set()
        assert release.wait(5)
        return original(*args, **kwargs)

    monkeypatch.setattr(session, "observe_parts", blocked)
    try:
        assert entered.wait(3)
        handle.finish_references()
        assert handle.snapshot()["status"] == "running"
        if cancel:
            handle.cancel("stop before finish callback returns")
        with pytest.raises(RuntimeFault) as caught:
            run.start_playback(motion())
        assert caught.value.code == FaultCode.BUSY
    finally:
        release.set()
    terminal = handle.wait()
    assert terminal["dispatched"] == ready["dispatched"]
    if cancel:
        assert terminal["failure"]["code"] == "cancelled"
        assert terminal["cleanup"]["outcome"] == "acknowledged"
    else:
        assert terminal["status"] == "references_dispatched"
        assert terminal["feedback"]["observed_ns"] > ready["feedback"]["observed_ns"]
        assert terminal["cleanup"] is None


def test_default_policy_remains_decodable_without_deferred_completion():
    from waddle_sdk_relay.wire import decode, encode

    source = request()
    encoded = encode(source)
    assert "defer_reference_completion" not in repr(encoded)
    assert decode(encoded) == source
    enabled = motion()
    assert decode(encode(enabled)) == enabled


def test_deferred_completion_requires_its_declared_support(opened, monkeypatch):
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
                        f
                        for f in row.facts
                        if f != SupportFact.DEFERRED_REFERENCE_COMPLETION
                    ),
                )
                for row in support.rows
            ),
        ),
    )
    with pytest.raises(RuntimeFault) as caught:
        run.start_playback(motion())
    assert caught.value.code == FaultCode.UNSUPPORTED
