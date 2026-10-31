"""Public liveness receipts over the actual native owner, without hardware."""

import threading
from dataclasses import replace

import pytest
from test_finite_playback import motion
from test_finite_playback import opened as opened  # noqa: PLC0414 -- pytest fixture
from waddle_sdk.runtime import (
    FaultCode,
    PlaybackWatchdog,
    RuntimeFault,
    SupportFact,
    WatchdogPlaybackHandlePort,
)


def request(watchdog):
    value = motion((0, 0), (0.02, -0.01))
    return replace(value, policy=replace(value.policy, watchdog=watchdog))


def test_expired_playback_never_writes_and_retains_native_hold(opened):
    session, run = opened
    assert SupportFact.WATCHDOG_PLAYBACK in session.support().rows[0].facts
    handle = run.start_playback(request(PlaybackWatchdog(0, 1)))
    assert isinstance(handle, WatchdogPlaybackHandlePort)
    result = handle.wait()
    assert result["failure"]["code"] == "watchdog_expired", result
    assert result["dispatched"] == 0
    assert result["cleanup"]["error"] is None
    assert result["watchdog"] == {"observed_ns": 0, "timeout_ns": 1, "deadline_ns": 1}
    with pytest.raises(RuntimeFault) as caught:
        handle.refresh_watchdog(session.observe().session_ns)
    assert caught.value.code == FaultCode.INVALID_REQUEST


def test_watchdog_requires_separate_support_and_valid_native_bounds(
    opened, monkeypatch
):
    session, run = opened
    support = session.support()
    for watchdog in (
        PlaybackWatchdog(-1, 100),
        PlaybackWatchdog(0, 0),
        PlaybackWatchdog(0, 3_000_000_000),
    ):
        with pytest.raises(RuntimeFault) as caught:
            run.start_playback(request(watchdog))
        assert caught.value.code == FaultCode.INVALID_REQUEST
    monkeypatch.setattr(
        session,
        "support",
        lambda: replace(
            support,
            rows=tuple(
                replace(
                    row,
                    facts=tuple(
                        f for f in row.facts if f != SupportFact.WATCHDOG_PLAYBACK
                    ),
                )
                for row in support.rows
            ),
        ),
    )
    with pytest.raises(RuntimeFault) as caught:
        run.start_playback(request(PlaybackWatchdog(0, 1)))
    assert caught.value.code == FaultCode.UNSUPPORTED
    assert session._managed.accepted == 0


def test_watchdog_renewal_is_pending_until_worker_accepts_feedback(opened, monkeypatch):
    session, run = opened
    driver = session._managed.arms["arm"].driver
    original = driver.read
    entered, release = threading.Event(), threading.Event()

    def blocked():
        entered.set()
        assert release.wait(5)
        return original()

    stamp = session.observe().session_ns
    value = request(PlaybackWatchdog(stamp, 30_000_000_000))
    value = replace(value, policy=replace(value.policy, timeout_ns=30_000_000_000))
    monkeypatch.setattr(driver, "read", blocked)
    handle = run.start_playback(value)
    try:
        assert entered.wait(5)
        handle.refresh_watchdog(stamp)
        assert handle.snapshot()["watchdog_refresh"] == {
            "observed_ns": stamp,
            "status": "pending",
            "deadline_ns": None,
            "error": None,
        }
        handle.refresh_watchdog(stamp)
        with pytest.raises(RuntimeFault) as caught:
            handle.refresh_watchdog(stamp + 1)
        assert caught.value.code == FaultCode.BUSY
    finally:
        release.set()
    result = handle.wait()
    assert result["status"] == "arrived", result
    assert result["watchdog_refresh"] == {
        "observed_ns": stamp,
        "status": "accepted",
        "deadline_ns": stamp + 30_000_000_000,
        "error": None,
    }
