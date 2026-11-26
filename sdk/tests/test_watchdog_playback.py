"""Public liveness receipts over the actual native owner, without hardware."""

import json
import threading
import time
from dataclasses import replace

import pytest
from test_finite_playback import motion
from test_finite_playback import opened as native_site_fixture
from waddle_sdk.runtime import (
    FaultCode,
    PlaybackWatchdog,
    RuntimeFault,
    SupportFact,
    WatchdogPlaybackHandlePort,
)


opened = native_site_fixture


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
        PlaybackWatchdog(0, 180_000_000_001),
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


@pytest.mark.parametrize("boundary", ["gate", "driver"])
def test_delayed_dispatch_cannot_cross_deadline_or_resume(
    boundary, opened, monkeypatch, record_property, tmp_path
):
    from waddle_sdk.robots import base

    session, run = opened
    assert SupportFact.EMISSION_DEADLINE_PLAYBACK in session.support().rows[0].facts
    clock = session._managed.core.stamp
    arm = session._managed.arms["arm"]
    writes, holds = [], []
    original_write, original_hold = arm.driver.write, arm.driver.hold

    def write(target):
        writes.append(clock().session_ns)
        original_write(target)

    def hold():
        original_hold()
        holds.append(clock().session_ns)

    monkeypatch.setattr(arm.driver, "write", write)
    monkeypatch.setattr(arm.driver, "hold", hold)
    entered, release = threading.Event(), threading.Event()
    owner, method = (run, "_step") if boundary == "gate" else (base, "apply_decision")
    original = getattr(owner, method)

    def delayed(*args, **kwargs):
        if len(writes) == 1:
            entered.set()
            assert release.wait(5)
        return original(*args, **kwargs)

    monkeypatch.setattr(owner, method, delayed)
    stamp = session.observe().session_ns
    watchdog = PlaybackWatchdog(stamp, 300_000_000)
    deadline = stamp + watchdog.timeout_ns
    value = request(watchdog)
    handle = run.start_playback(value)
    try:
        assert entered.wait(2)
        remaining = (deadline - clock().session_ns) * 1e-9
        assert remaining > 0
        time.sleep(remaining + 0.02)
        late = clock().session_ns
        handle.refresh_watchdog(late)
        assert handle.snapshot()["watchdog_refresh"]["status"] == "pending"
    finally:
        release.set()
    result = handle.wait()
    assert result["failure"]["outcome_unknown"] is False, result
    assert "deadline expired" in result["failure"]["detail"], result
    assert result["watchdog"]["deadline_ns"] == deadline
    assert result["watchdog_refresh"]["status"] != "accepted"
    assert result["dispatched"] == len(writes) == 1
    assert max(writes) < deadline <= late
    assert result["cleanup"]["outcome"] == "acknowledged", result
    assert holds and late <= holds[-1] <= result["cleanup"]["completed_ns"]
    record_property(
        "emission_deadline",
        json.dumps(
            {
                "recording_root": str(tmp_path / "recordings"),
                "deadline_ns": deadline,
                "late_renewal_ns": late,
                "driver_position_ns": writes,
                "driver_hold_ns": holds,
                "watchdog_refresh": result["watchdog_refresh"],
                "failure": result["failure"],
                "cleanup": result["cleanup"],
            }
        ),
    )


def test_timely_renewal_carries_its_deadline_to_later_writes(
    opened, monkeypatch, record_property, tmp_path
):
    session, run = opened
    clock = session._managed.core.stamp
    driver = session._managed.arms["arm"].driver
    original = driver.write
    writes = []

    def write(target):
        writes.append(clock().session_ns)
        original(target)

    monkeypatch.setattr(driver, "write", write)
    value = motion(*[(0.001 * i, 0) for i in range(8)])
    age = 280_000_000
    first = session.observe().session_ns
    value = replace(
        value,
        policy=replace(
            value.policy, reference_only=True, watchdog=PlaybackWatchdog(first, age)
        ),
    )
    handle = run.start_playback(value)
    time.sleep(0.15)
    replacement = clock().session_ns
    assert replacement < first + age
    handle.refresh_watchdog(replacement)
    result = handle.wait()
    assert result["status"] == "references_dispatched", result
    assert result["watchdog_refresh"]["status"] == "accepted"
    assert result["watchdog"]["deadline_ns"] == replacement + age
    assert min(writes) < first + age <= max(writes) < replacement + age
    assert len(writes) == result["dispatched"] == 8
    record_property(
        "emission_deadline",
        json.dumps(
            {
                "recording_root": str(tmp_path / "recordings"),
                "deadline_ns": first + age,
                "renewed_deadline_ns": replacement + age,
                "driver_position_ns": writes,
                "watchdog_refresh": result["watchdog_refresh"],
            }
        ),
    )


def test_longer_proof_window_preserves_short_motion_timeout(opened, monkeypatch):
    session, run = opened
    driver = session._managed.arms["arm"].driver
    monkeypatch.setattr(driver, "write", lambda target: None)
    stamp = session.observe().session_ns
    value = request(PlaybackWatchdog(stamp, 3_000_000_000))
    value = replace(value, policy=replace(value.policy, timeout_ns=250_000_000))
    result = run.start_playback(value).wait()
    assert result["failure"]["code"] == "deadline", result
    assert result["cleanup"]["outcome"] == "acknowledged"
    assert result["cleanup"]["completed_ns"] < result["watchdog"]["deadline_ns"]


@pytest.mark.parametrize("feedforward", [False, True])
def test_yam_final_setpoint_respects_expiry_after_vendor_lock(
    opened, monkeypatch, feedforward, record_property
):
    """Pinned YAM publication code and a delayed vendor lock, with no CAN device."""
    from types import SimpleNamespace

    import numpy as np
    from waddle_sdk.robots import yam
    from waddle_sdk.robots._i2rt_patches import _command_joint_state_atomic

    session, run = opened
    clock = session._managed.core.stamp
    entered, release = threading.Event(), threading.Event()
    latches, holds = [], []

    class Commands:
        @classmethod
        def init_all_zero(cls, size):
            return cls()

    class CommandLock:
        def __enter__(self):
            if latches:
                entered.set()
                assert release.wait(5)

        def __exit__(self, *_args):
            pass

    class Vendor:
        command_joint_state = _command_joint_state_atomic
        _kp = _kd = np.ones(7)
        _commands = Commands()
        _command_lock = CommandLock()
        motor_chain = range(7)
        remapper = SimpleNamespace(
            to_robot_joint_pos_space=np.asarray, to_robot_joint_vel_space=np.asarray
        )
        _clip_robot_joint_pos_command = staticmethod(np.asarray)

        def __setattr__(self, name, value):
            if name == "_commands":
                latches.append(clock().session_ns)
            super().__setattr__(name, value)

        def command_joint_pos(self, target):
            holds.append(clock().session_ns)

    driver = yam.LiveDriver.__new__(yam.LiveDriver)
    driver._robot = Vendor()
    driver._lock = threading.RLock()
    driver._zero_gravity = driver._estopped = False
    driver._velocity_feedforward = driver._can_command_state = True
    driver._max_feedforward_vel_rad_s = 1.0
    driver.read = lambda: (np.zeros(7), np.zeros(7))
    fixture = session._managed.arms["arm"].driver
    monkeypatch.setattr(fixture, "write", lambda q: driver.write(np.pad(q, (0, 5))))
    monkeypatch.setattr(
        fixture,
        "write_position_velocity",
        lambda q, v: driver.write_position_velocity(
            np.pad(q, (0, 5)), np.pad(v, (0, 5))
        ),
    )
    monkeypatch.setattr(fixture, "hold", driver.hold)
    stamp = session.observe().session_ns
    age = 300_000_000
    deadline = stamp + age
    value = motion((0, 0), (0.02, -0.01), feedforward=feedforward)
    value = replace(
        value, policy=replace(value.policy, watchdog=PlaybackWatchdog(stamp, age))
    )
    handle = run.start_playback(value)
    try:
        assert entered.wait(2), handle.snapshot()
        remaining = deadline - clock().session_ns
        assert remaining > 0
        time.sleep(remaining * 1e-9 + 0.02)
    finally:
        release.set()
    result = handle.wait()
    assert len(latches) == result["dispatched"] == 1 and latches[0] < deadline
    assert result["failure"]["outcome_unknown"] is False, result
    assert result["failure"]["evidence"]["context"]["valid_until_ns"] == deadline
    assert result["cleanup"]["outcome"] == "acknowledged"
    assert holds and deadline < holds[-1] <= result["cleanup"]["completed_ns"]
    record_property(
        "yam_latch",
        json.dumps(
            {
                "deadline_ns": deadline,
                "position_latch_ns": latches,
                "hold_ns": holds,
                "cleanup": result["cleanup"],
            }
        ),
    )
