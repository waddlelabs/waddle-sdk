"""Run-bound input over the public native owner and the device relay."""

import threading
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace

import pytest
from test_finite_playback import motion
from test_finite_playback import opened as opened  # noqa: PLC0414
from waddle_sdk.runtime import FaultCode, PlaybackWatchdog, RuntimeFault, SupportFact
from waddle_sdk_relay.device import Device
from waddle_sdk_relay.errors import RelayError
from waddle_sdk_relay.protocol import SAFETY_METHODS

TIMEOUT = (
    30_000_000_000  # Wall-clock integration; expiry is fake-clock-tested natively.
)


def test_input_sequences_and_release_fence_delayed_motion(opened):
    session, run = opened
    assert SupportFact.INPUT_WATCHDOG_PLAYBACK in session.support().rows[0].facts
    control = run.start_playback_input(TIMEOUT)
    initial = control.snapshot()
    accepted = control.refresh(2)
    assert accepted["episode_id"] == run.id
    assert accepted["id"] == initial["id"]
    assert accepted["sequence"] == 2
    assert accepted["watchdog"]["observed_ns"] >= initial["watchdog"]["observed_ns"]
    for sequence in (2, 1, 0):
        assert control.refresh(sequence) == accepted
    control.release("button released before planning finished")
    assert control.snapshot()["failure"]["code"] == "cancelled"
    with pytest.raises(RuntimeFault):
        control.refresh(3)
    with pytest.raises(RuntimeFault):
        run.start_playback(motion(), input=control)
    assert session._managed.accepted == 0


def test_input_renewal_does_not_wait_for_owner_read_and_release_holds(
    opened, monkeypatch
):
    session, run = opened
    control = run.start_playback_input(TIMEOUT)
    driver = session._managed.arms["arm"].driver
    original = driver.read
    entered, finish = threading.Event(), threading.Event()

    def blocked():
        entered.set()
        assert finish.wait(5)
        return original()

    monkeypatch.setattr(driver, "read", blocked)
    value = motion((0, 0), (0.02, -0.01))
    value = replace(value, policy=replace(value.policy, timeout_ns=TIMEOUT))
    handle = run.start_playback(value, input=control)
    try:
        assert entered.wait(5)
        assert control.refresh(1)["sequence"] == 1
        with pytest.raises(RuntimeFault):
            handle.refresh_watchdog(1)
        control.release("button released during read")
    finally:
        finish.set()
    result = handle.wait()
    assert result["failure"]["code"] == "cancelled", result
    assert result["dispatched"] == 0
    assert result["cleanup"]["outcome"] == "acknowledged"


def test_input_binding_and_dual_watchdog_are_refused(opened):
    session, run = opened
    control = run.start_playback_input(TIMEOUT)
    value = motion()
    value = replace(
        value, policy=replace(value.policy, watchdog=PlaybackWatchdog(0, 1))
    )
    with pytest.raises(RuntimeFault) as dual:
        run.start_playback(value, input=control)
    assert dual.value.code == FaultCode.INVALID_REQUEST
    with pytest.raises(RuntimeFault):
        run.start_playback(motion(), input=object())
    run.__exit__(None, None, None)
    assert control.snapshot()["failure"]["code"] == "cancelled"
    with session.run(task="another input run", actor="test") as other:
        with pytest.raises(RuntimeFault):
            other.start_playback(motion(), input=control)
    assert session._managed.accepted == 0


def test_missing_input_support_never_admits_motion(opened, monkeypatch):
    session, run = opened
    control = run.start_playback_input(TIMEOUT)
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
                        if fact != SupportFact.INPUT_WATCHDOG_PLAYBACK
                    ),
                )
                for row in support.rows
            ),
        ),
    )
    for operation in (
        lambda: run.start_playback_input(TIMEOUT),
        lambda: run.start_playback(motion(), input=control),
    ):
        with pytest.raises(RuntimeFault) as caught:
            operation()
        assert caught.value.code == FaultCode.UNSUPPORTED
    assert session._managed.accepted == 0


def test_relay_input_bypasses_blocked_admission_and_release_uses_stop_capacity(
    opened, tmp_path
):
    session, run = opened
    device = Device(session.site, tmp_path)
    device.sdk, device.run = session, run
    initial = device.dispatch(
        "playback_input_start", {"run_id": run.id, "timeout_ns": TIMEOUT}
    )
    binding = {"run_id": run.id, "id": initial["id"]}
    assert "playback_input_release" in SAFETY_METHODS
    with ThreadPoolExecutor(max_workers=1) as executor:
        with device._admission:
            accepted = executor.submit(
                device.dispatch, "playback_input_refresh", {**binding, "sequence": 1}
            ).result(timeout=2)
            assert accepted["sequence"] == 1
            executor.submit(
                device.dispatch,
                "playback_input_release",
                {**binding, "reason": "released"},
            ).result(timeout=2)
    assert (
        device.dispatch("playback_input_status", binding)["failure"]["code"]
        == "cancelled"
    )
    with pytest.raises(RuntimeFault):
        device.dispatch(
            "playback_start",
            {"run_id": run.id, "motion": motion(), "input_id": initial["id"]},
        )
    with pytest.raises(RelayError) as foreign:
        device.dispatch(
            "playback_input_refresh", {**binding, "run_id": "foreign", "sequence": 2}
        )
    assert foreign.value.code == "sdk_playback_input_expired"
    assert session._managed.accepted == 0


def test_relay_input_binds_real_native_motion_and_retains_terminal_input(
    opened, tmp_path
):
    session, run = opened
    device = Device(session.site, tmp_path)
    device.sdk, device.run = session, run
    initial = device.dispatch(
        "playback_input_start", {"run_id": run.id, "timeout_ns": TIMEOUT}
    )
    value = motion((0, 0), (0.02, -0.01))
    value = replace(value, policy=replace(value.policy, timeout_ns=TIMEOUT))
    admitted = device.dispatch(
        "playback_start", {"run_id": run.id, "motion": value, "input_id": initial["id"]}
    )
    result = device._playbacks[(run.id, admitted["id"])].wait()
    assert result["status"] == "arrived", result
    assert result["dispatched"] == 2
    device.dispatch("run_exit", {})
    retained = device.dispatch(
        "playback_input_status", {"run_id": run.id, "id": initial["id"]}
    )
    assert retained["failure"]["code"] == "cancelled"
