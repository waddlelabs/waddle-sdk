"""Regression tests for the pinned I2RT starvation-safe CAN receive path."""

from __future__ import annotations

import logging
import sys
import time
import types

import pytest
from waddle_sdk.robots._i2rt_patches import (
    _receive_message_starvation_tolerant,
    apply_recv_starvation_patch,
)


class _StubBus:
    def __init__(self, script):
        self.script = list(script)
        self.calls: list[float] = []

    def recv(self, timeout=0.0):
        self.calls.append(timeout)
        if not self.script:
            time.sleep(timeout)
            return None
        item = self.script.pop(0)
        if item is None:
            time.sleep(timeout)
            return None
        return item


class _StubInterface:
    use_buffered_reader = False
    name = "yam_stub"

    def __init__(self, script):
        self.bus = _StubBus(script)


_FRAME = object()


def test_receive_uses_one_kernel_wait_for_the_remaining_budget() -> None:
    interface = _StubInterface([_FRAME])

    received = _receive_message_starvation_tolerant(
        interface, motor_id=3, timeout=0.009
    )

    assert received is _FRAME
    assert len(interface.bus.calls) == 1
    assert interface.bus.calls[0] >= 0.007


def test_receive_drains_a_reply_queued_after_a_late_wake() -> None:
    interface = _StubInterface([None, _FRAME])

    received = _receive_message_starvation_tolerant(
        interface, motor_id=5, timeout=0.005
    )

    assert received is _FRAME
    assert interface.bus.calls[-1] == 0.0


def test_genuine_timeout_preserves_the_vendor_warning(caplog) -> None:
    interface = _StubInterface([None, None])

    with caplog.at_level(logging.WARNING):
        received = _receive_message_starvation_tolerant(
            interface, motor_id=4, timeout=0.005
        )

    assert received is None
    assert any(
        "Failed to receive message, yam_stub motor id 4 motor timeout" in record.message
        for record in caplog.records
    )


def test_zero_timeout_still_checks_the_socket_once() -> None:
    interface = _StubInterface([_FRAME])

    assert (
        _receive_message_starvation_tolerant(interface, motor_id=2, timeout=0.0)
        is _FRAME
    )
    assert interface.bus.calls == [0.0]


def test_apply_is_exact_signature_checked_and_idempotent(monkeypatch) -> None:
    class CanInterface:
        def _receive_message(
            self,
            motor_id=None,
            timeout=0.009,
            supress_warning=False,
        ):
            del self, motor_id, timeout, supress_warning

    i2rt = types.ModuleType("i2rt")
    motor_drivers = types.ModuleType("i2rt.motor_drivers")
    can_interface = types.ModuleType("i2rt.motor_drivers.can_interface")
    can_interface.CanInterface = CanInterface
    monkeypatch.setitem(sys.modules, "i2rt", i2rt)
    monkeypatch.setitem(sys.modules, "i2rt.motor_drivers", motor_drivers)
    monkeypatch.setitem(sys.modules, "i2rt.motor_drivers.can_interface", can_interface)

    apply_recv_starvation_patch()
    installed = CanInterface._receive_message
    apply_recv_starvation_patch()

    assert getattr(installed, "_waddle_starvation_patch", False)
    assert CanInterface._receive_message is installed


def test_apply_refuses_an_unverified_vendor_signature(monkeypatch) -> None:
    class CanInterface:
        def _receive_message(self, timeout=0.009):
            del self, timeout

    i2rt = types.ModuleType("i2rt")
    motor_drivers = types.ModuleType("i2rt.motor_drivers")
    can_interface = types.ModuleType("i2rt.motor_drivers.can_interface")
    can_interface.CanInterface = CanInterface
    monkeypatch.setitem(sys.modules, "i2rt", i2rt)
    monkeypatch.setitem(sys.modules, "i2rt.motor_drivers", motor_drivers)
    monkeypatch.setitem(sys.modules, "i2rt.motor_drivers.can_interface", can_interface)

    with pytest.raises(RuntimeError, match="re-verify"):
        apply_recv_starvation_patch()
