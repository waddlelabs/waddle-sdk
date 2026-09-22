"""Regression tests for the pinned I2RT starvation-safe CAN receive path."""

from __future__ import annotations

import logging
import sys
import time
import types
from functools import wraps

import pytest
from waddle_sdk.robots import _i2rt_patches as patches
from waddle_sdk.robots._i2rt_patches import (
    _command_joint_state_atomic,
    _receive_message_starvation_tolerant,
    apply_command_state_atomic_patch,
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


class _Clock:
    now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, delay):
        self.now += delay


class _TransactionBus:
    channel_info = "socketcan channel 'test_can'"

    def __init__(self, clock):
        self.clock = clock
        self.script = []
        self.sent = []
        self.waits = []
        self.unrelated = None

    def send(self, message):
        self.sent.append(message)

    def recv(self, timeout):
        self.waits.append(timeout)
        if self.script:
            elapsed, reply = self.script.pop(0)
            self.clock.sleep(elapsed)
            return reply
        if self.unrelated is not None:
            self.clock.sleep(0.002)
            return self.unrelated
        self.clock.sleep(timeout)
        return None


def _install_interface(monkeypatch, interface):
    for name in ("i2rt", "i2rt.motor_drivers", "i2rt.motor_drivers.can_interface"):
        monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    sys.modules["i2rt.motor_drivers.can_interface"].CanInterface = interface


@pytest.fixture
def transaction(monkeypatch):
    clock = _Clock()
    monkeypatch.setattr(patches, "time", clock)
    can = types.ModuleType("can")
    can.Message = lambda **kwargs: types.SimpleNamespace(**kwargs)
    can.CanError = type("CanError", (Exception,), {})
    monkeypatch.setitem(sys.modules, "can", can)

    class CanInterface:
        use_buffered_reader = False
        name = "yam_test"
        receive_mode = types.SimpleNamespace(get_receive_id=lambda motor: motor + 16)
        _receive_message = _receive_message_starvation_tolerant

        def _send_message_get_response(
            self, id, motor_id, data, max_retry=5, expected_id=None
        ):
            # Pinned vendor's first-frame match plus discarded recovery receive.
            for _ in range(max_retry):
                self.bus.send(
                    can.Message(arbitration_id=id, data=data, is_extended_id=False)
                )
                reply = _receive_message_starvation_tolerant(self, motor_id, 0.01)
                expected = motor_id + 16 if expected_id is None else expected_id
                if reply is not None and reply.arbitration_id == expected:
                    return reply
                _receive_message_starvation_tolerant(self, motor_id, 0.009, True)
                clock.sleep(0.001)
            raise AssertionError(
                f"fail to communicate with the motor {id} on {self.name} "
                f"at can channel {self.bus.channel_info}"
            )

    _install_interface(monkeypatch, CanInterface)
    original_transaction = CanInterface._send_message_get_response
    apply_recv_starvation_patch()
    interface = CanInterface()
    interface.bus = _TransactionBus(clock)
    interface.original_transaction = original_transaction.__get__(interface)
    return interface, clock


@pytest.mark.parametrize("status", [0x15, 0xD5])
def test_transaction_returns_matching_frame_unchanged(transaction, status):
    interface, _ = transaction
    reply = types.SimpleNamespace(arbitration_id=21, data=bytes([status]))
    interface.bus.script = [(0.0005, reply)]

    assert interface._send_message_get_response(5, 5, [3, 4]) is reply
    sent = interface.bus.sent
    assert [(m.arbitration_id, m.data, m.is_extended_id) for m in sent] == [
        (5, [3, 4], False)
    ]


@pytest.mark.parametrize("expected_id", [None, 0x50F, 0])
def test_transaction_skips_unrelated_reply_without_resending(transaction, expected_id):
    interface, _ = transaction
    expected = 21 if expected_id is None else expected_id
    reply = types.SimpleNamespace(arbitration_id=expected)
    interface.bus.script = [
        (0.003, types.SimpleNamespace(arbitration_id=18)),
        (0.001, reply),
    ]

    assert (
        interface._send_message_get_response(5, 5, [3], expected_id=expected_id)
        is reply
    )
    assert len(interface.bus.sent) == 1
    assert interface.bus.waits == pytest.approx([0.01, 0.007])


def test_transaction_accepts_queued_matching_reply_after_scheduling_delay(transaction):
    interface, _ = transaction
    reply = types.SimpleNamespace(arbitration_id=21)
    interface.bus.script = [(0.020, None), (0, reply)]

    assert interface._send_message_get_response(5, 5, [3]) is reply
    assert len(interface.bus.sent) == 1
    assert interface.bus.waits == pytest.approx([0.01, 0])


@pytest.mark.parametrize("recovery_delay", [0.001, 0.008])
@pytest.mark.parametrize("status", [0x15, 0xD5])
def test_transaction_accepts_matching_reply_in_original_recovery_budget(
    transaction, recovery_delay, status
):
    interface, clock = transaction
    reply = types.SimpleNamespace(arbitration_id=21, data=bytes([status]))
    interface.bus.script = [(0.010, None), (0, None), (recovery_delay, reply)]

    with pytest.raises(AssertionError, match="fail to communicate with the motor 5"):
        interface.original_transaction(5, 5, [3], max_retry=1)
    interface.bus.sent.clear()
    interface.bus.waits.clear()
    interface.bus.script = [(0.010, None), (0, None), (recovery_delay, reply)]
    start = clock.now
    assert interface._send_message_get_response(5, 5, [3]) is reply
    assert len(interface.bus.sent) == 1
    assert clock.now - start == pytest.approx(0.010 + recovery_delay)
    assert interface.bus.waits == pytest.approx([0.010, 0, 0.009])


def test_transaction_does_not_restart_recovery_budget_after_late_initial_wait(
    transaction,
):
    interface, clock = transaction
    interface.bus.script = [(0.015, None), (0, None)]

    with pytest.raises(AssertionError, match="fail to communicate with the motor 5"):
        interface._send_message_get_response(5, 5, [3], max_retry=1)

    assert interface.bus.waits == pytest.approx([0.010, 0, 0.004, 0])
    assert clock.now == pytest.approx(0.020)  # Original 19 ms plus retry sleep.


@pytest.mark.parametrize("unrelated", [False, True])
def test_transaction_exhausts_original_retry_budget_with_scoped_error(
    transaction, unrelated
):
    interface, clock = transaction
    if unrelated:
        interface.bus.unrelated = types.SimpleNamespace(arbitration_id=18)

    with pytest.raises(AssertionError) as caught:
        interface._send_message_get_response(5, 5, [3], max_retry=3)

    assert (
        str(caught.value)
        == "fail to communicate with the motor 5 on yam_test at can channel socketcan channel 'test_can'"
    )
    assert len(interface.bus.sent) == 3
    assert clock.now <= 0.070
    assert max(interface.bus.waits) <= 0.010000001


def test_reapplying_preserves_transaction_instrumentation(transaction, monkeypatch):
    interface, _ = transaction
    cls = type(interface)
    original = cls._send_message_get_response
    calls = []

    @wraps(original)
    def recorded(self, *args, **kwargs):
        calls.append(args)
        return original(self, *args, **kwargs)

    monkeypatch.setattr(cls, "_send_message_get_response", recorded)
    apply_recv_starvation_patch()
    reply = types.SimpleNamespace(arbitration_id=21)
    interface.bus.script = [(0, reply)]
    assert interface._send_message_get_response(5, 5, [3]) is reply
    assert calls == [(5, 5, [3])]


@pytest.mark.parametrize("receive_already_patched", [False, True])
def test_transaction_signature_drift_refuses_without_partial_install(
    monkeypatch, receive_already_patched
):
    class CanInterface:
        def _receive_message(self, motor_id=None, timeout=0.009, supress_warning=False):
            raise AssertionError("unpatched receive must not be called")

        def _send_message_get_response(self, id, motor_id, data):
            raise AssertionError("unknown transaction must not be called")

    if receive_already_patched:
        CanInterface._receive_message = _receive_message_starvation_tolerant
    original_receive = CanInterface._receive_message
    original_send = CanInterface._send_message_get_response
    _install_interface(monkeypatch, CanInterface)
    with pytest.raises(
        RuntimeError, match=r"CanInterface\._send_message_get_response signature"
    ):
        apply_recv_starvation_patch()
    assert CanInterface._receive_message is original_receive
    assert CanInterface._send_message_get_response is original_send


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

        def _send_message_get_response(
            self, id, motor_id, data, max_retry=5, expected_id=None
        ):
            del self, id, motor_id, data, max_retry, expected_id

    i2rt = types.ModuleType("i2rt")
    motor_drivers = types.ModuleType("i2rt.motor_drivers")
    can_interface = types.ModuleType("i2rt.motor_drivers.can_interface")
    can_interface.CanInterface = CanInterface
    monkeypatch.setitem(sys.modules, "i2rt", i2rt)
    monkeypatch.setitem(sys.modules, "i2rt.motor_drivers", motor_drivers)
    monkeypatch.setitem(sys.modules, "i2rt.motor_drivers.can_interface", can_interface)

    apply_recv_starvation_patch()
    installed = CanInterface._receive_message
    transaction_installed = CanInterface._send_message_get_response
    apply_recv_starvation_patch()

    assert getattr(installed, "_waddle_starvation_patch", False)
    assert CanInterface._receive_message is installed
    assert CanInterface._send_message_get_response is transaction_installed
    assert getattr(transaction_installed, "_waddle_matching_patch", False)


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


class _Command:
    def __init__(self) -> None:
        self.pos = None
        self.vel = None
        self.kp = None
        self.kd = None

    @classmethod
    def init_all_zero(cls, _count: int) -> _Command:
        return cls()


class _Remapper:
    def to_robot_joint_pos_space(self, value):
        return ("position", value)

    def to_robot_joint_vel_space(self, value):
        return ("velocity", value)


class _AssertAtomicLock:
    def __init__(self, robot, original) -> None:
        self.robot = robot
        self.original = original

    def __enter__(self) -> None:
        assert self.robot._commands is self.original

    def __exit__(self, *_args) -> None:
        return None


def test_command_state_builds_complete_command_before_locked_publication() -> None:
    robot = types.SimpleNamespace()
    original = _Command()
    robot._commands = original
    robot.motor_chain = [1, 2]
    robot.remapper = _Remapper()
    robot._kp = "default-kp"
    robot._kd = "default-kd"
    robot._clip_robot_joint_pos_command = lambda value: ("clipped", value)
    robot._command_lock = _AssertAtomicLock(robot, original)

    _command_joint_state_atomic(robot, {"pos": "p", "vel": "v"})

    assert robot._commands is not original
    assert robot._commands.pos == ("position", ("clipped", "p"))
    assert robot._commands.vel == ("velocity", "v")
    assert robot._commands.kp == "default-kp"
    assert robot._commands.kd == "default-kd"


def test_apply_command_state_patch_is_signature_checked_and_idempotent(
    monkeypatch,
) -> None:
    class MotorChainRobot:
        def command_joint_state(self, joint_state):
            del self, joint_state

    i2rt = types.ModuleType("i2rt")
    robots = types.ModuleType("i2rt.robots")
    motor_chain_robot = types.ModuleType("i2rt.robots.motor_chain_robot")
    motor_chain_robot.MotorChainRobot = MotorChainRobot
    monkeypatch.setitem(sys.modules, "i2rt", i2rt)
    monkeypatch.setitem(sys.modules, "i2rt.robots", robots)
    monkeypatch.setitem(sys.modules, "i2rt.robots.motor_chain_robot", motor_chain_robot)

    apply_command_state_atomic_patch()
    installed = MotorChainRobot.command_joint_state
    apply_command_state_atomic_patch()

    assert getattr(installed, "_waddle_atomic_command_patch", False)
    assert MotorChainRobot.command_joint_state is installed


def test_apply_command_state_patch_refuses_unverified_signature(monkeypatch) -> None:
    class MotorChainRobot:
        def command_joint_state(self, position, velocity):
            del self, position, velocity

    i2rt = types.ModuleType("i2rt")
    robots = types.ModuleType("i2rt.robots")
    motor_chain_robot = types.ModuleType("i2rt.robots.motor_chain_robot")
    motor_chain_robot.MotorChainRobot = MotorChainRobot
    monkeypatch.setitem(sys.modules, "i2rt", i2rt)
    monkeypatch.setitem(sys.modules, "i2rt.robots", robots)
    monkeypatch.setitem(sys.modules, "i2rt.robots.motor_chain_robot", motor_chain_robot)

    with pytest.raises(RuntimeError, match="re-verify atomic YAM commands"):
        apply_command_state_atomic_patch()


def test_apply_command_state_patch_preserves_position_only_vendor_types() -> None:
    class PositionOnlyRobot:
        pass

    assert apply_command_state_atomic_patch(PositionOnlyRobot) is False
