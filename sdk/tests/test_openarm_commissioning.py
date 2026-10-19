"""Attended commissioning behavior against a stateful fake CAN controller."""

import itertools
import struct
import time
from collections import deque
from types import SimpleNamespace

import pytest
from test_robots_openarm import frame
from waddle_sdk.robots import _openarm_transport as wire
from waddle_sdk.robots import openarm_commissioning as commissioning


class Controller:
    def __init__(self, clock):
        self.clock = clock
        self.queue = deque()
        self.sent = []
        self.closed = False
        self.status = 0
        self.position = 0.2
        self.parameters = {9: 0, 10: 2}
        self.last_control = -100
        self.watchdog = True
        self.bad_enable = False
        self.fail_control = False
        self.bad_disable = False
        self.peer_status = 0
        self.clear_error_works = True
        self.watchdog_status = 13
        self.tracking_fraction = 1.0

    def settimeout(self, _):
        pass

    def state(self, motor):
        if (
            self.status == 1
            and self.watchdog
            and self.parameters[9]
            and self.clock[0] - self.last_control > self.parameters[9] * 0.00005
        ):
            self.status = self.watchdog_status
        status = self.status if motor == 7 else self.peer_status
        q = round(((self.position if motor == 7 else 0.2) / 12.5 + 1) * 65535 / 2)
        return bytes((motor | (status << 4), q >> 8, q & 255, 128, 8, 0, 24, 25))

    def send(self, raw):
        can_id, size, data = wire._CAN_FRAME.unpack(raw)
        assert size == 8
        self.sent.append((self.clock[0], can_id, data))
        if can_id == 0x7FF:
            motor, opcode, rid = struct.unpack("<HBB", data[:4])
            if opcode == 0x33:
                values = {
                    7: motor + 16,
                    8: motor,
                    21: 12.5,
                    22: 30.0,
                    23: 10.0,
                    **self.parameters,
                }
                result = data[:4] + struct.pack(
                    "<f" if rid in (21, 22, 23) else "<I", values[rid]
                )
            elif opcode == 0x55:
                assert motor == 7 and rid in (9, 10)
                self.parameters[rid] = struct.unpack("<I", data[4:])[0]
                result = data
            else:
                assert opcode == 0xCC
                result = self.state(motor)
        else:
            assert can_id == 7, "commissioning actuated an unselected motor"
            motor = 7
            if data == bytes([255]) * 7 + b"\xfc":
                self.status = 0 if self.bad_enable else 1
                self.last_control = self.clock[0]
            elif data == bytes([255]) * 7 + b"\xfd":
                if not self.bad_disable and self.status != 13:
                    self.status = 0
            elif data == bytes([255]) * 7 + b"\xfb":
                if self.clear_error_works:
                    self.status = 0
                # Real CLI does not require an immediate clear acknowledgement.
                return len(raw)
            else:
                assert data != bytes([255]) * 7 + b"\xfe"
                kp = ((data[3] & 15) << 8) | data[4]
                if self.fail_control and self.status == 1 and kp:
                    return len(raw)
                if self.status == 1 and kp:
                    desired = (int.from_bytes(data[:2], "big") / 65535 * 2 - 1) * 12.5
                    self.position += self.tracking_fraction * (desired - self.position)
                self.last_control = self.clock[0]
            result = self.state(motor)
        self.queue.append(frame(motor + 16, result))
        return len(raw)

    def recvmsg(self, *_):
        if self.queue:
            return self.queue.popleft()
        raise TimeoutError

    def close(self):
        self.closed = True


@pytest.fixture
def controller(monkeypatch):
    clock = [100.0]
    controller = Controller(clock)
    monkeypatch.setattr(
        commissioning,
        "time",
        SimpleNamespace(
            monotonic=lambda: clock[0],
            time_ns=time.time_ns,
            sleep=lambda delay: clock.__setitem__(0, clock[0] + delay),
        ),
    )
    monkeypatch.setattr(wire, "_usb_serial", lambda _: "commissioning-test")
    monkeypatch.setattr(wire, "_open_socket", lambda _: controller)
    monkeypatch.setattr(
        wire.socketcan,
        "read_socketcan_state",
        lambda name: wire.socketcan.SocketCanState(name, True, 1_000_000),
    )
    yield controller
    assert controller.closed


def owner(*, excursion=0.06):
    return commissioning.CommissioningOwner(
        interface="can_test",
        usb_serial="commissioning-test",
        motors=tuple(wire.MotorAddress(i, i + 16) for i in range(1, 9)),
        selected=wire.MotorAddress(7, 23),
        guards=commissioning.Guards(True, True, excursion, 0.2, 0.3, 50, 0.02),
    )


def test_single_motor_out_and_back_qualifies_silent_watchdog_and_restores(controller):
    with owner() as session:
        report = session.run()
    assert report["error"] is None, report
    assert report["cleanup_errors"] == []
    assert report["disabled_confirmed"] and report["restored"]
    assert report["watchdog"]["status"] == 13
    assert all(row["arrived"] for row in report["arrivals"])
    assert controller.parameters == {9: 0, 10: 2}
    assert controller.status == 0
    control_times = [t for t, cid, data in controller.sent if cid == 7]
    assert any(b - a >= 0.2 for a, b in itertools.pairwise(control_times))
    # Every transmission in the silence gap is prohibited, including queries.
    assert any(b[0] - a[0] >= 0.2 for a, b in zip(controller.sent, controller.sent[1:]))
    commands = [row for row in report["samples"] if row["kind"] == "command"]
    assert commands[0]["decoded"]["kp"] == commands[0]["decoded"]["kd"] == 0
    assert 0 < abs(commands[0]["decoded"]["feedforward"]) <= 10 / 4095 + 1e-9
    assert max(abs(row["implied_nominal_effort_nm"]) for row in commands) < 0.3


@pytest.mark.parametrize("fault", ["watchdog", "bad_enable", "fail_control"])
def test_no_advance_after_fault_and_independent_disable_restore(controller, fault):
    setattr(controller, fault, fault != "watchdog")
    session = owner()
    report = session.run()
    assert report["error"] is not None
    assert report["disabled_confirmed"] and report["restored"]
    assert controller.status == 0
    assert controller.parameters == {9: 0, 10: 2}
    with pytest.raises(ValueError, match="cannot be reused"):
        session.run()


def test_failed_disable_does_not_restore_or_claim_success(controller):
    controller.watchdog = False
    controller.bad_disable = True
    report = owner().run(offset_rad=0)
    assert report["error"] is not None
    assert report["cleanup_errors"]
    assert not report["disabled_confirmed"] and not report["restored"]
    assert controller.parameters == {9: 2000, 10: 1}


def test_enabled_peer_prevents_all_writes(controller):
    controller.peer_status = 1
    with pytest.raises(Exception, match="unexpected motor status"):
        owner()
    assert all(
        cid == 0x7FF and data[2] in (0x33, 0xCC) for _, cid, data in controller.sent
    )


def test_invalid_trial_refused_before_writes(controller):
    with owner() as session:
        before = len(controller.sent)
        for kwargs in (
            {"offset_rad": 0.101},
            {"kp": 3},
            {"kd": 0.501},
            {"duration_s": float("nan")},
            {"slew_rad_s": 0},
            {"slew_rad_s": 0.026},
        ):
            with pytest.raises(ValueError):
                session.run(**kwargs)
        assert len(controller.sent) == before


def test_zero_gain_watchdog_only_never_sends_position_gain(controller):
    report = owner().run(offset_rad=0)
    assert report["error"] is None
    assert "arrivals" not in report
    assert all(
        row["decoded"]["kp"] == row["decoded"]["kd"] == 0
        for row in report["samples"]
        if row["kind"] == "command"
    )


def test_cleanup_attempts_disable_even_when_queued_receive_is_broken(
    controller, monkeypatch
):
    session = owner()
    session._changed = True
    session._bus._failure = session._fault("original", "original transport failure")
    monkeypatch.setattr(
        session._bus,
        "_receive",
        lambda _: (_ for _ in ()).throw(OSError("queued CAN error")),
    )
    session.close()
    assert controller.sent[-1][1:] == (7, bytes([255]) * 7 + b"\xfd")
    assert session.report["cleanup_errors"]
    assert not session.report["disabled_confirmed"]


def test_reference_guard_checks_decoded_absolute_window_before_send(controller):
    with owner() as session:
        measured = session._batch()[session.selected]
        before = len(controller.sent)
        with pytest.raises(Exception, match="original local window"):
            session._command(
                session.baseline[session.selected] + 0.061, 2, 0.1, measured
            )
        assert len(controller.sent) == before


def test_mode_readback_and_neutral_preseed_precede_each_enable(controller):
    report = owner().run()
    assert report["error"] is None
    mode_written = mode_read_back = False
    previous_control = None
    enables = 0
    for _, identifier, payload in controller.sent:
        if identifier == 0x7FF:
            motor, opcode, rid = struct.unpack("<HBB", payload[:4])
            if (
                motor == 7
                and rid == 10
                and opcode == 0x55
                and struct.unpack("<I", payload[4:])[0] == 1
            ):
                mode_written = True
            if motor == 7 and rid == 10 and opcode == 0x33 and mode_written:
                mode_read_back = True
        elif payload == bytes([255]) * 7 + b"\xfc":
            assert mode_read_back and previous_control is not None
            assert ((previous_control[3] & 15) << 8 | previous_control[4]) == 0
            assert (previous_control[5] << 4 | previous_control[6] >> 4) == 0
            enables += 1
        elif payload not in (bytes([255]) * 7 + b"\xfd", bytes([255]) * 7 + b"\xfb"):
            previous_control = payload
    assert enables == 2


def test_nonfinite_or_unattended_guard_refusal_before_open():
    for args in (
        (False, True, 0.06, 0.2, 0.3, 50, 0.02),
        (True, True, 0.06, float("inf"), 0.3, 50, 0.02),
    ):
        with pytest.raises(ValueError):
            commissioning.Guards(*args)


def test_cancellation_after_active_reply_prevents_next_control_and_disables(
    controller, monkeypatch
):
    session = owner()
    original_send = controller.send
    cancelled_at = []

    def send(raw):
        count = original_send(raw)
        identifier, _, payload = wire._CAN_FRAME.unpack(raw)
        if identifier == 7 and payload[:7] != bytes([255]) * 7:
            kp = (payload[3] & 15) << 8 | payload[4]
            if kp and not cancelled_at:
                session._cancel.set()
                cancelled_at.append(len(controller.sent))
        return count

    monkeypatch.setattr(controller, "send", send)
    report = session.run()
    assert report["error"]["context"]["reason"] == "openarm.commissioning.cancelled"
    assert report["disabled_confirmed"] and report["restored"]
    assert cancelled_at
    assert all(
        identifier == 0x7FF or payload == bytes([255]) * 7 + b"\xfd"
        for _, identifier, payload in controller.sent[cancelled_at[0] :]
    )


def test_sticky_watchdog_clear_is_once_and_ack_independent(controller):
    result = owner().run(offset_rad=0)
    assert result["error"] is None and result["restored"]
    assert result["watchdog_clear"] == {"sent_once": True, "fresh_status": 0}
    assert (
        sum(
            identifier == 7 and payload == bytes([255]) * 7 + b"\xfb"
            for _, identifier, payload in controller.sent
        )
        == 1
    )


def test_unrelated_fault_is_never_cleared_or_followed_by_motion(controller):
    controller.watchdog_status = 10
    result = owner().run()
    assert result["error"]["context"]["status"] == 10
    assert not any(
        payload == bytes([255]) * 7 + b"\xfb" for _, _, payload in controller.sent
    )
    assert all(
        row["decoded"]["kp"] == row["decoded"]["kd"] == 0
        for row in result["samples"]
        if row["kind"] == "command"
    )


def test_explicit_retained_watchdog_recovery_has_no_enable_or_mit(controller):
    controller.clear_error_works = False
    first = owner().run(offset_rad=0)
    assert first["error"] and not first["disabled_confirmed"]
    assert controller.status == 13 and controller.parameters == {9: 2000, 10: 1}
    before = len(controller.sent)
    controller.closed = False
    controller.clear_error_works = True
    result = commissioning.CommissioningOwner.recover_watchdog(
        prior_report=first,
        interface="can_test",
        usb_serial="commissioning-test",
        motors=tuple(wire.MotorAddress(i, i + 16) for i in range(1, 9)),
        selected=wire.MotorAddress(7, 23),
        guards=commissioning.Guards(True, True, 0.06, 0.2, 0.3, 50, 0.02),
    )
    assert (
        result["error"] is None and result["restored"] and result["disabled_confirmed"]
    )
    assert controller.parameters == {9: 0, 10: 2} and controller.status == 0
    assert all(
        identifier == 0x7FF
        or payload in (bytes([255]) * 7 + b"\xfb", bytes([255]) * 7 + b"\xfd")
        for _, identifier, payload in controller.sent[before:]
    )


def test_recovery_refuses_unrelated_evidence_before_open(controller):
    with owner() as session:
        before = len(controller.sent)
        with pytest.raises(ValueError, match="retained zero-gain"):
            commissioning.CommissioningOwner.recover_watchdog(
                prior_report={},
                interface="can_test",
                usb_serial="commissioning-test",
                motors=session.motors,
                selected=session.selected,
                guards=session.guards,
            )
        assert len(controller.sent) == before


def test_tracking_shortfall_is_measured_separately_from_motor_failure(controller):
    controller.tracking_fraction = 0
    result = owner().run()
    assert (
        result["error"] is None and result["disabled_confirmed"] and result["restored"]
    )
    assert not result["arrivals"][0]["arrived"]
    assert result["arrivals"][0]["error_rad"] > 0.02
    assert not result["motion"]["observed_requested_direction"]
    assert result["motion"]["peak_displacement_rad"] == 0


def test_explicit_larger_relative_trial_keeps_effort_and_local_bounds(controller):
    controller.tracking_fraction = 0
    result = owner(excursion=0.12).run(offset_rad=0.1, slew_rad_s=0.025)
    assert result["error"] is None and result["restored"]
    assert not result["arrivals"][0]["arrived"]
    assert result["arrivals"][0]["error_rad"] > 0.09
    assert not result["motion"]["observed_requested_direction"]
    commands = [row for row in result["samples"] if row["kind"] == "command"]
    assert 0.18 < max(abs(row["implied_nominal_effort_nm"]) for row in commands) < 0.3
    assert (
        max(
            abs(row["decoded"]["position"] - result["motion"]["start_rad"])
            for row in commands
        )
        <= 0.101
    )


def test_higher_damping_still_refuses_excess_implied_effort_before_send(controller):
    with owner(excursion=0.12) as session:
        measured = session._batch()[session.selected]
        measured = {
            **measured,
            "position_rad": session.baseline[session.selected] - 0.02,
            "velocity_rad_s": -0.2,
        }
        before = len(controller.sent)
        with pytest.raises(Exception) as error:
            session._command(session.baseline[session.selected] + 0.1, 2, 0.5, measured)
        assert error.value.code.value == "safety_refusal"
        assert error.value.context["reason"] == "openarm.commissioning.command_effort"
        assert error.value.context["estimate"] > 0.3
        assert len(controller.sent) == before


def test_explicit_higher_damping_preserves_bounds_and_no_automatic_escalation(
    controller,
):
    controller.tracking_fraction = 0
    result = owner(excursion=0.12).run(offset_rad=0.1, slew_rad_s=0.025, kd=0.5)
    assert result["error"] is None and result["restored"]
    commands = [row for row in result["samples"] if row["kind"] == "command"]
    assert max(row["decoded"]["kd"] for row in commands) < 0.501
    assert max(abs(row["implied_nominal_effort_nm"]) for row in commands) < 0.3
    assert not result["arrivals"][0]["arrived"]
