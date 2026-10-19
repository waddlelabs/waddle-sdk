"""Attended, bounded commissioning of one supported OpenArm motor.

This is a commissioning owner, not a Site driver or a calibrated robot model.
It shares the monitor's USB ownership and fresh SocketCAN framing. Only one
selected motor may receive enable/MIT/disable or reversible TIMEOUT/mode writes.
No zero-setting, flash storage, automatic recovery, or all-motor enable exists.

Protocol: openarm_can f340d4b808fb177e1f297af54eb55fd51c6c7c10 and Damiao
debugging-assistant manual V1.4 (2024-03-06), TIMEOUT units 50 microseconds.
MIT gains and decoded effort are nominal controller quantities, not certified
torque/current limits. Physical support and an attended cutoff remain required.
"""

from __future__ import annotations

import hashlib
import json
import math
import struct
import threading
import time
from dataclasses import asdict, dataclass

from ..runtime import FaultCode, RuntimeFault
from . import _openarm_transport as wire


@dataclass(frozen=True)
class Guards:
    """Explicit operator-reviewed local bounds, never inferred mechanical limits."""

    all_bus_actuators_supported_and_clear: bool
    cutoff_attended: bool
    max_excursion_rad: float
    max_velocity_rad_s: float
    max_nominal_effort_nm: float
    max_temperature_c: float
    max_peer_drift_rad: float

    def __post_init__(self):
        for name in ("all_bus_actuators_supported_and_clear", "cutoff_attended"):
            if getattr(self, name) is not True:
                raise ValueError(f"Commissioning requires explicit {name}=True")
        for name in (
            "max_excursion_rad",
            "max_velocity_rad_s",
            "max_nominal_effort_nm",
            "max_temperature_c",
            "max_peer_drift_rad",
        ):
            value = getattr(self, name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (float, int))
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(f"{name} must be finite and positive")


def _encode(value, low, high, bits):
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"MIT value {value} outside encoding range [{low}, {high}]")
    integer = round((value - low) / (high - low) * ((1 << bits) - 1))
    return integer, low + integer / ((1 << bits) - 1) * (high - low)


def mit_packet(position, kp, kd, spans):
    """Encode stationary-reference MIT and return its actually decoded values."""
    pmax, vmax, tmax = spans
    p, pd = _encode(position, -pmax, pmax, 16)
    v, vd = _encode(0.0, -vmax, vmax, 12)
    k, kpd = _encode(kp, 0.0, 500.0, 12)
    d, kdd = _encode(kd, 0.0, 5.0, 12)
    t, td = _encode(0.0, -tmax, tmax, 12)
    data = bytes(
        (
            p >> 8,
            p & 255,
            v >> 4,
            ((v & 15) << 4) | (k >> 8),
            k & 255,
            d >> 4,
            ((d & 15) << 4) | (t >> 8),
            t & 255,
        )
    )
    return data, {
        "position": pd,
        "velocity": vd,
        "kp": kpd,
        "kd": kdd,
        "feedforward": td,
    }


class CommissioningOwner:
    """Exclusive bus owner with one explicit actuation address and bounded runs.

    Construction performs reads only. ``run`` qualifies watchdog withdrawal,
    performs one optional relative out-and-back trial, then disables and restores
    original RAM settings. Reports retain the original error and every cleanup
    error. A failed/finished owner cannot be reused. Other buses need their own
    independent observation owner; this owner checks every declared same-bus peer.
    """

    def __init__(
        self,
        *,
        interface,
        usb_serial,
        motors,
        selected,
        guards: Guards,
        timeout_s=0.05,
        bitrate=1_000_000,
        _recovery_evidence=None,
    ):
        self.motors = tuple(motors)
        if not isinstance(guards, Guards):
            raise TypeError("Explicit commissioning Guards required")
        if (
            len(self.motors) != 8
            or len(set(self.motors)) != 8
            or selected not in self.motors
        ):
            raise ValueError(
                "Declare all eight distinct bus motors and one selected address"
            )
        if (
            len({m.receive_id for m in self.motors}) != 8
            or len({m.send_id for m in self.motors}) != 8
        ):
            raise ValueError("Bus send/reply identities must be unique")
        self.selected = selected
        self.guards = guards
        self._operation = threading.Lock()
        self._cancel = threading.Event()
        self._used = False
        self._closed = False
        self._changed = False
        self._deadline = math.inf
        self.report = {
            "interface": interface,
            "usb_serial": usb_serial,
            "selected": asdict(selected),
            "guards": asdict(guards),
            "samples": [],
            "cleanup_errors": [],
            "watchdog": None,
            "error": None,
            "disabled_confirmed": False,
            "restored": False,
        }
        restore_parameters = None
        if _recovery_evidence is not None:
            restore_parameters = self._validate_recovery(
                _recovery_evidence, interface, usb_serial
            )
            self.report["recovery_source_sha256"] = hashlib.sha256(
                json.dumps(_recovery_evidence, sort_keys=True).encode()
            ).hexdigest()
        self._bus = wire.ReadOnlyBus(
            interface=interface,
            usb_serial=usb_serial,
            bitrate=bitrate,
            configure_can=False,
            timeout_s=timeout_s,
        )
        try:
            self.spans = {}
            for motor in self.motors:
                if (
                    self._bus.parameter(motor, 7) != motor.receive_id
                    or self._bus.parameter(motor, 8) != motor.send_id
                ):
                    raise self._fault(
                        "identity_mismatch",
                        "declared motor IDs do not match",
                        motor_id=motor.send_id,
                    )
                spans = tuple(self._bus.parameter(motor, rid) for rid in (21, 22, 23))
                if any(not math.isfinite(v) or v <= 0 for v in spans):
                    raise self._fault(
                        "encoding",
                        "invalid motor encoding spans",
                        motor_id=motor.send_id,
                    )
                self.spans[motor] = spans
            self.original = {rid: self._parameter(rid) for rid in (9, 10)}
            if restore_parameters is not None:
                if self.original != {9: 2000, 10: 1}:
                    raise self._fault(
                        "recovery_parameters",
                        "retained watchdog parameters no longer match",
                        actual=self.original,
                    )
                self.report["recovery_initial_parameters"] = dict(self.original)
                self.original = restore_parameters
            elif self.original[10] != 2:
                raise self._fault(
                    "mode", "initial commissioning currently requires POS_VEL mode 2"
                )
            self.report["original_parameters"] = dict(self.original)
            self.report["encoding_spans"] = {
                str(m.send_id): v for m, v in self.spans.items()
            }
            initial = self._batch()
            self.baseline = {m: row["position_rad"] for m, row in initial.items()}
            self._guard(initial, expected_status=13 if restore_parameters else 0)
        except BaseException:
            self._bus.close()
            raise

    def _validate_recovery(self, report, interface, usb_serial):
        """Refuse unrelated/general fault reset before opening the bus."""
        commands = [
            row.get("decoded", {})
            for row in report.get("samples", ())
            if row.get("kind") == "command"
        ]
        original = {int(k): v for k, v in report.get("original_parameters", {}).items()}
        spans = report.get("encoding_spans", {}).get(str(self.selected.send_id), ())
        tmax = spans[2] if len(spans) == 3 else 0
        if not (
            report.get("interface") == interface
            and report.get("usb_serial") == usb_serial
            and report.get("selected") == asdict(self.selected)
            and report.get("trial", {}).get("offset_rad") == 0
            and report.get("watchdog", {}).get("status") == 13
            and report.get("watchdog", {}).get("ticks") == 2000
            and original == {9: 0, 10: 2}
            and commands
            and all(row.get("kp") == row.get("kd") == 0 for row in commands)
            and isinstance(tmax, (float, int))
            and math.isfinite(tmax)
            and tmax > 0
            and all(
                isinstance(row.get("feedforward"), (float, int))
                and math.isfinite(row["feedforward"])
                and abs(row["feedforward"]) <= tmax / 4095 + 1e-9
                for row in commands
            )
            and report.get("disabled_confirmed") is False
            and report.get("restored") is False
        ):
            raise ValueError(
                "Recovery requires matching retained zero-gain watchdog-only failure evidence"
            )
        return original

    @classmethod
    def recover_watchdog(cls, *, prior_report, **connection):
        """Explicit no-enable recovery of one retained watchdog-only failure.

        Requires exact retained identity, original mode2/TIMEOUT0 and a completed
        zero-gain watchdog probe. Fresh mode1/TIMEOUT2000/status13 and disabled
        peers must still match. Sends clear-error once, confirms status0 through
        fresh refresh, disables, and restores RAM. No MIT/enable is emitted.
        """
        owner = cls(**connection, _recovery_evidence=prior_report)
        owner._used = True
        owner._changed = True
        owner._deadline = time.monotonic() + 2.0
        try:
            owner._clear_expected_watchdog()
            owner._special(0xFD, 0)
        except BaseException as error:  # noqa: BLE001 — keep recovery and cleanup faults separately.
            owner.report["error"] = owner._error(error)
        finally:
            owner._cleanup()
        return owner.report

    def _fault(self, reason, message, **context):
        code = FaultCode.MOTOR_FAILURE
        if reason in {
            "excursion",
            "reference_excursion",
            "velocity",
            "effort",
            "command_effort",
            "temperature",
            "deadline",
            "cancelled",
        }:
            code = FaultCode.SAFETY_REFUSAL
        elif reason in {
            "response_timeout",
            "transport",
            "short_write",
            "malformed",
            "backlog",
            "identity",
            "encoding",
            "parameter_readback",
        }:
            code = FaultCode.TRANSPORT_LOST
        elif reason == "busy":
            code = FaultCode.BUSY
        elif reason == "status" and context.get("status") in (0, 1):
            code = FaultCode.SAFETY_REFUSAL
        return self._bus._fault(
            "commissioning." + reason,
            message,
            code=code,
            selected_motor_id=self.selected.send_id,
            **context,
        )

    def _check_running(self):
        if self._cancel.is_set():
            raise self._fault("cancelled", "commissioning cancellation requested")
        if time.monotonic() >= self._deadline:
            raise self._fault("deadline", "commissioning deadline elapsed")

    def _parameter(self, register, *, cleanup=False):
        if not cleanup:
            self._check_running()
        data, _ = self._bus._exchange((self.selected,), register)[self.selected]
        return struct.unpack("<I", data[4:])[0]

    def _decode(self, motor, data, stamp):
        pmax, vmax, tmax = self.spans[motor]
        return {
            "motor_id": motor.send_id,
            "status": data[0] >> 4,
            "position_rad": (int.from_bytes(data[1:3], "big") / 65535 * 2 - 1) * pmax,
            "velocity_rad_s": (((data[3] << 4) | data[4] >> 4) / 4095 * 2 - 1) * vmax,
            "nominal_effort_nm": (((data[4] & 15) << 8 | data[5]) / 4095 * 2 - 1)
            * tmax,
            "mos_temperature_c": data[6],
            "rotor_temperature_c": data[7],
            "received_unix_ns": stamp,
        }

    def _batch(self):
        self._check_running()
        rows = self._bus._exchange(self.motors, None)
        result = {m: self._decode(m, *rows[m]) for m in self.motors}
        self.report["samples"].append(
            {
                "kind": "bus",
                "host_monotonic_ns": int(time.monotonic() * 1e9),
                "rows": list(result.values()),
            }
        )
        return result

    def _packet(self, payload, *, register=None, cleanup=False, clear_only=False):
        """Fresh matched native response; cleanup bypasses only normal poison."""
        bus = self._bus
        if clear_only and payload != bytes([255]) * 7 + b"\xfb":
            raise ValueError("No-response special path permits only clear-error")
        if not cleanup:
            self._check_running()
        if bus._failure is not None and not cleanup:
            raise bus._failure
        if not bus._lock.acquire(timeout=bus.timeout_s):
            raise self._fault("busy", "transaction lock unavailable")
        try:
            deadline = time.monotonic() + bus.timeout_s
            if not cleanup:
                deadline = min(deadline, self._deadline)
            # Emergency disable must get a transmit attempt even if incoming
            # errors/backlog made the normal transaction unusable. Kernel time
            # still excludes every pre-send reply from its acknowledgement.
            if not cleanup:
                for _ in range(256):
                    if bus._receive(0) is None:
                        break
                else:
                    raise self._fault("backlog", "receive backlog exceeds drain bound")
            sent = time.time_ns()
            if not cleanup:
                self._check_running()
            if deadline <= time.monotonic():
                raise self._fault("deadline", "commissioning deadline elapsed")
            bus._socket.settimeout(max(0.000001, deadline - time.monotonic()))
            can_id = self.selected.send_id if register is None else 0x7FF
            if (
                bus._socket.send(wire._CAN_FRAME.pack(can_id, 8, payload))
                != wire._CAN_FRAME.size
            ):
                raise self._fault("short_write", "incomplete control write")
            if clear_only:
                return None
            for _ in range(1024):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                frame = bus._receive(remaining)
                if frame is None:
                    break
                identifier, size, data, stamp = frame
                if identifier != self.selected.receive_id or stamp < sent:
                    continue
                if size != 8:
                    raise self._fault(
                        "malformed", "selected motor response requires eight bytes"
                    )
                if register is not None:
                    if data == payload:
                        return None
                    continue
                if data[:2] == struct.pack("<H", self.selected.send_id) and data[2] in (
                    0x33,
                    0x55,
                ):
                    continue
                if data[0] & 15 != self.selected.send_id & 15:
                    raise self._fault(
                        "identity", "selected feedback payload ID mismatch"
                    )
                row = self._decode(self.selected, data, stamp)
                self.report["samples"].append(
                    {
                        "kind": "control",
                        "host_monotonic_ns": int(time.monotonic() * 1e9),
                        "row": row,
                    }
                )
                return row
            raise self._fault(
                "response_timeout",
                "no fresh matched selected motor response",
                register=register,
            )
        except BaseException as error:
            if not cleanup:
                bus._failure = (
                    error
                    if isinstance(error, RuntimeFault)
                    else self._fault("transport", str(error))
                )
            raise
        finally:
            bus._lock.release()

    def _set(self, rid, value, *, cleanup=False):
        if rid not in (9, 10):
            raise ValueError("Only reversible timeout/mode RAM writes are supported")
        self._changed = True
        self._packet(
            struct.pack("<HBBI", self.selected.send_id, 0x55, rid, value),
            register=rid,
            cleanup=cleanup,
        )
        if cleanup:
            # Restore queries must remain possible after an earlier poisoned read.
            saved = self._bus._failure
            self._bus._failure = None
            try:
                actual = self._parameter(rid, cleanup=True)
            finally:
                self._bus._failure = saved
        else:
            actual = self._parameter(rid)
        if actual != value:
            raise self._fault(
                "parameter_readback",
                "RAM write readback differs",
                register=rid,
                expected=value,
                actual=actual,
            )

    def _guard(self, rows, *, expected_status):
        for motor, row in rows.items():
            expected = expected_status if motor == self.selected else 0
            if row["status"] != expected:
                raise self._fault(
                    "status",
                    "unexpected motor status",
                    motor_id=motor.send_id,
                    status=row["status"],
                    expected=expected,
                )
            limit = (
                self.guards.max_excursion_rad
                if motor == self.selected
                else self.guards.max_peer_drift_rad
            )
            if abs(row["position_rad"] - self.baseline[motor]) > limit:
                raise self._fault(
                    "excursion",
                    "measured motor excursion exceeded local bound",
                    motor_id=motor.send_id,
                    measured=row["position_rad"],
                    baseline=self.baseline[motor],
                    limit=limit,
                )
            if abs(row["velocity_rad_s"]) > self.guards.max_velocity_rad_s:
                raise self._fault(
                    "velocity",
                    "measured speed exceeded local bound",
                    motor_id=motor.send_id,
                    measured=row["velocity_rad_s"],
                )
            if abs(row["nominal_effort_nm"]) > self.guards.max_nominal_effort_nm:
                raise self._fault(
                    "effort",
                    "reported nominal effort exceeded local bound",
                    motor_id=motor.send_id,
                    measured=row["nominal_effort_nm"],
                )
            if (
                max(row["mos_temperature_c"], row["rotor_temperature_c"])
                > self.guards.max_temperature_c
            ):
                raise self._fault(
                    "temperature",
                    "motor temperature exceeded local bound",
                    motor_id=motor.send_id,
                )

    def _special(self, opcode, expected_status, *, cleanup=False):
        if opcode not in (0xFC, 0xFD):
            raise ValueError("Only enable and disable special commands are supported")
        row = self._packet(bytes([255]) * 7 + bytes([opcode]), cleanup=cleanup)
        if row["status"] != expected_status:
            raise self._fault(
                "status",
                "enable/disable acknowledgement state differs",
                opcode=opcode,
                status=row["status"],
                expected=expected_status,
            )
        return row

    def _clear_expected_watchdog(self):
        # Damiao control manual V1.4 section4.8 PDFpage37 and pinned OpenArm
        # setup/cli/commands/clear_error_commands.cpp: FF*7 FB clears errors.
        # The CLI does not promise an immediate acknowledgement. Fresh refresh
        # must prove normal disabled0; unexpected enabled1 is never accepted.
        before = self._batch()
        self._guard(before, expected_status=13)
        self._packet(bytes([255]) * 7 + b"\xfb", clear_only=True)
        after = self._batch()
        self._guard(after, expected_status=0)
        self.report["watchdog_clear"] = {"sent_once": True, "fresh_status": 0}

    def _command(self, target, kp, kd, measured, *, expected_status=1):
        payload, decoded = mit_packet(target, kp, kd, self.spans[self.selected])
        if any(
            abs(value - self.baseline[self.selected]) > self.guards.max_excursion_rad
            for value in (target, decoded["position"])
        ):
            raise self._fault(
                "reference_excursion",
                "MIT reference exceeds original local window",
                requested=target,
                decoded=decoded["position"],
            )
        estimate = (
            decoded["kp"] * (decoded["position"] - measured["position_rad"])
            + decoded["kd"] * (decoded["velocity"] - measured["velocity_rad_s"])
            + decoded["feedforward"]
        )
        if abs(estimate) > self.guards.max_nominal_effort_nm:
            raise self._fault(
                "command_effort",
                "decoded MIT implied effort exceeds local bound",
                estimate=estimate,
                decoded=decoded,
            )
        self.report["samples"].append(
            {
                "kind": "command",
                "host_monotonic_ns": int(time.monotonic() * 1e9),
                "decoded": decoded,
                "implied_nominal_effort_nm": estimate,
            }
        )
        row = self._packet(payload)
        self._guard({self.selected: row}, expected_status=expected_status)
        return row

    def run(
        self,
        *,
        offset_rad=0.03,
        kp=2.0,
        kd=0.1,
        slew_rad_s=0.02,
        duration_s=10.0,
        arrival_tolerance_rad=0.01,
    ):
        """Qualify watchdog, then optionally execute one bounded out-and-back.

        ``offset_rad=0`` qualifies only the watchdog. Returns evidence, including
        errors, instead of hiding cleanup failures behind an original exception.
        Nominal arrival is encoder-only and never establishes calibrated accuracy.
        """
        values = (offset_rad, kp, kd, slew_rad_s, duration_s, arrival_tolerance_rad)
        if any(
            isinstance(v, bool)
            or not isinstance(v, (float, int))
            or not math.isfinite(v)
            for v in values
        ):
            raise ValueError("Trial settings must be finite numbers")
        if not (
            abs(offset_rad) <= 0.1
            and abs(offset_rad) < self.guards.max_excursion_rad
            and 0 < kp <= 2
            and 0 < kd <= 0.5
            and 0 < slew_rad_s <= 0.025
            and 0 < duration_s <= 10
            and 0 < arrival_tolerance_rad <= 0.01
        ):
            raise ValueError("Trial exceeds commissioning bounds")
        if 2 * abs(offset_rad) / slew_rad_s + 1.0 > duration_s:
            raise ValueError(
                "Trial duration cannot accommodate the bounded out-and-back"
            )
        if not self._operation.acquire(blocking=False):
            raise ValueError("Commissioning owner is already running")
        try:
            if self._used or self._closed:
                raise ValueError("Commissioning owner cannot be reused")
            self._used = True
            self._deadline = time.monotonic() + duration_s
            self.report["trial"] = {
                "offset_rad": offset_rad,
                "kp": kp,
                "kd": kd,
                "slew_rad_s": slew_rad_s,
                "duration_s": duration_s,
            }
            try:
                current = self._batch()
                self._guard(current, expected_status=0)
                self._set(9, 2000)  # 100 ms, RAM only.
                self._set(10, 1)  # Disabled mode switch clears old command values.
                measured = self._command(
                    current[self.selected]["position_rad"],
                    0,
                    0,
                    current[self.selected],
                    expected_status=0,
                )
                self._special(0xFC, 1)
                measured = self._command(measured["position_rad"], 0, 0, measured)
                quiet_start = time.monotonic()
                time.sleep(0.2)  # Absolutely no CAN traffic during qualification.
                rows = self._batch()
                quiet_elapsed = time.monotonic() - quiet_start
                self._guard(rows, expected_status=13)
                self.report["watchdog"] = {
                    "ticks": 2000,
                    "nominal_s": 0.1,
                    "silence_s": 0.2,
                    "observed_after_s": quiet_elapsed,
                    "status": 13,
                }
                self._clear_expected_watchdog()
                self._special(0xFD, 0)
                if offset_rad:
                    current = self._batch()
                    self._guard(current, expected_status=0)
                    start = current[self.selected]["position_rad"]
                    measured = self._command(
                        start, 0, 0, current[self.selected], expected_status=0
                    )
                    self._special(0xFC, 1)
                    leg_s = abs(offset_rad) / slew_rad_s
                    arrivals = []
                    measured_positions = [start]
                    reference = start
                    last_step = time.monotonic()
                    for goal in (start + offset_rad, start):
                        leg_deadline = time.monotonic() + leg_s + 0.3
                        while time.monotonic() < leg_deadline:
                            now = time.monotonic()
                            if now >= self._deadline:
                                raise self._fault(
                                    "deadline", "trial exceeded total duration"
                                )
                            # Cap per-command progression after scheduling delays.
                            advance = slew_rad_s * min(now - last_step, 0.02)
                            last_step = now
                            reference += max(-advance, min(advance, goal - reference))
                            measured = self._command(reference, kp, kd, measured)
                            rows = self._batch()
                            self._guard(rows, expected_status=1)
                            measured = rows[self.selected]
                            measured_positions.append(measured["position_rad"])
                            time.sleep(0.01)
                        arrivals.append(
                            {
                                "goal_rad": goal,
                                "measured_rad": measured["position_rad"],
                                "error_rad": abs(goal - measured["position_rad"]),
                                "arrived": abs(goal - measured["position_rad"])
                                <= arrival_tolerance_rad,
                            }
                        )
                    self.report["arrivals"] = arrivals
                    resolution = 2 * self.spans[self.selected][0] / 65535
                    outward = arrivals[0]["measured_rad"] - start
                    self.report["motion"] = {
                        "start_rad": start,
                        "outbound_displacement_rad": outward,
                        "return_displacement_rad": arrivals[-1]["measured_rad"] - start,
                        "outbound_end_rad": arrivals[0]["measured_rad"],
                        "return_end_rad": arrivals[-1]["measured_rad"],
                        "peak_displacement_rad": max(
                            abs(value - start) for value in measured_positions
                        ),
                        "encoder_resolution_rad": resolution,
                        "observed_requested_direction": outward
                        * math.copysign(1, offset_rad)
                        > 2 * resolution,
                    }
            except BaseException as error:  # noqa: BLE001 — retain interrupts through physical cleanup.
                self.report["error"] = self._error(error)
            finally:
                self._cleanup()
            return self.report
        finally:
            self._operation.release()

    @staticmethod
    def _error(error):
        details = (
            error.as_dict() if hasattr(error, "as_dict") else {"message": str(error)}
        )
        return dict(type=type(error).__name__, **details)

    def _cleanup(self):
        try:
            if self._changed:
                try:
                    self._special(0xFD, 0, cleanup=True)
                    self.report["disabled_confirmed"] = True
                except BaseException as error:  # noqa: BLE001 — report uncertain disable faithfully.
                    self.report["cleanup_errors"].append(self._error(error))
                if self.report["disabled_confirmed"]:
                    for rid in (10, 9):
                        try:
                            self._set(rid, self.original[rid], cleanup=True)
                        except BaseException as error:  # noqa: BLE001 — retain each rollback failure.
                            self.report["cleanup_errors"].append(self._error(error))
                    self.report["restored"] = not self.report["cleanup_errors"]
        finally:
            self._bus.close()
            self._closed = True

    def close(self):
        """Request cancellation before waiting for the current bounded exchange."""
        self._cancel.set()
        with self._operation:
            if not self._closed:
                self._cleanup()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
