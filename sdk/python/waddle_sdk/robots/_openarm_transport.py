"""Bounded, read-only Damiao transactions on one explicitly bound SocketCAN bus.

Protocol source: Enactic openarm_can f340d4b808fb177e1f297af54eb55fd51c6c7c10,
src/openarm/damiao_motor/dm_motor_control.cpp. Only parameter-read (0x33)
and feedback-refresh (0xCC) requests can be constructed here. Encoding spans
are not mechanical joint limits. Linux receive timestamps prove receipt
freshness, not motor acquisition time or synchronized sampling.
"""

from __future__ import annotations

import math
import re
import socket
import struct
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from ..ownership import SiteOwnershipError, _SiteLock
from ..runtime import FaultCode, RuntimeFault
from . import socketcan

_CAN_FRAME = struct.Struct("=IB3x8s")
_TIMESPEC = struct.Struct("@ll")
_SO_TIMESTAMPNS = getattr(socket, "SO_TIMESTAMPNS", 35)
# Linux UAPI linux/can/raw.h; some Python builds omit this socket constant.
_CAN_RAW_ERR_FILTER = getattr(socket, "CAN_RAW_ERR_FILTER", 2)
_CAN_ERR_FLAG = 0x20000000
_READABLE_PARAMETERS = frozenset((7, 8, 10, 21, 22, 23))
_FAULT_NAMES = {
    8: "overvoltage",
    9: "undervoltage",
    10: "overcurrent",
    11: "mos_overheat",
    12: "coil_overheat",
    13: "communication_lost",
    14: "overload",
}


@dataclass(frozen=True)
class MotorAddress:
    send_id: int
    receive_id: int

    def __post_init__(self):
        if any(
            type(value) is not int or not 1 <= value < 0x7FF
            for value in (self.send_id, self.receive_id)
        ):
            raise ValueError("OpenArm motor IDs must be integers in 1..2046")


@dataclass(frozen=True)
class MotorFeedback:
    """Raw motor-coordinate feedback and kernel receipt evidence, not joint FK."""

    send_id: int
    receive_id: int
    position_rad: float
    velocity_rad_s: float
    status: int
    mos_temperature_c: int
    rotor_temperature_c: int
    received_unix_ns: int


def _usb_serial(interface: str) -> str:
    device = (Path("/sys/class/net") / interface / "device").resolve(strict=True)
    for parent in (device, *device.parents):
        serial = parent / "serial"
        if serial.is_file():
            return serial.read_text(encoding="ascii").strip()
    raise ValueError(f"SocketCAN {interface}: no USB adapter serial is available")


def _open_socket(interface: str):
    if not hasattr(socket, "AF_CAN") or _TIMESPEC.size != 16:
        raise RuntimeFault(
            FaultCode.UNSUPPORTED, "OpenArm monitoring requires 64-bit Linux SocketCAN"
        )
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        sock.setsockopt(socket.SOL_SOCKET, _SO_TIMESTAMPNS, 1)
        # Receive bus errors as well as ordinary frames. Never mistake bus-off
        # or controller errors for an otherwise healthy no-response timeout.
        sock.setsockopt(
            socket.SOL_CAN_RAW, _CAN_RAW_ERR_FILTER, struct.pack("=I", 0x1FFFFFFF)
        )
        sock.bind((interface,))
    except BaseException:
        sock.close()
        raise
    return sock


class ReadOnlyBus:
    """One exclusive local query owner; no control, mode, or enable encoder.

    A failed transaction poisons this handle until close/reopen, so its delayed
    reply cannot satisfy a retry. Each successful read drains queued traffic and
    rejects frames whose kernel receive time precedes that motor's request.
    The protocol has no sequence counter: later unsolicited state feedback is
    fresh received feedback, not proof of a causally paired sensor sample.
    """

    def __init__(
        self,
        *,
        interface: str,
        usb_serial: str,
        bitrate: int,
        configure_can: bool,
        timeout_s: float,
    ):
        if not isinstance(interface, str) or not re.fullmatch(
            r"[\w.-]{1,15}", interface, re.ASCII
        ):
            raise ValueError("OpenArm interface must be an exact Linux interface name")
        if (
            not isinstance(usb_serial, str)
            or not usb_serial
            or usb_serial.strip() != usb_serial
        ):
            raise ValueError("OpenArm requires the exact nonempty USB adapter serial")
        if type(bitrate) is not int or not 1 <= bitrate <= 10_000_000:
            raise ValueError("OpenArm bitrate must be an integer in 1..10000000")
        if type(configure_can) is not bool:
            raise ValueError("OpenArm configure_can must be a boolean")
        if (
            isinstance(timeout_s, bool)
            or not isinstance(timeout_s, (int, float))
            or not math.isfinite(timeout_s)
            or not 0 < timeout_s <= 1
        ):
            raise ValueError("OpenArm timeout_s must be finite in (0, 1]")
        self.interface = interface
        self.usb_serial = usb_serial
        self.timeout_s = float(timeout_s)
        self._lock = threading.Lock()
        self._socket = None
        self._ownership = None
        self._failure: RuntimeFault | None = None
        try:
            self._check_identity()
            # Also fences two differently named Site owners using this adapter.
            # It cannot exclude unrelated raw-CAN programs or other OS users.
            self._ownership = _SiteLock(f"openarm-usb:{usb_serial}")
            if configure_can:
                socketcan.ensure_socketcan_up(interface, bitrate=bitrate)
            else:
                link = socketcan.read_socketcan_state(interface)
                if not link.up or link.bitrate != bitrate:
                    raise self._fault(
                        "link_unavailable",
                        "declared CAN link must be up at the declared bitrate",
                        up=link.up,
                        actual_bitrate=link.bitrate,
                        expected_bitrate=bitrate,
                    )
            self._socket = _open_socket(interface)
            self._check_identity()
        except BaseException as error:
            self.close()
            if isinstance(error, (RuntimeFault, KeyboardInterrupt, SystemExit)):
                raise
            code = (
                FaultCode.BUSY
                if isinstance(error, SiteOwnershipError)
                else FaultCode.TRANSPORT_LOST
            )
            raise self._fault("open_failed", str(error), code=code) from error

    def _fault(
        self, reason: str, detail: str, *, code=FaultCode.TRANSPORT_LOST, **context
    ):
        return RuntimeFault(
            code,
            f"OpenArm {self.interface}: {detail}",
            context={
                "reason": f"openarm.{reason}",
                "interface": self.interface,
                "usb_serial": self.usb_serial,
                **context,
            },
        )

    def _check_identity(self):
        actual = _usb_serial(self.interface)
        if actual != self.usb_serial:
            raise self._fault(
                "identity_mismatch",
                "USB adapter serial does not match",
                expected_serial=self.usb_serial,
                actual_serial=actual,
            )

    def _receive(self, timeout: float):
        self._socket.settimeout(timeout)
        try:
            raw, ancillary, flags, _address = self._socket.recvmsg(
                _CAN_FRAME.size, socket.CMSG_SPACE(_TIMESPEC.size)
            )
        except (TimeoutError, BlockingIOError):
            return None
        if (
            flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC)
            or len(raw) != _CAN_FRAME.size
        ):
            raise self._fault("malformed_frame", "truncated CAN frame or timestamp")
        can_id, size, data = _CAN_FRAME.unpack(raw)
        if can_id & _CAN_ERR_FLAG:
            raise self._fault(
                "can_error",
                "CAN controller reported an error",
                can_id=can_id,
                payload=data.hex(),
            )
        stamps = [
            payload
            for level, kind, payload in ancillary
            if level == socket.SOL_SOCKET and kind == _SO_TIMESTAMPNS
        ]
        if len(stamps) != 1 or len(stamps[0]) != _TIMESPEC.size:
            raise self._fault(
                "missing_timestamp", "CAN frame has no exact kernel receive timestamp"
            )
        sec, nanos = _TIMESPEC.unpack(stamps[0])
        if sec < 0 or not 0 <= nanos < 1_000_000_000:
            raise self._fault("malformed_timestamp", "invalid kernel receive timestamp")
        return can_id, size, data, sec * 1_000_000_000 + nanos

    def _exchange(self, motors: tuple[MotorAddress, ...], register: int | None):
        if not self._lock.acquire(timeout=self.timeout_s):
            raise self._fault(
                "busy", "another CAN transaction did not finish", code=FaultCode.BUSY
            )
        try:
            if self._failure is not None:
                raise self._failure
            if self._socket is None:
                raise self._fault(
                    "closed", "CAN query owner is closed", code=FaultCode.NOT_OPEN
                )
            deadline = time.monotonic() + self.timeout_s
            # A bounded drain never interprets cached frames as new device data.
            for _ in range(256):
                if self._receive(0) is None:
                    break
            else:
                raise self._fault(
                    "receive_backlog", "CAN receive backlog exceeds 256 frames"
                )
            pending = {}
            for motor in motors:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise self._fault(
                        "query_timeout",
                        "query deadline expired before sending all requests",
                        motor_id=motor.send_id,
                        register=register,
                    )
                opcode = 0xCC if register is None else 0x33
                payload = struct.pack("<HBB4x", motor.send_id, opcode, register or 0)
                sent_ns = time.time_ns()
                self._socket.settimeout(remaining)
                count = self._socket.send(_CAN_FRAME.pack(0x7FF, 8, payload))
                if count != _CAN_FRAME.size:
                    raise self._fault(
                        "short_write",
                        "incomplete CAN query write",
                        motor_id=motor.send_id,
                    )
                pending[motor.receive_id] = (motor, sent_ns)
            results = {}
            for _ in range(1024):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                frame = self._receive(remaining)
                if frame is None:
                    break
                can_id, size, data, received_ns = frame
                if can_id > 0x7FF and (can_id & 0x1FFFFFFF) in pending:
                    raise self._fault(
                        "malformed_feedback",
                        "expected a standard CAN data frame",
                        can_id=can_id,
                        register=register,
                    )
                if can_id not in pending:
                    continue
                motor, sent_ns = pending[can_id]
                if received_ns < sent_ns:
                    continue
                if size != 8:
                    raise self._fault(
                        "malformed_feedback",
                        "motor reply requires eight bytes",
                        motor_id=motor.send_id,
                        receive_id=can_id,
                        size=size,
                    )
                if register is not None:
                    if data[:4] != struct.pack("<HBB", motor.send_id, 0x33, register):
                        continue
                else:
                    # Delayed parameter replies are not motor state packets.
                    if data[:2] == struct.pack("<H", motor.send_id) and data[2] in (
                        0x33,
                        0x55,
                    ):
                        continue
                    if data[0] & 0xF != motor.send_id & 0xF:
                        raise self._fault(
                            "motor_id_mismatch",
                            "feedback payload motor ID does not match",
                            motor_id=motor.send_id,
                            receive_id=can_id,
                            payload=data.hex(),
                        )
                results[motor] = (data, received_ns)
                del pending[can_id]
                if not pending:
                    return results
            raise self._fault(
                "feedback_timeout",
                "incomplete fresh motor reply set",
                motor_ids=[m.send_id for m, _ in pending.values()],
                receive_ids=list(pending),
                register=register,
                timeout_s=self.timeout_s,
            )
        except RuntimeFault as error:
            self._failure = error
            raise
        except OSError as error:
            fault = self._fault(
                "socket_error",
                str(error),
                errno=error.errno,
                motor_ids=[motor.send_id for motor in motors],
                register=register,
            )
            self._failure = fault
            raise fault from error
        finally:
            self._lock.release()

    def parameter(self, motor: MotorAddress, register: int) -> float | int:
        """Read a supported identity/mode/encoding parameter, never write one."""
        if type(register) is not int or register not in _READABLE_PARAMETERS:
            raise ValueError("Unsupported OpenArm read-only parameter")
        data, _stamp = self._exchange((motor,), register)[motor]
        value = struct.unpack("<f" if register in (21, 22, 23) else "<I", data[4:])[0]
        if not math.isfinite(value):
            fault = self._fault(
                "invalid_parameter",
                "motor parameter is not finite",
                motor_id=motor.send_id,
                register=register,
                payload=data.hex(),
            )
            self._failure = fault
            raise fault
        return value

    def feedback(
        self, motors: tuple[MotorAddress, ...], spans: tuple[tuple[float, float], ...]
    ):
        """Request one complete new state batch; positions/velocities stay raw radians."""
        rows = self._exchange(motors, None)
        result = []
        for motor, (pmax, vmax) in zip(motors, spans, strict=True):
            data, received_ns = rows[motor]
            status = data[0] >> 4
            if status not in (0, 1):
                reason = _FAULT_NAMES.get(status, "unknown_status")
                fault = self._fault(
                    reason,
                    f"motor {motor.send_id} reported {reason}",
                    code=FaultCode.MOTOR_FAILURE,
                    motor_id=motor.send_id,
                    receive_id=motor.receive_id,
                    status=status,
                    payload=data.hex(),
                )
                self._failure = fault
                raise fault
            position = (int.from_bytes(data[1:3], "big") / 65535 * 2 - 1) * pmax
            velocity = (((data[3] << 4) | (data[4] >> 4)) / 4095 * 2 - 1) * vmax
            result.append(
                MotorFeedback(
                    motor.send_id,
                    motor.receive_id,
                    position,
                    velocity,
                    status,
                    data[6],
                    data[7],
                    received_ns,
                )
            )
        return tuple(result)

    def close(self):
        """Close the socket and local ownership only; transmit no CAN frame."""
        with self._lock:
            if self._socket is not None:
                self._socket.close()
                self._socket = None
            if self._ownership is not None:
                self._ownership.release()
                self._ownership = None
