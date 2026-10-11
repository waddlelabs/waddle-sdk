"""Exact-device classic CAN transport for Candlelight/gs_usb firmware.

This optional provider uses python-can's public bus contract and PyUSB directly.
Opening starts the selected CAN channel; importing/enumerating does not. The
provider is a qualification candidate, not the default YAM transport. No controller
or motor policy belongs here. See ``docs/porting/robot.md`` for qualification scope.
"""

from __future__ import annotations

import math
import struct
import threading
import time
from collections import deque
from dataclasses import asdict

import can
import usb.core
import usb.util
from usb.backend import libusb1

from ..scheduling import periodic_thread
from .usbcan_config import UsbCanSelector

_RX_ECHO = 0xFFFFFFFF
_EXTENDED = 0x80000000
_REMOTE = 0x40000000
_ERROR = 0x20000000
_BERR_REPORTING = 1 << 12
_FRAME = struct.Struct("<IIBBBB8s")


class UsbCanFault(RuntimeError):
    """A terminal scoped transport fault; intentionally not a retryable CanError."""

    def __init__(self, code, detail, *, selector, operation, cause=None, **context):
        super().__init__(detail)
        self.code, self.detail = code, detail
        self.context = {
            "operation": operation,
            "device": asdict(selector),
            **context,
        }
        if cause is not None:
            self.__cause__ = cause
            self.context["original_error"] = {
                "type": f"{type(cause).__module__}.{type(cause).__qualname__}",
                "message": str(cause),
                "errno": getattr(cause, "errno", None),
                "backend_error_code": getattr(cause, "backend_error_code", None),
            }

    def as_dict(self):
        return {"code": self.code, "detail": self.detail, "context": self.context}


class _Device:
    """One claimed interface, no USB reset, kernel detach, index scan or peer writes."""

    def __init__(self, selector, bitrate, library):
        self.selector, self.usb = selector, None
        self.claimed = self.mode_requested = False
        self.berr_reporting = False
        self.timeout_ms = 100
        try:
            backend = libusb1.get_backend(
                **({"find_library": lambda _name: library} if library else {})
            )
            if backend is None:
                raise RuntimeError("libusb backend is unavailable")
            self.usb = usb.core.find(
                idVendor=selector.vendor_id,
                idProduct=selector.product_id,
                bus=selector.bus,
                address=selector.address,
                backend=backend,
            )
            if self.usb is None:
                raise RuntimeError("Declared USB-CAN location is absent")
            if self.usb.serial_number != selector.serial_number:
                raise RuntimeError("Declared USB-CAN serial differs at this location")
            interface = self.usb.get_active_configuration()[(0, 0)]
            endpoints = {(ep.bEndpointAddress, ep.bmAttributes & 3) for ep in interface}
            if not {(0x81, 2), (0x02, 2)} <= endpoints:
                raise RuntimeError(
                    "Selected interface lacks the classic gs_usb bulk endpoints"
                )
            usb.util.claim_interface(self.usb, 0)
            self.claimed = True
            # Reset only this CAN channel before programming it. Never reset USB.
            self.mode_requested = True
            self.control(2, struct.pack("<II", 0, 0))
            self.control(0, struct.pack("<I", 0x0000BEEF))
            info = self.control(5, 12, read=True)
            _r1, _r2, _r3, last_channel, _firmware, _hardware = struct.unpack(
                "<4B2I", info
            )
            if last_channel != 0:
                raise RuntimeError(
                    "This provider requires a single-channel gs_usb device"
                )
            values = struct.unpack("<10I", self.control(4, 40, read=True))
            (
                features,
                clock,
                t1_min,
                t1_max,
                t2_min,
                t2_max,
                sjw_max,
                brp_min,
                brp_max,
                brp_inc,
            ) = values
            timing = can.BitTiming.from_sample_point(
                f_clock=clock, bitrate=bitrate, sample_point=87.5
            )
            if not (
                t1_min <= timing.tseg1 <= t1_max
                and t2_min <= timing.tseg2 <= t2_max
                and 1 <= timing.sjw <= sjw_max
                and brp_min <= timing.brp <= brp_max
                and brp_inc > 0
                and (timing.brp - brp_min) % brp_inc == 0
            ):
                raise RuntimeError(
                    "Requested bitrate exceeds declared device timing limits"
                )
            self.control(
                1,
                struct.pack(
                    "<5I", 1, timing.tseg1 - 1, timing.tseg2, timing.sjw, timing.brp
                ),
            )
            self.berr_reporting = bool(features & _BERR_REPORTING)
            # Disable hardware timestamps/FD/padding: the exact 20-byte classic
            # frame format below is then independent of timestamp wraparound.
            self.control(2, struct.pack("<II", 1, features & _BERR_REPORTING))
        except Exception as error:
            fault = self.fault("open", error)
            try:
                self.close()
            except UsbCanFault as cleanup:
                fault.context["cleanup_error"] = cleanup.as_dict()
            raise fault from error

    def fault(self, operation, error):
        return UsbCanFault(
            "hardware.usb_can_io",
            str(error),
            selector=self.selector,
            operation=operation,
            cause=error,
        )

    def control(self, request, data, *, read=False):
        return self.usb.ctrl_transfer(
            0xC1 if read else 0x41, request, 0, 0, data, timeout=self.timeout_ms
        )

    def read(self):
        try:
            return bytes(self.usb.read(0x81, _FRAME.size, timeout=5))
        except usb.core.USBTimeoutError:
            return None
        except Exception as error:
            raise self.fault("read", error) from error

    def write(self, packet, timeout_ms):
        try:
            count = self.usb.write(0x02, packet, timeout=timeout_ms)
            if count != len(packet):
                raise RuntimeError(f"Short USB-CAN write: {count}/{len(packet)} bytes")
        except Exception as error:
            raise self.fault("write", error) from error

    def close(self):
        errors = []
        if self.mode_requested:
            self.mode_requested = False
            try:
                self.control(2, struct.pack("<II", 0, 0))
            except Exception as error:  # noqa: BLE001 -- drain every owned USB resource
                errors.append(self.fault("stop CAN channel", error))
        if self.claimed:
            self.claimed = False
            try:
                usb.util.release_interface(self.usb, 0)
            except Exception as error:  # noqa: BLE001 -- drain every owned USB resource
                errors.append(self.fault("release USB interface", error))
        if self.usb is not None:
            device, self.usb = self.usb, None
            try:
                usb.util.dispose_resources(device)
            except Exception as error:  # noqa: BLE001 -- drain every owned USB resource
                errors.append(self.fault("dispose USB resources", error))
        if errors:
            errors[0].context["additional_errors"] = [e.as_dict() for e in errors[1:]]
            raise errors[0]


class UsbCanBus(can.BusABC):
    """Bounded USB I/O and a bounded receive queue with terminal fault fencing.

    ``recv(0)`` only inspects the queue. USB timeouts occur in a separate reader;
    access/disconnect faults, malformed frames and overflows fence reads AND sends.
    Frames are stamped at host acquisition, without claiming a hardware clock.
    ``send`` acknowledges USB submission, not CAN delivery or motor arrival.
    """

    def __init__(
        self,
        selector: UsbCanSelector,
        *,
        bitrate=1_000_000,
        capacity=256,
        write_timeout_s=0.005,
        libusb_library=None,
        can_filters=None,
    ):
        if not isinstance(selector, UsbCanSelector):
            raise TypeError("selector must be UsbCanSelector")
        if type(bitrate) is not int or not 1 <= bitrate <= 1_000_000:
            raise ValueError("Classic USB-CAN bitrate must be 1..1000000 bit/s")
        if type(capacity) is not int or not 1 <= capacity <= 4096:
            raise ValueError("USB-CAN queue capacity must be 1..4096")
        if (
            type(write_timeout_s) not in (int, float)
            or not math.isfinite(write_timeout_s)
            or not 0 < write_timeout_s <= 0.1
        ):
            raise ValueError(
                "USB-CAN write timeout must be finite, positive and at most 100 ms"
            )
        self.selector, self._capacity = selector, capacity
        self._write_timeout_s = write_timeout_s
        self._condition, self._tx_lock = threading.Condition(), threading.Lock()
        self._messages, self._fault = deque(), None
        self._closing, self._closed = threading.Event(), False
        self._reader, self._device = None, None
        self._echo = 0
        self.channel_info = (
            f"gs_usb:{selector.serial_number}@{selector.bus}:{selector.address}"
        )
        self._can_protocol = can.CanProtocol.CAN_20
        super().__init__(channel=self.channel_info, can_filters=can_filters)
        try:
            self._device = _Device(selector, bitrate, libusb_library)
            self._reader = threading.Thread(
                target=self._read, name=f"usbcan-{selector.serial_number}", daemon=True
            )
            self._reader.start()
        except Exception as error:
            try:
                self.shutdown()
            except UsbCanFault as cleanup:
                if isinstance(error, UsbCanFault):
                    error.context["bus_cleanup_error"] = cleanup.as_dict()
            raise

    def _error(self, code, detail, operation, **context):
        return UsbCanFault(
            code, detail, selector=self.selector, operation=operation, **context
        )

    def _latch(self, fault):
        with self._condition:
            if self._fault is None:
                self._fault = fault
            self._condition.notify_all()

    def _check(self):
        if self._fault is not None:
            raise self._fault
        if self._closing.is_set():
            raise self._error(
                "hardware.usb_can_closed", "USB-CAN bus is closing or closed", "use bus"
            )

    def _read(self):
        try:
            while not self._closing.is_set():
                packet = self._device.read()
                if packet is None:
                    continue
                acquired = time.time()
                if len(packet) != _FRAME.size:
                    raise self._error(
                        "hardware.usb_can_frame",
                        "Malformed classic USB-CAN frame length",
                        "decode",
                        packet_hex=packet.hex(),
                    )
                echo, can_id, dlc, channel, flags, reserved, payload = _FRAME.unpack(
                    packet
                )
                if channel != 0 or dlc > 8 or reserved or flags & ~1:
                    raise self._error(
                        "hardware.usb_can_frame",
                        "Unsupported or malformed classic USB-CAN frame",
                        "decode",
                        packet_hex=packet.hex(),
                    )
                if flags & 1:
                    raise self._error(
                        "hardware.usb_can_overflow",
                        "USB-CAN device reports lost frames",
                        "read",
                        packet_hex=packet.hex(),
                    )
                if not can_id & (_EXTENDED | _ERROR) and can_id & 0x1FFFFFFF > 0x7FF:
                    raise self._error(
                        "hardware.usb_can_frame",
                        "Invalid standard CAN identifier",
                        "decode",
                        packet_hex=packet.hex(),
                    )
                if can_id & _ERROR:
                    raise self._error(
                        "hardware.can_bus_error",
                        "CAN controller reported an error frame",
                        "read",
                        can_id=can_id,
                        data_hex=payload[:dlc].hex(),
                    )
                if echo != _RX_ECHO:
                    continue
                message = can.Message(
                    timestamp=acquired,
                    arbitration_id=can_id & 0x1FFFFFFF,
                    is_extended_id=bool(can_id & _EXTENDED),
                    is_remote_frame=bool(can_id & _REMOTE),
                    dlc=dlc,
                    data=b"" if can_id & _REMOTE else payload[:dlc],
                    channel=self.channel_info,
                    check=True,
                )
                with self._condition:
                    if len(self._messages) >= self._capacity:
                        raise self._error(
                            "hardware.usb_can_overflow",
                            "Host USB-CAN receive queue overflow",
                            "enqueue",
                            capacity=self._capacity,
                        )
                    self._messages.append(message)
                    self._condition.notify_all()
        except Exception as error:  # noqa: BLE001 -- surface thread faults to all callers
            self._latch(
                error
                if isinstance(error, UsbCanFault)
                else self._device.fault("receive worker", error)
            )

    def recv(self, timeout=None):
        if timeout is not None and (
            type(timeout) not in (int, float)
            or not math.isfinite(timeout)
            or timeout < 0
        ):
            raise ValueError("CAN receive timeout must be finite, non-negative or None")
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._condition:
            while True:
                self._check()
                while self._messages:
                    message = self._messages.popleft()
                    if self._matches_filters(message):
                        return message
                remaining = (
                    None if deadline is None else max(0, deadline - time.monotonic())
                )
                if remaining == 0:
                    return None
                # Only the blocking primitive requests this budget. Codec and
                # customer work run under the caller's restored host policy.
                with periodic_thread(
                    period_s=0.004, computation_s=0.001, constraint_s=0.002
                ):
                    self._condition.wait(remaining)

    def _recv_internal(self, timeout):
        return self.recv(timeout), True

    def send(self, msg, timeout=None):
        if msg.is_fd or msg.is_error_frame:
            raise ValueError("USB-CAN sends support classic data/remote frames")
        can.Message(
            timestamp=msg.timestamp,
            arbitration_id=msg.arbitration_id,
            is_extended_id=msg.is_extended_id,
            is_remote_frame=msg.is_remote_frame,
            dlc=msg.dlc,
            data=msg.data,
            bitrate_switch=msg.bitrate_switch,
            error_state_indicator=msg.error_state_indicator,
            check=True,
        )
        if timeout is not None and (
            type(timeout) not in (int, float)
            or not math.isfinite(timeout)
            or timeout <= 0
        ):
            raise ValueError("USB-CAN sends require a finite positive timeout")
        budget = (
            self._write_timeout_s
            if timeout is None
            else min(timeout, self._write_timeout_s)
        )
        if type(budget) not in (int, float) or not math.isfinite(budget) or budget <= 0:
            raise ValueError("USB-CAN sends require a finite positive timeout")
        started = time.monotonic()
        if not self._tx_lock.acquire(timeout=budget):
            raise self._error(
                "hardware.usb_can_write_timeout",
                "USB-CAN write lock deadline expired",
                "send",
                phase="write_lock",
                timeout_s=budget,
                elapsed_s=time.monotonic() - started,
            )
        try:
            self._check()
            elapsed_before_write = time.monotonic() - started
            remaining = budget - elapsed_before_write
            if remaining <= 0:
                raise self._error(
                    "hardware.usb_can_write_timeout",
                    "USB-CAN write deadline expired",
                    "send",
                    phase="before_submission",
                    timeout_s=budget,
                    elapsed_s=elapsed_before_write,
                )
            can_id = (
                msg.arbitration_id
                | (_EXTENDED if msg.is_extended_id else 0)
                | (_REMOTE if msg.is_remote_frame else 0)
            )
            packet = _FRAME.pack(
                self._echo, can_id, msg.dlc, 0, 0, 0, bytes(msg.data).ljust(8, b"\0")
            )
            self._echo = (self._echo + 1) % _RX_ECHO
            try:
                # USB timeout zero means infinite. Round UP with a 1 ms floor;
                # report any resulting budget overrun rather than hiding it.
                usb_timeout_ms = max(1, math.ceil(remaining * 1000))
                self._device.write(packet, usb_timeout_ms)
                elapsed_s = time.monotonic() - started
                if elapsed_s > budget:
                    raise self._error(
                        "hardware.usb_can_write_timeout",
                        "USB-CAN submission exceeded its deadline",
                        "send",
                        timeout_s=budget,
                        elapsed_s=elapsed_s,
                        elapsed_before_write_s=elapsed_before_write,
                        usb_timeout_ms=usb_timeout_ms,
                        phase="after_submission",
                        timing_provenance="host caller wall time; includes Python scheduling",
                    )
            except Exception as error:
                self._latch(error)
                raise
        finally:
            self._tx_lock.release()

    @property
    def state(self):
        self._check()
        return can.BusState.ACTIVE

    @property
    def transport_evidence(self):
        self._check()
        return {
            "kind": "gs_usb_classic/v1",
            "device": asdict(self.selector),
            "berr_reporting": self._device.berr_reporting,
            "receive_timestamp": "host_acquisition",
            "queue_capacity": self._capacity,
            "qualified": False,
        }

    def shutdown(self):
        if self._closed:
            return
        self._closing.set()
        with self._condition:
            self._condition.notify_all()
        super().shutdown()
        if self._reader is not None:
            self._reader.join(timeout=0.2)
            if self._reader.is_alive():
                raise self._error(
                    "hardware.usb_can_drain",
                    "USB-CAN reader has not drained; device remains owned",
                    "shutdown",
                )
        # Serialize release against an in-flight, bounded USB write.
        if not self._tx_lock.acquire(timeout=0.2):
            raise self._error(
                "hardware.usb_can_drain",
                "USB-CAN write has not drained; device remains owned",
                "shutdown",
            )
        try:
            if self._device is not None:
                self._device.close()
                self._device = None
            self._closed = True
        finally:
            self._tx_lock.release()


__all__ = ["UsbCanBus", "UsbCanFault", "UsbCanSelector"]
