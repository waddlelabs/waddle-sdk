"""Bind the reviewed YAM motor interface to one explicitly selected USB bus."""

from __future__ import annotations

import math
import time

from ..runtime import FaultCode, RuntimeFault
from ._i2rt_runtime import verify_transport_runtime


def prepare(config):
    """Verify source before optional/vendor imports; constructing opens no bus.

    Each interface owns a fresh bus. The reviewed I2RT implementation retains
    motor codecs, feedback, gains, gravity compensation and gripper behavior.
    """
    source = verify_transport_runtime()
    from i2rt.motor_drivers.dm_driver import DMSingleMotorCanInterface

    from .usbcan import UsbCanBus

    def bus_factory(*, bustype, channel, bitrate):
        if channel != config.selector.channel or bitrate != 1_000_000:
            raise ValueError(
                "Reviewed USB YAM requires its selected channel at 1000000 bit/s"
            )
        return UsbCanBus(
            config.selector, bitrate=bitrate, libusb_library=config.libusb_library
        )

    def motor_interface_factory(**kwargs):
        return DMSingleMotorCanInterface(
            **kwargs, bustype="gs_usb", bus_factory=bus_factory
        )

    return motor_interface_factory, source


def disable_motors(chain, channel, evidence):
    """Confirm fresh disabled replies after both reviewed writers have drained.

    This is only the explicitly selected USB path. Submission retains the bus's
    original write budget. Neither interface closure nor a cached/healthy reply
    proves motor disable; all seven motors must acknowledge state zero.
    """
    import can

    interface = chain.motor_interface
    bus = interface.bus
    motors = list(chain.motor_list)
    identifiers = [row[0] for row in motors]
    evidence.update(status="uncertain", motors=[], errors=[], reply_timeout_s=0.05)
    if (
        len(motors) != 7
        or len(set(identifiers)) != 7
        or any(type(value) is not int or not 1 <= value <= 15 for value in identifiers)
    ):
        original = RuntimeFault(
            FaultCode.MOTOR_FAILURE,
            f"{channel}: USB motor disable requires the complete reviewed motor list",
            context={"channel": channel, "reason": "stop_motor_identity"},
        )
        evidence["errors"].append(original.as_dict())
        raise original
    errors = []
    for motor_id, _motor_type in motors:
        row = {"motor_id": motor_id, "disabled": False}
        evidence["motors"].append(row)
        try:
            expected_id = interface.receive_mode.get_receive_id(motor_id)
            # USB receive timestamps are host acquisition, never a hardware clock.
            submitted_after = time.time()
            bus.send(
                can.Message(
                    arbitration_id=interface._get_frame_id(motor_id),
                    data=[0xFF] * 7 + [0xFD],
                    is_extended_id=False,
                )
            )
            deadline = time.monotonic() + evidence["reply_timeout_s"]
            while time.monotonic() < deadline:
                reply = bus.recv(timeout=min(0.01, max(0, deadline - time.monotonic())))
                if reply is None or reply.arbitration_id != expected_id:
                    continue
                if (
                    not math.isfinite(reply.timestamp)
                    or reply.timestamp < submitted_after
                ):
                    continue
                if (
                    reply.is_extended_id
                    or reply.is_remote_frame
                    or reply.is_error_frame
                    or len(reply.data) != 8
                    or reply.data[0] & 0x0F != motor_id
                ):
                    raise RuntimeFault(
                        FaultCode.MOTOR_FAILURE,
                        f"{channel}: malformed USB motor-disable reply for motor {motor_id}",
                        context={
                            "channel": channel,
                            "reason": "stop_reply_invalid",
                            "motor_id": motor_id,
                        },
                    )
                state = reply.data[0] >> 4
                row.update(
                    state=state,
                    data_hex=bytes(reply.data).hex(),
                    timestamp=reply.timestamp,
                )
                if state == 0:
                    row["disabled"] = True
                    break
                # An enabled scan may have been in flight when the disable was
                # submitted. Keep it as evidence; it cannot satisfy the stop.
            if not row["disabled"]:
                raise RuntimeFault(
                    FaultCode.MOTOR_FAILURE,
                    f"{channel}: motor {motor_id} did not confirm disabled within 0.050 s",
                    context={
                        "channel": channel,
                        "reason": "stop_unconfirmed",
                        "motor_id": motor_id,
                        "last_state": row.get("state"),
                    },
                )
        except BaseException as error:  # noqa: BLE001 -- attempt every motor, retain the original first fault.
            errors.append(error)
            evidence["errors"].append(RuntimeFault.from_exception(error).as_dict())
    if errors:
        raise errors[0]
    evidence["status"] = "all_motors_disabled"
