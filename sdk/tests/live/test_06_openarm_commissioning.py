"""Explicit attended single-ID qualification; never selected from a monitor profile.

Bench JSON must contain torque_release_authorized=true and
openarm_commissioning={part, motor_id, feedback_id, guards, trial}. ``guards`` is the
explicit Guards constructor mapping; ``trial`` explicitly supplies offset_rad,
kp, kd, slew_rad_s, duration_s and arrival_tolerance_rad. The declared monitor
part supplies exact USB/bus/motor identities only. Its observation envelope is
not a mechanical limit or authorization to actuate. No Site/run is opened.
"""

import json
import struct
import time
from pathlib import Path
from uuid import uuid4

import pytest
from waddle_sdk import load_site
from waddle_sdk.robots import _openarm_transport as wire
from waddle_sdk.robots.openarm_commissioning import CommissioningOwner, Guards

pytestmark = [pytest.mark.motion, pytest.mark.requires_site]


def test_attended_single_motor_watchdog_and_bounded_relative_trial(
    request, monkeypatch, part
):
    assert request.config.getoption("--live")
    config = request.config.live_bench
    profile = config.get("openarm_commissioning")
    if not profile or profile.get("part") != part:
        pytest.skip("requires an explicit single-part OpenArm commissioning profile")
    assert config.get("torque_release_authorized") is True
    assert set(profile) == {"part", "motor_id", "feedback_id", "guards", "trial"}
    assert set(profile["trial"]) == {
        "offset_rad",
        "kp",
        "kd",
        "slew_rad_s",
        "duration_s",
        "arrival_tolerance_rad",
    }
    guards = Guards(**profile["guards"])
    original = load_site(config["site"])
    declaration = original.manifest["parts"][part]
    assert declaration["driver"] == "waddle_sdk.robots.openarm:arm"
    connection = declaration["connection"]
    motors = tuple(
        wire.MotorAddress(a, b)
        for a, b in zip(
            connection["motor_ids"], connection["feedback_ids"], strict=True
        )
    )
    selected = wire.MotorAddress(profile["motor_id"], profile["feedback_id"])
    assert selected in motors
    evidence = {
        "profile": profile,
        "part_configuration": declaration,
        "wire": [],
        "result": None,
    }
    open_socket = wire._open_socket

    class RecordedSocket:
        def __init__(self, interface):
            self.socket = open_socket(interface)

        def settimeout(self, value):
            return self.socket.settimeout(value)

        def send(self, raw):
            can_id, size, data = wire._CAN_FRAME.unpack(raw)
            if can_id == 0x7FF:
                motor, opcode, register = struct.unpack("<HBB", data[:4])
                allowed = motor in {m.send_id for m in motors} and (
                    (
                        opcode == 0x33
                        and register in (7, 8, 9, 10, 21, 22, 23)
                        and data[4:] == bytes(4)
                    )
                    or (opcode == 0xCC and data[3:] == bytes(5))
                    or (
                        opcode == 0x55
                        and motor == selected.send_id
                        and register in (9, 10)
                    )
                )
            else:
                # MIT encoder plus enable/disable; the zero special packet is
                # explicitly refused before any physical send.
                allowed = can_id == selected.send_id and (
                    data[:7] != bytes([255]) * 7 or data[7] in (0xFB, 0xFC, 0xFD)
                )
            assert allowed and size == 8, "unexpected commissioning packet refused"
            evidence["wire"].append(
                {
                    "direction": "tx",
                    "can_id": can_id,
                    "payload": data.hex(),
                    "host_monotonic_ns": time.monotonic_ns(),
                }
            )
            return self.socket.send(raw)

        def recvmsg(self, *args):
            result = self.socket.recvmsg(*args)
            evidence["wire"].append(
                {
                    "direction": "rx",
                    "frame": result[0].hex(),
                    "host_monotonic_ns": time.monotonic_ns(),
                }
            )
            return result

        def close(self):
            self.socket.close()

    monkeypatch.setattr(wire, "_open_socket", RecordedSocket)
    directory = Path(config["evidence_directory"])
    directory.mkdir(parents=True, exist_ok=True)
    output = directory / f"openarm-commissioning-{uuid4().hex}.json"
    request.node.user_properties.append(("evidence", str(output)))
    try:
        with CommissioningOwner(
            interface=connection["interface"],
            usb_serial=connection["usb_serial"],
            motors=motors,
            selected=selected,
            guards=guards,
        ) as owner:
            evidence["result"] = owner.run(**profile["trial"])
        result = evidence["result"]
        assert result["error"] is None, result
        assert result["cleanup_errors"] == [], result
        assert result["disabled_confirmed"] and result["restored"], result
        assert result["watchdog"]["status"] == 13, result
        # Observed requested-direction travel and exact arrival are separate.
        if profile["trial"]["offset_rad"]:
            assert result["motion"]["observed_requested_direction"], result
    except BaseException as error:
        evidence["test_error"] = CommissioningOwner._error(error)
        raise
    finally:
        output.write_text(
            json.dumps(evidence, indent=2, default=str) + "\n", encoding="utf-8"
        )
