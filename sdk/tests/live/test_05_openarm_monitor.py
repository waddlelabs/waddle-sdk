"""Read both real OpenArm buses through a passive Site and retain wire evidence."""

import hashlib
import json
import struct
import time
from dataclasses import asdict, replace
from pathlib import Path
from uuid import uuid4

import numpy as np
import pytest
from waddle_sdk import load_site
from waddle_sdk.robots import _openarm_transport as wire
from waddle_sdk.runtime import RuntimeFault, SupportFact

pytestmark = [pytest.mark.openarm_monitor, pytest.mark.requires_site]


def test_two_openarm_parts_supply_fresh_feedback_without_actuation(
    request, monkeypatch
):
    assert request.config.getoption("--live")
    config = request.config.live_bench
    selected = tuple(config["parts"])
    original = load_site(config["site"])
    declarations = {name: original.manifest["parts"][name] for name in selected}
    assert len(selected) == 2 and all(
        row["driver"] == "waddle_sdk.robots.openarm:arm" and row["posture"] == "monitor"
        for row in declarations.values()
    )
    # Preserve original Site identity, paths and envelope. No camera or unrelated
    # robot owner is opened by this read-only acceptance case.
    site = replace(
        original, manifest={**original.manifest, "parts": declarations, "cameras": {}}
    )
    report = {
        "site_id": site.id,
        "selected_parts": list(selected),
        "samples": [],
        "wire": [],
        "closed": False,
        "scope": "raw motor coordinates and kernel receipt freshness only",
        "source_manifest_sha256": hashlib.sha256(
            json.dumps(dict(original.describe()), sort_keys=True).encode()
        ).hexdigest(),
        "part_configuration": declarations,
    }
    interfaces = {
        row["connection"]["interface"]: set(row["connection"]["motor_ids"])
        for row in declarations.values()
    }
    assert len(interfaces) == 2, (
        "each selected part must have an independently bound bus"
    )
    sockets = []
    open_socket = wire._open_socket

    class GuardedSocket:
        def __init__(self, interface):
            self.interface = interface
            self.socket = open_socket(interface)
            self.closed = False
            sockets.append(self)

        def settimeout(self, value):
            return self.socket.settimeout(value)

        def send(self, raw):
            can_id, size, data = wire._CAN_FRAME.unpack(raw)
            motor, opcode, register = struct.unpack("<HBB", data[:4])
            safe = (
                can_id == 0x7FF
                and size == 8
                and data[4:] == bytes(4)
                and motor in interfaces[self.interface]
                and (
                    (opcode == 0x33 and register in (7, 8, 10, 21, 22, 23))
                    or (opcode == 0xCC and register == 0)
                )
            )
            row = {
                "interface": self.interface,
                "direction": "tx",
                "can_id": can_id,
                "payload": data.hex(),
                "monotonic_s": time.monotonic(),
                "allowed": safe,
            }
            report["wire"].append(row)
            assert safe, (
                "read-only live acceptance refuses an unexpected outgoing packet"
            )
            return self.socket.send(raw)

        def recvmsg(self, *args):
            result = self.socket.recvmsg(*args)
            report["wire"].append(
                {
                    "interface": self.interface,
                    "direction": "rx",
                    "frame": result[0].hex(),
                    "monotonic_s": time.monotonic(),
                }
            )
            return result

        def close(self):
            self.socket.close()
            self.closed = True

    monkeypatch.setattr(wire, "_open_socket", GuardedSocket)
    directory = Path(config.get("evidence_directory", ".pytest_cache/live"))
    directory.mkdir(parents=True, exist_ok=True)
    evidence = directory / f"openarm-monitor-{uuid4().hex}.json"
    request.node.user_properties.append(("evidence", str(evidence)))
    previous = {}
    try:
        with site.open(console=False) as session:
            support = session.support()
            report["support"] = support.as_dict()
            for part in selected:
                facts = set(
                    next(
                        row.facts
                        for row in support.rows
                        if row.scope == f"robot:{part}"
                    )
                )
                assert {
                    SupportFact.PART_OBSERVATION,
                    SupportFact.JOINT_POSITION_OBSERVATION,
                    SupportFact.JOINT_VELOCITY_OBSERVATION,
                } <= facts
                assert not facts & {
                    SupportFact.PART_ACTION,
                    SupportFact.JOINT_POSITION_ACTION,
                    SupportFact.SEND_GRANT,
                    SupportFact.HOLD_GRANT,
                    SupportFact.ESTOP_GRANT,
                    SupportFact.FORWARD_KINEMATICS,
                    SupportFact.GRIPPER_MAPPING,
                }
            for _ in range(10):
                observation = session.observe_parts(selected)
                sample = {
                    "session_ns": observation.session_ns,
                    "parts": {},
                    "faults": {
                        name: fault.as_dict()
                        for name, fault in observation.faults.items()
                    },
                }
                report["samples"].append(sample)
                if observation.faults:
                    raise next(iter(observation.faults.values()))
                assert set(observation.parts) == set(selected)
                for part in selected:
                    state = observation.parts[part]
                    assert (
                        state.joint_position.shape == state.joint_velocity.shape == (8,)
                    )
                    assert (
                        np.isfinite(state.joint_position).all()
                        and np.isfinite(state.joint_velocity).all()
                    )
                    assert state.ee_pose_wxyz is None
                    driver = session._managed.arms[part].driver
                    # The background pump can complete an additional batch
                    # before this snapshot. This is latest device-receipt
                    # evidence, not an exact observation/acquisition binding.
                    feedback = driver.last_feedback
                    stamps = tuple(row.received_unix_ns for row in feedback)
                    assert len(stamps) == 8 and all(value > 0 for value in stamps)
                    if part in previous:
                        assert all(
                            now > old
                            for now, old in zip(stamps, previous[part], strict=True)
                        )
                    previous[part] = stamps
                    sample["parts"][part] = {
                        "position_rad": state.joint_position.tolist(),
                        "velocity_rad_s": state.joint_velocity.tolist(),
                        "latest_feedback": [asdict(row) for row in feedback],
                    }
            fault_events = [
                event for event in session.events() if event.kind == "robot.part_fault"
            ]
            assert not fault_events, [event.data for event in fault_events]
        report["closed"] = len(sockets) == 2 and all(sock.closed for sock in sockets)
        assert report["closed"], (
            "ordinary Site teardown did not close both query owners"
        )
        fault_events = [
            event for event in session.events() if event.kind == "robot.part_fault"
        ]
        report["part_fault_events"] = [event.data for event in fault_events]
        assert not fault_events, report["part_fault_events"]
    except BaseException as error:
        report["error"] = RuntimeFault.from_exception(error).as_dict()
        raise
    finally:
        report["socket_close_evidence"] = [
            {"interface": sock.interface, "closed": sock.closed} for sock in sockets
        ]
        evidence.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
