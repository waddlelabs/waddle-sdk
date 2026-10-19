"""OpenArm fake-wire acceptance: measured data, bounded refusal, zero controls."""

from __future__ import annotations

import json
import socket
import struct
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from types import SimpleNamespace
from uuid import uuid4

import numpy as np
import pytest
import yaml
from test_site_api import _write_site
from waddle_sdk import load_site
from waddle_sdk.robots import _openarm_transport as wire
from waddle_sdk.robots import openarm
from waddle_sdk.robots.site import PartConfig
from waddle_sdk.runtime import (
    FaultCode,
    JointPositionCommand,
    RuntimeFault,
    SupportFact,
)


def configuration(**changes):
    return replace(
        PartConfig(
            name="observed_arm",
            posture="monitor",
            connection={
                "interface": "can_test",
                "usb_serial": "sdk-test-openarm",
                "motor_ids": list(range(1, 9)),
                "feedback_ids": list(range(17, 25)),
            },
            joint_limits=[[-3.0, 3.0]] * 8,
            workspace_bounds={},
            envelope={},
            options={"timeout_s": 0.05, "rate_hz": 10},
        ),
        **changes,
    )


def frame(can_id, data, *, stamp=None, size=8):
    stamp = time.time_ns() if stamp is None else stamp
    sec, nanos = divmod(stamp, 1_000_000_000)
    return (
        wire._CAN_FRAME.pack(can_id, size, data),
        [(socket.SOL_SOCKET, wire._SO_TIMESTAMPNS, wire._TIMESPEC.pack(sec, nanos))],
        0,
        ("can_test",),
    )


class FakeSocket:
    def __init__(self):
        self.sent = []
        self.queue = deque()
        self.closed = False
        self.mutate = lambda motor, opcode, register, response: [response]
        self.generation = 0
        self.after_send = None

    def settimeout(self, timeout):
        self.timeout = timeout

    def send(self, raw):
        self.sent.append(raw)
        can_id, size, data = wire._CAN_FRAME.unpack(raw)
        assert can_id == 0x7FF and size == 8
        motor, opcode, register = struct.unpack("<HBB", data[:4])
        assert data[4:] == bytes(4)
        # The complete lifecycle must contain only these two read requests.
        assert opcode in (0x33, 0xCC)
        if opcode == 0x33:
            value = {7: motor + 16, 8: motor, 10: 1, 21: 12.5, 22: 30.0, 23: 10.0}[
                register
            ]
            payload = data[:4] + struct.pack(
                "<f" if register in (21, 22, 23) else "<I", value
            )
        else:
            assert register == 0
            self.generation += 1
            # Different values prove that read() does not replay its startup cache.
            q = 32000 + self.generation
            dq = 2050 + motor
            payload = bytes(
                [motor, q >> 8, q & 255, dq >> 4, (dq & 15) << 4, 0, 24, 25]
            )
        self.queue.extend(
            self.mutate(motor, opcode, register, frame(motor + 16, payload))
        )
        if self.after_send is not None:
            self.after_send(motor, opcode, register)
        return len(raw)

    def recvmsg(self, size, ancillary_size):
        if self.queue:
            return self.queue.popleft()
        raise TimeoutError

    def close(self):
        self.closed = True


@pytest.fixture
def sockets(monkeypatch):
    opened = []

    def open_socket(interface):
        sock = FakeSocket()
        opened.append(sock)
        return sock

    monkeypatch.setattr(wire, "_usb_serial", lambda interface: "sdk-test-openarm")
    monkeypatch.setattr(
        wire.socketcan,
        "read_socketcan_state",
        lambda interface: wire.socketcan.SocketCanState(interface, True, 1_000_000),
    )
    monkeypatch.setattr(wire, "_open_socket", open_socket)
    yield opened
    for sock in opened:
        assert sock.closed, "test leaked a query socket"


def opened_driver(sockets, config=None):
    return openarm.arm(config=config or configuration()).arms()[""].driver


def test_factory_is_lazy_and_every_lifecycle_operation_is_read_only(sockets):
    rig = openarm.arm(config=configuration())
    assert sockets == []
    assert (
        tuple(joint.name for joint in rig.robot().action_space.joints)
        == openarm.JOINT_NAMES
    )
    assert all(joint.max_velocity is None for joint in rig.robot().action_space.joints)
    arm = rig.arms()[""]
    driver = arm.driver
    try:
        initial = driver.last_feedback
        position, velocity = driver.read()
        assert len(position) == len(velocity) == 8
        assert driver.last_feedback[0].position_rad != initial[0].position_rad
        assert np.any(velocity != 0)
        assert all(
            row.received_unix_ns > old.received_unix_ns
            for row, old in zip(driver.last_feedback, initial, strict=True)
        )
        assert driver.encoding_spans == ((12.5, 30.0),) * 8
        assert arm.joint_limits == ((-3.0, 3.0),) * 8  # Not +-12.5 encoding span.
        assert arm.ee_pose(position) is None
        sent = len(sockets[0].sent)
        for operation in (
            lambda: driver.write(position),
            driver.hold,
            driver.estop,
            driver.re_enable,
        ):
            with pytest.raises(RuntimeFault) as error:
                operation()
            assert error.value.code == FaultCode.UNSUPPORTED
        assert len(sockets[0].sent) == sent
        assert driver.home(position) is False
        assert not driver.estopped
        assert (
            driver.actuation_supported
            is driver.estop_supported
            is driver.close_releases_torque
            is False
        )
    finally:
        driver.close()
        driver.close()
    with pytest.raises(RuntimeFault) as error:
        driver.read()
    assert error.value.code == FaultCode.NOT_OPEN


@pytest.mark.parametrize("posture", ["supervised", "active", "sim"])
def test_unsupported_postures_refuse_before_hardware(posture, sockets):
    with pytest.raises(ValueError, match="only monitor"):
        openarm.arm(config=configuration(posture=posture))
    assert sockets == []


@pytest.mark.parametrize(
    "changes",
    [
        {"joint_limits": {}},
        {"joint_limits": [[-12.5, float("inf")]] * 8},
        {"joint_limits": [[True, 3]] * 8},
        {"joint_limits": [[3, -3]] * 8},
        {"workspace_bounds": {"min": [-1] * 3, "max": [1] * 3}},
        {"options": {"timeout_s": 0}},
        {"options": {"timeout_s": float("nan")}},
        {"options": {"rate_hz": True}},
        {"options": {"enable_motors": True}},
        {"connection": {"interface": "can_test"}},
    ],
)
def test_invalid_declaration_is_nonopening(changes, sockets):
    with pytest.raises(ValueError):
        openarm.arm(config=configuration(**changes))
    assert sockets == []


def test_swapped_usb_identity_fails_before_socket_or_link_change(sockets, monkeypatch):
    monkeypatch.setattr(wire, "_usb_serial", lambda interface: "different-adapter")
    monkeypatch.setattr(
        wire.socketcan,
        "read_socketcan_state",
        lambda _: pytest.fail("inspected mismatched bus"),
    )
    with pytest.raises(RuntimeFault) as error:
        opened_driver(sockets)
    assert error.value.context["reason"] == "openarm.identity_mismatch"
    assert sockets == []


def test_duplicate_bus_owner_is_refused_and_close_releases_it(sockets):
    first = opened_driver(sockets)
    try:
        with pytest.raises(RuntimeFault) as error:
            opened_driver(sockets)
        assert error.value.code == FaultCode.BUSY
        assert len(sockets) == 1
    finally:
        first.close()
    second = opened_driver(sockets)
    second.close()
    assert len(sockets) == 2


@pytest.mark.parametrize(
    "corruption,reason",
    [
        ("old_timestamp", "openarm.feedback_timeout"),
        ("wrong_id", "openarm.feedback_timeout"),
        ("missing_motor", "openarm.feedback_timeout"),
        ("short_payload", "openarm.malformed_feedback"),
        ("bad_payload_id", "openarm.motor_id_mismatch"),
        ("overheat", "openarm.mos_overheat"),
        ("unknown_status", "openarm.unknown_status"),
        ("can_error", "openarm.can_error"),
        ("no_timestamp", "openarm.missing_timestamp"),
    ],
)
def test_invalid_fresh_batch_fails_without_cached_or_partial_measurements(
    sockets, corruption, reason
):
    driver = opened_driver(sockets)
    before = driver.last_feedback
    sock = sockets[0]

    def mutate(motor, opcode, register, response):
        if motor != 3:
            return [response]
        raw, _anc, flags, address = response
        can_id, _size, data = wire._CAN_FRAME.unpack(raw)
        if corruption == "old_timestamp":
            return [frame(can_id, data, stamp=1)]
        if corruption == "wrong_id":
            return [frame(90, data)]
        if corruption == "missing_motor":
            return []
        if corruption == "short_payload":
            return [frame(can_id, data, size=7)]
        if corruption == "bad_payload_id":
            return [frame(can_id, bytes([4]) + data[1:])]
        if corruption == "overheat":
            return [frame(can_id, bytes([3 | 0xB0]) + data[1:])]
        if corruption == "unknown_status":
            return [frame(can_id, bytes([3 | 0x20]) + data[1:])]
        if corruption == "can_error":
            return [frame(wire._CAN_ERR_FLAG | 0x40, data)]
        return [(raw, [], flags, address)]

    sock.mutate = mutate
    try:
        with pytest.raises(RuntimeFault) as error:
            driver.read()
        assert error.value.context["reason"] == reason
        assert error.value.context["interface"] == "can_test"
        assert driver.last_feedback is before
        sent = len(sock.sent)
        # No retry can consume a delayed reply from the failed transaction.
        with pytest.raises(RuntimeFault) as repeated:
            driver.read()
        assert repeated.value is error.value
        assert len(sock.sent) == sent
    finally:
        driver.close()


def test_cached_frames_are_drained_and_unrelated_frames_do_not_hide_fresh_data(sockets):
    driver = opened_driver(sockets)
    sock = sockets[0]
    try:
        sock.queue.extend(frame(motor + 16, bytes(8), stamp=1) for motor in range(1, 9))
        sock.mutate = lambda motor, opcode, register, response: [
            frame(500, bytes(8)),
            response,
        ]
        position, velocity = driver.read()
        assert len(position) == 8 and np.any(velocity)
    finally:
        driver.close()


@pytest.mark.parametrize(
    "register,value,reason",
    [
        (7, 55, "openarm.motor_identity_mismatch"),
        (21, float("nan"), "openarm.invalid_parameter"),
        (22, 0.0, "openarm.invalid_encoding_spans"),
    ],
)
def test_bad_parameter_fails_startup_and_closes_transport(
    sockets, monkeypatch, register, value, reason
):
    def open_socket(interface):
        sock = FakeSocket()
        sockets.append(sock)

        def mutate(motor, opcode, rid, response):
            if motor == 2 and opcode == 0x33 and rid == register:
                data = struct.pack("<HBB", motor, opcode, rid) + struct.pack(
                    "<f" if rid in (21, 22) else "<I", value
                )
                return [frame(motor + 16, data)]
            return [response]

        sock.mutate = mutate
        return sock

    monkeypatch.setattr(wire, "_open_socket", open_socket)
    with pytest.raises(RuntimeFault) as error:
        opened_driver(sockets)
    assert error.value.context["reason"] == reason
    assert sockets[0].closed


def test_parallel_reads_serialize_one_complete_motor_batch(sockets, monkeypatch):
    monkeypatch.setattr(
        wire, "time", SimpleNamespace(monotonic=lambda: 0.0, time_ns=time.time_ns)
    )
    driver = opened_driver(sockets)
    entered, release = threading.Event(), threading.Event()
    sock = sockets[0]
    first = True

    def after_send(motor, opcode, register):
        nonlocal first
        if first:
            first = False
            entered.set()
            assert release.wait(2)

    sock.after_send = after_send
    sent = len(sock.sent)
    try:
        with ThreadPoolExecutor(max_workers=2) as pool:
            one = pool.submit(driver.read)
            assert entered.wait(2)
            two = pool.submit(driver.read)
            assert len(sock.sent) == sent + 1
            release.set()
            one.result(timeout=2)
            two.result(timeout=2)
        assert [
            struct.unpack("<H", wire._CAN_FRAME.unpack(raw)[2][:2])[0]
            for raw in sock.sent[sent:]
        ] == list(range(1, 9)) * 2
    finally:
        release.set()
        driver.close()


def test_ordinary_monitor_site_observes_and_refuses_actions_without_parking(
    sockets, tmp_path
):
    path = _write_site(tmp_path)
    document = yaml.safe_load(path.read_text())
    document["metadata"]["id"] = f"openarm-monitor-{uuid4()}"
    config = configuration()
    document["parts"] = {
        "observed_arm": {
            "driver": "waddle_sdk.robots.openarm:arm",
            "posture": "monitor",
            "connection": config.connection,
            "joint_limits": config.joint_limits,
            "options": config.options,
        }
    }
    document["cameras"] = {}
    path.write_text(yaml.safe_dump(document))
    with load_site(path).open() as sdk:
        observation = sdk.observe_parts()
        assert len(observation.parts["observed_arm"].joint_position) == 8
        row = next(
            row for row in sdk.support().rows if row.scope == "robot:observed_arm"
        )
        assert SupportFact.PART_OBSERVATION in row.facts
        for fact in (
            SupportFact.JOINT_POSITION_ACTION,
            SupportFact.PART_ACTION,
            SupportFact.FORWARD_KINEMATICS,
            SupportFact.GRIPPER_MAPPING,
        ):
            assert fact not in row.facts
        with sdk.run(task="observe", actor="test") as run:
            result = run.step_parts({"observed_arm": JointPositionCommand([0.0] * 8)})[
                "observed_arm"
            ]
            assert not result.dispatched
        for stop in (sdk.hold, sdk.estop):
            with pytest.raises(RuntimeFault) as error:
                stop("not available")
            assert error.value.code == FaultCode.UNSUPPORTED
    assert sockets[0].closed


def test_socket_setup_uses_linux_timestamp_and_error_options_without_transmitting(
    monkeypatch,
):
    calls = []

    class Socket:
        def setsockopt(self, *args):
            calls.append(("option", args))

        def bind(self, value):
            calls.append(("bind", value))

        def close(self):
            calls.append(("close",))

    sock = Socket()
    monkeypatch.setattr(wire.socket, "socket", lambda *args: sock)
    assert wire._open_socket("can_test") is sock
    assert calls == [
        ("option", (socket.SOL_SOCKET, wire._SO_TIMESTAMPNS, 1)),
        ("option", (socket.SOL_CAN_RAW, 2, struct.pack("=I", 0x1FFFFFFF))),
        ("bind", ("can_test",)),
    ]
    sock.close()


def test_parameter_api_cannot_encode_writes_and_mismatched_register_times_out(sockets):
    driver = opened_driver(sockets)
    sock = sockets[0]
    try:
        before = len(sock.sent)
        for register in (0x55, 0xCC, True, -1):
            with pytest.raises(ValueError):
                driver._bus.parameter(wire.MotorAddress(1, 17), register)
        assert len(sock.sent) == before

        def wrong_register(motor, opcode, register, response):
            data = struct.pack("<HBBI", motor, 0x33, 8, motor)
            return [frame(motor + 16, data)]

        sock.mutate = wrong_register
        with pytest.raises(RuntimeFault) as error:
            driver._bus.parameter(wire.MotorAddress(1, 17), 7)
        assert error.value.context["register"] == 7
        assert error.value.context["motor_ids"] == [1]
        assert error.value.context["reason"] == "openarm.feedback_timeout"
    finally:
        driver.close()


def test_identity_change_during_bind_closes_socket_and_releases_owner(
    sockets, monkeypatch
):
    identities = iter(("sdk-test-openarm", "swapped-device"))
    monkeypatch.setattr(wire, "_usb_serial", lambda _: next(identities))
    with pytest.raises(RuntimeFault) as error:
        opened_driver(sockets)
    assert error.value.context["reason"] == "openarm.identity_mismatch"
    assert sockets[0].closed
    monkeypatch.setattr(wire, "_usb_serial", lambda _: "sdk-test-openarm")
    driver = opened_driver(sockets)
    driver.close()


def test_bus_flood_is_bounded_by_frame_budget(sockets, monkeypatch):
    monkeypatch.setattr(
        wire, "time", SimpleNamespace(monotonic=lambda: 0.0, time_ns=time.time_ns)
    )
    driver = opened_driver(sockets)
    sock = sockets[0]
    try:
        sock.mutate = lambda motor, opcode, register, response: (
            [frame(501, bytes(8))] * 2048
        )
        with pytest.raises(RuntimeFault) as error:
            driver.read()
        assert error.value.context["reason"] == "openarm.feedback_timeout"
        assert len(sock.queue) == 8 * 2048 - 1024
    finally:
        driver.close()


def test_sysfs_usb_binding_uses_device_ancestor_serial(tmp_path, monkeypatch):
    net = tmp_path / "net"
    device = tmp_path / "usb-device"
    interface = device / "interface"
    interface.mkdir(parents=True)
    (device / "serial").write_text("selected-adapter\n")
    (net / "can_test").mkdir(parents=True)
    (net / "can_test" / "device").symlink_to(interface, target_is_directory=True)
    monkeypatch.setattr(wire, "Path", lambda _: net)
    assert wire._usb_serial("can_test") == "selected-adapter"


def test_live_monitor_harness_retains_actual_wire_and_close_evidence(
    sockets, tmp_path, monkeypatch
):
    from live.test_05_openarm_monitor import (
        test_two_openarm_parts_supply_fresh_feedback_without_actuation as trial,
    )

    path = _write_site(tmp_path)
    document = yaml.safe_load(path.read_text())
    document["metadata"]["id"] = f"openarm-live-harness-{uuid4()}"
    document["parts"] = {}
    for name in ("bus_a", "bus_b"):
        config = configuration()
        connection = {
            **config.connection,
            "interface": name,
            "usb_serial": f"test-{name}",
        }
        document["parts"][name] = {
            "driver": "waddle_sdk.robots.openarm:arm",
            "posture": "monitor",
            "connection": connection,
            "joint_limits": config.joint_limits,
            "options": config.options,
        }
    document["cameras"] = {}
    path.write_text(yaml.safe_dump(document))
    monkeypatch.setattr(wire, "_usb_serial", lambda interface: f"test-{interface}")
    request = SimpleNamespace(
        config=SimpleNamespace(
            getoption=lambda name: True,
            live_bench={
                "site": str(path),
                "parts": ["bus_a", "bus_b"],
                "evidence_directory": str(tmp_path),
            },
        ),
        node=SimpleNamespace(user_properties=[]),
    )
    trial(request, monkeypatch)
    report = json.loads(next(tmp_path.glob("openarm-monitor-*.json")).read_text())
    assert report["closed"] is True and len(report["samples"]) == 10
    assert report["selected_parts"] == ["bus_a", "bus_b"]
    assert len(sockets) == 2 and all(sock.closed for sock in sockets)
    assert all(row["allowed"] for row in report["wire"] if row["direction"] == "tx")
    assert report["part_fault_events"] == []
