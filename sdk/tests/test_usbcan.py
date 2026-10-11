"""Classic CAN integrity, deadline/fault fencing and exact-device lifecycle.

All USB operations are doubles. No adapter, robot or motor is opened.
"""

import queue
import struct
import threading
import time
from types import SimpleNamespace

import pytest

can = pytest.importorskip("can")
pytest.importorskip("usb")
from waddle_sdk.robots import usbcan
from waddle_sdk.runtime import RuntimeFault

SELECTOR = usbcan.UsbCanSelector(0x1D50, 0x606F, 7, 3, "selected-adapter")


def test_reviewed_vendor_interface_owns_selected_bus_and_preserves_messages(device):
    from importlib import metadata

    from waddle_sdk.robots._i2rt_runtime import TRANSPORT_VERSION
    from waddle_sdk.robots._yam_usb_can import prepare
    from waddle_sdk.robots.usbcan_config import UsbCanConfig

    try:
        version = metadata.version("i2rt")
    except metadata.PackageNotFoundError:
        pytest.skip("Reviewed I2RT runtime is not installed")
    if version != TRANSPORT_VERSION:
        pytest.skip("The installed I2RT runtime is not the explicit USB candidate")
    factory, source = prepare(UsbCanConfig(SELECTOR))
    assert source["version"] == TRANSPORT_VERSION
    interface = factory(
        channel=SELECTOR.channel,
        bitrate=1_000_000,
        name="yam_real",
        use_buffered_reader=False,
    )
    try:
        original = can.Message(arbitration_id=7, data=[1, 2], is_extended_id=False)
        device.frames.put(packet(0x17, b"\x02\x03"))
        pending(interface.bus, 1)
        measured = interface._send_message_get_response(7, 7, list(original.data))
        assert device.writes[0][0] == packet(7, bytes(original.data), echo=0)
        assert measured.arbitration_id == 0x17 and measured.data == b"\x02\x03"
        assert interface.bus.transport_evidence["qualified"] is False
        error = usbcan.usb.core.USBError(
            "Selected adapter removed", error_code=-4, errno=19
        )
        device.write_error = error
        with pytest.raises(usbcan.UsbCanFault) as caught:
            interface._send_message_get_response(7, 7, [1, 2])
        assert caught.value.__cause__ is error
        assert caught.value.code == "hardware.usb_can_io"
        assert (
            len(device.writes) == 1
        )  # Terminal transport faults are not motor retries.
    finally:
        interface.close()
    assert device.claims == device.releases == [(device, 0)]
    assert device.disposes == [device]


class Device:
    serial_number = SELECTOR.serial_number

    def __init__(self):
        self.frames = queue.Queue()
        self.controls, self.writes = [], []
        self.claims, self.releases, self.disposes = [], [], []
        self.control_error = self.write_error = None
        self.features = 1 << 12

    def get_active_configuration(self):
        return {
            (0, 0): [
                SimpleNamespace(bEndpointAddress=ep, bmAttributes=2)
                for ep in (0x81, 0x02)
            ]
        }

    def ctrl_transfer(self, direction, request, value, index, data, timeout):
        assert value == index == 0 and timeout == 100
        self.controls.append((direction, request, data))
        if self.control_error is not None:
            raise self.control_error
        if direction == 0xC1:
            if request == 5:
                return struct.pack("<4B2I", 0, 0, 0, 0, 25, 1)
            if request == 4:
                return struct.pack(
                    "<10I", self.features, 48_000_000, 1, 16, 1, 8, 4, 1, 1024, 1
                )
        return len(data)

    def read(self, endpoint, size, timeout):
        assert endpoint == 0x81 and size == 20 and timeout == 5
        try:
            value = self.frames.get(timeout=0.005)
        except queue.Empty:
            raise usbcan.usb.core.USBTimeoutError("No USB packet", errno=110) from None
        if isinstance(value, Exception):
            raise value
        return value

    def write(self, endpoint, data, timeout):
        assert endpoint == 0x02 and 1 <= timeout <= 100
        if self.write_error is not None:
            raise self.write_error
        self.writes.append((bytes(data), timeout))
        return len(data)


@pytest.fixture
def device(monkeypatch):
    dev = Device()
    backend = object()
    monkeypatch.setattr(usbcan.libusb1, "get_backend", lambda **kwargs: backend)

    def find(**kwargs):
        assert kwargs == {
            "idVendor": SELECTOR.vendor_id,
            "idProduct": SELECTOR.product_id,
            "bus": SELECTOR.bus,
            "address": SELECTOR.address,
            "backend": backend,
        }
        return dev

    monkeypatch.setattr(usbcan.usb.core, "find", find)
    monkeypatch.setattr(
        usbcan.usb.util,
        "claim_interface",
        lambda actual, number: dev.claims.append((actual, number)),
    )
    monkeypatch.setattr(
        usbcan.usb.util,
        "release_interface",
        lambda actual, number: dev.releases.append((actual, number)),
    )
    monkeypatch.setattr(
        usbcan.usb.util, "dispose_resources", lambda actual: dev.disposes.append(actual)
    )
    return dev


def packet(can_id=0x21, data=b"\x01\x02", *, echo=0xFFFFFFFF, flags=0, dlc=None):
    return struct.pack(
        "<IIBBBB8s",
        echo,
        can_id,
        len(data) if dlc is None else dlc,
        0,
        flags,
        0,
        data.ljust(8, b"\0"),
    )


def pending(bus, count):
    deadline = time.monotonic() + 1
    with bus._condition:
        while len(bus._messages) < count and bus._fault is None:
            remaining = deadline - time.monotonic()
            assert remaining > 0, "Receive worker failed to publish"
            bus._condition.wait(remaining)


def test_selected_bus_preserves_protocol_payload_and_independent_cleanup(device):
    bus = usbcan.UsbCanBus(SELECTOR)
    try:
        message = can.Message(
            arbitration_id=7, data=[0xFF] * 7 + [0xFC], is_extended_id=False, check=True
        )
        original = bytes(message.data)
        bus.send(message)
        assert bytes(message.data) == original
        assert device.writes[0][0] == packet(7, original, echo=0)
        assert bus.transport_evidence["berr_reporting"] is True
        assert bus.transport_evidence["qualified"] is False
        device.frames.put(packet(7, original, echo=0))
        device.frames.put(packet(0x17, b"\x02\x03"))
        returned = bus.recv(1)
        assert returned.arbitration_id == 0x17 and returned.is_rx
        assert returned.data == bytearray(b"\x02\x03")
        assert returned.timestamp > 0 and returned.channel == bus.channel_info
        assert bus.recv(0) is None
    finally:
        bus.shutdown()
    bus.shutdown()
    assert device.claims == device.releases == [(device, 0)]
    assert device.disposes == [device]
    assert device.controls[-1] == (0x41, 2, struct.pack("<II", 0, 0))


def test_submillisecond_and_nonblocking_reads_never_become_infinite_usb_reads(device):
    bus = usbcan.UsbCanBus(SELECTOR)
    try:
        started = time.monotonic()
        for _ in range(20):
            assert bus.recv(0) is None
        assert time.monotonic() - started < 0.02
        started = time.monotonic()
        assert bus.recv(0.0002) is None
        assert time.monotonic() - started < 0.05
    finally:
        bus.shutdown()


@pytest.mark.parametrize("read", [True, False])
def test_original_usb_fault_fences_all_io_and_survives_runtime_boundary(device, read):
    bus = usbcan.UsbCanBus(SELECTOR)
    original = usbcan.usb.core.USBError(
        "Selected adapter disconnected", error_code=-4, errno=19
    )
    try:
        if read:
            device.frames.put(original)
            action = lambda: bus.recv(1)
        else:
            device.write_error = original
            action = lambda: bus.send(
                can.Message(arbitration_id=1, data=[1], is_extended_id=False)
            )
        with pytest.raises(usbcan.UsbCanFault) as caught:
            action()
        fault = caught.value
        assert fault.__cause__ is original
        assert fault.context["original_error"]["errno"] == 19
        public = RuntimeFault.from_exception(fault).as_dict()
        assert public["causes"][0]["code"] == "hardware.usb_can_io"
        assert public["causes"][0]["causes"][0]["detail"] == str(original)
        for action in (
            lambda: bus.recv(0),
            lambda: bus.send(can.Message(data=[2])),
            lambda: bus.state,
        ):
            with pytest.raises(usbcan.UsbCanFault) as repeated:
                action()
            assert repeated.value is fault
        assert device.writes == []
    finally:
        bus.shutdown()


@pytest.mark.parametrize("source", ["host", "device", "can", "malformed"])
def test_lost_or_invalid_frames_are_terminal_faults(device, source):
    bus = usbcan.UsbCanBus(SELECTOR, capacity=2)
    try:
        packets = {
            "host": [packet(), packet(), packet()],
            "device": [packet(flags=1)],
            "can": [packet(can_id=0x20000040, data=b"\x00\x01")],
            "malformed": [b"wrong length"],
        }[source]
        for value in packets:
            device.frames.put(value)
        pending(bus, 3)
        with pytest.raises(usbcan.UsbCanFault) as caught:
            bus.recv(0)
        assert (
            caught.value.code
            == {
                "host": "hardware.usb_can_overflow",
                "device": "hardware.usb_can_overflow",
                "can": "hardware.can_bus_error",
                "malformed": "hardware.usb_can_frame",
            }[source]
        )
        with pytest.raises(usbcan.UsbCanFault):
            bus.send(can.Message(data=[1]))
    finally:
        bus.shutdown()


def test_filters_and_remote_frames_preserve_classic_can_semantics(device):
    bus = usbcan.UsbCanBus(SELECTOR, can_filters=[{"can_id": 0x25, "can_mask": 0x7FF}])
    try:
        device.frames.put(packet(0x26))
        device.frames.put(packet(0x40000025, data=b"", dlc=8))
        pending(bus, 2)
        returned = bus.recv(0)
        assert returned.arbitration_id == 0x25 and returned.is_remote_frame
        assert returned.dlc == 8 and returned.data == bytearray()
        assert bus.recv(0) is None
    finally:
        bus.shutdown()


def test_reordered_adapter_never_receives_a_control_request(device):
    device.serial_number = "a-different-adapter"
    with pytest.raises(usbcan.UsbCanFault, match="serial differs"):
        usbcan.UsbCanBus(SELECTOR)
    assert device.controls == device.claims == device.releases == []
    assert device.disposes == [device]


def test_partial_open_preserves_primary_error_and_drains_selected_resources(device):
    original = usbcan.usb.core.USBError("Access denied", error_code=-3, errno=13)
    device.control_error = original
    with pytest.raises(usbcan.UsbCanFault) as caught:
        usbcan.UsbCanBus(SELECTOR)
    assert caught.value.__cause__ is original
    assert (
        caught.value.context["cleanup_error"]["context"]["original_error"]["errno"]
        == 13
    )
    assert device.claims == device.releases == [(device, 0)]
    assert device.disposes == [device]


def test_shutdown_wakes_indefinite_receive_and_drains_before_release(device):
    bus = usbcan.UsbCanBus(SELECTOR)
    outcome = []
    entered = threading.Event()

    def receive():
        entered.set()
        try:
            bus.recv(None)
        except usbcan.UsbCanFault as error:
            outcome.append(error.code)

    caller = threading.Thread(target=receive)
    caller.start()
    assert entered.wait(1)
    bus.shutdown()
    caller.join(1)
    assert not caller.is_alive() and not bus._reader.is_alive()
    assert outcome == ["hardware.usb_can_closed"]
    assert device.disposes == [device]
