"""Explicit transport admission and shared YAM supervision, without motor I/O."""

from __future__ import annotations

import copy
import json
import sys
import threading
import time
import types
from dataclasses import asdict
from pathlib import Path

import pytest
import test_robots_yam as yam_test_vendor
import waddle_sdk
from waddle_sdk.robots import _yam_usb_can, yam
from waddle_sdk.robots._i2rt_runtime import I2rtRuntimeError
from waddle_sdk.robots.usbcan_config import UsbCanConfig, UsbCanSelector

SELECTOR = UsbCanSelector(0x1D50, 0x606F, 7, 3, "selected-yam")
CONFIG = {"selector": asdict(SELECTOR)}
vendor = yam_test_vendor.vendor  # Reuse the existing non-opening vendor fixture.


@pytest.fixture
def transport(monkeypatch, vendor):
    can = pytest.importorskip("can")
    pytest.importorskip("usb")
    from waddle_sdk.robots import usbcan

    events = []

    def verify():
        events.append("verified")
        return {"qualification": "source-only"}

    class Bus:
        def __init__(self, selector, **kwargs):
            self.transport_evidence = {"qualified": False, "provider": "gs_usb"}
            events.append((selector, kwargs))
            self.pending = []

        def send(self, message):
            self.pending.append(
                can.Message(
                    arbitration_id=message.arbitration_id + 16,
                    timestamp=time.time(),
                    data=[message.arbitration_id] + [0] * 7,
                    is_extended_id=False,
                )
            )

        def recv(self, timeout):
            return self.pending.pop(0) if self.pending else None

    class Interface:
        def __init__(self, *, bus_factory, **kwargs):
            self.kwargs = kwargs
            self.receive_mode = types.SimpleNamespace(
                get_receive_id=lambda value: value + 16
            )
            self.bus = bus_factory(
                bustype=kwargs["bustype"],
                channel=kwargs["channel"],
                bitrate=kwargs["bitrate"],
            )

        def _get_frame_id(self, motor_id):
            return motor_id

    module = types.ModuleType("i2rt.motor_drivers.dm_driver")
    module.DMSingleMotorCanInterface = Interface
    monkeypatch.setitem(sys.modules, module.__name__, module)
    monkeypatch.setattr(_yam_usb_can, "verify_transport_runtime", verify)
    monkeypatch.setattr(usbcan, "UsbCanBus", Bus)
    monkeypatch.setattr(
        yam,
        "ensure_socketcan_up",
        lambda *a, **k: pytest.fail("USB must not touch SocketCAN"),
    )
    return events


def test_exact_usb_selection_is_frozen_until_hardware_open(vendor, transport):
    config = copy.deepcopy(CONFIG)
    rig = yam.arm(
        workspace=None, usb_can=config, gripper_limits=(1.7, 0.1), report=lambda _: None
    )
    config["selector"]["serial_number"] = "changed-after-declaration"
    assert not transport and not vendor.calls
    arms = rig.arms()
    try:
        assert transport == ["verified"]
        (call,) = vendor.calls
        assert call["channel"] == SELECTOR.channel
        assert call["zero_gravity_mode"] is False
        factory = call["motor_interface_factory"]
        interface = factory(
            channel=SELECTOR.channel,
            bitrate=1_000_000,
            receive_mode="p16",
            name="yam_real",
            control_mode="MIT",
            use_buffered_reader=False,
        )
        vendor.robots[0].stop_control_loop = lambda: None
        vendor.robots[0].motor_chain = types.SimpleNamespace(
            motor_interface=interface,
            stop_control_loop=lambda: None,
            motor_list=[(value, "reviewed") for value in range(1, 8)],
        )
        assert interface.kwargs == {
            "channel": SELECTOR.channel,
            "bitrate": 1_000_000,
            "receive_mode": "p16",
            "name": "yam_real",
            "control_mode": "MIT",
            "use_buffered_reader": False,
            "bustype": "gs_usb",
        }
        assert transport[1] == (
            SELECTOR,
            {"bitrate": 1_000_000, "libusb_library": None},
        )
        evidence = arms[""].driver.transport_evidence
        assert evidence["qualified"] is False
        assert evidence["runtime_source"] == {"qualification": "source-only"}
        event = threading.Event()
        event.set()
        assert call["control_wait"](event, 0.01, period_s=0.001)
        with pytest.raises(ValueError, match="selected channel"):
            factory(channel="another-device", bitrate=1_000_000)
        assert len(transport) == 2
    finally:
        arms[""].driver.close()


def test_unreviewed_runtime_refuses_before_vendor_or_transport_open(
    vendor, monkeypatch
):
    original = I2rtRuntimeError(
        "Reviewed source changed", file="i2rt/robots/get_robot.py"
    )

    def refuse():
        raise original

    monkeypatch.setattr(_yam_usb_can, "verify_transport_runtime", refuse)
    with pytest.raises(I2rtRuntimeError) as caught:
        yam.LiveDriver(usb_can=CONFIG, gripper_limits=(1.7, 0.1), report=lambda _: None)
    assert caught.value is original
    assert not vendor.calls


@pytest.mark.parametrize(
    "options",
    [
        {"channel": "can0"},
        {"configure_can": True},
        {"can_bitrate": 500_000},
        {"can_bitrate": True},
    ],
)
def test_ambiguous_or_unsupported_usb_choice_refuses_before_open(vendor, options):
    with pytest.raises(ValueError):
        yam.arm(workspace=None, usb_can=CONFIG, report=lambda _: None, **options)
    assert not vendor.calls


@pytest.mark.parametrize(
    "selector",
    [
        SELECTOR,
        UsbCanSelector(0x1D50, 0x606F, 7, 4, "selected-yam"),
        UsbCanSelector(0x1D50, 0x606F, 7, 3, "another-yam"),
    ],
)
def test_parts_cannot_claim_the_same_identity_or_location(vendor, selector):
    with pytest.raises(ValueError, match="share one USB-CAN adapter"):
        yam.bimanual(
            workspace=None,
            left=yam.ArmSite(usb_can=CONFIG),
            right=yam.ArmSite(usb_can=UsbCanConfig(selector)),
            report=lambda _: None,
        )
    assert not vendor.calls


def test_native_config_and_simulation_do_not_import_optional_transports(monkeypatch):
    # A site can retain its live selector while running ordinary simulation.
    import builtins

    original = builtins.__import__

    def guarded(name, *args, **kwargs):
        if name in {"can", "usb"} or name.startswith(("usb.", "i2rt.")):
            pytest.fail(f"Configuration/simulation imported optional driver {name}")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", guarded)
    rig = yam.arm(workspace=None, usb_can=CONFIG, sim=True, report=lambda _: None)
    arm = rig.arms()[""]
    try:
        position, velocity = arm.state()
        assert len(position) == len(velocity) == yam.JOINT_COUNT
    finally:
        arm.driver.close()


def test_site_manifest_consumes_usb_facts_through_the_ordinary_factory(tmp_path):
    example = Path(__file__).resolve().parents[1] / "examples" / "site.yaml"
    manifest = dict(waddle_sdk.load_site(example).describe())
    manifest["parts"]["arm"]["connection"] = {"usb_can": copy.deepcopy(CONFIG)}
    manifest["calibration"]["artifacts"] = "calib/"
    manifest["recording"]["root"] = "recordings/"
    path = tmp_path / "site.json"
    path.write_text(json.dumps(manifest))
    site = waddle_sdk.load_site(path)
    with (
        site.open(console=False, _testing=True) as session,
        session.run(task={"id": "usb-site-facts"}, actor={"id": "test"}) as run,
    ):
        measured = run.observe().parts["arm"].joint_position
        assert len(measured) == yam.JOINT_COUNT


@pytest.mark.parametrize(
    "value",
    [
        {**CONFIG, "index": 0},
        {"selector": {"vendor_id": 0x1D50}},
        {"selector": {**CONFIG["selector"], "vendor_id": float(0x1D50)}},
        {**CONFIG, "libusb_library": "relative/libusb.dylib"},
    ],
)
def test_invalid_site_transport_facts_are_refused_without_optional_imports(value):
    with pytest.raises((ValueError, TypeError)):
        UsbCanConfig.from_value(value)
