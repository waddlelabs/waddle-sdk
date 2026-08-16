"""Fake-vendor lifecycle tests for Alicia-M and Alicia-D adapters."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from waddle_sdk.robots import alicia, alicia_d
from waddle_sdk.robots.site import PartConfig


def _config(*, posture="supervised", options=None) -> PartConfig:
    return PartConfig(
        name="arm",
        posture=posture,
        connection={"port": "/dev/fake"},
        joint_limits={},
        workspace_bounds={},
        envelope={"static_keepouts": [], "self_collision": {}},
        options=options or {},
        site_root=Path("."),
    )


class FakeAliciaM:
    def __init__(self, *, connected=True) -> None:
        self.connected = connected
        self.calls: list[tuple[str, object]] = []
        self.angles = [0.0] * 6
        self.velocities = [0.1] * 6
        self.gripper = 1000.0
        self.enable_error = False

    def is_connected(self):
        return self.connected

    def get_robot_state(self, kind):
        self.calls.append(("get_robot_state", kind))
        return SimpleNamespace(
            angles=list(self.angles),
            velocities=list(self.velocities),
            gripper=self.gripper,
        )

    def set_robot_state(self, **kwargs):
        self.calls.append(("set_robot_state", kwargs))
        self.angles = list(kwargs["target_joints"])
        self.gripper = kwargs["gripper_value"]

    def set_gripper_params(self, value):
        self.calls.append(("set_gripper_params", value))

    def disable_robot(self):
        self.calls.append(("disable_robot", None))

    def enable_robot(self):
        self.calls.append(("enable_robot", None))
        if self.enable_error:
            raise RuntimeError("enable failed")

    def disconnect(self):
        self.calls.append(("disconnect", None))


class FakeAliciaD:
    def __init__(self, *, connects=True) -> None:
        self.connects = connects
        self.calls: list[tuple[str, object]] = []
        self.angles = [0.0] * 6
        self.gripper = 1000
        self.torque_on = True

    def connect(self):
        self.calls.append(("connect", None))
        return self.connects

    def get_robot_state(self, **kwargs):
        self.calls.append(("get_robot_state", kwargs))
        return SimpleNamespace(angles=list(self.angles), gripper=self.gripper)

    def set_robot_state(self, **kwargs):
        self.calls.append(("set_robot_state", kwargs))
        self.angles = list(kwargs["target_joints"])
        self.gripper = kwargs["gripper_value"]
        return True

    def torque_control(self, value):
        self.calls.append(("torque_control", value))
        return not (value == "on" and not self.torque_on)

    def disconnect(self):
        self.calls.append(("disconnect", None))


def test_alicia_m_factory_is_lazy_and_converts_gripper(monkeypatch):
    devices = []

    def create_robot(**kwargs):
        devices.append(FakeAliciaM())
        devices[-1].calls.append(("create", kwargs))
        return devices[-1]

    monkeypatch.setattr(
        alicia,
        "_vendor_module",
        lambda: SimpleNamespace(create_robot=create_robot),
    )
    rig = alicia.arm(config=_config(options={"gripper_target_force": 12.0}))
    assert devices == []
    assert len(rig.robot().action_space.joints) == 7

    driver = rig.arms()[""].driver
    device = devices[0]
    position, velocity = driver.read()
    assert position.tolist() == [0.0] * 6 + [1.0]
    assert velocity.tolist() == [0.1] * 6 + [0.0]
    driver.write(np.asarray([0.2] * 6 + [0.25]))
    call = next(value for name, value in device.calls if name == "set_robot_state")
    assert call["joint_format"] == "rad"
    assert call["gripper_value"] == 250.0
    assert not call["wait_for_completion"]
    driver.close()
    assert [name for name, _value in device.calls].count("disconnect") == 1


def test_alicia_m_monitor_and_estop_are_locally_latched(monkeypatch):
    devices = []
    monkeypatch.setattr(
        alicia,
        "_vendor_module",
        lambda: SimpleNamespace(
            create_robot=lambda **_kwargs: devices.append(FakeAliciaM()) or devices[-1]
        ),
    )
    monitor = alicia.arm(config=_config(posture="monitor")).arms()[""].driver
    assert ("disable_robot", None) in devices[-1].calls
    with pytest.raises(RuntimeError, match="read-only"):
        monitor.write(np.zeros(7))
    monitor.close()

    driver = alicia.arm(config=_config()).arms()[""].driver
    device = devices[-1]
    driver.estop()
    assert driver.estopped
    device.enable_error = True
    with pytest.raises(RuntimeError, match="enable failed"):
        driver.re_enable()
    assert driver.estopped
    device.enable_error = False
    driver.re_enable()
    assert not driver.estopped
    driver.close()


def test_alicia_m_failed_open_disconnects(monkeypatch):
    device = FakeAliciaM(connected=False)
    monkeypatch.setattr(
        alicia,
        "_vendor_module",
        lambda: SimpleNamespace(create_robot=lambda **_kwargs: device),
    )
    with pytest.raises(RuntimeError, match="failed to connect"):
        alicia.arm(config=_config()).arms()
    assert ("disconnect", None) in device.calls


def test_alicia_d_factory_is_lazy_and_uses_nonblocking_combined_write(monkeypatch):
    devices = []

    def create_robot(**kwargs):
        devices.append(FakeAliciaD())
        devices[-1].calls.append(("create", kwargs))
        return devices[-1]

    monkeypatch.setattr(alicia_d, "_vendor_create", lambda: create_robot)
    rig = alicia_d.arm(config=_config())
    assert devices == []
    driver = rig.arms()[""].driver
    position, _velocity = driver.read()
    assert position.tolist() == [0.0] * 6 + [1.0]
    driver.write(np.asarray([0.3] * 6 + [0.4]))
    call = next(value for name, value in devices[0].calls if name == "set_robot_state")
    assert call["gripper_value"] == 400
    assert call["joint_format"] == "rad"
    assert not call["wait_for_completion"]
    driver.close()


def test_alicia_d_monitor_and_failed_reenable_remain_safe(monkeypatch):
    devices = []
    monkeypatch.setattr(
        alicia_d,
        "_vendor_create",
        lambda: lambda **_kwargs: devices.append(FakeAliciaD()) or devices[-1],
    )
    monitor = alicia_d.arm(config=_config(posture="monitor")).arms()[""].driver
    assert ("torque_control", "off") in devices[-1].calls
    with pytest.raises(RuntimeError, match="read-only"):
        monitor.write(np.zeros(7))
    monitor.close()

    driver = alicia_d.arm(config=_config()).arms()[""].driver
    device = devices[-1]
    driver.estop()
    device.torque_on = False
    with pytest.raises(RuntimeError, match="refused"):
        driver.re_enable()
    assert driver.estopped
    device.torque_on = True
    driver.re_enable()
    assert not driver.estopped
    driver.close()


def test_alicia_d_failed_open_disconnects(monkeypatch):
    device = FakeAliciaD(connects=False)
    monkeypatch.setattr(alicia_d, "_vendor_create", lambda: lambda **_kwargs: device)
    with pytest.raises(RuntimeError, match="failed to connect"):
        alicia_d.arm(config=_config()).arms()
    assert ("disconnect", None) in device.calls
