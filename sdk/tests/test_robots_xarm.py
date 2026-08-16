"""Fake-vendor contract tests for the UFactory xArm adapter."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from waddle_sdk.robots import xarm
from waddle_sdk.robots.site import PartConfig


class FakeXArm:
    def __init__(self, ip: str, *, is_radian: bool) -> None:
        self.ip = ip
        self.is_radian = is_radian
        self.connected = True
        self.calls: list[tuple[str, object]] = []
        self.q = [0.0] * 7
        self.gripper_mm = 84.0
        self.fail: dict[str, int] = {}

    def _call(self, name: str, value: object = None) -> int:
        self.calls.append((name, value))
        return self.fail.get(name, 0)

    def clean_warn(self):
        return self._call("clean_warn")

    def clean_error(self):
        return self._call("clean_error")

    def motion_enable(self, *, enable: bool):
        return self._call("motion_enable", enable)

    def set_state(self, state: int):
        return self._call("set_state", state)

    def set_mode(self, mode: int):
        return self._call("set_mode", mode)

    def set_tcp_offset(self, value, *, is_radian: bool):
        return self._call("set_tcp_offset", (tuple(value), is_radian))

    def set_linear_spd_limit_factor(self, value: float):
        return self._call("set_linear_spd_limit_factor", value)

    def set_self_collision_detection(self, value: int):
        return self._call("set_self_collision_detection", value)

    def set_collision_tool_model(self, value: int):
        return self._call("set_collision_tool_model", value)

    def set_reduced_tcp_boundary(self, value):
        return self._call("set_reduced_tcp_boundary", tuple(value))

    def set_reduced_max_tcp_speed(self, value: float):
        return self._call("set_reduced_max_tcp_speed", value)

    def set_reduced_mode(self, value: int):
        return self._call("set_reduced_mode", value)

    def set_gripper_g2_enable(self, enabled: bool):
        return self._call("set_gripper_g2_enable", enabled)

    def get_servo_angle(self, *, is_radian: bool):
        self.calls.append(("get_servo_angle", is_radian))
        return self.fail.get("get_servo_angle", 0), list(self.q)

    def get_gripper_g2_position(self):
        self.calls.append(("get_gripper_g2_position", None))
        return self.fail.get("get_gripper_g2_position", 0), self.gripper_mm

    def set_servo_angle(self, *, angle, is_radian: bool, speed: float, wait: bool):
        self.q[: len(angle)] = angle
        return self._call("set_servo_angle", (tuple(angle), is_radian, speed, wait))

    def set_gripper_g2_position(
        self, value: float, *, speed: int, force: int, wait: bool
    ):
        self.gripper_mm = value
        return self._call("set_gripper_g2_position", (value, speed, force, wait))

    def emergency_stop(self):
        return self._call("emergency_stop")

    def disconnect(self):
        return self._call("disconnect")


def _config(*, posture: str = "supervised", options=None) -> PartConfig:
    return PartConfig(
        name="assembly-arm",
        posture=posture,
        connection={"ip": "192.0.2.4"},
        joint_limits={},
        workspace_bounds={},
        envelope={"static_keepouts": [], "self_collision": {}},
        options=options or {"model": "xarm7"},
        site_root=Path("."),
    )


def _fake_vendor(monkeypatch):
    devices: list[FakeXArm] = []

    def api(ip: str, *, is_radian: bool):
        device = FakeXArm(ip, is_radian=is_radian)
        devices.append(device)
        return device

    monkeypatch.setattr(xarm, "_vendor_api", lambda: api)
    return devices


def test_factory_is_lazy_and_supervised_lifecycle_matches_vendor(monkeypatch):
    devices = _fake_vendor(monkeypatch)
    rig = xarm.arm(config=_config())
    assert devices == []
    assert len(rig.robot().action_space.joints) == 8

    arms = rig.arms()
    device = devices[0]
    driver = arms[""].driver
    assert device.ip == "192.0.2.4"
    assert device.is_radian
    assert device.calls[:3] == [
        ("clean_warn", None),
        ("clean_error", None),
        ("motion_enable", True),
    ]
    assert ("set_mode", 0) in device.calls

    position, velocity = driver.read()
    assert position.tolist() == [0.0] * 7 + [1.0]
    assert velocity.tolist() == [0.0] * 8
    driver.write(np.asarray([0.1] * 7 + [0.5]))
    assert device.q == [0.1] * 7
    assert device.gripper_mm == 42.0

    driver.close()
    driver.close()
    assert [name for name, _value in device.calls].count("disconnect") == 1


def test_monitor_posture_never_enables_or_writes(monkeypatch):
    devices = _fake_vendor(monkeypatch)
    driver = xarm.arm(config=_config(posture="monitor")).arms()[""].driver
    device = devices[0]
    assert "motion_enable" not in [name for name, _value in device.calls]
    with pytest.raises(RuntimeError, match="read-only"):
        driver.write(np.zeros(8))
    driver.hold()
    assert "set_servo_angle" not in [name for name, _value in device.calls]
    driver.close()


def test_estop_latches_until_successful_re_enable(monkeypatch):
    devices = _fake_vendor(monkeypatch)
    driver = xarm.arm(config=_config()).arms()[""].driver
    device = devices[0]
    driver.estop()
    assert driver.estopped
    with pytest.raises(RuntimeError, match="e-stopped"):
        driver.write(np.zeros(8))

    device.fail["motion_enable"] = 9
    with pytest.raises(RuntimeError, match="code 9"):
        driver.re_enable()
    assert driver.estopped

    device.fail.clear()
    driver.re_enable()
    assert not driver.estopped
    driver.close()


def test_failed_open_disconnects_half_open_device(monkeypatch):
    devices = _fake_vendor(monkeypatch)

    original = xarm._vendor_api()

    def api(ip: str, *, is_radian: bool):
        device = original(ip, is_radian=is_radian)
        device.fail["motion_enable"] = 3
        return device

    monkeypatch.setattr(xarm, "_vendor_api", lambda: api)
    with pytest.raises(RuntimeError, match="motion_enable returned code 3"):
        xarm.arm(config=_config()).arms()
    assert ("disconnect", None) in devices[0].calls


def test_workspace_and_unknown_models_fail_before_hardware(monkeypatch):
    devices = _fake_vendor(monkeypatch)
    config = _config()
    config = PartConfig(
        **{
            **config.__dict__,
            "workspace_bounds": {"min": [-1, -1, -1], "max": [1, 1, 1]},
        }
    )
    with pytest.raises(ValueError, match="local FK model"):
        xarm.arm(config=config)
    with pytest.raises(ValueError, match="model must be"):
        xarm.arm(config=_config(options={"model": "xarm42"}))
    assert devices == []
