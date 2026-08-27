"""UFactory xArm 6/7 manifest adapter.

The vendor package is imported only when :func:`arm`'s returned rig opens its
driver.  This module owns device lifecycle and physical controller setup; it
contains no claim, lease, handoff, or transport policy.
"""

from __future__ import annotations

import importlib
import math
import threading
from collections.abc import Mapping, Sequence
from typing import Any

import numpy as np

from .. import descriptors
from . import base
from .site import PartConfig

__all__ = ["XArmDriver", "arm"]

_MODE_POSITION = 0
_STATE_READY = 0
_STATE_STOP = 4
_G2_STROKE_MM = 84.0

_MODEL_LIMITS: dict[str, tuple[tuple[float, float], ...]] = {
    "xarm7": (
        (-2.0 * math.pi, 2.0 * math.pi),
        (-2.059, 2.0944),
        (-2.0 * math.pi, 2.0 * math.pi),
        (-0.19198, 3.927),
        (-2.0 * math.pi, 2.0 * math.pi),
        (-1.69297, 3.14159),
        (-2.0 * math.pi, 2.0 * math.pi),
    ),
    "xarm6": (
        (-2.0 * math.pi, 2.0 * math.pi),
        (-2.059, 2.0944),
        (-3.927, 0.19198),
        (-2.0 * math.pi, 2.0 * math.pi),
        (-1.69297, math.pi),
        (-2.0 * math.pi, 2.0 * math.pi),
    ),
}


def _vendor_api():
    try:
        return importlib.import_module("xarm.wrapper").XArmAPI
    except ModuleNotFoundError as exc:
        if exc.name not in {"xarm", "xarm.wrapper"}:
            raise
        raise RuntimeError(
            "xArm hardware needs the vendor SDK: pip install 'waddle-sdk[xarm]'"
        ) from exc


def _unwrap(value: Any) -> tuple[int, Any]:
    if isinstance(value, tuple):
        if not value:
            return 0, None
        return int(value[0]), value[1] if len(value) > 1 else None
    if isinstance(value, int):
        return value, None
    return 0, value


def _expect(name: str, value: Any) -> Any:
    code, payload = _unwrap(value)
    if code not in (0, None):
        raise RuntimeError(f"xArm {name} returned code {code}")
    return payload


class XArmDriver:
    """Thread-safe SDK driver over one already-authorized xArm connection."""

    kind = "live"

    def __init__(
        self,
        *,
        ip: str,
        dof: int,
        posture: str,
        gripper: bool,
        joint_speed_rad_s: float,
        gripper_speed: int,
        gripper_force: int,
        tcp_offset: Sequence[float] | None,
        linear_speed_limit_factor: float | None,
        self_collision_detection: bool,
        collision_tool_type: int | None,
        reduced_tcp_boundary_mm: Sequence[float] | None,
        reduced_max_tcp_speed_mm_s: float | None,
    ) -> None:
        if not ip.strip():
            raise ValueError("xArm connection.ip must be non-empty")
        self._lock = threading.RLock()
        self._dof = int(dof)
        self._monitor = posture == "monitor"
        self._gripper = bool(gripper)
        self._joint_speed = float(joint_speed_rad_s)
        self._gripper_speed = int(gripper_speed)
        self._gripper_force = int(gripper_force)
        self._estopped = False
        self._closed = False

        api = _vendor_api()
        device = api(ip.strip(), is_radian=True)
        self._device: Any | None = device
        if not getattr(device, "connected", False):
            self._device = None
            self._disconnect(device)
            raise RuntimeError(f"xArm controller at {ip!r} is not connected")
        try:
            if not self._monitor:
                _expect("clean_warn", device.clean_warn())
                _expect("clean_error", device.clean_error())
                _expect("motion_enable", device.motion_enable(enable=True))
                self._configure_controller(
                    tcp_offset=tcp_offset,
                    linear_speed_limit_factor=linear_speed_limit_factor,
                    self_collision_detection=self_collision_detection,
                    collision_tool_type=collision_tool_type,
                    reduced_tcp_boundary_mm=reduced_tcp_boundary_mm,
                    reduced_max_tcp_speed_mm_s=reduced_max_tcp_speed_mm_s,
                )
                self._set_position_mode()
                self._configure_gripper()
        except BaseException:
            self._device = None
            self._disconnect(device)
            raise

    @staticmethod
    def _disconnect(device: Any) -> None:
        disconnect = getattr(device, "disconnect", None)
        if disconnect is not None:
            disconnect()

    def _require_device(self) -> Any:
        if self._device is None or self._closed:
            raise RuntimeError("xArm driver is closed")
        return self._device

    def _set_position_mode(self) -> None:
        device = self._require_device()
        _expect("set_state(stop)", device.set_state(_STATE_STOP))
        _expect("set_mode(position)", device.set_mode(_MODE_POSITION))
        _expect("set_state(ready)", device.set_state(_STATE_READY))

    def _optional_expect(self, method: str, *args: Any, **kwargs: Any) -> None:
        device = self._require_device()
        call = getattr(device, method, None)
        if call is not None:
            _expect(method, call(*args, **kwargs))

    def _configure_controller(
        self,
        *,
        tcp_offset: Sequence[float] | None,
        linear_speed_limit_factor: float | None,
        self_collision_detection: bool,
        collision_tool_type: int | None,
        reduced_tcp_boundary_mm: Sequence[float] | None,
        reduced_max_tcp_speed_mm_s: float | None,
    ) -> None:
        if tcp_offset is not None:
            self._optional_expect("set_tcp_offset", list(tcp_offset), is_radian=True)
        if linear_speed_limit_factor is not None:
            self._optional_expect(
                "set_linear_spd_limit_factor", float(linear_speed_limit_factor)
            )
        if self_collision_detection:
            self._optional_expect("set_self_collision_detection", 1)
        if collision_tool_type is not None:
            self._optional_expect("set_collision_tool_model", collision_tool_type)
        reduced = False
        if reduced_tcp_boundary_mm is not None:
            self._optional_expect(
                "set_reduced_tcp_boundary", list(reduced_tcp_boundary_mm)
            )
            reduced = True
        if reduced_max_tcp_speed_mm_s is not None:
            self._optional_expect(
                "set_reduced_max_tcp_speed", float(reduced_max_tcp_speed_mm_s)
            )
            reduced = True
        if reduced:
            self._optional_expect("set_reduced_mode", 1)

    def _configure_gripper(self) -> None:
        if not self._gripper:
            return
        device = self._require_device()
        for name in ("set_gripper_g2_enable", "set_gripper_enable"):
            call = getattr(device, name, None)
            if call is not None:
                _expect(name, call(True))
                return

    @property
    def estopped(self) -> bool:
        with self._lock:
            return self._estopped

    def read(self) -> tuple[np.ndarray, np.ndarray]:
        with self._lock:
            device = self._require_device()
            angles = _expect("get_servo_angle", device.get_servo_angle(is_radian=True))
            if angles is None or len(angles) < self._dof:
                raise RuntimeError(
                    f"xArm returned {0 if angles is None else len(angles)} joints; "
                    f"expected {self._dof}"
                )
            position = [float(value) for value in angles[: self._dof]]
            if self._gripper:
                getter = getattr(device, "get_gripper_g2_position", None)
                if getter is None:
                    raise RuntimeError("xArm G2 gripper has no position getter")
                opening_mm = _expect("get_gripper_g2_position", getter())
                if opening_mm is None:
                    raise RuntimeError("xArm G2 gripper returned no position")
                position.append(float(opening_mm) / _G2_STROKE_MM)
            return np.asarray(position, dtype=float), np.zeros(len(position))

    def write(self, target: np.ndarray) -> None:
        values = np.asarray(target, dtype=float).reshape(-1)
        expected = self._dof + int(self._gripper)
        if values.size != expected:
            raise ValueError(f"xArm target has {values.size} rows; expected {expected}")
        with self._lock:
            device = self._require_device()
            if self._monitor:
                raise RuntimeError(
                    "xArm was opened in monitor posture and is read-only"
                )
            if self._estopped:
                raise RuntimeError("xArm is e-stopped; re-enable it at the site")
            _expect(
                "set_servo_angle",
                device.set_servo_angle(
                    angle=values[: self._dof].tolist(),
                    is_radian=True,
                    speed=self._joint_speed,
                    wait=False,
                ),
            )
            if self._gripper:
                _expect(
                    "set_gripper_g2_position",
                    device.set_gripper_g2_position(
                        float(values[-1]) * _G2_STROKE_MM,
                        speed=self._gripper_speed,
                        force=self._gripper_force,
                        wait=False,
                    ),
                )

    def hold(self) -> None:
        with self._lock:
            if self._monitor or self._estopped or self._device is None:
                return
            position, _velocity = self.read()
            _expect(
                "set_servo_angle(hold)",
                self._device.set_servo_angle(
                    angle=position[: self._dof].tolist(),
                    is_radian=True,
                    speed=self._joint_speed,
                    wait=False,
                ),
            )

    def estop(self) -> None:
        with self._lock:
            self._estopped = True
            device = self._require_device()
            emergency_stop = getattr(device, "emergency_stop", None)
            if emergency_stop is None:
                _expect("set_state(stop)", device.set_state(_STATE_STOP))
            else:
                _expect("emergency_stop", emergency_stop())

    def re_enable(self) -> None:
        with self._lock:
            if self._monitor:
                raise RuntimeError(
                    "xArm monitor posture cannot be re-enabled for motion"
                )
            device = self._require_device()
            _expect("clean_warn", device.clean_warn())
            _expect("clean_error", device.clean_error())
            _expect("motion_enable", device.motion_enable(enable=True))
            self._set_position_mode()
            self._configure_gripper()
            self._estopped = False

    def step(self, dt: float) -> None:
        del dt

    def home(self, values: Sequence[float]) -> bool:
        del values
        return False

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            device = self._device
            if device is None:
                self._closed = True
                return
            try:
                if not self._monitor and not self._estopped:
                    self.hold()
            finally:
                self._device = None
                self._closed = True
                self._disconnect(device)


def _model(config: PartConfig) -> tuple[str, tuple[tuple[float, float], ...]]:
    model = str(config.options.get("model", "xarm7")).lower()
    try:
        defaults = _MODEL_LIMITS[model]
    except KeyError as exc:
        raise ValueError(f"xArm model must be one of {sorted(_MODEL_LIMITS)}") from exc
    raw = config.joint_limits
    if not raw:
        return model, defaults
    rows = tuple(raw.values()) if isinstance(raw, Mapping) else tuple(raw)
    limits = tuple((float(row[0]), float(row[1])) for row in rows)
    if len(limits) != len(defaults):
        raise ValueError(
            f"{model} needs {len(defaults)} joint-limit rows, got {len(limits)}"
        )
    return model, limits


def arm(*, config: PartConfig) -> base.Rig:
    """Build one lazy xArm part from a strict site manifest."""

    _model_name, arm_limits = _model(config)
    dof = len(arm_limits)
    gripper = str(config.options.get("gripper", "g2")).lower()
    if gripper not in {"g2", "none"}:
        raise ValueError("xArm gripper must be 'g2' or 'none'")
    has_gripper = gripper == "g2"
    names = tuple(f"joint_{index + 1}" for index in range(dof))
    limits = arm_limits + (((0.0, 1.0),) if has_gripper else ())
    if has_gripper:
        names += ("gripper",)
    rate_hz = float(config.options.get("rate_hz", 50.0))
    joint_speed = float(config.options.get("max_joint_speed_rad_s", 1.0))
    gripper_speed_per_s = float(config.options.get("max_gripper_speed_per_s", 1.0))
    if not all(
        math.isfinite(value) and value > 0.0
        for value in (rate_hz, joint_speed, gripper_speed_per_s)
    ):
        raise ValueError("xArm rates and speeds must be finite and positive")
    step_caps = tuple(joint_speed / rate_hz for _ in range(dof))
    if has_gripper:
        step_caps += (gripper_speed_per_s / rate_hz,)

    workspace = config.workspace_bounds
    if workspace:
        raise ValueError(
            "xArm workspace_bounds require a local FK model; this adapter refuses "
            "to pretend controller FK is a pre-dispatch hard-safety check"
        )

    connection = config.connection
    options = config.options

    def build_arms() -> dict[str, base.Arm]:
        driver = XArmDriver(
            ip=str(connection.get("ip", "")),
            dof=dof,
            posture=config.posture,
            gripper=has_gripper,
            joint_speed_rad_s=joint_speed,
            gripper_speed=int(options.get("gripper_speed", 100)),
            gripper_force=int(options.get("gripper_force", 50)),
            tcp_offset=options.get("tcp_offset"),
            linear_speed_limit_factor=options.get("linear_speed_limit_factor", 5.0),
            self_collision_detection=bool(
                options.get("controller_self_collision", True)
            ),
            collision_tool_type=options.get("collision_tool_type", 1),
            reduced_tcp_boundary_mm=options.get("reduced_tcp_boundary_mm"),
            reduced_max_tcp_speed_mm_s=options.get("reduced_max_tcp_speed_mm_s"),
        )
        return {
            "": base.Arm(
                part="",
                driver=driver,
                joint_names=names,
                joint_limits=limits,
                step_caps=step_caps,
                base_frame=config.base_frame or "",
                arm_dof=dof,
                rate_hz=rate_hz,
            )
        }

    declaration = descriptors.Robot(
        name=config.name,
        action_space=descriptors.JointSpace(
            joints=tuple(
                descriptors.Joint(
                    name=name,
                    min_position=lower,
                    max_position=upper,
                    max_velocity=cap * rate_hz,
                )
                for name, (lower, upper), cap in zip(
                    names, limits, step_caps, strict=True
                )
            ),
            rate_hz=rate_hz,
        ),
    )
    return base.Rig(
        declaration=declaration,
        build_arms=build_arms,
        rate_hz=rate_hz,
        posture=config.posture,
    )
