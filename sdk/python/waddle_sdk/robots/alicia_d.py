"""Synria Alicia-D manifest adapter with lazy vendor loading."""

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

__all__ = ["AliciaDDriver", "arm"]

_ARM_DOF = 6
_GRIPPER_MAX = 1000.0
# Synriard Alicia_D_v5_6_gripper_50mm.urdf.
_JOINT_LIMITS = (
    (-2.749, 2.749),
    (-2.0, 2.0),
    (-0.5, 3.14159),
    (-2.79, 2.79),
    (-1.57, 1.57),
    (-3.14159, 3.14159),
)


def _vendor_create():
    try:
        return importlib.import_module("alicia_d_sdk").create_robot
    except ModuleNotFoundError as exc:
        if exc.name != "alicia_d_sdk":
            raise
        raise RuntimeError(
            "Alicia-D hardware needs its vendor SDK: pip install 'waddle-sdk[alicia-d]'"
        ) from exc


class AliciaDDriver:
    """One blocking Alicia-D connection behind the synchronous Driver seam."""

    kind = "live"

    def __init__(
        self,
        *,
        port: str,
        version: str,
        gripper_type: str,
        base_link: str,
        end_link: str,
        posture: str,
        speed_deg_s: float,
        gripper_speed_deg_s: float,
        timeout_s: float,
    ) -> None:
        self._lock = threading.RLock()
        self._monitor = posture == "monitor"
        self._speed = float(speed_deg_s)
        self._gripper_speed = float(gripper_speed_deg_s)
        self._timeout = float(timeout_s)
        self._estopped = False
        self._closed = False
        robot = _vendor_create()(
            port=port,
            version=version,
            gripper_type=gripper_type,
            base_link=base_link,
            end_link=end_link,
            auto_connect=False,
        )
        self._robot: Any | None = robot
        try:
            if not robot.connect():
                raise RuntimeError(f"Alicia-D failed to connect on port {port!r}")
            if self._monitor:
                self._torque("off")
        except BaseException:
            self._robot = None
            robot.disconnect()
            raise

    def _require_robot(self) -> Any:
        if self._robot is None or self._closed:
            raise RuntimeError("Alicia-D driver is closed")
        return self._robot

    def _torque(self, state: str) -> None:
        result = self._require_robot().torque_control(state)
        if result is False:
            raise RuntimeError(f"Alicia-D torque_control({state!r}) refused")

    @property
    def estopped(self) -> bool:
        with self._lock:
            return self._estopped

    def read(self) -> tuple[np.ndarray, np.ndarray]:
        with self._lock:
            state = self._require_robot().get_robot_state(
                info_type="joint_gripper",
                timeout=self._timeout,
                cache=True,
            )
            if state is None or len(state.angles) < _ARM_DOF:
                raise RuntimeError("Alicia-D returned incomplete joint state")
            gripper = getattr(state, "gripper", None)
            if gripper is None:
                raise RuntimeError("Alicia-D returned no gripper position")
            position = [float(value) for value in state.angles[:_ARM_DOF]]
            position.append(float(gripper) / _GRIPPER_MAX)
            return np.asarray(position), np.zeros(_ARM_DOF + 1)

    def write(self, target: np.ndarray) -> None:
        values = np.asarray(target, dtype=float).reshape(-1)
        if values.size != _ARM_DOF + 1:
            raise ValueError(
                f"Alicia-D target has {values.size} rows; expected {_ARM_DOF + 1}"
            )
        with self._lock:
            robot = self._require_robot()
            if self._monitor:
                raise RuntimeError(
                    "Alicia-D was opened in monitor posture and is read-only"
                )
            if self._estopped:
                raise RuntimeError("Alicia-D is e-stopped; re-enable it at the site")
            result = robot.set_robot_state(
                target_joints=values[:_ARM_DOF].tolist(),
                joint_format="rad",
                speed_deg_s=self._speed,
                gripper_value=round(float(values[-1]) * _GRIPPER_MAX),
                gripper_speed_deg_s=self._gripper_speed,
                wait_for_completion=False,
                timeout=self._timeout,
            )
            if result is False:
                raise RuntimeError("Alicia-D set_robot_state refused the target")

    def hold(self) -> None:
        with self._lock:
            if self._monitor or self._estopped or self._robot is None:
                return
            position, _velocity = self.read()
            self.write(position)

    def estop(self) -> None:
        with self._lock:
            self._estopped = True
            self._torque("off")

    def re_enable(self) -> None:
        with self._lock:
            if self._monitor:
                raise RuntimeError(
                    "Alicia-D monitor posture cannot be re-enabled for motion"
                )
            self._torque("on")
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
            robot = self._robot
            if robot is None:
                self._closed = True
                return
            try:
                if not self._monitor and not self._estopped:
                    self.hold()
            finally:
                self._robot = None
                self._closed = True
                robot.disconnect()


def _limits(config: PartConfig) -> tuple[tuple[float, float], ...]:
    raw = config.joint_limits
    if not raw:
        return _JOINT_LIMITS
    rows = tuple(raw.values()) if isinstance(raw, Mapping) else tuple(raw)
    limits = tuple((float(row[0]), float(row[1])) for row in rows)
    if len(limits) != _ARM_DOF:
        raise ValueError(f"Alicia-D needs {_ARM_DOF} joint-limit rows")
    return limits


def arm(*, config: PartConfig) -> base.Rig:
    """Build one lazy Alicia-D part from a strict site manifest."""

    if config.workspace_bounds:
        raise ValueError(
            "Alicia-D workspace_bounds require Metal kinematics and cannot be "
            "enforced by this low-level adapter"
        )
    limits = _limits(config) + ((0.0, 1.0),)
    names = tuple(f"joint_{index + 1}" for index in range(_ARM_DOF)) + ("gripper",)
    rate_hz = float(config.options.get("rate_hz", 30.0))
    max_speed = float(config.options.get("max_joint_speed_rad_s", 1.0))
    grip_speed = float(config.options.get("max_gripper_speed_per_s", 1.0))
    if not all(
        math.isfinite(value) and value > 0.0
        for value in (rate_hz, max_speed, grip_speed)
    ):
        raise ValueError("Alicia-D rates and speeds must be finite and positive")
    step_caps = tuple(max_speed / rate_hz for _ in range(_ARM_DOF)) + (
        grip_speed / rate_hz,
    )

    def build_arms() -> dict[str, base.Arm]:
        driver = AliciaDDriver(
            port=str(config.connection.get("port", "")),
            version=str(config.options.get("version", "v5_6")),
            gripper_type=str(config.options.get("gripper_type", "50mm")),
            base_link=str(config.options.get("base_link", "base_link")),
            end_link=str(config.options.get("end_link", "tool0")),
            posture=config.posture,
            speed_deg_s=float(config.options.get("default_speed_deg_s", 60.0)),
            gripper_speed_deg_s=float(
                config.options.get("default_gripper_speed_deg_s", 200.0)
            ),
            timeout_s=float(config.options.get("command_timeout_s", 1.0)),
        )
        return {
            "": base.Arm(
                part="",
                driver=driver,
                joint_names=names,
                joint_limits=limits,
                step_caps=step_caps,
                base_frame=config.base_frame or "",
                arm_dof=_ARM_DOF,
                rate_hz=rate_hz,
            )
        }

    return base.Rig(
        declaration=descriptors.Robot(
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
        ),
        build_arms=build_arms,
        rate_hz=rate_hz,
        posture=config.posture,
    )
