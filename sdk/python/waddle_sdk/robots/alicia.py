"""Synria Alicia-M manifest adapter with lazy vendor loading."""

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

__all__ = ["AliciaDriver", "arm"]

_ARM_DOF = 6
_GRIPPER_MAX = 1000.0
# Synriard Alicia_M_v1_1_follower.urdf.  These are controller-facing limits;
# Metal owns IK and model loading.
_JOINT_LIMITS = (
    (-2.7475, 2.7475),
    (-3.14, 0.0),
    (-3.14, 0.0),
    (-1.57, 1.57),
    (-1.57, 1.57),
    (-2.791, 2.791),
)


def _vendor_module():
    try:
        return importlib.import_module("alicia_m_sdk")
    except ModuleNotFoundError as exc:
        if exc.name != "alicia_m_sdk":
            raise
        raise RuntimeError(
            "Alicia-M hardware needs its vendor SDK: pip install 'waddle-sdk[alicia]'"
        ) from exc


class AliciaDriver:
    """One Alicia-M bus connection satisfying the SDK Driver protocol."""

    kind = "live"

    def __init__(
        self,
        *,
        port: str,
        baudrate: int,
        posture: str,
        control_aim: str,
        control_mode: str,
        speed: int,
        gripper_target_force: float | None,
    ) -> None:
        self._lock = threading.RLock()
        self._monitor = posture == "monitor"
        self._speed = int(speed)
        self._estopped = False
        self._closed = False
        sdk = _vendor_module()
        robot = sdk.create_robot(
            port=port,
            baudrate=int(baudrate),
            control_aim=control_aim,
            control_mode=control_mode,
            auto_connect=True,
        )
        self._robot: Any | None = robot
        try:
            if not robot.is_connected():
                raise RuntimeError(f"Alicia-M failed to connect on port {port!r}")
            if gripper_target_force is not None:
                robot.set_gripper_params({"target_force": float(gripper_target_force)})
            if self._monitor:
                robot.disable_robot()
        except BaseException:
            self._robot = None
            robot.disconnect()
            raise

    def _require_robot(self) -> Any:
        if self._robot is None or self._closed:
            raise RuntimeError("Alicia-M driver is closed")
        return self._robot

    @property
    def estopped(self) -> bool:
        with self._lock:
            return self._estopped

    def read(self) -> tuple[np.ndarray, np.ndarray]:
        with self._lock:
            state = self._require_robot().get_robot_state("all")
            if state is None or len(state.angles) < _ARM_DOF:
                raise RuntimeError("Alicia-M returned incomplete joint state")
            angles = [float(value) for value in state.angles[:_ARM_DOF]]
            gripper = getattr(state, "gripper", None)
            if gripper is None:
                raise RuntimeError("Alicia-M returned no gripper position")
            position = np.asarray(angles + [float(gripper) / _GRIPPER_MAX])
            raw_velocity = getattr(state, "velocities", None)
            velocity = (
                [float(value) for value in raw_velocity[:_ARM_DOF]]
                if raw_velocity is not None and len(raw_velocity) >= _ARM_DOF
                else [0.0] * _ARM_DOF
            )
            return position, np.asarray(velocity + [0.0])

    def write(self, target: np.ndarray) -> None:
        values = np.asarray(target, dtype=float).reshape(-1)
        if values.size != _ARM_DOF + 1:
            raise ValueError(
                f"Alicia-M target has {values.size} rows; expected {_ARM_DOF + 1}"
            )
        with self._lock:
            robot = self._require_robot()
            if self._monitor:
                raise RuntimeError(
                    "Alicia-M was opened in monitor posture and is read-only"
                )
            if self._estopped:
                raise RuntimeError("Alicia-M is e-stopped; re-enable it at the site")
            robot.set_robot_state(
                target_joints=values[:_ARM_DOF].tolist(),
                gripper_value=float(values[-1]) * _GRIPPER_MAX,
                joint_format="rad",
                speed=self._speed,
                wait_for_completion=False,
            )

    def hold(self) -> None:
        with self._lock:
            if self._monitor or self._estopped or self._robot is None:
                return
            position, _velocity = self.read()
            self.write(position)

    def estop(self) -> None:
        with self._lock:
            self._estopped = True
            self._require_robot().disable_robot()

    def re_enable(self) -> None:
        with self._lock:
            if self._monitor:
                raise RuntimeError(
                    "Alicia-M monitor posture cannot be re-enabled for motion"
                )
            self._require_robot().enable_robot()
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
        raise ValueError(f"Alicia-M needs {_ARM_DOF} joint-limit rows")
    return limits


def arm(*, config: PartConfig) -> base.Rig:
    """Build one lazy Alicia-M part from a strict site manifest."""

    if config.workspace_bounds:
        raise ValueError(
            "Alicia-M workspace_bounds require Metal kinematics and cannot be "
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
        raise ValueError("Alicia-M rates and speeds must be finite and positive")
    step_caps = tuple(max_speed / rate_hz for _ in range(_ARM_DOF)) + (
        grip_speed / rate_hz,
    )

    def build_arms() -> dict[str, base.Arm]:
        driver = AliciaDriver(
            port=str(config.connection.get("port", "")),
            baudrate=int(config.connection.get("baudrate", 1_000_000)),
            posture=config.posture,
            control_aim=str(config.options.get("control_aim", "follower")),
            control_mode=str(config.options.get("control_mode", "pv")),
            speed=int(config.options.get("default_joint_speed", 15)),
            gripper_target_force=config.options.get("gripper_target_force"),
        )
        return {
            "": base.Arm(
                part="",
                driver=driver,
                joint_names=names,
                joint_limits=limits,
                step_caps=step_caps,
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
