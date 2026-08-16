"""Manifest-native MuJoCo joint-target adapter.

The adapter loads a customer-owned MJCF only when the returned rig opens.  It
is deliberately a joint target sink: Metal owns IK and planning, while this
module owns simulator lifecycle, state, stepping, and hard-safety geometry.
"""

from __future__ import annotations

import importlib
import math
import threading
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

import numpy as np

from .. import descriptors
from . import base
from .site import PartConfig

__all__ = ["MujocoDriver", "arm"]


def _mujoco_module():
    try:
        return importlib.import_module("mujoco")
    except ModuleNotFoundError as exc:
        if exc.name != "mujoco":
            raise
        raise RuntimeError(
            "MuJoCo simulation needs its optional package: "
            "pip install 'waddle-sdk[mujoco]'"
        ) from exc


def _model_path(config: PartConfig) -> Path:
    value = config.connection.get("model")
    if not isinstance(value, str) or not value or "\x00" in value or "\\" in value:
        raise ValueError("MuJoCo connection.model must be a portable relative path")
    relative = Path(value)
    if relative.is_absolute() or any(part == ".." for part in relative.parts):
        raise ValueError("MuJoCo connection.model must stay beneath the site root")
    root = config.site_root.resolve()
    resolved = (root / relative).resolve(strict=False)
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise ValueError("MuJoCo connection.model escapes the site root") from exc
    if not resolved.is_file():
        raise ValueError(f"MuJoCo model does not exist: {resolved}")
    return resolved


class MujocoDriver:
    """One deterministic MuJoCo model behind the SDK Driver protocol."""

    kind = "sim"

    def __init__(
        self,
        *,
        model_path: Path,
        joint_names: Sequence[str],
        joint_limits: Sequence[Sequence[float]],
        actuator_names: Sequence[str],
        home: Sequence[float],
        tool_site: str | None,
        collision_bodies: Sequence[Mapping[str, object]],
    ) -> None:
        self._mj = _mujoco_module()
        self._lock = threading.RLock()
        self._model = self._mj.MjModel.from_xml_path(str(model_path))
        self._data = self._mj.MjData(self._model)
        self._scratch = self._mj.MjData(self._model)
        self._closed = False
        self._estopped = False
        self._joint_ids = tuple(
            self._id(self._mj.mjtObj.mjOBJ_JOINT, name) for name in joint_names
        )
        self._joint_qpos = tuple(
            int(self._model.jnt_qposadr[identifier]) for identifier in self._joint_ids
        )
        self._joint_dof = tuple(
            int(self._model.jnt_dofadr[identifier]) for identifier in self._joint_ids
        )
        self._actuators = tuple(
            self._id(self._mj.mjtObj.mjOBJ_ACTUATOR, name) for name in actuator_names
        )
        self._tool_site = (
            None
            if tool_site is None
            else self._id(self._mj.mjtObj.mjOBJ_SITE, tool_site)
        )
        self._collision_bodies = self._load_collision_bodies(collision_bodies)
        self._validate_model(joint_names, joint_limits, actuator_names, home)
        self.home(home)

    def _id(self, kind: Any, name: str) -> int:
        identifier = int(self._mj.mj_name2id(self._model, kind, name))
        if identifier < 0:
            raise ValueError(f"MuJoCo model has no object named {name!r}")
        return identifier

    def _load_collision_bodies(
        self, rows: Sequence[Mapping[str, object]]
    ) -> tuple[tuple[str, int, float], ...]:
        bodies: list[tuple[str, int, float]] = []
        for row in rows:
            if not isinstance(row, Mapping):
                raise TypeError("MuJoCo collision_bodies rows must be mappings")
            try:
                name = str(row["name"])
                body = str(row["body"])
                radius = float(row["radius_m"])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(
                    "MuJoCo collision bodies need name, body, and radius_m"
                ) from exc
            if not name or not body or not math.isfinite(radius) or radius <= 0.0:
                raise ValueError(
                    "MuJoCo collision bodies need non-empty names/body and "
                    "positive radii"
                )
            bodies.append((name, self._id(self._mj.mjtObj.mjOBJ_BODY, body), radius))
        return tuple(bodies)

    def _validate_model(
        self,
        joint_names: Sequence[str],
        joint_limits: Sequence[Sequence[float]],
        actuator_names: Sequence[str],
        home: Sequence[float],
    ) -> None:
        widths = {
            len(joint_names),
            len(joint_limits),
            len(actuator_names),
            len(home),
        }
        if len(widths) != 1 or not joint_names:
            raise ValueError(
                "MuJoCo joint_names, joint_limits, actuator_names, and home "
                "must have equal non-zero width"
            )
        if len(set(joint_names)) != len(joint_names):
            raise ValueError("MuJoCo joint_names must be unique")
        if len(set(actuator_names)) != len(actuator_names):
            raise ValueError("MuJoCo actuator_names must be unique")
        hinge = int(self._mj.mjtJoint.mjJNT_HINGE)
        slide = int(self._mj.mjtJoint.mjJNT_SLIDE)
        for name, identifier, declared in zip(
            joint_names, self._joint_ids, joint_limits, strict=True
        ):
            if int(self._model.jnt_type[identifier]) not in (hinge, slide):
                raise ValueError(
                    f"MuJoCo joint {name!r} is not a scalar hinge or slide joint"
                )
            lower, upper = (float(value) for value in declared)
            if bool(self._model.jnt_limited[identifier]):
                model_lower, model_upper = (
                    float(value) for value in self._model.jnt_range[identifier]
                )
                if lower < model_lower or upper > model_upper:
                    raise ValueError(
                        f"MuJoCo declared limits for {name!r} [{lower}, {upper}] "
                        f"widen model limits [{model_lower}, {model_upper}]"
                    )
        timestep = float(self._model.opt.timestep)
        if not math.isfinite(timestep) or timestep <= 0.0:
            raise ValueError("MuJoCo model timestep must be finite and positive")

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("MuJoCo driver is closed")

    def _set_scratch(self, target: Sequence[float]) -> Any:
        values = np.asarray(target, dtype=float).reshape(-1)
        if values.size != len(self._joint_qpos):
            raise ValueError(
                f"MuJoCo target has {values.size} rows; expected {len(self._joint_qpos)}"
            )
        self._scratch.qpos[:] = self._data.qpos
        for address, value in zip(self._joint_qpos, values, strict=True):
            self._scratch.qpos[address] = value
        self._mj.mj_forward(self._model, self._scratch)
        return self._scratch

    @property
    def estopped(self) -> bool:
        with self._lock:
            return self._estopped

    def read(self) -> tuple[np.ndarray, np.ndarray]:
        with self._lock:
            self._require_open()
            return (
                np.asarray([self._data.qpos[index] for index in self._joint_qpos]),
                np.asarray([self._data.qvel[index] for index in self._joint_dof]),
            )

    def write(self, target: np.ndarray) -> None:
        values = np.asarray(target, dtype=float).reshape(-1)
        if values.size != len(self._actuators):
            raise ValueError(
                f"MuJoCo target has {values.size} rows; expected {len(self._actuators)}"
            )
        with self._lock:
            self._require_open()
            if self._estopped:
                raise RuntimeError("MuJoCo is e-stopped; re-enable it at the site")
            for actuator, value in zip(self._actuators, values, strict=True):
                self._data.ctrl[actuator] = value

    def hold(self) -> None:
        with self._lock:
            if self._closed:
                return
            position, _velocity = self.read()
            for actuator, value in zip(self._actuators, position, strict=True):
                self._data.ctrl[actuator] = value
            for address in self._joint_dof:
                self._data.qvel[address] = 0.0

    def estop(self) -> None:
        with self._lock:
            self._estopped = True
            self.hold()

    def re_enable(self) -> None:
        with self._lock:
            self._require_open()
            self.hold()
            self._estopped = False

    def step(self, dt: float) -> None:
        with self._lock:
            self._require_open()
            if self._estopped:
                return
            duration = float(dt)
            if not math.isfinite(duration) or duration < 0.0:
                raise ValueError("MuJoCo step duration must be finite and non-negative")
            timestep = float(self._model.opt.timestep)
            count = math.ceil(duration / timestep)
            for _ in range(count):
                self._mj.mj_step(self._model, self._data)

    def home(self, values: Sequence[float]) -> bool:
        target = np.asarray(values, dtype=float).reshape(-1)
        with self._lock:
            self._require_open()
            if self._estopped:
                return False
            if target.size != len(self._joint_qpos):
                raise ValueError("MuJoCo home must have one value per joint")
            for qpos, dof, actuator, value in zip(
                self._joint_qpos,
                self._joint_dof,
                self._actuators,
                target,
                strict=True,
            ):
                self._data.qpos[qpos] = value
                self._data.qvel[dof] = 0.0
                self._data.ctrl[actuator] = value
            self._mj.mj_forward(self._model, self._data)
            return True

    def forward_kinematics(
        self, target: Sequence[float]
    ) -> tuple[np.ndarray, np.ndarray]:
        with self._lock:
            if self._tool_site is None:
                raise RuntimeError("MuJoCo adapter declared no tool_site")
            scratch = self._set_scratch(target)
            return (
                np.asarray(scratch.site_xpos[self._tool_site], dtype=float).copy(),
                np.asarray(scratch.site_xmat[self._tool_site], dtype=float)
                .reshape(3, 3)
                .copy(),
            )

    def collision_spheres(
        self, target: Sequence[float]
    ) -> tuple[base.CollisionSphere, ...]:
        with self._lock:
            scratch = self._set_scratch(target)
            return tuple(
                base.CollisionSphere(
                    name=name,
                    center_m=np.asarray(scratch.xpos[body], dtype=float).copy(),
                    radius_m=radius,
                )
                for name, body, radius in self._collision_bodies
            )

    def close(self) -> None:
        with self._lock:
            self._closed = True


def _limits_and_names(
    config: PartConfig,
) -> tuple[tuple[str, ...], tuple[tuple[float, float], ...]]:
    raw = config.joint_limits
    names_option = config.options.get("joint_names", ())
    if isinstance(raw, Mapping) and raw:
        names = tuple(str(name) for name in raw)
        rows = tuple(raw.values())
    elif isinstance(raw, Sequence) and not isinstance(raw, (str, bytes)) and raw:
        names = tuple(str(name) for name in names_option)
        rows = tuple(raw)
    else:
        raise ValueError(
            "MuJoCo requires explicit joint_limits so its declaration stays "
            "lazy and does not import/load the model before Site.open()"
        )
    limits = tuple((float(row[0]), float(row[1])) for row in rows)
    if not names or len(names) != len(limits):
        raise ValueError("MuJoCo joint_names and joint_limits must have equal width")
    return names, limits


def arm(*, config: PartConfig) -> base.Rig:
    """Build one lazy MuJoCo part from an MJCF path and explicit mappings."""

    model_path = _model_path(config)
    names, limits = _limits_and_names(config)
    actuator_names = tuple(
        str(value) for value in config.options.get("actuator_names", names)
    )
    home = tuple(
        float(value)
        for value in config.options.get(
            "home", tuple((lower + upper) / 2.0 for lower, upper in limits)
        )
    )
    rate_hz = float(config.options.get("rate_hz", 100.0))
    max_speed = float(config.options.get("max_joint_speed_per_s", 1.0))
    if not all(math.isfinite(value) and value > 0.0 for value in (rate_hz, max_speed)):
        raise ValueError("MuJoCo rate and speed must be finite and positive")
    step_caps = tuple(max_speed / rate_hz for _ in names)
    tool_site_raw = config.options.get("tool_site")
    tool_site = None if tool_site_raw is None else str(tool_site_raw)
    collision_bodies = tuple(config.options.get("collision_bodies", ()))
    collision_frame = str(config.options.get("collision_frame", "world"))
    workspace = config.workspace_bounds
    if workspace and tool_site is None:
        raise ValueError("MuJoCo workspace_bounds require options.tool_site")
    workspace_box = (
        None if not workspace else (tuple(workspace["min"]), tuple(workspace["max"]))
    )

    def build_arms() -> dict[str, base.Arm]:
        driver = MujocoDriver(
            model_path=model_path,
            joint_names=names,
            joint_limits=limits,
            actuator_names=actuator_names,
            home=home,
            tool_site=tool_site,
            collision_bodies=collision_bodies,
        )
        return {
            "": base.Arm(
                part="",
                driver=driver,
                joint_names=names,
                joint_limits=limits,
                step_caps=step_caps,
                base_frame=collision_frame,
                workspace=workspace_box,
                fk=driver.forward_kinematics if tool_site is not None else None,
                collision_spheres=(
                    driver.collision_spheres if collision_bodies else None
                ),
                collision_frame=collision_frame if collision_bodies else "",
                home_values=home,
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
                        max_velocity=max_speed,
                    )
                    for name, (lower, upper) in zip(names, limits, strict=True)
                ),
                rate_hz=rate_hz,
            ),
        ),
        build_arms=build_arms,
        rate_hz=rate_hz,
        posture=config.posture,
    )
