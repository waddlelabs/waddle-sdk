"""Manifest-native MuJoCo simulation backend and legacy arm adapter.

The modular backend loads one customer-owned MJCF or URDF world for every part
and camera that references it.  The legacy :func:`arm` factory remains a
single-part convenience.  Both are deliberately joint target sinks: Application code
owns IK and planning, while this module owns simulator lifecycle, state,
stepping, rendering, and hard-safety geometry.
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
from ..cameras import CameraFrame
from ..cameras.site import CameraConfig
from ..simulation import WorldConfig
from . import base
from .site import PartConfig

__all__ = [
    "MujocoBackend",
    "MujocoCameraDriver",
    "MujocoDriver",
    "arm",
    "backend",
]


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


def _model_path(config: PartConfig | WorldConfig) -> Path:
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


class MujocoBackend:
    """One shared, lazily opened MuJoCo physics and rendering world."""

    def __init__(self, *, model_path: Path) -> None:
        self._model_path = model_path
        self._lock = threading.RLock()
        self._mj = None
        self._model = None
        self._data = None
        self._scratch = None
        self._opened = False
        self._closed = False

    def _require_open(self) -> None:
        if not self._opened or self._closed:
            raise RuntimeError("MuJoCo world is not open")

    @property
    def mj(self):
        self._require_open()
        return self._mj

    @property
    def model(self):
        self._require_open()
        return self._model

    @property
    def data(self):
        self._require_open()
        return self._data

    @property
    def scratch(self):
        self._require_open()
        return self._scratch

    def open(self) -> None:
        with self._lock:
            if self._opened or self._closed:
                raise RuntimeError("MuJoCo world instances may be opened only once")
            mj = _mujoco_module()
            model = mj.MjModel.from_xml_path(str(self._model_path))
            data = mj.MjData(model)
            scratch = mj.MjData(model)
            self._mj = mj
            self._model = model
            self._data = data
            self._scratch = scratch
            self._opened = True
            mj.mj_forward(model, data)

    def part(self, *, config: PartConfig) -> base.Rig:
        """Return one declaration-only part backed by this shared world."""

        return _arm_rig(config, world=self)

    def camera(self, *, config: CameraConfig) -> MujocoCameraDriver:
        """Open one renderer over the shared world."""

        self._require_open()
        return MujocoCameraDriver(world=self, config=config)

    def step(self, dt: float) -> None:
        duration = float(dt)
        if not math.isfinite(duration) or duration < 0.0:
            raise ValueError("MuJoCo step duration must be finite and non-negative")
        with self._lock:
            self._require_open()
            timestep = float(self.model.opt.timestep)
            count = math.ceil(duration / timestep)
            for _ in range(count):
                self.mj.mj_step(self.model, self.data)

    def reset(self) -> bool:
        with self._lock:
            self._require_open()
            self.mj.mj_resetData(self.model, self.data)
            self.mj.mj_forward(self.model, self.data)
            return True

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._opened = False
            self._scratch = None
            self._data = None
            self._model = None
            self._mj = None


class MujocoDriver:
    """One deterministic MuJoCo model behind the SDK Driver protocol."""

    kind = "sim"

    def __init__(
        self,
        *,
        model_path: Path | None = None,
        world: MujocoBackend | None = None,
        joint_names: Sequence[str],
        joint_limits: Sequence[Sequence[float]],
        actuator_names: Sequence[str],
        home: Sequence[float],
        tool_site: str | None,
        collision_bodies: Sequence[Mapping[str, object]],
        model_joint_names: Sequence[str] | None = None,
    ) -> None:
        if (model_path is None) == (world is None):
            raise TypeError("MujocoDriver needs exactly one of model_path or world")
        self._owns_world = world is None
        self._world = world or MujocoBackend(model_path=model_path)  # type: ignore[arg-type]
        if self._owns_world:
            self._world.open()
        self._mj = self._world.mj
        self._lock = self._world._lock
        self._model = self._world.model
        self._data = self._world.data
        self._scratch = self._world.scratch
        self._closed = False
        self._estopped = False
        internal_joint_names = (
            tuple(joint_names)
            if model_joint_names is None
            else tuple(model_joint_names)
        )
        try:
            if len(internal_joint_names) != len(joint_names):
                raise ValueError(
                    "MuJoCo model_joint_names and public joint_names must have equal width"
                )
            self._joint_ids = tuple(
                self._id(self._mj.mjtObj.mjOBJ_JOINT, name)
                for name in internal_joint_names
            )
            self._joint_qpos = tuple(
                int(self._model.jnt_qposadr[identifier])
                for identifier in self._joint_ids
            )
            self._joint_dof = tuple(
                int(self._model.jnt_dofadr[identifier])
                for identifier in self._joint_ids
            )
            self._actuators = tuple(
                self._id(self._mj.mjtObj.mjOBJ_ACTUATOR, name)
                for name in actuator_names
            )
            self._tool_site = (
                None
                if tool_site is None
                else self._id(self._mj.mjtObj.mjOBJ_SITE, tool_site)
            )
            self._collision_bodies = self._load_collision_bodies(collision_bodies)
            self._validate_model(joint_names, joint_limits, actuator_names, home)
            self.home(home)
        except BaseException:
            if self._owns_world:
                self._world.close()
            raise

    def _id(self, kind: Any, name: str) -> int:
        identifier = int(self._mj.mj_name2id(self._model, kind, name))
        if identifier < 0:
            raise ValueError(f"MuJoCo model has no object named {name!r}")
        return identifier

    def _load_collision_bodies(
        self, rows: Sequence[Mapping[str, object]]
    ) -> tuple[tuple[str, int, float, np.ndarray], ...]:
        bodies: list[tuple[str, int, float, np.ndarray]] = []
        for row in rows:
            if not isinstance(row, Mapping):
                raise TypeError("MuJoCo collision_bodies rows must be mappings")
            try:
                name = str(row["name"])
                body = str(row["body"])
                radius = float(row["radius_m"])
                offset = np.asarray(
                    row.get("center_m", (0.0, 0.0, 0.0)), dtype=float
                )
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(
                    "MuJoCo collision bodies need name, body, and radius_m"
                ) from exc
            if (
                not name
                or not body
                or not math.isfinite(radius)
                or radius <= 0.0
                or offset.shape != (3,)
                or not np.all(np.isfinite(offset))
            ):
                raise ValueError(
                    "MuJoCo collision bodies need non-empty names/body, positive "
                    "radii, and finite three-element center_m offsets"
                )
            bodies.append(
                (
                    name,
                    self._id(self._mj.mjtObj.mjOBJ_BODY, body),
                    radius,
                    offset,
                )
            )
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
        self._world._require_open()

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
        self._require_open()
        duration = float(dt)
        if not math.isfinite(duration) or duration < 0.0:
            raise ValueError("MuJoCo step duration must be finite and non-negative")
        if self._owns_world and not self._estopped:
            self._world.step(duration)

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
                    center_m=(
                        np.asarray(scratch.xpos[body], dtype=float)
                        + np.asarray(scratch.xmat[body], dtype=float).reshape(3, 3)
                        @ offset
                    ),
                    radius_m=radius,
                )
                for name, body, radius, offset in self._collision_bodies
            )

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            if self._owns_world:
                self._world.close()


class MujocoCameraDriver:
    """One RGB-D renderer over an opened :class:`MujocoBackend`."""

    def __init__(self, *, world: MujocoBackend, config: CameraConfig) -> None:
        self._world = world
        self._closing = threading.Event()
        self._width = int(config.stream["width"])
        self._height = int(config.stream["height"])
        camera = config.connection.get("camera")
        if not isinstance(camera, str) or not camera:
            raise ValueError("MuJoCo camera connection.camera must name an MJCF camera")
        self._camera = camera
        identifier = int(
            world.mj.mj_name2id(world.model, world.mj.mjtObj.mjOBJ_CAMERA, camera)
        )
        if identifier < 0:
            raise ValueError(f"MuJoCo model has no camera named {camera!r}")
        self._camera_id = identifier
        depth = config.options.get("depth", True)
        if not isinstance(depth, bool):
            raise TypeError("MuJoCo camera options.depth must be boolean")
        self._depth = depth
        configured_scale = (
            None
            if config.intrinsics is None
            else config.intrinsics.get("depth_scale_mm")
        )
        option_scale = config.options.get("depth_scale_mm")
        if (
            configured_scale is not None
            and option_scale is not None
            and float(configured_scale) != float(option_scale)
        ):
            raise ValueError(
                "MuJoCo camera options.depth_scale_mm conflicts with declared "
                "intrinsics"
            )
        self._depth_scale_mm = float(
            configured_scale
            if configured_scale is not None
            else 1.0 if option_scale is None else option_scale
        )
        if not math.isfinite(self._depth_scale_mm) or self._depth_scale_mm <= 0.0:
            raise ValueError("MuJoCo camera depth scale must be finite and positive")
        # OpenGL/EGL contexts are thread-affine. Site camera drivers are built
        # on the lifecycle thread and captured by a CameraPump thread, so the
        # renderer must be created lazily by the first capture rather than here.
        self._renderer = None
        self._renderer_thread: int | None = None

    def _thread_renderer(self):
        current = threading.get_ident()
        if self._renderer is None:
            self._renderer = self._world.mj.Renderer(
                self._world.model,
                height=self._height,
                width=self._width,
            )
            self._renderer_thread = current
        elif self._renderer_thread != current:
            raise RuntimeError("MuJoCo camera capture must stay on one render thread")
        return self._renderer

    def intrinsics(self) -> descriptors.Intrinsics:
        with self._world._lock:
            self._world._require_open()
            fovy = float(self._world.model.cam_fovy[self._camera_id])
            if not math.isfinite(fovy) or fovy <= 0.0 or fovy >= 180.0:
                raise RuntimeError("MuJoCo camera has an invalid vertical field of view")
            focal = (self._height / 2.0) / math.tan(math.radians(fovy) / 2.0)
            return descriptors.Intrinsics(
                fx=focal,
                fy=focal,
                cx=(self._width - 1.0) / 2.0,
                cy=(self._height - 1.0) / 2.0,
                depth_scale_mm=self._depth_scale_mm,
            )

    def capture(self) -> CameraFrame:
        if self._closing.is_set():
            raise RuntimeError("MuJoCo camera is closed")
        with self._world._lock:
            self._world._require_open()
            renderer = self._thread_renderer()
            renderer.disable_depth_rendering()
            renderer.update_scene(self._world.data, camera=self._camera)
            rgb = np.array(
                renderer.render(), dtype=np.uint8, order="C", copy=True
            )
            depth = None
            if self._depth:
                renderer.enable_depth_rendering()
                try:
                    metres = np.asarray(renderer.render(), dtype=float)
                finally:
                    renderer.disable_depth_rendering()
                valid = np.isfinite(metres) & (metres > 0.0)
                scaled = np.zeros(metres.shape, dtype=np.uint16)
                raw = np.rint(metres[valid] * (1000.0 / self._depth_scale_mm))
                in_range = raw <= np.iinfo(np.uint16).max
                valid_values = np.zeros(raw.shape, dtype=np.uint16)
                valid_values[in_range] = raw[in_range].astype(np.uint16)
                scaled[valid] = valid_values
                depth = scaled
            return CameraFrame(rgb=rgb, depth=depth)

    def close(self) -> None:
        self._closing.set()
        with self._world._lock:
            # CameraPump.close() first signals from the lifecycle thread so a
            # blocking physical driver can wake. The pump's finally block then
            # calls close again on the capture thread; only that owner may tear
            # down this renderer context.
            if (
                self._renderer is not None
                and self._renderer_thread == threading.get_ident()
            ):
                close = getattr(self._renderer, "close", None)
                if callable(close):
                    close()
                self._renderer = None
                self._renderer_thread = None


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


def _arm_rig(
    config: PartConfig, *, world: MujocoBackend | None = None
) -> base.Rig:
    """Build one lazy MuJoCo part from explicit manifest mappings."""

    model_path = None if world is not None else _model_path(config)
    names, limits = _limits_and_names(config)
    actuator_names = tuple(
        str(value) for value in config.options.get("actuator_names", names)
    )
    model_joint_names = tuple(
        str(value) for value in config.options.get("model_joint_names", names)
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
    legacy_frame = str(config.options.get("collision_frame", "world"))
    if (
        config.base_frame
        and "collision_frame" in config.options
        and config.base_frame != legacy_frame
    ):
        raise ValueError("MuJoCo base_frame conflicts with legacy options.collision_frame")
    collision_frame = config.base_frame or legacy_frame
    workspace = config.workspace_bounds
    if workspace and tool_site is None:
        raise ValueError("MuJoCo workspace_bounds require options.tool_site")
    workspace_box = (
        None if not workspace else (tuple(workspace["min"]), tuple(workspace["max"]))
    )

    def build_arms() -> dict[str, base.Arm]:
        driver = MujocoDriver(
            model_path=model_path,
            world=world,
            joint_names=names,
            joint_limits=limits,
            actuator_names=actuator_names,
            home=home,
            tool_site=tool_site,
            collision_bodies=collision_bodies,
            model_joint_names=model_joint_names,
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


def arm(*, config: PartConfig) -> base.Rig:
    """Build one legacy private-world MuJoCo part.

    New multi-part or RGB-D sites should declare ``worlds`` and select
    :func:`backend`; this factory remains compatible with existing manifests.
    """

    return _arm_rig(config)


def backend(*, config: WorldConfig) -> MujocoBackend:
    """Build one unopened shared MuJoCo world selected by ``site.yaml``."""

    return MujocoBackend(model_path=_model_path(config))
