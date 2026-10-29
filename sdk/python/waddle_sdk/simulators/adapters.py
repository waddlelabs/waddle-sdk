"""Session-owned physics behind the ordinary SDK Driver and CameraDriver ports."""

from __future__ import annotations

import math
import multiprocessing
import os
import subprocess
import sys
import threading
import time
from collections.abc import Sequence
from pathlib import Path

import numpy as np

from .. import descriptors
from ..cameras.base import CameraFrame
from ..cameras.site import CameraConfig
from ..cameras.timing import CameraContentTiming
from ..robots import base
from ..robots.site import PartConfig
from ..simulation import WorldConfig
from .description import collision_bounds, description
from .scene import load_scene, profile

# Command lead each reference arm joint admits unless the site declares its own bound:
# the YAM hardware factory default, so simulated and physical moves admit alike.
DEFAULT_MAX_JOINT_POSITION_ERROR_RAD = 0.2


class World:
    """One lazy world owned by the standard SDK simulation lifecycle."""

    def __init__(
        self, config: dict, *, reset_on_episode: bool = False, real_time: bool = True
    ):
        if type(reset_on_episode) is not bool:
            raise ValueError("reset_on_episode must be a boolean")
        if type(real_time) is not bool:
            raise ValueError("real_time must be a boolean")
        self._real_time = real_time
        self.config = config
        self._reset_on_episode = reset_on_episode
        self._lock = threading.RLock()
        self._requests = threading.Condition(self._lock)
        self._pending: list[tuple[bool, int]] = []
        self._request_sequence = 0
        self._busy = False
        self._process: subprocess.Popen | None = None
        self._connection = None
        self._failed = False
        self._closing = False
        self._part_names: set[str] = set()
        self._part_frames: dict[str, str | None] = {}
        self._renderer = None
        self._independent_render = False
        self._generation = 0
        self.content_timing_kind = None

    @property
    def supports_gripper_force(self) -> bool:
        """Whether this world serves the measured gripper force operations.

        MuJoCo worlds do. A world that relays a fixed set of operations to its
        engine overrides this to False, so its arms advertise position-only jaws.
        """
        return self.config["backend"] == "mujoco"

    def open(self) -> None:
        with self._lock:
            if self._failed:
                raise RuntimeError("simulation failed; reopen the site to recover")
            if self._process is None:
                self._closing = False
                self._generation = 0
                self._independent_render = False
                if os.name != "posix":
                    raise RuntimeError(
                        "physics workspace workers currently require Linux or macOS"
                    )
                parent, child = multiprocessing.Pipe(duplex=True)
                env = {
                    k: v
                    for k, v in os.environ.items()
                    if not k.startswith("WADDLE_")
                    and not k.endswith(("_API_KEY", "_TOKEN", "_SECRET"))
                }
                source = str(Path(__file__).resolve().parents[2])
                env["PYTHONPATH"] = os.pathsep.join(
                    filter(None, (source, env.get("PYTHONPATH")))
                )
                try:
                    self._process = subprocess.Popen(
                        [
                            self.config.get("worker_python", sys.executable),
                            "-m",
                            "waddle_sdk.simulators.worker",
                            str(child.fileno()),
                        ],
                        pass_fds=(child.fileno(),),
                        env=env,
                        stdin=subprocess.DEVNULL,
                        stdout=sys.stderr,
                        start_new_session=True,
                    )
                    self._connection = parent
                    child.close()
                    parent.send({**self.config, "_real_time": self._real_time})
                    ready = self._receive(180.0)
                    self.content_timing_kind = (
                        ready.get("camera_content_timing_kind")
                        if isinstance(ready, dict)
                        else None
                    )
                    if self.content_timing_kind not in (
                        None,
                        "sensor_exposure",
                        "simulated_state",
                    ):
                        raise ValueError(
                            "simulation worker advertised invalid camera timing"
                        )
                    if isinstance(ready, dict) and ready.get("independent_render"):
                        self._start_renderer(env)
                except BaseException:
                    child.close()
                    parent.close()
                    self._shutdown()
                    self._failed = True
                    raise

    def _start_renderer(self, env):
        from .render_client import RenderClient

        parent, child = multiprocessing.Pipe(duplex=True)
        process = None
        try:
            process = subprocess.Popen(
                [
                    self.config.get("worker_python", sys.executable),
                    "-m",
                    "waddle_sdk.simulators.render_worker",
                    str(child.fileno()),
                ],
                pass_fds=(child.fileno(),),
                env=env,
                stdin=subprocess.DEVNULL,
                stdout=sys.stderr,
                start_new_session=True,
            )
            child.close()
            snapshot = self.call("render_snapshot", None)
            parent.send((self.config, snapshot))
            if not parent.poll(180):
                raise TimeoutError("simulation renderer startup timed out")
            ok, result = parent.recv()
            if not ok:
                raise RuntimeError(f"simulation renderer startup failed: {result}")
            self._renderer = RenderClient(
                process,
                parent,
                cameras=self.config["cameras"],
                generation=snapshot["generation"],
            )
            self._independent_render = True
        except BaseException:
            child.close()
            parent.close()
            if process is not None:
                process.kill()
                process.wait()
            raise

    def capture(self, name, *, stamp=None, cancelled=lambda: False):
        renderer = self._renderer
        if renderer is None:
            raise RuntimeError("simulation renderer is unavailable")

        def acquire(known_generation):
            model = None
            for _ in range(3):
                if cancelled():
                    raise RuntimeError("simulation camera is closed")
                snapshot = self.call("render_snapshot", known_generation)
                while snapshot.get("model") is not None:
                    # Model replacement can be large. Acquire fresh feedback
                    # after transfer; its cost cannot restamp old state.
                    model = snapshot["model"]
                    snapshot = self.call("render_snapshot", snapshot["generation"])
                captured_stamp = None if stamp is None else stamp()
                if (
                    stamp is None
                    or time.monotonic_ns() - snapshot["interval"][1] <= 50_000_000
                ):
                    snapshot["model"] = model
                    return snapshot, captured_stamp
                # Discard a late acquisition completely. A fresh snapshot can
                # recover a scheduling interruption without relaxing pairing
                # or relabeling old feedback with a new clock stamp.
                known_generation = snapshot["generation"]
            raise RuntimeError(
                "simulation camera snapshot transport exceeded 50 ms pairing bound"
            )

        while not cancelled():
            rgb, depth, snapshot, captured_stamp = renderer.capture(
                name,
                acquire,
                cancelled,
            )
            if snapshot["generation"] == self._generation:
                return rgb, depth, snapshot, captured_stamp
        raise RuntimeError("simulation camera is closed")

    def _receive(self, timeout: float):
        if self._connection is None or not self._connection.poll(timeout):
            raise TimeoutError("simulation worker did not respond before its deadline")
        ok, result = self._connection.recv()
        if not ok:
            raise RuntimeError(f"{self.config['backend']} simulation failed: {result}")
        return result

    def call(self, operation: str, *arguments):
        if operation == "capture" and (
            self._independent_render or self._renderer is not None
        ):
            rgb, depth, snapshot, _ = self.capture(arguments[0])
            if len(arguments) == 2 and arguments[1] is True:
                return rgb, depth, _capture_timing(snapshot, depth)
            return rgb, depth
        # An active snapshot must finish, but queued camera/viewer snapshots
        # must not get ahead of state/control requests. This also covers the
        # independent renderer, which acquires state via render_snapshot.
        # Preserve FIFO within each priority and one transaction on the pipe.
        with self._requests:
            ticket = (
                operation in {"capture", "render_snapshot", "evaluation_snapshot"},
                self._request_sequence,
            )
            self._request_sequence += 1
            self._pending.append(ticket)
            try:
                self._requests.wait_for(
                    lambda: not self._busy and ticket == min(self._pending)
                )
            except BaseException:
                self._pending.remove(ticket)
                self._requests.notify_all()
                raise
            self._pending.remove(ticket)
            self._busy = True
        try:
            if (
                self._failed
                or self._connection is None
                or (self._closing and operation != "close")
            ):
                raise RuntimeError("simulation world is unavailable")
            try:
                if operation in {
                    "reset",
                    "evaluation_reset",
                    "evaluation_replace_scene",
                }:
                    self._generation += 1
                self._connection.send((operation, arguments))
                return self._receive(15.0 if operation == "capture" else 5.0)
            except (OSError, EOFError, TimeoutError):
                with self._lock:
                    self._failed = True
                    self._shutdown()
                raise RuntimeError(
                    "simulation connection lost; reopen the site"
                ) from None
        finally:
            with self._requests:
                self._busy = False
                self._requests.notify_all()

    def step(self, dt: float) -> None:
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("world step must be finite and non-negative")
        # Real-time workers advance elapsed time before every observable read,
        # write and capture. Sending an additional no-op step on each SDK pump
        # tick only competes with those requests for the shared connection.
        if not self._real_time:
            self.call("step", dt)

    def reset(self) -> bool:
        if self._reset_on_episode:
            return self.call("reset")
        # A control run is not a request to rearrange the physical scene.
        # Interactive press/release and later tasks continue from measured state.
        self.call("hold")
        return True

    def part(self, *, config: PartConfig) -> base.Rig:
        if config.name not in self.config["parts"]:
            raise ValueError("robot part is absent from the simulation profile")
        if config.name in self._part_names:
            raise ValueError("robot part was declared more than once")
        rig = _arm(self, config=config)
        self._part_names.add(config.name)
        self._part_frames[config.name] = config.base_frame
        return rig

    def part_call(self, operation: str, part: str, *arguments):
        if len(self.config["parts"]) == 1:
            return self.call(operation, *arguments)
        return self.call(f"{operation}_part", part, *arguments)

    def camera(self, *, config: CameraConfig) -> Camera:
        return _camera(self, config=config)

    def close(self) -> None:
        renderer = None
        try:
            with self._lock:
                self._closing = True
                renderer, self._renderer = self._renderer, None
                if self._connection is not None and not self._failed:
                    try:
                        self.call("close")
                    except (RuntimeError, OSError):
                        pass
                self._shutdown()
        finally:
            if renderer is not None:
                renderer.close()

    def _shutdown(self):
        renderer, self._renderer = self._renderer, None
        if renderer is not None:
            renderer.close()
        process, self._process = self._process, None
        if self._connection is not None:
            self._connection.close()
            self._connection = None
        if process is not None:
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()


class AdministrativeWorld(World):
    """Reference MuJoCo world with a separate trusted evaluation facet."""

    @property
    def supports_scene_replacement(self) -> bool:
        return "native_scene" in self.config

    def evaluation_snapshot(self):
        return self.call("evaluation_snapshot")

    def evaluation_replace_scene(self, config: WorldConfig) -> bool:
        _, definition = load_scene(
            config.site_root, config.connection.get("simulation")
        )
        accepted = self.call("evaluation_replace_scene", definition)
        if accepted:
            self.config = definition
        return accepted

    def evaluation_reset(
        self,
        *,
        seed: int,
        variation=None,
        preserve_free_bodies=(),
        retire_free_bodies=(),
    ) -> bool:
        if isinstance(seed, bool) or not isinstance(seed, int):
            raise TypeError("simulation reset seed must be an integer")
        if seed < 0 or seed > 2**63 - 1:
            raise ValueError("simulation reset seed must be between 0 and 2^63-1")
        return self.call(
            "evaluation_reset",
            seed,
            variation,
            preserve_free_bodies,
            retire_free_bodies,
        )


def backend(*, config: WorldConfig) -> World:
    """Declare a reference scene through the public SimulationBackend contract."""
    _, definition = load_scene(config.site_root, config.connection.get("simulation"))
    world_type = AdministrativeWorld if definition["backend"] == "mujoco" else World
    return world_type(
        definition,
        reset_on_episode=config.options.get("reset_on_episode", False),
        real_time=config.options.get("real_time", True),
    )


class Driver:
    kind = "sim"

    def __init__(
        self,
        world: World,
        *,
        part: str,
        posture: str,
        max_joint_speed: float = 0.5,
    ):
        self.world = world
        self.part = part
        self.profile = profile(world.config["robot"])
        self._monitor = posture == "monitor"
        self._closed = False
        self._estopped = False
        self._max_joint_speed = max_joint_speed

    @property
    def estopped(self):
        return self._estopped

    def read(self):
        q, dq = (
            np.asarray(x, dtype=float) for x in self.world.part_call("read", self.part)
        )
        # The public hand channel is a bounded open fraction, like the live
        # adapters. Soft contact limits can overshoot a native slide by microns.
        q[-1] = np.clip(q[-1], 0.0, 1.0)
        return q, dq

    @property
    def gripper_force_supported(self):
        return self.world.supports_gripper_force and self.profile.name == "yam"

    @property
    def gripper_force_limits_n(self):
        return tuple(self.world.part_call("gripper_force_limits", self.part))

    def read_gripper_force(self):
        from ..runtime import GripperForce

        return GripperForce(
            float(self.world.part_call("gripper_force", self.part)),
            "mujoco.yam.actuator_force",
            True,
        )

    def write_gripper_force(self, target, force_n, velocity=None):
        if self._monitor or self._estopped:
            raise RuntimeError("simulation arm is monitor-only or e-stopped")
        values = np.asarray(target, dtype=float)
        self.profile.poses(values)
        if any(
            x < lo or x > hi
            for x, (lo, hi) in zip(values, self.profile.limits, strict=True)
        ):
            raise ValueError("simulation target exceeds the robot's joint limits")
        self.world.part_call("write_force", self.part, values, force_n, velocity)

    def write(self, target):
        self._write(target)

    def write_position_velocity(self, target, velocity_feedforward_rad_s):
        velocity = np.asarray(velocity_feedforward_rad_s, dtype=float)
        if (
            velocity.shape != (len(self.profile.names),)
            or not np.isfinite(velocity).all()
        ):
            raise ValueError("simulation velocity must match the finite joint vector")
        velocity = np.clip(velocity, -self._max_joint_speed, self._max_joint_speed)
        velocity[-1] = 0.0  # A blocked jaw remains a position latch.
        self._write(target, velocity)
        return True

    def _write(self, target, velocity=None):
        if self._monitor or self._estopped:
            raise RuntimeError("simulation arm is monitor-only or e-stopped")
        values = np.asarray(target, dtype=float)
        self.profile.poses(values)  # shared width/finite validation
        if any(
            x < lo or x > hi
            for x, (lo, hi) in zip(values, self.profile.limits, strict=True)
        ):
            raise ValueError("simulation target exceeds the robot's joint limits")
        self.world.part_call("write", self.part, values, velocity)

    def hold(self):
        if not self._closed:
            self.world.part_call("hold", self.part)

    def estop(self):
        self._estopped = True
        self.hold()

    def re_enable(self):
        self.hold()
        self._estopped = False

    def step(self, dt):
        # The SDK's shared-world clock runs between commands.
        # Per-arm pump ticks must never advance shared physics a second time.
        pass

    def home(self, values):
        if self._monitor or self._estopped:
            return False
        self.profile.poses(values)
        if any(
            x < lo or x > hi
            for x, (lo, hi) in zip(values, self.profile.limits, strict=True)
        ):
            raise ValueError("simulation home exceeds the robot's joint limits")
        return self.world.part_call("home", self.part, tuple(values))

    def forward_kinematics(self, q):
        values = tuple(q) + ((0.0,) if len(q) == self.profile.dof else ())
        tcp = description(self.profile.name).poses(values)["tcp"]
        return tcp[:3, 3].copy(), tcp[:3, :3].copy()

    def collision_spheres(self, q):
        # Arm supplies only the arm rows to FK/geometry; bound the hand at its
        # widest opening when a prospective jaw position is not available.
        q = tuple(q) + ((1.0,) if len(q) == self.profile.dof else ())
        poses = description(self.profile.name).poses(q)
        result = []
        for name, link, center, radius in collision_bounds(self.profile.name):
            pose = poses[link]
            result.append(
                base.CollisionSphere(
                    name=name,
                    center_m=pose[:3, 3] + pose[:3, :3] @ center,
                    radius_m=radius,
                )
            )
        return tuple(result)

    def close(self):
        if not self._closed:
            self._closed = True


def _arm(owner: World, *, config: PartConfig) -> base.Rig:
    p = profile(owner.config["robot"])
    # Validate and cache the conservative CAD cover while assembling the site.
    # The first real-time command must not pay mesh parsing latency inside its
    # dispatch deadline.
    collision_bounds(p.name)
    if not isinstance(config.base_frame, str) or not config.base_frame.strip():
        raise ValueError("simulation requires a declared robot base frame")
    if set(config.joint_limits) != set(p.names):
        raise ValueError("simulation joint names/order must match the robot profile")
    limits = tuple(tuple(config.joint_limits[name]) for name in p.names)
    for requested, physical in zip(limits, p.limits, strict=True):
        if not physical[0] <= requested[0] < requested[1] <= physical[1]:
            raise ValueError("simulation owner limits may only tighten robot limits")
    if any(not lo <= q <= hi for q, (lo, hi) in zip(p.home, limits, strict=True)):
        raise ValueError("simulation owner limits must include the reference home")
    rate = float(config.options.get("rate_hz", 50.0))
    speed = float(config.options.get("max_joint_speed_rad_s", 0.5))
    hand_speed = float(config.options.get("max_gripper_speed_per_s", 1.0))
    if not all(math.isfinite(x) and x > 0 for x in (rate, speed, hand_speed)):
        raise ValueError("simulation rate and speed must be positive")
    velocities = (speed,) * p.dof + (hand_speed,)
    requested_error = config.options.get("max_joint_position_error_rad")
    if requested_error is None:
        requested_error = DEFAULT_MAX_JOINT_POSITION_ERROR_RAD
    values = (
        (requested_error,) * p.dof
        if isinstance(requested_error, (int, float))
        else requested_error
    )
    if (
        isinstance(requested_error, (str, bytes, bool))
        or not isinstance(values, Sequence)
        or len(values) != p.dof
        or any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value <= 0
            for value in values
        )
    ):
        raise ValueError(
            "max_joint_position_error_rad must be one finite positive value "
            f"or {p.dof} finite positive arm-joint values"
        )
    position_error_caps = tuple(float(value) for value in values) + (hand_speed / rate,)

    def build():
        driver = Driver(
            owner,
            part=config.name,
            posture=config.posture,
            max_joint_speed=speed,
        )
        try:
            return {
                "": base.Arm(
                    part="",
                    driver=driver,
                    joint_names=p.names,
                    joint_limits=limits,
                    step_caps=tuple(v / rate for v in velocities),
                    position_error_caps=position_error_caps,
                    base_frame=config.base_frame,
                    workspace=(
                        tuple(config.workspace_bounds["min"]),
                        tuple(config.workspace_bounds["max"]),
                    )
                    if config.workspace_bounds
                    else None,
                    fk=driver.forward_kinematics,
                    collision_spheres=driver.collision_spheres,
                    collision_frame=config.base_frame,
                    # The shared world owns initialization/reset; the ordinary
                    # per-arm episode hook must not teleport it independently.
                    home_values=None,
                    arm_dof=p.dof,
                    rate_hz=rate,
                )
            }
        except BaseException:
            driver.close()
            raise

    return base.Rig(
        declaration=descriptors.Robot(
            name=config.name,
            action_space=descriptors.JointSpace(
                joints=tuple(
                    descriptors.Joint(
                        name=name, min_position=lo, max_position=hi, max_velocity=v
                    )
                    for name, (lo, hi), v in zip(
                        p.names, limits, velocities, strict=True
                    )
                ),
                rate_hz=rate,
            ),
        ),
        build_arms=build,
        # Report at the same rate as the physical controller. Native physics
        # still advances at the scene's fixed substep inside the world worker.
        rate_hz=rate,
        posture=config.posture,
    )


class Camera:
    def __init__(self, owner: World, config: CameraConfig):
        self.name = config.name
        self._intrinsics = descriptors.Intrinsics(**dict(config.intrinsics or {}))
        self._closed = False
        self.world = owner
        self.content_timing_kind = getattr(owner, "content_timing_kind", None)

    def intrinsics(self):
        return self._intrinsics

    @property
    def capture_state_supported(self):
        return self.world._independent_render or self.world._renderer is not None

    @property
    def capture_revision(self):
        return str(self.world._generation)

    def capture_with_state(self, stamp):
        from ..cameras.capture_state import CameraCaptureState
        from ..runtime import PartObservation

        if not self.capture_state_supported:
            return self.capture()
        rgb, depth, snapshot, captured_stamp = self.world.capture(
            self.name,
            stamp=stamp,
            cancelled=lambda: self._closed,
        )
        parts = {}
        for name, (q, dq) in snapshot["parts"].items():
            q = np.asarray(q, dtype=float).copy()
            q[-1] = np.clip(q[-1], 0, 1)
            robot = self.world.config["robot"]
            arm_q = tuple(q[: profile(robot).dof]) + (0.0,)
            tcp = description(robot).poses(arm_q)["tcp"]
            frame = self.world._part_frames.get(name)
            parts[name] = PartObservation(
                q,
                np.asarray(dq, dtype=float),
                None
                if frame is None
                else np.concatenate((tcp[:3, 3], base.quaternion_wxyz(tcp[:3, :3]))),
                frame,
            )
        return CameraFrame(
            rgb=rgb,
            depth=depth,
            content_timing=CameraContentTiming(**_capture_timing(snapshot, depth)),
            capture_state=CameraCaptureState(
                captured_stamp, parts, str(snapshot["generation"])
            ),
        )

    def capture(self):
        if self._closed:
            raise RuntimeError("simulation camera is closed")
        if self.capture_state_supported:
            rgb, depth, snapshot, _ = self.world.capture(
                self.name,
                cancelled=lambda: self._closed,
            )
            timing = _capture_timing(snapshot, depth)
            return CameraFrame(
                rgb=rgb,
                depth=depth,
                content_timing=None
                if timing is None
                else CameraContentTiming(**timing),
            )
        if self.content_timing_kind is None:
            rgb, depth = self.world.call("capture", self.name)
            return CameraFrame(rgb=rgb, depth=depth)
        rgb, depth, timing = self.world.call("capture", self.name, True)
        return CameraFrame(
            rgb=rgb, depth=depth, content_timing=CameraContentTiming(**timing)
        )

    def close(self):
        if not self._closed:
            self._closed = True


def _capture_timing(snapshot, depth):
    interval = snapshot.get("interval")
    if interval is None:
        return None
    return {
        "kind": "simulated_state",
        "clock_revision": "local-host-monotonic/v1",
        "rgb_monotonic_ns": interval,
        "depth_monotonic_ns": interval if depth is not None else None,
    }


def _camera(owner: World, *, config: CameraConfig) -> Camera:
    row = owner.config["cameras"].get(config.name)
    if row is None:
        raise ValueError("camera name is absent from the simulation profile")
    mount = (
        None
        if config.mount is None
        else {
            "kind": config.mount.kind,
            **({"part": config.mount.part} if config.mount.part else {}),
        }
    )
    for key, value in (
        ("stream", dict(config.stream)),
        ("intrinsics", dict(config.intrinsics or {})),
        ("frame_id", config.frame_id),
        ("mount", mount),
        ("options", dict(config.options)),
    ):
        if row[key] != value:
            raise ValueError(
                f"camera {config.name} {key} differs from the simulation profile"
            )
    return Camera(owner, config)
