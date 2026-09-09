"""Session-owned physics behind the ordinary SDK Driver and CameraDriver ports."""

from __future__ import annotations

import math
import multiprocessing
import os
import subprocess
import sys
import threading
from pathlib import Path

import numpy as np

from .. import descriptors
from ..cameras.base import CameraFrame
from ..cameras.site import CameraConfig
from ..robots import base
from ..robots.site import PartConfig
from ..simulation import WorldConfig
from .description import collision_bounds, description
from .scene import load_scene, profile


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
        self._process: subprocess.Popen | None = None
        self._connection = None
        self._failed = False
        self._part_name: str | None = None

    def open(self) -> None:
        with self._lock:
            if self._failed:
                raise RuntimeError("simulation failed; reopen the site to recover")
            if self._process is None:
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
                    self._receive(180.0)
                except BaseException:
                    child.close()
                    parent.close()
                    self._shutdown()
                    self._failed = True
                    raise

    def _receive(self, timeout: float):
        if self._connection is None or not self._connection.poll(timeout):
            raise TimeoutError("simulation worker did not respond before its deadline")
        ok, result = self._connection.recv()
        if not ok:
            raise RuntimeError(f"{self.config['backend']} simulation failed: {result}")
        return result

    def call(self, operation: str, *arguments):
        with self._lock:
            if self._failed or self._connection is None:
                raise RuntimeError("simulation world is unavailable")
            try:
                self._connection.send((operation, arguments))
                return self._receive(15.0 if operation == "capture" else 5.0)
            except (OSError, EOFError, TimeoutError):
                self._failed = True
                self._shutdown()
                raise RuntimeError(
                    "simulation connection lost; reopen the site"
                ) from None

    def step(self, dt: float) -> None:
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("world step must be finite and non-negative")
        self.call("step", dt)

    def reset(self) -> bool:
        if self._reset_on_episode:
            return self.call("reset")
        # A control run is not a request to rearrange the physical scene.
        # Interactive press/release and later tasks continue from measured state.
        self.call("hold")
        return True

    def part(self, *, config: PartConfig) -> base.Rig:
        if self._part_name is not None and self._part_name != config.name:
            raise ValueError("a reference simulation world contains one robot part")
        rig = _arm(self, config=config)
        self._part_name = config.name
        return rig

    def camera(self, *, config: CameraConfig) -> Camera:
        return _camera(self, config=config)

    def close(self) -> None:
        with self._lock:
            if self._connection is not None and not self._failed:
                try:
                    self.call("close")
                except (RuntimeError, OSError):
                    pass
            self._shutdown()

    def _shutdown(self):
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


def backend(*, config: WorldConfig) -> World:
    """Declare a reference scene through the public SimulationBackend contract."""
    _, definition = load_scene(config.site_root, config.connection.get("simulation"))
    return World(
        definition,
        reset_on_episode=config.options.get("reset_on_episode", False),
        real_time=config.options.get("real_time", True),
    )


class Driver:
    kind = "sim"

    def __init__(self, world: World, *, posture: str, max_joint_speed: float = 0.5):
        self.world = world
        self.profile = profile(world.config["robot"])
        self._monitor = posture == "monitor"
        self._closed = False
        self._estopped = False
        self._max_joint_speed = max_joint_speed

    @property
    def estopped(self):
        return self._estopped

    def read(self):
        q, dq = (np.asarray(x, dtype=float) for x in self.world.call("read"))
        # The public hand channel is a bounded open fraction, like the live
        # adapters. Soft contact limits can overshoot a native slide by microns.
        q[-1] = np.clip(q[-1], 0.0, 1.0)
        return q, dq

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
        self.world.call("write", values, velocity)

    def hold(self):
        if not self._closed:
            self.world.call("hold")

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
        return self.world.call("home", tuple(values))

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

    def build():
        driver = Driver(owner, posture=config.posture, max_joint_speed=speed)
        try:
            return {
                "": base.Arm(
                    part="",
                    driver=driver,
                    joint_names=p.names,
                    joint_limits=limits,
                    step_caps=tuple(v / rate for v in velocities),
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
        # Separate state reporting from native physics substeps, as in
        # ManiSkill. Oversample control to avoid whole command-interval jumps,
        # without a process round trip and FK report for every 2 ms substep.
        # The action space and owner step limits keep their declared rate.
        rate_hz=max(100.0, 2 * rate),
        posture=config.posture,
    )


class Camera:
    def __init__(self, owner: World, config: CameraConfig):
        self.name = config.name
        self._intrinsics = descriptors.Intrinsics(**dict(config.intrinsics or {}))
        self._closed = False
        self.world = owner

    def intrinsics(self):
        return self._intrinsics

    def capture(self):
        if self._closed:
            raise RuntimeError("simulation camera is closed")
        rgb, depth = self.world.call("capture", self.name)
        return CameraFrame(rgb=rgb, depth=depth)

    def close(self):
        if not self._closed:
            self._closed = True


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
    ):
        if row[key] != value:
            raise ValueError(
                f"camera {config.name} {key} differs from the simulation profile"
            )
    return Camera(owner, config)
