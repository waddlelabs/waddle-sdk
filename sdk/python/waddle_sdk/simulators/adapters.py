"""Session-owned physics behind the ordinary SDK Driver and CameraDriver ports."""

from __future__ import annotations

import json
import math
import multiprocessing
import os
import subprocess
import sys
import threading
from pathlib import Path
from typing import Any

import numpy as np

from .. import descriptors
from ..cameras.base import CameraFrame
from ..cameras.site import CameraConfig
from ..robots import base
from ..robots.site import PartConfig
from .model import robot_links
from .scene import load_scene, profile


class World:
    """One lazy, reference-counted world owned exclusively by one Site.open()."""

    def __init__(self, config: dict):
        self.config = config
        self._lock = threading.RLock()
        self._users = 0
        self._process: subprocess.Popen | None = None
        self._connection = None
        self._failed = False

    def acquire(self) -> World:
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
                            "waddle_sdk.simulation.worker",
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
                    parent.send(self.config)
                    self._receive(180.0)
                except BaseException:
                    child.close()
                    parent.close()
                    self._shutdown()
                    self._failed = True
                    raise
            self._users += 1
            return self

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
                raise RuntimeError("simulation connection lost; reopen the site") from None

    def release(self):
        with self._lock:
            self._users -= 1
            if self._users == 0:
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


def _world(config: PartConfig | CameraConfig) -> World:
    path, definition = load_scene(config.site_root, config.connection.get("simulation"))
    key = (World, path)
    owner = config.resources.setdefault(key, World(definition))
    if json.dumps(owner.config, sort_keys=True) != json.dumps(definition, sort_keys=True):
        raise ValueError("simulation configuration changed during site assembly")
    return owner


class Driver:
    kind = "sim"

    def __init__(self, world: World, *, posture: str):
        self.world = world.acquire()
        self.profile = profile(world.config["robot"])
        self._monitor = posture == "monitor"
        self._closed = False
        self._estopped = False

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
        if self._monitor or self._estopped:
            raise RuntimeError("simulation arm is monitor-only or e-stopped")
        values = np.asarray(target, dtype=float)
        self.profile.poses(values)  # shared width/finite validation
        if any(
            x < lo or x > hi for x, (lo, hi) in zip(values, self.profile.limits, strict=True)
        ):
            raise ValueError("simulation target exceeds the robot's joint limits")
        self.world.call("write", values)

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
        # The one world clock runs in the process, including between commands.
        # Per-arm pump ticks must never advance shared physics a second time.
        pass

    def home(self, values):
        if self._monitor or self._estopped:
            return False
        self.profile.poses(values)
        if any(
            x < lo or x > hi for x, (lo, hi) in zip(values, self.profile.limits, strict=True)
        ):
            raise ValueError("simulation home exceeds the robot's joint limits")
        return self.world.call("home", tuple(values))

    def forward_kinematics(self, q):
        values = tuple(q) + ((0.0,) if len(q) == self.profile.dof else ())
        tcp = self.profile.poses(values)[-1]
        return tcp[:3, 3].copy(), tcp[:3, :3].copy()

    def collision_spheres(self, q):
        # Arm supplies only the arm rows to FK/geometry; bound the hand at its
        # widest opening when a prospective jaw position is not available.
        q = tuple(q) + ((1.0,) if len(q) == self.profile.dof else ())
        poses = self.profile.poses(q)
        result = []
        for index, link in enumerate(robot_links(self.profile)):
            if index < len(poses):
                pose = poses[index]
            else:
                pose = poses[-1].copy()
                pose[:3, 3] += pose[:3, :3] @ (
                    np.asarray(link.axis) * q[-1] * self.profile.opening / 2
                )
            for shape_index, shape in enumerate(link.shapes):
                if shape.kind == "box":
                    radius = float(np.linalg.norm(shape.size) / 2)
                elif shape.kind == "cylinder":
                    radius = math.hypot(shape.size[0], shape.size[1] / 2)
                else:
                    radius = shape.size[0]
                center = pose[:3, 3] + pose[:3, :3] @ shape.xyz
                result.append(
                    base.CollisionSphere(
                        name=f"{link.name}_{shape_index}",
                        center_m=center,
                        radius_m=radius,
                    )
                )
        return tuple(result)

    def close(self):
        if not self._closed:
            self._closed = True
            self.world.release()


def arm(*, config: PartConfig) -> base.Rig:
    owner = _world(config)
    p = profile(owner.config["robot"])
    if config.base_frame != p.frame:
        raise ValueError(f"{p.name} simulation requires its declared base frame {p.frame}")
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
        driver = Driver(owner, posture=config.posture)
        try:
            return {
                "": base.Arm(
                    part="",
                    driver=driver,
                    joint_names=p.names,
                    joint_limits=limits,
                    step_caps=tuple(v / rate for v in velocities),
                    base_frame=p.frame,
                    workspace=(
                        tuple(config.workspace_bounds["min"]),
                        tuple(config.workspace_bounds["max"]),
                    )
                    if config.workspace_bounds
                    else None,
                    fk=driver.forward_kinematics,
                    collision_spheres=driver.collision_spheres,
                    collision_frame=p.frame,
                    home_values=p.home,
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
                    for name, (lo, hi), v in zip(p.names, limits, velocities, strict=True)
                ),
                rate_hz=rate,
            ),
        ),
        build_arms=build,
        rate_hz=rate,
        posture=config.posture,
    )


class Camera:
    def __init__(self, owner: World, config: CameraConfig):
        self.name = config.name
        self._intrinsics = descriptors.Intrinsics(**dict(config.intrinsics or {}))
        self._closed = False
        self.world = owner.acquire()

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
            self.world.release()


def camera(*, config: CameraConfig) -> Camera:
    owner = _world(config)
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
            raise ValueError(f"camera {config.name} {key} differs from the simulation profile")
    return Camera(owner, config)
