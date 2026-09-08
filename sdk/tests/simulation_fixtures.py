"""Dependency-free modular simulation backend used by SDK contract tests."""

from __future__ import annotations

import threading

import numpy as np
from waddle_sdk import descriptors
from waddle_sdk.cameras import CameraFrame
from waddle_sdk.robots import base

events: list[str] = []


def reset() -> None:
    events.clear()


class _Driver(base.SimDriver):
    def __init__(self, world: "World") -> None:
        self._world = world
        super().__init__(
            [0.0, 0.0],
            lower=[-1.0, -1.0],
            upper=[1.0, 1.0],
            step_caps=[0.2, 0.2],
            rate_hz=20.0,
        )

    def step(self, dt: float) -> None:
        # World-backed parts are advanced once by World.step(), not once per arm.
        del dt

    def close(self) -> None:
        events.append("arm.close")


class _Camera:
    def __init__(self, world: "World") -> None:
        self._world = world
        self._closing = threading.Event()

    def capture(self) -> CameraFrame:
        if self._closing.wait(0.005):
            raise RuntimeError("camera closed")
        if not self._world.opened:
            raise RuntimeError("world is not open")
        return CameraFrame(
            rgb=np.full((2, 3, 3), 17, dtype=np.uint8),
            depth=np.full((2, 3), 750, dtype=np.uint16),
        )

    def close(self) -> None:
        if not self._closing.is_set():
            events.append("camera.close")
            self._closing.set()


class World:
    def __init__(self, config) -> None:
        events.append("world.factory")
        assert config.name == "cell"
        assert config.connection == {"scene": "scenes/cell.usd"}
        assert config.options == {"renderer": "test"}
        self.opened = False
        self.closed = False
        self.steps = 0
        self.resets = 0

    def part(self, *, config) -> base.Rig:
        events.append("world.part")
        assert not self.opened
        assert config.world == "cell"
        assert config.name in {"arm", "arm2"}

        def build_arms():
            if not self.opened:
                raise RuntimeError("world is not open")
            events.append("arm.open")
            return {
                "": base.Arm(
                    part="",
                    driver=_Driver(self),
                    joint_names=("j0", "j1"),
                    joint_limits=((-1.0, 1.0), (-1.0, 1.0)),
                    step_caps=(0.2, 0.2),
                    base_frame=config.base_frame or "",
                    rate_hz=20.0,
                    home_values=(0.0, 0.0),
                )
            }

        return base.Rig(
            declaration=descriptors.Robot(
                name=config.name,
                action_space=descriptors.JointSpace(
                    joints=("j0", "j1"), rate_hz=20.0
                ),
            ),
            build_arms=build_arms,
            rate_hz=20.0,
            posture=config.posture,
            report=lambda _line: None,
        )

    def camera(self, *, config):
        if not self.opened:
            raise RuntimeError("world is not open")
        events.append("camera.open")
        assert config.world == "cell"
        assert config.mount is not None and config.mount.kind == "scene"
        return _Camera(self)

    def open(self) -> None:
        events.append("world.open")
        self.opened = True

    def step(self, dt: float) -> None:
        assert dt > 0.0
        if not self.opened:
            raise RuntimeError("world is not open")
        self.steps += 1

    def reset(self) -> bool:
        if not self.opened:
            raise RuntimeError("world is not open")
        events.append("world.reset")
        self.resets += 1
        return True

    def close(self) -> None:
        if not self.closed:
            events.append("world.close")
            self.closed = True
            self.opened = False


worlds: list[World] = []


def backend(*, config):
    world = World(config)
    worlds.append(world)
    return world


class NoCameraWorld(World):
    camera = None


def no_camera_backend(*, config):
    return NoCameraWorld(config)


class BrokenCameraWorld(World):
    def camera(self, *, config):
        del config
        events.append("camera.open.failed")
        raise RuntimeError("renderer failed")


def broken_camera_backend(*, config):
    world = BrokenCameraWorld(config)
    worlds.append(world)
    return world
