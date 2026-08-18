"""Fake site drivers used by the primary-API contract tests."""

from __future__ import annotations

import threading
import time

import numpy as np

from waddle_sdk import descriptors
from waddle_sdk.cameras import CameraFrame
from waddle_sdk.robots import base

opened = {"arms": 0, "cameras": 0}
closed = {"arms": 0, "cameras": 0}
velocity_commands: list[tuple[np.ndarray, np.ndarray]] = []


class _Driver(base.SimDriver):
    def __init__(self) -> None:
        super().__init__(
            [0.0, 0.0],
            lower=[-1.0, -1.0],
            upper=[1.0, 1.0],
            step_caps=[0.2, 0.2],
            rate_hz=20.0,
        )

    def close(self) -> None:
        closed["arms"] += 1

    def write_position_velocity(self, target, velocity_feedforward_rad_s) -> bool:
        velocity_commands.append(
            (
                np.asarray(target, dtype=float).copy(),
                np.asarray(velocity_feedforward_rad_s, dtype=float).copy(),
            )
        )
        super().write(target)
        return True


class _Camera:
    def __init__(self) -> None:
        self._closing = threading.Event()
        self._sequence = 0

    def capture(self) -> CameraFrame:
        if self._closing.wait(0.01):
            raise RuntimeError("camera closed")
        self._sequence += 1
        return CameraFrame(
            rgb=np.full((2, 2, 3), self._sequence % 255, dtype=np.uint8),
            depth=np.full((2, 2), 1000, dtype=np.uint16),
        )

    def close(self) -> None:
        if not self._closing.is_set():
            closed["cameras"] += 1
            self._closing.set()


def reset() -> None:
    for values in (opened, closed):
        values["arms"] = 0
        values["cameras"] = 0
    velocity_commands.clear()


def _collision_spheres(q):
    return (
        base.CollisionSphere("link_0", (float(q[0]), 0.0, 0.0), 0.02),
        base.CollisionSphere(
            "link_1", (float(q[1]) + 0.2, float(q[0]) + float(q[1]), 0.0), 0.02
        ),
    )


def part(*, config) -> base.Rig:
    assert config.name == "arm"
    assert config.posture == "supervised"

    def build_arms():
        opened["arms"] += 1
        return {
            "": base.Arm(
                part="",
                driver=_Driver(),
                joint_names=("j0", "j1"),
                joint_limits=((-1.0, 1.0), (-1.0, 1.0)),
                step_caps=(0.2, 0.2),
                collision_spheres=_collision_spheres,
                collision_frame="cell",
                rate_hz=20.0,
                home_values=(0.0, 0.0),
            )
        }

    return base.Rig(
        declaration=descriptors.Robot(
            name="fake-arm",
            action_space=descriptors.JointSpace(joints=("j0", "j1"), rate_hz=20.0),
        ),
        build_arms=build_arms,
        rate_hz=20.0,
        posture=config.posture,
        report=lambda _line: None,
    )


def camera(*, config):
    assert config.stream["width"] == 2
    opened["cameras"] += 1
    return _Camera()
