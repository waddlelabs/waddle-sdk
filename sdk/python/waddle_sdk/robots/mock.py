"""Dependency-free manifest adapter for a deterministic simulated arm."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

import numpy as np

from .. import descriptors
from . import base
from .site import PartConfig

__all__ = ["arm", "safety_presets"]


def safety_presets(*, factory: str, options: Mapping[str, object]):
    """Return a complete planar reach box for the dependency-free twin."""

    if factory != "arm":
        return ()
    count = int(options.get("joint_count", 6))
    link_length_m = float(options.get("link_length_m", 0.1))
    body_radius_m = float(options.get("body_radius_m", 0.02))
    reach = count * link_length_m + body_radius_m
    from .safety import SafetyPreset

    return (
        SafetyPreset(
            identifier="mock-planar-reach",
            label="Simulation reach envelope",
            workspace_bounds={
                "min": [-reach, -reach, -body_radius_m],
                "max": [reach, reach, body_radius_m],
            },
            review="Derived from the configured planar link count, length, and body radius.",
        ),
    )


def _limits(
    config: PartConfig,
) -> tuple[tuple[str, ...], tuple[tuple[float, float], ...]]:
    declared = config.joint_limits
    names_option = config.options.get("joint_names", ())
    if isinstance(declared, Mapping) and declared:
        names = tuple(str(name) for name in declared)
        limits = tuple((float(row[0]), float(row[1])) for row in declared.values())
    elif isinstance(declared, Sequence) and not isinstance(declared, (str, bytes)):
        limits = tuple((float(row[0]), float(row[1])) for row in declared)
        names = tuple(str(value) for value in names_option) or tuple(
            f"joint_{index}" for index in range(len(limits))
        )
    else:
        names = tuple(str(value) for value in names_option)
        if not names:
            count = int(config.options.get("joint_count", 6))
            if count <= 0:
                raise ValueError("mock joint_count must be positive")
            names = tuple(f"joint_{index}" for index in range(count))
        limits = tuple((-math.pi, math.pi) for _ in names)
    if len(names) != len(limits) or not names:
        raise ValueError(
            "mock joint_names and joint_limits must have equal non-zero width"
        )
    if len(set(names)) != len(names):
        raise ValueError("mock joint_names must be unique")
    return names, limits


def _planar_points(
    q: Sequence[float], *, link_length_m: float
) -> tuple[np.ndarray, ...]:
    angle = 0.0
    point = np.zeros(3, dtype=float)
    points = []
    for value in q:
        angle += float(value)
        point = point + np.array(
            [link_length_m * math.cos(angle), link_length_m * math.sin(angle), 0.0]
        )
        points.append(point.copy())
    return tuple(points)


def arm(*, config: PartConfig) -> base.Rig:
    """Build one lazy simulated part from a strict :class:`PartConfig`."""

    names, limits = _limits(config)
    rate_hz = float(config.options.get("rate_hz", 20.0))
    if not math.isfinite(rate_hz) or rate_hz <= 0.0:
        raise ValueError("mock rate_hz must be finite and positive")
    raw_caps = config.options.get("step_caps")
    step_caps = (
        tuple(float(value) for value in raw_caps)
        if isinstance(raw_caps, Sequence) and not isinstance(raw_caps, (str, bytes))
        else tuple(0.2 for _ in names)
    )
    if len(step_caps) != len(names):
        raise ValueError("mock step_caps must have one value per joint")
    raw_home = config.options.get("home")
    home = (
        tuple(float(value) for value in raw_home)
        if isinstance(raw_home, Sequence) and not isinstance(raw_home, (str, bytes))
        else tuple((lower + upper) / 2.0 for lower, upper in limits)
    )
    if len(home) != len(names):
        raise ValueError("mock home must have one value per joint")
    link_length_m = float(config.options.get("link_length_m", 0.1))
    body_radius_m = float(config.options.get("body_radius_m", 0.02))
    if not math.isfinite(link_length_m) or link_length_m <= 0.0:
        raise ValueError("mock link_length_m must be finite and positive")
    if not math.isfinite(body_radius_m) or body_radius_m <= 0.0:
        raise ValueError("mock body_radius_m must be finite and positive")
    legacy_frame = str(config.options.get("collision_frame", "site"))
    if (
        config.base_frame
        and "collision_frame" in config.options
        and config.base_frame != legacy_frame
    ):
        raise ValueError("mock base_frame conflicts with legacy options.collision_frame")
    collision_frame = config.base_frame or legacy_frame

    def points(q: Sequence[float]) -> tuple[np.ndarray, ...]:
        return _planar_points(q, link_length_m=link_length_m)

    def fk(q: Sequence[float]) -> tuple[np.ndarray, np.ndarray]:
        return points(q)[-1], np.eye(3)

    def collision_spheres(q: Sequence[float]) -> tuple[base.CollisionSphere, ...]:
        return tuple(
            base.CollisionSphere(
                name=f"link_{index}",
                center_m=point,
                radius_m=body_radius_m,
            )
            for index, point in enumerate(points(q))
        )

    workspace = config.workspace_bounds
    workspace_box = (
        None if not workspace else (tuple(workspace["min"]), tuple(workspace["max"]))
    )

    def build_arms() -> dict[str, base.Arm]:
        driver = base.SimDriver(
            home,
            lower=[lower for lower, _upper in limits],
            upper=[upper for _lower, upper in limits],
            step_caps=step_caps,
            rate_hz=rate_hz,
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
                fk=fk,
                collision_spheres=collision_spheres,
                collision_frame=collision_frame,
                home_values=home,
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
