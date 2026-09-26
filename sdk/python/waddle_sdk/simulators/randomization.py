"""Deterministic bounded pose variation for reference task environments."""

from __future__ import annotations

import hashlib
from dataclasses import dataclass

from .scene import TASK_ENVIRONMENTS

# The standard YAM S02 pose distribution keeps the cube close to the base while
# covering the reachable tray-free tabletop area on repeated evaluation resets.
YAM_PICK_LIFT_POSE = {
    "x_offset_m": -0.03,
    "x_half_range_m": 0.04,
    "y_half_range_m": 0.06,
}


@dataclass(frozen=True)
class PoseGroup:
    """Root bodies transformed together to preserve assembled free fixtures."""

    bodies: tuple[str, ...]
    translation_xy_m: float
    yaw_rad: float


_FIXED_GROUPS = {
    "drawer": (("cabinet",),),
    "touch_target": (
        ("distractor_pad_left",),
        ("target_pad",),
        ("distractor_pad_right",),
    ),
    "operate_control": (("control_panel",),),
    "close-drawer": (("cabinet",),),
    "open-hinged-door": (("door_frame",),),
}

_COUPLED_FREE_GROUPS = {
    "stabilize-remove-lid": (("movable_box", "box_lid"),),
    "loaded-tray-transport": (("loaded_tray", "tray_content_1", "tray_content_2"),),
    "uncap-return-test-tube": (("target_test_tube", "target_tube_cap"),),
}

_BOUNDS = {
    "drawer": (0.012, 0.035),
    "touch_target": (0.015, 0.035),
    "operate_control": (0.015, 0.035),
    "close-drawer": (0.012, 0.035),
    "open-hinged-door": (0.012, 0.035),
    "retrieve-bottle-clutter": (0.003, 0.06),
    "uncap-return-test-tube": (0.001, 0.035),
    # Keep densely packed candy starts physically separated while retaining
    # seed-dependent grasp choices and orientations.
    "candy-bin-transfer": (0.002, 0.05),
    # Keep the twenty single-layer chocolates separated inside their source tray.
    "chocolate-packing": (0.001, 0.05),
    "shampoo-packing": (0.0, 0.0),
}


def pose_groups(environment, object_groups):
    """Return the finite development distribution for a built-in task scene."""

    if environment != "drawer" and environment not in TASK_ENVIRONMENTS:
        return ()
    roots = {group[0].name: group[0] for group in object_groups if group}
    declared = list(_FIXED_GROUPS.get(environment, ()))
    coupled = _COUPLED_FREE_GROUPS.get(environment, ())
    declared.extend(coupled)
    claimed = {name for group in declared for name in group}
    declared.extend(
        (name,)
        for name, link in roots.items()
        if link.kind == "free" and name not in claimed
    )
    translation, yaw = _BOUNDS.get(environment, (0.012, 0.08726646259971647))
    result = tuple(
        PoseGroup(tuple(group), translation, yaw)
        for group in declared
        if all(name in roots for name in group)
    )
    if not result:
        raise ValueError("task environment has no bounded pose randomization group")
    return result


def sample(seed, environment, group_index, field, limit):
    """Map an exact seed/key to a stable closed-open value inside +/- limit."""

    if seed == 0:
        return 0.0
    payload = f"waddle.mujoco.pose/v1\0{seed}\0{environment}\0{group_index}\0{field}"
    integer = int.from_bytes(hashlib.sha256(payload.encode()).digest()[:8], "big")
    unit = integer / 2**64
    return (2.0 * unit - 1.0) * limit


def variation_sample(seed, environment, group, field, limit):
    """Return a stable bounded value for non-pose variation dimensions."""

    if seed == 0:
        return 0.0
    payload = f"waddle.mujoco.variation/v1\0{seed}\0{environment}\0{group}\0{field}"
    integer = int.from_bytes(hashlib.sha256(payload.encode()).digest()[:8], "big")
    unit = integer / 2**64
    return (2.0 * unit - 1.0) * limit


def held_out_sample(seed, environment, field, inner, outer):
    """Select a stable signed value outside the open interval +/- ``inner``."""

    if seed == 0:
        raise ValueError("held-out variation requires a nonzero seed")
    if not 0 <= inner < outer:
        raise ValueError("held-out bounds must satisfy 0 <= inner < outer")
    payload = f"waddle.mujoco.held-out/v1\0{seed}\0{environment}\0{field}"
    digest = hashlib.sha256(payload.encode()).digest()
    unit = int.from_bytes(digest[:8], "big") / 2**64
    sign = -1.0 if digest[8] & 1 else 1.0
    return sign * (inner + unit * (outer - inner))
