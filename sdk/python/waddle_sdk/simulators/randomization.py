"""Deterministic bounded pose variation for reference task environments."""

from __future__ import annotations

import hashlib
from dataclasses import dataclass

from .scene import TASK_ENVIRONMENTS


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

    payload = f"waddle.mujoco.pose/v1\0{seed}\0{environment}\0{group_index}\0{field}"
    integer = int.from_bytes(hashlib.sha256(payload.encode()).digest()[:8], "big")
    unit = integer / 2**64
    return (2.0 * unit - 1.0) * limit
