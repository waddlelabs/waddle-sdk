"""Pure interpretation of public runtime action and physical gripper metadata.

These helpers accept ``SdkRuntimePort.describe()`` data and never open hardware.
Missing or invalid optional mappings do not prevent other parts from being used.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass


def part_action_spaces(description: Mapping) -> dict[str, dict]:
    """Return explicitly named part action spaces, without guessing topology."""
    action = description.get("robot", {}).get("actionSpace", {})
    if "composite" in action:
        return {p["name"]: p["space"] for p in action["composite"]["parts"]}
    parts = description.get("parts", {})
    return {next(iter(parts)): action} if len(parts) == 1 else {}


def _finite(value: object) -> bool:
    try:
        return (
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(value)
        )
    except OverflowError:
        return False


@dataclass(frozen=True)
class GripperMapping:
    """An affine jaw-opening/action mapping, including reversed action ranges.

    Use :func:`gripper_mapping` to validate public metadata against its action row.
    Observation conversion does not clamp: out-of-range measurements remain
    visible. Command admission and motion timing belong to the caller.
    """

    joint: str
    closed_m: float
    open_m: float
    closed_action: float
    open_action: float

    def __post_init__(self) -> None:
        if (
            not isinstance(self.joint, str)
            or not self.joint
            or not all(
                _finite(v)
                for v in (
                    self.closed_m,
                    self.open_m,
                    self.closed_action,
                    self.open_action,
                )
            )
            or not 0 <= self.closed_m < self.open_m
            or self.closed_action == self.open_action
            or not _finite(self.open_m - self.closed_m)
            or not _finite(self.open_action - self.closed_action)
        ):
            raise ValueError(
                "gripper mapping requires a named joint and finite nonzero spans"
            )

        if not _finite(self.metres_per_action) or self.metres_per_action == 0:
            raise ValueError("gripper conversion factor must be finite and nonzero")

    @property
    def metres_per_action(self) -> float:
        """Signed conversion factor for differences (negative for reversed actions)."""
        return (self.open_m - self.closed_m) / (self.open_action - self.closed_action)

    def opening(self, action: float) -> float:
        """Convert a finite measured action coordinate to jaw metres, unclamped."""
        if not _finite(action):
            raise ValueError("gripper action must be finite")
        return self.closed_m + (action - self.closed_action) * self.metres_per_action

    def action(self, opening_m: float) -> float:
        """Convert a finite opening in the declared physical range; never clamp."""
        if not _finite(opening_m) or not self.closed_m <= opening_m <= self.open_m:
            raise ValueError(
                "opening must be inside the declared physical gripper range"
            )
        return self.closed_action + (opening_m - self.closed_m) / self.metres_per_action


def gripper_mapping(
    mapping: object, joints: Sequence[Mapping]
) -> GripperMapping | None:
    """Resolve an optional physical mapping against exactly one named action row.

    Invalid/missing mappings return ``None``. Arbitrary joint counts, names and
    action units are supported; no arm or gripper topology is inferred.
    """
    if not isinstance(mapping, Mapping):
        return None
    try:
        result = GripperMapping(
            **{
                k: mapping[k]
                for k in ("joint", "closed_m", "open_m", "closed_action", "open_action")
            }
        )
    except (KeyError, TypeError, ValueError):
        return None
    rows = [j for j in joints if j.get("name") == result.joint]
    if len(rows) != 1:
        return None
    lower, upper = rows[0].get("minPosition"), rows[0].get("maxPosition")
    if (
        not _finite(lower)
        or not _finite(upper)
        or not all(
            lower <= value <= upper
            for value in (result.closed_action, result.open_action)
        )
    ):
        return None
    return result
