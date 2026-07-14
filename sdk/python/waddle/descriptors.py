"""Descriptor sugar: pure-Python dataclasses that compile to canonical
proto3 JSON for ``waddle.v0`` messages (lowerCamelCase keys, int64 as
decimal strings, full prefixed enum names, proto3 defaults omitted).

These validate *shape* ("must declare"), never *behavior* — all semantic
validation happens in waddle-core when the JSON crosses the shim
(hollow-frontend rule).
"""

from __future__ import annotations

import abc
from collections.abc import Sequence
from dataclasses import dataclass, field

__all__ = [
    "Camera",
    "Chunking",
    "Composite",
    "EEDelta",
    "Gripper",
    "JointSpace",
    "Opaque",
    "Robot",
]


def _enum_name(value: str, prefix: str, allowed: dict[str, str], field_name: str) -> str:
    """Map a short case-insensitive name (or an already-prefixed full enum
    name) onto the full proto enum value name."""
    key = value.strip().lower()
    if key.startswith(prefix.lower() + "_"):
        key = key[len(prefix) + 1 :]
    if key not in allowed:
        options = ", ".join(sorted(allowed))
        raise ValueError(f"{field_name}={value!r}: expected one of {options}")
    return f"{prefix}_{allowed[key]}"


@dataclass(frozen=True)
class Chunking:
    """Chunk semantics for the declared action space.

    ``replan`` accepts short case-insensitive names (``"immediate"``,
    ``"chunk_boundary"``, ``"blend"``); ``interp`` accepts ``"hold"``,
    ``"linear"``, ``"cubic"``.
    """

    horizon: int
    replan: str = "IMMEDIATE"
    interp: str = "hold"

    def _compile(self) -> dict:
        out: dict = {}
        if self.horizon:
            out["horizonSteps"] = int(self.horizon)
        out["replan"] = _enum_name(
            self.replan,
            "REPLAN_POLICY",
            {"immediate": "IMMEDIATE", "chunk_boundary": "CHUNK_BOUNDARY", "blend": "BLEND"},
            "replan",
        )
        out["interpolation"] = _enum_name(
            self.interp,
            "INTERPOLATION",
            {"hold": "HOLD", "linear": "LINEAR", "cubic": "CUBIC"},
            "interp",
        )
        return out


@dataclass(frozen=True)
class Gripper:
    """A gripper declaration. Use the constructors: ``Gripper.parallel()``
    or ``Gripper.suction()``."""

    _kind: dict

    @classmethod
    def parallel(cls, *, dim: int = -1, open: float = 1.0, closed: float = 0.0) -> "Gripper":
        spec: dict = {}
        if open:
            spec["openValue"] = float(open)
        if closed:
            spec["closedValue"] = float(closed)
        if dim:
            spec["actionDim"] = int(dim)
        return cls({"parallel": spec})

    @classmethod
    def suction(cls) -> "Gripper":
        return cls({"suction": {}})

    def _compile(self) -> dict:
        return self._kind


class _Space(abc.ABC):
    """Common surface of every action-space descriptor."""

    rate_hz: float | None
    chunking: Chunking | None
    gripper: Gripper | None

    @abc.abstractmethod
    def _compile_kind(self) -> dict:
        """The space oneof arm, e.g. ``{"jointPosition": {...}}``."""

    @abc.abstractmethod
    def _space_kind(self) -> str:
        """The ``SPACE_KIND_*`` enum name (grants derivation)."""

    def _compile_space(self) -> dict:
        out = self._compile_kind()
        if self.rate_hz is not None:
            out["rateHz"] = float(self.rate_hz)
        if self.chunking is not None:
            out["chunking"] = self.chunking._compile()
        if self.gripper is not None:
            out["gripper"] = self.gripper._compile()
        return out


@dataclass(frozen=True)
class JointSpace(_Space):
    """Absolute joint positions, radians (v0 pins radians)."""

    joints: Sequence[str]
    rate_hz: float | None = None
    units: str | None = None
    chunking: Chunking | None = None
    gripper: Gripper | None = None

    def __post_init__(self) -> None:
        if self.units not in (None, "rad"):
            raise ValueError(
                f"units={self.units!r}: waddle v0 pins joint units to radians "
                '(pass "rad" or omit)'
            )
        if not self.joints:
            raise ValueError("JointSpace must declare at least one joint")

    def _compile_kind(self) -> dict:
        return {"jointPosition": {"joints": [{"name": str(j)} for j in self.joints]}}

    def _space_kind(self) -> str:
        return "SPACE_KIND_JOINT_POSITION"


@dataclass(frozen=True)
class EEDelta(_Space):
    """End-effector pose deltas. Rotation encoding and delta frame are
    must-declare (there is no safe default convention)."""

    frame_id: str
    rotation: str
    delta_frame: str
    rate_hz: float | None = None
    max_linear_step_m: float | None = None
    max_angular_step_rad: float | None = None
    chunking: Chunking | None = None
    gripper: Gripper | None = None

    def __post_init__(self) -> None:
        if not self.frame_id:
            raise ValueError("EEDelta must declare frame_id")

    def _compile_kind(self) -> dict:
        spec: dict = {
            "frameId": self.frame_id,
            "rotationEncoding": _enum_name(
                self.rotation,
                "ROTATION_ENCODING",
                {
                    "axis_angle": "AXIS_ANGLE",
                    "rotvec": "ROTVEC",
                    "euler_rpy": "EULER_RPY",
                    "euler_xyz": "EULER_XYZ",
                    "quat_xyzw": "QUAT_XYZW",
                    "quat_wxyz": "QUAT_WXYZ",
                },
                "rotation",
            ),
            "deltaFrame": _enum_name(
                self.delta_frame,
                "DELTA_FRAME",
                {"base": "BASE", "body": "BODY"},
                "delta_frame",
            ),
        }
        if self.max_linear_step_m is not None:
            spec["maxLinearStepM"] = float(self.max_linear_step_m)
        if self.max_angular_step_rad is not None:
            spec["maxAngularStepRad"] = float(self.max_angular_step_rad)
        return {"eeDelta": spec}

    def _space_kind(self) -> str:
        return "SPACE_KIND_EE_POSE_DELTA"


class Composite(_Space):
    """Named parts; keyword-argument insertion order is the normative part
    order (it defines the concatenated action-vector layout)."""

    def __init__(
        self,
        *,
        rate_hz: float | None = None,
        chunking: Chunking | None = None,
        gripper: Gripper | None = None,
        **parts: _Space,
    ) -> None:
        if not parts:
            raise ValueError("Composite must declare at least one part")
        for name, space in parts.items():
            if not isinstance(space, _Space):
                raise TypeError(f"part {name!r} must be an action-space descriptor")
        self.rate_hz = rate_hz
        self.chunking = chunking
        self.gripper = gripper
        self.parts = dict(parts)

    def _compile_kind(self) -> dict:
        return {
            "composite": {
                "parts": [
                    {"name": name, "space": space._compile_space()}
                    for name, space in self.parts.items()
                ]
            }
        }

    def _space_kind(self) -> str:
        return "SPACE_KIND_COMPOSITE"


@dataclass(frozen=True)
class Opaque(_Space):
    """Monitor-only escape hatch: never executable by Waddle."""

    format_hint: str
    dim: int | None = None
    rate_hz: float | None = None
    chunking: Chunking | None = None
    gripper: Gripper | None = None

    def _compile_kind(self) -> dict:
        spec: dict = {}
        if self.format_hint:
            spec["formatHint"] = self.format_hint
        if self.dim is not None:
            spec["dim"] = int(self.dim)
        return {"opaque": spec}

    def _space_kind(self) -> str:
        return "SPACE_KIND_OPAQUE"


@dataclass(frozen=True)
class Camera:
    """Declaration-only camera description (no capture tap in v1)."""

    width: int
    height: int
    fps: float
    encoding: str = "rgb8"
    frame_id: str | None = None

    def _compile(self, name: str) -> dict:
        out: dict = {
            "name": name,
            "width": int(self.width),
            "height": int(self.height),
            "fps": float(self.fps),
            "encoding": _enum_name(
                self.encoding,
                "CAMERA_ENCODING",
                {"rgb8": "RGB8", "bgr8": "BGR8", "z16": "Z16", "jpeg": "JPEG", "h264": "H264"},
                "encoding",
            ),
        }
        if self.frame_id:
            out["frameId"] = self.frame_id
        return out


@dataclass(frozen=True)
class Robot:
    """The robot declaration compiled to ``waddle.v0.RobotDescription``.
    Grants are NOT declared here — ``waddle.init`` derives them from which
    ``Control`` verbs are provided."""

    name: str
    action_space: _Space
    robot_id: str = ""
    cell_id: str = ""
    cameras: dict[str, Camera] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not isinstance(self.action_space, _Space):
            raise TypeError("action_space must be an action-space descriptor")

    def _compile(self, grants: list[dict]) -> dict:
        out: dict = {
            "name": self.name,
            "actionSpace": self.action_space._compile_space(),
        }
        if self.robot_id:
            out["robotId"] = self.robot_id
        if self.cell_id:
            out["cellId"] = self.cell_id
        if self.cameras:
            out["cameras"] = [cam._compile(name) for name, cam in self.cameras.items()]
        if grants:
            out["grants"] = grants
        return out
