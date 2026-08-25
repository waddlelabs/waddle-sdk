"""Typed contracts shared by local and remote SDK runtime adapters.

The protocol is deliberately structural: Metal can depend on this public
module without importing a transport implementation or any SDK internals.
Concrete authority, timing, gating, and recording remain native-core owned.
"""

from __future__ import annotations

import enum
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import TYPE_CHECKING, Any, Protocol, TypeAlias, runtime_checkable

import numpy as np
import numpy.typing as npt

from .cameras import CameraSample

JSONScalar: TypeAlias = str | int | float | bool | None
JSONValue: TypeAlias = JSONScalar | list["JSONValue"] | dict[str, "JSONValue"]
SUPPORT_CONTRACT_VERSION = "waddle.sdk.support/v1"


def _validate_sha256_digest(value: str) -> None:
    if len(value) != 64 or any(char not in "0123456789abcdef" for char in value):
        raise ValueError("embodiment digest must be a lowercase sha256 hex digest")


def _freeze_json(value: object) -> object:
    """Take an immutable snapshot of one JSON-shaped value."""
    if isinstance(value, Mapping):
        return MappingProxyType(
            {str(key): _freeze_json(item) for key, item in value.items()}
        )
    if isinstance(value, (list, tuple)):
        return tuple(_freeze_json(item) for item in value)
    return value


def _thaw_json(value: object) -> JSONValue:
    """Return an ordinary JSON-shaped copy of an immutable snapshot."""
    if isinstance(value, Mapping):
        return {str(key): _thaw_json(item) for key, item in value.items()}
    if isinstance(value, tuple):
        return [_thaw_json(item) for item in value]
    return value  # type: ignore[return-value]


class SupportFact(str, enum.Enum):
    """One SDK implementation or declaration fact used by Metal.

    These are support facts, not robot skill capabilities. A fact can satisfy
    one prerequisite of a Metal capability, but it never grants permission or
    says that a complete skill is available.
    """

    JOINT_POSITION_OBSERVATION = "observation.joint_position"
    JOINT_VELOCITY_OBSERVATION = "observation.joint_velocity"
    EE_POSE_OBSERVATION = "observation.ee_pose"
    JOINT_POSITION_ACTION = "action.joint_position"
    VELOCITY_FEEDFORWARD = "actuation.velocity_feedforward"
    FORWARD_KINEMATICS = "kinematics.fk"
    BODY_SPHERES = "geometry.body_spheres"
    WORKSPACE_BOUNDS = "geometry.workspace_bounds"
    URDF_MODEL = "model.urdf"
    BASE_FRAME = "frame.base"
    POSITION_LIMITS = "limits.position"
    VELOCITY_LIMITS = "limits.velocity"
    GRIPPER_MAPPING = "gripper.mapping"
    GRIPPER_GEOMETRY = "gripper.geometry"
    CAMERA_RGB = "camera.rgb"
    CAMERA_INTRINSICS = "camera.intrinsics"
    SEND_GRANT = "grant.send"
    HOLD_GRANT = "grant.hold"
    RESUME_GRANT = "grant.resume"
    HOME_GRANT = "grant.home"
    ESTOP_GRANT = "grant.estop"


@dataclass(frozen=True)
class SupportRow:
    """Immutable support facts for one named robot part or camera."""

    scope: str
    embodiment_digest: str
    facts: tuple[SupportFact, ...]

    def __post_init__(self) -> None:
        if not (self.scope.startswith("robot:") or self.scope.startswith("camera:")):
            raise ValueError("support scope must start with 'robot:' or 'camera:'")
        if not self.scope.split(":", 1)[1]:
            raise ValueError("support scope must name a robot part or camera")
        _validate_sha256_digest(self.embodiment_digest)
        facts = tuple(
            sorted(
                {SupportFact(fact) for fact in self.facts},
                key=lambda fact: fact.value,
            )
        )
        object.__setattr__(self, "facts", facts)

    def as_dict(self) -> dict[str, JSONValue]:
        return {
            "scope": self.scope,
            "embodimentDigest": self.embodiment_digest,
            "facts": [fact.value for fact in self.facts],
        }


@dataclass(frozen=True)
class SupportMatrix:
    """The SDK support envelope for one opened hardware session.

    ``action_space`` and ``grants`` are immutable snapshots of the exact
    declaration registered with the native core. ``rows`` only report facts;
    they cannot widen either the action space or its permissions.
    """

    contract_version: str
    embodiment_digest: str
    action_space: Mapping[str, object]
    grants: tuple[Mapping[str, object], ...]
    rows: tuple[SupportRow, ...]

    def __post_init__(self) -> None:
        if self.contract_version != SUPPORT_CONTRACT_VERSION:
            raise ValueError("unsupported SDK support contract version")
        _validate_sha256_digest(self.embodiment_digest)
        action_space = _freeze_json(self.action_space)
        if not isinstance(action_space, Mapping):
            raise TypeError("action_space must be a JSON object")
        grants = tuple(_freeze_json(grant) for grant in self.grants)
        if not all(isinstance(grant, Mapping) for grant in grants):
            raise TypeError("every grant must be a JSON object")
        rows = tuple(self.rows)
        if len({row.scope for row in rows}) != len(rows):
            raise ValueError("support row scopes must be unique")
        object.__setattr__(self, "action_space", action_space)
        object.__setattr__(self, "grants", grants)
        object.__setattr__(self, "rows", rows)

    def as_dict(self) -> dict[str, JSONValue]:
        return {
            "contractVersion": self.contract_version,
            "embodimentDigest": self.embodiment_digest,
            "actionSpace": _thaw_json(self.action_space),
            "grants": [_thaw_json(grant) for grant in self.grants],
            "rows": [row.as_dict() for row in self.rows],
        }


@dataclass(frozen=True)
class Pose:
    """A frame-tagged Cartesian pose using the SDK's pinned wxyz order."""

    position_m: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]
    frame_id: str

    def __post_init__(self) -> None:
        position = tuple(float(value) for value in self.position_m)
        quaternion = tuple(float(value) for value in self.quaternion_wxyz)
        if len(position) != 3 or not all(np.isfinite(position)):
            raise ValueError("position_m must contain three finite values")
        if len(quaternion) != 4 or not all(np.isfinite(quaternion)):
            raise ValueError("quaternion_wxyz must contain four finite values")
        if not self.frame_id:
            raise ValueError("Pose must declare frame_id")
        norm = float(np.linalg.norm(quaternion))
        if not np.isclose(norm, 1.0, rtol=1e-6, atol=1e-8):
            raise ValueError("quaternion_wxyz must be a unit quaternion")
        object.__setattr__(self, "position_m", position)
        object.__setattr__(self, "quaternion_wxyz", quaternion)


@dataclass(frozen=True)
class BodySphere:
    """One conservative named body sphere in one declared frame."""

    name: str
    center_m: tuple[float, float, float]
    radius_m: float
    frame_id: str

    def __post_init__(self) -> None:
        center = tuple(float(value) for value in self.center_m)
        radius = float(self.radius_m)
        if not self.name:
            raise ValueError("BodySphere must declare name")
        if len(center) != 3 or not all(np.isfinite(center)):
            raise ValueError("center_m must contain three finite values")
        if not np.isfinite(radius) or radius <= 0.0:
            raise ValueError("radius_m must be finite and positive")
        if not self.frame_id:
            raise ValueError("BodySphere must declare frame_id")
        object.__setattr__(self, "center_m", center)
        object.__setattr__(self, "radius_m", radius)


@dataclass(frozen=True)
class JointPositionCommand:
    """One joint-position action plus an optional known velocity hint.

    ``velocity_feedforward_rad_s`` is actuator feedforward, not a second
    motion target.  A trajectory producer may set it only when it knows the
    commanded path's velocity; SDK drivers that do not implement the optional
    velocity-aware extension execute ``positions`` normally.  In particular,
    nobody differentiates measured positions or an IK stream to invent this
    value.
    """

    positions: tuple[float, ...]
    velocity_feedforward_rad_s: tuple[float, ...] | None = None

    def __init__(
        self,
        positions: Sequence[float] | npt.NDArray[np.float64],
        velocity_feedforward_rad_s: (
            Sequence[float] | npt.NDArray[np.float64] | None
        ) = None,
    ) -> None:
        position_values = tuple(float(value) for value in positions)
        if not position_values or not all(np.isfinite(position_values)):
            raise ValueError("positions must be a non-empty finite joint vector")
        velocity_values = (
            None
            if velocity_feedforward_rad_s is None
            else tuple(float(value) for value in velocity_feedforward_rad_s)
        )
        if velocity_values is not None:
            if len(velocity_values) != len(position_values):
                raise ValueError(
                    "velocity_feedforward_rad_s must have the same width as positions"
                )
            if not all(np.isfinite(velocity_values)):
                raise ValueError("velocity_feedforward_rad_s must contain finite values")
        object.__setattr__(self, "positions", position_values)
        object.__setattr__(self, "velocity_feedforward_rad_s", velocity_values)


if TYPE_CHECKING:
    Action: TypeAlias = (
        Sequence[float] | npt.NDArray[np.float64] | JointPositionCommand
    )
else:
    # Keep clean imports compatible with numpy's minimal runtime surface.
    # Protocol annotations are postponed; only static consumers need the
    # expanded union.
    Action: TypeAlias = object


class FaultCode(str, enum.Enum):
    INVALID_REQUEST = "invalid_request"
    BUSY = "busy"
    NOT_OPEN = "not_open"
    NOT_FOUND = "not_found"
    CONFLICT = "conflict"
    UNSUPPORTED = "unsupported"
    SAFETY_REFUSAL = "safety_refusal"
    TRANSPORT_LOST = "transport_lost"
    INTERNAL = "internal"


@dataclass(frozen=True)
class RuntimeFaultCause:
    """One structured lower-level cause safe to carry across SDK boundaries.

    Implementations must not place credentials, customer paths, or raw vendor
    exception text in any field.
    """

    code: str
    detail: str
    context: Mapping[str, JSONValue] = field(default_factory=dict)
    causes: tuple[RuntimeFaultCause, ...] = ()

    def as_dict(self) -> dict[str, JSONValue]:
        return {
            "code": self.code,
            "detail": self.detail,
            "context": dict(self.context),
            "causes": [cause.as_dict() for cause in self.causes],
        }


@dataclass
class RuntimeFault(Exception):
    """A concise public runtime failure consumable by Metal.

    ``detail``, ``context``, and ``causes`` cross process and tenant-aware
    logging boundaries. Implementations must keep them free of credentials,
    customer paths, and arbitrary vendor exception strings.
    """

    code: FaultCode
    detail: str
    retryable: bool = False
    context: Mapping[str, JSONValue] = field(default_factory=dict)
    causes: tuple[RuntimeFaultCause, ...] = ()

    def __str__(self) -> str:
        return f"{self.code.value}: {self.detail}"

    def as_dict(self) -> dict[str, JSONValue]:
        """Return the complete transport-safe fault without flattening causes."""
        return {
            "code": self.code.value,
            "detail": self.detail,
            "retryable": self.retryable,
            "context": dict(self.context),
            "causes": [cause.as_dict() for cause in self.causes],
        }


@dataclass(frozen=True)
class RuntimeEvent:
    cursor: int
    kind: str
    session_ns: int
    data: Mapping[str, JSONValue] = field(default_factory=dict)


@dataclass(frozen=True, eq=False)
class PartObservation:
    joint_position: npt.NDArray[np.float64]
    joint_velocity: npt.NDArray[np.float64]
    ee_pose_wxyz: npt.NDArray[np.float64] | None = None
    frame_id: str | None = None


@dataclass(frozen=True, eq=False)
class Observation:
    session_ns: int
    unix_ns: int
    parts: Mapping[str, PartObservation]
    cameras: Mapping[str, CameraSample]

    def gate_vector(self) -> npt.NDArray[np.float64]:
        """Flatten joint positions in declaration order for the native gate."""
        if not self.parts:
            return np.empty(0, dtype=np.float64)
        return np.concatenate(
            [part.joint_position for part in self.parts.values()], dtype=np.float64
        )


@dataclass(frozen=True)
class SubmitResult:
    dispatched: bool
    gate: str
    part: str | None = None
    detail: str = ""


@runtime_checkable
class RunPort(Protocol):
    @property
    def id(self) -> str: ...

    def observe(self) -> Observation: ...

    def step(
        self,
        action: Action,
        observation: Observation | Sequence[float] | npt.NDArray[np.float64] | None = None,
    ) -> SubmitResult: ...

    def hold(self, reason: str) -> None: ...


@runtime_checkable
class SdkRuntimePort(Protocol):
    """The sole surface Metal needs from a local or remote SDK session."""

    def describe(self) -> Mapping[str, JSONValue]: ...

    def begin_run(
        self,
        *,
        task: str | Mapping[str, JSONValue],
        actor: str | Mapping[str, JSONValue],
    ) -> RunPort: ...

    def observe(self) -> Observation: ...

    def submit(
        self,
        action: Action,
        observation: Observation | Sequence[float] | npt.NDArray[np.float64] | None = None,
    ) -> SubmitResult: ...

    def hold(self, reason: str) -> None: ...

    def estop(self, reason: str) -> None: ...

    def events(self, after_cursor: int = 0) -> tuple[RuntimeEvent, ...]: ...

    def calibration_measurement(
        self,
        *,
        calibration_id: str,
        sample_id: str,
        camera: str,
        frame_sequence: int,
        x: int,
        y: int,
    ) -> Mapping[str, JSONValue]: ...


@runtime_checkable
class SdkSupportPort(Protocol):
    """Optional SDK facet exposing a conservative support matrix."""

    def support(self) -> SupportMatrix: ...


@runtime_checkable
class SdkKinematicsPort(Protocol):
    """Optional SDK facet for hardware-specific forward kinematics."""

    def forward_kinematics(
        self, part: str, joint_position: Sequence[float] | npt.NDArray[np.float64]
    ) -> Pose: ...


@runtime_checkable
class SdkGeometryPort(Protocol):
    """Optional SDK facet for configuration-dependent conservative geometry."""

    def body_geometry(
        self, part: str, joint_position: Sequence[float] | npt.NDArray[np.float64]
    ) -> tuple[BodySphere, ...]: ...


__all__ = [
    "Action",
    "BodySphere",
    "FaultCode",
    "JointPositionCommand",
    "JSONValue",
    "Observation",
    "PartObservation",
    "Pose",
    "RunPort",
    "SdkGeometryPort",
    "SdkKinematicsPort",
    "RuntimeEvent",
    "RuntimeFault",
    "RuntimeFaultCause",
    "SdkRuntimePort",
    "SdkSupportPort",
    "SubmitResult",
    "SUPPORT_CONTRACT_VERSION",
    "SupportFact",
    "SupportMatrix",
    "SupportRow",
]
