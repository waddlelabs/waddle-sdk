"""Typed contracts shared by local and remote SDK runtime adapters.

The protocol is deliberately structural: Metal can depend on this public
module without importing a transport implementation or any SDK internals.
Concrete authority, timing, gating, and recording remain native-core owned.
"""

from __future__ import annotations

import enum
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Protocol, TypeAlias, runtime_checkable

import numpy as np
import numpy.typing as npt

from .cameras import CameraSample

JSONScalar: TypeAlias = str | int | float | bool | None
JSONValue: TypeAlias = JSONScalar | list["JSONValue"] | dict[str, "JSONValue"]


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


@dataclass
class RuntimeFault(Exception):
    code: FaultCode
    detail: str
    retryable: bool = False
    context: Mapping[str, JSONValue] = field(default_factory=dict)

    def __str__(self) -> str:
        return f"{self.code.value}: {self.detail}"


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


__all__ = [
    "Action",
    "FaultCode",
    "JointPositionCommand",
    "JSONValue",
    "Observation",
    "PartObservation",
    "RunPort",
    "RuntimeEvent",
    "RuntimeFault",
    "SdkRuntimePort",
    "SubmitResult",
]
