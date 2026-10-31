"""Typed contracts shared by local and remote SDK runtime adapters.

The protocol is deliberately structural: applications can depend on this public
module without importing a transport implementation or any SDK internals.
Concrete authority, timing, gating, and recording remain native-core owned.
"""

from __future__ import annotations

import enum
import math
import re
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import TYPE_CHECKING, Protocol, TypeAlias, runtime_checkable

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
    """One SDK implementation or declaration fact used by applications.

    These are support facts, not robot skill capabilities. A fact can satisfy
    one prerequisite of an application capability, but it never grants permission or
    says that a complete skill is available.
    """

    JOINT_POSITION_OBSERVATION = "observation.joint_position"
    JOINT_VELOCITY_OBSERVATION = "observation.joint_velocity"
    EE_POSE_OBSERVATION = "observation.ee_pose"
    JOINT_POSITION_ACTION = "action.joint_position"
    PART_OBSERVATION = "observation.named_parts"
    PART_ACTION = "action.named_parts"
    COMPLETED_STOP = "control.completed_stop"
    FINITE_JOINT_PLAYBACK = "execution.finite_joint_playback"
    COORDINATED_JOINT_PLAYBACK = "execution.coordinated_joint_playback"
    COORDINATED_SUBSET_PLAYBACK = "execution.coordinated_subset_playback"
    PARALLEL_JOINT_PLAYBACK = "execution.parallel_joint_playback"
    WATCHDOG_PLAYBACK = "execution.watchdog_playback"
    STREAMED_JOINT_PLAYBACK = "execution.streamed_joint_playback"
    PART_HOLD_PLAYBACK = "execution.part_hold_playback"
    JOINT_PLAYBACK_BOUNDS = "execution.joint_playback_bounds"
    PACED_JOINT_PLAYBACK = "execution.paced_joint_playback"
    FEEDBACK_JOINT_PLAYBACK = "execution.feedback_joint_playback"
    SAMPLED_JOINT_PLAYBACK = "execution.sampled_joint_playback"
    REFERENCE_JOINT_PLAYBACK = "execution.reference_joint_playback"
    CHECKPOINTED_JOINT_PLAYBACK = "execution.checkpointed_joint_playback"
    REVISABLE_JOINT_PLAYBACK = "execution.revisable_joint_playback"
    PLAYBACK_FEEDBACK_REFERENCE = "execution.playback_feedback_reference"
    WITHDRAWABLE_JOINT_PLAYBACK = "execution.withdrawable_joint_playback"
    PACED_PLAYBACK_DEADLINE = "execution.paced_playback_deadline"
    GRIPPER_PLAYBACK = "execution.gripper_playback"
    VELOCITY_FEEDFORWARD = "actuation.velocity_feedforward"
    FORWARD_KINEMATICS = "kinematics.fk"
    BODY_SPHERES = "geometry.body_spheres"
    WORKSPACE_BOUNDS = "geometry.workspace_bounds"
    URDF_MODEL = "model.urdf"
    BASE_FRAME = "frame.base"
    POSITION_LIMITS = "limits.position"
    POSITION_ERROR_LIMITS = "limits.position_error"
    VELOCITY_LIMITS = "limits.velocity"
    GRIPPER_FORCE = "actuation.gripper_force"
    GRIPPER_FORCE_OBSERVATION = "observation.gripper_force"
    GRIPPER_MAPPING = "gripper.mapping"
    GRIPPER_GEOMETRY = "gripper.geometry"
    CAMERA_RGB = "camera.rgb"
    CAMERA_INTRINSICS = "camera.intrinsics"
    CAMERA_CONTENT_TIMING = "camera.content_timing"
    CAMERA_CAPTURE_STATE = "camera.capture_state"
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

    ``gripper_force_n`` is an optional named-part actuation mode. A positive
    value activates supported driver force control, zero releases it, and None
    preserves the current mode during an arm write. Position references bound
    the approach; on acquisition the driver maintains force within physical
    jaw/speed limits. The native gate and owner envelope admit every reference
    before activation. Substituted supervision actions clear the caller mode.
    """

    positions: tuple[float, ...]
    velocity_feedforward_rad_s: tuple[float, ...] | None = None
    gripper_force_n: float | None = None

    def __init__(
        self,
        positions: Sequence[float] | npt.NDArray[np.float64],
        velocity_feedforward_rad_s: (
            Sequence[float] | npt.NDArray[np.float64] | None
        ) = None,
        *,
        gripper_force_n: float | None = None,
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
                raise ValueError(
                    "velocity_feedforward_rad_s must contain finite values"
                )
        if gripper_force_n is not None:
            if isinstance(gripper_force_n, bool):
                raise ValueError("gripper_force_n must be a finite number, not boolean")
            gripper_force_n = float(gripper_force_n)
            if not math.isfinite(gripper_force_n) or gripper_force_n < 0:
                raise ValueError("gripper_force_n must be finite and non-negative")
        object.__setattr__(self, "gripper_force_n", gripper_force_n)
        object.__setattr__(self, "positions", position_values)
        object.__setattr__(self, "velocity_feedforward_rad_s", velocity_values)


if TYPE_CHECKING:
    Action: TypeAlias = Sequence[float] | npt.NDArray[np.float64] | JointPositionCommand
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
    MOTOR_FAILURE = "motor_failure"
    INTERNAL = "internal"


_CREDENTIAL_FIELD = re.compile(
    r"(?:.*[_.-])?(?:authorization|api[_-]?key|access[_-]?token|refresh[_-]?token|"
    r"token|password|passwd|secret|credentials?)$",
    re.IGNORECASE,
)
_CREDENTIAL_TEXT = re.compile(
    r"(\b(?:authorization|api[_-]?key|access[_-]?token|refresh[_-]?token|"
    r"token|password|passwd|secret|credentials?)\s*[:=]\s*)"
    r"(?:Bearer\s+[^\s,;]+|\"[^\"]*\"|'[^']*'|[^\s,;]+)",
    re.IGNORECASE,
)
_BEARER_TEXT = re.compile(r"(\bBearer\s+)[^\s,;]+", re.IGNORECASE)


def _diagnostic_text(value: str) -> str:
    value = _CREDENTIAL_TEXT.sub(r"\1[REDACTED]", value)
    return _BEARER_TEXT.sub(r"\1[REDACTED]", value)


def _diagnostic_json(value: object, depth: int = 0) -> JSONValue:
    """Copy bounded JSON metadata without serializing arbitrary Python objects."""
    if depth > 8:
        raise ValueError("metadata nesting exceeds eight levels")
    if value is None or isinstance(value, (bool, int)):
        return value
    if isinstance(value, str):
        return _diagnostic_text(value)
    if isinstance(value, float) and math.isfinite(value):
        return value
    if isinstance(value, Mapping) and len(value) <= 128:
        result = {}
        for key, item in value.items():
            if not isinstance(key, str):
                raise TypeError("metadata keys must be strings")
            result[key] = (
                "[REDACTED]"
                if _CREDENTIAL_FIELD.fullmatch(key)
                else _diagnostic_json(item, depth + 1)
            )
        return result
    if isinstance(value, (list, tuple)) and len(value) <= 128:
        return [_diagnostic_json(item, depth + 1) for item in value]
    raise ValueError("metadata must contain bounded JSON values")


def _exception_cause(
    error: BaseException, seen: tuple[int, ...] = ()
) -> RuntimeFaultCause:
    if isinstance(error, RuntimeFault):
        return RuntimeFaultCause(
            error.code.value, error.detail, error.context, error.causes
        )
    if id(error) in seen or len(seen) >= 8:
        return RuntimeFaultCause(
            "cause_limit", "Cause chain is cyclic or exceeds eight levels"
        )
    error_type = f"{type(error).__module__}.{type(error).__qualname__}"
    context: dict[str, JSONValue] = {"error_type": error_type}
    omitted = []
    for index, (key, value) in enumerate(vars(error).items()):
        if index >= 128:
            context["metadata_limit"] = "Exception metadata exceeds 128 attributes"
            break
        try:
            context.update(_diagnostic_json({key: value}))
        except (TypeError, ValueError):
            omitted.append(key)
    if isinstance(error, OSError):
        for key in ("errno", "filename", "filename2"):
            if (value := getattr(error, key, None)) is not None:
                context[key] = _diagnostic_json(value)
    if omitted:
        context["omitted_non_json_fields"] = omitted
    parent = error.__cause__
    if parent is None and not error.__suppress_context__:
        parent = error.__context__
    native_code = getattr(error, "code", error_type)
    if isinstance(native_code, enum.Enum):
        native_code = native_code.value
    return RuntimeFaultCause(
        code=str(native_code) if isinstance(native_code, (str, int)) else error_type,
        detail=_diagnostic_text(str(error)),
        context=context,
        causes=()
        if parent is None
        else (_exception_cause(parent, (*seen, id(error))),),
    )


@dataclass(frozen=True)
class RuntimeFaultCause:
    """A lower-level failure retaining its original diagnostic and metadata.

    Credentials must not be included; benign device paths and vendor messages
    are diagnostic information and must not be replaced with generic summaries.
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
    """A public runtime failure consumable by applications without information loss.

    ``detail``, ``context``, and ``causes`` retain scoped device diagnostics
    across process boundaries. Typed producers keep credentials out of those
    fields; ``from_exception`` removes explicitly labelled credentials from
    otherwise unchanged lower-level text and JSON metadata.
    """

    code: FaultCode
    detail: str
    retryable: bool = False
    context: Mapping[str, JSONValue] = field(default_factory=dict)
    causes: tuple[RuntimeFaultCause, ...] = ()

    @classmethod
    def from_exception(
        cls,
        error: Exception,
        *,
        code: FaultCode = FaultCode.INTERNAL,
        context: Mapping[str, JSONValue] | None = None,
    ) -> RuntimeFault:
        """Preserve a lower-level failure, redacting only credential material.

        Existing RuntimeFault objects pass through unchanged. Other exceptions
        retain their message, type, JSON attributes, errno and Python cause
        chain. Non-JSON metadata fields are named rather than repr-serialized;
        metadata is bounded to eight levels and 128 entries per container.
        Credential-named fields/assignments and Bearer values are redacted.
        """
        if isinstance(error, cls):
            return error
        cause = _exception_cause(error)
        return cls(
            code,
            cause.detail,
            retryable=isinstance(error, TimeoutError),
            context=_diagnostic_json(dict(context or {})),
            causes=(cause,),
        )

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


@dataclass(frozen=True)
class GripperForce:
    """Closing force in N; estimates must identify their conversion source."""

    force_n: float
    source: str
    estimated: bool = True

    def __post_init__(self):
        if not math.isfinite(self.force_n) or self.force_n < 0 or not self.source:
            raise ValueError("gripper force must be finite, non-negative and sourced")


@dataclass(frozen=True, eq=False)
class PartObservation:
    joint_position: npt.NDArray[np.float64]
    joint_velocity: npt.NDArray[np.float64]
    ee_pose_wxyz: npt.NDArray[np.float64] | None = None
    frame_id: str | None = None
    gripper_force: GripperForce | None = None


@dataclass(frozen=True, eq=False)
class Observation:
    session_ns: int
    unix_ns: int
    parts: Mapping[str, PartObservation]
    cameras: Mapping[str, CameraSample]
    faults: Mapping[str, RuntimeFault] = field(default_factory=dict)

    def gate_vector(self) -> npt.NDArray[np.float64]:
        """Flatten joint positions in declaration order for the native gate."""
        if not self.parts:
            return np.empty(0, dtype=np.float64)
        return np.concatenate(
            [part.joint_position for part in self.parts.values()], dtype=np.float64
        )


@dataclass(frozen=True)
class SubmitResult:
    """Dispatch receipt; an owner refusal retains its exact structured fault."""

    dispatched: bool
    gate: str
    part: str | None = None
    detail: str = ""
    fault: RuntimeFault | None = None
    dispatch_started_monotonic_s: float | None = None
    dispatch_completed_monotonic_s: float | None = None


@dataclass(frozen=True)
class JointReference:
    """One exact position reference in declared joint units; held until the next."""

    offset_ns: int
    command: JointPositionCommand


@dataclass(frozen=True)
class PacedJointCurve:
    """An already checked polynomial, evaluated locally without path fitting.

    Coefficients use normalized segment time, in segment/power/coordinate order,
    with six powers (zero through five). Coordinates match policy ``indices``.
    Progress advances only on acknowledged submissions, within measured position
    headroom and a minimum local command period. Optional exact sample times
    retain waypoints on rest-to-rest quintics. This performs no planning.
    """

    times_s: tuple[float, ...]
    positions: tuple[tuple[float, ...], ...]
    coefficients: tuple[tuple[tuple[float, ...], ...], ...]
    period_ns: int
    position_error: tuple[float, ...]
    tracking_tolerance: tuple[float, ...]
    sample_times_s: tuple[float, ...] | None = None

    def __post_init__(self) -> None:
        for name in ("times_s", "position_error", "tracking_tolerance"):
            object.__setattr__(self, name, tuple(getattr(self, name)))
        object.__setattr__(
            self, "positions", tuple(tuple(row) for row in self.positions)
        )
        object.__setattr__(
            self,
            "coefficients",
            tuple(
                tuple(tuple(row) for row in segment) for segment in self.coefficients
            ),
        )
        if self.sample_times_s is not None:
            object.__setattr__(self, "sample_times_s", tuple(self.sample_times_s))


@dataclass(frozen=True)
class HeldJointGroup:
    """Hold unselected rows toward their unchanged target with measured headroom.

    ``position_error`` is the per-row cap before 90% headroom. Opposing measured
    velocity reserves another ``period_ns`` of travel; all rows scale together.
    The ordinary owner envelope still decides whether each write is accepted.
    """

    indices: tuple[int, ...]
    position_error: tuple[float, ...]
    period_ns: int

    def __post_init__(self) -> None:
        object.__setattr__(self, "indices", tuple(self.indices))
        object.__setattr__(self, "position_error", tuple(self.position_error))


@dataclass(frozen=True)
class PositionWindow:
    """Measure encoder drift over ``PlaybackPolicy.settle_ns``.

    At least three fresh samples must span the window. Velocity remains
    diagnostic. A declared gap resets history; ``None`` adds no gap limit.
    ``during_motion`` allows eligible pre-endpoint history, but completion still
    requires the final acknowledgement and fresh feedback. ``consecutive_samples``
    counts qualifying windows after that acknowledgement.
    """

    max_drift: tuple[float, ...]
    max_gap_ns: int | None = None
    during_motion: bool = False
    consecutive_samples: int = 1

    def __post_init__(self) -> None:
        object.__setattr__(self, "max_drift", tuple(self.max_drift))


@dataclass(frozen=True)
class PlaybackFeedbackPolicy:
    """Optional local reference adaptation; never a planner or force controller.

    Held groups and measured rows must be disjoint from each other and from the
    moving rows. ``measured_indices`` copies feedback, preserving an already
    active local jaw mode without asserting grasp/contact. Velocity hints use
    ``curve`` unchanged, ``toward_reference`` only toward current position error,
    or ``toward_goal`` with a per-row stop latch after crossing the final goal.
    """

    held_groups: tuple[HeldJointGroup, ...] = ()
    measured_indices: tuple[int, ...] = ()
    velocity_hints: str = "curve"
    position_window: PositionWindow | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "held_groups", tuple(self.held_groups))
        object.__setattr__(self, "measured_indices", tuple(self.measured_indices))


@dataclass(frozen=True)
class GripperContactPolicy:
    """Explicit closing progress, demand and sustained stall requirements."""

    minimum_opening_m: float
    minimum_progress_m: float
    target_shortfall_m: float
    commanded_shortfall_m: float
    max_drift_m: float
    window_ns: int


@dataclass(frozen=True)
class GripperForcePolicy:
    """Positive force needs three stationary samples above target minus tolerance.

    The closed-stop exclusion wins even if motor effort meets the target. Zero
    explicitly releases an existing force mode and uses positional/contact
    completion. Force estimates retain their original source and provenance.
    """

    target_n: float
    tolerance_n: float
    closed_tolerance_m: float


@dataclass(frozen=True)
class GripperPlaybackPolicy:
    """Local jaw execution against the Site's exact declared gripper mapping.

    The parent selects only the jaw row. Nominal references advance once per
    acknowledgement, with measured step limits and no wall-clock catch-up.
    Positive force pursues the final target immediately through the same envelope.
    This is separate from checked polynomial playback for arm coordinates.
    """

    period_ns: int
    position_error: float
    tracking_tolerance: float
    closed_position: float
    open_position: float
    closed_m: float
    open_m: float
    contact: GripperContactPolicy | None = None
    force: GripperForcePolicy | None = None


@dataclass(frozen=True)
class PlaybackCurveUpdate:
    """A checked future suffix for a revisable, reference-only paced execution.

    ``switch_ns`` uses the original execution's reference phase, not wall time.
    ``curve`` starts at zero and must preserve position/velocity/acceleration at
    that switch. Limits, coordinate order, held rows and the original timeout
    cannot change. Only one future suffix may await its acknowledged handoff.
    Stale revisions and discontinuities refuse without disturbing the incumbent.
    """

    expected_revision: int
    switch_ns: int
    curve: PacedJointCurve


@dataclass(frozen=True)
class PlaybackWatchdog:
    """Local caller-liveness bound anchored to acquired SDK feedback time.

    ``observed_ns`` comes from this session's observation clock. The native
    worker expires at that stamp plus ``timeout_ns``; delayed request arrival
    cannot renew liveness or revive an expired execution. This does not change
    the motion deadline, authorize commands or preempt a blocked driver callback.
    """

    observed_ns: int
    timeout_ns: int


@dataclass(frozen=True)
class PlaybackPolicy:
    """Explicit measured completion and local timing bounds; validated by core.

    ``minimum_settle_samples`` requires fresh eligible observations in addition
    to the settling span, with separately declared sampled-playback support.
    Velocity/goal loss resets the count. Position windows retain at least this
    many samples (and never fewer than three). Gripper policies keep their own
    outcome-specific sample rules and require the default value here.

    ``reference_only`` uses separately advertised reference-playback support to
    report ``references_dispatched`` after all acknowledgements and fresh local
    feedback. It does not measure arrival/stationarity and excludes gripper
    outcome policies. Snapshots retain ``last_acknowledged_ns`` on the session
    clock; remote receipt time must not be substituted for it.

    ``checkpoint_offsets_ns`` requests up to 256 strictly increasing diagnostic
    markers within the buffer, with checkpointed-playback support. Each retains
    the first fresh local feedback after acknowledged reference phase crosses
    its offset. These are not intermediate arrival gates or interpolated
    measurements. Gripper outcome policies cannot request checkpoints.

    ``allow_curve_updates`` requires revisable-playback support and reference-only
    paced execution without checkpoints, exact sample grids or goal-latched hints.
    Updates use the same local worker; they do not extend the original timeout.

    ``credit_pacing_delay`` requires separately advertised paced-deadline support
    and reference-only polynomial playback. Only a complete acknowledged moving
    tick credits its excess local wall time to the original budget. Exact
    constant tails and stalled phase cannot renew it. Snapshots retain the
    accumulated ``curve_progress.pacing_extension_ns``; cloud receipt time never
    contributes. Default false keeps the original absolute timeout. Updates and
    withdrawals preserve, but do not themselves increase, the accumulated credit.

    ``part_hold`` opts a supported named buffer into local per-part Hold on
    cancellation or failure. It leaves siblings running and retains the cleanup
    scope and owner error. Global control verbs and shutdown still drain all
    buffers. Default false uses site-wide cleanup. This is not collision proof.

    ``watchdog`` optionally requires caller liveness on the local session clock,
    under separately declared watchdog-playback support. Expiry uses the same
    failure/Hold path and retains its original error and cleanup receipt.

    ``timeout_ns=None`` explicitly selects a watchdog-limited reference stream
    under streamed-playback support. It requires revisable reference-only curves
    and a watchdog, without pacing credit. Each finite curve ends at zero velocity
    and acceleration; its acknowledged resting tail remains supervised until a
    continuation, cancellation or watchdog expiry. Finite integer timeouts retain
    their original completion and deadline semantics.
    """

    indices: tuple[int, ...]
    start_tolerance: tuple[float, ...]
    position_tolerance: tuple[float, ...]
    stationary_velocity: tuple[float, ...]
    settle_ns: int
    max_lateness_ns: int
    timeout_ns: int | None
    allow_settled: bool = False
    paced_curve: PacedJointCurve | None = None
    feedback: PlaybackFeedbackPolicy | None = None
    gripper: GripperPlaybackPolicy | None = None
    minimum_settle_samples: int = 1
    reference_only: bool = False
    checkpoint_offsets_ns: tuple[int, ...] = ()
    allow_curve_updates: bool = False
    credit_pacing_delay: bool = False
    part_hold: bool = False
    watchdog: PlaybackWatchdog | None = None

    def __post_init__(self) -> None:
        for name in (
            "indices",
            "start_tolerance",
            "position_tolerance",
            "stationary_velocity",
            "checkpoint_offsets_ns",
        ):
            object.__setattr__(self, name, tuple(getattr(self, name)))


@dataclass(frozen=True)
class FiniteJointMotion:
    """An immutable, complete buffer for one named part, never a planner request.

    Offsets use the device session's local start, not a remote clock. Bounds,
    scope, measured start and each reference remain subject to native admission
    and the ordinary gate/owner envelope. Set a constant force request through
    the optional gripper policy, not individual nominal references. An existing
    held-jaw mode is preserved on ordinary arm-only playback.
    """

    part: str
    references: tuple[JointReference, ...]
    policy: PlaybackPolicy

    def __post_init__(self) -> None:
        object.__setattr__(self, "references", tuple(self.references))


@dataclass(frozen=True)
class CoordinatedJointMotion:
    """One finite buffer for declared parts, under one local clock.

    ``parts`` must contain at least two distinct parts in declaration order.
    A subset requires ``execution.coordinated_subset_playback`` on its members;
    older devices require the complete Site layout. Reference
    positions, velocity hints, curve rows and policy indices use that flattened
    joint layout. Every part must advertise coordinated playback and all selected
    policy support. Each group reference passes the existing native gate and
    all addressed owner envelopes before any write; driver writes themselves are
    sequential, not physically atomic. Failure/cancel uses the normal site Hold.
    Scalar gripper outcome/force policies are not supported for a group.
    """

    parts: tuple[str, ...]
    references: tuple[JointReference, ...]
    policy: PlaybackPolicy

    def __post_init__(self) -> None:
        object.__setattr__(self, "parts", tuple(self.parts))
        object.__setattr__(self, "references", tuple(self.references))


@runtime_checkable
class PlaybackHandlePort(Protocol):
    """Retained execution identity and measured status; terminal means drained.

    Snapshots contain id, episode_id, dispatched, references, feedback, status
    (running/arrived/settled/contact_stall/force_reached/failed), and original
    failure evidence. Optional cleanup retains the completed Hold/e-stop callback
    and its original error separately. A terminal reference loop or successful
    callback alone does not prove that a safety verb stopped hardware.
    """

    def snapshot(self) -> Mapping[str, JSONValue]: ...

    def wait(self) -> Mapping[str, JSONValue]: ...

    def cancel(self, reason: str) -> None: ...


@runtime_checkable
class WatchdogPlaybackHandlePort(PlaybackHandlePort, Protocol):
    """Renew optional local liveness using an acquired SDK observation stamp.

    Queueing returns promptly; ``watchdog_refresh`` retains pending/accepted/
    rejected evidence. Only acceptance changes ``watchdog.deadline_ns``. Repeated
    stamps cannot extend it, and rejected stamps leave the incumbent unchanged.
    Expiry is checked before queued renewals, so delayed requests cannot revive
    execution. Neither the timeout interval nor motion deadline can change.
    """

    def refresh_watchdog(self, observed_ns: int) -> None: ...


@runtime_checkable
class RevisablePlaybackHandlePort(PlaybackHandlePort, Protocol):
    """Optional future updates, separate from ordinary finite playback support.

    ``update_curve`` returns a request ID without waiting for device I/O. The
    latest ``curve_update`` snapshot receipt is pending/accepted/rejected; accepted
    means scheduled, not executed. ``curve_progress.revision`` advances only at
    an acknowledged handoff. Serialize requests and reconcile uncertain replies
    from this same retained handle; never replay an update after a lost response.
    """

    def update_curve(self, update: PlaybackCurveUpdate) -> int: ...


@runtime_checkable
class WithdrawablePlaybackHandlePort(RevisablePlaybackHandlePort, Protocol):
    """Optional withdrawal of an accepted future suffix, before its handoff.

    ``request_id`` identifies the original accepted update; ``expected_revision``
    binds the still-active incumbent. The returned ID identifies a new receipt
    in ``curve_update`` with ``withdraws_request_id``. Only its accepted result
    confirms removal. A late, wrong-ID or stale withdrawal refuses without
    changing the trajectory. Withdrawal does not stop motion or extend timeouts;
    the original incumbent continues. Lost replies never authorize replay.
    """

    def withdraw_curve(self, request_id: int, expected_revision: int) -> int: ...


@runtime_checkable
class FinitePlaybackRunPort(Protocol):
    """Optional finite execution, declared by a per-part support fact."""

    def start_playback(
        self, motion: FiniteJointMotion | CoordinatedJointMotion
    ) -> PlaybackHandlePort: ...


@dataclass(frozen=True)
class JointPositionBounds:
    """Inclusive bounds in one declared part's joint order and position units."""

    lower: tuple[float, ...]
    upper: tuple[float, ...]

    def __post_init__(self) -> None:
        object.__setattr__(self, "lower", tuple(self.lower))
        object.__setattr__(self, "upper", tuple(self.upper))


@runtime_checkable
class BoundedPlaybackRunPort(FinitePlaybackRunPort, Protocol):
    """Optional run-wide local bounds for playback feedback and references.

    Callers supply and serialize their own checked bounds. A successful setter
    acknowledges configuration, never movement or physical stopping. An uncertain
    reply leaves the installed bounds unknown; do not continue dependent motion.
    An empty mapping removes the bounds. Existing site limits remain authoritative.
    """

    def set_playback_bounds(
        self, bounds: Mapping[str, JointPositionBounds]
    ) -> None: ...


@runtime_checkable
class RunPort(Protocol):
    @property
    def id(self) -> str: ...

    def observe(self) -> Observation: ...

    def step(
        self,
        action: Action,
        observation: Observation
        | Sequence[float]
        | npt.NDArray[np.float64]
        | None = None,
    ) -> SubmitResult: ...

    def hold(self, reason: str) -> None: ...


@runtime_checkable
class NamedPartsRunPort(Protocol):
    """Optional independent submissions through the same supervised run.

    Results belong to individual parts, not an atomic multi-part transaction.
    A receipt records dispatch, never physical arrival. Global supervision and
    configured cross-part envelope dependencies can still refuse a submission.
    """

    def step_parts(
        self,
        commands: Mapping[str, JointPositionCommand],
        observation: Observation | None = None,
    ) -> Mapping[str, SubmitResult]: ...


@runtime_checkable
class NamedPartsObservationPort(Protocol):
    """Optional partial observations; missing measurements carry exact faults."""

    def observe_parts(self, parts: Sequence[str] | None = None) -> Observation: ...


@runtime_checkable
class MediaRuntimePort(Protocol):
    """Optional native publisher evidence, independent of motion support."""

    def media_tracks(self) -> list[dict[str, JSONValue]]: ...


@runtime_checkable
class SdkRuntimePort(Protocol):
    """The structural surface applications need from a local or remote SDK session."""

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
        observation: Observation
        | Sequence[float]
        | npt.NDArray[np.float64]
        | None = None,
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
class CompletedStopPort(Protocol):
    """Optional local stop completion, distinct from queue-only control verbs."""

    def stop(self, reason: str, *, estop: bool = False) -> Mapping[str, JSONValue]: ...


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
    "SUPPORT_CONTRACT_VERSION",
    "Action",
    "BodySphere",
    "FaultCode",
    "FiniteJointMotion",
    "FinitePlaybackRunPort",
    "GripperContactPolicy",
    "GripperForce",
    "GripperForcePolicy",
    "GripperPlaybackPolicy",
    "HeldJointGroup",
    "JSONValue",
    "JointPositionCommand",
    "JointReference",
    "MediaRuntimePort",
    "NamedPartsObservationPort",
    "NamedPartsRunPort",
    "Observation",
    "PacedJointCurve",
    "PartObservation",
    "PlaybackFeedbackPolicy",
    "PlaybackHandlePort",
    "PlaybackPolicy",
    "PlaybackWatchdog",
    "Pose",
    "PositionWindow",
    "RunPort",
    "RuntimeEvent",
    "RuntimeFault",
    "RuntimeFaultCause",
    "SdkGeometryPort",
    "SdkKinematicsPort",
    "SdkRuntimePort",
    "SdkSupportPort",
    "SubmitResult",
    "SupportFact",
    "SupportMatrix",
    "SupportRow",
    "WatchdogPlaybackHandlePort",
]
