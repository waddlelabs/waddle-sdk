"""Descriptor sugar: pure-Python dataclasses that compile to canonical
proto3 JSON for ``waddle.v0`` messages (lowerCamelCase keys, int64 as
decimal strings, full prefixed enum names, proto3 defaults omitted).

These validate *shape* ("must declare"), never *behavior* — all semantic
validation happens in waddle-core when the JSON crosses the shim
(hollow-frontend rule).
"""

from __future__ import annotations

import abc
import base64
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path

__all__ = [
    "Camera",
    "Chunking",
    "Composite",
    "EEDelta",
    "FrameTransform",
    "Gripper",
    "Intrinsics",
    "Joint",
    "JointSpace",
    "Opaque",
    "Robot",
    "StreamPolicy",
    "TimeSeries",
    "Uplink",
]

# Shared by Camera and Uplink (a camera's uplink policy re-declares its own
# encoding, independent of the local-capture encoding).
_CAMERA_ENCODINGS = {"rgb8": "RGB8", "bgr8": "BGR8", "z16": "Z16", "jpeg": "JPEG", "h264": "H264"}


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
class Joint:
    """A named joint with optional limits (radians/SI, per the
    ``JointDescriptor`` proto comments: revolute joints in radians,
    prismatic in meters; rate limits rad/s or m/s; effort N*m or N).

    Use a bare string in ``JointSpace(joints=...)`` / ``Gripper.dexterous``
    for the names-only form (unchanged); use ``Joint(...)`` per-joint only
    where a limit needs declaring.
    """

    name: str
    min_position: float | None = None
    max_position: float | None = None
    max_velocity: float | None = None
    max_effort: float | None = None

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("Joint must declare a name")
        if (
            self.min_position is not None
            and self.max_position is not None
            and self.min_position > self.max_position
        ):
            raise ValueError(
                f"joint {self.name!r}: min_position ({self.min_position}) must be "
                f"<= max_position ({self.max_position})"
            )
        if self.max_velocity is not None and self.max_velocity < 0:
            raise ValueError(f"joint {self.name!r}: max_velocity must be >= 0")
        if self.max_effort is not None and self.max_effort < 0:
            raise ValueError(f"joint {self.name!r}: max_effort must be >= 0")

    def _compile(self) -> dict:
        out: dict = {"name": self.name}
        if self.min_position is not None:
            out["minPosition"] = float(self.min_position)
        if self.max_position is not None:
            out["maxPosition"] = float(self.max_position)
        if self.max_velocity is not None:
            out["maxVelocity"] = float(self.max_velocity)
        if self.max_effort is not None:
            out["maxEffort"] = float(self.max_effort)
        return out


def _compile_joint(item: str | Joint) -> dict:
    """Compile one ``joints=[...]`` entry: a bare name (names-only form,
    unchanged) or a ``Joint`` with limits."""
    if isinstance(item, Joint):
        return item._compile()
    return {"name": str(item)}


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

    @classmethod
    def dexterous(cls, joints: Sequence[str | Joint]) -> "Gripper":
        """A multi-joint hand. ``joints`` entries follow the same
        names-only-or-``Joint`` rule as ``JointSpace.joints``."""
        compiled = [_compile_joint(j) for j in joints]
        if not compiled:
            raise ValueError("Gripper.dexterous must declare at least one joint")
        return cls({"dexterous": {"joints": compiled}})

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
    """Absolute joint positions, radians (v0 pins radians). Each entry in
    ``joints`` is either a bare name (names-only form, unchanged) or a
    :class:`Joint` declaring per-joint limits."""

    joints: Sequence[str | Joint]
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
        return {"jointPosition": {"joints": [_compile_joint(j) for j in self.joints]}}

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
class Intrinsics:
    """Pinhole + distortion, ROS ``CameraInfo`` lineage. ``distortion_model``
    accepts short case-insensitive names (``"plumb_bob"``,
    ``"rational_polynomial"``, ``"kannala_brandt"``) or ``"unspecified"``
    (the default: no distortion model declared)."""

    fx: float
    fy: float
    cx: float
    cy: float
    distortion_model: str = "unspecified"
    distortion: tuple[float, ...] = ()
    depth_scale_mm: float | None = None

    def __post_init__(self) -> None:
        if self.depth_scale_mm is not None and self.depth_scale_mm <= 0:
            raise ValueError("depth_scale_mm must be > 0")

    def _compile(self) -> dict:
        out: dict = {
            "fx": float(self.fx),
            "fy": float(self.fy),
            "cx": float(self.cx),
            "cy": float(self.cy),
        }
        model = _enum_name(
            self.distortion_model,
            "DISTORTION_MODEL",
            {
                "unspecified": "UNSPECIFIED",
                "plumb_bob": "PLUMB_BOB",
                "rational_polynomial": "RATIONAL_POLYNOMIAL",
                "kannala_brandt": "KANNALA_BRANDT",
            },
            "distortion_model",
        )
        if model != "DISTORTION_MODEL_UNSPECIFIED":
            out["model"] = model
        if self.distortion:
            out["distortion"] = [float(d) for d in self.distortion]
        if self.depth_scale_mm is not None:
            out["depthScaleMm"] = float(self.depth_scale_mm)
        return out


@dataclass(frozen=True)
class Uplink:
    """The uplink half of a camera's :class:`StreamPolicy`: what leaves the
    site, as opposed to the local full-rate archive."""

    fps: float
    encoding: str
    max_kbps: int | None = None

    def __post_init__(self) -> None:
        if self.fps <= 0:
            raise ValueError("fps must be > 0")
        if self.max_kbps is not None and self.max_kbps <= 0:
            raise ValueError("max_kbps must be > 0")

    def _compile(self) -> dict:
        out: dict = {
            "fps": float(self.fps),
            "encoding": _enum_name(
                self.encoding, "CAMERA_ENCODING", _CAMERA_ENCODINGS, "encoding"
            ),
        }
        if self.max_kbps is not None:
            out["maxKbps"] = int(self.max_kbps)
        return out


@dataclass(frozen=True)
class StreamPolicy:
    """What persists locally vs. what flows uplink for a camera.
    ``local_full_rate`` keeps the full-rate archive on the local recorder
    regardless of ``uplink``; ``uplink`` is the video path (the media
    plane).

    ``still_fps`` declares the ONE bounded exception to "nothing
    high-bandwidth touches the control plane" (protocol flag
    ``waddle.v0.obs.stills``): published frames are sampled at this rate
    into low-rate JPEG stills that ride the control plane, so a
    Waddle-hosted agent (a hosted Metal run) can see the scene without a
    media plane wired at all. It is bounded by declaration and is never a
    video path — for live video to a human teleoperator, declare
    ``uplink`` and wire ``media=`` instead. ``None`` (the default) and
    ``0`` both mean no stills, matching the wire ("0/absent means no
    stills")."""

    local_full_rate: bool = False
    uplink: Uplink | None = None
    still_fps: float | None = None

    def __post_init__(self) -> None:
        if self.still_fps is not None and self.still_fps < 0:
            raise ValueError("still_fps must be >= 0 (0 or None = no stills)")

    def _compile(self) -> dict:
        out: dict = {}
        if self.local_full_rate:
            out["localFullRate"] = True
        if self.uplink is not None:
            out["uplink"] = self.uplink._compile()
        if self.still_fps is not None:
            # `optional double` (explicit presence): a declared 0.0 is sent
            # as 0.0, not omitted — same meaning either way.
            out["stillFps"] = float(self.still_fps)
        return out


@dataclass(frozen=True)
class Camera:
    """Declaration-only camera description (no capture tap in v1)."""

    width: int
    height: int
    fps: float
    encoding: str = "rgb8"
    frame_id: str | None = None
    intrinsics: Intrinsics | None = None
    stream_policy: StreamPolicy | None = None
    vendor: dict[str, str] | None = None

    def _compile(self, name: str) -> dict:
        out: dict = {
            "name": name,
            "width": int(self.width),
            "height": int(self.height),
            "fps": float(self.fps),
            "encoding": _enum_name(
                self.encoding, "CAMERA_ENCODING", _CAMERA_ENCODINGS, "encoding"
            ),
        }
        if self.frame_id:
            out["frameId"] = self.frame_id
        if self.intrinsics is not None:
            out["intrinsics"] = self.intrinsics._compile()
        if self.stream_policy is not None:
            # CameraDescription's field is named `stream`, not `streamPolicy`.
            out["stream"] = self.stream_policy._compile()
        if self.vendor:
            out["vendor"] = {str(k): str(v) for k, v in self.vendor.items()}
        return out


@dataclass(frozen=True)
class FrameTransform:
    """A named static transform: the pose of ``child`` expressed in
    ``parent``. ``quaternion`` is **wxyz** order (w first) — this
    protocol's pinned convention (see ``descriptors.proto``'s ``Quat``); a
    transposed xyzw quaternion is the classic conversion bug."""

    parent: str
    child: str
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    quaternion: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        if not self.parent:
            raise ValueError("FrameTransform must declare parent")
        if not self.child:
            raise ValueError("FrameTransform must declare child")
        if len(self.position) != 3:
            raise ValueError("position must be an (x, y, z) triple")
        if len(self.quaternion) != 4:
            raise ValueError("quaternion must be a (w, x, y, z) quadruple")

    def _compile(self) -> dict:
        px, py, pz = self.position
        w, x, y, z = self.quaternion
        return {
            "parent": self.parent,
            "child": self.child,
            # Pose.frame_id: the frame this pose's numbers are expressed
            # in — always `parent`, per the proto's own field comment.
            "transform": {
                "position": {"x": float(px), "y": float(py), "z": float(pz)},
                "rotation": {"w": float(w), "x": float(x), "y": float(y), "z": float(z)},
                "frameId": self.parent,
            },
        }


@dataclass(frozen=True)
class TimeSeries:
    """A generic non-camera sensor stream: joint states, F/T, IMU — without
    enumerating sensor types forever."""

    dtype: str = "f32"
    shape: tuple[int, ...] = ()
    units: str = ""
    frame_id: str | None = None
    rate_hz: float | None = None

    def __post_init__(self) -> None:
        if any(s < 0 for s in self.shape):
            raise ValueError("shape entries must be >= 0")
        if self.rate_hz is not None and self.rate_hz <= 0:
            raise ValueError("rate_hz must be > 0")

    def _compile(self, name: str) -> dict:
        out: dict = {
            "name": name,
            "dtype": _enum_name(
                self.dtype,
                "DTYPE",
                {"f32": "F32", "f64": "F64", "i32": "I32", "i64": "I64", "u8": "U8"},
                "dtype",
            ),
        }
        if self.shape:
            out["shape"] = [int(s) for s in self.shape]
        if self.units:
            out["units"] = self.units
        if self.frame_id:
            out["frameId"] = self.frame_id
        if self.rate_hz is not None:
            out["rateHz"] = float(self.rate_hz)
        return out


@dataclass(frozen=True)
class Robot:
    """The robot declaration compiled to ``waddle.v0.RobotDescription``.
    Grants are NOT declared here — ``the Site lifecycle`` derives them from which
    ``Control`` verbs are provided.

    ``kinematics_urdf`` accepts raw ``bytes`` (passed through as-is) or a
    path (``str`` / ``pathlib.Path``) that is read at compile time; to
    embed literal URDF XML text, encode it yourself
    (``robot_xml.encode()``).
    """

    name: str
    action_space: _Space
    robot_id: str = ""
    cell_id: str = ""
    cameras: dict[str, Camera] = field(default_factory=dict)
    kinematics_urdf: bytes | str | Path | None = None
    frames: tuple[FrameTransform, ...] = ()
    series: dict[str, TimeSeries] = field(default_factory=dict)

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
        if self.kinematics_urdf is not None:
            data = (
                self.kinematics_urdf
                if isinstance(self.kinematics_urdf, bytes)
                else Path(self.kinematics_urdf).read_bytes()
            )
            # proto3 canonical JSON encodes `bytes` fields as base64.
            out["kinematicsUrdf"] = base64.b64encode(data).decode("ascii")
        if self.frames:
            out["frames"] = {"transforms": [ft._compile() for ft in self.frames]}
        if self.series:
            out["series"] = [ts._compile(name) for name, ts in self.series.items()]
        if grants:
            out["grants"] = grants
        return out
