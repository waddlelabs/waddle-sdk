"""Waddle: supervision for real-world robot policy rollouts.

The six-line tutorial loop::

    with waddle_sdk.rollout(task="fold the towel") as ep:
        while not ep.done:
            obs = get_obs()
            action = policy(obs)
            action = ep.gate(action, obs)
            if action is not None:
                send(action)

``ep.gate()`` always answers "what you should send, or ``None`` if you must
not send": Pass returns your exact object, Substitute/Blend a fresh float64
ndarray, Noop and Hold return ``None``.

A robot that declares named parts (:class:`Composite` — a bimanual cell,
say) can be intervened on ONE part at a time, so on such a declaration every
intervention payload is keyed by part instead: ``ep.gate()`` returns
``{"right": ndarray}`` for an action addressing the right arm and a dict of
every declared part for a whole-robot one, and a dispatched chunk's step
values follow the same rule. ``ep.last_gate.part`` names the addressed part
(``None`` = the whole robot). Nothing changes for a robot without parts.
``session.report_proprio(part=..., joint_pos=...)`` reports back the same
way, one part at a time.

Cameras declared on the :class:`Robot` become live once a media plane is
wired (``media=`` on :func:`init`, or ``_testing=True``): call
``session.publish_frame(name, frame)`` with a numpy ``uint8`` array shaped
``(height, width, 3)`` (packed row-major RGB8) each time a new frame is
available — the ``name`` must match a declared camera. The core validates
the camera and frame shape, polices the camera's declared ``StreamPolicy``
uplink fps (dropping frames faster than the declared rate — the policy
working as intended, never an error), and publishes the track lazily on the
first frame. A camera declared with no media plane wired is a cheap no-op.

The camera's ``Uplink.encoding`` (``rgb8``/``bgr8``/``jpeg``) declares
bandwidth-intent for the video track, not a literal wire format: every one
of those publishes this same raw RGB8 frame through to the track, and the
wired transport (a real LiveKit session) converts and compresses it itself
— WebRTC tracks carry raw pixels, never a pre-encoded still image.
``h264`` is the one unsupported encoding: declaring it against a wired
media plane is a clean error at :func:`init` time, not a per-frame failure.

Everything above works with no supervision plane at all: with
``recording_dir`` set, every episode lands as a local sidecar + MCAP.
``init(transport=waddle_sdk.Grpc(url, token))`` connects the session to a
plane, which is what makes supervision (teleoperator intervention, remote
reset windows) and :func:`agent` — "Waddle, drive this one" — possible. The
control transport is compiled into the published ``waddle-sdk`` wheel; the
LiveKit media plane :class:`LiveKit` declares rides the teleop companion
wheel (``pip install 'waddle-sdk[teleop]'``), which is also the only
difference between the two.

This package is a hollow frontend: every claim/lease/handoff/timeline
decision is made in waddle-core (the Rust runtime under ``waddle_sdk._core``);
Python only declares and marshals.
"""

from __future__ import annotations

import atexit
import enum
import json
import threading
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from os import PathLike
from pathlib import Path
from typing import TYPE_CHECKING

from . import _native, descriptors
from ._native import core
from .cameras import CameraDriver, CameraFrame, CameraSample
from .descriptors import (
    Camera,
    Chunking,
    Composite,
    EEDelta,
    FrameTransform,
    Gripper,
    Intrinsics,
    Joint,
    JointSpace,
    Opaque,
    Robot,
    StreamPolicy,
    TimeSeries,
    Uplink,
)
from ._services import (
    CalibrationMeasurement,
    ExecutionBackend,
    TaskSession,
    WorkspaceArtifactRequest,
    execution_backends,
)
from . import _services
from ._ui import UIHandle

if TYPE_CHECKING:  # `_core.pyi` types whichever core `_native` selected
    from . import _core
    from .robots.base import Rig, RigSession

#: The compiled core's version — the ONE version this package has (the
#: Python surface and the shim ship together, from one Cargo.toml).
__version__: str = core.__version__
SessionStamp = core.SessionStamp

__all__ = [
    "AgentReset",
    "AgentResult",
    "Camera",
    "CameraDriver",
    "CameraFrame",
    "CameraSample",
    "CalibrationMeasurement",
    "Chunking",
    "Composite",
    "Control",
    "EEDelta",
    "ExecutionBackend",
    "FrameTransform",
    "Gripper",
    "Grpc",
    "Handoff",
    "Intrinsics",
    "Joint",
    "JointSpace",
    "LiveKit",
    "Opaque",
    "Outcome",
    "Robot",
    "SessionStamp",
    "StreamPolicy",
    "TaskSession",
    "TeleopReset",
    "TimeSeries",
    "Uplink",
    "UIHandle",
    "WorkspaceArtifactRequest",
    "agent",
    "calibration_click",
    "calibration_updates",
    "descriptors",
    "execution_backends",
    "init",
    "request_workspace_artifact",
    "rollout",
    "shutdown",
    "task_session",
    "ui",
]


class Outcome(str, enum.Enum):
    """Terminal episode outcomes settable by the caller."""

    SUCCESS = "success"
    FAILURE = "failure"
    ABORT = "abort"


@dataclass(frozen=True)
class Control:
    """The five-verb control contract: each verb is a callable you provide;
    the grants Waddle plans against are derived from which verbs are set.

    ``send`` receives a chunk with ``steps`` (a list of
    ``(values, gripper, offset_ns)`` tuples), ``provenance`` and ``seq``.
    A step's ``values`` is an ndarray of the declared action space's width,
    except for a gripper-only step — "hold the arm, move the gripper" —
    whose array is EMPTY and whose ``gripper`` is set: command the gripper
    and leave the arm target where it was. The unit verbs take no arguments.
    All verbs are invoked from a single core-owned dispatch thread, never
    concurrently; a raised exception is a failed verb, never a crashed
    session.

    On a :class:`Composite` declaration ``values`` is instead a ``dict``
    keyed by declared part — ``{"right": ndarray}`` for a step addressing
    one part ("move this part, hold the rest": the parts absent from the
    dict are commanded nothing), every declared part for a whole-robot step,
    sliced by the declared layout. A gripper-only step maps its parts to
    empty arrays, the same "hold the arm" as above.
    """

    send: Callable | None = None
    hold: Callable | None = None
    resume: Callable | None = None
    home: Callable | None = None
    estop: Callable | None = None
    estop_hardware: bool = False
    estop_latency_bound_ms: float | None = None

    def __post_init__(self) -> None:
        if isinstance(self.send, dict):
            raise TypeError(
                "Control.send takes ONE callable in v1 "
                "(multi-interface send lands with the ee_delta teleop path)"
            )
        for name in ("send", "hold", "resume", "home", "estop"):
            value = getattr(self, name)
            if value is not None and not callable(value):
                raise TypeError(f"Control.{name} must be callable or None")


@dataclass(frozen=True)
class Grpc:
    """Declare the session's control-plane transport: the connection over
    which the supervision plane sees this session's timeline and sends its
    directives (claims, remote reset windows, agent task updates). Without
    one, a session is a local recorder — everything still runs, nothing is
    supervised.

    ``token`` is the plane's own credential for this session; this SDK
    never mints one. Leave it ``None`` only for a plane that asks for no
    credential (a local development plane).

    Pure declaration: connecting, backoff, replay after a partition, and
    every directive's meaning are waddle-core's, on its own thread — this
    call does not dial anything."""

    url: str
    token: str | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.url, str) or not self.url:
            raise ValueError("Grpc.url must be a non-empty str")
        if self.token is not None and (
            not isinstance(self.token, str) or not self.token
        ):
            raise ValueError("Grpc.token must be a non-empty str or None")


@dataclass(frozen=True)
class LiveKit:
    """Declare a real LiveKit-backed media plane: camera frames
    (``session.publish_frame``), teleop pose/clutch/mark, and telemetry all
    ride this WebRTC connection instead of the in-process ``_testing``
    loopback.

    This is the *teleop* media path — a live human at the other end of a
    WebRTC session — and it is the one thing the default ``waddle-sdk``
    wheel does not carry: LiveKit's ``webrtc-sys``/libwebrtc dependency
    chain is ~690 MB of build, which an install that only supervises a
    policy should never pay for. It ships in the companion wheel instead::

        pip install 'waddle-sdk[teleop]'

    Passing this to :func:`init` on a core without the ``livekit`` feature
    raises a ``RuntimeError`` naming that extra rather than degrading to a
    session with no media. Low-rate stills for a Waddle-hosted agent are a
    different path entirely (they ride the control plane, bounded by each
    camera's declared ``StreamPolicy.still_fps``) and need no media plane.
    Use ``waddle_sdk.init(_testing=True)`` (the in-process loopback) to
    exercise ``publish_frame`` and the teleop stream in tests."""

    url: str
    token: str

    def __post_init__(self) -> None:
        if not isinstance(self.url, str) or not self.url:
            raise ValueError("LiveKit.url must be a non-empty str")
        if not isinstance(self.token, str) or not self.token:
            raise ValueError("LiveKit.token must be a non-empty str")


@dataclass(frozen=True)
class _Handoff:
    kind: str
    ns: int = 0


class Handoff:
    """Lease-handoff policy declarations (sugar over the wire shapes).
    Python only declares these; all handoff sequencing runs in core."""

    HOLD_FIRST = _Handoff("hold_first")

    @staticmethod
    def IMMEDIATE(blend_ms: float = 0.0) -> _Handoff:
        return _Handoff("immediate", int(blend_ms * 1_000_000))

    @staticmethod
    def CHUNK_BOUNDARY(max_wait_ms: float = 0.0) -> _Handoff:
        return _Handoff("chunk_boundary", int(max_wait_ms * 1_000_000))


def _validate_reset_marker(cls_name: str, prompt: object, timeout_s: object) -> None:
    """Shape validation shared by :class:`TeleopReset`/:class:`AgentReset`
    (identical contract, only the expected actor kind differs between
    them) — never reset behavior, just "is this a well-formed marker"."""
    if not isinstance(prompt, str):
        raise TypeError(f"{cls_name}.prompt must be a str")
    if isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)):
        raise TypeError(f"{cls_name}.timeout_s must be a positive number of seconds")
    if timeout_s <= 0:
        raise ValueError(f"{cls_name}.timeout_s must be a positive number of seconds")


@dataclass(frozen=True)
class TeleopReset:
    """Declare that this reset phase (pre or post) is a remote reset window
    performed by a teleoperator through the SDK (FSM.md §1.4, flag
    `waddle.v0.reset.remote`): a claim is granted for the window, the lease
    hands to the claimant, and the gate goes RESET until they signal
    complete. Pure declaration — Python never drives the window itself; the
    claim/lease/gate-mode sequencing lives entirely in waddle-core.

    A window declared this way needs a connected supervision plane to grant
    and complete it — ``waddle_sdk.init(transport=waddle_sdk.Grpc(url, token))``.
    With no plane declared, the window opens and can only run out its
    timeout; the private `waddle_sdk._testing` reset-window hooks (tests only)
    are what drive one without a plane."""

    prompt: str
    timeout_s: float = field(default=600.0, kw_only=True)

    def __post_init__(self) -> None:
        _validate_reset_marker("TeleopReset", self.prompt, self.timeout_s)


@dataclass(frozen=True)
class AgentReset:
    """Declare that this reset phase is a remote reset window performed by
    an autonomous reset agent through the SDK. Same shape and semantics as
    :class:`TeleopReset` (including the connected-plane requirement); only
    the expected actor kind differs (FSM.md §1.4 guard C6 — a window
    declared for one actor kind rejects a claim from the other).

    A *reset* agent cleaning the scene between episodes; distinct from
    :func:`agent`, which hands a whole episode to a Waddle-hosted agent."""

    prompt: str
    timeout_s: float = field(default=600.0, kw_only=True)

    def __post_init__(self) -> None:
        _validate_reset_marker("AgentReset", self.prompt, self.timeout_s)


def _normalize_reset_hook(fn: Callable) -> Callable[[str], tuple]:
    """Wrap a caller-supplied reset hook so the FFI always receives
    ``(bool, Optional[bool])``, regardless of whether the caller's hook
    returns a bare ``bool`` or the full tuple.

    A bare ``True``/``False`` vouches for its own verification — it
    normalizes to ``(ok, ok)``, not ``(ok, None)``: under the default
    "blocking" verification mode a hook that never mentions ``verified``
    must still be able to reach READY, and ``(ok, None)`` would hang a
    trivial ``lambda task: True`` forever waiting for a verification
    opinion that never comes. Anything else — the wrong arity, a non-bool
    first element, or a second element that is neither bool nor None —
    raises ``TypeError`` naming the contract.

    That ``TypeError`` never reaches the caller of :func:`rollout`,
    though: ``PyResetHook::call`` (``sdk/rust/src/verbs.rs``) invokes this
    wrapper from core-owned code and unconditionally catches whatever it
    raises — this wrapper's own ``TypeError`` included — reporting it only
    via ``PyErr::write_unraisable`` (``sys.unraisablehook``: a stderr
    print normally, a ``PytestUnraisableExceptionWarning`` under pytest)
    and normalizing the outcome to ``(False, None)``, exactly like a hook
    that legitimately returns ``False``. So the ``TypeError``'s only
    effect is a diagnostic breadcrumb on a side channel the caller isn't
    necessarily watching; the caller of :func:`rollout` sees the same
    generic ``RuntimeError: reset failed`` either way and cannot
    distinguish "my hook is malformed" from "my hook correctly reported
    failure" from that exception alone."""

    def wrapped(task: str) -> tuple:
        result = fn(task)
        if isinstance(result, bool):
            return (result, result)
        if (
            isinstance(result, tuple)
            and len(result) == 2
            and isinstance(result[0], bool)
            and (result[1] is None or isinstance(result[1], bool))
        ):
            return result
        raise TypeError(
            "a reset hook must return bool (ok) or (bool, Optional[bool]) "
            f"(ok, verified); got {result!r}"
        )

    return wrapped


def _reset_spec_kwargs(label: str, value: Callable | TeleopReset | AgentReset | None) -> dict:
    """Map one reset-phase declaration (``None`` | callable | TeleopReset |
    AgentReset) onto the `_core` FFI's ``{label}_kind``/``_hook``/
    ``_prompt``/``_timeout_ns`` kwargs (``label`` is ``"pre_reset"`` or
    ``"post_reset"``). Pure type dispatch and kwarg marshalling — the reset
    semantics themselves (which strategy runs, how verification gates the
    transition, the window's claim/lease/gate-mode sequencing) live
    entirely in waddle-core; this only decides which kind string names the
    marker's type."""
    if value is None:
        return {f"{label}_kind": "none"}
    if isinstance(value, TeleopReset):
        return {
            f"{label}_kind": "teleop",
            f"{label}_prompt": value.prompt,
            f"{label}_timeout_ns": int(value.timeout_s * 1_000_000_000),
        }
    if isinstance(value, AgentReset):
        return {
            f"{label}_kind": "agent",
            f"{label}_prompt": value.prompt,
            f"{label}_timeout_ns": int(value.timeout_s * 1_000_000_000),
        }
    if callable(value):
        return {f"{label}_kind": "hook", f"{label}_hook": _normalize_reset_hook(value)}
    raise TypeError(
        f"{label} must be None, a callable, waddle_sdk.TeleopReset, or waddle_sdk.AgentReset "
        f"(got {type(value).__name__})"
    )


class _UnsetType:
    """The `rollout()` sentinel meaning "inherit whatever `init()`
    declared" — distinct from an explicit `None` ("disable this phase for
    this one episode only")."""

    def __repr__(self) -> str:
        return "<UNSET>"


_UNSET = _UnsetType()


def _reset_override_kwargs(
    label: str, value: Callable | TeleopReset | AgentReset | None | _UnsetType
) -> dict:
    """Like `_reset_spec_kwargs`, but for `rollout()`'s per-episode
    overrides: `_UNSET` (the default) maps to no kwargs at all, so
    `_core.Session.start_episode`'s own `None`-means-inherit default
    applies untouched."""
    if value is _UNSET:
        return {}
    return _reset_spec_kwargs(label, value)


_lock = threading.Lock()
_session: _core.Session | None = None
# Did the open session declare a supervision plane at all — a `transport`,
# or the private `_testing` loopback that stands in for one in tests? The
# single thing :func:`agent` must know before marshalling a prompt into
# core, since a session that declared no plane can only ever run the invite
# out to its deadline. A declaration fact recorded at `init` time, never
# plane state: nothing here observes a connection.
_session_has_plane = False
_recording_dir: Path | None = None
_ui_handle: UIHandle | None = None
_managed_rig: RigSession | None = None
_session_starting = False
_session_closing = False
_atexit_registered = False


def _derive_grants(control: Control, space: descriptors._Space) -> list[dict]:
    """Presence → JSON marshalling, not policy: a grant exists exactly when
    its verb callable does."""
    grants: list[dict] = []
    if control.send is not None:
        grants.append({"verb": "VERB_SEND", "sendInterfaces": [space._space_kind()]})
    if control.hold is not None:
        grants.append({"verb": "VERB_HOLD"})
    if control.resume is not None:
        grants.append({"verb": "VERB_RESUME"})
    if control.home is not None:
        grants.append({"verb": "VERB_HOME"})
    if control.estop is not None:
        grant: dict = {"verb": "VERB_ESTOP"}
        if control.estop_hardware:
            grant["hardware"] = True
        if control.estop_latency_bound_ms is not None:
            # int64 crosses canonical proto3 JSON as a decimal string.
            grant["declaredLatencyBoundNs"] = str(
                int(control.estop_latency_bound_ms * 1_000_000)
            )
        grants.append(grant)
    return grants


def _create_core_session(
    project: str,
    robot: Robot,
    control: Control,
    *,
    recording_dir: str | PathLike | None = None,
    handoff: _Handoff = Handoff.HOLD_FIRST,
    lease_enforcement: str = "advisory",
    transport: Grpc | None = None,
    media: LiveKit | None = None,
    pre_reset: Callable | TeleopReset | AgentReset | None = None,
    post_reset: Callable | TeleopReset | AgentReset | None = None,
    reset_verification: str = "blocking",
    _testing: bool = False,
) -> _core.Session:
    """Build one core session without registering module lifecycle ownership."""
    if not isinstance(robot, Robot):
        raise TypeError("robot must be a waddle_sdk.Robot")
    if not isinstance(control, Control):
        raise TypeError("control must be a waddle_sdk.Control")
    if not isinstance(handoff, _Handoff):
        raise TypeError("handoff must be a waddle_sdk.Handoff declaration")
    if transport is not None and not isinstance(transport, Grpc):
        raise TypeError("transport must be a waddle_sdk.Grpc or None")
    if media is not None and not isinstance(media, LiveKit):
        raise TypeError("media must be a waddle_sdk.LiveKit or None")
    if transport is not None and _testing:
        raise ValueError(
            "transport and _testing=True are mutually exclusive: the loopback "
            "session stands in for a plane, it does not dial one"
        )
    if media is not None and _testing:
        raise ValueError(
            "media and _testing=True both wire a media plane — pass only one"
        )
    if media is not None and "livekit" not in _native.FEATURES:
        raise RuntimeError(
            "LiveKit media is teleop-only and not compiled into this core — "
            "install the teleop extra: pip install 'waddle-sdk[teleop]'"
        )
    if transport is not None and "grpc" not in _native.FEATURES:
        raise RuntimeError(
            "the control-plane transport is not compiled into this core: this is "
            "a from-source build without the `grpc` cargo feature (the published "
            "waddle-sdk wheel carries it) — rebuild the extension with "
            "`maturin develop --features grpc`"
        )

    reset_kwargs = {
        **_reset_spec_kwargs("pre_reset", pre_reset),
        **_reset_spec_kwargs("post_reset", post_reset),
    }
    robot_json = json.dumps(robot._compile(_derive_grants(control, robot.action_space)))
    return core.create_session(
        project=project,
        robot_json=robot_json,
        send=control.send,
        hold=control.hold,
        resume=control.resume,
        home=control.home,
        estop=control.estop,
        estop_hardware=control.estop_hardware,
        estop_latency_bound_ns=(
            int(control.estop_latency_bound_ms * 1_000_000)
            if control.estop_latency_bound_ms is not None
            else None
        ),
        recording_dir=(None if recording_dir is None else str(recording_dir)),
        handoff_kind=handoff.kind,
        handoff_ns=handoff.ns,
        lease_enforcement=lease_enforcement,
        reset_verification=reset_verification,
        testing_loopback=_testing,
        transport_url=(None if transport is None else transport.url),
        transport_token=(None if transport is None else transport.token),
        media_url=(None if media is None else media.url),
        media_token=(None if media is None else media.token),
        **reset_kwargs,
    )


def init(
    project: str,
    robot: Robot | None = None,
    control: Control | None = None,
    *,
    rig: Rig | None = None,
    send: Callable[[object], None] | None = None,
    console: bool = True,
    recording_dir: str | PathLike | None = None,
    handoff: _Handoff = Handoff.HOLD_FIRST,
    lease_enforcement: str = "advisory",
    transport: Grpc | None = None,
    media: LiveKit | None = None,
    pre_reset: Callable | TeleopReset | AgentReset | None | _UnsetType = _UNSET,
    post_reset: Callable | TeleopReset | AgentReset | None = None,
    reset_verification: str = "blocking",
    _testing: bool = False,
) -> _core.Session:
    """Open the process's one supervision session.

    Pass the legacy ``robot`` and ``control`` pair, or pass ``rig=`` and let
    the SDK own that rig's arms, reporting pump and camera capture until
    :func:`shutdown`. The two forms are mutually exclusive. ``send`` and
    ``console`` are rig-only equivalents of :meth:`Rig.session`'s keywords.
    A rig inherits its own pre-reset when ``pre_reset`` is omitted; an explicit
    ``None`` disables it. The legacy path keeps its historical no-reset default.
    """
    global _session, _session_has_plane, _recording_dir, _managed_rig
    global _session_starting, _atexit_registered

    managed: RigSession | None = None
    if rig is not None:
        if robot is not None or control is not None:
            raise ValueError("rig is mutually exclusive with robot and control")
        from .robots.base import RIG_DEFAULT, Rig

        if not isinstance(rig, Rig):
            raise TypeError("rig must be a waddle_sdk.robots.base.Rig")
        if not isinstance(console, bool):
            raise TypeError("console must be a bool")
        managed = rig.session(
            project,
            send=send,
            transport=transport,
            media=media,
            recording_dir=recording_dir,
            handoff=handoff,
            lease_enforcement=lease_enforcement,
            pre_reset=(RIG_DEFAULT if pre_reset is _UNSET else pre_reset),
            post_reset=post_reset,
            reset_verification=reset_verification,
            console=console,
            _testing=_testing,
        )
    else:
        if robot is None or control is None:
            raise TypeError("waddle_sdk.init() needs either rig= or both robot and control")
        if send is not None:
            raise ValueError(
                "send is a rig-only keyword; put it on Control for this path"
            )
        if console is not True:
            raise ValueError("console is a rig-only keyword")
        pre_reset = None if pre_reset is _UNSET else pre_reset

    with _lock:
        if _session is not None or _session_starting or _session_closing:
            raise RuntimeError(
                "waddle_sdk.init() called while a session is open or closing; "
                "call waddle_sdk.shutdown() first"
            )
        _session_starting = True
    try:
        if managed is not None:
            managed._open(_create_core_session)
            assert managed.core is not None
            session = managed.core
        else:
            assert robot is not None and control is not None
            session = _create_core_session(
                project,
                robot,
                control,
                recording_dir=recording_dir,
                handoff=handoff,
                lease_enforcement=lease_enforcement,
                transport=transport,
                media=media,
                pre_reset=pre_reset,
                post_reset=post_reset,
                reset_verification=reset_verification,
                _testing=_testing,
            )
        with _lock:
            _session = session
            _managed_rig = managed
            _session_has_plane = transport is not None or _testing
            _recording_dir = None if recording_dir is None else Path(recording_dir)
            if not _atexit_registered:
                atexit.register(shutdown)
                _atexit_registered = True
        return session
    finally:
        with _lock:
            _session_starting = False


def ui(
    *,
    joint_step_rad: float = 0.01,
    linear_step_m: float = 0.005,
    angular_step_rad: float = 0.02,
) -> UIHandle:
    """Start this session's authenticated local browser UI.

    Requires an active :func:`init` session and binds only to an
    OS-selected port on ``127.0.0.1``. The printed URL carries a per-run
    256-bit token in its fragment; every data/control request must present
    it in a custom header. Repeated calls for the same session return the
    existing handle. There is deliberately no standalone ``waddle ui``
    command: the controls and cameras belong to the in-process customer
    session and its registered ``Control`` callbacks.

    The three positive finite increments are local presentation settings
    for this UI run only. Browser changes update only this handle. Neither
    this function nor the browser clamps a command: core constructs one
    declared-space step, and the owner's envelope remains the final
    whole-command refusal.
    """

    global _ui_handle
    # Validate before consulting the existing handle so a malformed call is
    # never silently accepted. `_ui.UIHandle` applies the same checks again
    # when it owns the configuration.
    from ._ui import _positive

    increments = {
        "joint_step_rad": _positive("joint_step_rad", joint_step_rad),
        "linear_step_m": _positive("linear_step_m", linear_step_m),
        "angular_step_rad": _positive("angular_step_rad", angular_step_rad),
    }
    with _lock:
        if _session is None:
            raise RuntimeError("waddle_sdk.ui() requires an active waddle_sdk.init() session")
        if _ui_handle is not None and not _ui_handle.closed:
            return _ui_handle
        handle = UIHandle(
            _session,
            _recording_dir,
            managed_rig=_managed_rig,
            **increments,
        )
        _ui_handle = handle
        status = dict(_session.status())
    local = "available (state, e-stop, jog, cameras, recordings)"
    chat = (
        "available when the invited host is alive"
        if status.get("plane_connected") and status.get("chat_negotiated")
        else "unavailable on the current connection; local controls remain available"
    )
    print(f"Waddle UI: {handle.url}\nLocal: {local}\nChat: {chat}")
    return handle


def _require_session() -> _core.Session:
    with _lock:
        if _session is None:
            raise RuntimeError("waddle_sdk.init() has not been called")
        return _session


def task_session(
    name: str, *, task_session_id: str | None = None
) -> TaskSession:
    """Create or resume a named durable hosted task conversation.

    A new handle submits CREATE immediately. Pass a plane-issued
    ``task_session_id`` to resume a durable conversation before sending a
    message, interjection, or interrupt. Public-safe live output and history
    are available through :meth:`TaskSession.events` and ``history``.
    """
    return TaskSession(_require_session(), name, task_session_id=task_session_id)


def calibration_click(
    calibration_id: str,
    sample_id: str,
    camera: str,
    frame_sequence: int,
    x: int,
    y: int,
) -> CalibrationMeasurement:
    """Resolve a managed rig's retained RGB-D pixel and submit only its point."""
    session = _require_session()
    with _lock:
        managed = _managed_rig
    return _services.submit_calibration_click(
        session,
        managed,
        calibration_id=calibration_id,
        sample_id=sample_id,
        camera=camera,
        frame_sequence=frame_sequence,
        x=x,
        y=y,
    )


def calibration_updates(
    calibration_id: str,
    *,
    after_sequence: int = 0,
    timeout_s: float = 0.0,
) -> list[dict]:
    """Poll bounded public calibration lifecycle updates from the plane."""
    return _services.calibration_updates(
        _require_session(),
        calibration_id,
        after_sequence=after_sequence,
        timeout_s=timeout_s,
    )


def request_workspace_artifact(
    *,
    graph_ids: tuple[str, ...] | list[str] = (),
    calibration_names: tuple[str, ...] | list[str] = (),
) -> WorkspaceArtifactRequest:
    """Request a signed allowlisted workspace and poll its bounded status.

    The returned metadata contains an opaque one-time ``download_ref`` for a
    separate authenticated artifact endpoint. Archive bytes never transit the
    session's GateActions stream or this helper.
    """
    return WorkspaceArtifactRequest(
        _require_session(), graph_ids, calibration_names
    )


class _Rollout:
    """Context manager for one rollout attempt. Exiting while the episode is
    non-terminal terminates it ``abort`` — never success (silently inflating
    SR denominators is what amendment N2 exists to prevent). Exceptions are
    never swallowed.

    Post-reset exit contract: ``ep.done`` flips to ``True`` the instant the
    episode's terminal outcome is decided (POST_RESET entry), even though a
    declared post-reset's scene cleanup may still be running. So by the
    time this checks ``self._episode.done``, an in-flight post-reset already
    reads as done and this is a no-op — it never calls ``terminate()``
    again and never interferes with (or aborts) a post-reset still running
    in the background. See :func:`rollout` for the full contract."""

    def __init__(self, episode: _core.Episode) -> None:
        self._episode = episode

    def __enter__(self) -> _core.Episode:
        return self._episode

    def __exit__(self, exc_type, exc, tb) -> bool:
        if not self._episode.done:
            reason = (
                "rollout exited before a terminal outcome"
                if exc_type is None
                else f"unhandled {exc_type.__name__}: {exc}"
            )
            self._episode.terminate("abort", reason)
        return False


def _task_metadata(value: Mapping[str, str] | None) -> dict[str, str]:
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise TypeError("task_metadata must be a mapping of strings to strings")
    result: dict[str, str] = {}
    for key, item in value.items():
        if not isinstance(key, str) or not isinstance(item, str):
            raise TypeError("task_metadata must be a mapping of strings to strings")
        result[key] = item
    return result


def rollout(
    task: str,
    *,
    task_metadata: Mapping[str, str] | None = None,
    pre_reset: Callable | TeleopReset | AgentReset | None | _UnsetType = _UNSET,
    post_reset: Callable | TeleopReset | AgentReset | None | _UnsetType = _UNSET,
) -> _Rollout:
    """Open an episode on the module session.

    Blocks until the pre-reset completes — under "blocking" verification
    (the session default), until it is both ``ok`` *and* ``verified``;
    under "optimistic", as soon as it is ``ok`` — the design contract: this
    call never yields an invalid scene. Raises ``RuntimeError`` if the
    pre-reset fails (or every configured strategy is exhausted).

    ``task_metadata`` is optional generic string context persisted with the
    episode. It is never interpreted as capability or authority.

    ``pre_reset``/``post_reset`` default to inheriting whatever ``init()``
    declared for the session; pass ``None`` to disable a phase for this one
    episode only, or a fresh callable/:class:`TeleopReset`/
    :class:`AgentReset` to override it for this episode only (see
    :func:`init` for the shared marker contract, including the remote-
    window narrowing rule).

    Post-reset exit contract: once this episode's terminal outcome is
    decided, ``ep.done`` flips to ``True`` immediately — even though its
    declared post-reset (scene cleanup) may still be running. Calling
    ``ep.terminate(...)`` explicitly *blocks* until that cleanup fully
    finishes, so the ordinary ``with waddle_sdk.rollout(...) as ep: ...
    ep.terminate(...)`` pattern already blocks the ``with``-exit through
    POST_RESET. If the ``with`` block exits some other way while POST_RESET
    is still running (an exception, or a loop that only checks ``ep.done``
    and happens to observe it flip mid-cleanup), ``__exit__`` sees the
    episode already ``done`` and is a no-op: it never aborts, and never
    otherwise interferes with, an in-flight post-reset. A post-reset that
    fails (a ``False``/invalid hook result, an exhausted remote window, a
    window timeout, or an estop) never changes the already-pinned outcome —
    it only sets ``ep.post_reset_failed`` (check it after ``ep.done`` if
    the post-reset's own result matters to your caller).
    """
    kwargs = {
        **_reset_override_kwargs("pre_reset", pre_reset),
        **_reset_override_kwargs("post_reset", post_reset),
    }
    return _Rollout(
        _require_session().start_episode(
            task, task_metadata=_task_metadata(task_metadata), **kwargs
        )
    )


@dataclass(frozen=True)
class AgentResult:
    """What one :func:`agent` run produced. Every field is core's word,
    marshalled verbatim — nothing here is computed in Python.

    ``outcome`` uses the same spelling ``ep.outcome`` does (``"success"``,
    ``"failure"``, ``"abort"``, or ``"aborted_retake"`` when a retake
    replaced the episode). ``episode_id`` names the episode the agent drove,
    so the run is findable in the recordings. ``recording_ref`` is the
    plane's own opaque reference to its recording of the run, when the
    plane reported one; ``detail`` is the plane's last word about the task
    (why it was declined, or what it did), empty when it said nothing."""

    outcome: str
    episode_id: str
    recording_ref: str | None
    detail: str


def agent(
    prompt: str,
    *,
    timeout_s: float = 600.0,
    task_metadata: Mapping[str, str] | None = None,
    pre_reset: Callable | TeleopReset | AgentReset | None | _UnsetType = _UNSET,
    post_reset: Callable | TeleopReset | AgentReset | None | _UnsetType = _UNSET,
) -> AgentResult:
    """Ask Waddle to drive one episode (protocol flag ``waddle.v0.agent``).

    Opens an *agent-invited* episode on the module session and blocks until
    it reaches a terminal outcome::

        result = waddle_sdk.agent("clear the table and stack the cups")
        if result.outcome == "success":
            ...

    ``prompt`` is both the invite and the episode's task. ``task_metadata``
    is optional generic string context persisted in the sidecar and forwarded
    with the invite; it is never interpreted as capability or authority.
    ``timeout_s`` is
    the invite deadline: if no agent claims the episode in that time the
    episode aborts and this returns ``outcome == "abort"`` — a normal
    result, not an exception. ``pre_reset``/``post_reset`` override this
    one episode's reset phases exactly as they do on :func:`rollout` (the
    default inherits whatever :func:`init` declared; ``None`` disables a
    phase for this episode only). The same goes for a plane that declines the
    task; ``result.detail`` carries its reason.

    While this blocks, this thread is not the one driving the robot: the
    invited agent claims the episode through the very same intervention
    machinery a teleoperator uses, and its actions reach the robot through
    the ``send`` verb you registered at :func:`init`. Your own ``gate()``
    ticks would not dispatch in such an episode anyway (FSM.md E24) — which
    is why this call blocks instead of handing back an episode handle. A
    Ctrl-C asks the core to end the run and raises ``KeyboardInterrupt``
    once it has, never leaving an agent driving a robot whose caller
    walked away.

    Raises ``RuntimeError`` if no session is open, if the session declared
    neither a ``transport`` nor the private ``_testing`` loopback that
    stands in for one (there would be nobody to ask), or — surfaced
    verbatim from core — if this session registered no way for the agent to
    actuate (a ``send`` verb, and ``hold`` under the default HOLD_FIRST
    handoff): an invite that no engage could ever carry would otherwise
    stall with nothing to diagnose. A plane that never negotiated
    ``waddle.v0.agent`` is *not* an error here — it simply never routes an
    invite, and the deadline returns the ordinary ``"abort"``.

    Everything else about the run (the invite, its deadline, who may claim
    the episode, when the actions dispatch, how the outcome is decided) is
    waddle-core's; this function marshals a prompt in and an
    :class:`AgentResult` out."""
    if not isinstance(prompt, str) or not prompt:
        raise ValueError("waddle_sdk.agent() needs a non-empty prompt")
    if isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)):
        raise TypeError("timeout_s must be a positive number of seconds")
    if timeout_s <= 0:
        raise ValueError("timeout_s must be a positive number of seconds")
    session = _require_session()
    if not _session_has_plane:
        raise RuntimeError(
            "waddle_sdk.agent() asks the supervision plane to drive the episode, and "
            "this session declared no transport (nor the private _testing "
            "loopback that stands in for one) — pass "
            "transport=waddle_sdk.Grpc(url, token) to waddle_sdk.init()"
        )
    kwargs = {
        **_reset_override_kwargs("pre_reset", pre_reset),
        **_reset_override_kwargs("post_reset", post_reset),
    }
    result = session.agent(
        prompt,
        int(timeout_s * 1_000_000_000),
        task_metadata=_task_metadata(task_metadata),
        **kwargs,
    )
    return AgentResult(
        outcome=result.outcome,
        episode_id=result.episode_id,
        recording_ref=result.recording_ref,
        detail=result.detail,
    )


def shutdown() -> None:
    """Join core threads and close every owner-side managed rig resource."""
    global _session, _session_has_plane, _recording_dir, _ui_handle
    global _managed_rig, _session_closing
    with _lock:
        session, _session = _session, None
        handle, _ui_handle = _ui_handle, None
        managed, _managed_rig = _managed_rig, None
        _session_has_plane = False
        _recording_dir = None
        if session is None and managed is None and handle is None:
            return
        _session_closing = True
    try:
        if handle is not None:
            handle.close()
        if managed is not None:
            managed.close()
        elif session is not None:
            session.shutdown()
    finally:
        with _lock:
            _session_closing = False
