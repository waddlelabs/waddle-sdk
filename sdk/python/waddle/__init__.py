"""Waddle: supervision for real-world robot policy rollouts.

The six-line tutorial loop::

    with waddle.rollout(task="fold the towel") as ep:
        while not ep.done:
            obs = get_obs()
            action = policy(obs)
            action = ep.gate(action, obs)
            if action is not None:
                send(action)

``ep.gate()`` always answers "what you should send, or ``None`` if you must
not send": Pass returns your exact object, Substitute/Blend a fresh float64
ndarray, Noop and Hold return ``None``.

This package is a hollow frontend: every claim/lease/handoff/timeline
decision is made in waddle-core (the Rust runtime under ``waddle._core``);
Python only declares and marshals.
"""

from __future__ import annotations

import atexit
import enum
import json
import threading
from collections.abc import Callable
from dataclasses import dataclass
from os import PathLike

from . import _core, descriptors
from .descriptors import (
    Camera,
    Chunking,
    Composite,
    EEDelta,
    Gripper,
    JointSpace,
    Opaque,
    Robot,
)

__all__ = [
    "Camera",
    "Chunking",
    "Composite",
    "Control",
    "EEDelta",
    "Gripper",
    "Handoff",
    "JointSpace",
    "Opaque",
    "Outcome",
    "Robot",
    "descriptors",
    "init",
    "rollout",
    "shutdown",
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
    ``(ndarray, gripper, offset_ns)`` tuples), ``provenance`` and ``seq``.
    The unit verbs take no arguments. All verbs are invoked from a single
    core-owned dispatch thread, never concurrently; a raised exception is a
    failed verb, never a crashed session.
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


_lock = threading.Lock()
_session: _core.Session | None = None
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


def init(
    project: str,
    robot: Robot,
    control: Control,
    *,
    recording_dir: str | PathLike | None = None,
    handoff: _Handoff = Handoff.HOLD_FIRST,
    lease_enforcement: str = "advisory",
    _testing: bool = False,
) -> _core.Session:
    """Open the supervision session. One session per process in v1."""
    global _session, _atexit_registered
    if not isinstance(robot, Robot):
        raise TypeError("robot must be a waddle.Robot")
    if not isinstance(control, Control):
        raise TypeError("control must be a waddle.Control")
    if not isinstance(handoff, _Handoff):
        raise TypeError("handoff must be a waddle.Handoff declaration")

    robot_json = json.dumps(robot._compile(_derive_grants(control, robot.action_space)))
    with _lock:
        if _session is not None:
            raise RuntimeError("waddle.init() called while a session is open; "
                               "call waddle.shutdown() first")
        session = _core.create_session(
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
            testing_loopback=_testing,
        )
        _session = session
        if not _atexit_registered:
            atexit.register(shutdown)
            _atexit_registered = True
    return session


def _require_session() -> _core.Session:
    with _lock:
        if _session is None:
            raise RuntimeError("waddle.init() has not been called")
        return _session


class _Rollout:
    """Context manager for one rollout attempt. Exiting while the episode is
    non-terminal terminates it ``abort`` — never success (silently inflating
    SR denominators is what amendment N2 exists to prevent). Exceptions are
    never swallowed."""

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


def rollout(task: str) -> _Rollout:
    """Open an episode on the module session; blocks until the scene reset
    completes (the design contract: it does not yield an invalid scene)."""
    return _Rollout(_require_session().start_episode(task))


def shutdown() -> None:
    """Join all core threads and flush recorders. Idempotent; also
    registered via ``atexit`` by ``init``."""
    global _session
    with _lock:
        session, _session = _session, None
    if session is not None:
        session.shutdown()
