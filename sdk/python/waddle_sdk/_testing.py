"""Private, unstable test hooks. Requires ``waddle_sdk.init(_testing=True)``,
which wires an in-process loopback media plane whose far end these drive.

Everything here is marshalling into core-side helpers — no session-event
vocabulary exists in Python.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

if TYPE_CHECKING:  # annotations only — the session object comes from the
    # caller, and which core built it is `_native`'s decision, not ours.
    from . import _core


def engage(session: _core.Session, claim_id: str, source: str = "teleop") -> None:
    """Grant and engage a local claim (what a plane directive would do)."""
    session._testing_engage(claim_id, source)


def release(session: _core.Session, claim_id: str) -> None:
    """Release the claim; the lease returns to the customer loop."""
    session._testing_release(claim_id)


def push_teleop(
    session: _core.Session,
    values: Sequence[float],
    gripper: float | None = None,
) -> None:
    """Push one teleop stream packet through the loopback media plane (the
    same wire packet a teleoperator console would send)."""
    session._testing_push_teleop([float(v) for v in values], gripper)


def push_chunk(
    session: _core.Session,
    values: Sequence[float],
    part: str | None = None,
    gripper: float | None = None,
    offset_ns: int = 0,
) -> None:
    """Push one intervention step into the session's intervention stream (the
    same message a supervision plane sends), so a test can drive an
    intervention with no control-plane transport.

    ``part`` addresses one declared part by name (``Action.part``, flag
    ``waddle.v0.parts``); ``None`` is the whole robot. A claim must be active
    — nothing is buffered without one — and the core's own intake decides
    everything after that: validation, refusals, and whether the jitter
    buffer ever plays the step out."""
    session._testing_push_chunk(
        [float(v) for v in values], part, gripper, int(offset_ns)
    )


def reset_window_engage(session: _core.Session, claim_id: str, actor: str = "teleop") -> None:
    """Engage an already-open reset window (what a plane ENGAGE directive
    would do): grants the claim, hands the lease to it, and turns the gate
    RESET for `actor` ("teleop" | "agent" — must match the window's
    declared expected actor, FSM.md guard C6)."""
    session._testing_reset_window_engage(claim_id, actor)


def reset_window_complete(
    session: _core.Session, claim_id: str, ok: bool, verified: bool | None = None
) -> None:
    """Complete an engaged reset window (what a plane COMPLETE directive
    would do): the pipeline result applies as if the pre/post-reset had
    produced it directly."""
    session._testing_reset_window_complete(claim_id, ok, verified)


def mark_done(
    session: _core.Session, outcome: str = "success", reason: str = ""
) -> None:
    """End the live episode the way a plane `EpisodeDirective{MARK_DONE}`
    would: the terminal outcome comes from outside the customer's loop.
    This is how a `waddle_sdk.agent(...)` run finishes without a plane — the
    caller of `agent()` is blocked and holds no episode handle, so there is
    nothing else for a test to terminate through."""
    session._testing_mark_done(outcome, reason)


def frames(session: _core.Session, camera: str) -> list[bytes]:
    """Every raw frame payload the loopback media plane's far end has
    received for `camera` so far, in publish order — lets tests observe
    `session.publish_frame(camera, ...)` without a real transport."""
    return session._testing_frames(camera)
