"""Private, unstable test hooks. Requires ``waddle.init(_testing=True)``,
which wires an in-process loopback media plane whose far end these drive.

Everything here is marshalling into core-side helpers — no session-event
vocabulary exists in Python.
"""

from __future__ import annotations

from collections.abc import Sequence

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


def frames(session: _core.Session, camera: str) -> list[bytes]:
    """Every raw frame payload the loopback media plane's far end has
    received for `camera` so far, in publish order — lets tests observe
    `session.publish_frame(camera, ...)` without a real transport."""
    return session._testing_frames(camera)
