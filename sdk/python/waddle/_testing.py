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
