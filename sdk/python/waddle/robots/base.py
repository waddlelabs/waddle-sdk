"""The vendor-neutral half of a robot module.

A robot module (`waddle.robots.yam`, say) is a vendor's FACTS plus a driver
that speaks that vendor's bus. Everything else a program needs around those
two things is here, once:

* :class:`Driver` — the ten members a thing that moves must have. A protocol,
  not a base class: your own driver satisfies it by having them.
* :class:`SimDriver` — a rate-limited kinematic twin of any joint robot, so
  the sim run is a rehearsal of the live one rather than an easier version.
* :class:`Arm` — the ENVELOPE seam. One object every command crosses, whoever
  sent it: the program's own policy, a teleoperator's jog, a Waddle-hosted
  agent's trajectory. It rejects; it never clamps.
* :class:`RejectLog`, :class:`ParkGate`, :func:`apply_console_gesture`,
  :func:`start_console_recovery` — bounded reporting, and the one path by
  which a human at the machine clears an e-stop latch.
* :class:`RobotPump` + :func:`proprio_tick` — the loop that keeps reporting
  while the caller's thread is busy (blocked inside `waddle.agent()`, say).
* :func:`chunk_sender`, :func:`apply_decision`, :func:`split_by_part` — the
  `Control.send` verb over a set of arms, and the declared-layout arithmetic
  it routes with.

**Waddle never provides the envelope; the owner does.** What this module
ships is a parameterized default built out of the owner's own numbers —
declared joint limits, a per-step travel cap, an optional workspace box —
placed where nothing can route around it. It is a default, not a wall: a
customer who wants different arithmetic writes their own `send` callable and
keeps everything else here.

**This layer contains no authority logic.** It never asks who may command
what; it applies what the core handed it, to the part the core named, and
either accepts or refuses it on the owner's own physical grounds. Claims,
leases, handoffs and timelines are waddle-core's, exactly as they are for a
program that writes its own driver.
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable

import numpy as np

from .. import Control
from ..descriptors import Robot

__all__ = [
    "Arm",
    "Driver",
    "PARK_WORD",
    "PARK_WORDS",
    "POSTURES",
    "ParkGate",
    "RESUME_WORDS",
    "RejectLog",
    "Rig",
    "RobotPump",
    "SimDriver",
    "apply_console_gesture",
    "apply_decision",
    "chain_fk",
    "chunk_sender",
    "closing_drops_torque",
    "console_is_at_the_machine",
    "control",
    "estop_all",
    "hold_all",
    "latched_parts",
    "proprio_tick",
    "quaternion_wxyz",
    "rpy_matrix",
    "scene_reset",
    "split_by_part",
    "start_console_recovery",
    "status",
]


def status(message: str) -> None:
    """Say one line to whoever is watching this program run.

    The ONE place this layer speaks, so it can be redirected in one place:
    every function here that reports takes a ``report=`` callable and this is
    only its default. Pass a logger's ``.info`` (or ``lambda line: None``) to
    route these somewhere other than stdout — nothing here decides anything
    from what it prints, so silencing it costs only the record."""
    print(f"[waddle.robots] {message}", flush=True)


# ---------------------------------------------------------------------------
# Kinematics — generic chain math, no vendor in it
# ---------------------------------------------------------------------------


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF ``rpy`` as a rotation matrix: ``Rz(yaw) @ Ry(pitch) @ Rx(roll)``."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def chain_fk(
    origins: Sequence[Sequence[float]],
    rpys: Sequence[Sequence[float]],
    tool_xyz: Sequence[float],
    tool_rpy: Sequence[float],
    q: Sequence[float],
) -> tuple[np.ndarray, np.ndarray]:
    """Walk a serial chain of revolute joints, each turning about its own
    local **+Z**, and return ``(position, rotation)`` of the tool frame in the
    chain's base frame.

    ``origins``/``rpys`` are each joint frame's origin in its PARENT's frame,
    with the rotation applied as ``Rz(yaw) @ Ry(pitch) @ Rx(roll)`` (the URDF
    convention), and ``tool_xyz``/``tool_rpy`` are the fixed tool joint at the
    end of it. A robot whose joints do not all turn about local +Z declares
    its own forward kinematics and hands it to :class:`Arm` — this helper is
    the common case, not a requirement.
    """
    rotation = np.eye(3)
    position = np.zeros(3)
    for origin, rpy, angle in zip(origins, rpys, q, strict=True):
        position = position + rotation @ np.asarray(origin, dtype=float)
        c, s = math.cos(float(angle)), math.sin(float(angle))
        turn = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        rotation = rotation @ rpy_matrix(*rpy) @ turn
    position = position + rotation @ np.asarray(tool_xyz, dtype=float)
    rotation = rotation @ rpy_matrix(*tool_rpy)
    return position, rotation


def quaternion_wxyz(r: np.ndarray) -> tuple[float, float, float, float]:
    """Rotation matrix -> unit quaternion in **wxyz** order (w first).

    wxyz is this protocol's pinned convention, and handing it an xyzw
    quaternion is the classic silent-corruption bug — so nothing here
    shortcuts through a library's default ordering."""
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return (0.25 * s, (r[2, 1] - r[1, 2]) / s,
                (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s)
    if r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        return ((r[2, 1] - r[1, 2]) / s, 0.25 * s,
                (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s)
    if r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        return ((r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s,
                0.25 * s, (r[1, 2] + r[2, 1]) / s)
    s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
    return ((r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s,
            (r[1, 2] + r[2, 1]) / s, 0.25 * s)


# ---------------------------------------------------------------------------
# Drivers: what an arm actually is on the other side of the envelope
# ---------------------------------------------------------------------------


@runtime_checkable
class Driver(Protocol):
    """What a thing that moves must be able to do. Ten members, each
    load-bearing somewhere in this layer:

    ``kind``
        ``"sim"``, ``"live"``, or your own word for it. Read where the
        question is "does closing this drop torque" or "is homing this a
        motion nobody is watching" — asked of the object that HAS the
        property, never of the flag that built it.
    ``estopped``
        Whether the owner's stop latch is set on this unit.
    ``read()``
        ``(position, velocity)`` as arrays of the declared width. A read that
        disagrees with the declared joint list is a driver/declaration
        disagreement, and :meth:`Arm.check` names it as one — it is never
        broadcast into the envelope's arithmetic.
    ``write(target)``
        Latch a joint-position target. Called only for a command the envelope
        already admitted.
    ``hold()``
        Command where the unit already is. The reject path calls this, and so
        does Waddle's own hold verb.
    ``estop()``
        The owner's stop, and it LATCHES: after it, every write is refused
        until :meth:`re_enable`.
    ``re_enable()``
        The only exit from that latch, driven by a human at the machine.
    ``step(dt)``
        Integrate a twin by one control period. A real unit integrates itself
        and does nothing here.
    ``home(values) -> bool``
        Snap to a pose for a scene reset, answering whether it did. A live
        unit answers False: an unattended homing motion is a motion nobody is
        watching.
    ``close()``
        Drop the connection.

    A protocol rather than a base class on purpose: a driver you wrote, a
    vendor's own object with a thin shim over it, and the twin below are all
    equally admissible, and `isinstance(obj, Driver)` asks only whether the
    members are there."""

    kind: str

    @property
    def estopped(self) -> bool: ...

    def read(self) -> tuple[np.ndarray, np.ndarray]: ...

    def write(self, target: np.ndarray) -> None: ...

    def hold(self) -> None: ...

    def estop(self) -> None: ...

    def re_enable(self) -> None: ...

    def step(self, dt: float) -> None: ...

    def home(self, values: Sequence[float]) -> bool: ...

    def close(self) -> None: ...


class SimDriver:
    """A rate-limited kinematic twin of one unit: an integrator over the
    declared limits and the envelope's own per-step caps.

    Commands set a joint-position target; :meth:`step` walks the state toward
    it by at most one accepted command's worth of travel per control period
    and clamps to the declared limits. That reuse is deliberate — a twin that
    could move faster than a single accepted command allows would make the sim
    run easier than the live one, and the sim run is meant to be a rehearsal.

    Every method is safe to call from any thread: Waddle invokes the control
    verbs from its own dispatch thread while the program's loop runs on
    another, which is the concurrency a real driver faces."""

    kind = "sim"

    def __init__(
        self,
        home: Sequence[float],
        *,
        lower: Sequence[float],
        upper: Sequence[float],
        step_caps: Sequence[float],
        rate_hz: float,
    ) -> None:
        self._lower = np.asarray(lower, dtype=float)
        self._upper = np.asarray(upper, dtype=float)
        self._caps = np.asarray(step_caps, dtype=float)
        self._rate_hz = float(rate_hz)
        self._lock = threading.Lock()
        self._q = np.asarray(home, dtype=float).copy()
        self._qd = np.zeros(self._q.size)
        self._target = self._q.copy()
        self._estopped = False
        widths = {self._q.size, self._lower.size, self._upper.size, self._caps.size}
        if len(widths) != 1:
            raise ValueError(
                f"home ({self._q.size}), lower ({self._lower.size}), upper "
                f"({self._upper.size}) and step_caps ({self._caps.size}) must all "
                "describe the same joints"
            )

    @property
    def estopped(self) -> bool:
        with self._lock:
            return self._estopped

    def read(self) -> tuple[np.ndarray, np.ndarray]:
        with self._lock:
            return self._q.copy(), self._qd.copy()

    def write(self, target: np.ndarray) -> None:
        values = np.asarray(target, dtype=float).reshape(-1)
        with self._lock:
            if self._estopped:
                return
            if values.size == 0:
                # An empty row is "hold this unit, the motion is elsewhere"
                # (a gripper-only step, say). Keeping the target is what that
                # shape means; writing the empty row into it would raise, and
                # widening it to the current pose would jump on every grip.
                return
            self._target = np.clip(values, self._lower, self._upper)

    def hold(self) -> None:
        with self._lock:
            self._target = self._q.copy()
            self._qd[:] = 0.0

    def estop(self) -> None:
        with self._lock:
            self._estopped = True
            self._target = self._q.copy()
            self._qd[:] = 0.0

    def re_enable(self) -> None:
        """Clear the latch and hold where the twin is.

        A twin has no gains to put back — that is a live driver's half of this
        method — but it clears the same way and through the same gesture, so
        the recovery a site operator learns at the twin is the one they
        perform at the metal."""
        with self._lock:
            self._estopped = False
            self._target = self._q.copy()
            self._qd[:] = 0.0

    def home(self, values: Sequence[float]) -> bool:
        """Snap back to a declared pose — what "reset the scene" amounts to
        for a twin. Returns False, having moved nothing, while the e-stop is
        latched: the latch is the owner's and clearing it is a human action at
        the machine, so a reset that undid it would mean every e-stop Waddle
        asked for got cancelled by the next episode."""
        with self._lock:
            if self._estopped:
                return False
            self._q = np.asarray(values, dtype=float).copy()
            self._qd[:] = 0.0
            self._target = self._q.copy()
            return True

    def step(self, dt: float) -> None:
        with self._lock:
            if self._estopped:
                self._qd[:] = 0.0
                return
            travel = self._caps * (dt * self._rate_hz)
            delta = np.clip(self._target - self._q, -travel, travel)
            self._q = np.clip(self._q + delta, self._lower, self._upper)
            self._qd = delta / dt if dt > 0.0 else np.zeros(self._q.size)

    def close(self) -> None:
        return None


# ---------------------------------------------------------------------------
# Bounded reporting
# ---------------------------------------------------------------------------

#: How often a refused command may print, per subject, by default.
REJECT_LOG_PERIOD_S = 1.0


class RejectLog:
    """Bounded reporting for refused commands: at most one line per period,
    carrying how many were suppressed behind it.

    Rejections are a signal, and a signal that arrives at the control rate is
    noise. Nothing is dropped silently, though: the count is the record of
    what was suppressed, and it rides the next line.

    ``subject`` is what the line is about — ``part=left_arm`` for a target one
    arm refused, ``step`` for a whole step the declaration has no channel for.
    One vocabulary either way: everything this layer refuses says ``envelope
    reject``, because everything it refuses is refused for the same reason
    (nothing is clamped, narrowed, or partially applied)."""

    def __init__(
        self,
        subject: str,
        *,
        period_s: float = REJECT_LOG_PERIOD_S,
        report: Callable[[str], None] = status,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._subject = subject
        self._period_s = float(period_s)
        self._report = report
        self._clock = clock
        self._next = 0.0
        self._suppressed = 0

    def __call__(self, reason: str) -> None:
        now = self._clock()
        if now < self._next:
            self._suppressed += 1
            return
        tail = (
            f" (+{self._suppressed} more since the last line)" if self._suppressed else ""
        )
        self._report(f"envelope reject {self._subject} {reason}{tail}")
        self._suppressed = 0
        self._next = now + self._period_s


# ---------------------------------------------------------------------------
# The envelope: one seam, every command
# ---------------------------------------------------------------------------


@dataclass(kw_only=True)
class Arm:
    """One declared part: its driver, and the envelope every command crosses.

    The envelope lives HERE, on the one object every path goes through — the
    program's own policy, a teleoperator's jog, a hosted agent's trajectory —
    so there is no route to the hardware that skips it.

    The numbers are the OWNER's, and this class is arithmetic over them:

    ``joint_names``/``joint_limits``
        This part's declared rows, and the box each one may be commanded in.
    ``step_caps``
        The largest jump a SINGLE accepted command may make per row, measured
        against where the unit actually is. This is the speed cap: at the
        declared rate it bounds the unit to ``cap * rate_hz`` per second.
    ``workspace``
        ``((min_x, min_y, min_z), (max_x, max_y, max_z))`` in metres, applied
        to the forward kinematics of a command before it is accepted. Optional
        — and it requires ``fk``, since a box is a statement about a TCP.
    ``fk``
        ``q -> (position, rotation)`` for the first ``arm_dof`` rows, in this
        part's own base frame. OPT-IN: an arm built without it is legal and
        reports joint positions only — :meth:`ee_pose` answers ``None`` rather
        than inventing a frame, and no workspace box may be declared.
    ``home_values``
        Where a scene reset snaps a twin. ``None`` = this part has no home
        (which is the honest answer for most live units).

    Reject, never clamp: a target that fails any check is refused WHOLE, the
    unit is held where it is, and one line says which check refused it and by
    how much. A clamped command is a command nobody wrote, executed
    faithfully."""

    part: str
    driver: Driver
    joint_names: Sequence[str]
    joint_limits: Sequence[Sequence[float]]
    step_caps: Sequence[float]
    base_frame: str = ""
    workspace: Sequence[Sequence[float]] | None = None
    fk: Callable[[Sequence[float]], tuple[np.ndarray, np.ndarray]] | None = None
    arm_dof: int | None = None
    home_values: Sequence[float] | None = None
    rate_hz: float | None = None
    report: Callable[[str], None] = status
    accepted: int = field(default=0, init=False)
    rejected: int = field(default=0, init=False)

    def __post_init__(self) -> None:
        self.joint_names = tuple(str(n) for n in self.joint_names)
        self.joint_limits = tuple(
            (float(lo), float(hi)) for lo, hi in self.joint_limits
        )
        self.step_caps = tuple(float(c) for c in self.step_caps)
        width = len(self.joint_names)
        if len(self.joint_limits) != width:
            raise ValueError(
                f"part {self.part!r}: {len(self.joint_limits)} joint_limits for "
                f"{width} joints — one limit pair per declared joint"
            )
        if len(self.step_caps) != width:
            raise ValueError(
                f"part {self.part!r}: {len(self.step_caps)} step_caps for {width} "
                "joints — one per-step cap per declared joint"
            )
        for name, (lo, hi) in zip(self.joint_names, self.joint_limits, strict=True):
            if lo > hi:
                raise ValueError(f"joint {name!r}: lower limit {lo} above upper {hi}")
        if self.workspace is not None and self.fk is None:
            raise ValueError(
                f"part {self.part!r}: a workspace box is a statement about the TCP, "
                "and this part declared no `fk` to produce one — pass forward "
                "kinematics, or drop the box (joint limits and step caps still "
                "apply)"
            )
        if self.arm_dof is None:
            self.arm_dof = width
        if not (0 < self.arm_dof <= width):
            raise ValueError(
                f"part {self.part!r}: arm_dof={self.arm_dof} is not within its "
                f"{width} declared joints"
            )
        if self.home_values is not None:
            self.home_values = tuple(float(v) for v in self.home_values)
        self._reject = RejectLog(f"part={self.part}", report=self.report)

    # -- reads ------------------------------------------------------------

    def state(self) -> tuple[np.ndarray, np.ndarray]:
        return self.driver.read()

    @property
    def estopped(self) -> bool:
        """Whether this part's e-stop latch is set. The ONE thing that reads a
        driver's latch: the rest of a program asks the arm."""
        return bool(self.driver.estopped)

    def ee_pose(self) -> np.ndarray | None:
        """xyz + wxyz quaternion of this part's TCP in its OWN base frame —
        the seven values ``report_proprio`` takes — or ``None`` when this arm
        declared no forward kinematics."""
        if self.fk is None:
            return None
        position, _ = self.state()
        translation, rotation = self.fk(position[: self.arm_dof])
        return np.array([*translation, *quaternion_wxyz(rotation)])

    # -- the envelope -----------------------------------------------------

    def check(self, target: np.ndarray, current: np.ndarray) -> str | None:
        """``None`` when ``target`` may be applied, else why not.

        Order matters only for the message: the first failing check names
        itself and the command is refused whole either way.

        Both sides of the arithmetic are checked, not just the commanded one.
        ``current`` is whatever this part's driver just measured, and a driver
        is any object with the right members — so a read that has drifted from
        the declared joint list is the mistake this seam exists to name, and
        it is named here rather than surfacing as a broadcast error from
        somewhere inside the step-cap comparison."""
        width = len(self.joint_names)
        if target.shape != (width,):
            return (
                f"width {target.shape} — this part declares {width} joints "
                f"({', '.join(self.joint_names)})"
            )
        measured = np.asarray(current, dtype=float).reshape(-1)
        if measured.size != width:
            return (
                f"its driver measured {measured.size} joints where this part "
                f"declares {width} ({', '.join(self.joint_names)}) — the driver "
                "and the declaration disagree, so nothing here can say whether "
                "this target is one step or a jump"
            )
        if not np.all(np.isfinite(target)):
            return f"non-finite values {list(np.round(target, 4))}"
        for i, (value, (lo, hi)) in enumerate(
            zip(target, self.joint_limits, strict=True)
        ):
            if not (lo <= value <= hi):
                return (
                    f"{self.joint_names[i]}={value:.4f} outside its declared limits "
                    f"[{lo:.4f}, {hi:.4f}]"
                )
        step = np.abs(target - measured)
        for i, (moved, cap) in enumerate(zip(step, self.step_caps, strict=True)):
            if moved > cap:
                rate = (
                    f" (at {self.rate_hz:g} Hz that is {moved * self.rate_hz:.3f} per "
                    "second)"
                    if self.rate_hz
                    else ""
                )
                return (
                    f"{self.joint_names[i]} would move {moved:.4f} in one command, "
                    f"cap {cap:.4f}{rate}"
                )
        if self.workspace is not None and self.fk is not None:
            tcp, _ = self.fk(target[: self.arm_dof])
            lo_box, hi_box = (np.asarray(v, dtype=float) for v in self.workspace)
            if not (np.all(tcp >= lo_box) and np.all(tcp <= hi_box)):
                return (
                    f"tcp {list(np.round(tcp, 4))} outside the declared workspace box "
                    f"{list(np.round(lo_box, 3))}..{list(np.round(hi_box, 3))}"
                )
        return None

    def command(self, values: Sequence[float]) -> bool:
        """Apply ``values`` if the envelope admits it; otherwise HOLD and say
        why. Returns whether it was applied.

        An EMPTY value vector is the wire's "hold this part" — a step that
        addresses this part with no motion for it — and is honoured as such:
        nothing is written, nothing is refused, and the unit keeps the target
        it already had.

        A LATCHED e-stop refuses everything else. This is the one check that
        is not about the target: a stopped unit has no gains, so a command it
        "accepted" would be a command nothing executed — counted as applied,
        recorded as an action, and read downstream as a rollout that did
        something. Refusing keeps ``accepted`` meaning what it says."""
        target = np.asarray(values, dtype=float).reshape(-1)
        if target.size == 0:
            return True
        if self.estopped:
            self.rejected += 1
            self._reject(
                "e-stopped — this part has no gains until the latch is cleared at "
                "the machine, so nothing here would move it"
            )
            return False
        current, _ = self.state()
        reason = self.check(target, current)
        if reason is not None:
            self.rejected += 1
            self.driver.hold()
            self._reject(reason)
            return False
        self.accepted += 1
        self.driver.write(target)
        return True

    # -- the unit verbs ---------------------------------------------------

    def hold(self) -> None:
        self.driver.hold()

    def estop(self) -> None:
        self.driver.estop()

    def re_enable(self) -> None:
        self.driver.re_enable()

    def step(self, dt: float) -> None:
        self.driver.step(dt)

    def home(self, values: Sequence[float] | None = None) -> bool:
        """Snap to ``values``, or to this part's declared ``home_values``.
        Answers False when nothing moved — including "this part has no home",
        which is a fact about the part, not a failure."""
        target = self.home_values if values is None else values
        if target is None:
            return False
        return bool(self.driver.home(target))

    def close(self) -> None:
        self.driver.close()


# ---------------------------------------------------------------------------
# Routing: the declared layout, and what the gate hands back
# ---------------------------------------------------------------------------


def split_by_part(
    arms: Mapping[str, Arm], values: Sequence[float]
) -> dict[str, np.ndarray]:
    """A whole-robot action vector -> one row block per part, by the DECLARED
    layout (``arms`` in declaration order, each part as wide as its declared
    joints). Pure arithmetic over the declaration, never a guess."""
    flat = np.asarray(values, dtype=float).reshape(-1)
    widths = {part: len(arm.joint_names) for part, arm in arms.items()}
    total = sum(widths.values())
    if flat.size != total:
        layout = " + ".join(f"{part}:{width}" for part, width in widths.items())
        raise ValueError(
            f"a whole-robot action is {total} rows ({layout}), got {flat.size}"
        )
    rows: dict[str, np.ndarray] = {}
    offset = 0
    for part, width in widths.items():
        rows[part] = flat[offset : offset + width]
        offset += width
    return rows


def apply_decision(arms: Mapping[str, Arm], decided) -> None:
    """Apply what the gate handed back, whatever shape it came in.

    On a `Composite` declaration an intervention arrives keyed by part —
    ``{"right_arm": ndarray}`` for a step that addresses one arm, "move this
    part, hold the rest" — while a passthrough hands back the caller's own
    whole-robot vector. One rule either way: turn it into rows per part, and
    push each part's rows through that part's envelope.

    The part is the core's answer, so it is indexed, not validated: the core
    refuses an undeclared part long before this, and a name here that these
    arms do not carry means the arms and the declaration disagree — a
    construction bug in the program, surfaced as a failed verb rather than
    swallowed."""
    rows = decided if isinstance(decided, dict) else split_by_part(arms, decided)
    for part, values in rows.items():
        arms[part].command(values)


def chunk_sender(
    arms: Mapping[str, Arm], *, report: Callable[[str], None] = status
) -> Callable[[object], None]:
    """Build the ``send`` verb over these arms.

    Waddle drives them through the returned callable, from its own dispatch
    thread, whenever something holds the lease: a teleoperator, a reset agent,
    or the hosted agent :func:`waddle.agent` invites.

    A step may carry a GRIPPER value on the sidechannel, and this layer models
    a hand as a JOINT row — so there is nowhere to put one. Such a step is
    refused WHOLE and said out loud rather than applied without its hand: half
    a command nobody wrote is still a command nobody wrote. A robot that does
    declare a `Gripper` writes its own send callable and keeps the rest of
    this layer; the envelope here is a default, never a wall.

    A real controller would schedule each step at its own ``offset_ns``; this
    one retargets to the newest and lets the per-step cap cover the
    difference."""
    sidechannel = RejectLog("step", report=report)

    def send(chunk) -> None:
        for values, gripper, _offset_ns in chunk.steps:
            if gripper is not None:
                sidechannel(
                    f"carries gripper={gripper} on the sidechannel, and this robot "
                    "declares none — its hand is a joint row, so there is no channel "
                    "to apply this on; the step is refused whole rather than applied "
                    "without its hand"
                )
                continue
            apply_decision(arms, values)

    return send


def hold_all(arms: Mapping[str, Arm]) -> None:
    """Every part commands where it already is — the ``hold`` verb."""
    for arm in arms.values():
        arm.hold()


# ---------------------------------------------------------------------------
# The e-stop latch, and the human who clears it
# ---------------------------------------------------------------------------

#: The words the resume gesture answers to. Spelled out rather than matched
#: loosely: a stray line at the terminal of a program driving a robot must not
#: be able to re-energize it.
RESUME_WORDS = frozenset({"resume", "re-enable", "reenable"})

#: The word that releases the hold a finished mission takes on live hardware,
#: and the synonyms it answers to. It is a statement of fact by the site
#: operator — "the machine is parked or supported" — which is why it is
#: spelled as one.
PARK_WORD = "parked"
PARK_WORDS = frozenset({PARK_WORD, "park", "shutdown"})


def latched_parts(arms: Mapping[str, Arm]) -> list[str]:
    """The parts whose e-stop latch is set, sorted.

    The ONE place the question "is this robot stopped" is asked, because every
    caller of it makes the same decision from the answer: no episode opens, no
    command is applied, and no run that ends this way is a success."""
    return sorted(part for part, arm in arms.items() if arm.estopped)


def closing_drops_torque(arms: Mapping[str, Arm]) -> bool:
    """Whether closing these drivers drops all torque — asked of the DRIVERS,
    not of the flag that built them.

    A twin has nothing to hold and nothing to sag. A live unit typically holds
    its pose only while this process keeps the vendor's command re-send alive:
    close the connection and the motors' own watchdog cuts everything, gravity
    compensation included. That difference is the whole reason a finished
    mission may exit on its own in sim and may not on metal."""
    return any(getattr(arm.driver, "kind", "") == "live" for arm in arms.values())


def estop_all(
    arms: Mapping[str, Arm], *, report: Callable[[str], None] = status
) -> None:
    """Ask the owner's stop to fire on EVERY part, then report.

    Every part gets the call even if an earlier one raised. A loop that let
    the first failure propagate would leave the second unit energized because
    the first one's bus write timed out — which is the exact shape of "the
    e-stop worked, mostly". Failures are collected and re-raised afterwards,
    so the session still records a failed verb."""
    failures: list[str] = []
    for part, arm in arms.items():
        try:
            arm.estop()
        except Exception as e:  # noqa: BLE001 — a vendor call can throw anything
            failures.append(f"{part}: {e!r}")
    latched = latched_parts(arms)
    report(
        f"estop latched parts={','.join(latched) or 'none'} — nothing may command "
        "these parts until a human clears it at the machine"
    )
    if failures:
        raise RuntimeError("e-stop raised on " + "; ".join(failures))


def console_is_at_the_machine() -> bool:
    """Whether this program has a terminal a human is standing at.

    Two conditions, both needed. A pipe is not a person — under a harness
    there is no gesture at all. And a process that reads a TTY it is not the
    FOREGROUND of gets SIGTTIN and stops dead, which for a program holding a
    robot is worse than having no recovery: hence the process-group check."""
    try:
        stdin = sys.stdin
        if stdin is None or not stdin.isatty():
            return False
        return os.getpgrp() == os.tcgetpgrp(stdin.fileno())
    except (OSError, ValueError, AttributeError):
        return False


class ParkGate:
    """The site operator's confirmation, standing between a finished mission
    and a shutdown that drops all torque.

    Two facts rather than one, so the word cannot be typed ahead of time: a
    gesture is honoured only while the program is actually HOLDING (set for
    exactly that stretch), and it is answered either way. A ``parked``
    accepted at some quiet moment and remembered would release a hold nobody
    was standing at."""

    def __init__(self) -> None:
        self._holding = threading.Event()
        self._released = threading.Event()

    @property
    def released(self) -> bool:
        return self._released.is_set()

    def begin(self) -> None:
        self._released.clear()
        self._holding.set()

    def end(self) -> None:
        self._holding.clear()

    def confirm(self) -> bool:
        """The gesture. ``False`` when nothing is being held, so the caller
        can say so instead of arming a shutdown nobody asked for yet."""
        if not self._holding.is_set():
            return False
        self._released.set()
        return True


def apply_console_gesture(
    line: str,
    arms: Mapping[str, Arm],
    park: ParkGate | None = None,
    *,
    report: Callable[[str], None] = status,
) -> None:
    """One line typed at the terminal, applied to the arms.

    Two gestures: ``resume`` clears a latched e-stop, ``parked`` says the
    machine is safe to de-energize and releases the hold a finished mission
    takes on live hardware.

    Every outcome says something: an unknown word, a resume with nothing
    latched, a ``parked`` with nothing waiting on it, a part that refused to
    come back. A gesture that silently did nothing would be
    indistinguishable, at the rig, from a program that had stopped listening.
    A refusal is reported and the remaining parts still get their turn — one
    part that cannot recover is not a reason to leave another one floating."""
    word = line.strip().lower()
    if not word:
        return
    if word in PARK_WORDS:
        if park is not None and park.confirm():
            report("console: parked — closing now")
        else:
            report(
                f"console: nothing is waiting on {word!r} (it confirms the machine "
                "is parked, once this program has finished its mission and is "
                "holding it)"
            )
        return
    if word not in RESUME_WORDS:
        report(
            f"console: {word!r} is not a gesture this program knows (`resume` "
            f"clears a latched e-stop, `{PARK_WORD}` releases a finished mission's "
            "hold)"
        )
        return
    latched = {part: arm for part, arm in arms.items() if arm.estopped}
    if not latched:
        report("console: nothing is e-stopped")
        return
    for part, arm in sorted(latched.items()):
        try:
            arm.re_enable()
        except Exception as e:  # noqa: BLE001 — a refusal is a message, not a crash
            report(f"console: part={part} refused to re-enable: {e}")
        else:
            report(f"resume part={part} restored, holding the measured pose")


def start_console_recovery(
    arms: Mapping[str, Arm],
    park: ParkGate | None = None,
    *,
    report: Callable[[str], None] = status,
) -> threading.Thread | None:
    """Start the ONE path that clears an e-stop latch — and the one that ends
    a finished mission: a word typed here. Returns the reader thread, or
    ``None`` when there is no terminal to be told at.

    An e-stop latch is the owner's, and clearing it is a human action AT THE
    MACHINE. Not the next episode's reset — a reset that cleared it would mean
    every e-stop Waddle asked for got cancelled by the next rollout. Not the
    wire either: ``VERB_RESUME`` releases a *hold*, and hanging an owner's
    e-stop latch on it would put the last software stop the owner has on the
    supervision plane's side of the line.

    So the gesture is the terminal a live-hardware runbook already has the
    site operator standing at. With no such terminal the program says so once
    and the latch is then cleared only by supporting the machine and
    restarting — which is the honest fallback, not a degraded one."""
    if not console_is_at_the_machine():
        report(
            "console: none (stdin is not a terminal in the foreground) — a latched "
            "e-stop is cleared only by supporting the machine and restarting this "
            "program"
        )
        return None

    def reader() -> None:
        for line in sys.stdin:
            apply_console_gesture(line, arms, park, report=report)

    thread = threading.Thread(target=reader, name="waddle-robots-console", daemon=True)
    thread.start()
    report(
        "console: type `resume` here to clear a latched e-stop (restores what the "
        f"driver snapshotted, then holds the measured pose), or `{PARK_WORD}` to "
        "release the hold this program takes when its mission ends"
    )
    return thread


# ---------------------------------------------------------------------------
# The scene reset
# ---------------------------------------------------------------------------


def scene_reset(
    arms: Mapping[str, Arm], *, report: Callable[[str], None] = status
) -> Callable[[str], bool]:
    """Build the default pre-reset hook over these arms. ``True`` vouches for
    the scene; ``False`` keeps the episode out of RESETTING (the FSM aborts
    it) rather than handing a policy an invalid scene.

    A LATCHED e-stop refuses first, on every backing. That is the whole point
    of a latch: an episode that opened anyway would be Waddle asking for a
    stop and the next rollout cancelling it, and on metal it would be a
    rollout driving a unit that has no gains.

    Past the latch, a twin snaps back to its declared home and a LIVE unit
    does not move: an unattended homing motion is exactly what a runbook
    forbids, and the site operator standing at the rig is the reset. That is
    stated on every episode rather than assumed.

    It is a default like everything else here — a rig with a scene of its own
    (a fixture to re-seed, a part feeder to advance) passes its own callable
    to `waddle.init` instead."""

    def pre_reset(task: str) -> bool:
        report(f"pre_reset {task!r}")
        latched = latched_parts(arms)
        if latched:
            report(
                f"pre_reset refused: parts={','.join(latched)} e-stopped — clear the "
                "latch at the machine (`resume` here, or support the machine and "
                "restart)"
            )
            return False
        for part, arm in arms.items():
            if getattr(arm.driver, "kind", "") == "live":
                report(f"pre_reset part={part}: no motion — the site operator is the reset")
                continue
            if arm.home_values is None:
                report(f"pre_reset part={part}: no home declared — nothing to snap to")
                continue
            if not arm.home():
                report(f"pre_reset refused: part={part} would not home")
                return False
        return True

    return pre_reset


# ---------------------------------------------------------------------------
# The robot's own loop
# ---------------------------------------------------------------------------


def proprio_tick(session, arms: Mapping[str, Arm]) -> Callable[[float], None]:
    """One turn of the robot's own loop: integrate every part, then report it.

    Separate from the gate tick on purpose — this has to keep running on a
    background thread while the caller's thread is blocked inside
    ``waddle.agent()``, because the machine still moves and the agent still
    needs to see it.

    ``joint_pos`` is passed explicitly for every part. A per-part sample
    cannot ride the gate's flat ``obs`` vector: the observation layout is the
    customer's own and no declaration describes it, so slicing it by action
    parts would invent a mapping nobody declared.

    A part with no forward kinematics reports joint positions and velocities
    and nothing else — the degradation forward kinematics being opt-in buys,
    named here rather than filled in with a frame nobody declared."""

    def tick(dt: float) -> None:
        for part, arm in arms.items():
            arm.step(dt)
            position, velocity = arm.state()
            pose = arm.ee_pose()
            if pose is None:
                session.report_proprio(part=part, joint_pos=position, joint_vel=velocity)
            else:
                session.report_proprio(
                    part=part,
                    joint_pos=position,
                    joint_vel=velocity,
                    ee_pose=pose,
                    ee_pose_frame=arm.base_frame,
                )

    return tick


class RobotPump(threading.Thread):
    """Runs one tick callable at a declared rate on its own thread.

    A loop, not a robot: it knows nothing about arms, and a program with its
    own reporting (a camera to publish, a force reading to add) hands it its
    own tick. The usual one is :func:`proprio_tick`.

    It exists because the robot's own housekeeping cannot pause while the
    caller's thread is elsewhere — blocked inside ``waddle.agent()``, or
    sitting in a monitor-only session with no rollout loop at all. ``stop()``
    joins."""

    def __init__(
        self,
        tick: Callable[[float], None],
        rate_hz: float,
        *,
        name: str = "waddle-robot-pump",
    ) -> None:
        super().__init__(name=name, daemon=True)
        if rate_hz <= 0:
            raise ValueError("rate_hz must be > 0")
        self._tick = tick
        self._period = 1.0 / float(rate_hz)
        # NOT `_stop`: threading.Thread already owns that name internally, and
        # shadowing it breaks `join()`.
        self._stopping = threading.Event()

    def run(self) -> None:
        deadline = time.monotonic()
        while not self._stopping.is_set():
            self._tick(self._period)
            deadline += self._period
            self._stopping.wait(max(0.0, deadline - time.monotonic()))

    def stop(self, timeout: float = 5.0) -> None:
        self._stopping.set()
        self.join(timeout=timeout)


# ---------------------------------------------------------------------------
# Composition: what a vendor's factory hands back
# ---------------------------------------------------------------------------

#: How a session is POSTURED, and the only thing the choice touches: which
#: control verbs the session registers, and therefore which grants Waddle
#: plans against.
#:
#: ``"monitor"``
#:     Nothing may command this robot. One verb — the owner's stop — is
#:     registered, so the session says on the wire that it accepts no motion
#:     rather than accepting motion it intends to drop. This is bring-up stage
#:     one, and it is a property of the declaration rather than of a flag
#:     somebody remembered to check.
#:
#:     No ``hold`` either — and NOT because a hold would be meaningless:
#:     ``VERB_HOLD`` is "freeze safely, hold position", which is exactly what
#:     a hand-guided arm can honour. It is that waddle-core reads a registered
#:     ``hold`` as a live engage path and refuses to build any session that
#:     offers one with no ``send`` to follow it. A posture with no ``send``
#:     therefore has no room for a ``hold``, and the owner's stop is the one
#:     verb it offers the supervision side.
#:
#:     For the same reason a monitor session wires no MEDIA plane. The media
#:     plane carries the teleoperator's stream as well as the video, so wiring
#:     one IS an intervention path: `waddle.init(media=...)` — and
#:     ``_testing=True``, which is that same plane in process — refuses a
#:     session with no ``send`` verb, naming the verb rather than the posture.
#:     Watching is undiminished: ``transport=`` uplinks proprioception and each
#:     camera's declared low-rate stills over the CONTROL plane, and
#:     ``recording_dir=`` keeps the full-rate archive locally. If a
#:     teleoperator may take the machine over, that is ``posture="supervised"``
#:     — choosing between the two is the whole of that decision.
#: ``"supervised"``
#:     ``send``, ``hold`` and ``estop``: the ordinary posture, in which a
#:     teleoperator, a reset agent or a Waddle-hosted agent can drive this
#:     robot through the owner's envelope.
#:
#: A posture is NOT an authority decision and adds none: who may command a
#: robot, when, and under what claim is waddle-core's, unchanged either way.
#: Whether a rollout is agent-driven or windowed stays a call-site choice —
#: `waddle.agent()` versus `waddle.rollout()` — never a construction one.
POSTURES = ("monitor", "supervised")


def control(
    arms: Mapping[str, Arm],
    *,
    posture: str = "supervised",
    send: Callable[[object], None] | None = None,
    estop_hardware: bool = False,
    estop_latency_bound_ms: float | None = None,
    report: Callable[[str], None] = status,
) -> Control:
    """Build the `waddle.Control` over these arms for one posture.

    ``posture`` is :data:`POSTURES`, which is also where what a ``monitor``
    session may and may not be wired to is written down — the choice reaches
    `waddle.init` as which verbs exist, and nothing else here reads it.

    ``send`` REPLACES the default envelope-crossing sender
    (:func:`chunk_sender`) — the whole envelope, since that callable is where
    it lives. Waddle never provides the envelope; what this layer ships is a
    default built from the owner's own numbers, and a customer who wants
    different arithmetic passes their own callable here and keeps the twin,
    the latch, the loop and the console recovery."""
    if posture not in POSTURES:
        raise ValueError(f"posture={posture!r}: expected one of {', '.join(POSTURES)}")

    def estop() -> None:
        estop_all(arms, report=report)

    if posture == "monitor":
        if send is not None:
            raise ValueError(
                "posture='monitor' registers no send verb — nothing may command "
                "this robot — so a send callable here would be declared and never "
                "used; pass posture='supervised' if this session may be driven"
            )
        return Control(
            estop=estop,
            estop_hardware=estop_hardware,
            estop_latency_bound_ms=estop_latency_bound_ms,
        )
    return Control(
        send=send if send is not None else chunk_sender(arms, report=report),
        hold=lambda: hold_all(arms),
        estop=estop,
        estop_hardware=estop_hardware,
        estop_latency_bound_ms=estop_latency_bound_ms,
    )


@dataclass(frozen=True)
class Rig:
    """One robot module's finished product: a declaration, a way to open the
    arms behind it, and the rate they run at.

    A rig is DECLARATION ONLY until you ask it for arms — constructing one
    opens no bus and starts no thread, so a factory call is cheap, testable,
    and safe to make in a program that then decides not to run. `arms()` is
    where hardware opens, and where a failure to open it lands.

    Every piece is separately usable, which is the point of the layering:

    * ``rig.robot()`` is the `waddle.Robot` — hand it to `waddle.init`
      yourself and none of the rest of this module is involved;
    * ``rig.arms()`` builds the arms, each with the owner's envelope on it;
    * ``rig.control(arms)`` maps the posture onto verbs (and takes your own
      ``send=`` if the default envelope is not the arithmetic you want);
    * ``rig.pre_reset(arms)`` is the default scene reset — pass your own
      callable to `waddle.init` instead if your scene has more to it;
    * ``rig.pump(session, arms)`` is the reporting loop, and
      :class:`RobotPump` runs any tick you write instead."""

    declaration: Robot
    build_arms: Callable[[], dict[str, Arm]]
    rate_hz: float
    posture: str = "supervised"
    estop_hardware: bool = False
    report: Callable[[str], None] = status

    def __post_init__(self) -> None:
        if not isinstance(self.declaration, Robot):
            raise TypeError("Rig.declaration must be a waddle.Robot")
        if self.posture not in POSTURES:
            raise ValueError(
                f"posture={self.posture!r}: expected one of {', '.join(POSTURES)}"
            )
        if self.rate_hz <= 0:
            raise ValueError("Rig.rate_hz must be > 0")

    def robot(self) -> Robot:
        """The declaration this rig registers — the same object a vendor
        module's ``declaration()`` hands back for a hand-wired
        `waddle.init`."""
        return self.declaration

    def arms(self) -> dict[str, Arm]:
        """Open the drivers and build one :class:`Arm` per declared part. The
        hardware opens HERE."""
        return self.build_arms()

    def control(
        self, arms: Mapping[str, Arm], *, send: Callable[[object], None] | None = None
    ) -> Control:
        """This rig's posture as `waddle.Control` verbs (see :func:`control`)."""
        return control(
            arms,
            posture=self.posture,
            send=send,
            estop_hardware=self.estop_hardware,
            report=self.report,
        )

    def pre_reset(self, arms: Mapping[str, Arm]) -> Callable[[str], bool]:
        """The default scene reset over these arms (see :func:`scene_reset`)."""
        return scene_reset(arms, report=self.report)

    def pump(self, session, arms: Mapping[str, Arm]) -> RobotPump:
        """A :class:`RobotPump` reporting every part of these arms at this
        rig's declared rate. Not started."""
        return RobotPump(proprio_tick(session, arms), self.rate_hz)
