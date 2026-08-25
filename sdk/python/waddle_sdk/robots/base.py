"""The vendor-neutral half of a robot module.

A robot module (`waddle_sdk.robots.yam`, say) is a vendor's FACTS plus a driver
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
  :func:`start_console_recovery` + :class:`ConsoleRecovery` — bounded
  reporting, and the one path by which a human at the machine clears an
  e-stop latch. One reader per terminal, aimed at the arms of whoever started
  it and retired with them.
* :class:`RobotPump` + :func:`proprio_tick` — the loop that keeps reporting
  while the caller's thread is busy (blocked inside `a Metal-hosted run`, say).
* :func:`chunk_sender`, :func:`apply_decision`, :func:`split_by_part` — the
  `Control.send` verb over a set of arms, and the declared-layout arithmetic
  it routes with.
* :class:`Rig` + :class:`RigSession` — the composition, and nothing that is
  not composition: `rig.session(...)` opens the arms, registers the verbs,
  starts the console recovery and the reporting loop, holds live arms until a
  human says they are parked, and finalizes the recording on the way out.
  Every piece it composes is usable alone, and a program that wires them by
  hand gets the same session (`tests/test_site_api.py` pins that).

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
from dataclasses import dataclass, field, replace
from typing import Protocol, runtime_checkable

import numpy as np

from .._session import Control, create_core_session
from ..cameras import CameraCalibrationDriver, CameraDriver, CameraFrame, CameraSample
from ..cameras.base import _depth_preview_rgb
from ..descriptors import Camera as CameraDescription
from ..descriptors import FrameTransform, Intrinsics, Robot

__all__ = [
    "CONSOLE_THREAD_NAME",
    "PARK_WORD",
    "PARK_WORDS",
    "POSTURES",
    "RESUME_WORDS",
    "RIG_DEFAULT",
    "TWIN_KIND",
    "Arm",
    "CameraDriver",
    "CameraFrame",
    "CameraPump",
    "CameraSample",
    "CollisionSphere",
    "ConsoleRecovery",
    "CrossArm",
    "Driver",
    "ParkGate",
    "PositionVelocityDriver",
    "RejectLog",
    "Rig",
    "RigSession",
    "RobotPump",
    "SimDriver",
    "apply_console_gesture",
    "apply_decision",
    "chain_fk",
    "chunk_sender",
    "close_all",
    "closing_drops_torque",
    "console_is_at_the_machine",
    "control",
    "drives_metal",
    "estop_all",
    "hold_all",
    "hold_until_parked",
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
    print(f"[waddle_sdk.robots] {message}", flush=True)


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
        return (
            0.25 * s,
            (r[2, 1] - r[1, 2]) / s,
            (r[0, 2] - r[2, 0]) / s,
            (r[1, 0] - r[0, 1]) / s,
        )
    if r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        return (
            (r[2, 1] - r[1, 2]) / s,
            0.25 * s,
            (r[0, 1] + r[1, 0]) / s,
            (r[0, 2] + r[2, 0]) / s,
        )
    if r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        return (
            (r[0, 2] - r[2, 0]) / s,
            (r[0, 1] + r[1, 0]) / s,
            0.25 * s,
            (r[1, 2] + r[2, 1]) / s,
        )
    s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
    return (
        (r[1, 0] - r[0, 1]) / s,
        (r[0, 2] + r[2, 0]) / s,
        (r[1, 2] + r[2, 1]) / s,
        0.25 * s,
    )


@dataclass(frozen=True)
class CrossArm:
    """Where a SECOND unit's base stands in the FIRST unit's base frame:
    ``xyz`` in metres, ``rpy`` in radians.

    A rig-level fact, and the only thing that makes a cross-arm pose mean
    anything — everything downstream composes through it. It is measured at
    YOUR rig and written down; nothing infers it at run time, and a rig that
    declares none is a rig where a pose expressed in the other arm's frame
    refuses loudly rather than resolving through an identity nobody wrote.

    It is stated as rpy because that is how a bench measurement is taken and
    how a URDF states one, and converted here — once — to the **wxyz**
    quaternion this protocol pins. Handing a declaration an xyzw quaternion is
    the classic silent-corruption bug, so no caller does that conversion."""

    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]

    def __post_init__(self) -> None:
        for field_name in ("xyz", "rpy"):
            values = getattr(self, field_name)
            if len(values) != 3 or not all(math.isfinite(float(v)) for v in values):
                raise ValueError(
                    f"CrossArm.{field_name} must be three finite numbers, got "
                    f"{values!r}"
                )

    def transform(self, parent: str, child: str) -> FrameTransform:
        """This pair as the one declared edge between two named base frames."""
        return FrameTransform(
            parent=parent,
            child=child,
            position=tuple(float(v) for v in self.xyz),
            quaternion=quaternion_wxyz(rpy_matrix(*(float(v) for v in self.rpy))),
        )


# ---------------------------------------------------------------------------
# Drivers: what an arm actually is on the other side of the envelope
# ---------------------------------------------------------------------------


@runtime_checkable
class Driver(Protocol):
    """What a thing that moves must be able to do. Ten members, each
    load-bearing somewhere in this layer:

    ``kind``
        :data:`TWIN_KIND` (``"sim"``) for a twin; ``"live"``, or your own word
        for it, for anything that moves real metal. Read where the question is
        "does closing this drop torque" (:func:`closing_drops_torque`) or "is
        homing this a motion nobody is watching" (:func:`scene_reset`) — asked
        of the object that HAS the property, never of the flag that built it.
        Both of those have an unsafe answer, so the word is read in ONE
        direction (:func:`drives_metal`): ``"sim"`` alone selects the harmless
        branch, and every other word — a vendor's, yours, one this layer has
        never seen — is read as metal. A driver you wrote therefore needs no
        particular word for metal; one that is a TWIN has to say ``"sim"``, or
        it is treated as something that can hurt somebody.
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


@runtime_checkable
class PositionVelocityDriver(Protocol):
    """Optional driver extension for a known joint-velocity feedforward.

    The ordinary :class:`Driver` contract remains position-only so existing
    hardware integrations keep working unchanged.  A driver implementing
    this extension receives the already-admitted position target plus a
    trajectory producer's known velocity.  It returns ``True`` when the
    hardware accepted both and ``False`` when it deliberately degraded to a
    position-only command.

    This is never permission to estimate velocity from measurements or a
    noisy IK stream.  The producer either knows the commanded trajectory
    velocity and supplies it, or the ordinary ``write(target)`` path is used.
    """

    def write_position_velocity(
        self, target: np.ndarray, velocity_feedforward_rad_s: np.ndarray
    ) -> bool: ...


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
            f" (+{self._suppressed} more since the last line)"
            if self._suppressed
            else ""
        )
        self._report(f"envelope reject {self._subject} {reason}{tail}")
        self._suppressed = 0
        self._next = now + self._period_s


# ---------------------------------------------------------------------------
# The envelope: one seam, every command
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class CollisionSphere:
    """One conservative robot-body sphere in an arm's collision frame.

    Vendor adapters derive these spheres deterministically from joint
    positions and their shipped geometry. The SDK owns all intersection and
    refusal logic; a driver supplies geometry only.
    """

    name: str
    center_m: Sequence[float]
    radius_m: float

    def __post_init__(self) -> None:
        center = tuple(float(value) for value in self.center_m)
        radius = float(self.radius_m)
        if not self.name:
            raise ValueError("collision sphere name must be non-empty")
        if len(center) != 3 or not np.all(np.isfinite(center)):
            raise ValueError(
                f"collision sphere {self.name!r} needs three finite center coordinates"
            )
        if not math.isfinite(radius) or radius <= 0.0:
            raise ValueError(
                f"collision sphere {self.name!r} radius_m must be finite and positive"
            )
        object.__setattr__(self, "center_m", center)
        object.__setattr__(self, "radius_m", radius)


def _collision_key(first: str, second: str) -> tuple[str, str]:
    return tuple(sorted((first, second)))


def _spheres_overlap(
    first: CollisionSphere,
    second: CollisionSphere,
    *,
    margin_m: float,
) -> bool:
    distance = float(
        np.linalg.norm(
            np.asarray(first.center_m, dtype=float)
            - np.asarray(second.center_m, dtype=float)
        )
    )
    return distance <= first.radius_m + second.radius_m + margin_m


def _sphere_hits_keepout(
    sphere: CollisionSphere,
    keepout: Mapping[str, object],
) -> bool:
    center = np.asarray(sphere.center_m, dtype=float)
    margin = float(keepout.get("margin_m", 0.0))
    if keepout["kind"] == "sphere":
        obstacle = np.asarray(keepout["center"], dtype=float)
        radius = float(keepout["radius_m"])
        return float(np.linalg.norm(center - obstacle)) <= (
            sphere.radius_m + radius + margin
        )
    lower = np.asarray(keepout["min"], dtype=float)
    upper = np.asarray(keepout["max"], dtype=float)
    nearest = np.minimum(np.maximum(center, lower), upper)
    return float(np.linalg.norm(center - nearest)) <= sphere.radius_m + margin


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
        to the forward kinematics of a command and every adapter-supplied
        conservative collision sphere before it is accepted. Optional — and it
        requires ``fk``, since a box is at minimum a statement about a TCP.
    ``fk``
        ``q -> (position, rotation)`` for the first ``arm_dof`` rows, in this
        part's own base frame. OPT-IN: an arm built without it is legal and
        reports joint positions only — :meth:`ee_pose` answers ``None`` rather
        than inventing a frame, and no workspace box may be declared.
    ``collision_spheres`` / ``collision_frame``
        Deterministic conservative body geometry supplied by the driver
        adapter. The callable maps arm joint positions to named spheres. The
        SDK, not the vendor package, applies static keep-outs and self/body
        collision policy to those spheres.
    ``static_keepouts`` / ``self_collision_*``
        SDK-owned hard-safety rules compiled from ``site.yaml``. Rules are
        checked before any driver write and reject the complete command.
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
    collision_spheres: Callable[[Sequence[float]], Sequence[CollisionSphere]] | None = (
        None
    )
    collision_frame: str = ""
    static_keepouts: Sequence[Mapping[str, object]] = ()
    self_collision_enabled: bool = False
    self_collision_margin_m: float = 0.0
    self_collision_ignore_pairs: Sequence[Sequence[str]] = ()
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
        if self.workspace is not None:
            workspace = np.asarray(self.workspace, dtype=float)
            if (
                workspace.shape != (2, 3)
                or not np.all(np.isfinite(workspace))
                or np.any(workspace[0] > workspace[1])
            ):
                raise ValueError(
                    f"part {self.part!r}: workspace needs finite min/max xyz rows "
                    "with min <= max"
                )
            self.workspace = (
                tuple(float(value) for value in workspace[0]),
                tuple(float(value) for value in workspace[1]),
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
        self._ignored_collision_pairs: frozenset[tuple[str, str]] = frozenset()
        self._validate_static_safety()

    def _body_name(self, name: str) -> str:
        return f"{self.part}/{name}" if self.part and "/" not in name else name

    def _validate_static_safety(self) -> None:
        self.static_keepouts = tuple(dict(rule) for rule in self.static_keepouts)
        margin = float(self.self_collision_margin_m)
        if not math.isfinite(margin) or margin < 0.0:
            raise ValueError(
                f"part {self.part!r}: self-collision margin must be finite and non-negative"
            )
        self.self_collision_margin_m = margin
        ignored: set[tuple[str, str]] = set()
        for pair in self.self_collision_ignore_pairs:
            if len(pair) != 2:
                raise ValueError(
                    f"part {self.part!r}: ignored collision pairs need exactly two bodies"
                )
            first, second = (self._body_name(str(value)) for value in pair)
            if not first or not second or first == second:
                raise ValueError(
                    f"part {self.part!r}: ignored collision pairs need two distinct names"
                )
            ignored.add(_collision_key(first, second))
        self.self_collision_ignore_pairs = tuple(tuple(pair) for pair in ignored)
        self._ignored_collision_pairs = frozenset(ignored)
        rules_active = bool(self.static_keepouts) or self.self_collision_enabled
        if rules_active and self.collision_spheres is None:
            raise ValueError(
                f"part {self.part!r}: static hard safety requires the driver adapter "
                "to provide collision_spheres"
            )
        if rules_active and not self.collision_frame:
            raise ValueError(
                f"part {self.part!r}: static hard safety requires a collision_frame"
            )
        for rule in self.static_keepouts:
            if str(rule.get("frame") or "") != self.collision_frame:
                raise ValueError(
                    f"part {self.part!r}: keep-out {rule.get('id')!r} is in frame "
                    f"{rule.get('frame')!r}, not collision frame {self.collision_frame!r}"
                )

    def configure_static_safety(
        self,
        *,
        static_keepouts: Sequence[Mapping[str, object]],
        self_collision: Mapping[str, object],
    ) -> None:
        """Compile one site's immutable hard-safety rules onto this arm."""

        selected_keepouts = []
        for rule in static_keepouts:
            parts = tuple(str(value) for value in rule.get("parts", ()))
            if not parts or self.part in parts:
                selected_keepouts.append(rule)
        collision_parts = tuple(str(value) for value in self_collision.get("parts", ()))
        self.static_keepouts = tuple(selected_keepouts)
        self.self_collision_enabled = bool(
            self_collision.get("enabled", False)
            and (not collision_parts or self.part in collision_parts)
        )
        self.self_collision_margin_m = float(self_collision.get("margin_m", 0.0))
        self.self_collision_ignore_pairs = tuple(
            tuple(str(value) for value in pair)
            for pair in self_collision.get("ignore_pairs", ())
        )
        self._validate_static_safety()

    def collision_snapshot(
        self, target: Sequence[float]
    ) -> tuple[CollisionSphere, ...]:
        """Return validated, part-qualified body geometry for one target."""

        if self.collision_spheres is None:
            return ()
        raw = tuple(self.collision_spheres(target[: self.arm_dof]))
        spheres: list[CollisionSphere] = []
        names: set[str] = set()
        for item in raw:
            if not isinstance(item, CollisionSphere):
                raise TypeError(
                    f"part {self.part!r}: collision_spheres returned "
                    f"{type(item).__name__}, expected CollisionSphere"
                )
            name = self._body_name(item.name)
            if name in names:
                raise ValueError(
                    f"part {self.part!r}: duplicate collision body name {name!r}"
                )
            names.add(name)
            spheres.append(
                CollisionSphere(
                    name=name,
                    center_m=item.center_m,
                    radius_m=item.radius_m,
                )
            )
        return tuple(spheres)

    def _static_collision_reason(self, target: np.ndarray) -> str | None:
        if not self.static_keepouts and not self.self_collision_enabled:
            return None
        try:
            spheres = self.collision_snapshot(target)
        except Exception as error:  # noqa: BLE001 -- malformed geometry is a refusal
            return f"collision geometry unavailable ({type(error).__name__})"
        if not spheres:
            return "collision geometry provider returned no robot bodies"
        for sphere in spheres:
            for keepout in self.static_keepouts:
                if _sphere_hits_keepout(sphere, keepout):
                    return (
                        f"body {sphere.name!r} intersects static keep-out "
                        f"{keepout['id']!r}"
                    )
        if self.self_collision_enabled:
            for index, first in enumerate(spheres):
                for second in spheres[index + 1 :]:
                    pair = _collision_key(first.name, second.name)
                    if pair in self._ignored_collision_pairs:
                        continue
                    if _spheres_overlap(
                        first, second, margin_m=self.self_collision_margin_m
                    ):
                        return (
                            f"self-collision between bodies {first.name!r} and "
                            f"{second.name!r}"
                        )
        return None

    # -- reads ------------------------------------------------------------

    def state(self) -> tuple[np.ndarray, np.ndarray]:
        return self.driver.read()

    @property
    def estopped(self) -> bool:
        """Whether this part's e-stop latch is set. The ONE thing that reads a
        driver's latch: the rest of a program asks the arm."""
        return bool(self.driver.estopped)

    def ee_pose(self, position: Sequence[float] | None = None) -> np.ndarray | None:
        """xyz + wxyz quaternion of this part's TCP in its OWN base frame —
        the seven values ``report_proprio`` takes — or ``None`` when this arm
        declared no forward kinematics.

        A caller assembling one coherent observation may pass the joint sample
        it already read.  Omitting it retains the convenient standalone read.
        """
        if self.fk is None:
            return None
        sampled = (
            self.state()[0] if position is None else np.asarray(position, dtype=float)
        )
        translation, rotation = self.fk(sampled[: self.arm_dof])
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
                # Both rates, each named as what it is. The cap is a
                # PER-COMMAND number, and the speed it stands for is what the
                # owner actually chose — so a line that converted only the ask
                # would put the one number this command may NOT have directly
                # after the cap, where it reads as the cap's own allowance.
                rate = (
                    f" ({cap * self.rate_hz:.3f} per second at {self.rate_hz:g} Hz); "
                    f"this asks for {moved * self.rate_hz:.3f} per second"
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
            if self.collision_spheres is not None:
                try:
                    bodies = self.collision_snapshot(target)
                except Exception as error:  # noqa: BLE001 -- fail closed
                    return (
                        "collision geometry unavailable for workspace check "
                        f"({type(error).__name__})"
                    )
                if not bodies:
                    return "collision geometry provider returned no robot bodies"
                for body in bodies:
                    center = np.asarray(body.center_m, dtype=float)
                    lower_overshoot = lo_box - (center - body.radius_m)
                    upper_overshoot = (center + body.radius_m) - hi_box
                    if np.any(lower_overshoot > 0.0) or np.any(upper_overshoot > 0.0):
                        return (
                            f"body {body.name!r} outside the declared workspace box "
                            f"{list(np.round(lo_box, 3))}..{list(np.round(hi_box, 3))}"
                        )
        collision_reason = self._static_collision_reason(target)
        if collision_reason is not None:
            return collision_reason
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


def _cross_arm_collision_refusal(
    prepared: Sequence[tuple[Arm, np.ndarray]],
) -> tuple[Arm, str] | None:
    enabled = [
        (arm, target)
        for arm, target in prepared
        if target.size and arm.self_collision_enabled
    ]
    snapshots: list[tuple[Arm, tuple[CollisionSphere, ...]]] = []
    for arm, target in enabled:
        try:
            spheres = arm.collision_snapshot(target)
        except Exception as error:  # noqa: BLE001 -- malformed geometry is a refusal
            return (
                arm,
                f"collision geometry unavailable ({type(error).__name__})",
            )
        if not spheres:
            return (arm, "collision geometry provider returned no robot bodies")
        snapshots.append((arm, spheres))
    for index, (first_arm, first_spheres) in enumerate(snapshots):
        for second_arm, second_spheres in snapshots[index + 1 :]:
            if first_arm.collision_frame != second_arm.collision_frame:
                return (
                    first_arm,
                    (
                        "cross-part self-collision requires one shared collision frame; "
                        f"{first_arm.part!r} uses {first_arm.collision_frame!r} and "
                        f"{second_arm.part!r} uses {second_arm.collision_frame!r}"
                    ),
                )
            ignored = (
                first_arm._ignored_collision_pairs | second_arm._ignored_collision_pairs
            )
            margin = max(
                first_arm.self_collision_margin_m,
                second_arm.self_collision_margin_m,
            )
            for first in first_spheres:
                for second in second_spheres:
                    if _collision_key(first.name, second.name) in ignored:
                        continue
                    if _spheres_overlap(first, second, margin_m=margin):
                        return (
                            first_arm,
                            (
                                f"self-collision between bodies {first.name!r} and "
                                f"{second.name!r}"
                            ),
                        )
    return None


def apply_decision(
    arms: Mapping[str, Arm],
    decided,
    *,
    velocity_feedforward_rad_s: (
        Mapping[str, Sequence[float]] | Sequence[float] | None
    ) = None,
) -> bool:
    """Apply one gate decision atomically across every addressed part.

    The declared layout is resolved first, then every target is checked against
    a fresh measurement before any driver receives a write.  One refusal holds
    every addressed part and rejects the whole decision; a multi-part command
    can therefore never move its first part before discovering that its second
    part is outside the owner envelope.  The return value reports whether the
    complete decision reached the drivers.
    """
    rows = decided if isinstance(decided, dict) else split_by_part(arms, decided)
    velocity_rows: Mapping[str, Sequence[float]]
    if velocity_feedforward_rad_s is None:
        velocity_rows = {}
    elif isinstance(velocity_feedforward_rad_s, Mapping):
        velocity_rows = velocity_feedforward_rad_s
    elif isinstance(decided, dict):
        raise TypeError("a part-keyed decision needs part-keyed velocity feedforward")
    else:
        velocity_rows = split_by_part(arms, velocity_feedforward_rad_s)

    unknown_velocity_parts = set(velocity_rows) - set(rows)
    if unknown_velocity_parts:
        raise ValueError(
            "velocity feedforward names parts absent from the decision: "
            + ", ".join(sorted(unknown_velocity_parts))
        )

    prepared: list[tuple[Arm, np.ndarray, np.ndarray | None]] = []
    refusal: tuple[Arm, str] | None = None
    for part, values in rows.items():
        arm = arms[part]
        target = np.asarray(values, dtype=float).reshape(-1)
        raw_velocity = velocity_rows.get(part)
        velocity = (
            None
            if raw_velocity is None
            else np.asarray(raw_velocity, dtype=float).reshape(-1)
        )
        if velocity is not None and (
            velocity.size != target.size or not np.all(np.isfinite(velocity))
        ):
            raise ValueError(
                f"part {part!r} velocity feedforward must contain {target.size} "
                "finite values"
            )
        if target.size == 0:
            prepared.append((arm, target, velocity))
            continue
        if arm.estopped:
            reason = (
                "e-stopped — this part has no gains until the latch is cleared at "
                "the machine, so nothing here would move it"
            )
        else:
            current, _velocity = arm.state()
            reason = arm.check(target, current)
        prepared.append((arm, target, velocity))
        if reason is not None and refusal is None:
            refusal = (arm, reason)

    if refusal is None:
        refusal = _cross_arm_collision_refusal(
            [(arm, target) for arm, target, _velocity in prepared]
        )

    if refusal is not None:
        failed, reason = refusal
        failed.rejected += 1
        failed._reject(reason)
        for arm, _target, _velocity in prepared:
            arm.hold()
        return False

    for arm, target, velocity in prepared:
        if target.size == 0:
            continue
        if velocity is not None and isinstance(arm.driver, PositionVelocityDriver):
            arm.driver.write_position_velocity(target, velocity)
        else:
            arm.driver.write(target)
        arm.accepted += 1
    return True


def chunk_sender(
    arms: Mapping[str, Arm], *, report: Callable[[str], None] = status
) -> Callable[[object], None]:
    """Build the ``send`` verb over these arms.

    Waddle drives them through the returned callable, from its own dispatch
    thread, whenever something holds the lease: a teleoperator, a reset agent,
    or the hosted agent a hosted Metal run invites.

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
        velocities = getattr(chunk, "velocity_feedforwards", [None] * len(chunk.steps))
        for (values, gripper, _offset_ns), velocity in zip(
            chunk.steps, velocities, strict=True
        ):
            if gripper is not None:
                sidechannel(
                    f"carries gripper={gripper} on the sidechannel, and this robot "
                    "declares none — its hand is a joint row, so there is no channel "
                    "to apply this on; the step is refused whole rather than applied "
                    "without its hand"
                )
                continue
            apply_decision(
                arms,
                values,
                velocity_feedforward_rad_s=velocity,
            )

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


#: The one word this layer reads off a driver's ``kind``, and the only one
#: that selects a harmless branch: what :class:`SimDriver` — and any other
#: twin — carries. See :func:`drives_metal`.
TWIN_KIND = "sim"


def drives_metal(driver: Driver) -> bool:
    """Whether this driver moves something that can hurt somebody — the ONE
    place a driver's ``kind`` is read.

    Read in one direction on purpose. Both questions this layer asks of it
    have an unsafe answer — closing a live unit drops all torque, and homing
    one is an unattended motion a runbook forbids — and ``kind`` is the
    DRIVER's own word, since a driver written by a customer or a vendor is a
    supported thing to hand this layer. So :data:`TWIN_KIND` is the only word
    that means "nothing here can hurt anyone", and everything else, including
    a word this layer has never seen, is treated as metal.

    The cost is borne by the safe side: a twin whose author called it
    something else is held for a park gesture it never needed, and says so
    while it waits. The other direction would drop torque on real hardware
    with none of the warning this layer exists to give."""
    return getattr(driver, "kind", "") != TWIN_KIND


def closing_drops_torque(arms: Mapping[str, Arm]) -> bool:
    """Whether closing these drivers drops all torque — asked of the DRIVERS
    (:func:`drives_metal`), not of the flag that built them.

    A twin has nothing to hold and nothing to sag. A live unit typically holds
    its pose only while this process keeps the vendor's command re-send alive:
    close the connection and the motors' own watchdog cuts everything, gravity
    compensation included. That difference is the whole reason a finished
    mission may exit on its own in sim and may not on metal."""
    return any(drives_metal(arm.driver) for arm in arms.values())


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


def close_all(
    units: Mapping[str, Arm | Driver], *, report: Callable[[str], None] = status
) -> None:
    """Drop every one of these connections, even if an earlier one raised.

    The same doctrine as :func:`estop_all` — one unit that will not answer is
    never a reason to leave the next one open — with the opposite ending: a
    close that fails is REPORTED, never raised. Closing is what a program does
    on its way out of something that has already gone wrong (a rig that failed
    part-way through opening its arms, a session unwinding), and an exception
    from here would replace the reason it is unwinding with a footnote about a
    bus that did not answer.

    What closing COSTS is the unit's own answer, not this function's: see
    :func:`closing_drops_torque`."""
    for part, unit in units.items():
        try:
            unit.close()
        except Exception as e:  # noqa: BLE001 — a vendor call can throw anything
            report(
                f"close part={part} raised {e!r} — this unit may still be connected "
                "and energized"
            )


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

    def wait(self, timeout: float | None = None) -> bool:
        """Block until the gesture arrives (the holding side of the gate).

        Waited on rather than polled: the thread that holds a finished mission
        has nothing else to do, and a poll would put a period between the word
        being typed and the arms being closed for no reason. On the main
        thread a Ctrl-C still interrupts this, which is the other way a site
        operator ends the hold."""
        return self._released.wait(timeout)

    def wait_holding(self, timeout: float | None = None) -> bool:
        """Block until a hold BEGINS (the gesturing side of the gate).

        Something has to be able to observe the begin, because
        :meth:`confirm` before it is deliberately refused — the console reader
        answers a gesture typed early rather than remembering it, and a
        supervising thread (or a test) that means to release the hold must be
        able to wait for the hold rather than guess at when it starts."""
        return self._holding.wait(timeout)


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


#: What a console reader thread is called. One per terminal, not one per
#: session — see :class:`ConsoleRecovery`.
CONSOLE_THREAD_NAME = "waddle-robots-console"

#: The reader this process has on its console, and the lock that keeps two
#: sessions from starting a second one. Module state because the thing it
#: describes is: there is one stdin per process.
_console_lock = threading.Lock()
_console_reader: _ConsoleReader | None = None


class _ConsoleReader(threading.Thread):
    """The thread reading one input stream, and the recovery it currently
    feeds. Private: what a caller holds is the :class:`ConsoleRecovery` it was
    handed."""

    def __init__(self, stream) -> None:
        super().__init__(name=CONSOLE_THREAD_NAME, daemon=True)
        self.stream = stream
        self._lock = threading.Lock()
        self._aimed: ConsoleRecovery | None = None
        self._report: Callable[[str], None] = status

    def aim(self, recovery: ConsoleRecovery) -> None:
        with self._lock:
            self._aimed = recovery
            self._report = recovery.report

    def retire(self, recovery: ConsoleRecovery) -> None:
        """Drop this aim, waiting out a gesture already being applied — so a
        caller that retires and then closes its drivers cannot close them
        underneath a half-applied ``resume``."""
        with self._lock:
            if self._aimed is recovery:
                self._aimed = None

    def aims_at(self, recovery: ConsoleRecovery) -> bool:
        with self._lock:
            return self._aimed is recovery

    def run(self) -> None:
        for line in self.stream:
            with self._lock:
                aimed, report = self._aimed, self._report
                if aimed is not None:
                    aimed.apply(line)
                    continue
            word = line.strip()
            if word:
                report(
                    f"console: nothing is listening for {word!r} — no session is "
                    "open on this terminal right now"
                )


class ConsoleRecovery:
    """A console reader aimed at one set of arms: what
    :func:`start_console_recovery` hands back, and what retires it.

    There is one reader per TERMINAL in a process, not one per session. stdin
    is a single stream, and two threads reading it deal alternate lines to
    each other — so a word typed at the machine would reach the session that
    is running only half the time, while the other half is answered plausibly
    by a session nobody is driving. The word at stake is ``resume``, the ONE
    path that clears an owner's e-stop latch. A second session in the same
    process therefore RE-AIMS the reader the first one left.

    :meth:`retire` does not kill the thread: nothing portably interrupts a
    thread parked mid-read, and one that could would lose the line. It drops
    the aim, which is what matters — the arms and the :class:`ParkGate` this
    recovery held are released, so nothing can drive a closed session's
    drivers, and a word that arrives with nothing aimed is answered as such
    rather than swallowed."""

    def __init__(
        self,
        reader: _ConsoleReader,
        arms: Mapping[str, Arm],
        park: ParkGate | None,
        report: Callable[[str], None],
    ) -> None:
        self._reader = reader
        self._arms = arms
        self._park = park
        self.report = report

    @property
    def listening(self) -> bool:
        """Whether a word typed NOW would reach these arms — the question
        anything that offers a console gesture has to ask (there being a
        terminal is a different one; see :func:`hold_until_parked`)."""
        return self._reader.is_alive() and self._reader.aims_at(self)

    def apply(self, line: str) -> None:
        """One line, applied to the arms this recovery holds."""
        apply_console_gesture(line, self._arms, self._park, report=self.report)

    def retire(self) -> None:
        """Give the terminal back: these arms stop being what a word typed
        here reaches. Idempotent, and safe to call from anything unwinding."""
        self._reader.retire(self)

    def join(self, timeout: float | None = None) -> None:
        """Wait for the reader to reach the end of its input.

        Only ever returns at end-of-input — a terminal a human is standing at
        does not have one — so this is for a program feeding the reader a
        stream it knows will end, never for a session on its way out. Retiring
        is what a session does."""
        self._reader.join(timeout)


def start_console_recovery(
    arms: Mapping[str, Arm],
    park: ParkGate | None = None,
    *,
    report: Callable[[str], None] = status,
) -> ConsoleRecovery | None:
    """Start the ONE path that clears an e-stop latch — and the one that ends
    a finished mission: a word typed here. Returns a :class:`ConsoleRecovery`
    to RETIRE when the program that started it is done with these arms, or
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
    global _console_reader
    if not console_is_at_the_machine():
        report(
            "console: none (stdin is not a terminal in the foreground) — a latched "
            "e-stop is cleared only by supporting the machine and restarting this "
            "program"
        )
        return None
    stream = sys.stdin
    with _console_lock:
        reader = _console_reader
        # A reader that has ended (its input did) or that is parked in some
        # OTHER stream cannot carry this aim, and one that is already reading
        # this terminal must not be doubled.
        fresh = reader is None or not reader.is_alive() or reader.stream is not stream
        if fresh:
            reader = _ConsoleReader(stream)
            _console_reader = reader
        recovery = ConsoleRecovery(reader, arms, park, report)
        reader.aim(recovery)
        if fresh:
            reader.start()
    report(
        "console: type `resume` here to clear a latched e-stop (restores what the "
        f"driver snapshotted, then holds the measured pose), or `{PARK_WORD}` to "
        "release the hold this program takes when its mission ends"
    )
    return recovery


def hold_until_parked(
    arms: Mapping[str, Arm],
    park: ParkGate,
    *,
    console: ConsoleRecovery | None = None,
    report: Callable[[str], None] = status,
) -> None:
    """A finished mission on live units holds them until a human says they are
    parked. Returns immediately for anything that closing costs nothing (see
    :func:`closing_drops_torque`).

    A finished leg is not a finished session on metal. Returning from here
    goes on to shut the session down and close the drivers, which stops the
    vendor's command re-send: a fraction of a second later the motors' own
    watchdog cuts everything, gravity compensation included, and the arms sag
    from wherever the mission left them — which after an agent run is a pose
    nobody chose in advance. Every park warning this layer has is otherwise
    attached to a Ctrl-C the site operator TYPED; finishing normally has none,
    and that is the one ending nobody is standing ready for.

    So the program does not decide that moment: the operator does, with the
    same console gesture that clears an e-stop latch. The caller keeps
    reporting meanwhile — this waits, it does not stop anything — so the arms
    hold their pose, the plane keeps seeing them, and nothing is left
    half-alive while a human walks to the bench.

    ``console`` is the reader this program started
    (:func:`start_console_recovery`), and it decides which ending is OFFERED.
    The question is not "is there a terminal" but "is anybody reading it":
    a program whose stdin belongs to something else (`rig.session(...,
    console=False)`, a REPL, a supervising harness) or whose reader has
    already reached end-of-input is standing at a terminal with nobody
    listening, and sending a site operator to type a word nothing will
    receive is worse than telling them the truth at the one ending nobody is
    standing ready for. With no listener this holds until the program is
    signalled, which is the honest fallback: the alternative is dropping
    torque on a schedule nobody is watching."""
    if not closing_drops_torque(arms):
        return
    park.begin()
    report(
        "mission over — these parts are STILL HOLDING and this program is still "
        "reporting them. Closing stops the vendor's command re-send, and the "
        "motors' own watchdog then cuts ALL torque, gravity compensation "
        "included: they sag from where they are now."
    )
    if console is not None and console.listening:
        report(
            f"park or support the machine, then type `{PARK_WORD}` here to shut "
            "down (Ctrl-C does the same, once it is safe)"
        )
    else:
        report(
            "nothing is reading this program's console, so no typed gesture can "
            "reach it: it holds here until it is signalled — park or support the "
            "machine, THEN stop it (Ctrl-C)"
        )
    try:
        park.wait()
    except KeyboardInterrupt:
        # The operator answered with the other gesture they have. Ending the
        # hold is what they asked for; re-raising here would replace the
        # reason this session is closing with the answer to its own question.
        report("interrupted while holding — closing now")
    finally:
        park.end()
    report(f"{PARK_WORD} — closing")


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

    Past the latch, a twin snaps back to its declared home and anything this
    layer reads as METAL (:func:`drives_metal`) does not move: an unattended
    homing motion is exactly what a runbook forbids, and the site operator
    standing at the rig is the reset. That is stated on every episode rather
    than assumed.

    It is a default like everything else here — a rig with a scene of its own
    (a fixture to re-seed, a part feeder to advance) passes its own callable
    to `the Site lifecycle` instead."""

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
            if drives_metal(arm.driver):
                report(
                    f"pre_reset part={part}: no motion — the site operator is the reset"
                )
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
    ``a Metal-hosted run``, because the machine still moves and the agent still
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
            pose = arm.ee_pose(position)
            if pose is None:
                session.report_proprio(
                    part=part, joint_pos=position, joint_vel=velocity
                )
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
    caller's thread is elsewhere — blocked inside ``a Metal-hosted run``, or
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


class _LatestCameraSamples:
    """One correlated RGB/RGB-D sample per camera, with observable updates."""

    def __init__(self) -> None:
        self._samples: dict[str, CameraSample] = {}
        self._changed = threading.Condition()

    def clear(self) -> None:
        with self._changed:
            self._samples.clear()
            self._changed.notify_all()

    def publish(self, name: str, sample: CameraSample) -> None:
        with self._changed:
            self._samples[name] = sample
            self._changed.notify_all()

    def get(self, name: str) -> CameraSample | None:
        with self._changed:
            return self._samples.get(name)

    def wait(
        self, name: str, *, after_session_ns: int = -1, timeout: float | None = None
    ) -> CameraSample | None:
        deadline = None if timeout is None else time.monotonic() + timeout
        with self._changed:
            while True:
                sample = self._samples.get(name)
                if sample is not None and sample.session_ns > after_session_ns:
                    return sample
                if deadline is None:
                    self._changed.wait()
                    continue
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._changed.wait(remaining)


class CameraPump(threading.Thread):
    """Capture one declared camera into a latest-only, timestamped local slot.

    RGB is passed to ``Session.publish_frame`` for the existing still and media
    paths. Pixel-aligned metric depth remains in :class:`CameraSample` for local
    geometry/perception, while a deterministic RGB8 visualization of that exact
    paired plane is published on the media-only ``<camera>/depth`` track.
    """

    def __init__(
        self,
        name: str,
        description: CameraDescription,
        driver: CameraDriver,
        session,
        latest: _LatestCameraSamples,
        *,
        report: Callable[[str], None] = status,
    ) -> None:
        super().__init__(name=f"waddle-camera-{name}", daemon=True)
        self._camera_name = name
        self._description = description
        self._driver = driver
        self._session = session
        self._latest = latest
        self._report = report
        self._stopping = threading.Event()
        self._closed = False
        self._close_lock = threading.Lock()
        self._next_sequence = 0

    def run(self) -> None:
        period = 1.0 / float(self._description.fps)
        deadline = time.monotonic()
        while not self._stopping.is_set():
            try:
                frame = self._driver.capture()
                if self._stopping.is_set():
                    return
                if not isinstance(frame, CameraFrame):
                    raise TypeError("CameraDriver.capture() must return CameraFrame")
                self._next_sequence += 1
                sample = CameraSample(
                    stamp=self._session.stamp(),
                    rgb=frame.rgb,
                    depth=frame.depth,
                    frame_sequence=self._next_sequence,
                    point_resolver=frame.point_resolver,
                )
                expected = (self._description.height, self._description.width)
                if sample.rgb.shape[:2] != expected:
                    raise ValueError(
                        f"camera {self._camera_name!r} captured "
                        f"{sample.rgb.shape[1]}x{sample.rgb.shape[0]}, declaration is "
                        f"{self._description.width}x{self._description.height}"
                    )
                self._latest.publish(self._camera_name, sample)
                self._session.publish_frame(self._camera_name, sample.rgb)
                if sample.depth is not None:
                    intrinsics = self._description.intrinsics
                    self._session.publish_depth_preview(
                        self._camera_name,
                        _depth_preview_rgb(
                            sample.depth,
                            None if intrinsics is None else intrinsics.depth_scale_mm,
                        ),
                    )
            except Exception as exc:  # noqa: BLE001 — vendor capture can throw anything
                if not self._stopping.is_set():
                    self._report(
                        f"camera={self._camera_name} capture stopped after {exc!r}"
                    )
                return
            deadline += period
            self._stopping.wait(max(0.0, deadline - time.monotonic()))

    def stop(self, timeout: float = 5.0) -> None:
        self._stopping.set()
        with self._close_lock:
            if not self._closed:
                self._closed = True
                try:
                    self._driver.close()
                except Exception as exc:  # noqa: BLE001 — vendor close may throw
                    self._report(
                        f"close camera={self._camera_name} raised {exc!r} — this "
                        "camera may still be connected"
                    )
        if threading.current_thread() is not self:
            self.join(timeout=timeout)
            if self.is_alive():
                self._report(
                    f"camera={self._camera_name} capture did not stop after {timeout}s"
                )


# ---------------------------------------------------------------------------
# Composition: what a vendor's factory hands back
# ---------------------------------------------------------------------------


class _RigDefault:
    """The type of :data:`RIG_DEFAULT`."""

    __slots__ = ()

    def __repr__(self) -> str:
        return "<the rig's own default>"


#: "Whatever this rig declares for that" — distinct from ``None``, which is
#: `the Site lifecycle`'s own "declare nothing for that phase". A `RigSession` uses
#: it for ``pre_reset``: the default is the rig's scene reset, and passing
#: ``None`` is how a program declares no pre-reset at all.
RIG_DEFAULT = _RigDefault()

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
#:     one IS an intervention path: `the Site lifecycle(media=...)` — and
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
#: `a Metal-hosted run` versus `a Site Run` — never a construction one.
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
    """Build the `the driver-facing verb bundle` over these arms for one posture.

    ``posture`` is :data:`POSTURES`, which is also where what a ``monitor``
    session may and may not be wired to is written down — the choice reaches
    `the Site lifecycle` as which verbs exist, and nothing else here reads it.

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

    * ``rig.robot()`` is the `waddle_sdk.descriptors.Robot` — hand it to `the Site lifecycle`
      yourself and none of the rest of this module is involved;
    * ``rig.arms()`` builds the arms, each with the owner's envelope on it;
    * ``rig.control(arms)`` maps the posture onto verbs (and takes your own
      ``send=`` if the default envelope is not the arithmetic you want);
    * ``rig.pre_reset(arms)`` is the default scene reset — pass your own
      callable to `the Site lifecycle` instead if your scene has more to it;
    * ``rig.pump(session, arms)`` is the reporting loop, and
      :class:`RobotPump` runs any tick you write instead."""

    declaration: Robot
    build_arms: Callable[[], dict[str, Arm]]
    rate_hz: float
    posture: str = "supervised"
    estop_hardware: bool = False
    report: Callable[[str], None] = status
    build_cameras: Callable[[], dict[str, CameraDriver]] | None = None
    _camera_samples: _LatestCameraSamples = field(
        default_factory=_LatestCameraSamples, init=False, repr=False, compare=False
    )

    def __post_init__(self) -> None:
        if not isinstance(self.declaration, Robot):
            raise TypeError("Rig.declaration must be a waddle_sdk.descriptors.Robot")
        if self.posture not in POSTURES:
            raise ValueError(
                f"posture={self.posture!r}: expected one of {', '.join(POSTURES)}"
            )
        if self.rate_hz <= 0:
            raise ValueError("Rig.rate_hz must be > 0")
        if self.build_cameras is not None and not callable(self.build_cameras):
            raise TypeError("Rig.build_cameras must be callable or None")

    def robot(self) -> Robot:
        """The declaration this rig registers — the same object a vendor
        module's ``declaration()`` hands back for a hand-wired
        `the Site lifecycle`."""
        return self.declaration

    def arms(self) -> dict[str, Arm]:
        """Open the drivers and build one :class:`Arm` per declared part. The
        hardware opens HERE."""
        return self.build_arms()

    def cameras(self) -> dict[str, CameraDriver]:
        """Open this rig's optional camera drivers and validate their names.

        A builder describes every declared camera or none. Any returned driver
        is closed before a declaration/driver mismatch is raised.
        """
        if self.build_cameras is None:
            return {}
        drivers = dict(self.build_cameras())
        try:
            declared = set(self.declaration.cameras)
            actual = set(drivers)
            if actual != declared:
                raise ValueError(
                    "camera drivers must exactly match the declaration: "
                    f"declared={sorted(declared)!r}, built={sorted(actual)!r}"
                )
            for name, driver in drivers.items():
                if not isinstance(driver, CameraDriver):
                    raise TypeError(
                        f"camera driver {name!r} must provide capture() and close()"
                    )
        except BaseException:
            close_all(drivers, report=self.report)
            raise
        self._camera_samples.clear()
        return drivers

    def camera_sample(self, name: str) -> CameraSample | None:
        """Return the latest local correlated sample for a declared camera."""
        if name not in self.declaration.cameras:
            raise ValueError(f"camera {name!r} is not declared by this rig")
        return self._camera_samples.get(name)

    def wait_camera(
        self,
        name: str,
        *,
        after_session_ns: int = -1,
        timeout_s: float | None = None,
    ) -> CameraSample | None:
        """Wait for a newer local sample; ``None`` means the timeout elapsed."""
        if name not in self.declaration.cameras:
            raise ValueError(f"camera {name!r} is not declared by this rig")
        if timeout_s is not None and timeout_s < 0:
            raise ValueError("timeout_s must be >= 0 or None")
        return self._camera_samples.wait(
            name, after_session_ns=after_session_ns, timeout=timeout_s
        )

    def resolve_pixel(
        self,
        name: str,
        x: int,
        y: int,
        *,
        frame_sequence: int | None = None,
    ) -> tuple[float, float, float]:
        """Resolve one pixel against the latest aligned depth, entirely locally."""
        description = self.declaration.cameras.get(name)
        if description is None:
            raise ValueError(f"camera {name!r} is not declared by this rig")
        if description.intrinsics is None:
            raise ValueError(f"camera {name!r} declares no intrinsics")
        sample = self.camera_sample(name)
        if sample is None:
            raise RuntimeError(f"camera {name!r} has not captured a sample")
        if frame_sequence is not None and sample.frame_sequence != frame_sequence:
            raise RuntimeError(
                f"camera {name!r} frame {frame_sequence} is no longer retained; "
                f"latest is {sample.frame_sequence}"
            )
        return sample.point_at(x, y, description.intrinsics)

    def camera_pumps(
        self, session, drivers: Mapping[str, CameraDriver]
    ) -> dict[str, CameraPump]:
        """Build this rig's capture pumps. They are returned not started."""
        return {
            name: CameraPump(
                name,
                self.declaration.cameras[name],
                driver,
                session,
                self._camera_samples,
                report=self.report,
            )
            for name, driver in drivers.items()
        }

    def control(
        self, arms: Mapping[str, Arm], *, send: Callable[[object], None] | None = None
    ) -> Control:
        """This rig's posture as `the driver-facing verb bundle` verbs (see :func:`control`)."""
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

    def session(
        self,
        project: str,
        *,
        send: Callable[[object], None] | None = None,
        transport=None,
        media=None,
        recording_dir=None,
        pre_reset=RIG_DEFAULT,
        post_reset=None,
        reset_verification: str = "blocking",
        console: bool = True,
        _testing: bool = False,
    ) -> RigSession:
        """Open a :class:`RigSession` over this rig — the whole program::

            rig = yam.bimanual(workspace=..., gripper_limits=..., sim=True)
            with rig.session("my-project", transport=waddle_sdk.Grpc(url, token)) as s:
                result = a Metal-hosted run("stack the cups")

        Every keyword that is not this rig's own goes straight to
        `the Site lifecycle` and means exactly what it means there; ``send``
        REPLACES the shipped envelope (see :func:`control`), and
        ``pre_reset`` defaults to this rig's own scene reset.

        ``console`` (this rig's own, and the last one that is) starts the
        console recovery when stdin is a foreground terminal: the ONE path
        that clears an e-stop latch, and the gesture that releases the hold a
        finished mission takes on live hardware. Pass ``False`` when this
        program's stdin belongs to something else — a REPL, a supervising
        harness, another library reading it — and this session must not take
        it. Nothing then clears a latch but supporting the machine and
        restarting, and the hold at the end says exactly that instead of
        naming a gesture nothing would receive."""
        return RigSession(
            self,
            project,
            send=send,
            transport=transport,
            media=media,
            recording_dir=recording_dir,
            pre_reset=pre_reset,
            post_reset=post_reset,
            reset_verification=reset_verification,
            console=console,
            _testing=_testing,
        )


class RigSession:
    """The shared lifecycle behind ``rig.session`` and ``the Site lifecycle``.

    Hardware opens inside :meth:`_open`; every later failure closes all opened
    arms and cameras. Normal close retires local recovery, stops capture and
    proprio pumps, finalizes the core session, and only then closes the arms.
    """

    def __init__(
        self,
        rig: Rig,
        project: str,
        *,
        send: Callable[[object], None] | None = None,
        transport=None,
        media=None,
        recording_dir=None,
        pre_reset=RIG_DEFAULT,
        post_reset=None,
        reset_verification: str = "blocking",
        console: bool = True,
        _testing: bool = False,
    ) -> None:
        self._rig = rig
        self._project = project
        self._send = send
        self._init_kwargs = dict(
            transport=transport,
            media=media,
            recording_dir=recording_dir,
            post_reset=post_reset,
            reset_verification=reset_verification,
            _testing=_testing,
        )
        self._pre_reset = pre_reset
        self._console_wanted = console
        self._report = rig.report
        self._close_session: Callable[[], None] | None = None
        self._lifecycle_lock = threading.Lock()
        self._closing = False
        self._finished = False
        self._opened = False
        self.arms: dict[str, Arm] = {}
        self.cameras: dict[str, CameraDriver] = {}
        self.camera_pumps: dict[str, CameraPump] = {}
        self._robot: Robot | None = None
        self.core = None
        self.control: Control | None = None
        self.park = ParkGate()
        self.pump: RobotPump | None = None
        self.console: ConsoleRecovery | None = None

    @property
    def robot(self) -> Robot:
        """The declaration this session registered."""
        return self._robot or self._rig.robot()

    @property
    def accepted(self) -> int:
        """Commands the envelope applied, across every part."""
        return sum(arm.accepted for arm in self.arms.values())

    @property
    def rejected(self) -> int:
        """Commands the envelope refused, across every part. Refused whole."""
        return sum(arm.rejected for arm in self.arms.values())

    def camera_sample(self, name: str) -> CameraSample | None:
        return self._rig.camera_sample(name)

    def wait_camera(
        self,
        name: str,
        *,
        after_session_ns: int = -1,
        timeout_s: float | None = None,
    ) -> CameraSample | None:
        return self._rig.wait_camera(
            name, after_session_ns=after_session_ns, timeout_s=timeout_s
        )

    def resolve_pixel(
        self,
        name: str,
        x: int,
        y: int,
        *,
        frame_sequence: int | None = None,
    ) -> tuple[float, float, float]:
        description = self.robot.cameras.get(name)
        if description is None:
            raise ValueError(f"camera {name!r} is not declared by this rig")
        if description.intrinsics is None:
            raise ValueError(f"camera {name!r} declares no intrinsics")
        sample = self.camera_sample(name)
        if sample is None:
            raise RuntimeError(f"camera {name!r} has not captured a sample")
        if frame_sequence is not None and sample.frame_sequence != frame_sequence:
            raise RuntimeError(
                f"camera {name!r} frame {frame_sequence} is no longer retained; "
                f"latest is {sample.frame_sequence}"
            )
        return sample.point_at(x, y, description.intrinsics)

    def __enter__(self) -> RigSession:
        return self._open(create_core_session)

    def _open(
        self,
        open_session: Callable[..., object],
        *,
        close_session: Callable[[], None] | None = None,
    ) -> RigSession:
        """Open through one supplied core-session builder.

        ``rig.session`` supplies the public ``the Site lifecycle``/``shutdown`` pair.
        Managed ``the Site lifecycle(rig=...)`` supplies the unregistered core builder,
        so module ownership is registered only after every pump is alive.
        """
        with self._lifecycle_lock:
            if self._opened or self._finished:
                raise RuntimeError("this RigSession has already been opened")
            self._opened = True
        try:
            self.arms = self._rig.arms()
            self.cameras = self._rig.cameras()
            declarations = dict(self._rig.robot().cameras)
            changed = False
            for name, description in declarations.items():
                if description.intrinsics is not None:
                    continue
                driver = self.cameras.get(name)
                if not isinstance(driver, CameraCalibrationDriver):
                    continue
                try:
                    intrinsics = driver.intrinsics()
                except RuntimeError:
                    continue
                if not isinstance(intrinsics, Intrinsics):
                    raise TypeError(
                        f"camera {name!r} intrinsics() must return "
                        "waddle_sdk.descriptors.Intrinsics"
                    )
                declarations[name] = replace(description, intrinsics=intrinsics)
                changed = True
            declared = self._rig.robot()
            self._robot = (
                replace(declared, cameras=declarations) if changed else declared
            )
            self.control = self._rig.control(self.arms, send=self._send)
            pre_reset = (
                self._rig.pre_reset(self.arms)
                if self._pre_reset is RIG_DEFAULT
                else self._pre_reset
            )
            self.core = open_session(
                self._project,
                self.robot,
                self.control,
                pre_reset=pre_reset,
                **self._init_kwargs,
            )
            self._close_session = close_session or self.core.shutdown

            if self._console_wanted:
                self.console = start_console_recovery(
                    self.arms, self.park, report=self._report
                )
            pump = self._rig.pump(self.core, self.arms)
            pump.start()
            self.pump = pump

            for name, camera_pump in self._rig.camera_pumps(
                self.core, self.cameras
            ).items():
                camera_pump.start()
                self.camera_pumps[name] = camera_pump
            return self
        except BaseException:
            self._report(
                "this session could not open — closing the hardware that did, "
                "since nothing is being handed its drivers"
            )
            self._finish()
            raise

    def __exit__(self, exc_type, exc, tb) -> bool:
        interrupted = exc_type is not None and issubclass(exc_type, KeyboardInterrupt)
        self.close(interrupted=interrupted)
        return False

    def close(self, *, interrupted: bool = False) -> None:
        """Close once; used by context exit and module-level ``shutdown``."""
        with self._lifecycle_lock:
            if self._finished or self._closing:
                return
            self._closing = True
        try:
            if not interrupted:
                hold_until_parked(
                    self.arms, self.park, console=self.console, report=self._report
                )
        finally:
            self._finish()

    def _finish(self) -> None:
        """Stop every owner-side activity and close every opened handle once."""
        with self._lifecycle_lock:
            if self._finished:
                return
            self._finished = True
            self._closing = True

        if self.console is not None:
            self.console.retire()
        closed_cameras: set[str] = set()
        for name, camera_pump in list(self.camera_pumps.items()):
            camera_pump.stop()
            closed_cameras.add(name)
        self.camera_pumps.clear()
        if self.pump is not None:
            self.pump.stop()
            self.pump = None

        for name, driver in self.cameras.items():
            if name in closed_cameras:
                continue
            try:
                driver.close()
            except Exception as exc:  # noqa: BLE001 — vendor close can throw anything
                self._report(
                    f"close camera={name} raised {exc!r} — this camera may still "
                    "be connected"
                )

        close_session, self._close_session = self._close_session, None
        try:
            if close_session is not None:
                close_session()
        finally:
            close_all(self.arms, report=self._report)
