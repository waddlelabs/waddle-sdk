"""The I2RT YAM: what one is, in numbers, and what you can build out of one.

A robot module is facts plus a driver plus a factory, on top of
:mod:`waddle.robots.base`, and this file is all three for the YAM:

* the FACTS — the six arm joints, their limits, the kinematic chain, the tool
  frame, the hand's stroke — and the :func:`forward_kinematics` they describe;
* :class:`LiveDriver`, the thin honest layer over the vendor's own calls, with
  the e-stop latch the vendor's zero-torque mode makes necessary;
* :func:`declaration`, :func:`bimanual` and :func:`arm` — the declaration a
  hand-wired program can take on its own, and the rigs that pair it with
  drivers, the owner's envelope and a reporting loop.

Every one of those stands alone: take the declaration and wire `waddle.init`
yourself, take the driver and put your own envelope in front of it, or take a
rig and get all of it. Nothing here holds a lease or decides who may command
anything — that is waddle-core's, whichever piece you take.

**No number below stands on its own word.** The model the vendor publishes
ships beside this file (``yam_data/yam.urdf``, pinned at :data:`I2RT_PIN`)
and ``tests/test_yam_facts.py`` reads it: a declared position limit must sit
INSIDE the model's, an effort ceiling must be ``<=`` it, and every other fact
the model states — chain origins, rpys, axes, the tool frame — must match to
a nanometre. Two independent statements of the same fact, one numeric gate. A
number edited here without editing its source is a number nothing checked,
and the gate is what says so.

The facts the shipped model cannot state carry their source in the comment
instead, which is the other half of the same rule. There are three. The arm
limits are the URDF ∧ MJCF intersection: MuJoCo Menagerie's ``i2rt_yam`` is
tighter on ``joint1`` and is not shipped here, so that tightening is named in
the comment (and asserted, since the URDF's own looser number is visible).
Both hand facts — the normalized gripper row and the jaw stroke — come from
the pinned vendor tree, because the URDF carries no finger geometry at all;
their tests pin the value and the arithmetic, which catches an edit made here
but cannot catch one made upstream. Re-vendoring against :data:`I2RT_PIN` is
what catches that, and is why the pin is a fact too.

Conventions, stated once:

* Joint values are RADIANS for the six arm joints and the vendor's
  NORMALIZED 0..1 (0 = closed, 1 = open) for the seventh, which is the
  gripper. That is the layout the vendor's ``command_joint_pos`` takes, so a
  YAM part declares seven joints and needs no gripper sidechannel:
  commanding the hand is an ordinary joint command.
* Poses are METRES in the arm's own base frame (:data:`URDF_BASE_LINK`), at
  the TCP (:data:`URDF_TCP_FRAME` — the frame every YAM consumer speaks).
* No MEASUREMENT here is a site fact. The workspace box, the bench-measured
  gripper motor limits, the CAN interface and where a second arm stands
  relative to the first belong to YOUR rig, are arguments to the factories,
  and have no defaults to inherit by accident. What the factories DO default
  — a rate, a speed, part and frame names, where a twin starts — are choices
  rather than facts, are marked as such where they are written down, and are
  arguments too.

Driving a real arm needs the vendor's own package, which is not a dependency
of this one and cannot be an extra of it: it is not published on PyPI, and
the tree behind it is not something an install that only supervises a policy
should resolve. :data:`I2RT_INSTALL` is the command, built from
:data:`I2RT_PIN` so it cannot drift from the commit these facts are stated
against, and :class:`LiveDriver` prints it when the import fails. Importing
this module needs none of it.

This module is public on purpose. What a customer needs in order to drive
their own arm through their own envelope belongs in the customer's own hands,
so it ships in the open under this repo's licence; the supervision side's
in-cell material for keeping a fleet of these alive is a different artifact
and is not here.
"""

from __future__ import annotations

import math
import threading
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from importlib.resources import files

import numpy as np

from ..descriptors import (
    Camera,
    Chunking,
    Composite,
    FrameTransform,
    Joint,
    JointSpace,
    Robot,
)
from . import base
from .base import CrossArm

__all__ = [
    "ARM_JOINT_COUNT",
    "ARM_JOINT_LIMITS_RAD",
    "ARM_JOINT_NAMES",
    "BASE_FRAME",
    "CHAIN_AXIS",
    "CHAIN_ORIGIN_RPY_RAD",
    "CHAIN_ORIGIN_XYZ_M",
    "DEFAULT_MAX_GRIPPER_SPEED_PER_S",
    "DEFAULT_MAX_JOINT_SPEED_RAD_S",
    "DEFAULT_RATE_HZ",
    "DEFAULT_SIM_HOME",
    "GRIPPER_JOINT_LIMITS",
    "GRIPPER_JOINT_NAME",
    "GRIPPER_MAX_OPENING_M",
    "I2RT_INSTALL",
    "I2RT_PIN",
    "I2RT_REPO",
    "JOINT_COUNT",
    "JOINT_LIMITS",
    "JOINT_NAMES",
    "LEFT_BASE_FRAME",
    "LEFT_PART",
    "MAX_JOINT_EFFORT_NM",
    "RIGHT_BASE_FRAME",
    "RIGHT_PART",
    "TOOL_ORIGIN_RPY_RAD",
    "TOOL_ORIGIN_XYZ_M",
    "URDF_BASE_LINK",
    "URDF_TCP_FRAME",
    "ArmSite",
    "CrossArm",
    "LiveDriver",
    "arm",
    "bimanual",
    "declaration",
    "forward_kinematics",
    "urdf_text",
]

#: The upstream commit every fact in this file is pinned to, and the commit
#: the shipped ``yam_data/yam.urdf`` was vendored from. The vendor's Python
#: package is installed by the same pin (it is not on PyPI), so a module that
#: drives an arm and a model that describes one cannot drift apart silently.
I2RT_PIN = "570ef66681ff12bd8298aba34084307cfecc9f05"

#: Where that commit lives, and the ONE command that installs it.
#:
#: The vendor package is not on PyPI and cannot become a
#: ``waddle-sdk[yam]`` extra: PyPI rejects direct references, and the
#: dependency tree behind this one (an exact ``numpy``, plus a simulator
#: stack) is not something an install that only supervises a policy should
#: resolve. So it is a documented command rather than a dependency — and it
#: is BUILT from :data:`I2RT_PIN`, so the command a failure prints and the
#: commit these facts are stated against cannot drift apart.
I2RT_REPO = "https://github.com/i2rt-robotics/i2rt"
I2RT_INSTALL = f'pip install "i2rt @ git+{I2RT_REPO}@{I2RT_PIN}"'

# ---------------------------------------------------------------------------
# Names and arity
# ---------------------------------------------------------------------------

#: The six revolute joints, in the shipped URDF's chain order
#: (``base_link`` -> ``link_6``).
ARM_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")

#: The seventh row of a part's action vector. NOT a URDF joint — the shipped
#: model stops at the six arm joints plus a fixed ``grasp_link`` and carries
#: no finger geometry at all — but it IS the seventh element the vendor's
#: ``command_joint_pos`` takes, so it is a joint everywhere this module
#: speaks.
GRIPPER_JOINT_NAME = "gripper"

#: One part's declared joint vector: six arm joints, then the gripper.
JOINT_NAMES = ARM_JOINT_NAMES + (GRIPPER_JOINT_NAME,)

ARM_JOINT_COUNT = len(ARM_JOINT_NAMES)
JOINT_COUNT = len(JOINT_NAMES)

# ---------------------------------------------------------------------------
# Position limits
# ---------------------------------------------------------------------------

#: Per-joint ``(lower, upper)`` in radians — the URDF ∧ MJCF intersection, so
#: this ceiling is never optimistic against either model.
#:
#: Five rows are the shipped URDF's own ``<limit>`` values. ``joint1``'s upper
#: is the MJCF's 3.05433 rather than the URDF's 3.13: MuJoCo Menagerie's
#: ``i2rt_yam`` (derived from i2rt commit ``d4efb66d81bd8bde42909880b16591d4af82e8c0``)
#: is tighter there, and an intersection takes the smaller. That model is not
#: shipped in this wheel, so the fact gate asserts the tightening rather than
#: the source: a table "corrected" to the URDF's 3.13 fails it.
#:
#: The gate is directional — every row must sit INSIDE the shipped model's
#: interval — so tightening these for your own rig is always allowed, and
#: widening one past the hardware is what it exists to refuse.
ARM_JOINT_LIMITS_RAD = (
    (-2.61799, 3.05433),
    (0.0, 3.65),
    (0.0, 3.13),
    (-1.5708, 1.5708),
    (-1.5708, 1.5708),
    (-2.0944, 2.0944),
)

#: The gripper joint's range, in the VENDOR's normalized units: 0 = closed,
#: 1 = open. That is what the seventh element of ``command_joint_pos`` takes,
#: so its source is the vendor's Python package at :data:`I2RT_PIN` and not
#: the shipped model, which carries no finger geometry to gate it against.
#:
#: Waddle mandates no units here: ``GripperCommand.position`` is "in the
#: declared ``GripperSpec``'s open/closed units", which is why
#: :meth:`waddle.descriptors.Gripper.parallel` takes ``open`` and ``closed``
#: at all. So a YAM declaration must state ``open=1.0, closed=0.0`` — do that
#: and no conversion happens anywhere: the number a teleoperator's command
#: carries is the number the motor takes. Move these units and the
#: declaration moves with them; the wire has no opinion to violate.
GRIPPER_JOINT_LIMITS = (0.0, 1.0)

#: One part's full limit table, the layout :data:`JOINT_NAMES` declares.
JOINT_LIMITS = ARM_JOINT_LIMITS_RAD + (GRIPPER_JOINT_LIMITS,)

#: Per-joint effort ceiling (N·m), the shipped URDF's ``<limit effort="10">``.
#: Declaration only — this module commands positions and never torques — and
#: gated ``<=``, like every other ceiling here.
MAX_JOINT_EFFORT_NM = 10.0

# ---------------------------------------------------------------------------
# The kinematic chain
# ---------------------------------------------------------------------------
#
# Each entry is one revolute joint: (origin xyz metres, origin rpy radians) of
# the joint frame in its PARENT's frame, with the rotation applied as
# Rz(yaw) @ Ry(pitch) @ Rx(roll) — the URDF convention — followed by the
# joint's own rotation about its axis. Every YAM joint's axis is local +Z,
# which is why one constant replaces an axis table and why the fact gate
# asserts that instead of this file assuming it quietly.
#
# The numbers are the shipped URDF's, with the CAD export's float dust (terms
# down at 1e-13) written as the zeros they are. The gate compares them to a
# nanometre, which is four orders clear of the dust in one direction and of
# the smallest meaningful number in the table in the other.

CHAIN_ORIGIN_XYZ_M = (
    (0.0, 0.0, 0.0631),
    (2.5e-05, -0.02, 0.0409),
    (0.264, 0.0, 0.0),
    (-0.245, -0.0600003, 0.0),
    (-0.0739968, -0.0395003, 2.44738e-05),
    (0.0, 0.0353, 0.0395),
)

CHAIN_ORIGIN_RPY_RAD = (
    (0.0, 0.0, 1.5708),
    (1.5708, 0.0, 1.5708),
    (-3.14159, 0.0, 0.0),
    (0.0, 0.0, 0.0),
    (-1.5708, 1.5708, 0.0),
    (-1.5708, 0.0, 0.0),
)

#: Every joint turns about its own local +Z on this arm.
CHAIN_AXIS = (0.0, 0.0, 1.0)

#: The fixed ``grasp_joint``: ``link_6`` -> ``grasp_link``, the TCP frame
#: every consumer of a YAM pose speaks (NOT ``link_6``, which sits 90° away
#: from it — stating a tool fact in the flange's frame is how three
#: orientation bugs survived on this arm).
TOOL_ORIGIN_XYZ_M = (0.0, 0.0, 0.1347)
TOOL_ORIGIN_RPY_RAD = (0.0, 0.0, -1.5708)

#: The two frame names inside the shipped URDF, so a declaration that carries
#: that URDF and a consumer that speaks poses name the same two frames.
URDF_BASE_LINK = "base_link"
URDF_TCP_FRAME = "grasp_link"

# ---------------------------------------------------------------------------
# The hand
# ---------------------------------------------------------------------------

#: Maximum jaw separation, metres — geometrically derived from the PINNED
#: vendor's own model of this hand, since no datasheet figure is published
#: for it. ``i2rt/robot_models/gripper/linear_4310/linear_4310.xml`` models
#: the jaws as two slide joints, each ranged ``0 0.0475`` along exactly
#: opposed axes and tied by an ``<equality>`` constraint, so the separation
#: moves ``2 × 0.0475`` end to end. The same tree's
#: ``i2rt/robots/config/linear_4310.yml`` declares ``gripper_stroke: 0.096``,
#: which this is 1 mm short of — conservative against the vendor's own
#: number rather than equal to it by luck.
#:
#: This RETIRES the 0.075 m figure (2 × 0.037524, from the MuJoCo Menagerie
#: MJCF's finger range): that model was derived from i2rt commit
#: ``d4efb66d81bd8bde42909880b16591d4af82e8c0``, one hardware revision behind
#: :data:`I2RT_PIN`, and the hand changed.
#:
#: Honesty caveat: this is the full MECHANICAL stroke. A unit whose
#: bench-measured ``gripper_limits`` stop short of full travel — which is
#: normal, and which is why those are a per-unit argument — opens less than
#: this at a commanded 1.0. It is reported so a reader of a recording knows
#: what "1.0" on the gripper row is worth in millimetres, and it is not a
#: promise about your jaws.
GRIPPER_MAX_OPENING_M = 2 * 0.0475

# ---------------------------------------------------------------------------
# The shipped model
# ---------------------------------------------------------------------------


def urdf_text() -> str:
    """The vendored YAM URDF that ships with this package, as text.

    This is the model the constants above are gated against, and the model a
    single-chain declaration carries as ``kinematics_urdf`` so a consumer of
    the recording can place the arm's frames. It is read from the installed
    package, never from a path relative to this file, so it works the same
    from a wheel, an editable install and a source checkout.

    Its ``<mesh>`` references are unresolved by design: this copy is the
    kinematic contract, not a visual model, and the STLs are not shipped —
    see ``yam_data/README.md`` for the provenance, the patches and the
    vendor's MIT licence.
    """
    return (files(__package__) / "yam_data" / "yam.urdf").read_text(
        encoding="utf-8"
    )


# ---------------------------------------------------------------------------
# Forward kinematics — the declared chain, walked
# ---------------------------------------------------------------------------


def forward_kinematics(
    arm_q: Sequence[float],
) -> tuple[np.ndarray, np.ndarray]:
    """TCP ``(position, rotation)`` for the six arm joints, in the arm's own
    base frame — metres and a 3x3 rotation matrix.

    Public and OPT-IN, both deliberately. Public, because a program that
    wants a TCP pose for its own reasons — a workspace check of its own, a
    log line, a policy input — should not have to reach into a rig to get
    one. Opt-in, because an arm that is handed no forward kinematics is a
    legal arm: it reports joint positions, its proprioception carries no
    ``ee_pose``, and the features that need one degrade by NAME rather than
    by producing a pose from nowhere. Hand this function to
    :class:`~waddle.robots.base.Arm` (the YAM factories do) to have the pose.

    Takes the SIX arm joints, not the seven-row part vector: the seventh row
    is the gripper, and walking it into the chain would put the tool
    somewhere nobody commanded. The refusal is structural — the underlying
    walk zips strictly — rather than a comment asking callers to slice.
    """
    return base.chain_fk(
        CHAIN_ORIGIN_XYZ_M,
        CHAIN_ORIGIN_RPY_RAD,
        TOOL_ORIGIN_XYZ_M,
        TOOL_ORIGIN_RPY_RAD,
        arm_q,
    )


# ---------------------------------------------------------------------------
# Defaults a rig may take, and none of them a fact
# ---------------------------------------------------------------------------
#
# Everything above this line is what a YAM IS, gated against the vendor's own
# model. Everything below is what a rig built out of one may DO, and those are
# choices — conservative ones, made here so a first program has somewhere to
# start, and every one of them a factory argument. Nothing below is gated
# against anything, because there is nothing to gate a choice against.

#: The declared control rate of one part, Hz. Deliberately far below the
#: vendor's ~1 kHz servo: a rig that goes wrong at 10 Hz goes wrong ten times
#: more slowly than one at 100.
DEFAULT_RATE_HZ = 10.0

#: The joint speed a rig declares and holds itself to, rad/s. Well under the
#: arm's own ceiling — the per-step cap the envelope enforces is DERIVED from
#: this and the rate (``speed / rate_hz``), so raising one raises the other
#: and there is no pair of numbers here that can disagree with itself.
DEFAULT_MAX_JOINT_SPEED_RAD_S = 1.0

#: The same rule for the hand, in its normalized units per second. At the
#: default rate that is a quarter of full travel per accepted command: a full
#: open or close takes four commands, fast enough to be useful and slow enough
#: to stop.
DEFAULT_MAX_GRIPPER_SPEED_PER_S = 2.5

#: Where a TWIN starts, per arm, in that arm's own seven-row joint vector.
#: Two distinct rows on purpose: two twins that started identical would be
#: told apart only by their names, and which arm is which is the whole
#: index-map question a bimanual rig has to be able to answer. Live arms have
#: no home — they start wherever the site operator left them, and nothing here
#: drives one to a pose it did not receive.
DEFAULT_SIM_HOME = (
    (0.20, 1.00, 1.00, 0.10, -0.50, 0.05, 0.00),
    (-0.20, 1.10, 0.90, -0.10, -0.40, -0.05, 0.20),
)

#: The part names a bimanual rig declares. Declaration order IS the layout of
#: the concatenated action vector, so ``left_arm`` occupies rows 0..6 and
#: ``right_arm`` rows 7..13 — everywhere, for everyone, including whatever
#: maps a teleoperator's station onto an arm.
LEFT_PART = "left_arm"
RIGHT_PART = "right_arm"

#: Default frame names, one per arm's base. Deliberately not "base": a frame
#: name is spoken, and "base" means *whose* base. A site with its own naming
#: passes its own through :class:`ArmSite`.
LEFT_BASE_FRAME = "yam_left_base"
RIGHT_BASE_FRAME = "yam_right_base"
BASE_FRAME = "yam_base"


@dataclass(frozen=True)
class ArmSite:
    """One arm's SITE facts — the things that are true of YOUR unit on YOUR
    bench, and of no other.

    Every field defaults to "the rig-level answer", never to a number: an arm
    that needs its own says so.

    ``channel``
        The SocketCAN interface this arm is on (``can_left``, ``can0``, ...).
        Required when the rig is live; ignored, and permitted, in sim — so one
        call site can carry both configurations and the program text does not
        change across the flip.
    ``base_frame``
        What this arm's base is called in the frames the rig declares.
    ``gripper_limits``
        This unit's own bench-measured ``[closed, open]`` in MOTOR RADIANS,
        when it differs from the rig's. Hands vary between units; that is the
        whole reason this is a measurement and not a constant.
    ``sim_home``
        Where this arm's twin starts. Live arms have no home.
    """

    channel: str | None = None
    base_frame: str | None = None
    gripper_limits: Sequence[float] | None = None
    sim_home: Sequence[float] | None = None


# ---------------------------------------------------------------------------
# The live driver: one real YAM on one CAN bus
# ---------------------------------------------------------------------------


class LiveDriver:
    """One real YAM on one CAN bus — the thin vendor calls and the latch.

    Deliberately thin. What it reproduces is only what the vendor package
    requires, and the vendor package is imported LAZILY, inside ``__init__``:
    importing :mod:`waddle.robots.yam` on a machine that has never seen a YAM
    is an ordinary import, and only asking for a live arm requires the
    vendor's code to be there.

    * ``get_yam_robot(channel, gripper_type, zero_gravity_mode,
      gripper_limits_override)`` — the override is PINNED from the site's own
      measurement, which is what skips the connect-time auto-calibration that
      physically drives the jaws (~0.5 N·m, up to 2 s per direction) on every
      connect. Nothing here auto-ranges a hand.
    * ``robot.command_joint_pos(vec)`` takes the FULL vector: six arm joints
      in radians plus the gripper normalized 0..1. It is non-blocking — it
      latches a setpoint into the vendor's own ~1 kHz server thread, which
      re-sends it forever — so there is no keepalive to write here.
    * ``robot.get_observations()`` -> ``joint_pos`` (6, rad), ``joint_vel``
      (6), ``gripper_pos`` ([1], normalized). Read defensively: an absent
      velocity is reported as zero because the wire has no "unknown" for one,
      and an absent POSITION is a fault, not a guess — the hand included,
      since this module declares it as a joint row rather than a sidechannel.
      A fabricated 0.0 there would be recorded as a measured closed hand, and
      it is the number the envelope measures the next command's per-step cap
      against, so guessing it would let a large uncommanded jaw motion through
      the check that exists to refuse one.
    * ``robot.zero_torque_mode()`` is the stop the vendor offers, and it is
      HONEST about what it is: the arm goes compliant and FLOATS under the
      always-on gravity compensation. It does not freeze in place. The site's
      physical e-stop is the real one. It also zeros the internal **kp/kd**,
      and no vendor call undoes that on its own — hence the latch.
    * ``robot.get_robot_info()`` -> a dict carrying those ``kp``/``kd``
      arrays, and ``robot.update_kp_kd(kp, kd)`` puts them back. They are
      snapshotted at construction, because by the time you want them the stop
      has already destroyed them.

    **The latch is the load-bearing part.** After ``zero_torque_mode()`` the
    vendor's server thread happily accepts every ``command_joint_pos`` and the
    arm does not move: gains of zero make a setpoint a suggestion. A driver
    without a latch therefore reports commands as applied while the arm hangs
    limp, and every episode after the stop reads SUCCESS. So the latch is set
    with the stop, every write is refused while it holds, and the one way out
    is :meth:`re_enable` — gains back, measured pose held — driven by a human
    at the machine (`waddle.robots.base.start_console_recovery`), never by the
    wire and never by the next episode's reset.

    ``zero_gravity=True`` builds the arm compliant and hand-movable, and this
    driver then refuses to write at all — so "nothing can command it" is a
    property of the object rather than of a flag somebody remembered to check.
    It is what ``posture="monitor"`` builds.

    What this driver deliberately does NOT do: bring a CAN interface up, patch
    the vendor's transport, or work around a bus that starves its receiver.
    Those are fleet-keeping concerns, they are specific to how a site cables
    and loads its machines, and they belong to whoever runs the fleet — not to
    a driver whose job is to be the honest thin layer over the vendor's own
    calls.
    """

    kind = "live"

    def __init__(
        self,
        channel: str,
        *,
        gripper_limits: Sequence[float],
        zero_gravity: bool = False,
        report: Callable[[str], None] = base.status,
    ) -> None:
        try:
            from i2rt.robots.get_robot import get_yam_robot
            from i2rt.robots.utils import GripperType
        except ImportError as e:
            raise RuntimeError(
                f"{channel}: driving a real YAM needs the I2RT vendor package, and "
                f"importing it raised {e!r}.\n\n"
                f"    {I2RT_INSTALL}\n\n"
                "It is not a dependency of this package and cannot be an extra of "
                "one: it is not published on PyPI, and the tree behind it (an "
                "exact numpy, a simulator stack) is not something an install that "
                "only supervises a policy should resolve. The commit above is the "
                "same one every fact in this module is stated against."
            ) from e

        self.channel = channel
        self._zero_gravity = bool(zero_gravity)
        self._report = report
        self._lock = threading.Lock()
        self._estopped = False
        self._robot = get_yam_robot(
            channel=channel,
            gripper_type=GripperType.LINEAR_4310,
            zero_gravity_mode=self._zero_gravity,
            gripper_limits_override=np.asarray(gripper_limits, dtype=float),
        )
        try:
            dofs = int(self._robot.num_dofs())
            if dofs != JOINT_COUNT:
                raise RuntimeError(
                    f"{channel}: the arm reports {dofs} DOF, this module declares "
                    f"{JOINT_COUNT} ({', '.join(JOINT_NAMES)}) — reject, never adapt "
                    "silently"
                )
            self._default_kp, self._default_kd = self._snapshot_gains()
        except BaseException:
            # The bus is already open by the time anything here can refuse, and
            # a constructor that raises hands its caller an exception instead of
            # a driver — so nothing will ever hold this handle again, while the
            # vendor's own ~1 kHz server thread is already running against it.
            # Close it here or the arm stays energized and unreachable.
            self._report(
                f"live {self.channel}: refused after the bus opened — closing it"
            )
            try:
                self.close()
            except Exception as e:  # noqa: BLE001 — the refusal is the news, not this
                self._report(
                    f"live {self.channel}: close() raised {e!r} while backing out of "
                    "a failed open — this arm may still be connected and energized"
                )
            raise

    def _snapshot_gains(self):
        """The PD gains this arm was built with, read ONCE and kept.

        Read here and nowhere else because the e-stop destroys them: by the
        time :meth:`re_enable` wants them there is nothing left to ask. An arm
        built compliant has no stiff gains to snapshot and commands nothing
        anyway, so it does not pretend to. Failure is loud and non-fatal — the
        arm still runs, and :meth:`re_enable` refuses rather than guessing."""
        if self._zero_gravity:
            return None, None
        try:
            info = self._robot.get_robot_info() or {}
        except Exception as e:  # noqa: BLE001 — best effort; re_enable degrades loudly
            self._report(
                f"live {self.channel}: get_robot_info() raised {e!r} — gains not "
                "snapshotted; re_enable will refuse to guess them"
            )
            return None, None
        kp, kd = info.get("kp"), info.get("kd")
        if kp is None or kd is None:
            self._report(
                f"live {self.channel}: get_robot_info() carried no kp/kd — "
                "re_enable will refuse to restore gains it never snapshotted"
            )
        return kp, kd

    @property
    def estopped(self) -> bool:
        return self._estopped

    def read(self) -> tuple[np.ndarray, np.ndarray]:
        obs = self._robot.get_observations() or {}
        joint_pos = obs.get("joint_pos")
        if joint_pos is None:
            raise RuntimeError(
                f"{self.channel}: get_observations() carried no joint_pos"
            )
        gripper = obs.get("gripper_pos")
        if gripper is None or np.size(gripper) == 0:
            raise RuntimeError(
                f"{self.channel}: get_observations() carried no gripper_pos — this "
                "module declares the hand as a JOINT row, so the same rule binds it "
                "as the other six: a zero here would be recorded as a closed hand, "
                "and it is the position the envelope measures the next command's "
                "per-step cap against, so a large uncommanded jaw motion would pass "
                "the check that exists to refuse one"
            )
        position = np.zeros(JOINT_COUNT)
        position[:ARM_JOINT_COUNT] = np.asarray(joint_pos, dtype=float)[
            :ARM_JOINT_COUNT
        ]
        position[ARM_JOINT_COUNT] = float(np.reshape(gripper, -1)[0])
        velocity = np.zeros(JOINT_COUNT)
        raw_vel = obs.get("joint_vel")
        if raw_vel is not None and np.size(raw_vel) > 0:
            velocity[:ARM_JOINT_COUNT] = np.asarray(raw_vel, dtype=float)[
                :ARM_JOINT_COUNT
            ]
        return position, velocity

    def write(self, target: np.ndarray) -> None:
        if self._zero_gravity:
            raise RuntimeError(
                f"{self.channel}: this arm was opened in zero-gravity mode "
                "(compliant, hand movable) and this driver commands nothing"
            )
        with self._lock:
            if self._estopped:
                raise RuntimeError(
                    f"{self.channel}: e-stopped — zero_torque_mode() left this arm "
                    "with no gains, so a command here would latch a setpoint that "
                    "moves nothing. Clear the latch at the machine."
                )
            self._robot.command_joint_pos(np.asarray(target, dtype=float))

    def hold(self) -> None:
        if self._zero_gravity:
            return
        position, _ = self.read()
        with self._lock:
            if self._estopped:
                return
            self._robot.command_joint_pos(np.asarray(position, dtype=float))

    def estop(self) -> None:
        """Zero torque, and LATCH.

        The latch is set BEFORE the vendor call, so a call that raises still
        leaves this arm refusing commands: a stop that half-happened is still
        a stop, and the one thing that must not follow it is a program that
        believes it can drive again."""
        with self._lock:
            self._estopped = True
            self._robot.zero_torque_mode()

    def re_enable(self) -> None:
        """Gains back, measured pose held — the only exit from the latch.

        Refuses rather than guesses. Gains this driver never snapshotted are
        gains nobody knows, and a made-up kp is how an arm slams; a refusal
        leaves the latch set and the arm floating, which is the state the site
        operator can already see. Everything runs under this driver's own
        lock, so nothing observes half a recovery, and the latch clears LAST:
        a vendor call that raises leaves this arm e-stopped."""
        with self._lock:
            if self._zero_gravity:
                raise RuntimeError(
                    f"{self.channel}: this arm was opened in zero-gravity mode — it "
                    "commands nothing, so there is nothing to re-enable"
                )
            if self._default_kp is None or self._default_kd is None:
                raise RuntimeError(
                    f"{self.channel}: no snapshotted kp/kd (get_robot_info had none "
                    "at connect) — refusing to guess gains. Support the arm and "
                    "restart the program."
                )
            position, _ = self.read()
            self._robot.update_kp_kd(self._default_kp, self._default_kd)
            self._robot.command_joint_pos(np.asarray(position, dtype=float))
            self._estopped = False

    def step(self, dt: float) -> None:
        return None  # a real arm integrates itself

    def home(self, values: Sequence[float]) -> bool:
        """A live arm has no home to snap to. Homing one is a motion, and an
        unattended motion is exactly what a live rig does not make."""
        return False

    def close(self) -> None:
        with self._lock:
            self._robot.close()


# ---------------------------------------------------------------------------
# The declaration
# ---------------------------------------------------------------------------


def _part_space(
    rate_hz: float,
    max_joint_speed_rad_s: float,
    joint_limits: Sequence[Sequence[float]] = JOINT_LIMITS,
) -> JointSpace:
    """One arm: six joints plus the gripper row, at the declared rate."""
    return JointSpace(
        joints=[
            Joint(
                name=name,
                min_position=lo,
                max_position=hi,
                max_velocity=max_joint_speed_rad_s,
                max_effort=MAX_JOINT_EFFORT_NM,
            )
            for name, (lo, hi) in zip(JOINT_NAMES, joint_limits, strict=True)
        ],
        rate_hz=rate_hz,
        # One action per tick, replaced as soon as the next arrives.
        chunking=Chunking(horizon=1, replan="immediate", interp="hold"),
    )


def declaration(
    *,
    parts: Sequence[str] | None = None,
    name: str = "yam",
    robot_id: str = "",
    cell_id: str = "",
    rate_hz: float = DEFAULT_RATE_HZ,
    max_joint_speed_rad_s: float = DEFAULT_MAX_JOINT_SPEED_RAD_S,
    joint_limits: Sequence[Sequence[float]] = JOINT_LIMITS,
    base_frame: str = BASE_FRAME,
    declare_urdf: bool | None = None,
    frames: Sequence[FrameTransform] = (),
    cameras: Mapping[str, Camera] | None = None,
) -> Robot:
    """Everything Waddle needs to know about a rig of YAMs — and nothing else.

    Public and standing alone on purpose. The factories below build one of
    these, but a program that wants none of the rest of this module — its own
    driver, its own loop, a plain ``waddle.init`` — should not have to reach
    into a rig to get the declaration, and should get exactly the one a
    factory would have registered.

    ``parts``
        The part names, in DECLARATION ORDER, which is the layout of the
        concatenated action vector. ``None`` (the default) declares a bare
        joint space with no named parts — one arm, addressed as the whole
        robot.
    ``joint_limits``
        The interval each row accepts, defaulting to the shipped model's
        ``JOINT_LIMITS``. Pass a rig's own when it differs from the model
        (see :func:`bimanual`): what the declaration carries and what the
        envelope enforces are then the same numbers, which is the only way a
        teleoperator or an agent is shown the range this rig really has.
    ``declare_urdf``
        Whether to carry the shipped model as ``kinematics_urdf``. Defaults to
        "yes if this declaration describes ONE chain". A URDF field describes
        one chain and a multi-part rig has several, so declaring one there
        would name a second arm's tool frame as something it is not — asked
        for explicitly, that is refused rather than silently dropped.
    ``base_frame``
        What this arm's base is called. It reaches the declaration only when
        the model is carried: the model's own root link is
        ``base_link``, and a consumer handed both a model and poses in another
        frame name has two unrelated trees unless something says they are the
        same frame. So the rename is declared as the identity edge it is.
    """
    part_names = tuple(parts) if parts is not None else ()
    if part_names:
        action_space = Composite(
            rate_hz=rate_hz,
            chunking=Chunking(horizon=1, replan="immediate", interp="hold"),
            **{
                part: _part_space(rate_hz, max_joint_speed_rad_s, joint_limits)
                for part in part_names
            },
        )
    else:
        action_space = _part_space(rate_hz, max_joint_speed_rad_s, joint_limits)

    one_chain = len(part_names) <= 1
    if declare_urdf is None:
        declare_urdf = one_chain
    elif declare_urdf and not one_chain:
        raise ValueError(
            f"declare_urdf=True with {len(part_names)} parts: a kinematics_urdf "
            "field describes one chain, and naming this one would name every "
            "other part's tool frame as something it is not — declare each part's "
            "own frames instead (each arm reports its own ee_pose in its own base "
            "frame)"
        )

    declared_frames = tuple(frames)
    if declare_urdf and base_frame != URDF_BASE_LINK:
        # Not an identity nobody wrote: the arm's base IS the model's root
        # link, and this states that rename so a consumer can compose the two.
        declared_frames += (
            FrameTransform(parent=base_frame, child=URDF_BASE_LINK),
        )

    return Robot(
        name=name,
        robot_id=robot_id,
        cell_id=cell_id,
        action_space=action_space,
        cameras=dict(cameras or {}),
        kinematics_urdf=urdf_text().encode("utf-8") if declare_urdf else None,
        frames=declared_frames,
    )


# ---------------------------------------------------------------------------
# The factories
# ---------------------------------------------------------------------------


def _checked_gripper_limits(pair: Sequence[float], where: str) -> tuple[float, float]:
    """The bench-measured ``[closed, open]`` pair, validated for SHAPE.

    Required even in sim, and validated the same way there, so the program
    text is identical across the sim->live flip; only the live driver reads
    the values."""
    try:
        values = tuple(float(v) for v in pair)
    except TypeError:
        values = ()
    if len(values) != 2 or not all(math.isfinite(v) for v in values):
        raise ValueError(
            f"{where} gripper_limits={pair!r}: expected a finite (closed, open) "
            "pair in MOTOR RADIANS, measured at YOUR bench — the reference rig's "
            "pair is not yours, and pinning it is what skips the connect-time "
            "auto-calibration that drives the jaws"
        )
    if not values[0] < values[1]:
        raise ValueError(
            f"{where} gripper_limits={pair!r}: closed ({values[0]}) must be below "
            f"open ({values[1]})"
        )
    return values


def _checked_joint_limits(
    limits: Sequence[Sequence[float]] | None,
    where: str,
    *,
    report: Callable[[str], None],
) -> tuple[tuple[float, float], ...]:
    """The intervals THIS rig accepts, defaulting to the shipped model's.

    `JOINT_LIMITS` describes a YAM as the vendor's model states it; a rig is a
    particular machine. The two differ for reasons that are facts about the
    machine and not about the model — a motor zeroed a few milliradians off
    reads (and rests) just outside a theoretical range, and a hold that echoes
    that reading would be a command its own envelope refuses forever. So the
    interval is the owner's to state, and stating it is not editing the model.

    Widening past the model is legal and LOUD: every row that reaches beyond
    the shipped interval is reported by name and by how far. Nothing here
    clamps and nothing here quietly accepts — this is the number the envelope
    will judge every command by, and the same number the declaration carries
    to the plane, so a teleoperator and a Waddle-hosted agent are shown the
    range this rig really has."""
    if limits is None:
        return JOINT_LIMITS
    try:
        rows = tuple(tuple(float(v) for v in row) for row in limits)
    except TypeError:
        rows = ()
    if len(rows) != JOINT_COUNT or any(len(row) != 2 for row in rows):
        raise ValueError(
            f"{where} joint_limits={limits!r}: expected {JOINT_COUNT} (lower, "
            f"upper) pairs, one per row of a YAM part "
            f"({', '.join(JOINT_NAMES)})"
        )
    for name, row in zip(JOINT_NAMES, rows, strict=True):
        if not all(math.isfinite(v) for v in row) or not row[0] < row[1]:
            raise ValueError(
                f"{where} joint_limits: {name}={row!r} is not a finite (lower, "
                "upper) interval with lower below upper"
            )
    widened = [
        f"{name} [{row[0]:.4f}, {row[1]:.4f}] vs the model's "
        f"[{model[0]:.4f}, {model[1]:.4f}]"
        for name, row, model in zip(JOINT_NAMES, rows, JOINT_LIMITS, strict=True)
        if row[0] < model[0] or row[1] > model[1]
    ]
    if widened:
        report(
            f"{where}: this rig declares joint limits WIDER than the shipped "
            f"model — {'; '.join(widened)}. That is the owner's call to make "
            "and it is now the envelope this program enforces and the range it "
            "declares; a motor whose zero is off is cured properly by re-zeroing "
            "it (the vendor's set_zero.py), not by the margin"
        )
    return rows


def _checked_workspace(
    box: Sequence[Sequence[float]] | None,
    *,
    report: Callable[[str], None],
) -> tuple[tuple[float, float, float], tuple[float, float, float]] | None:
    if box is None:
        report(
            "no workspace box declared — the declared joint limits and per-step "
            "caps still bound every command, and the TCP is unbounded"
        )
        return None
    try:
        corners = tuple(tuple(float(v) for v in corner) for corner in box)
    except TypeError:
        corners = ()
    if len(corners) != 2 or any(len(corner) != 3 for corner in corners):
        raise ValueError(
            f"workspace={box!r}: expected ((min_x, min_y, min_z), (max_x, max_y, "
            "max_z)) in metres, in the arm's own base frame"
        )
    if any(lo > hi for lo, hi in zip(*corners, strict=True)):
        raise ValueError(
            f"workspace={box!r}: every minimum must be at or below its maximum"
        )
    return corners  # type: ignore[return-value]


def _step_caps(
    rate_hz: float, max_joint_speed_rad_s: float, max_gripper_speed_per_s: float
) -> tuple[float, ...]:
    """The largest jump a SINGLE accepted command may make, per row.

    DERIVED from the declared speeds and rate rather than stated beside them:
    one number cannot then disagree with the other, and the declaration a
    teleoperator reads (``Joint.max_velocity``) is the same statement the
    envelope enforces."""
    if rate_hz <= 0:
        raise ValueError("rate_hz must be > 0")
    if max_joint_speed_rad_s <= 0 or max_gripper_speed_per_s <= 0:
        raise ValueError("the declared speeds must be > 0")
    return (max_joint_speed_rad_s / rate_hz,) * ARM_JOINT_COUNT + (
        max_gripper_speed_per_s / rate_hz,
    )


def _resolved_site(
    site: ArmSite | None,
    *,
    where: str,
    sim: bool,
    base_frame: str,
    gripper_limits: tuple[float, float],
    sim_home: Sequence[float],
) -> ArmSite:
    """One arm's site facts with every rig-level default filled in, and the
    one thing a live arm cannot do without refused here — at the factory call,
    where the program can still be fixed, rather than at the bus."""
    site = site if site is not None else ArmSite()
    channel = site.channel
    if not sim and not channel:
        raise ValueError(
            f"{where}: a live rig needs a channel (the SocketCAN interface this "
            "arm is on) — pass it, or pass sim=True. Nothing here reaches for a "
            "bus because an argument was missing"
        )
    limits = (
        _checked_gripper_limits(site.gripper_limits, where)
        if site.gripper_limits is not None
        else gripper_limits
    )
    rows = site.sim_home if site.sim_home is not None else sim_home
    home = tuple(float(v) for v in rows)
    if len(home) != JOINT_COUNT:
        raise ValueError(
            f"{where}: sim_home has {len(home)} values, a YAM part declares "
            f"{JOINT_COUNT} ({', '.join(JOINT_NAMES)})"
        )
    return ArmSite(
        channel=channel,
        base_frame=site.base_frame or base_frame,
        gripper_limits=limits,
        sim_home=home,
    )


def _build_arms(
    sites: Mapping[str, ArmSite],
    *,
    sim: bool,
    zero_gravity: bool,
    workspace,
    fk,
    step_caps: Sequence[float],
    joint_limits: Sequence[Sequence[float]],
    rate_hz: float,
    report: Callable[[str], None],
) -> Callable[[], dict[str, base.Arm]]:
    """How to open these arms. Called by `Rig.arms()`, never by the factory:
    the hardware opens there, and a failure to open it lands there.

    Arms open ONE AT A TIME, so this can fail with some of them already
    connected — a second arm that reports the wrong DOF, an argument mistake
    the first arm's `Arm` construction did not reach. The caller is then handed
    an exception rather than a rig, which on live hardware would leave every
    arm that did open energized, still being re-sent its last setpoint by the
    vendor's own server thread, and unreachable by the program that opened it.
    So a failure closes what it opened before it re-raises: half a rig is not a
    rig, and the exception is still the news."""

    def build() -> dict[str, base.Arm]:
        arms: dict[str, base.Arm] = {}
        opened: dict[str, base.Driver] = {}
        try:
            for part, site in sites.items():
                if sim:
                    driver: base.Driver = base.SimDriver(
                        site.sim_home,
                        lower=[lo for lo, _ in joint_limits],
                        upper=[hi for _, hi in joint_limits],
                        step_caps=step_caps,
                        rate_hz=rate_hz,
                    )
                else:
                    driver = LiveDriver(
                        site.channel,
                        gripper_limits=site.gripper_limits,
                        zero_gravity=zero_gravity,
                        report=report,
                    )
                opened[part] = driver
                arms[part] = base.Arm(
                    part=part,
                    driver=driver,
                    joint_names=JOINT_NAMES,
                    joint_limits=joint_limits,
                    step_caps=step_caps,
                    base_frame=site.base_frame,
                    workspace=workspace,
                    fk=fk,
                    arm_dof=ARM_JOINT_COUNT,
                    home_values=site.sim_home if sim else None,
                    rate_hz=rate_hz,
                    report=report,
                )
        except BaseException:
            if opened:
                report(
                    f"opening this rig failed with parts={','.join(opened)} already "
                    "open — closing them, since nothing is being handed a driver for "
                    "them"
                )
                base.close_all(opened, report=report)
            raise
        return arms

    return build


def bimanual(
    *,
    workspace: Sequence[Sequence[float]] | None,
    gripper_limits: Sequence[float],
    cross_arm: CrossArm | None = None,
    left: ArmSite | None = None,
    right: ArmSite | None = None,
    sim: bool = False,
    posture: str = "supervised",
    fk: Callable[[Sequence[float]], tuple[np.ndarray, np.ndarray]] | None = (
        forward_kinematics
    ),
    joint_limits: Sequence[Sequence[float]] | None = None,
    rate_hz: float = DEFAULT_RATE_HZ,
    max_joint_speed_rad_s: float = DEFAULT_MAX_JOINT_SPEED_RAD_S,
    max_gripper_speed_per_s: float = DEFAULT_MAX_GRIPPER_SPEED_PER_S,
    name: str = "yam-bimanual",
    robot_id: str = "",
    cell_id: str = "",
    cameras: Mapping[str, Camera] | None = None,
    estop_hardware: bool = False,
    report: Callable[[str], None] = base.status,
) -> base.Rig:
    """Two YAMs, declared as ONE robot with two named parts, so a teleoperator
    or a Waddle-hosted agent can address either arm by name.

    Declaration only: this opens no bus and starts no thread. ``rig.arms()``
    is where the hardware opens.

    The required arguments are the ones nothing can default: ``workspace`` (a
    box in each arm's own base frame — pass ``None`` explicitly to declare
    none) and ``gripper_limits`` (the bench-measured ``[closed, open]`` motor
    radians). ``cross_arm`` is optional and its absence is meaningful: with no
    declared edge, a pose expressed in the other arm's frame refuses loudly
    downstream rather than resolving through an identity nobody measured.

    ``sim`` is explicit and never inferred — nothing here try-imports the
    vendor package to decide what a program meant. ``posture`` picks which
    control verbs the session registers (see
    `waddle.robots.base.POSTURES`), and on live hardware ``"monitor"``
    additionally opens the arms compliant, so nothing can command them at
    either end.

    ``fk`` is the forward kinematics each part reports its TCP from, and it is
    OPT-IN: pass ``None`` (with ``workspace=None``) for a rig that reports
    joint positions only.

    ``joint_limits`` is the interval each row accepts, defaulting to the
    shipped model's ``JOINT_LIMITS``. It is the OWNER's envelope, so a rig
    whose machine differs from the model states its own — the usual reason
    being a motor zeroed a few milliradians off, which rests just outside a
    theoretical range and makes a hold of its own measured pose a command the
    envelope refuses forever. A row wider than the model's is reported, never
    silent. Whatever is declared here is both what the envelope enforces and
    what the declaration carries to the plane.
    """
    limits = _checked_gripper_limits(gripper_limits, "bimanual")
    joints = _checked_joint_limits(joint_limits, "bimanual", report=report)
    box = _checked_workspace(workspace, report=report)
    caps = _step_caps(rate_hz, max_joint_speed_rad_s, max_gripper_speed_per_s)
    sites = {
        LEFT_PART: _resolved_site(
            left,
            where=f"part={LEFT_PART}",
            sim=sim,
            base_frame=LEFT_BASE_FRAME,
            gripper_limits=limits,
            sim_home=DEFAULT_SIM_HOME[0],
        ),
        RIGHT_PART: _resolved_site(
            right,
            where=f"part={RIGHT_PART}",
            sim=sim,
            base_frame=RIGHT_BASE_FRAME,
            gripper_limits=limits,
            sim_home=DEFAULT_SIM_HOME[1],
        ),
    }
    frames = (
        (
            cross_arm.transform(
                sites[LEFT_PART].base_frame, sites[RIGHT_PART].base_frame
            ),
        )
        if cross_arm is not None
        else ()
    )
    return base.Rig(
        declaration=declaration(
            parts=tuple(sites),
            name=name,
            robot_id=robot_id,
            cell_id=cell_id,
            rate_hz=rate_hz,
            max_joint_speed_rad_s=max_joint_speed_rad_s,
            joint_limits=joints,
            frames=frames,
            cameras=cameras,
        ),
        build_arms=_build_arms(
            sites,
            sim=sim,
            zero_gravity=posture == "monitor",
            workspace=box,
            fk=fk,
            step_caps=caps,
            joint_limits=joints,
            rate_hz=rate_hz,
            report=report,
        ),
        rate_hz=rate_hz,
        posture=posture,
        estop_hardware=estop_hardware,
        report=report,
    )


def arm(
    *,
    workspace: Sequence[Sequence[float]] | None,
    gripper_limits: Sequence[float],
    channel: str | None = None,
    sim: bool = False,
    posture: str = "supervised",
    fk: Callable[[Sequence[float]], tuple[np.ndarray, np.ndarray]] | None = (
        forward_kinematics
    ),
    base_frame: str = BASE_FRAME,
    sim_home: Sequence[float] | None = None,
    joint_limits: Sequence[Sequence[float]] | None = None,
    rate_hz: float = DEFAULT_RATE_HZ,
    max_joint_speed_rad_s: float = DEFAULT_MAX_JOINT_SPEED_RAD_S,
    max_gripper_speed_per_s: float = DEFAULT_MAX_GRIPPER_SPEED_PER_S,
    name: str = "yam",
    robot_id: str = "",
    cell_id: str = "",
    cameras: Mapping[str, Camera] | None = None,
    declare_urdf: bool = True,
    estop_hardware: bool = False,
    report: Callable[[str], None] = base.status,
) -> base.Rig:
    """One YAM, declared as the whole robot — a bare joint space, no named
    parts, and (unlike a bimanual rig) the shipped model carried as
    ``kinematics_urdf``, since a single arm IS one chain.

    Every argument means what it means on :func:`bimanual`; the difference is
    that this rig has one arm, so its site facts are arguments rather than an
    :class:`ArmSite`.
    """
    limits = _checked_gripper_limits(gripper_limits, "arm")
    joints = _checked_joint_limits(joint_limits, "arm", report=report)
    box = _checked_workspace(workspace, report=report)
    caps = _step_caps(rate_hz, max_joint_speed_rad_s, max_gripper_speed_per_s)
    site = _resolved_site(
        ArmSite(channel=channel, base_frame=base_frame, sim_home=sim_home),
        where="arm",
        sim=sim,
        base_frame=base_frame,
        gripper_limits=limits,
        sim_home=DEFAULT_SIM_HOME[0],
    )
    return base.Rig(
        declaration=declaration(
            name=name,
            robot_id=robot_id,
            cell_id=cell_id,
            rate_hz=rate_hz,
            max_joint_speed_rad_s=max_joint_speed_rad_s,
            joint_limits=joints,
            base_frame=site.base_frame,
            declare_urdf=declare_urdf,
            cameras=cameras,
        ),
        build_arms=_build_arms(
            {"": site},
            sim=sim,
            zero_gravity=posture == "monitor",
            workspace=box,
            fk=fk,
            step_caps=caps,
            joint_limits=joints,
            rate_hz=rate_hz,
            report=report,
        ),
        rate_hz=rate_hz,
        posture=posture,
        estop_hardware=estop_hardware,
        report=report,
    )
