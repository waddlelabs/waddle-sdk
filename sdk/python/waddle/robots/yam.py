"""The I2RT YAM: what one is, in numbers, and what you can build out of one.

A robot module is facts plus a driver plus a factory, on top of
:mod:`waddle.robots.base`. This file carries the first two for the YAM:

* the FACTS — the six arm joints, their limits, the kinematic chain, the tool
  frame, the hand's stroke — and the :func:`forward_kinematics` they describe;
* :class:`LiveDriver`, the thin honest layer over the vendor's own calls, with
  the e-stop latch the vendor's zero-torque mode makes necessary.

Both stand alone: take the facts and declare the robot yourself, or take the
driver and put your own envelope in front of it. Nothing here holds a lease
or decides who may command anything — that is waddle-core's either way.

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
  gripper motor limits and the CAN interface belong to YOUR rig, are
  arguments, and have no defaults to inherit by accident.

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

import threading
from collections.abc import Callable, Sequence
from importlib.resources import files

import numpy as np

from . import base

__all__ = [
    "ARM_JOINT_COUNT",
    "ARM_JOINT_LIMITS_RAD",
    "ARM_JOINT_NAMES",
    "CHAIN_AXIS",
    "CHAIN_ORIGIN_RPY_RAD",
    "CHAIN_ORIGIN_XYZ_M",
    "GRIPPER_JOINT_LIMITS",
    "GRIPPER_JOINT_NAME",
    "GRIPPER_MAX_OPENING_M",
    "I2RT_INSTALL",
    "I2RT_PIN",
    "I2RT_REPO",
    "JOINT_COUNT",
    "JOINT_LIMITS",
    "JOINT_NAMES",
    "MAX_JOINT_EFFORT_NM",
    "TOOL_ORIGIN_RPY_RAD",
    "TOOL_ORIGIN_XYZ_M",
    "URDF_BASE_LINK",
    "URDF_TCP_FRAME",
    "LiveDriver",
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
      and an absent position is a fault, not a guess.
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
    It is what a monitor posture builds.

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
        dofs = int(self._robot.num_dofs())
        if dofs != JOINT_COUNT:
            raise RuntimeError(
                f"{channel}: the arm reports {dofs} DOF, this module declares "
                f"{JOINT_COUNT} ({', '.join(JOINT_NAMES)}) — reject, never adapt "
                "silently"
            )
        self._default_kp, self._default_kd = self._snapshot_gains()

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
        position = np.zeros(JOINT_COUNT)
        position[:ARM_JOINT_COUNT] = np.asarray(joint_pos, dtype=float)[
            :ARM_JOINT_COUNT
        ]
        if gripper is not None and len(gripper) > 0:
            position[ARM_JOINT_COUNT] = float(gripper[0])
        velocity = np.zeros(JOINT_COUNT)
        raw_vel = obs.get("joint_vel")
        if raw_vel is not None and len(raw_vel) > 0:
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
