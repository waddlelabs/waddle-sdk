"""The I2RT YAM: what one is, in numbers, and the chain that produces a TCP.

A robot module is facts plus a driver plus a factory, on top of
:mod:`waddle.robots.base`. This file is the FACTS half for the YAM — the six
arm joints, their limits, the kinematic chain, the tool frame, the hand's
stroke — and the forward kinematics those facts describe. Nothing here opens
a socket, holds a lease or decides who may command anything.

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
* Nothing here is a site fact. The workspace box, the bench-measured gripper
  motor limits, the CAN interface and where a second arm stands relative to
  the first belong to YOUR rig, are arguments to the factories, and have no
  defaults to inherit by accident.

This module is public on purpose. What a customer needs in order to drive
their own arm through their own envelope belongs in the customer's own hands,
so it ships in the open under this repo's licence; the supervision side's
in-cell material for keeping a fleet of these alive is a different artifact
and is not here.
"""

from __future__ import annotations

from importlib.resources import files
from typing import Sequence

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
    "I2RT_PIN",
    "JOINT_COUNT",
    "JOINT_LIMITS",
    "JOINT_NAMES",
    "MAX_JOINT_EFFORT_NM",
    "TOOL_ORIGIN_RPY_RAD",
    "TOOL_ORIGIN_XYZ_M",
    "URDF_BASE_LINK",
    "URDF_TCP_FRAME",
    "forward_kinematics",
    "urdf_text",
]

#: The upstream commit every fact in this file is pinned to, and the commit
#: the shipped ``yam_data/yam.urdf`` was vendored from. The vendor's Python
#: package is installed by the same pin (it is not on PyPI), so a module that
#: drives an arm and a model that describes one cannot drift apart silently.
I2RT_PIN = "570ef66681ff12bd8298aba34084307cfecc9f05"

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
