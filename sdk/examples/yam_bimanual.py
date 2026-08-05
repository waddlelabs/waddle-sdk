#!/usr/bin/env python3
"""Two I2RT YAM arms, supervised, in five lines of Waddle.

This is the whole program for a rig `waddle.robots.yam` already knows: build
the rig out of YOUR site's numbers, open a session over it, ask Waddle to
drive an episode. The five lines at the bottom are the entire Waddle-facing
surface — everything above them is either a number measured at a bench or an
environment variable so this file can be run from a harness.

Run it against a supervision plane::

    WADDLE_YAM_TRANSPORT=http://<plane-host>:<port> \\
    WADDLE_YAM_TOKEN=<the plane's credential for this session> \\
      uv run python examples/yam_bimanual.py

A plane is what THIS program is for: its five lines end in `waddle.agent()`,
so with no ``WADDLE_YAM_TRANSPORT`` it says what it needs and exits(2) before
anything opens — no session, no twin stepping, no episode. (A rig needs no
plane in general: `rig.session(...)` with no transport is a local recorder a
program drives from its own loop, which is what ``examples/toy_robot.py``
runs offline. That is a different program, not a mode of this one.)

While a session is open, every episode lands in ``WADDLE_YAM_RECORDING_DIR``
— ``recordings/`` here, created if it is not there yet — as one sidecar and
one MCAP apiece.

`sim=True` is the default here and is EXPLICIT, never inferred: no code path
try-imports the vendor package to decide what you meant. Set
``WADDLE_YAM_SIM=0`` and give each arm its CAN interface
(``WADDLE_YAM_CAN_LEFT`` / ``_RIGHT``) to drive metal — and read
`waddle.robots.yam`'s own docstring first, because that needs the vendor
package this SDK deliberately does not depend on.

**The numbers below are the reference rig's, and they are not yours.** The
workspace box, the bench-measured gripper motor limits and the cross-arm
mounting are SITE facts: measure them at your own bench and edit them here.
That is why the factory has no defaults for them — a default would be a
measurement nobody took, and the arm executes it faithfully. What a YAM *is*
(joint limits, the chain, the tool frame, the hand's stroke) is a model fact
and ships in `waddle.robots.yam`, gated against the vendor's own model.
"""

from __future__ import annotations

import os
import sys

import waddle
from waddle.robots import yam

# --------------------------------------------------------------------------
# SITE FACTS — the reference rig's. Re-measure every one of them for yours.
# --------------------------------------------------------------------------

#: ((min_x, min_y, min_z), (max_x, max_y, max_z)) metres, in EACH arm's own
#: base frame, applied to the FK'd TCP of every command before it is accepted.
#: Tightening this is always safe; widening it is a decision about what may
#: happen in the room.
WORKSPACE_M = ((0.05, -0.45, 0.05), (0.60, 0.45, 0.70))

#: [closed, open] in MOTOR RADIANS, measured at the bench for THESE hands.
#: Required even in sim, so the program text does not change across the flip —
#: and required at all because building a live arm without it runs the
#: vendor's connect-time auto-calibration, which physically drives the jaws.
GRIPPER_LIMITS_MOTOR_RAD = (0.1, 1.7)

#: Where the RIGHT arm's base stands in the LEFT arm's base frame. NOMINAL on
#: the reference rig (two YAMs 600 mm apart, the right one toed slightly in) —
#: measure yours before any prompt that expresses one arm's target in the
#: other arm's frame. Pass `cross_arm=None` until you have: with no declared
#: edge such a pose refuses loudly instead of composing through an identity
#: nobody measured.
CROSS_ARM = yam.CrossArm(xyz=(0.0, -0.60, 0.0), rpy=(0.0, 0.0, -0.15))


def env(name: str, default: str | None = None) -> str | None:
    """An EMPTY value counts as unset, so a harness can pass
    ``VAR=${MAYBE_UNSET}`` straight through."""
    return os.environ.get(name) or default


SIM = env("WADDLE_YAM_SIM", "1") != "0"
TRANSPORT = env("WADDLE_YAM_TRANSPORT")
PROMPT = env("WADDLE_YAM_PROMPT", "move each arm to its taught home, one at a time")

# --------------------------------------------------------------------------
# The program.
# --------------------------------------------------------------------------

rig = yam.bimanual(
    workspace=WORKSPACE_M,
    gripper_limits=GRIPPER_LIMITS_MOTOR_RAD,
    cross_arm=CROSS_ARM,
    # A CAN interface is a site fact too, so it has no default here either:
    # in sim it is ignored, and a live rig without one refuses by name.
    left=yam.ArmSite(channel=env("WADDLE_YAM_CAN_LEFT")),
    right=yam.ArmSite(channel=env("WADDLE_YAM_CAN_RIGHT")),
    sim=SIM,
)

if not TRANSPORT:
    print(
        "no supervision plane configured: set WADDLE_YAM_TRANSPORT=<grpc url> "
        "(and WADDLE_YAM_TOKEN if your plane asks for one) — there is nobody "
        "to invite without one",
        flush=True,
    )
    sys.exit(2)

with rig.session(
    "waddle-yam-bimanual",
    transport=waddle.Grpc(TRANSPORT, env("WADDLE_YAM_TOKEN")),
    recording_dir=env("WADDLE_YAM_RECORDING_DIR", "recordings"),
) as session:
    result = waddle.agent(PROMPT, timeout_s=float(env("WADDLE_YAM_AGENT_TIMEOUT", "900")))

print(f"agent result {result.outcome} episode={result.episode_id} detail={result.detail!r}")
print(f"envelope accepted={session.accepted} rejected={session.rejected}")
sys.exit(0 if result.outcome == "success" else 1)
