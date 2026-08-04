"""The Python surface for part-addressed control (flag `waddle.v0.parts`).

The core declares that flag at Register for EVERY Composite declaration, and
the default `waddle-sdk` wheel is built with `grpc` — so a bimanual customer
on this distribution advertises it, and a plane that accepts it may send an
`Action` addressing one arm. Every payload path that carries such an action
into Python therefore has to be able to say which part it addressed;
otherwise one arm's 7 rows arrive indistinguishable from a 14-row whole-robot
command, which is exactly the confusion the flag exists to prevent.

MERGE GATE for the `waddle.v0.parts` work: the gate path is covered
(`GateInfo.part`, pinned below). The `send` path is NOT — `Chunk.steps` still
hands the customer's `send` verb a bare `(values, gripper, offset_ns)` tuple
per step with no part, and the bypass pump (the only path an agent-invited
episode ever takes) dispatches through it. Do not ship a release that
declares `waddle.v0.parts` until the dict-by-part `Chunk.steps` surface
lands; the flag would promise a behavior this distribution cannot express.
"""

from __future__ import annotations

import waddle
from waddle import _native


def test_gate_info_names_the_part_an_action_addressed():
    """`GateInfo.part` exists and is `None` for a whole-robot action.

    Pinned on the class rather than only on a live part-scoped
    intervention, because there is no Python hook yet that can drive one:
    this is the surface the flag's declaration depends on, and it must not
    quietly disappear.
    """
    assert hasattr(_native.core.GateInfo, "part")

    robot = waddle.Robot(
        name="pytest-parts-bot",
        robot_id="py-parts-01",
        cell_id="cell-py-parts",
        action_space=waddle.JointSpace(joints=["j0", "j1", "j2"], rate_hz=50),
    )
    control = waddle.Control(
        send=lambda chunk: None, hold=lambda: None, resume=lambda: None
    )
    waddle.init("py-parts", robot, control, _testing=True)
    try:
        with waddle.rollout(task="parts surface") as ep:
            ep.gate([0.0, 0.0, 0.0])
            # Passthrough is a whole-robot decision like any other: no part.
            assert ep.last_gate.part is None
            ep.terminate("success")
    finally:
        waddle.shutdown()
