"""Native pickup of the original flat hook, held transport, and gravity release.

Only the robot's open approach is initialized. Hook motion comes exclusively
from gravity and actuator-driven contact, without attachments or object resets.
These fixed joint paths test contact mechanics, not motion planning.
"""

import json
from pathlib import Path

import numpy as np
import pytest
from waddle_sdk.simulators.scene import make_site

FIXTURES = json.loads(
    (Path(__file__).parent / "fixtures" / "hook_transport.json").read_text()
)


@pytest.mark.parametrize("fixture", FIXTURES, ids=lambda f: f["robot"])
@pytest.mark.parametrize("miss", (False, True), ids=("pickup", "miss"))
def test_hook_retains_grasp_until_opened(tmp_path, fixture, miss):
    mj = pytest.importorskip("mujoco")
    from waddle_sdk.simulators.mujoco import Engine

    robot = fixture["robot"]
    _, config = make_site(
        "hook-retention", backend="mujoco", robot=robot, environment="use-hook"
    )
    engine = Engine(config, tmp_path)
    fingers = set(
        {
            "so101": ("gripper_link", "moving_jaw_so101_v1_link"),
            "yam": ("tip_left", "tip_right"),
            "xarm7": ("left_finger", "right_finger"),
        }[robot]
    )
    force = np.zeros(6)
    reference = None
    try:
        above, down, lifted, transfer = [
            np.array(fixture[key]) for key in ("approach", "grasp", "lift", "transfer")
        ]
        if miss:
            down = above.copy()
        closed = down.copy()
        closed[-1] = fixture["closed_opening"]
        engine.home(above)

        def move(start, end, seconds, *, retain=False, airborne=False):
            nonlocal reference
            steps = round(seconds / engine.model.opt.timestep)
            for step in range(steps):
                t = (step + 1) / steps
                smooth = t**3 * (10 - 15 * t + 6 * t * t)
                engine.write(start + smooth * (end - start))
                engine.step()
                if step % 10:
                    continue
                touched = set()
                for i, contact in enumerate(engine.data.contact):
                    names = {
                        engine.model.body(int(engine.model.geom_bodyid[g])).name
                        for g in contact.geom
                    }
                    if "table" in names and names & fingers:
                        assert contact.dist > -0.0005, (names, contact.dist)
                    if "hook" in names:
                        mj.mj_contactForce(engine.model, engine.data, i, force)
                        if force[0] > 0.01:
                            touched.update(names & fingers)
                if not retain:
                    continue
                assert touched == fingers, (robot, "lost bilateral contact", touched)
                body = engine.data.body("hook")
                tcp = engine.data.site("tcp_site")
                frame = tcp.xmat.reshape(3, 3).T
                position = frame @ (body.xpos - tcp.xpos)
                rotation = frame @ body.xmat.reshape(3, 3)
                if reference is None:
                    reference = (position.copy(), rotation.copy())
                drift = np.linalg.norm(position - reference[0])
                angle = np.arccos(
                    np.clip((np.trace(rotation @ reference[1].T) - 1) / 2, -1, 1)
                )
                assert drift < 0.006, (robot, "grasp drift", drift)
                assert angle < 0.15, (robot, "grasp rotation", angle)
                if airborne:
                    assert body.xpos[2] > initial_z + 0.025, (robot, "dropped hook")

        move(above, above, 0.5)
        initial_z = float(engine.data.body("hook").xpos[2])
        move(above, down, 2.0)
        move(down, down, 0.5)
        move(down, closed, 1.0)
        move(closed, closed, 1.0)
        move(closed, lifted, 2.0, retain=not miss)
        move(lifted, lifted, 20.0, retain=not miss, airborne=True)
        move(lifted, transfer, 3.0, retain=not miss, airborne=True)
        move(transfer, transfer, 20.0, retain=not miss, airborne=True)
        if miss:
            assert abs(engine.data.body("hook").xpos[2] - initial_z) < 0.001
            return
        opened = transfer.copy()
        opened[-1] = above[-1]
        move(transfer, opened, 1.0)
        move(opened, opened, 2.0)
        assert abs(engine.data.body("hook").xpos[2] - initial_z) < 0.001, (
            "Released hook must fall back to the table"
        )
    finally:
        engine.close()
