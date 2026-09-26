"""Native tube pickup, continuous retention and release across reference hands.

Only the open robot approach is initialized for the rack-loading scene; its
four lying tubes remain untouched. The capped-tube fixture additionally places
its tube on the table once to represent a dropped, uncapped tube. Thereafter
all object motion comes from gravity, contact and the normal position actuators.
These are mechanics witnesses, not planner or task-completion acceptance.
"""

import json
from pathlib import Path

import numpy as np
import pytest
from waddle_sdk.simulators.scene import make_site

FIXTURES = json.loads(
    (Path(__file__).parent / "fixtures" / "tube_transport.json").read_text()
)
REPRESENTATIVES = [
    row
    for row in FIXTURES
    if row["target"]
    == ("clear_test_tube_3" if row["robot"] == "so101" else "clear_test_tube_2")
]


def exercise_tube(tmp_path, fixture, *, dropped=False, miss=False):
    mj = pytest.importorskip("mujoco")
    from waddle_sdk.simulators.mujoco import Engine

    robot = fixture["robot"]
    _, config = make_site(
        "tube-retention",
        backend="mujoco",
        robot=robot,
        environment="uncap-return-test-tube" if dropped else "load-clear-test-tubes",
        arms=2 if dropped else 1,
        width=320,
        height=240,
    )
    engine = Engine(config, tmp_path)
    part = "right" if dropped else "arm"
    prefix = "right__" if dropped else ""
    target = "target_test_tube" if dropped else fixture["target"]
    fingers = {
        prefix + name
        for name in {
            "so101": ("gripper_link", "moving_jaw_so101_v1_link"),
            "yam": ("tip_left", "tip_right"),
            "xarm7": ("left_finger", "right_finger"),
        }[robot]
    }
    force = np.zeros(6)
    reference = None
    try:
        if dropped:
            # Test setup only, before stepping; the cap stays at the rack.
            base = np.asarray(config["parts"][part]["xyz"])
            xy = (0.24, 0.05) if robot == "so101" else (0.30, -0.07)
            joint = engine.model.joint(int(engine.model.body(target).jntadr[0]))
            address = int(joint.qposadr[0])
            engine.data.qpos[address : address + 3] = base + (*xy, 0.015)
            engine.data.qpos[address + 3 : address + 7] = [2**-0.5, 0, 2**-0.5, 0]
        above, down, lifted, transfer = [
            np.array(fixture[key]) for key in ("approach", "grasp", "lift", "transfer")
        ]
        if miss:
            down = above.copy()
        closed = down.copy()
        closed[-1] = fixture["closed_opening"]
        engine.home(part, above)

        def move(start, end, seconds, *, retain=False, airborne=False):
            nonlocal reference
            steps = round(seconds / engine.model.opt.timestep)
            for step in range(steps):
                t = (step + 1) / steps
                smooth = t**3 * (10 - 15 * t + 6 * t * t)
                engine.write(part, start + smooth * (end - start))
                engine.step()
                if step % 10:
                    continue
                touched = set()
                for i, contact in enumerate(engine.data.contact):
                    names = {
                        engine.model.body(int(engine.model.geom_bodyid[g])).name
                        for g in contact.geom
                    }
                    # A grasp cannot be qualified by pushing the hand through the table.
                    if "table" in names and names & fingers:
                        assert contact.dist > -0.0005, (names, contact.dist)
                    if target in names:
                        mj.mj_contactForce(engine.model, engine.data, i, force)
                        if force[0] > 0.01:
                            touched.update(names & fingers)
                if not retain:
                    continue
                assert touched == fingers, (
                    robot,
                    target,
                    "lost bilateral contact",
                    touched,
                )
                body = engine.data.body(target)
                tcp = engine.data.site(prefix + "tcp_site")
                relative = tcp.xmat.reshape(3, 3).T @ (body.xpos - tcp.xpos)
                if reference is None:
                    reference = relative.copy()
                assert np.linalg.norm(relative - reference) < 0.006, (
                    robot,
                    target,
                    "grasp drift",
                    relative - reference,
                )
                if airborne:
                    assert body.xpos[2] > initial_z + 0.025, (
                        robot,
                        target,
                        "dropped tube",
                        body.xpos,
                    )

        move(above, above, 0.5)
        # Measure lift from the gravity-settled tube, not its authored drop height.
        initial_z = float(engine.data.body(target).xpos[2])
        move(above, down, 2.0)
        move(down, down, 0.5)
        move(down, closed, 1.0)
        move(closed, closed, 1.0)
        move(closed, lifted, 2.0, retain=not miss)
        move(lifted, lifted, 8.0, retain=not miss, airborne=True)
        move(lifted, transfer, 3.0, retain=not miss, airborne=True)
        move(transfer, transfer, 8.0, retain=not miss, airborne=True)
        if miss:
            assert engine.data.body(target).xpos[2] < 0.02
            return
        opened = transfer.copy()
        opened[-1] = above[-1]
        move(transfer, opened, 1.0)
        move(opened, opened, 2.0)
        assert 0.004 < engine.data.body(target).xpos[2] < 0.025, (
            "Released tube must fall back to the table"
        )
    finally:
        engine.close()


@pytest.mark.parametrize(
    "fixture", FIXTURES, ids=lambda f: f"{f['robot']}-{f['target']}"
)
def test_lying_tubes_retain_grasp_through_lift_carry_and_release(tmp_path, fixture):
    exercise_tube(tmp_path, fixture)


@pytest.mark.parametrize("fixture", REPRESENTATIVES, ids=lambda f: f["robot"])
def test_dropped_uncapping_tube_can_be_picked_up(tmp_path, fixture):
    exercise_tube(tmp_path, fixture, dropped=True)


@pytest.mark.parametrize("fixture", REPRESENTATIVES, ids=lambda f: f["robot"])
def test_closing_above_a_tube_does_not_attach_it(tmp_path, fixture):
    exercise_tube(tmp_path, fixture, miss=True)
