"""Native pile stability and initial robot clearance for every reference family."""

import numpy as np
import pytest
from waddle_sdk.simulators.scene import make_site


@pytest.mark.parametrize("robot", ["yam", "so101", "xarm7"])
def test_gravity_pile_is_stable_varied_and_inside_box(tmp_path, robot):
    pytest.importorskip("mujoco")
    from waddle_sdk.simulators.mujoco import Engine

    _, config = make_site(
        "shampoo-pile",
        backend="mujoco",
        robot=robot,
        environment="shampoo-packing",
        width=320,
        height=240,
    )
    engine = Engine(config, tmp_path)
    try:
        names = [f"shampoo_{i:02d}" for i in range(1, 21)]
        initial = np.array([engine.data.body(n).xpos.copy() for n in names])
        tilts = [np.arccos(abs(engine.data.body(n).xmat[8])) for n in names]
        assert sum(t > 1.3 for t in tilts) >= 4
        assert sum(0.3 < t < 1.3 for t in tilts) >= 4
        for _ in range(round(3 / engine.model.opt.timestep)):
            engine.step()
            for c in engine.data.contact:
                bodies = [
                    engine.model.body(int(engine.model.geom_bodyid[g])).name
                    for g in c.geom
                ]
                robot_contact = any(
                    n not in engine._prop_links and n not in ("table", "world")
                    for n in bodies
                )
                assert not (
                    robot_contact
                    and any(n in engine._prop_links or n == "table" for n in bodies)
                    and c.dist < -0.0001
                ), bodies
        positions = np.array([engine.data.body(n).xpos for n in names])
        assert np.max(np.linalg.norm(positions - initial, axis=1)) < 0.003
        assert np.all(np.abs(positions[:, :2] - [0.29, -0.20]) < [0.124, 0.114])
        assert np.all((positions[:, 2] > 0.02) & (positions[:, 2] < 0.15))
        for n in names:
            j = engine.model.joint(int(engine.model.body(n).jntadr[0]))
            v = int(j.dofadr[0])
            assert np.linalg.norm(engine.data.qvel[v : v + 3]) < 0.002
    finally:
        engine.close()
