"""Native retention through descent, grasp, lift, carry, and release.

Unlike an endpoint-only pinch, these probes expose contact loss while lifting a
small cylinder. Robot coordinates initialize an open approach; object state is
never written. All subsequent movement uses the reference position actuators.
"""

import numpy as np
import pytest
from waddle_sdk.simulators.scene import make_site

ABOVE = (
    -0.5133919951,
    1.9035043043,
    1.6255589442,
    -1.2928592084,
    -7.1335e-06,
    -0.5133770019,
)
TRANSFER = (
    0.0768580457,
    1.7291559463,
    1.4068279147,
    -1.2484765357,
    -7.5819e-06,
    0.076873151,
)
# Independently solved downward TCP poses over the canonical chocolate_13.
GRASPS = {
    0.018: (
        -0.5133911879,
        1.9638334077,
        1.3762743849,
        -0.983245543,
        -6.9837e-06,
        -0.5133762513,
    ),
    0.021: (
        -0.5133912186,
        1.9600190149,
        1.3834686449,
        -0.9942541977,
        -6.993e-06,
        -0.5133762782,
    ),
    0.024: (
        -0.5133912495,
        1.9563215283,
        1.390801007,
        -1.0052840469,
        -7.0021e-06,
        -0.5133763054,
    ),
    0.027: (
        -0.5133912802,
        1.9527415116,
        1.3982720697,
        -1.0163351266,
        -7.0109e-06,
        -0.5133763326,
    ),
}


@pytest.mark.parametrize(
    "height, opening, retained",
    [
        (height, opening, True)
        for height in (0.018, 0.021, 0.024)
        for opening in (0.0, 0.018)
    ]
    + [(0.027, 0.0, False)],
)
def test_cylinder_retains_bilateral_grasp_through_transport(
    tmp_path, height, opening, retained
):
    mujoco = pytest.importorskip("mujoco")
    from waddle_sdk.simulators.mujoco import Engine

    _, config = make_site(
        "chocolate-retention",
        backend="mujoco",
        robot="yam",
        environment="chocolate-packing",
        width=320,
        height=240,
    )
    engine = Engine(config, tmp_path)
    target = "chocolate_13"
    finger_names = {"tip_left", "tip_right"}
    force = np.zeros(6)

    def move(start, end, seconds, check=False):
        for step in range(round(seconds / engine.model.opt.timestep)):
            fraction = (step + 1) / round(seconds / engine.model.opt.timestep)
            smooth = fraction**3 * (10 - 15 * fraction + 6 * fraction**2)
            engine.write(start + smooth * (end - start))
            engine.step()
            if check and step % 10 == 0:
                assert engine.data.body(target).xpos[2] > 0.065, (
                    "Chocolate fell during transport"
                )
                touched = set()
                for index in range(engine.data.ncon):
                    contact = engine.data.contact[index]
                    bodies = {
                        engine.model.body(int(engine.model.geom_bodyid[g])).name
                        for g in (contact.geom1, contact.geom2)
                    }
                    if target in bodies:
                        mujoco.mj_contactForce(engine.model, engine.data, index, force)
                        if force[0] > 0.01:
                            touched.update(bodies & finger_names)
                assert touched == finger_names, "Lost bilateral contact while carrying"

    try:
        above = np.r_[ABOVE, 0.032 / engine.profile.opening]
        down = np.r_[GRASPS[height], above[-1]]
        closed = down.copy()
        closed[-1] = opening / engine.profile.opening
        lifted = np.r_[ABOVE, closed[-1]]
        transfer = np.r_[TRANSFER, closed[-1]]
        engine.home(above)
        move(above, above, 0.5)
        move(above, down, 2.0)
        move(down, down, 0.5)
        move(down, closed, 0.8)
        move(closed, closed, 0.5)
        move(closed, lifted, 1.5)
        move(lifted, lifted, 2.0, check=retained)
        move(lifted, transfer, 3.0, check=retained)
        move(transfer, transfer, 2.0, check=retained)
        if not retained:
            assert engine.data.body(target).xpos[2] < 0.02, (
                "An above-object close must not attach it"
            )
            return
        np.testing.assert_allclose(
            engine.data.body(target).xpos[:2], (0.312, 0.024), atol=0.003
        )
        opened = transfer.copy()
        opened[-1] = above[-1]
        move(transfer, opened, 0.8)
        move(opened, opened, 1.0)
        assert 0.005 < engine.data.body(target).xpos[2] < 0.015, (
            "Released chocolate must fall to the table"
        )
    finally:
        engine.close()
