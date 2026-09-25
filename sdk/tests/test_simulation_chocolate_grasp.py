"""Native retention through descent, grasp, lift, carry, and release.

Unlike an endpoint-only pinch, these probes expose contact loss while lifting a
small cylinder. Robot coordinates initialize an open approach; object state is
never written. All subsequent movement uses the reference position actuators.
"""

import numpy as np
import pytest
from waddle_sdk.simulators.scene import make_site

ABOVE = (
    -0.5085160982,
    1.8996384844,
    1.6203781003,
    -1.2915441853,
    -0.000007143,
    -0.5085011016,
)
TRANSFER = (
    0.0832253161,
    1.7297615136,
    1.4075339546,
    -1.2485770079,
    -0.00000758,
    0.0832404214,
)
# Independently solved downward TCP poses over the canonical chocolate_13.
GRASPS = {
    0.018: (
        -0.508515288,
        1.9603478283,
        1.3713086883,
        -0.9817654249,
        -0.0000069922,
        -0.508500348,
    ),
    0.021: (
        -0.5085153191,
        1.9565185419,
        1.3785012983,
        -0.9927873242,
        -0.0000070016,
        -0.5085003752,
    ),
    0.024: (
        -0.50851535,
        1.9528063309,
        1.3858316805,
        -1.0038299174,
        -0.0000070108,
        -0.5085004025,
    ),
    0.027: (
        -0.508515381,
        1.9492117615,
        1.3933004514,
        -1.0148932577,
        -0.0000070196,
        -0.5085004298,
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
            engine.data.body(target).xpos[:2], (0.312, 0.026), atol=0.003
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
