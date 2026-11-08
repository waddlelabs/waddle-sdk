"""Workstations retain native mechanics, usable tool contact and exact reset."""

import numpy as np
import pytest
from waddle_sdk.simulators.playground import compile_scene
from waddle_sdk.simulators.scene import load_scene, make_site


def workstation(tmp_path, fixture, *, arms=1, seed=0):
    pytest.importorskip("mujoco")
    from waddle_sdk.simulators.mujoco import Engine

    _, config = make_site(
        "workstation",
        robot="yam",
        arms=arms,
        backend="mujoco",
        environment="two_cubes",
        width=96,
        height=72,
    )
    output = tmp_path / "scene"
    compile_scene(
        config,
        {
            "id": fixture,
            "name": fixture,
            "fixture": fixture,
            "seed": seed,
            "objects": [],
        },
        output,
    )
    _, compiled = load_scene(output, "simulation.json")
    return Engine(compiled, output)


def advance(engine, seconds):
    for _ in range(round(seconds / engine.model.opt.timestep)):
        engine.step()


@pytest.mark.parametrize("fixture", ["drawer_cabinet", "cleanup_station"])
@pytest.mark.parametrize("arms", [1, 2])
@pytest.mark.parametrize("seed", [0, 9])
def test_workstation_is_stable_visible_and_resets_exactly(
    tmp_path, fixture, arms, seed
):
    engine = workstation(tmp_path, fixture, arms=arms, seed=seed)
    try:
        initial = engine.data.qpos.copy()
        advance(engine, 1)
        assert np.isfinite(engine.data.qpos).all()
        for contact in engine.data.contact:
            names = [
                engine.model.body(int(engine.model.geom_bodyid[g])).name
                for g in contact.geom
            ]
            robot = [
                name
                for name in names
                if name.startswith(("arm__", "left__", "right__"))
            ]
            same_arm = any(
                all(name.startswith(part + "__") for name in names)
                for part in engine.parts
            )
            assert not (robot and not same_arm and contact.dist < -0.002), names
        names = (
            ["target_object"]
            if fixture == "drawer_cabinet"
            else ["brush", *[f"debris_{i}" for i in range(6)]]
        )
        state = engine.evaluation_snapshot()
        for name in names:
            assert state["bodies"][name]["position_m"][2] > 0
            assert np.linalg.norm(state["bodies"][name]["linear_velocity_m_s"]) < 0.02
        for camera in engine.config["cameras"]:
            rgb, depth = engine.capture(camera)
            assert rgb.shape == (72, 96, 3) and np.ptp(rgb) > 20
            assert depth.shape == (72, 96) and (depth > 0).any()
        engine.reset()
        np.testing.assert_array_equal(engine.data.qpos, initial)
    finally:
        engine.close()


def test_drawer_load_opens_and_closes_with_object_supported(tmp_path):
    engine = workstation(tmp_path, "drawer_cabinet")
    try:
        # Isolate the passive drawer from the parked arm, as the reference
        # drawer test does. A manipulating hand would follow the handle.
        clear = engine.read()[0].copy()
        clear[0] = np.pi / 2
        engine.home(clear)
        advance(engine, 0.5)
        slide = engine.model.joint("drawer_slide")
        qpos, dof = int(slide.qposadr[0]), int(slide.dofadr[0])
        start = engine.data.body("target_object").xpos.copy()
        engine.data.qfrc_applied[dof] = 0.8
        advance(engine, 2)
        assert engine.data.qpos[qpos] > 0.15
        opened = engine.data.body("target_object").xpos.copy()
        assert opened[0] < start[0] - 0.10
        assert opened[2] == pytest.approx(start[2], abs=0.004)
        engine.data.qfrc_applied[dof] = -0.8
        advance(engine, 2)
        assert engine.data.qpos[qpos] < 0.01
        assert engine.data.body("target_object").xpos[2] == pytest.approx(
            start[2], abs=0.004
        )
    finally:
        engine.close()


@pytest.mark.parametrize("lane,enters", [(0.10, True), (-0.21, False)])
def test_brush_contact_moves_pieces_up_the_pan_lip_and_a_miss_does_not(
    tmp_path, lane, enters
):
    engine = workstation(tmp_path, "cleanup_station")
    try:
        clear = engine.read()[0].copy()
        clear[0] = np.pi / 2
        engine.home(clear)
        # Initialize an independent tool-contact witness. During the sweep only
        # external tool loads are applied; no piece pose or joint is rewritten.
        joint = engine.model.joint("brush_free")
        address = int(joint.qposadr[0])
        engine.data.qpos[address : address + 3] = [0.19, lane, 0.0125]
        engine.mj.mj_forward(engine.model, engine.data)
        advance(engine, 0.25)
        brush = engine.data.body("brush")
        target = engine.data.body("debris_4")
        dof = int(joint.dofadr[0])
        start = target.xpos.copy()
        for step in range(round(3 / engine.model.opt.timestep)):
            elapsed = step * engine.model.opt.timestep
            desired = [0.19 + min(0.08 * elapsed, 0.23), lane, 0.0125]
            velocity = engine.data.qvel[dof : dof + 6]
            # A finite held-tool spring resists riding over the pieces while
            # permitting the pad to follow the pan's 3 mm entrance ramp.
            engine.data.xfrc_applied[brush.id, :3] = (
                np.array([150, 150, 1500]) * (np.asarray(desired) - brush.xpos)
                - np.array([3, 3, 12]) * velocity[:3]
            )
            engine.data.xfrc_applied[brush.id, 3:] = (
                -2 * engine.data.qpos[address + 4 : address + 7] - 0.005 * velocity[3:]
            )
            engine.step()
        assert engine.data.warning[engine.mj.mjtWarning.mjWARN_BADQACC].number == 0
        if enters:
            assert 0.38 < target.xpos[0] < 0.52
            assert abs(target.xpos[1] - 0.08) < 0.08
            assert 0.009 < target.xpos[2] < 0.018
        else:
            np.testing.assert_allclose(target.xpos, start, atol=0.001)
    finally:
        engine.close()


@pytest.mark.parametrize("fixture", ["drawer_cabinet", "cleanup_station"])
@pytest.mark.parametrize("robot", ["so101", "xarm7"])
def test_workstation_refuses_unqualified_embodiments_before_writing(
    tmp_path, fixture, robot
):
    _, config = make_site(
        "unsupported", robot=robot, backend="mujoco", environment="two_cubes"
    )
    output = tmp_path / "unsupported"
    with pytest.raises(ValueError, match="requires one or two YAMs"):
        compile_scene(
            config,
            {"id": "task", "name": "Task", "objects": [], "fixture": fixture},
            output,
        )
    assert not output.exists()
