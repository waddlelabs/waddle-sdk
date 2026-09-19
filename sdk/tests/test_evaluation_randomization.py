"""Native acceptance for private seeded task-scene reset."""

from __future__ import annotations

import math

import numpy as np
import pytest
from waddle_sdk.simulation import SimulationVariation
from waddle_sdk.simulators.description import description
from waddle_sdk.simulators.scene import (
    DUAL_ARM_TASK_ENVIRONMENTS,
    ROBOTS,
    TASK_ENVIRONMENTS,
    make_site,
)

EVALUATION_ENVIRONMENTS = ("drawer", *TASK_ENVIRONMENTS)


def _body_poses(snapshot, names):
    return {
        name: {
            key: snapshot["bodies"][name][key]
            for key in ("position_m", "orientation_wxyz")
        }
        for name in names
    }


def _multiply_quaternions(first, second):
    aw, ax, ay, az = first
    bw, bx, by, bz = second
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


@pytest.mark.parametrize("robot", ROBOTS)
@pytest.mark.parametrize("environment", EVALUATION_ENVIRONMENTS)
def test_seeded_evaluation_reset_covers_complete_task_matrix(
    tmp_path, monkeypatch, robot, environment
):
    pytest.importorskip("mujoco")
    monkeypatch.setenv("MUJOCO_GL", "egl")
    from waddle_sdk.simulators.mujoco import Engine

    arms = 2 if environment in DUAL_ARM_TASK_ENVIRONMENTS else 1
    _, config = make_site(
        "randomization-test",
        backend="mujoco",
        robot=robot,
        environment=environment,
        width=192,
        height=144,
        arms=arms,
    )
    engine = Engine(config, tmp_path)
    try:
        names = tuple(name for group in engine._pose_groups for name in group.bodies)
        assert names
        canonical_snapshot = engine.evaluation_snapshot()
        canonical = _body_poses(canonical_snapshot, names)

        assert engine.evaluation_reset(seed=712)
        first_snapshot = engine.evaluation_snapshot()
        first = _body_poses(first_snapshot, names)
        first_variation_digest = first_snapshot["variation"]["resolved_digest"]
        assert engine.evaluation_reset(seed=712)
        assert _body_poses(engine.evaluation_snapshot(), names) == first
        assert (
            engine.evaluation_snapshot()["variation"]["resolved_digest"]
            == first_variation_digest
        )

        assert engine.evaluation_reset(seed=713)
        second_snapshot = engine.evaluation_snapshot()
        second = _body_poses(second_snapshot, names)
        assert second != first
        assert second_snapshot["variation"]["resolved_digest"] != first_variation_digest

        engine.data.qpos[0] += 0.001
        engine.mj.mj_forward(engine.model, engine.data)
        assert (
            engine.evaluation_snapshot()["variation"]["resolved_digest"]
            == second_snapshot["variation"]["resolved_digest"]
        )

        assert engine.evaluation_reset(seed=0)
        zero_snapshot = engine.evaluation_snapshot()
        assert _body_poses(zero_snapshot, names) == canonical
        robot_names = {link.name for link in description(robot).links}
        if arms == 2:
            robot_names = {
                f"{part}__{name}" for part in engine.parts for name in robot_names
            }
        for task_snapshot in (first_snapshot, second_snapshot, zero_snapshot):
            for contact in task_snapshot["contacts"]:
                pair = {
                    contact["first"]["body"],
                    contact["second"]["body"],
                }
                assert not (pair & robot_names and pair - robot_names - {"table"}), pair

        for group in engine._pose_groups:
            initial_positions = np.array(
                [canonical[name]["position_m"] for name in group.bodies]
            )
            positions = np.array([first[name]["position_m"] for name in group.bodies])
            translation = positions[:, :2].mean(axis=0) - initial_positions[:, :2].mean(
                axis=0
            )
            assert np.all(np.abs(translation) <= group.translation_xy_m)
            np.testing.assert_allclose(positions[:, 2], initial_positions[:, 2])

            yaws = []
            for name in group.bodies:
                initial_q = np.asarray(canonical[name]["orientation_wxyz"])
                randomized_q = np.asarray(first[name]["orientation_wxyz"])
                delta = _multiply_quaternions(
                    randomized_q,
                    initial_q * np.array([1.0, -1.0, -1.0, -1.0]),
                )
                np.testing.assert_allclose(delta[1:3], 0.0, atol=1e-12)
                yaw = 2.0 * math.atan2(delta[3], delta[0])
                assert abs(yaw) <= group.yaw_rad
                yaws.append(yaw)
            np.testing.assert_allclose(yaws, yaws[0], atol=1e-12)

            cosine, sine = math.cos(yaws[0]), math.sin(yaws[0])
            rotation = np.array([[cosine, -sine], [sine, cosine]])
            initial_relative = initial_positions[:, :2] - initial_positions[:, :2].mean(
                axis=0
            )
            relative = positions[:, :2] - positions[:, :2].mean(axis=0)
            np.testing.assert_allclose(
                relative, initial_relative @ rotation.T, atol=1e-15
            )

        for part in engine.parts:
            np.testing.assert_allclose(engine.read(part)[0], engine.profile.home)
        for name, value in engine._prop_initial.items():
            assert first_snapshot["joints"][name]["qpos"] == [value]

        assert engine.reset()
        restored = _body_poses(engine.evaluation_snapshot(), names)
        assert restored == canonical
    finally:
        engine.close()


def test_evaluation_reset_refuses_compiled_robot_prop_penetration(
    tmp_path, monkeypatch
):
    pytest.importorskip("mujoco")
    monkeypatch.setenv("MUJOCO_GL", "egl")
    from waddle_sdk.simulators.mujoco import Engine

    _, config = make_site(
        "invalid-reset-test",
        backend="mujoco",
        robot="so101",
        environment="pick_lift",
        width=192,
        height=144,
        arms=1,
        render_quality="fast",
    )
    engine = Engine(config, tmp_path)
    original_randomize = engine._apply_pose_randomization

    def place_prop_inside_robot(seed, level):
        original_randomize(seed, level)
        prop = engine.model.body("target_cube")
        joint_id = int(prop.jntadr[0])
        address = int(engine.model.jnt_qposadr[joint_id])
        robot_geom = next(
            geom_id
            for geom_id in range(engine.model.ngeom)
            if int(engine.model.geom_bodyid[geom_id])
            not in {*engine._prop_body_ids, int(engine.model.body("table").id)}
        )
        engine.data.qpos[address : address + 3] = engine.data.geom_xpos[robot_geom]

    monkeypatch.setattr(engine, "_apply_pose_randomization", place_prop_inside_robot)
    try:
        assert not engine.evaluation_reset(seed=1)
    finally:
        engine.close()


def test_candy_bin_transfer_keeps_one_color_under_appearance_variation(
    tmp_path, monkeypatch
):
    pytest.importorskip("mujoco")
    monkeypatch.setenv("MUJOCO_GL", "egl")
    from waddle_sdk.simulators.mujoco import Engine

    _, config = make_site(
        "candy-color-test",
        backend="mujoco",
        robot="xarm7",
        environment="candy-bin-transfer",
        width=192,
        height=144,
    )
    engine = Engine(config, tmp_path)
    try:
        assert engine.evaluation_reset(
            seed=712,
            variation=SimulationVariation(appearance="bounded").as_dict(),
        )
        colors = []
        for index in range(1, 19):
            body = engine.model.body(f"candy_{index}")
            for geom_id in range(
                int(body.geomadr[0]), int(body.geomadr[0] + body.geomnum[0])
            ):
                colors.append(engine.model.geom_rgba[geom_id])
        np.testing.assert_allclose(colors, np.repeat(colors[:1], len(colors), axis=0))
    finally:
        engine.close()


def test_evaluation_reset_refuses_compiled_cross_arm_penetration(tmp_path, monkeypatch):
    pytest.importorskip("mujoco")
    monkeypatch.setenv("MUJOCO_GL", "egl")
    from waddle_sdk.simulators.mujoco import Engine

    _, config = make_site(
        "invalid-dual-reset-test",
        backend="mujoco",
        robot="so101",
        environment="handover-block",
        width=192,
        height=144,
        arms=2,
        render_quality="fast",
    )
    engine = Engine(config, tmp_path)
    original_randomize = engine._apply_pose_randomization

    def overlap_robot_bases(seed, level):
        original_randomize(seed, level)
        left = engine.model.body("left__base")
        right = engine.model.body("right__base")
        right.pos[:] = left.pos
        right.quat[:] = left.quat

    monkeypatch.setattr(engine, "_apply_pose_randomization", overlap_robot_bases)
    try:
        assert not engine.evaluation_reset(seed=1)
    finally:
        engine.close()


def test_reset_profiles_change_only_the_selected_trusted_dimension(
    tmp_path, monkeypatch
):
    pytest.importorskip("mujoco")
    monkeypatch.setenv("MUJOCO_GL", "egl")
    from waddle_sdk.simulators.mujoco import Engine

    _, config = make_site(
        "variation-test",
        backend="mujoco",
        robot="so101",
        environment="insert-usb",
        width=192,
        height=144,
        arms=1,
        render_quality="fast",
    )
    engine = Engine(config, tmp_path)
    try:
        prop_geoms = np.asarray(engine._prop_geom_ids)
        prop_bodies = np.asarray(engine._prop_body_ids)
        canonical_rgba = engine.model.geom_rgba[prop_geoms].copy()
        canonical_size = engine.model.geom_size[prop_geoms].copy()
        canonical_mass = engine.model.body_mass[prop_bodies].copy()
        canonical_friction = engine.model.geom_friction[prop_geoms].copy()
        canonical_pose = _body_poses(
            engine.evaluation_snapshot(), tuple(engine._prop_links)
        )
        canonical_rgb, _ = engine.capture("scene")

        appearance = SimulationVariation(appearance="bounded")
        assert engine.evaluation_reset(seed=712, variation=appearance.as_dict())
        appearance_snapshot = engine.evaluation_snapshot()
        assert appearance_snapshot["variation"]["profile"] == dict(appearance.as_dict())
        appearance_digest = appearance_snapshot["variation"]["resolved_digest"]
        assert appearance_digest.startswith("sha256:")
        assert not np.array_equal(engine.model.geom_rgba[prop_geoms], canonical_rgba)
        assert np.array_equal(engine.model.geom_size[prop_geoms], canonical_size)
        np.testing.assert_allclose(engine.model.body_mass[prop_bodies], canonical_mass)
        assert (
            _body_poses(appearance_snapshot, tuple(engine._prop_links))
            == canonical_pose
        )
        changed_rgb, _ = engine.capture("scene")
        assert not np.array_equal(changed_rgb, canonical_rgb)
        first_rgba = engine.model.geom_rgba[prop_geoms].copy()
        assert engine.evaluation_reset(seed=712, variation=appearance.as_dict())
        np.testing.assert_array_equal(engine.model.geom_rgba[prop_geoms], first_rgba)
        assert (
            engine.evaluation_snapshot()["variation"]["resolved_digest"]
            == appearance_digest
        )
        assert engine.evaluation_reset(seed=711, variation=appearance.as_dict())
        assert (
            engine.evaluation_snapshot()["variation"]["resolved_digest"]
            != appearance_digest
        )

        physics = SimulationVariation(physics="bounded")
        assert engine.evaluation_reset(seed=713, variation=physics.as_dict())
        np.testing.assert_array_equal(
            engine.model.geom_rgba[prop_geoms], canonical_rgba
        )
        np.testing.assert_array_equal(
            engine.model.geom_size[prop_geoms], canonical_size
        )
        assert not np.array_equal(engine.model.body_mass[prop_bodies], canonical_mass)
        assert not np.array_equal(
            engine.model.geom_friction[prop_geoms], canonical_friction
        )
        first_mass = engine.model.body_mass[prop_bodies].copy()
        assert engine.evaluation_reset(seed=713, variation=physics.as_dict())
        np.testing.assert_array_equal(engine.model.body_mass[prop_bodies], first_mass)

        geometry = SimulationVariation(geometry="held_out")
        assert engine.evaluation_reset(seed=714, variation=geometry.as_dict())
        changed_size = engine.model.geom_size[prop_geoms]
        ratios = changed_size[canonical_size > 0] / canonical_size[canonical_size > 0]
        np.testing.assert_allclose(ratios, ratios[0], atol=1e-12)
        assert 0.96 <= ratios[0] <= 0.98 or 1.02 <= ratios[0] <= 1.04
        np.testing.assert_array_equal(
            engine.model.geom_rgba[prop_geoms], canonical_rgba
        )
        np.testing.assert_allclose(engine.model.body_mass[prop_bodies], canonical_mass)
        first_size = engine.model.geom_size[prop_geoms].copy()
        assert engine.evaluation_reset(seed=714, variation=geometry.as_dict())
        np.testing.assert_array_equal(engine.model.geom_size[prop_geoms], first_size)

        with pytest.raises(ValueError, match="nonzero seed"):
            engine.evaluation_reset(seed=0, variation=appearance.as_dict())

        assert engine.reset()
        np.testing.assert_array_equal(
            engine.model.geom_rgba[prop_geoms], canonical_rgba
        )
        np.testing.assert_array_equal(
            engine.model.geom_size[prop_geoms], canonical_size
        )
        np.testing.assert_allclose(engine.model.body_mass[prop_bodies], canonical_mass)
        assert engine.evaluation_snapshot()["variation"]["profile"] == dict(
            SimulationVariation().as_dict()
        )
    finally:
        engine.close()


def test_physics_profile_varies_passive_fixture_damping(tmp_path, monkeypatch):
    pytest.importorskip("mujoco")
    monkeypatch.setenv("MUJOCO_GL", "egl")
    from waddle_sdk.simulators.mujoco import Engine

    _, config = make_site(
        "damping-variation-test",
        backend="mujoco",
        robot="so101",
        environment="drawer",
        width=192,
        height=144,
        arms=1,
        render_quality="fast",
    )
    engine = Engine(config, tmp_path)
    try:
        prop_dofs = np.asarray(engine._prop_dof_ids)
        canonical = engine.model.dof_damping[prop_dofs].copy()
        assert np.any(canonical > 0)
        variation = SimulationVariation(physics="bounded")
        assert engine.evaluation_reset(seed=715, variation=variation.as_dict())
        changed = engine.model.dof_damping[prop_dofs]
        assert not np.array_equal(changed, canonical)
        assert np.all(changed[canonical > 0] > 0)
    finally:
        engine.close()


def test_contact_events_retain_a_physics_step_contact_until_reset(
    tmp_path, monkeypatch
):
    pytest.importorskip("mujoco")
    monkeypatch.setenv("MUJOCO_GL", "egl")
    from waddle_sdk.simulators.mujoco import Engine

    _, config = make_site(
        "contact-event-test",
        backend="mujoco",
        robot="so101",
        environment="insert-usb",
        width=192,
        height=144,
        arms=1,
        render_quality="fast",
    )
    engine = Engine(config, tmp_path)
    try:
        body = engine.model.body("usb_connector")
        joint_id = int(body.jntadr[0])
        qpos = int(engine.model.jnt_qposadr[joint_id])
        dof = int(engine.model.jnt_dofadr[joint_id])
        engine.data.qpos[qpos : qpos + 7] = (0.35, 0.10, 0.12, 1.0, 0.0, 0.0, 0.0)
        engine.data.qvel[dof : dof + 6] = 0.0
        engine.data.xfrc_applied[int(body.id), 0] = 3.0
        engine.mj.mj_forward(engine.model, engine.data)
        for _ in range(round(1.0 / config["timestep"])):
            engine.step()
        engine.data.xfrc_applied[int(body.id)] = 0.0

        engine.data.qpos[qpos : qpos + 7] = (
            0.25,
            -0.12,
            0.05,
            1.0,
            0.0,
            0.0,
            0.0,
        )
        engine.data.qvel[dof : dof + 6] = 0.0
        engine.mj.mj_forward(engine.model, engine.data)
        state = engine.evaluation_snapshot()
        assert not any(
            {row["first"]["body"], row["second"]["body"]}
            == {"usb_connector", "usb_port"}
            for row in state["contacts"]
        )
        event = next(
            row
            for row in state["contact_events"]["pairs"]
            if {row["first"]["body"], row["second"]["body"]}
            == {"usb_connector", "usb_port"}
        )
        assert event["maximum_normal_force_n"] > 0.0
        assert event["minimum_distance_m"] <= 0.0
        assert event["contact_samples"] > 0
        assert (
            0.0
            <= event["first_time_s"]
            <= event["last_time_s"]
            <= state["contact_events"]["through_time_s"]
            <= state["time_s"]
        )

        assert engine.reset()
        reset = engine.evaluation_snapshot()
        assert reset["contact_events"]["pairs"] == []
        assert reset["contact_events"]["through_time_s"] == 0.0
    finally:
        engine.close()
