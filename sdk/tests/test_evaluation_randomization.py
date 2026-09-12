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
