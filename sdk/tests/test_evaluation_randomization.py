"""Native acceptance for private seeded task-scene reset."""

from __future__ import annotations

import math

import numpy as np
import pytest
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
        assert engine.evaluation_reset(seed=712)
        assert _body_poses(engine.evaluation_snapshot(), names) == first

        assert engine.evaluation_reset(seed=713)
        second_snapshot = engine.evaluation_snapshot()
        second = _body_poses(second_snapshot, names)
        assert second != first

        assert engine.evaluation_reset(seed=0)
        zero_snapshot = engine.evaluation_snapshot()
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
