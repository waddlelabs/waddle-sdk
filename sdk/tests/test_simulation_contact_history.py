"""Recorded contact extrema/counts agree with the native per-point force API."""

import numpy as np
import pytest
from waddle_sdk.simulators.scene import make_site


def test_presentation_reads_match_current_poses_without_recomputing_effort(tmp_path):
    pytest.importorskip("mujoco")
    from waddle_sdk.simulators.mujoco import Engine

    _, config = make_site(
        "pose-read",
        backend="mujoco",
        robot="yam",
        environment="two_cubes",
        width=32,
        height=32,
    )
    engine = Engine(config, tmp_path)
    try:
        q = engine.read()[0]
        q[0] += 0.05
        engine.write(q)
        for _ in range(20):
            engine.step()
        before = engine.data.actuator_force.copy()
        integrated = engine.data.qpos.copy()
        poses = engine.presentation_snapshot()
        np.testing.assert_array_equal(engine.data.actuator_force, before)
        np.testing.assert_array_equal(engine.data.qpos, integrated)
        full = engine.evaluation_snapshot()
        assert poses["time_s"] == full["time_s"]
        for name, body in poses["bodies"].items():
            np.testing.assert_array_equal(
                body["position_m"], full["bodies"][name]["position_m"]
            )
            np.testing.assert_array_equal(
                body["orientation_wxyz"], full["bodies"][name]["orientation_wxyz"]
            )
        for name, joint in poses["joints"].items():
            assert joint == {key: full["joints"][name][key] for key in ("type", "qpos")}
    finally:
        engine.close()


@pytest.mark.parametrize("cone", ["elliptic", "pyramidal"])
def test_contact_history_retains_native_manifolds_and_reset(tmp_path, cone):
    mj = pytest.importorskip("mujoco")
    from waddle_sdk.simulators.mujoco import Engine

    _, config = make_site(
        "contact-history",
        backend="mujoco",
        robot="yam",
        environment="load-clear-test-tubes",
        width=32,
        height=32,
    )
    engine = Engine(config, tmp_path)
    try:
        engine.model.opt.cone = (
            mj.mjtCone.mjCONE_ELLIPTIC
            if cone == "elliptic"
            else mj.mjtCone.mjCONE_PYRAMIDAL
        )
        expected = {}
        force = np.zeros(6)
        for _ in range(100):
            engine.step()
            now = float(engine.data.time)
            for index, contact in enumerate(engine.data.contact):
                mj.mj_contactForce(engine.model, engine.data, index, force)
                normal = max(0.0, float(force[0]))
                if normal == 0:
                    continue
                key = tuple(sorted((int(contact.geom1), int(contact.geom2))))
                row = expected.setdefault(
                    key,
                    {
                        "first_time_s": now,
                        "last_time_s": now,
                        "minimum_distance_m": float(contact.dist),
                        "maximum_normal_force_n": normal,
                        "contact_samples": 0,
                    },
                )
                row["last_time_s"] = now
                row["minimum_distance_m"] = min(
                    row["minimum_distance_m"], float(contact.dist)
                )
                row["maximum_normal_force_n"] = max(
                    row["maximum_normal_force_n"], normal
                )
                row["contact_samples"] += 1
        assert len(expected) > 1
        assert sum(row["contact_samples"] for row in expected.values()) > 100
        assert engine._contact_events == expected
        assert engine._contact_events_through_s == engine.data.time
        engine.reset()
        assert not engine._contact_events
    finally:
        engine.close()
