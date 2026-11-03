"""Retained authoring models match portable source and remain owner-independent."""

import hashlib

import numpy as np
import pytest
from test_playground import scene


@pytest.mark.parametrize(
    "robot,arms", [("yam", 1), ("yam", 2), ("so101", 1), ("xarm7", 1)]
)
def test_prepared_scene_matches_source_and_caches_no_runtime_state(
    tmp_path, monkeypatch, robot, arms
):
    mj = pytest.importorskip("mujoco")
    from waddle_sdk.simulators.compiled_model import load_model
    from waddle_sdk.simulators.playground import prepare_scene
    from waddle_sdk.simulators.scene import make_site

    cache = tmp_path / "cache"
    monkeypatch.setenv("WADDLE_SIMULATION_MODEL_CACHE", str(cache))
    _, config = make_site(
        "prepared", backend="mujoco", robot=robot, arms=arms, environment="two_cubes"
    )
    prepared = prepare_scene(config, scene(), tmp_path / "scene", revision=7)
    native = prepared.config["native_scene"]
    assert prepared.config["scene_revision"] == "1.0.7"
    assert native["files"] == {
        name: hashlib.sha256(value).hexdigest()
        for name, value in prepared.contents.items()
    }
    assert prepared.contents == {
        name: (tmp_path / "scene" / name).read_bytes() for name in native["files"]
    }
    fresh = mj.MjModel.from_xml_path(str(tmp_path / "scene/world.xml"))
    for name in (
        "qpos0",
        "key_qpos",
        "key_ctrl",
        "key_qvel",
        "geom_size",
        "geom_rgba",
        "geom_friction",
        "body_mass",
        "body_inertia",
        "actuator_gainprm",
        "actuator_biasprm",
    ):
        np.testing.assert_array_equal(
            getattr(prepared.model, name), getattr(fresh, name)
        )
    prepared.model.geom_size[:] *= 2
    repeated = prepare_scene(config, scene(), tmp_path / "repeated", revision=8)
    np.testing.assert_array_equal(repeated.model.geom_size, fresh.geom_size)
    assert repeated.model is not prepared.model
    timing = {}
    cached = load_model("world.xml", prepared.contents, timings=timing)
    assert timing["cache_hit"]
    np.testing.assert_array_equal(cached.geom_size, fresh.geom_size)
    a, b = mj.MjData(cached), mj.MjData(fresh)
    for model, data in [(cached, a), (fresh, b)]:
        mj.mj_resetDataKeyframe(model, data, model.key("initial").id)
        for _ in range(10):
            mj.mj_step(model, data)
    np.testing.assert_array_equal(a.qpos, b.qpos)
    np.testing.assert_array_equal(a.qvel, b.qvel)
