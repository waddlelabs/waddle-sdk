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
    from waddle_sdk.simulators import authored_preparation
    from waddle_sdk.simulators.compiled_model import load_model
    from waddle_sdk.simulators.playground import prepare_scene
    from waddle_sdk.simulators.scene import make_site

    parse = authored_preparation.parse
    supplied = []

    def prepared_tree(data, *, _tree=None, _contacts=None):
        assert _tree is not None
        assert _contacts is not None and _contacts.comparison is not None
        supplied.append(True)
        return parse(data, _tree=_tree, _contacts=_contacts)

    monkeypatch.setattr(authored_preparation, "parse", prepared_tree)

    cache = tmp_path / "cache"
    monkeypatch.setenv("WADDLE_SIMULATION_MODEL_CACHE", str(cache))
    _, config = make_site(
        "prepared", backend="mujoco", robot=robot, arms=arms, environment="two_cubes"
    )
    prepared = prepare_scene(config, scene(), tmp_path / "scene", revision=7)
    assert supplied
    native = prepared.config["native_scene"]
    assert prepared.config["scene_revision"] == "1.0.7"
    assert native["files"] == {
        name: hashlib.sha256(value).hexdigest()
        for name, value in prepared.contents.items()
    }
    assert prepared.contents == {
        name: (tmp_path / "scene" / name).read_bytes() for name in native["files"]
    }
    from waddle_sdk.simulators.native_scene import replacement_signature
    from waddle_sdk.simulators.prepared_model import cache as handoffs

    handoff = handoffs.get(native["model"], native["files"])
    try:
        assert handoff.replacement_contract(prepared.config) == replacement_signature(
            prepared.config, _contents=prepared.contents
        )
    finally:
        handoff.close()
    from waddle_sdk.simulators import source_references
    from waddle_sdk.simulators.native_scene import resolve

    # Authoring supplied the exact tree behind the portable bytes. Validation
    # still captures/checks every file, without parsing that XML a second time.
    def reparsed(_data):
        pytest.fail("validated authoring discarded its prepared XML references")

    with monkeypatch.context() as reused:
        reused.setattr(source_references, "references", reparsed)
        captured = {}
        resolve(tmp_path / "scene", prepared.config, contents=captured)
        assert captured == prepared.contents
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


@pytest.mark.parametrize("handoff", ["sealed", "unavailable", "too_large"])
def test_resident_preparation_exports_one_binary_and_keeps_disk_fallback(
    tmp_path, monkeypatch, handoff
):
    import os

    mj = pytest.importorskip("mujoco")
    from waddle_sdk.simulators import prepared_model
    from waddle_sdk.simulators.compiled_model import load_model
    from waddle_sdk.simulators.playground import prepare_scene
    from waddle_sdk.simulators.scene import make_site

    cache = prepared_model.PreparedModelCache(
        max_bytes=1 if handoff == "too_large" else 256 * 1024 * 1024
    )
    monkeypatch.setattr(prepared_model, "cache", cache)
    monkeypatch.setenv("WADDLE_SIMULATION_MODEL_CACHE", str(tmp_path / "cache"))
    if handoff == "unavailable":

        def unavailable(*args, **kwargs):
            raise OSError("sealing unavailable")

        monkeypatch.setattr(os, "memfd_create", unavailable, raising=False)
    _, config = make_site(
        "resident", backend="mujoco", robot="so101", arms=1, environment="two_cubes"
    )
    saved = []
    save = mj.mj_saveModel

    def export(model, filename, *args, **kwargs):
        saved.append(filename)
        return save(model, filename, *args, **kwargs)

    monkeypatch.setattr(mj, "mj_saveModel", export)
    try:
        prepared = prepare_scene(
            config, scene(), tmp_path / "scene", _prefer_handoff=True
        )
        native = prepared.config["native_scene"]
        payload = cache.get(native["model"], native["files"])
        disk_exports = [name for name in saved if not name.startswith("/proc/self/fd/")]
        if handoff == "sealed" and payload is not None:
            assert not disk_exports and len(saved) == 1
            assert not (tmp_path / "cache").exists()
            try:
                model = prepared_model.restore(
                    native["model"], native["files"], payload
                )
            finally:
                payload.close()
        else:
            assert payload is None and len(disk_exports) == 1
            timing = {}
            model = load_model(native["model"], prepared.contents, timings=timing)
            assert timing["cache_hit"]
        assert model is not prepared.model
        np.testing.assert_array_equal(model.key_qpos, prepared.model.key_qpos)
        np.testing.assert_array_equal(model.key_ctrl, prepared.model.key_ctrl)
        sizes = model.geom_size.copy()
        prepared.model.geom_size[:] *= 2
        np.testing.assert_array_equal(model.geom_size, sizes)
    finally:
        cache.close()
