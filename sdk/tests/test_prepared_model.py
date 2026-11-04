"""Private whole-scene handoffs retain source identity and independent models."""

import hashlib
import os

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(
    not hasattr(os, "memfd_create"), reason="sealed model handoff requires Linux"
)


def test_sealed_handoff_survives_sender_release_and_cannot_be_changed():
    mj = pytest.importorskip("mujoco")
    import multiprocessing

    from waddle_sdk.simulators.prepared_model import PreparedModelCache, restore

    xml = b'<mujoco><worldbody><geom type="sphere" size=".1"/></worldbody></mujoco>'
    files = {"model.xml": hashlib.sha256(xml).hexdigest()}
    cache = PreparedModelCache()
    contract = [{"arm": "arm__"}, {"scene": "scene"}, "original comparison"]
    cache.remember(
        "model.xml",
        files,
        xml,
        mj.MjModel.from_xml_string(xml.decode()),
        _contract=contract,
    )
    payload = cache.get("model.xml", files)
    contract[-1] = "caller mutation"
    a, b = multiprocessing.Pipe()
    try:
        a.send(payload)
        payload.close()
        cache.close()
        received = b.recv()
        try:
            with pytest.raises(PermissionError):
                os.pwrite(received.descriptor(), b"changed", 0)
            assert restore("model.xml", files, received).geom_size[0, 0] == 0.1
            config = {
                "native_scene": {
                    "model": "model.xml",
                    "files": files,
                    "prefixes": {"arm": "arm__"},
                    "camera_names": {"scene": "scene"},
                }
            }
            value = received.replacement_contract(config)
            assert value[-1] == "original comparison"
            value[-1] = "consumer mutation"
            assert received.replacement_contract(config)[-1] == "original comparison"
            config["native_scene"]["prefixes"] = {"arm": "different__"}
            assert received.replacement_contract(config) is None
            config["native_scene"]["prefixes"] = {"arm": "arm__"}
            config["native_scene"]["camera_names"] = {"scene": "different"}
            assert received.replacement_contract(config) is None
            config["native_scene"]["camera_names"] = {"scene": "scene"}
            config["native_scene"]["files"] = {"model.xml": "changed"}
            assert received.replacement_contract(config) is None
        finally:
            received.close()
        with pytest.raises(ValueError, match="prepared_closed"):
            restore("model.xml", files, received)
    finally:
        payload.close()
        cache.close()
        a.close()
        b.close()


def test_unavailable_sealing_keeps_ordinary_loading(monkeypatch):
    mj = pytest.importorskip("mujoco")
    from waddle_sdk.simulators.prepared_model import PreparedModelCache

    xml = b'<mujoco><worldbody><geom type="sphere" size=".1"/></worldbody></mujoco>'
    files = {"model.xml": hashlib.sha256(xml).hexdigest()}

    def unavailable(*args, **kwargs):
        raise OSError("unavailable")

    monkeypatch.setattr(os, "memfd_create", unavailable)
    cache = PreparedModelCache()
    cache.remember("model.xml", files, xml, mj.MjModel.from_xml_string(xml.decode()))
    assert cache.get("model.xml", files) is None


def test_unsealed_model_is_refused_before_native_loading(monkeypatch):
    mj = pytest.importorskip("mujoco")
    from waddle_sdk.simulators.prepared_model import (
        PreparedModel,
        _identity,
        _runtime,
        restore,
    )

    files = {"model.xml": "source-digest"}
    fd = os.memfd_create("unsealed-model", os.MFD_CLOEXEC | os.MFD_ALLOW_SEALING)
    prepared = PreparedModel(_identity("model.xml", files), _runtime(), b"", fd, 0)

    def unexpected_load(*args, **kwargs):
        raise AssertionError("mutable native bytes must not reach MuJoCo")

    monkeypatch.setattr(mj.MjModel, "from_binary_path", unexpected_load)
    try:
        with pytest.raises(ValueError, match="prepared_mutable"):
            restore("model.xml", files, prepared)
    finally:
        prepared.close()


@pytest.mark.parametrize("native_failure", [False, True])
def test_failed_export_closes_descriptor_and_keeps_normal_loading(
    monkeypatch, native_failure
):
    mj = pytest.importorskip("mujoco")
    from waddle_sdk.simulators.prepared_model import PreparedModelCache

    xml = b'<mujoco><worldbody><geom type="sphere" size=".1"/></worldbody></mujoco>'
    files = {"model.xml": hashlib.sha256(xml).hexdigest()}
    descriptors = []
    create = os.memfd_create

    def opened(*args, **kwargs):
        fd = create(*args, **kwargs)
        descriptors.append(fd)
        return fd

    def failed_save(*args, **kwargs):
        if native_failure:
            raise mj.FatalError("cannot export compiled model")

    monkeypatch.setattr(os, "memfd_create", opened)
    monkeypatch.setattr(mj, "mj_saveModel", failed_save)
    cache = PreparedModelCache()
    cache.remember("model.xml", files, xml, mj.MjModel.from_xml_string(xml.decode()))
    assert cache.get("model.xml", files) is None
    assert len(descriptors) == 1
    with pytest.raises(OSError):
        os.fstat(descriptors[0])


def test_prepared_handoff_is_portable_independent_and_digest_bound(monkeypatch):
    mj = pytest.importorskip("mujoco")
    from waddle_sdk.simulators.prepared_model import PreparedModelCache, restore

    xml = b'<mujoco><worldbody><body><freejoint/><geom type="sphere" size=".1"/></body></worldbody></mujoco>'
    files = {"world.xml": hashlib.sha256(xml).hexdigest()}
    model = mj.MjModel.from_xml_string(xml.decode())
    cache = PreparedModelCache()
    cache.remember("world.xml", files, xml, model)
    payload = cache.get("nested/model.xml", {"nested/model.xml": files["world.xml"]})
    assert payload is not None
    model.geom_size[:] *= 3
    first = restore(
        "nested/model.xml", {"nested/model.xml": files["world.xml"]}, payload
    )
    second = restore("world.xml", files, payload)
    first.geom_size[:] *= 2
    np.testing.assert_array_equal(second.geom_size[0], [0.1, 0, 0])
    assert first is not second
    assert cache.get("world.xml", {"world.xml": "changed"}) is None
    with pytest.raises(ValueError, match="prepared_source"):
        restore("world.xml", {"world.xml": "changed"}, payload)
    monkeypatch.setattr(mj, "__version__", "incompatible")
    assert restore("world.xml", files, payload) is None


def test_prepared_handoff_cache_has_entry_and_byte_bounds():
    mj = pytest.importorskip("mujoco")
    from waddle_sdk.simulators.prepared_model import PreparedModelCache

    cache = PreparedModelCache(max_bytes=1)
    xml = b'<mujoco><worldbody><geom type="sphere" size=".1"/></worldbody></mujoco>'
    files = {"world.xml": hashlib.sha256(xml).hexdigest()}
    model = mj.MjModel.from_xml_string(xml.decode())
    cache.remember("world.xml", files, xml, model)
    assert cache.get("world.xml", files) is None
    cache.max_bytes = 1_000_000
    versions = []
    for size in (".1", ".2", ".3"):
        data = xml.replace(b".1", size.encode())
        inventory = {"world.xml": hashlib.sha256(data).hexdigest()}
        versions.append(inventory)
        cache.remember(
            "world.xml", inventory, data, mj.MjModel.from_xml_string(data.decode())
        )
    assert cache.get("world.xml", versions[0]) is None
    assert cache.get("world.xml", versions[-1]) is not None


def test_prepared_contract_counts_toward_handoff_byte_limit():
    mj = pytest.importorskip("mujoco")
    from waddle_sdk.simulators.prepared_model import PreparedModelCache

    xml = b'<mujoco><worldbody><geom type="sphere" size=".1"/></worldbody></mujoco>'
    files = {"world.xml": hashlib.sha256(xml).hexdigest()}
    model = mj.MjModel.from_xml_string(xml.decode())
    cache = PreparedModelCache()
    try:
        assert cache.remember("world.xml", files, xml, model)
        cache.max_bytes = next(iter(cache.entries.values())).size
        assert not cache.remember("world.xml", files, xml, model, _contract=["extra"])
        assert cache.get("world.xml", files) is None
    finally:
        cache.close()


def test_same_call_robot_signature_matches_bytes_without_mutating_tree():
    import xml.etree.ElementTree as ET

    from waddle_sdk.simulators.native_scene import replacement_signature

    xml = b"""<mujoco><asset>
      <mesh name="robot" file="robot.stl" content_type="model/stl"/>
      <material name="paint" texture="color"/><texture name="color" file="color.png"/>
      </asset><worldbody><body name="arm__base">
      <geom mesh="robot" material="paint"/></body></worldbody></mujoco>"""
    config = {
        "native_scene": {
            "model": "world.xml",
            "prefixes": {"arm": "arm__"},
            "camera_names": {},
            "files": {
                "world.xml": hashlib.sha256(xml).hexdigest(),
                "robot.stl": "mesh bytes",
                "color.png": "texture bytes",
            },
        }
    }
    tree = ET.fromstring(xml)
    original = ET.tostring(tree)
    signature = replacement_signature(config, _tree=tree)
    assert signature == replacement_signature(config, _contents={"world.xml": xml})
    assert ET.tostring(tree) == original
    assert signature[-1] == {"robot.stl": "mesh bytes", "color.png": "texture bytes"}
    assert "content_type" not in "".join(signature[-2])


def test_cached_authored_model_still_checks_sources_at_owner_open(
    tmp_path, monkeypatch
):
    pytest.importorskip("mujoco")
    from test_playground import scene
    from waddle_sdk.simulators.adapters import World
    from waddle_sdk.simulators.playground import prepare_scene
    from waddle_sdk.simulators.prepared_model import cache
    from waddle_sdk.simulators.scene import load_scene, make_site

    monkeypatch.setenv("WADDLE_SIMULATION_MODEL_CACHE", str(tmp_path / "cache"))
    _, config = make_site(
        "handoff", backend="mujoco", robot="yam", environment="two_cubes"
    )
    root = tmp_path / "source"
    prepare_scene(config, scene(), root)
    _, definition = load_scene(root, "simulation.json")
    native = definition["native_scene"]
    assert cache.get(native["model"], native["files"]) is not None
    world = World(definition, real_time=False)
    (root / "world.xml").write_bytes(b"changed after declaration")
    try:
        with pytest.raises(ValueError, match="digest differs"):
            world.open()
        assert world._process is None
    finally:
        world.close()
