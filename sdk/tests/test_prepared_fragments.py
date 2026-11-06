"""Repeated additions reuse parsed shapes/pairs, never mutable scene state."""

from copy import deepcopy
from xml.etree import ElementTree as ET

import numpy as np
import pytest
from test_authored_preparation import same_model
from waddle_sdk.simulators import source_buffers
from waddle_sdk.simulators.playground import CATALOG, prepare_scene
from waddle_sdk.simulators.scene import make_site

mj = pytest.importorskip("mujoco")


@pytest.mark.parametrize("kind", list(CATALOG))
def test_repeated_object_fragment_keeps_native_parity(tmp_path, monkeypatch, kind):
    buffers = source_buffers.SourceBuffers()
    monkeypatch.setattr(source_buffers, "cache", buffers)
    monkeypatch.setenv("WADDLE_SIMULATION_MODEL_CACHE", str(tmp_path / "cache"))
    _, config = make_site(
        "fragments",
        backend="mujoco",
        robot="yam",
        arms=2,
        environment="two_cubes",
        width=96,
        height=72,
    )
    config["render_quality"] = "fast"
    scene = {"id": "test", "name": "Test", "objects": []}
    prepare_scene(config, scene, tmp_path / "empty")
    size = CATALOG[kind]["size"]
    scene["objects"].append(
        {
            "id": "first",
            "kind": kind,
            "size": size,
            "position": [0.3, 0.1, size[2] / 2 + 0.001],
            "rotation": [0, 0, 0],
            "color": [0.3, 0.6, 0.8],
        }
    )
    first = prepare_scene(config, scene, tmp_path / "first")
    untouched = first.model.qpos0.copy()
    second = deepcopy(scene["objects"][0])
    second.update(
        id="second", position=[0.4, -0.1, size[2] / 2 + 0.005], rotation=[0, 0, 30]
    )
    scene["objects"].append(second)

    def unexpected(*args, **kwargs):
        pytest.fail("Repeated object reparsed its prepared contact fragment")

    with monkeypatch.context() as cached:
        cached.setattr(mj.MjSpec, "from_string", unexpected)
        cached.setattr(mj.MjSpec, "from_file", unexpected)
        actual = prepare_scene(config, scene, tmp_path / "second")
    with monkeypatch.context() as fresh:
        fresh.delenv("WADDLE_SIMULATION_MODEL_CACHE")
        expected = prepare_scene(config, scene, tmp_path / "fresh")
    assert actual.contents == expected.contents
    same_model(actual.model, expected.model)
    np.testing.assert_array_equal(first.model.qpos0, untouched)
    buffers.close()


def fragment(prefix="prop_first", *, friction=".8 .8 .001 .0001 .0001"):
    return ET.fromstring(f'''<mujoco><compiler angle="radian"/><asset>
      <mesh name="{prefix}_mesh" vertex="0 0 0 .1 0 0 0 .1 0 0 0 .1"/>
      </asset><worldbody><body name="{prefix}" pos=".1 .2 .3" quat="1 0 0 0">
      <freejoint name="{prefix}_joint"/>
      <geom name="{prefix}_geom" type="mesh" mesh="{prefix}_mesh"/>
      </body></worldbody><contact><pair geom1="finger" geom2="{prefix}_geom"
      friction="{friction}" solimp=".95 .99 .001"/></contact></mujoco>''')


def attach(cache, delta):
    parent = mj.MjSpec.from_string("""<mujoco><worldbody>
      <geom name="finger" size=".02"/>
      <geom name="different_finger" size=".02"/>
      <geom name="__waddle_prepared_prop_geom" size=".02"/>
      </worldbody></mujoco>""")
    parent.copy_during_attach = True
    cache.attach(parent, delta)
    return parent


def test_fragment_key_retains_geometry_material_and_external_references():
    from waddle_sdk.simulators.prepared_fragments import FragmentCache

    cache = FragmentCache()
    original = fragment()
    serialized = ET.tostring(original)
    first = attach(cache, original)
    second = attach(cache, fragment("prop_second"))
    assert len(cache.entries) == 1
    assert ET.tostring(original) == serialized
    assert first.body("prop_first") is not None
    assert second.body("prop_second") is not None
    second.pairs[0].friction[0] = 0.01
    assert attach(cache, fragment("prop_third")).pairs[0].friction[0] == 0.8
    changed = fragment(friction=".4 .4 .002 .0002 .0002")
    assert attach(cache, changed).pairs[0].friction[0] == 0.4
    assert len(cache.entries) == 2
    changed.find("contact/pair").set("geom1", "different_finger")
    assert attach(cache, changed).pairs[0].geomname1 == "different_finger"
    assert len(cache.entries) == 2
    changed.find("asset/mesh").set("vertex", "0 0 0 .2 0 0 0 .1 0 0 0 .1")
    assert attach(cache, changed).meshes[0].uservert[3] == pytest.approx(0.2)
    cache.close()
    assert not cache.entries


def test_fragment_bounds_and_ambiguous_names_keep_ordinary_parser():
    from waddle_sdk.simulators.prepared_fragments import FragmentCache

    cache = FragmentCache(max_bytes=1)
    assert attach(cache, fragment()).body("prop_first") is not None
    assert not cache.entries
    cache = FragmentCache(max_pairs=0)
    assert attach(cache, fragment()).pairs[0].geomname1 == "finger"
    assert not cache.entries
    cache = FragmentCache()
    ambiguous = fragment()
    ambiguous.find("contact/pair").set("geom1", "__waddle_prepared_prop_geom")
    assert attach(cache, ambiguous).pairs[0].geomname1 == "__waddle_prepared_prop_geom"
    assert not cache.entries
    invalid = fragment()
    invalid.find("contact/pair").set("solimp", "not-a-number")
    with pytest.raises(ValueError):
        attach(cache, invalid)
    assert not cache.entries


def test_failed_attachment_restores_private_template(monkeypatch):
    from waddle_sdk.simulators.prepared_fragments import FragmentCache

    cache = FragmentCache()
    attach(cache, fragment())

    def fail(*args, **kwargs):
        raise ValueError("attachment refused")

    with monkeypatch.context() as failed:
        failed.setattr(mj.MjSpec, "attach", fail)
        with pytest.raises(ValueError, match="attachment refused"):
            attach(cache, fragment("prop_failed"))
    result = attach(cache, fragment("prop_after_failure"))
    assert len(cache.entries) == 1
    assert result.body("prop_after_failure") is not None
    assert len(result.pairs) == 1
    assert result.pairs[0].geomname2 == "prop_after_failure_geom"
    np.testing.assert_array_equal(
        result.body("prop_after_failure").pos, [0.1, 0.2, 0.3]
    )


def test_attachment_always_creates_independent_parent_copies():
    from waddle_sdk.simulators.prepared_fragments import FragmentCache

    cache = FragmentCache()
    parent = mj.MjSpec.from_string(
        '<mujoco><worldbody><geom name="finger" size=".02"/></worldbody></mujoco>'
    )
    parent.copy_during_attach = False
    cache.attach(parent, fragment())
    parent.copy_during_attach = False
    cache.attach(parent, fragment("prop_second"))
    assert parent.body("prop_first") is not None
    assert parent.body("prop_second") is not None
    assert len(parent.pairs) == 2
    assert parent.compile().npair == 2
