"""Authored source reuse changes only edited components and reads fresh assets."""

from copy import deepcopy
from pathlib import Path

import pytest

pytest.importorskip("mujoco")

from test_authored_preparation import same_model
from test_playground import scene
from waddle_sdk.simulators import playground
from waddle_sdk.simulators.scene import make_site


def test_edit_reuses_robot_and_unchanged_object_sources(tmp_path, monkeypatch):
    monkeypatch.setenv("WADDLE_SIMULATION_MODEL_CACHE", str(tmp_path / "cache"))
    _, config = make_site(
        "source", backend="mujoco", robot="yam", environment="two_cubes"
    )
    authored = scene()
    authored["objects"] = [
        obj for obj in authored["objects"] if obj["kind"] in {"bowl", "block"}
    ]
    references, objects = [], []
    reference, object_source = playground.mjcf, playground._object

    def build_reference(*args, **kwargs):
        references.append(True)
        return reference(*args, **kwargs)

    def build_object(world, asset, obj, easy_grasp):
        objects.append(obj["id"])
        return object_source(world, asset, obj, easy_grasp)

    monkeypatch.setattr(playground, "mjcf", build_reference)
    monkeypatch.setattr(playground, "_object", build_object)
    playground.prepare_scene(config, authored, tmp_path / "first")
    references.clear()
    objects.clear()
    edited = deepcopy(authored)
    added = next(obj for obj in scene()["objects"] if obj["kind"] == "ball")
    edited["objects"].append(added)
    prepared = playground.prepare_scene(config, edited, tmp_path / "edited")
    assert references == []
    assert objects == [added["id"]]
    with monkeypatch.context() as fresh:
        fresh.delenv("WADDLE_SIMULATION_MODEL_CACHE")
        expected = playground.prepare_scene(config, edited, tmp_path / "fresh")
    same_model(prepared.model, expected.model)
    assert prepared.contents == {
        name: (tmp_path / "edited" / name).read_bytes() for name in prepared.contents
    }


@pytest.mark.parametrize("write", ["restored_mtime", "hardlink", "mapping"])
def test_reused_sources_read_asset_changes_without_metadata_changes(
    tmp_path, monkeypatch, write
):
    import mmap
    import os

    from waddle_sdk.simulators import model

    monkeypatch.setenv("WADDLE_SIMULATION_MODEL_CACHE", str(tmp_path / "cache"))
    original_objects = model.objects
    texture = tmp_path / "table.png"
    texture.write_bytes(
        Path(original_objects("two_cubes")[0][0].shapes[0].texture).read_bytes()
    )

    def objects(*args, **kwargs):
        props = original_objects(*args, **kwargs)
        props[0][0].shapes[0].texture = str(texture)
        return props

    monkeypatch.setattr(model, "objects", objects)
    _, config = make_site(
        "source", backend="mujoco", robot="so101", environment="two_cubes"
    )
    authored = scene()
    authored["objects"] = authored["objects"][:1]
    with texture.open("r+b") as stream, mmap.mmap(stream.fileno(), 0) as mapping:
        playground.prepare_scene(config, authored, tmp_path / "first")
        stamp = texture.stat()
        if write == "mapping":
            mapping[:8] = b"invalid!"
            mapping.flush()
        else:
            target = texture
            if write == "hardlink":
                target = tmp_path / "outside-alias.png"
                os.link(texture, target)
            with target.open("r+b") as changed:
                changed.write(b"invalid!")
            os.utime(texture, ns=(stamp.st_atime_ns, stamp.st_mtime_ns))
        with pytest.raises(ValueError):
            playground.prepare_scene(config, authored, tmp_path / "invalid")


def test_source_reuse_tracks_reference_configuration_and_description(
    tmp_path, monkeypatch
):
    from waddle_sdk.simulators import description

    monkeypatch.setenv("WADDLE_SIMULATION_MODEL_CACHE", str(tmp_path / "cache"))
    robot = deepcopy(description.description("so101"))
    monkeypatch.setattr(description, "description", lambda name: robot)
    _, config = make_site(
        "source", backend="mujoco", robot="so101", environment="two_cubes"
    )
    authored = scene()
    authored["objects"] = authored["objects"][:1]
    first = playground.prepare_scene(config, authored, tmp_path / "first")
    robot.exclusions = (*robot.exclusions, (robot.links[0].name, robot.links[-1].name))
    config["timestep"] /= 2
    config["render_quality"] = "high"
    changed = playground.prepare_scene(config, authored, tmp_path / "changed")
    assert changed.contents["world.xml"] != first.contents["world.xml"]
    with monkeypatch.context() as fresh:
        fresh.delenv("WADDLE_SIMULATION_MODEL_CACHE")
        expected = playground.prepare_scene(config, authored, tmp_path / "fresh")
    same_model(changed.model, expected.model)


@pytest.mark.parametrize(
    "fixture,arms", [("loaded_tray", 2), ("drawer_cabinet", 1), ("cleanup_station", 1)]
)
def test_reference_fixture_placement_changes_match_fresh_source(
    tmp_path, monkeypatch, fixture, arms
):
    monkeypatch.setenv("WADDLE_SIMULATION_MODEL_CACHE", str(tmp_path / "cache"))
    _, config = make_site(
        "fixture", backend="mujoco", robot="yam", arms=arms, environment="two_cubes"
    )
    authored = scene()
    authored.update(fixture=fixture, seed=0, objects=authored["objects"][:1])
    first = playground.prepare_scene(config, authored, tmp_path / "first")
    authored["seed"] = 41
    changed = playground.prepare_scene(config, authored, tmp_path / "changed")
    assert changed.contents["world.xml"] != first.contents["world.xml"]
    repeated = playground.prepare_scene(config, authored, tmp_path / "repeated")
    with monkeypatch.context() as fresh:
        fresh.delenv("WADDLE_SIMULATION_MODEL_CACHE")
        expected = playground.prepare_scene(config, authored, tmp_path / "fresh")
    same_model(changed.model, expected.model)
    same_model(repeated.model, expected.model)


def test_source_entries_are_detached_bounded_and_dropped_after_fork(monkeypatch):
    import os
    import xml.etree.ElementTree as ET

    from waddle_sdk.simulators.playground_source import SourceCache

    cache = SourceCache(max_entries=2, max_bytes=1024)

    def reference():
        return ET.fromstring("<mujoco><asset/><worldbody/></mujoco>"), (), ()

    first, *_ = cache.prepare("first", reference, None, [], True)
    ET.SubElement(first.find("worldbody"), "body", name="caller_change")
    again, *_ = cache.prepare("first", reference, None, [], True)
    assert not list(again.find("worldbody"))
    cache.prepare("second", reference, None, [], True)
    cache.prepare("third", reference, None, [], True)
    assert list(cache.entries) == ["second", "third"]
    cache.max_bytes = 1
    cache.prepare("large", reference, None, [], True)
    assert not cache.entries
    cache.max_bytes = 1024
    cache.prepare("before_fork", reference, None, [], True)
    pid = os.getpid()
    monkeypatch.setattr(os, "getpid", lambda: pid + 1)
    cache.prepare("after_fork", reference, None, [], True)
    assert list(cache.entries) == ["after_fork"]
