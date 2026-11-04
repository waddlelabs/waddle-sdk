"""Native file checks share directory work, never stale source contents."""

import hashlib
import mmap
import os
from pathlib import Path

import pytest
from waddle_sdk.simulators.native_scene import resolve


def source(root, *, count=12):
    (root / "assets").mkdir()
    (root / "world.xml").write_bytes(b"<mujoco><worldbody/></mujoco>")
    files = {"world.xml": hashlib.sha256((root / "world.xml").read_bytes()).hexdigest()}
    for index in range(count):
        name = f"assets/{index}.stl"
        data = f"mesh bytes {index}".encode()
        (root / name).write_bytes(data)
        files[name] = hashlib.sha256(data).hexdigest()
    return {
        "parts": {"arm": {}},
        "cameras": {"scene": {}},
        "native_scene": {
            "model": "world.xml",
            "files": files,
            "initial_keyframe": "initial",
            "prefixes": {"arm": "arm__"},
            "camera_names": {"scene": "scene"},
            "pose_groups": [],
        },
    }


@pytest.mark.parametrize("capture", [False, True])
@pytest.mark.parametrize("cached", [False, True])
@pytest.mark.parametrize("mutation", ["write", "hardlink", "mmap"])
def test_one_directory_resolution_per_pass_keeps_fresh_hashes(
    tmp_path, monkeypatch, capture, cached, mutation
):
    if cached:
        monkeypatch.setenv("WADDLE_SIMULATION_MODEL_CACHE", str(tmp_path / "cache"))
    else:
        monkeypatch.delenv("WADDLE_SIMULATION_MODEL_CACHE", raising=False)
    config = source(tmp_path)
    original = Path.resolve
    calls = []

    def resolved(path, *args, **kwargs):
        calls.append(path)
        return original(path, *args, **kwargs)

    monkeypatch.setattr(Path, "resolve", resolved)
    contents = {} if capture else None
    assert resolve(tmp_path, config, contents=contents) == tmp_path / "world.xml"
    assert len(calls) <= 3  # Root and the two distinct source directories.
    if capture:
        assert contents == {
            name: (tmp_path / name).read_bytes()
            for name in config["native_scene"]["files"]
        }
    path = tmp_path / "assets/0.stl"
    stamp = path.stat()
    changed = b"x" * stamp.st_size
    if mutation == "hardlink":
        alias = tmp_path / "alias"
        os.link(path, alias)
        alias.write_bytes(changed)
    elif mutation == "mmap":
        with path.open("r+b") as stream, mmap.mmap(stream.fileno(), 0) as mapped:
            mapped[:] = changed
            mapped.flush()
    else:
        path.write_bytes(changed)
    os.utime(path, ns=(stamp.st_atime_ns, stamp.st_mtime_ns))
    with pytest.raises(ValueError, match="digest differs"):
        resolve(tmp_path, config, contents={} if capture else None)


@pytest.mark.parametrize("kind", ["file_link", "directory_escape", "directory_file"])
def test_cached_parent_paths_never_survive_a_validation_pass(tmp_path, kind):
    root = tmp_path / "source"
    root.mkdir()
    config = source(root, count=1)
    resolve(root, config)
    original = root / "assets/0.stl"
    outside = tmp_path / "outside"
    outside.mkdir()
    (outside / "0.stl").write_bytes(original.read_bytes())
    original.unlink()
    if kind == "file_link":
        original.symlink_to(outside / "0.stl")
    else:
        (root / "assets").rmdir()
        if kind == "directory_escape":
            (root / "assets").symlink_to(outside, target_is_directory=True)
        else:
            (root / "assets").write_bytes(b"not a directory")
    with pytest.raises((ValueError, OSError)):
        resolve(root, config)


def test_confined_directory_alias_remains_supported(tmp_path):
    config = source(tmp_path, count=1)
    (tmp_path / "assets").rename(tmp_path / "original")
    (tmp_path / "assets").symlink_to(tmp_path / "original", target_is_directory=True)
    contents = {}
    resolve(tmp_path, config, contents=contents)
    assert contents["assets/0.stl"] == b"mesh bytes 0"


def test_reused_xml_references_still_require_current_manifest_and_relative_base(
    tmp_path, monkeypatch
):
    from waddle_sdk.simulators import source_references

    monkeypatch.setenv("WADDLE_SIMULATION_MODEL_CACHE", str(tmp_path / "cache"))
    monkeypatch.setattr(
        source_references, "cache", source_references.SourceReferences()
    )
    config = source(tmp_path, count=1)
    files = config["native_scene"]["files"]

    def publish(name, data):
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        files[name] = hashlib.sha256(data).hexdigest()

    child = b'<mujoco><compiler meshdir="assets"/><asset><mesh file="0.stl"/></asset></mujoco>'
    publish("world.xml", b'<mujoco><include file="nested/part.xml"/></mujoco>')
    publish("nested/part.xml", child)
    publish("nested/assets/0.stl", b"nested mesh")
    resolve(tmp_path, config)
    resolve(tmp_path, config, contents={})
    files.pop("nested/assets/0.stl")
    with pytest.raises(ValueError, match="unmanifested asset: nested/assets/0.stl"):
        resolve(tmp_path, config)
    publish("nested/assets/0.stl", b"nested mesh")
    publish("world.xml", b'<mujoco><include file="moved/part.xml"/></mujoco>')
    files.pop("nested/part.xml")
    publish("moved/part.xml", child)
    with pytest.raises(ValueError, match="unmanifested asset: moved/assets/0.stl"):
        resolve(tmp_path, config)
    publish("moved/assets/0.stl", b"moved mesh")
    resolve(tmp_path, config)
    publish("moved/part.xml", child.replace(b'file="0.stl"', b'file="../0.stl"'))
    with pytest.raises(ValueError, match="must remain portable"):
        resolve(tmp_path, config)


def test_reference_analysis_is_content_bound_and_bounded(monkeypatch):
    from waddle_sdk.simulators import source_references

    def xml(name):
        return f'<mujoco><include file="{name}"/></mujoco>'.encode()

    first, second, third = map(xml, ("first.xml", "other.xml", "third.xml"))
    cache = source_references.SourceReferences(max_bytes=len(first) * 2)
    analyze = source_references.references
    analyzed = []

    def tracked(data):
        analyzed.append(data)
        return analyze(data)

    monkeypatch.setattr(source_references, "references", tracked)

    def read(data):
        return cache.read(data, hashlib.sha256(data).hexdigest())

    assert read(first) == read(first) == ("first.xml",)
    assert analyzed == [first]
    assert read(second) == ("other.xml",)
    assert read(third) == ("third.xml",)
    assert read(first) == ("first.xml",)
    assert analyzed == [first, second, third, first]
    large = b"<mujoco>" + b" " * cache.max_bytes + b"</mujoco>"
    assert read(large) == ()
    assert not cache.entries
    read(first)
    monkeypatch.setattr(cache, "pid", -1)
    read(first)
    assert analyzed[-2:] == [first, first]
    with pytest.raises(ValueError, match="must remain portable"):
        read(xml("../escape.xml"))
    assert not cache.entries
