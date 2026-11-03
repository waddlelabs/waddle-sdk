"""Native file checks share directory work, never stale source contents."""

import hashlib
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
def test_one_directory_resolution_per_pass_keeps_fresh_hashes(
    tmp_path, monkeypatch, capture
):
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
    path.write_bytes(b"changed data")
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
