"""Reviewed site creation is atomic and never constructs a hardware owner."""

import json

import pytest

pytest.importorskip("fcntl")

from waddle_sdk import Site, load_site
from waddle_sdk.workspace.initialization import build_site, make_site
from waddle_sdk.workspace.profiles import camera_defaults, part_defaults


def document():
    arm = part_defaults("yam", "arm")
    arm["connection"]["channel"] = "test-can-not-opened"
    return make_site(
        site_id="reviewed-rig",
        parts={"arm": arm},
        cameras={"scene": camera_defaults("simulation", "scene")},
        workspace_bounds={"min": [-0.5, -0.5, 0], "max": [0.5, 0.5, 0.8]},
    )


def test_site_creation_preserves_reviewed_hardware_without_opening(
    tmp_path, monkeypatch
):
    monkeypatch.setattr(Site, "open", lambda *a, **k: pytest.fail("hardware opened"))
    value = document()
    built = build_site(tmp_path / "rig", value)
    loaded = load_site(built["site"])
    assert loaded.id == "reviewed-rig"
    assert loaded.manifest["parts"]["arm"] == value["parts"]["arm"]
    assert "site.yaml" in built["files"]
    assert not (tmp_path / "rig" / "workspace.yaml").exists()
    assert not list(tmp_path.glob(".rig.init-*"))


def test_import_preserves_calibration_and_source_identity(tmp_path):
    source = tmp_path / "source"
    build_site(source, document())
    calibration = source / "calibration" / "camera.json"
    calibration.write_text(json.dumps({"source": "measured", "value": 0.125}))
    build_site(tmp_path / "copy", source_site=source)
    assert (
        tmp_path / "copy/calibration/camera.json"
    ).read_bytes() == calibration.read_bytes()
    changed = document()
    changed["metadata"]["id"] = "different-owner"
    with pytest.raises(ValueError, match="identity"):
        build_site(tmp_path / "bad", changed, source_site=source)
    assert not (tmp_path / "bad").exists()


def test_failed_or_racing_application_preparation_does_not_publish(tmp_path):
    destination = tmp_path / "rig"

    def fail(stage, site):
        (stage / "application.json").write_text("prepared")
        raise ValueError("application preparation failed")

    with pytest.raises(ValueError, match="application preparation failed"):
        build_site(destination, document(), configure=fail)
    assert not destination.exists()
    assert not list(tmp_path.glob(".rig.init-*"))

    def raced(stage, site):
        destination.mkdir()
        (destination / "keep").write_text("other writer")

    with pytest.raises(FileExistsError):
        build_site(destination, document(), configure=raced)
    assert (destination / "keep").read_text() == "other writer"
    assert not list(tmp_path.glob(".rig.init-*"))


def test_source_change_and_symlink_fail_before_publication(tmp_path):
    source = tmp_path / "source"
    build_site(source, document())

    def changed(stage, site):
        (source / "site.yaml").write_text("changed")

    with pytest.raises(ValueError, match="changed"):
        build_site(tmp_path / "changed", source_site=source, configure=changed)
    assert not (tmp_path / "changed").exists()
    link = tmp_path / "link"
    link.symlink_to(source, target_is_directory=True)
    with pytest.raises(ValueError):
        build_site(tmp_path / "linked", source_site=link)
    assert not (tmp_path / "linked").exists()


def test_simulation_site_carries_its_exact_native_model(tmp_path):
    value = make_site(
        site_id="simulation",
        parts={"arm": part_defaults("simulation", "arm")},
        cameras={},
        workspace_bounds={},
    )
    build_site(tmp_path / "sim", value)
    model = tmp_path / "sim" / value["parts"]["arm"]["connection"]["model"]
    assert model.exists() and model.with_name("actuators.xml").exists()
