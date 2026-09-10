"""Source geometry acceptance without constructing any hardware driver."""

import importlib.metadata
import json
import sys

import numpy as np
import pytest
from waddle_sdk import Site
from waddle_sdk.robots import yam
from waddle_sdk.robots import yam_model as sources
from waddle_sdk.robots.yam_model import ModelSourceError


def test_modified_pinned_asset_rejected_before_publish(tmp_path, monkeypatch):
    pytest.importorskip("i2rt")
    original = importlib.metadata.distribution("i2rt")
    modified = tmp_path / "modified.xml"
    modified.write_text("unverified geometry")

    class ChangedDistribution:
        files = original.files
        read_text = original.read_text

        def locate_file(self, entry):
            return (
                modified
                if str(entry).endswith("linear_4310.xml")
                else original.locate_file(entry)
            )

    monkeypatch.setattr(
        sources.metadata, "distribution", lambda _: ChangedDistribution()
    )
    with pytest.raises(ModelSourceError, match="package record"):
        yam.model_sources()


def test_missing_or_wrong_vendor_revision_never_generates_placeholder(
    tmp_path, monkeypatch
):
    def missing(_):
        raise importlib.metadata.PackageNotFoundError("i2rt")

    monkeypatch.setattr(sources.metadata, "distribution", missing)
    with pytest.raises(ModelSourceError, match="pinned I2RT"):
        yam.model_sources()

    class WrongRevision:
        def read_text(self, _):
            return json.dumps(
                {"url": yam.I2RT_REPO, "vcs_info": {"commit_id": "0" * 40}}
            )

    monkeypatch.setattr(sources.metadata, "distribution", lambda _: WrongRevision())
    with pytest.raises(ModelSourceError, match="exact SDK I2RT Git pin"):
        yam.model_sources()


def test_complete_sources_are_immutable_and_do_not_open_hardware(monkeypatch):
    pytest.importorskip("i2rt")

    def forbidden(*args, **kwargs):
        pytest.fail("model inspection must not construct drivers or open a site")

    monkeypatch.setattr(yam, "arm", forbidden)
    monkeypatch.setattr(Site, "open", forbidden)
    before = {name for name in sys.modules if name.startswith("i2rt.")}
    bundle = yam.model_sources()
    assert {name for name in sys.modules if name.startswith("i2rt.")} == before
    assert bundle.arm_urdf == yam.urdf_text().encode()
    assert len(bundle.arm_assets) == 6 and len(bundle.hand_assets) == 3
    assert bundle.joint_names == yam.ARM_JOINT_NAMES
    assert bundle.base_link == yam.URDF_BASE_LINK
    assert bundle.vendor_commit == yam.I2RT_PIN
    assert len(bundle.vendor_sources_sha256) == 11
    assert "MIT License" in bundle.license_bytes.decode()
    assert np.asarray(bundle.hand_attachment).shape == (4, 4)
    for displacement in bundle.finger_displacement_mesh_m.values():
        assert np.linalg.norm(displacement) == pytest.approx(
            yam.GRIPPER_MAX_OPENING_M / 2
        )
    with pytest.raises(TypeError):
        bundle.arm_assets["new"] = b"bad"


@pytest.mark.parametrize("change", ["coupling", "range", "asset", "tcp"])
def test_changed_source_relationship_is_refused(monkeypatch, change):
    pytest.importorskip("i2rt")
    read, provenance, license_bytes = sources._vendor()

    def changed(name):
        data = read(name)
        if name.endswith("linear_4310.xml"):
            from xml.etree import ElementTree as ET

            root = ET.fromstring(data)
            if change == "coupling":
                root.find("equality/joint").set("polycoef", "0 2 0 0 0")
            elif change == "range":
                root.find("worldbody/body/body/joint").set("range", "0 0.2")
            elif change == "asset":
                root.find("asset/mesh").set("file", "../outside.stl")
            else:
                root.find("worldbody/body/site[@name='grasp_site']").set(
                    "name", "other"
                )
            return ET.tostring(root)
        return data

    monkeypatch.setattr(
        sources, "_vendor", lambda: (changed, provenance, license_bytes)
    )
    with pytest.raises(ModelSourceError):
        yam.model_sources()
