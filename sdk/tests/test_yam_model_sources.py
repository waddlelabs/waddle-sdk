"""Source geometry acceptance without constructing any hardware driver."""

import importlib.metadata
import json
import sys
from xml.etree import ElementTree as ET

import numpy as np
import pytest
from waddle_sdk import Site
from waddle_sdk.robots import yam
from waddle_sdk.robots import yam_model as sources
from waddle_sdk.robots.models import ModelSourceError


def sources_for_arm(gripper_metadata=None):
    return yam.model_sources(
        factory="arm",
        part_name="arm",
        part={
            "base_frame": "declared-base",
            "gripper": {
                "joint": yam.GRIPPER_JOINT_NAME,
                "closed_m": 0,
                "open_m": yam.GRIPPER_MAX_OPENING_M,
                "closed_action": 0,
                "open_action": 1,
                **(gripper_metadata or {}),
            },
        },
    )


def test_optional_grasp_metadata_preserves_standard_physical_sources():
    pytest.importorskip("i2rt")
    baseline = sources_for_arm()
    enriched = sources_for_arm(
        {
            "closing_axis_tcp": [0.0, 1.0, 0.0],
            "pinch_offset_tcp_m": [0.044, 0.0, -0.0049],
            "pointing_down_wxyz": [0.0, 0.0, 1.0, 0.0],
        }
    )
    assert enriched.model == baseline.model
    assert enriched.assets == baseline.assets
    assert enriched.provenance == baseline.provenance


@pytest.mark.parametrize(
    "changed",
    [
        {"joint": "other"},
        {"closed_m": 0.01},
        {"open_m": 0.2},
        {"closed_action": 1},
        {"open_action": 2},
    ],
)
def test_nonstandard_physical_gripper_mapping_remains_rejected(changed):
    with pytest.raises(ModelSourceError, match="physical SDK gripper mapping"):
        sources_for_arm(changed)


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
        sources_for_arm()


def test_missing_or_wrong_vendor_revision_never_generates_placeholder(
    tmp_path, monkeypatch
):
    def missing(_):
        raise importlib.metadata.PackageNotFoundError("i2rt")

    monkeypatch.setattr(sources.metadata, "distribution", missing)
    with pytest.raises(ModelSourceError, match="pinned I2RT"):
        sources_for_arm()

    class WrongRevision:
        def read_text(self, _):
            return json.dumps(
                {"url": yam.I2RT_REPO, "vcs_info": {"commit_id": "0" * 40}}
            )

    monkeypatch.setattr(sources.metadata, "distribution", lambda _: WrongRevision())
    with pytest.raises(ModelSourceError, match="exact SDK I2RT Git pin"):
        sources_for_arm()


def test_complete_sources_are_immutable_and_do_not_open_hardware(monkeypatch):
    pytest.importorskip("i2rt")

    def forbidden(*args, **kwargs):
        pytest.fail("model inspection must not construct drivers or open a site")

    monkeypatch.setattr(yam, "arm", forbidden)
    monkeypatch.setattr(Site, "open", forbidden)
    before = {name for name in sys.modules if name.startswith("i2rt.")}
    bundle = sources_for_arm()
    assert {name for name in sys.modules if name.startswith("i2rt.")} == before
    model = ET.fromstring(bundle.model)
    assert bundle.format == "mjcf" and len(bundle.assets) == 9
    assert bundle.joint_names == yam.ARM_JOINT_NAMES
    assert bundle.base_body == yam.URDF_BASE_LINK
    assert bundle.provenance["vendor_commit"] == yam.I2RT_PIN
    assert len(bundle.provenance["vendor_sources_sha256"]) == 11
    assert "MIT License" in bundle.licenses["LICENSE.i2rt"].decode()
    assert np.asarray(bundle.provenance["T_sdk_link6_vendor_hand"]).shape == (4, 4)
    slides = model.findall(".//joint[@type='slide']")
    assert len(slides) == 2
    assert model.find("equality/joint") is not None
    for joint in slides:
        assert [float(v) for v in joint.get("range").split()] == pytest.approx(
            [0, yam.GRIPPER_MAX_OPENING_M / 2]
        )
    with pytest.raises(TypeError):
        bundle.assets["new"] = b"bad"


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
        sources_for_arm()
