"""Reviewed source identity survives honest packaging and refuses source drift."""

import base64
import hashlib
import json
from importlib import metadata

import pytest
from waddle_sdk.robots import _i2rt_runtime as runtime
from waddle_sdk.robots import yam, yam_model
from waddle_sdk.robots.models import ModelSourceError


@pytest.fixture
def distribution(tmp_path, monkeypatch):
    content = {
        "i2rt/motor.py": b"position = 1\n",
        "i2rt/model.stl": b"reviewed model bytes",
        "i2rt/_transport_provenance.json": b'{"base": "reviewed"}',
        "i2rt-test.dist-info/METADATA": b"reviewed package metadata",
        "i2rt-test.dist-info/licenses/LICENSE": b"reviewed license",
    }
    hashes = {name: hashlib.sha256(data).hexdigest() for name, data in content.items()}
    expected = {
        "package_files": {n: h for n, h in hashes.items() if n.startswith("i2rt/")},
        "metadata": {n: h for n, h in hashes.items() if not n.startswith("i2rt/")},
        "version": runtime.TRANSPORT_VERSION,
        "upstream_repository": yam.I2RT_REPO,
        "upstream_commit": yam.I2RT_PIN,
        "transport_commit": "reviewed-change",
        "transport_patch_sha256": "reviewed-patch",
        "qualification": "physical acceptance pending",
    }

    class Distribution:
        version = runtime.TRANSPORT_VERSION

        def __init__(self):
            self.files = []
            self.origin = {"url": "file:///honest-derived-runtime.whl"}

        def locate_file(self, entry):
            return tmp_path / str(entry)

        def read_text(self, name):
            assert name == "direct_url.json"
            return json.dumps(self.origin)

        def write(self, name, data, *, record=True):
            path = self.locate_file(name)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
            if record:
                self.files = [entry for entry in self.files if str(entry) != name]
                entry = metadata.PackagePath(name)
                digest = (
                    base64.urlsafe_b64encode(hashlib.sha256(data).digest())
                    .decode()
                    .rstrip("=")
                )
                entry.hash = metadata.FileHash("sha256=" + digest)
                self.files.append(entry)

    selected = Distribution()
    for name, data in content.items():
        selected.write(name, data)
    monkeypatch.setattr(runtime, "reviewed_manifest", lambda: expected)
    monkeypatch.setattr(yam_model.metadata, "distribution", lambda _name: selected)
    return selected


def test_honest_derived_origin_retains_reviewed_source_identity(distribution):
    identity = runtime.verify_transport_runtime(distribution)
    assert identity["upstream_commit"] == yam.I2RT_PIN
    assert identity["qualification"] == "physical acceptance pending"
    read, provenance, license_bytes = yam_model._vendor()
    assert read("i2rt/model.stl") == b"reviewed model bytes"
    assert license_bytes == b"reviewed license"
    assert "i2rt/model.stl" in provenance


@pytest.mark.parametrize(
    "file",
    [
        "i2rt/motor.py",
        "i2rt/model.stl",
        "i2rt/_transport_provenance.json",
        "i2rt-test.dist-info/METADATA",
        "i2rt-test.dist-info/licenses/LICENSE",
    ],
)
@pytest.mark.parametrize("rewritten_record", [False, True])
def test_changed_source_or_license_is_refused_even_with_rewritten_record(
    distribution,
    file,
    rewritten_record,
):
    distribution.write(file, b"unreviewed replacement", record=rewritten_record)
    with pytest.raises(runtime.I2rtRuntimeError) as caught:
        runtime.verify_transport_runtime(distribution)
    assert caught.value.code == "hardware.vendor_runtime"
    assert caught.value.context["file"] == file
    with pytest.raises(ModelSourceError, match="derived runtime verification failed"):
        yam_model._vendor()


@pytest.mark.parametrize("recorded", [False, True])
def test_unreviewed_extra_vendor_code_cannot_join_the_runtime(distribution, recorded):
    distribution.write("i2rt/extra.py", b"unreviewed code", record=recorded)
    with pytest.raises(runtime.I2rtRuntimeError, match="inventory changed") as caught:
        runtime.verify_transport_runtime(distribution)
    assert caught.value.context["extra_files"] == ["i2rt/extra.py"]


def test_missing_model_bytes_and_missing_record_are_distinct_refusals(distribution):
    distribution.locate_file("i2rt/model.stl").unlink()
    with pytest.raises(runtime.I2rtRuntimeError, match="inventory changed") as caught:
        runtime.verify_transport_runtime(distribution)
    assert caught.value.context["missing_files"] == ["i2rt/model.stl"]
    distribution.write("i2rt/model.stl", b"reviewed model bytes")
    next(
        entry for entry in distribution.files if str(entry) == "i2rt/model.stl"
    ).hash = None
    with pytest.raises(runtime.I2rtRuntimeError, match="package record"):
        runtime.verify_transport_runtime(distribution)


def test_derived_code_cannot_bypass_review_by_claiming_pristine_origin(distribution):
    distribution.origin = {
        "url": yam.I2RT_REPO,
        "vcs_info": {"commit_id": yam.I2RT_PIN},
    }
    distribution.write("i2rt/motor.py", b"changed code", record=True)
    with pytest.raises(ModelSourceError, match="reviewed source"):
        yam_model._vendor()


def test_unreviewed_version_is_not_accepted_as_the_candidate(distribution):
    distribution.version = "1.1.2+another-change"
    with pytest.raises(runtime.I2rtRuntimeError, match="unreviewed version"):
        runtime.verify_transport_runtime(distribution)
    with pytest.raises(ModelSourceError, match="exact SDK I2RT Git pin"):
        yam_model._vendor()


def test_actual_selected_candidate_sources_are_nonopening_and_unchanged(monkeypatch):
    try:
        selected = metadata.distribution("i2rt")
    except metadata.PackageNotFoundError:
        pytest.skip("optional reviewed I2RT candidate not installed")
    if selected.version != runtime.TRANSPORT_VERSION:
        pytest.skip("optional reviewed I2RT candidate not selected")
    identity = runtime.verify_transport_runtime(selected)

    def forbidden(*args, **kwargs):
        pytest.fail("source verification must not open a driver")

    monkeypatch.setattr(yam, "LiveDriver", forbidden)
    read, hashes, _license = yam_model._vendor()
    assert identity["upstream_commit"] == yam.I2RT_PIN
    manifest = runtime.reviewed_manifest()
    for name in manifest["package_files"]:
        if name.startswith("i2rt/robot_models/"):
            read(name)
            assert hashes[name] == manifest["package_files"][name]
