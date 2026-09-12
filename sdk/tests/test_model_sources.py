"""Public source extension contract, independent of hardware brands and loaders."""

import sys
from types import ModuleType

import pytest
from waddle_sdk.robots.models import (
    ModelSourceError,
    ModelSources,
    model_sources_for_driver,
)


def bundle(**changes):
    return ModelSources(
        **{
            "format": "mjcf",
            "model": b"<mujoco/>",
            "assets": {},
            "joint_names": ("pivot",),
            "joint_units": ("rad",),
            "base_body": "mount",
            "tcp_site": "probe",
            "tcp_frame": "declared_probe",
            **changes,
        }
    )


def test_external_adapter_uses_the_same_contract_without_constructing_a_driver(
    monkeypatch,
):
    module = ModuleType("external_model_fixture")
    received = []

    def forbidden(**kwargs):
        pytest.fail("source resolution constructed a driver")

    module.arm = forbidden

    def model_sources(**kwargs):
        received.append(kwargs)
        with pytest.raises(TypeError):
            kwargs["part"]["options"]["variant"] = "changed"
        return bundle()

    module.model_sources = model_sources
    monkeypatch.setitem(sys.modules, module.__name__, module)
    result = model_sources_for_driver(
        module.__name__ + ":arm",
        part_name="measuring_arm",
        part={"options": {"variant": "short"}},
    )
    assert result.tcp_frame == "declared_probe"
    assert (
        received[0]["factory"] == "arm" and received[0]["part_name"] == "measuring_arm"
    )


def test_absent_extension_is_not_a_failure(monkeypatch):
    module = ModuleType("source_optional_fixture")
    monkeypatch.setitem(sys.modules, module.__name__, module)
    assert (
        model_sources_for_driver(module.__name__ + ":arm", part_name="arm", part={})
        is None
    )
    module.model_sources = lambda **kwargs: None
    assert (
        model_sources_for_driver(module.__name__ + ":arm", part_name="arm", part={})
        is None
    )


@pytest.mark.parametrize("kind", ["raises", "wrong_type", "not_callable"])
def test_selected_failure_is_explicit_and_does_not_expose_vendor_text(
    monkeypatch, kind
):
    module = ModuleType("source_failed_fixture")

    def failed(**kwargs):
        raise RuntimeError("credential=private-value")

    module.model_sources = (
        failed
        if kind == "raises"
        else ((lambda **kwargs: {}) if kind == "wrong_type" else 7)
    )
    monkeypatch.setitem(sys.modules, module.__name__, module)
    with pytest.raises(ModelSourceError) as error:
        model_sources_for_driver(module.__name__ + ":arm", part_name="arm", part={})
    assert error.value.code == (
        "source_provider_failed" if kind == "raises" else "source_provider_invalid"
    )
    assert "private-value" not in str(error.value)


def test_bundle_snapshots_metadata_and_assets():
    assets = {"links.xml": b"geometry"}
    evidence = {"source": {"revision": "1"}}
    result = bundle(
        assets=assets, provenance=evidence, licenses={"notices/LICENSE": b"license"}
    )
    assets["links.xml"] = b"modified"
    evidence["source"]["revision"] = "2"
    assert result.assets["links.xml"] == b"geometry"
    assert result.provenance["source"]["revision"] == "1"
    from dataclasses import replace

    assert (
        replace(result, tcp_frame="another_declared_tip").provenance
        == result.provenance
    )
    with pytest.raises(TypeError):
        result.provenance["source"]["revision"] = "3"


@pytest.mark.parametrize(
    "changes",
    [
        {"assets": {"../outside": b"x"}},
        {"assets": {"/absolute": b"x"}},
        {"assets": {"model.xml": b"x"}},
        {"assets": {"a\\b": b"x"}},
        {"assets": {"asset": "not bytes"}},
        {"provenance": {"bad": float("nan")}},
        {"joint_names": ("same", "same"), "joint_units": ("rad", "rad")},
        {"joint_units": ()},
        {"tcp_frame": ""},
    ],
)
def test_invalid_source_bundle_is_rejected(changes):
    with pytest.raises(ModelSourceError):
        bundle(**changes)
