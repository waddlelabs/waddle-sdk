"""Shipped shapes retain exact authoring and never carry another object's state."""

import xml.etree.ElementTree as ET
from copy import deepcopy

import pytest

from waddle_sdk.simulators import playground, playground_catalog


@pytest.fixture(autouse=True)
def clear_catalog():
    playground_catalog._load.cache_clear()
    yield
    playground_catalog._load.cache_clear()


@pytest.mark.parametrize("kind", list(playground.CATALOG))
@pytest.mark.parametrize("easy", [False, True])
def test_shipped_geometry_matches_fresh_authoring_for_private_instances(kind, easy):
    for name, position in [
        ("first", [0.31, -0.12, 0.08]),
        ("second", [0.42, 0.16, 0.11]),
    ]:
        obj = dict(
            id=name,
            kind=kind,
            size=playground.CATALOG[kind]["size"],
            position=position,
            rotation=[12, -34, 56],
            color=[0.23, 0.54, 0.67],
        )
        actual, assets = ET.Element("worldbody"), ET.Element("asset")
        expected, expected_assets = ET.Element("worldbody"), ET.Element("asset")
        names = playground_catalog.instantiate(actual, assets, obj, easy)
        assert names is not None
        assert names == playground._object_source(expected, expected_assets, obj, easy)
        assert ET.tostring(actual) == ET.tostring(expected)
        assert ET.tostring(assets) == ET.tostring(expected_assets)
        actual.find("body").set("pos", "99 99 99")
        assets.clear()


def test_bundle_is_reproducible_and_source_current():
    assert playground_catalog.build() == playground_catalog._BUNDLE.read_bytes()
    assert len(playground_catalog._load()) == 2 * len(playground.CATALOG)


def test_default_addition_never_generates_catalog_meshes(monkeypatch):
    def unexpected(*_args):
        raise AssertionError("default geometry was generated during object placement")

    monkeypatch.setattr(playground, "_object_source", unexpected)
    for kind, item in playground.CATALOG.items():
        obj = dict(
            id="placed",
            kind=kind,
            size=item["size"],
            position=[0.3, 0, 0.1],
            rotation=[0, 0, 0],
            color=[0.2, 0.3, 0.4],
        )
        playground._object(ET.Element("worldbody"), ET.Element("asset"), obj, True)


def test_custom_dimensions_keep_the_ordinary_compiler():
    obj = dict(
        id="custom",
        kind="bowl",
        size=[0.16, 0.16, 0.07],
        position=[0.3, 0, 0.1],
        rotation=[0, 0, 0],
        color=[0.2, 0.3, 0.4],
    )
    actual, assets = ET.Element("worldbody"), ET.Element("asset")
    expected, expected_assets = ET.Element("worldbody"), ET.Element("asset")
    assert playground_catalog.instantiate(actual, assets, obj, True) is None
    assert playground._object(actual, assets, obj, True) == playground._object_source(
        expected, expected_assets, obj, True
    )
    assert ET.tostring(actual) == ET.tostring(expected)
    assert ET.tostring(assets) == ET.tostring(expected_assets)


@pytest.mark.parametrize("failure", ["missing", "corrupt", "stale"])
def test_unusable_bundle_falls_back_without_changing_geometry(
    tmp_path, monkeypatch, failure
):
    if failure == "stale":
        monkeypatch.setattr(playground_catalog, "_fingerprint", lambda: {})
    else:
        path = tmp_path / "bundle.gz"
        if failure == "corrupt":
            path.write_bytes(b"invalid")
        monkeypatch.setattr(playground_catalog, "_BUNDLE", path)
    obj = dict(
        id="fallback",
        kind="bowl",
        size=deepcopy(playground.CATALOG["bowl"]["size"]),
        position=[0.3, 0, 0.1],
        rotation=[0, 0, 0],
        color=[0.2, 0.3, 0.4],
    )
    actual, assets = ET.Element("worldbody"), ET.Element("asset")
    expected, expected_assets = ET.Element("worldbody"), ET.Element("asset")
    assert playground._object(actual, assets, obj, True) == playground._object_source(
        expected, expected_assets, obj, True
    )
    assert ET.tostring(actual) == ET.tostring(expected)
    assert ET.tostring(assets) == ET.tostring(expected_assets)
