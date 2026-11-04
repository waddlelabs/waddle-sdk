"""Final placements retain fresh-authoring/reset semantics without geometry work."""

import xml.etree.ElementTree as ET
from copy import deepcopy

import numpy as np
import pytest
from test_playground import scene
from test_prepared_placement import same_model
from waddle_sdk.simulators.playground import prepare_scene
from waddle_sdk.simulators.playground_placement import (
    PlacementTemplate,
    position_changes,
)
from waddle_sdk.simulators.scene import make_site


@pytest.fixture
def authored(tmp_path, monkeypatch):
    pytest.importorskip("mujoco")
    monkeypatch.setenv("WADDLE_SIMULATION_MODEL_CACHE", str(tmp_path / "cache"))
    _, config = make_site("placement", backend="mujoco", robot="yam", arms=1)
    document = scene()
    for obj in document["objects"]:
        obj["rotation"] = [17, 0, -23]
    prepared = prepare_scene(config, document, tmp_path / "first", _prefer_handoff=True)
    return config, document, prepared


def test_final_positions_match_complete_fresh_models_and_reset_bytes(
    authored, tmp_path
):
    config, before, original = authored
    template = PlacementTemplate.capture(original, before)
    assert template is not None
    pristine_xml = original.contents["world.xml"]
    pristine_tree = ET.tostring(original._tree)
    original.model.geom_rgba[:] = 0  # Caller-owned model cannot poison the template.
    try:
        for index in range(len(before["objects"])):
            after = deepcopy(before)
            after["objects"][index]["position"] = [0.531234567891, -0.09, 0.17]
            actual = template.prepare(after)
            expected = prepare_scene(
                config, after, tmp_path / str(index), _prefer_handoff=True
            )
            assert actual is not None
            assert actual.config == expected.config
            assert actual.contents == expected.contents
            same_model(actual.model, expected.model)
            actual.model.body_pos[:] = 7
        assert original.contents["world.xml"] == pristine_xml
        assert ET.tostring(original._tree) == pristine_tree
        np.testing.assert_array_equal(original.model.geom_rgba, 0)
    finally:
        template.close()
    assert template.source is None and template.prepare(before) is None


@pytest.mark.parametrize(
    "field",
    ["size", "rotation", "color", "id", "fixture", "seed", "name", "membership"],
)
def test_other_authoring_changes_fall_back(authored, field):
    _, before, prepared = authored
    template = PlacementTemplate.capture(prepared, before)
    after = deepcopy(before)
    if field in {"size", "rotation", "color"}:
        after["objects"][0][field][0] += 0.01
    elif field == "id":
        after["objects"][0][field] = "renamed"
    elif field == "membership":
        after["objects"].pop()
    else:
        after[field] = {"fixture": "handover", "seed": 1, "name": "Other"}[field]
    try:
        assert position_changes(before, after) is None
        assert template.prepare(after) is None
    finally:
        template.close()


@pytest.mark.parametrize("dependency", ["sensor", "child", "statistics", "budget"])
def test_unhandled_native_dependencies_and_memory_limits_decline(authored, dependency):
    _, before, prepared = authored
    name = before["objects"][0]["id"]
    if dependency == "sensor":
        sensor = ET.SubElement(prepared._tree, "sensor")
        ET.SubElement(sensor, "framepos", objtype="body", objname="prop_" + name)
    elif dependency == "child":
        body = prepared._tree.find(f"worldbody/body[@name='prop_{name}']")
        ET.SubElement(body, "site", name="prop_probe_site")
    elif dependency == "statistics":
        ET.SubElement(prepared._tree, "statistic", extent="8")
    template = PlacementTemplate.capture(
        prepared, before, max_bytes=1 if dependency == "budget" else 128 * 1024 * 1024
    )
    if dependency in {"statistics", "budget"}:
        assert template is None
    else:
        after = deepcopy(before)
        after["objects"][0]["position"] = [0.5, 0, 0.2]
        try:
            assert template.prepare(after) is None
        finally:
            template.close()
