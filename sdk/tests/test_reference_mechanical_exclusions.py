"""Reference exports preserve declared internal mechanical contacts."""

import pytest
from waddle_sdk.simulators import reference_model_sources


@pytest.mark.parametrize("robot", ["yam", "so101", "xarm7"])
def test_reference_sources_preserve_declared_mechanical_exclusions(robot):
    from xml.etree import ElementTree as ET

    from waddle_sdk.simulators.description import description

    mj = pytest.importorskip("mujoco")
    bundle = reference_model_sources(robot, part_name="arm")
    root = ET.fromstring(bundle.model)
    declared = {
        tuple(sorted((e.get("body1"), e.get("body2"))))
        for e in root.findall("contact/exclude")
    }
    expected = {tuple(sorted(pair)) for pair in description(robot).exclusions}
    assert declared == expected
    model = mj.MjModel.from_xml_string(bundle.model.decode(), dict(bundle.assets))
    assert model.nexclude == len(expected)
