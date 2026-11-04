"""Prepared contacts preserve exact pair order and portable XML serialization."""

import os
import xml.etree.ElementTree as ET

import pytest
from waddle_sdk.simulators.playground import _contact_pairs
from waddle_sdk.simulators.playground_contacts import ContactCache, scene_xml


def test_contact_reuse_tracks_all_inputs_and_detaches_trees():
    cache = ContactCache()
    contact = ET.fromstring(
        '<contact><exclude body1="a" body2="b"/>'
        '<pair geom1="b" geom2="a" friction=".3 .3 0 0 0"/></contact>'
    )
    original = ET.tostring(contact)
    calls = []

    def build(node, fingers, props):
        calls.append(True)
        _contact_pairs(node, fingers, props)

    fingers, props = ["a", "b", "a"], ["a", "b", "c", "c"]
    first = cache.prepare(contact, fingers, props, build)
    fresh = ET.fromstring(original)
    _contact_pairs(fresh, fingers, props)
    assert first.xml == ET.tostring(fresh)
    assert len(first.node.findall("pair")) == 5
    first.node[0].set("body1", "caller_change")
    first.node.remove(first.node[-1])
    again = cache.prepare(contact, fingers, props, build)
    assert len(calls) == 1
    assert ET.tostring(again.node) == again.xml == ET.tostring(fresh)
    assert ET.tostring(contact) == original
    for changed_fingers, changed_props in [
        (fingers[1:] + fingers[:1], props[:-1]),
        (fingers, ["z"]),
    ]:
        actual = cache.prepare(contact, changed_fingers, changed_props, build)
        expected = ET.fromstring(original)
        _contact_pairs(expected, changed_fingers, changed_props)
        assert actual.xml == ET.tostring(expected)
    contact.find("pair").set("friction", ".7 .7 0 0 0")
    changed = cache.prepare(contact, fingers, props, build)
    assert changed.xml != again.xml
    assert len(calls) == 4


def test_contact_cache_bounds_fork_and_builder_failure(monkeypatch):
    cache = ContactCache(max_bytes=1024, max_entries=2)
    contact = ET.Element("contact")
    for prop in ("a", "b", "c"):
        cache.prepare(contact, ["finger"], [prop], _contact_pairs)
    assert [key[2] for key in cache.entries] == [("b",), ("c",)]
    cache.max_bytes = 1
    cache.prepare(contact, ["finger"], ["d"], _contact_pairs)
    assert not cache.entries
    cache.max_bytes = 1024
    cache.prepare(contact, ["finger"], ["before"], _contact_pairs)
    pid = os.getpid()
    monkeypatch.setattr(os, "getpid", lambda: pid + 1)
    cache.prepare(contact, ["finger"], ["after"], _contact_pairs)
    assert [key[2] for key in cache.entries] == [("after",)]

    def failed(*args):
        raise ValueError("builder failed")

    with pytest.raises(ValueError, match="builder failed"):
        cache.prepare(contact, ["finger"], ["invalid"], failed)
    assert not cache.entries


@pytest.mark.parametrize(
    "extra", ["", "<named xmlns='urn:test'/>", "<_waddle_contact_fragment_ />"]
)
def test_fragment_serialization_preserves_current_keys_assets_text_and_namespaces(
    extra,
):
    root = ET.fromstring(
        '<mujoco model="café &amp; &quot;quoted&quot;">\n'
        '<asset><mesh name="m" file="old.stl"/></asset>'
        '<contact><exclude body1="a" body2="b"/></contact>\n'
        '<keyframe><key name="initial"/></keyframe>' + extra + "</mujoco>"
    )
    fragment = ContactCache().prepare(
        root.find("contact"), ["finger"], ["prop"], _contact_pairs
    )
    root[list(root).index(root.find("contact"))] = fragment.node
    before = ET.tostring(root)
    assert scene_xml(root, fragment) == before
    assert ET.tostring(root) == before
    root.find("keyframe/key").set("qpos", "1 2 3")
    root.find("asset/mesh").set("file", "assets/new.stl")
    assert scene_xml(root, fragment) == ET.tostring(root)
    # A different contact element has no authority to reuse this fragment.
    root.remove(fragment.node)
    ET.SubElement(root, "contact")
    assert scene_xml(root, fragment) == ET.tostring(root)
