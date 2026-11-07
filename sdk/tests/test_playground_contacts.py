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


def test_incremental_contacts_preserve_bytes_order_and_only_build_new_pairs():
    cache = ContactCache()
    base = ET.fromstring('<contact><exclude body1="a" body2="b"/></contact>')
    calls = []

    def build(node, fingers, props):
        calls.append(tuple(props))
        _contact_pairs(node, fingers, props)

    fingers, props = ["f1", "f2", "f3"], ["p1", "p2"]
    first = cache.prepare(base, fingers, props, build, _incremental=True)
    extended = cache.prepare(
        base, fingers, [*props, "p3", "p4"], build, _incremental=True
    )
    expected = ET.fromstring(ET.tostring(base))
    _contact_pairs(expected, fingers, [*props, "p3", "p4"])
    assert ET.tostring(extended.node) == extended.xml == ET.tostring(expected)
    assert calls == [("p1", "p2"), ("p3", "p4")]
    assert extended.comparison.previous is first.comparison.token
    assert len(extended.comparison.added) == 6
    assert extended.comparison.entries[1] is first.comparison.entries[1]
    # Current source trees remain detached from shared comparison records.
    extended.node[1].set("geom1", "changed")
    again = cache.prepare(base, fingers, [*props, "p3", "p4"], build, _incremental=True)
    assert again.xml == ET.tostring(again.node) == ET.tostring(expected)
    assert again.comparison is extended.comparison
    assert first.comparison.entries[1][1].get("geom1") == "f1"


@pytest.mark.parametrize(
    "base,fingers,before,after",
    [
        ('<contact><pair geom1="f" geom2="p"/></contact>', ["f"], ["p"], ["p", "q"]),
        ("<contact/>", ["f", "f"], ["p"], ["p", "q"]),
        ("<contact/>", ["f"], ["p"], ["p", "f"]),
        ("<contact/>", ["f"], ["p"], ["p", "p"]),
        ("<contact/>", ["f"], ["p", "q"], ["q", "p", "r"]),
        ("<contact>\n</contact>", ["f"], ["p"], ["p", "q"]),
    ],
)
def test_unrecognized_contact_layout_keeps_exact_full_builder(
    base, fingers, before, after
):
    cache = ContactCache()
    base = ET.fromstring(base)
    cache.prepare(base, fingers, before, _contact_pairs, _incremental=True)
    actual = cache.prepare(base, fingers, after, _contact_pairs, _incremental=True)
    expected = ET.fromstring(ET.tostring(base))
    _contact_pairs(expected, fingers, after)
    assert actual.xml == ET.tostring(actual.node) == ET.tostring(expected)
    assert actual.comparison.previous is None


def test_incremental_contact_eviction_does_not_retain_a_preparation_chain():
    import gc
    import weakref

    cache = ContactCache(max_entries=1)
    base = ET.Element("contact")
    first = cache.prepare(base, ["f"], ["a"], _contact_pairs, _incremental=True)
    old = weakref.ref(first.comparison)
    second = cache.prepare(base, ["f"], ["a", "b"], _contact_pairs, _incremental=True)
    del first
    gc.collect()
    assert old() is None
    assert len(cache.entries) == 1
    assert second.comparison.previous is not None
    # Both the complete fragment and its reusable serialized pieces are charged.
    cache.max_bytes = len(ET.tostring(base)) + len(second.xml) + len("fab")
    cache.prepare(base, ["f"], ["a", "b"], _contact_pairs, _incremental=True)
    assert not cache.entries


def test_prepared_contact_comparison_matches_bytes_through_add_move_and_remove():
    from waddle_sdk.simulators.authored_preparation import _contact_delta, _key, parse

    cache = ContactCache()
    before = ordinary_before = None
    for props in ([], ["a"], ["a", "b"], ["a", "b"], ["a"]):
        tree = ET.fromstring(
            '<mujoco><compiler angle="radian"/><asset/><worldbody/>'
            '<contact/><keyframe><key name="initial"/></keyframe></mujoco>'
        )
        fragment = cache.prepare(
            tree.find("contact"), ["f"], props, _contact_pairs, _incremental=True
        )
        tree[list(tree).index(tree.find("contact"))] = fragment.node
        data = scene_xml(tree, fragment)
        actual = parse(data, _tree=tree, _contacts=fragment)
        ordinary = parse(data)
        assert actual.static == ordinary.static
        assert actual.defaults == ordinary.defaults
        assert {name: _key(node) for name, node in actual.parts.items()} == {
            name: _key(node) for name, node in ordinary.parts.items()
        }
        if before is not None:
            fast = _contact_delta(before, actual)
            slow = _contact_delta(ordinary_before, ordinary)
            assert [[_key(node) for node in nodes] for nodes in fast] == [
                [_key(node) for node in nodes] for nodes in slow
            ]
        tree.find("keyframe/key").set("qpos", "changed after parsing")
        ET.SubElement(fragment.node, "pair", geom1="caller", geom2="mutation")
        assert _key(actual.parts["contact"]) == _key(ordinary.parts["contact"])
        before, ordinary_before = actual, ordinary


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

    def ordinary():
        # Compare with the author's existing UTF-8 path, not ElementTree's
        # default ASCII serializer, which rewrites non-ASCII text as entities.
        return ET.tostring(root, encoding="unicode").encode()

    before = ordinary()
    assert scene_xml(root, fragment) == before
    assert ordinary() == before
    root.find("keyframe/key").set("qpos", "1 2 3")
    root.find("asset/mesh").set("file", "assets/new.stl")
    assert scene_xml(root, fragment) == ordinary()
    # A different contact element has no authority to reuse this fragment.
    root.remove(fragment.node)
    ET.SubElement(root, "contact")
    assert scene_xml(root, fragment) == ordinary()
