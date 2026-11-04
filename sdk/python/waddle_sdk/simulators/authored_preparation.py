"""Extend a privately prepared authored scene after an exact source comparison."""

import math
import xml.etree.ElementTree as ET
from collections import Counter
from copy import copy, deepcopy
from dataclasses import dataclass


def _read(data, *, tree=None):
    root = ET.fromstring(data) if tree is None else deepcopy(tree)
    for node in root.iter():
        if node.text is not None and not node.text.strip():
            node.text = None
        node.tail = None
        # MuJoCo sometimes spells out this filename-inferred STL type after its
        # first import. Native replacement contracts normalize the same default.
        if (
            node.tag == "mesh"
            and node.get("content_type") == "model/stl"
            and node.get("file", "").endswith(".stl")
        ):
            del node.attrib["content_type"]
    return root


def _encoded(nodes):
    return [_key(node) for node in nodes]


def _key(node):
    # Structural tuples retain every attribute and meaningful text without
    # serializing thousands of unchanged contact pairs on every edit.
    return (
        node.tag,
        tuple(sorted(node.attrib.items())),
        node.text,
        tuple(_key(child) for child in node),
    )


def _changes(before, after, updates):
    if before.tag != after.tag or before.attrib.keys() != after.attrib.keys():
        return False
    allowed = {
        "body": {"pos": 3, "quat": 4},
        "geom": {"pos": 3, "quat": 4, "size": 3, "rgba": 4, "mass": 1, "density": 1},
    }
    for key, value in after.attrib.items():
        if before.get(key) == value:
            continue
        width = allowed.get(after.tag, {}).get(key)
        if width is None or not after.get("name"):
            return False
        numbers = [float(part) for part in value.split()]
        if (
            len(numbers) != len(before.get(key).split())
            or not 0 < len(numbers) <= width
        ):
            return False
        numbers += [0.0] * (width - len(numbers))
        updates.append((after.tag, after.get("name"), key, numbers))
    if before.text != after.text or len(before) != len(after):
        return False
    return all(_changes(a, b, updates) for a, b in zip(before, after, strict=True))


def _pair_defaults(root):
    """Recognize source-only pair defaults that can be expanded for attachment."""
    sections = root.findall("default")
    if not sections:
        return {}
    if len(sections) != 1 or sections[0].attrib:
        return None
    result = {}
    for group in sections[0]:
        if (
            group.tag != "default"
            or set(group.attrib) != {"class"}
            or not group.get("class")
            or group.get("class") in result
            or len(group) != 1
            or group[0].tag != "pair"
            or len(group[0])
            or set(group[0].attrib)
            - {"condim", "friction", "margin", "gap", "solref", "solimp"}
        ):
            return None
        result[group.get("class")] = group[0].attrib
    return result


def _asset_changes(before, after, updates):
    if _key(before) == _key(after):
        return True
    # Bowl dimensions change their inline vertices without changing mesh names
    # or faces. Preserve the rest of the parsed scene and let the native compiler
    # recompute this mesh's normals, hull, mass properties and contacts normally.
    if (
        before.tag != "mesh"
        or after.tag != "mesh"
        or not after.get("name")
        or before.get("file")
        or after.get("file")
        or before.text != after.text
        or len(before)
        or len(after)
        or {k: v for k, v in before.attrib.items() if k != "vertex"}
        != {k: v for k, v in after.attrib.items() if k != "vertex"}
        or "vertex" not in before.attrib
        or "vertex" not in after.attrib
    ):
        return False
    values = [float(value) for value in after.get("vertex").split()]
    if (
        len(values) != len(before.get("vertex").split())
        or len(values) % 3
        or not all(math.isfinite(value) for value in values)
    ):
        return False
    updates.append(after.get("name"))
    return True


@dataclass(frozen=True)
class ContactIndex:
    """Private immutable-by-contract contact comparison and immediate delta."""

    node: ET.Element
    entries: tuple
    token: object
    previous: object = None
    added: tuple = ()


def contact_index(node):
    normalized = _read(b"", tree=node)
    return ContactIndex(
        normalized, tuple((_key(child), child) for child in normalized), object()
    )


@dataclass(frozen=True)
class Source:
    """Read-only comparison data owned by one private native preparation."""

    static: tuple
    defaults: dict
    parts: dict
    contacts: tuple
    contact_index: ContactIndex | None = None


def parse(data, *, _tree=None, _contacts=None):
    """Index current source, optionally using its trusted author's exact tree.

    A private same-call tree avoids reparsing bytes the author just serialized.
    Copy it before normalization: retained comparison records must not alias the
    caller, which later fills the canonical reset key. Ordinary byte callers
    retain full parsing; no mutable file path is cached.
    """
    index = None
    tree = _tree
    if (
        tree is not None
        and _contacts is not None
        and _contacts.comparison is not None
        and tree.findall("contact") == [_contacts.node]
    ):
        index = _contacts.comparison
        tree = copy(tree)
        position = list(tree).index(_contacts.node)
        tree.remove(_contacts.node)
        root = _read(data, tree=tree)
        root.insert(position, index.node)
    else:
        root = _read(data, tree=tree)
    compiler = root.find("compiler")
    defaults = _pair_defaults(root)
    if (
        root.find(".//include") is not None
        or defaults is None
        or compiler is None
        or compiler.attrib != {"angle": "radian"}
        or any(
            len(root.findall(tag)) != 1
            for tag in ("asset", "worldbody", "contact", "keyframe")
        )
    ):
        return None
    keys = root.find("keyframe")
    if len(keys) != 1 or keys[0].tag != "key" or keys[0].attrib != {"name": "initial"}:
        return None
    parts = {}
    for tag in ("asset", "worldbody", "contact"):
        node = root.find(tag)
        parts[tag] = node
        root.remove(node)
    return Source(
        _key(root),
        defaults,
        parts,
        index.entries
        if index is not None
        else tuple((_key(node), node) for node in parts["contact"]),
        index,
    )


def _contact_delta(old, new):
    before, after = old.contact_index, new.contact_index
    if before is not None and after is not None:
        if before.token is after.token:
            return (), ()
        if after.previous is before.token:
            return after.added, ()
        if before.previous is after.token:
            return (), before.added
    remaining = Counter(key for key, _ in old.contacts)
    added, removed = [], []
    for key, node in new.contacts:
        if remaining[key]:
            remaining[key] -= 1
        else:
            added.append(node)
    for key, node in old.contacts:
        if remaining[key]:
            removed.append(node)
            remaining[key] -= 1
    return added, removed


def revise(spec, old, new):
    """Copy preparation for supported authored edits; return None otherwise.

    The caller freshly hashes every asset and indexes the exact current XML.
    These private comparison records are never changed or reused for different
    bytes. Unsupported differences retain the full native parser.
    """
    import mujoco as mj

    if old is None or new is None or old.static != new.static:
        return None
    old_parts, new_parts = old.parts, new.parts
    for tag, node in old_parts.items():
        if node.attrib != new_parts[tag].attrib:
            return None
    old_assets, new_assets = list(old_parts["asset"]), list(new_parts["asset"])
    mesh_changes = []
    if len(new_assets) < len(old_assets) or not all(
        _asset_changes(a, b, mesh_changes)
        for a, b in zip(old_assets, new_assets[: len(old_assets)], strict=True)
    ):
        return None
    assets = new_assets[len(old_assets) :]
    rebuilt_meshes = []
    if mesh_changes:
        first = next(
            i for i, node in enumerate(old_assets) if node.get("name") in mesh_changes
        )
        rebuilt_meshes = [
            node for node in new_assets[first : len(old_assets)] if node.tag == "mesh"
        ]
        if any(
            set(node.attrib) - {"name", "vertex", "face"}
            or not node.get("name")
            or not node.get("vertex")
            for node in rebuilt_meshes
        ):
            return None

    def objects(world):
        return {
            node.get("name"): node
            for node in world
            if node.tag == "body" and node.get("name", "").startswith("prop_")
        }

    old_bodies, new_bodies = (
        objects(old_parts["worldbody"]),
        objects(new_parts["worldbody"]),
    )
    for world, objects_by_name in (
        (old_parts["worldbody"], old_bodies),
        (new_parts["worldbody"], new_bodies),
    ):
        authored = [
            node
            for node in world
            if node.tag == "body" and node.get("name", "").startswith("prop_")
        ]
        if len(authored) != len(objects_by_name):
            return None  # Duplicate names must reach the ordinary parser.
        if authored and list(world)[-len(authored) :] != authored:
            return None  # Authoring appends its objects after static world nodes.
    if _encoded(
        n for n in old_parts["worldbody"] if n not in old_bodies.values()
    ) != _encoded(n for n in new_parts["worldbody"] if n not in new_bodies.values()):
        return None
    if [name for name in old_bodies if name in new_bodies] != [
        name for name in new_bodies if name in old_bodies
    ]:
        return None
    common = [name for name in new_bodies if name in old_bodies]
    if list(new_bodies)[: len(common)] != common:
        return None  # Attachment appends; retain the source's exact body order.
    updates = []
    if not all(
        _changes(old_bodies[name], new_bodies[name], updates) for name in common
    ):
        return None
    bodies = [node for name, node in new_bodies.items() if name not in old_bodies]
    removed = [node for name, node in old_bodies.items() if name not in new_bodies]
    geoms = {node.get("name") for body in bodies for node in body.iter("geom")}
    retired = {node.get("name") for body in removed for node in body.iter("geom")}
    if None in geoms | retired:
        return None
    pairs, obsolete_pairs = _contact_delta(old, new)
    for node in pairs:
        if node.tag != "pair" or not {node.get("geom1"), node.get("geom2")} & geoms:
            return None
    remove_pairs = []
    for node in obsolete_pairs:
        if node.tag != "pair" or not {node.get("geom1"), node.get("geom2")} & retired:
            return None
        remove_pairs.append((node.get("geom1"), node.get("geom2")))
    # Authored mesh additions are inline. File-backed new assets would need
    # directory rebinding before attachment; let the full parser handle them.
    if any(node.get("file") for parent in [*bodies, *assets] for node in parent.iter()):
        return None
    result = spec.copy()
    # Body deletion remaps the native keyframe arrays. Clear the old reset key
    # first; deleting/recreating it afterwards resurrects a duplicate native key.
    for key in list(result.keys):
        result.delete(key)
    result.add_key(name="initial")
    if remove_pairs:
        counts = Counter(remove_pairs)
        for pair in result.pairs:
            key = (pair.geomname1, pair.geomname2)
            if counts[key]:
                result.delete(pair)
                counts[key] -= 1
        if any(counts.values()):
            return None
    for node in removed:
        result.delete(result.body(node.get("name")))
    for tag, name, attribute, numbers in updates:
        node = getattr(result, tag)(name)
        setattr(node, attribute, numbers[0] if len(numbers) == 1 else numbers)
    # A compiled mesh retains derived native data: updating only uservert can
    # leave stale bounds. Recreate the inline mesh suffix, preserving its original
    # order and leaving robot assets, bodies and contact pairs in the copied spec.
    # Use fresh typed mesh declarations: reparsing large vertex strings here can
    # fail inside the native XML numeric parser and discard the entire preparation.
    for node in rebuilt_meshes:
        result.delete(result.mesh(node.get("name")))
    for node in rebuilt_meshes:
        result.add_mesh(
            name=node.get("name"),
            uservert=[float(value) for value in node.get("vertex").split()],
            userface=[int(value) for value in node.get("face", "").split()],
        )
    if bodies:
        delta = ET.Element("mujoco")
        ET.SubElement(delta, "compiler", angle="radian")
        for tag, nodes in (
            ("asset", assets),
            ("worldbody", bodies),
            ("contact", pairs),
        ):
            ET.SubElement(delta, tag).extend(deepcopy(nodes))
        # Attachment parses an independent child. Resolve the admitted pair-only
        # defaults explicitly so it needs no duplicate class definitions.
        defaults = new.defaults
        for pair in delta.find("contact"):
            if name := pair.attrib.pop("class", None):
                if name not in defaults:
                    return None
                pair.attrib = {**defaults[name], **pair.attrib}
        child = mj.MjSpec.from_string(ET.tostring(delta, encoding="unicode"))
        result.copy_during_attach = True
        result.attach(child, prefix="", frame=result.worldbody.add_frame())
    elif assets or pairs:
        return None
    return result
