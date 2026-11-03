"""Extend a privately prepared authored scene after an exact source comparison."""

import xml.etree.ElementTree as ET
from collections import Counter
from copy import deepcopy


def _read(data):
    root = ET.fromstring(data)
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


def revise(spec, previous, current):
    """Copy preparation for supported authored edits; return None otherwise.

    The caller has freshly hashed every non-root asset and holds the matching
    virtual files. Only its trusted authoring path uses this optimization.
    Current XML remains authoritative; defaults, includes, changed topology in
    existing bodies and unfamiliar reset keys take the complete parser.
    """
    import mujoco as mj

    old, new = _read(previous), _read(current)
    for root in (old, new):
        compiler = root.find("compiler")
        if (
            root.find(".//include") is not None
            or root.find("default") is not None
            or compiler is None
            or compiler.attrib != {"angle": "radian"}
            or any(
                len(root.findall(tag)) != 1
                for tag in ("asset", "worldbody", "contact", "keyframe")
            )
        ):
            return None
        keys = root.find("keyframe")
        if (
            len(keys) != 1
            or keys[0].tag != "key"
            or keys[0].attrib != {"name": "initial"}
        ):
            return None
    old_parts, new_parts = {}, {}
    for root, parts in ((old, old_parts), (new, new_parts)):
        for tag in ("asset", "worldbody", "contact"):
            node = root.find(tag)
            parts[tag] = node
            root.remove(node)
    if _key(old) != _key(new):
        return None
    for tag, node in old_parts.items():
        if node.attrib != new_parts[tag].attrib:
            return None
    old_assets, new_assets = list(old_parts["asset"]), list(new_parts["asset"])
    if _encoded(new_assets[: len(old_assets)]) != _encoded(old_assets):
        return None
    assets = new_assets[len(old_assets) :]

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
    remaining = Counter(_encoded(old_parts["contact"]))
    pairs = []
    for node in new_parts["contact"]:
        key = _key(node)
        if remaining[key]:
            remaining[key] -= 1
        elif node.tag == "pair" and {node.get("geom1"), node.get("geom2")} & geoms:
            pairs.append(node)
        else:
            return None
    remove_pairs = []
    for node in old_parts["contact"]:
        key = _key(node)
        if remaining[key]:
            if (
                node.tag != "pair"
                or not {node.get("geom1"), node.get("geom2")} & retired
            ):
                return None
            remove_pairs.append((node.get("geom1"), node.get("geom2")))
            remaining[key] -= 1
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
    if bodies:
        delta = ET.Element("mujoco")
        ET.SubElement(delta, "compiler", angle="radian")
        for tag, nodes in (
            ("asset", assets),
            ("worldbody", bodies),
            ("contact", pairs),
        ):
            ET.SubElement(delta, tag).extend(deepcopy(nodes))
        child = mj.MjSpec.from_string(ET.tostring(delta, encoding="unicode"))
        result.copy_during_attach = True
        result.attach(child, prefix="", frame=result.worldbody.add_frame())
    elif assets or pairs:
        return None
    return result
