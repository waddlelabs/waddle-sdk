"""Private parsed object/contact templates for the trusted authored compiler.

Only one flat prop body with inline meshes is normalized. Every other byte,
including contact materials and external geometry names, remains in the key.
Two templates retain at most 8 MiB of XML and 16,384 pairs per template; native
specifications are additionally bounded by those declaration counts. No compiled
model, data, source-validity decision or running state is retained here.
"""

import math
import xml.etree.ElementTree as ET
from collections import OrderedDict
from copy import deepcopy

_PREFIX = "__waddle_prepared_prop"
_REFERENCES = ("name", "mesh", "geom1", "geom2")


def normalized(delta, max_pairs):
    if (
        delta.tag != "mujoco"
        or delta.attrib
        or [node.tag for node in delta] != ["compiler", "asset", "worldbody", "contact"]
        or delta.find("compiler").attrib != {"angle": "radian"}
    ):
        return None
    world, asset, contact = (
        delta.find(tag) for tag in ("worldbody", "asset", "contact")
    )
    if world is None or asset is None or contact is None or len(world) != 1:
        return None
    body = world[0]
    prefix = body.get("name", "")
    if (
        body.tag != "body"
        or not prefix.startswith("prop_")
        or set(body.attrib) != {"name", "pos", "quat"}
        or any(node.tag not in {"geom", "freejoint"} or len(node) for node in body)
        or len(body) > 129
        or len(asset) > 128
        or any(node.tag != "mesh" or node.get("file") or len(node) for node in asset)
        or len(contact) > max_pairs
        or any(node.tag != "pair" or len(node) for node in contact)
    ):
        return None
    pose = [[float(v) for v in body.get(key).split()] for key in ("pos", "quat")]
    if [len(v) for v in pose] != [3, 4] or not all(
        math.isfinite(v) for row in pose for v in row
    ):
        return None
    names = {node.get("name") for node in delta.iter() if node.get("name")}
    if not all(name.startswith(prefix) for name in names):
        return None
    mapping = {name: _PREFIX + name[len(prefix) :] for name in names}
    external = {
        node.get(key)
        for node in delta.iter()
        for key in _REFERENCES
        if node.get(key) not in names
    }
    if external.intersection(mapping.values()):
        return None
    copied = deepcopy(delta)
    for node in copied.iter():
        for key in _REFERENCES:
            if node.get(key) in mapping:
                node.set(key, mapping[node.get(key)])
    copied.find("worldbody/body").set("pos", "0 0 0")
    copied.find("worldbody/body").set("quat", "1 0 0 0")
    return (
        ET.tostring(copied, encoding="unicode"),
        {v: k for k, v in mapping.items()},
        pose,
    )


def _rename(spec, names):
    for collection in (spec.bodies, spec.geoms, spec.meshes, spec.joints, spec.pairs):
        for node in collection:
            if node.name in names:
                node.name = names[node.name]
    for geom in spec.geoms:
        if geom.meshname in names:
            geom.meshname = names[geom.meshname]
    for pair in spec.pairs:
        for key in ("geomname1", "geomname2"):
            if getattr(pair, key) in names:
                setattr(pair, key, names[getattr(pair, key)])


class FragmentCache:
    """Owned/serialized by SourceBuffers; native templates never escape the cache."""

    def __init__(self, *, max_bytes=8 * 1024 * 1024, max_entries=2, max_pairs=16384):
        self.max_bytes, self.max_entries, self.max_pairs = (
            max_bytes,
            max_entries,
            max_pairs,
        )
        self.entries = OrderedDict()

    def close(self):
        self.entries.clear()

    def attach(self, parent, delta):
        import mujoco as mj

        if not parent.copy_during_attach:
            raise ValueError("Prepared fragments require copying during attachment")

        def attach(child):
            return parent.attach(child, prefix="", frame=parent.worldbody.add_frame())

        source = normalized(delta, self.max_pairs)
        if source is None:
            return attach(mj.MjSpec.from_string(ET.tostring(delta, encoding="unicode")))
        xml, names, pose = source
        size = len(xml.encode())
        if size > self.max_bytes or not self.max_entries:
            return attach(mj.MjSpec.from_string(ET.tostring(delta, encoding="unicode")))
        entry = self.entries.pop(xml, None)
        if entry is None:
            entry = size, mj.MjSpec.from_string(xml)
        self.entries[xml] = entry
        while (
            len(self.entries) > self.max_entries
            or sum(x[0] for x in self.entries.values()) > self.max_bytes
        ):
            self.entries.popitem(last=False)
        child = entry[1]
        body = child.body(_PREFIX)
        # Standalone MjSpec.copy() discards contacts referring to the parent.
        # Copy during attachment instead, where those references can resolve.
        # SourceBuffers holds its lock throughout this private rebinding.
        try:
            body.pos, body.quat = pose
            _rename(child, names)
            return attach(child)
        finally:
            _rename(child, {actual: canonical for canonical, actual in names.items()})
            body.pos, body.quat = [0, 0, 0], [1, 0, 0, 0]
