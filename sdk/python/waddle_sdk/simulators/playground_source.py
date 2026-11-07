"""Bounded authored XML fragments; no native models or running state.

Reference assets are read and hashed again on every request. Returned XML is
private to the caller; unchanged source fragments never acquire live poses.
"""

import hashlib
import os
import threading
import xml.etree.ElementTree as ET
from collections import OrderedDict
from copy import deepcopy
from dataclasses import dataclass, field
from pathlib import Path


def capture(root):
    assets = {}
    for node in root.iter():
        if name := node.get("file"):
            path = Path(name)
            if path not in assets:
                data = path.read_bytes()
                assets[path] = data, hashlib.sha256(data).hexdigest()
    return assets


@dataclass
class Reference:
    root: ET.Element
    fingers: tuple
    props: tuple
    files: dict
    size: int
    objects: dict = field(default_factory=dict)


class SourceCache:
    def __init__(self, *, max_bytes=4 * 1024 * 1024, max_entries=2):
        self.max_bytes, self.max_entries = max_bytes, max_entries
        self.pid = os.getpid()
        self.lock = threading.Lock()
        self.entries = OrderedDict()

    def prepare(self, key, reference, object_source, objects, easy_grasp):
        if self.pid != os.getpid():
            self.lock = threading.Lock()
            self.entries = OrderedDict()
            self.pid = os.getpid()
        with self.lock:
            entry = self.entries.pop(key, None)
            assets = {} if entry is None else capture(entry.root)
            if entry is None or entry.files != {
                path: digest for path, (_, digest) in assets.items()
            }:
                root, fingers, props = reference()
                assets = capture(root)
                entry = Reference(
                    root,
                    tuple(fingers),
                    tuple(props),
                    {path: digest for path, (_, digest) in assets.items()},
                    len(ET.tostring(root)),
                )
            root = deepcopy(entry.root)
            world, asset = root.find("worldbody"), root.find("asset")
            selected, props = {}, []
            for obj in objects:
                previous = entry.objects.get(obj["id"])
                if previous is None or previous[0] != obj:
                    body, meshes = ET.Element("worldbody"), ET.Element("asset")
                    names = object_source(body, meshes, obj, easy_grasp)
                    previous = (
                        deepcopy(obj),
                        body,
                        meshes,
                        tuple(names),
                        len(ET.tostring(body)) + len(ET.tostring(meshes)),
                    )
                selected[obj["id"]] = previous
                world.extend(deepcopy(list(previous[1])))
                asset.extend(deepcopy(list(previous[2])))
                props.extend(previous[3])
            entry.objects = selected
            self.entries[key] = entry
            while (
                len(self.entries) > self.max_entries
                or sum(
                    item.size + sum(obj[4] for obj in item.objects.values())
                    for item in self.entries.values()
                )
                > self.max_bytes
            ):
                self.entries.popitem(last=False)
            return root, assets, entry.fingers, [*props, *entry.props]


cache = SourceCache()
