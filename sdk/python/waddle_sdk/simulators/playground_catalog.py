"""Published immutable object geometry; no scene state or native authority.

Regenerate the shipped bundle with ``python -m
waddle_sdk.simulators.playground_catalog`` after changing the object compiler.
Custom dimensions and absent/stale bundles keep ordinary source generation.
"""

import gzip
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from copy import deepcopy
from functools import lru_cache
from pathlib import Path

from .model import numbers
from .scene import quaternion, rotation

_ROOT = Path(__file__).parent
_BUNDLE = _ROOT / "data" / "playground-catalog.json.gz"
_SOURCES = (
    "playground.py",
    "playground_bowls.py",
    "model.py",
    "scene.py",
    "playground_catalog.py",
)
_PREFIX = "prop_catalog_object"


def _fingerprint():
    return {
        name: hashlib.sha256((_ROOT / name).read_bytes()).hexdigest()
        for name in _SOURCES
    }


def build():
    """Build all default object shapes offline, for both contact policies."""
    from .playground import CATALOG, _object_source

    entries = {}
    for kind, definition in CATALOG.items():
        for easy in (False, True):
            obj = {
                "id": "catalog_object",
                "kind": kind,
                "size": definition["size"],
                "position": [0, 0, 0],
                "rotation": [0, 0, 0],
                "color": [1, 1, 1],
            }
            world, asset = ET.Element("worldbody"), ET.Element("asset")
            names = _object_source(world, asset, obj, easy)
            entries[f"{kind}:{int(easy)}"] = {
                "size": definition["size"],
                "names": names,
                "world": ET.tostring(world, encoding="unicode"),
                "asset": ET.tostring(asset, encoding="unicode"),
            }
    document = {"version": 1, "sources": _fingerprint(), "entries": entries}
    return gzip.compress(
        json.dumps(document, sort_keys=True, separators=(",", ":")).encode(), mtime=0
    )


@lru_cache(maxsize=1)
def _load():
    try:
        document = json.loads(gzip.decompress(_BUNDLE.read_bytes()))
        if document["version"] != 1 or document["sources"] != _fingerprint():
            return {}
        return {
            key: (
                row["size"],
                tuple(row["names"]),
                ET.fromstring(row["world"]),
                ET.fromstring(row["asset"]),
            )
            for key, row in document["entries"].items()
        }
    except (OSError, EOFError, ValueError, KeyError, ET.ParseError):
        return {}


def instantiate(world, asset, obj, easy_grasp):
    """Bind identity, pose and color to private copies of shipped geometry."""
    row = _load().get(f"{obj['kind']}:{int(easy_grasp)}")
    if row is None or obj["size"] != row[0]:
        return None
    _, names, source_world, source_asset = row
    placed, meshes = deepcopy(source_world), deepcopy(source_asset)
    prefix = "prop_" + obj["id"]

    def rename(value):
        return prefix + value[len(_PREFIX) :] if value.startswith(_PREFIX) else value

    for root in (placed, meshes):
        for node in root.iter():
            for key in ("name", "mesh"):
                if value := node.get(key):
                    node.set(key, rename(value))
    body = placed.find("body")
    body.set("pos", numbers(obj["position"]))
    body.set(
        "quat",
        numbers(quaternion(rotation([math.radians(x) for x in obj["rotation"]]))),
    )
    for geom in body.iter("geom"):
        geom.set("rgba", numbers([*obj["color"], 1]))
    world.extend(placed)
    asset.extend(meshes)
    return [rename(name) for name in names]


def main():
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=_BUNDLE)
    args = parser.parse_args()
    payload = build()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(payload)
    print(f"Wrote {len(payload)} bytes of immutable catalog geometry to {args.output}")


if __name__ == "__main__":
    main()
