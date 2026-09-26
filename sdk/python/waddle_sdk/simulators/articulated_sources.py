"""Portable articulated reference assemblies, without a surrounding scene."""

from __future__ import annotations

import hashlib
import json
from importlib.resources import files
from pathlib import Path
from xml.etree import ElementTree as ET

from ..robots.models import ModelSources
from .description import description
from .model import mjcf
from .scene import profile


def articulated_model_sources(robot: str, *, part_name: str) -> ModelSources:
    """Export the reference native robot, preserving its complete moving hand.

    Uses the same MJCF assembly as the reference MuJoCo world: manufacturer
    visuals/inertias, collision pieces, arm and hand joints, coupling, contact
    exclusions, tendons and position actuators. The base is at the origin; callers
    supply its placement. No site, device, camera or environment is opened or
    exported. MuJoCo is needed only for this explicit source compilation.

    The bundle's primary joint bindings describe the arm. Hand joints and their
    units are retained in provenance and in the native model; they do not imply
    an affine physical jaw mapping. Servo/contact parameters are the reference
    simulator's assumptions, not measured hardware identification.
    """
    p = profile(robot)
    native = description(robot)
    config = {
        "robot": robot,
        "parts": {part_name: {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}},
        "timestep": 0.002,
        "cameras": {},
    }
    root = ET.fromstring(mjcf(p, config, robot_only=True))
    # Lighting and view configuration belong to the caller's scene.
    root.remove(root.find("visual"))
    world = root.find("worldbody")
    for light in list(world.findall("light")):
        world.remove(light)
    asset = root.find("asset")
    for texture in list(asset.findall("texture")):
        if texture.get("type") == "skybox":
            asset.remove(texture)
    assets = {}
    for element in root.iter():
        filename = element.get("file")
        if filename is None:
            continue
        path = Path(filename)
        data = path.read_bytes()
        relative = f"assets/{hashlib.sha256(data).hexdigest()}{path.suffix.lower()}"
        assets.setdefault(relative, data)
        element.set("file", relative)
        # MuJoCo may omit this inferred attribute once its asset cache is warm.
        element.attrib.pop("content_type", None)
    for compiler in root.findall("compiler"):
        for key in ("assetdir", "meshdir", "texturedir"):
            compiler.attrib.pop(key, None)
    ET.indent(root)
    manifest = json.loads(files(__package__).joinpath("data/models.json").read_text())
    token = {"so101": "SO101", "yam": "i2rt", "xarm7": "xArm"}[robot]
    hand = {
        link.joint: link
        for link in native.native_links()
        if link.joint in native.hand_names
    }
    return ModelSources(
        format="mjcf",
        model=ET.tostring(root, encoding="utf-8", xml_declaration=True),
        assets=assets,
        joint_names=p.names[:-1],
        joint_units=("rad",) * p.dof,
        base_body="base",
        tcp_site="tcp_site",
        tcp_frame=part_name + "_tool",
        licenses={
            f"LICENSE.{robot}": files(__package__)
            .joinpath(f"data/{robot}/LICENSE")
            .read_bytes()
        },
        provenance={
            "reference_robot": robot,
            "source_manifest": {
                url: sha for url, sha in manifest["sources"].items() if token in url
            },
            "geometry_scope": "Articulated reference robot at its base origin; no scene, props, cameras or other arm.",
            "hand_joints": {
                name: {
                    "unit": "m" if hand[name].kind == "prismatic" else "rad",
                    "limits": list(hand[name].limits),
                }
                for name in native.hand_names
            },
            "dynamics_scope": "Reference MuJoCo servo/contact assumptions; no physical identification or acceptance.",
        },
    )
