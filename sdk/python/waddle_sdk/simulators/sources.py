"""Portable planning sources derived from the reference robot assemblies."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import replace
from importlib.resources import files
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np

from ..robots.models import ModelSources
from .description import axis_rotation, description, mesh_triangles
from .model import Shape, urdf
from .scene import profile, rotation


def _rpy(matrix) -> tuple[float, float, float]:
    pitch = math.asin(float(np.clip(-matrix[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1e-9:
        roll = math.atan2(matrix[2, 1], matrix[2, 2])
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    else:
        roll = math.atan2(-matrix[1, 2], matrix[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def _fixed_hand(name: str):
    """Freeze the complete hand collision assembly at its open configuration."""

    robot = description(name)
    value = robot.hand_position(1.0)
    result = []
    for link in robot.native_links():
        if link.joint not in robot.hand_names:
            result.append(link)
            continue
        if link.kind == "prismatic":
            xyz = tuple(
                np.asarray(link.xyz)
                + rotation(link.rpy) @ (np.asarray(link.axis) * value)
            )
            rpy = link.rpy
        else:
            xyz = link.xyz
            rpy = _rpy(rotation(link.rpy) @ axis_rotation(link.axis, value))
        result.append(
            replace(
                link,
                xyz=xyz,
                rpy=rpy,
                joint=None,
                kind="fixed",
                mimic=None,
            )
        )
    return result


def _collision_boxes(name: str):
    """Build bounded planner geometry from every source collision triangle."""

    result = []
    for link in _fixed_hand(name):
        yam_mesh = {
            "base": "base_link_collision.stl",
            **{f"link_{index}": f"link_{index}_collision.stl" for index in range(1, 6)},
            "gripper": "gripper.stl",
            "tip_left": "tip_left.stl",
            "tip_right": "tip_right.stl",
        }.get(link.name)
        if name == "yam" and yam_mesh is not None:
            path = files(__package__).joinpath(f"data/yam/assets/{yam_mesh}")
            result.append(
                replace(
                    link,
                    shapes=[
                        Shape("mesh", (1.0, 1.0, 1.0), mesh=str(path), visual=False)
                    ],
                )
            )
            continue
        surfaces = []
        for shape in link.shapes:
            if not shape.collision:
                continue
            triangles = mesh_triangles(shape.mesh) * np.asarray(shape.size)
            surfaces.append(triangles @ rotation(shape.rpy).T + np.asarray(shape.xyz))
        shapes = []
        if surfaces:
            triangles = np.concatenate(surfaces)
            axis = int(np.argmax(np.ptp(triangles.reshape(-1, 3), axis=0)))
            centers = triangles.mean(axis=1)[:, axis]
            bins = np.floor((centers - centers.min()) / 0.02).astype(int)
            for bucket in np.unique(bins):
                vertices = triangles[bins == bucket].reshape(-1, 3)
                lower, upper = vertices.min(axis=0), vertices.max(axis=0)
                shapes.append(
                    Shape(
                        "box",
                        tuple(np.maximum(upper - lower, 2e-6)),
                        xyz=tuple((lower + upper) / 2),
                        visual=False,
                    )
                )
        result.append(replace(link, shapes=shapes))
    return result


def reference_model_sources(robot: str, *, part_name: str) -> ModelSources:
    """Return a non-opening, self-contained planner model for a reference robot.

    The planner receives the same pinned CAD, arm joints, limits, base and TCP
    as the runtime world. The hand is fixed at its maximum declared opening so
    collision checks retain every source link at that fixed hand configuration;
    gripper commands remain owned by the ordinary SDK action interface.
    """

    import mujoco

    p = profile(robot)
    source = ET.fromstring(urdf(_collision_boxes(robot), robot + "_planning"))
    ET.SubElement(
        ET.SubElement(source, "mujoco"),
        "compiler",
        discardvisual="false",
        fusestatic="false",
        strippath="false",
    )
    spec = mujoco.MjSpec.from_string(ET.tostring(source, encoding="unicode"))
    root = ET.fromstring(spec.to_xml())
    bodies = {body.get("name"): body for body in root.iter("body")}
    ET.SubElement(bodies["tcp"], "site", name="tcp_site", size=".001")
    assets = {}
    for element in root.iter():
        filename = element.get("file")
        if filename is None:
            continue
        data = Path(filename).read_bytes()
        target = (
            f"assets/{hashlib.sha256(data).hexdigest()}{Path(filename).suffix.lower()}"
        )
        assets.setdefault(target, data)
        element.set("file", target)
    ET.indent(root)
    manifest = json.loads(files(__package__).joinpath("data/models.json").read_text())
    source_token = {"so101": "SO101", "yam": "i2rt", "xarm7": "xArm"}[robot]
    source_manifest = {
        url: digest
        for url, digest in manifest["sources"].items()
        if source_token in url
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
            "source_manifest": source_manifest,
            "geometry_scope": (
                "Bounded collision geometry from every pinned source link "
                "with the hand fixed at its declared maximum opening; "
                "no scene, props or other arm."
            ),
        },
    )


__all__ = ["reference_model_sources"]
