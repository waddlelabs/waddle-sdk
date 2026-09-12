"""Portable planning sources derived from the reference robot assemblies."""

from __future__ import annotations

import hashlib
import json
import math
import tempfile
from dataclasses import replace
from importlib.resources import files
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np

from ..robots.models import ModelSources
from .description import axis_rotation, description, mesh_triangles
from .model import Shape, urdf
from .scene import profile, rotation

_HULL_BUCKET_WIDTH_M = 0.02


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


def _obj(vertices: np.ndarray, faces: np.ndarray, comment: str) -> bytes:
    lines = [f"# {comment}"]
    lines.extend(
        "v " + " ".join(format(float(value), ".17g") for value in point)
        for point in vertices
    )
    lines.extend(
        "f " + " ".join(str(int(index) + 1) for index in face) for face in faces
    )
    return ("\n".join(lines) + "\n").encode()


def _source_obj(triangles: np.ndarray) -> tuple[bytes, np.ndarray]:
    """Serialize complete source triangles for an initial MuJoCo hull pass."""

    vertices, inverse = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    return (
        _obj(
            vertices,
            inverse.reshape(-1, 3),
            "Complete source collision pieces before convex-hull compaction",
        ),
        vertices,
    )


def _compact_hull(model, mesh_id: int, vertices: np.ndarray) -> bytes:
    """Retain the exact MuJoCo hull vertices and polygon faces for one mesh."""

    first = int(model.mesh_polyadr[mesh_id])
    count = int(model.mesh_polynum[mesh_id])
    polygons = []
    for polygon_id in range(first, first + count):
        start = int(model.mesh_polyvertadr[polygon_id])
        length = int(model.mesh_polyvertnum[polygon_id])
        polygon = tuple(
            int(value) for value in model.mesh_polyvert[start : start + length]
        )
        if length < 3 or any(index < 0 or index >= len(vertices) for index in polygon):
            raise ValueError("MuJoCo returned an invalid convex mesh polygon")
        polygons.append(polygon)
    used = sorted({index for polygon in polygons for index in polygon})
    if len(used) < 4:
        raise ValueError("Planner collision hull requires four noncoplanar vertices")
    remap = {old: new for new, old in enumerate(used)}
    faces = []
    for polygon in polygons:
        for index in range(1, len(polygon) - 1):
            faces.append(
                (remap[polygon[0]], remap[polygon[index]], remap[polygon[index + 1]])
            )
    return _obj(
        vertices[used],
        np.asarray(faces, dtype=int),
        "Exact convex planner envelope from complete source collision pieces",
    )


def _collision_geometry(name: str, scratch: Path):
    """Build bounded hulls that enclose every complete source collision piece."""

    result = []
    generated = {}
    source_pieces = 0
    planner_geometries = 0
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
            source_pieces += 1
            planner_geometries += 1
            result.append(
                replace(
                    link,
                    shapes=[
                        Shape("mesh", (1.0, 1.0, 1.0), mesh=str(path), visual=False)
                    ],
                )
            )
            continue
        pieces = []
        for shape in link.shapes:
            if not shape.collision:
                continue
            triangles = mesh_triangles(shape.mesh) * np.asarray(shape.size)
            pieces.append(triangles @ rotation(shape.rpy).T + np.asarray(shape.xyz))
        shapes = []
        if pieces:
            source_pieces += len(pieces)
            vertices = np.concatenate(pieces).reshape(-1, 3)
            axis = int(np.argmax(np.ptp(vertices, axis=0)))
            centers = np.asarray(
                [piece.reshape(-1, 3).mean(axis=0)[axis] for piece in pieces]
            )
            bins = np.floor((centers - centers.min()) / _HULL_BUCKET_WIDTH_M).astype(
                int
            )
            for bucket in np.unique(bins):
                triangles = np.concatenate(
                    [
                        piece
                        for piece, selected in zip(pieces, bins, strict=True)
                        if selected == bucket
                    ]
                )
                path = scratch / f"{link.name}-{bucket}.obj"
                content, vertices = _source_obj(triangles)
                path.write_bytes(content)
                generated[path.resolve()] = vertices
                shapes.append(
                    Shape(
                        "mesh",
                        (1.0, 1.0, 1.0),
                        mesh=str(path),
                        visual=False,
                    )
                )
                planner_geometries += 1
        result.append(replace(link, shapes=shapes))
    return (
        result,
        generated,
        {
            "method": (
                "exact_source_meshes"
                if name == "yam"
                else "spatial_convex_hulls_of_complete_source_collision_pieces"
            ),
            "bucket_width_m": None if name == "yam" else _HULL_BUCKET_WIDTH_M,
            "source_piece_count": source_pieces,
            "planner_geometry_count": planner_geometries,
        },
    )


def reference_model_sources(robot: str, *, part_name: str) -> ModelSources:
    """Return a non-opening, self-contained planner model for a reference robot.

    The planner receives the same pinned CAD, arm joints, limits, base and TCP
    as the runtime world. The hand is fixed at its maximum declared opening so
    collision checks retain every source link at that fixed hand configuration;
    gripper commands remain owned by the ordinary SDK action interface.
    """

    import mujoco

    p = profile(robot)
    with tempfile.TemporaryDirectory(prefix="waddle-planner-source-") as directory:
        geometry, generated, proxy = _collision_geometry(robot, Path(directory))
        source = ET.fromstring(urdf(geometry, robot + "_planning"))
        ET.SubElement(
            ET.SubElement(source, "mujoco"),
            "compiler",
            discardvisual="false",
            fusestatic="false",
            strippath="false",
        )
        spec = mujoco.MjSpec.from_string(ET.tostring(source, encoding="unicode"))
        if generated:
            compiled = spec.compile()
            prepared = ET.fromstring(spec.to_xml())
            compacted = set()
            for mesh in prepared.findall("asset/mesh"):
                filename = Path(mesh.get("file", "")).resolve()
                if filename not in generated:
                    continue
                mesh_id = mujoco.mj_name2id(
                    compiled, mujoco.mjtObj.mjOBJ_MESH, mesh.get("name")
                )
                if mesh_id < 0:
                    raise ValueError(
                        "Generated planner mesh is absent after compilation"
                    )
                filename.write_bytes(
                    _compact_hull(compiled, mesh_id, generated[filename])
                )
                compacted.add(filename)
            if compacted != set(generated):
                raise ValueError("Generated planner meshes were not all compiled")
            spec = mujoco.MjSpec.from_string(ET.tostring(source, encoding="unicode"))
        root = ET.fromstring(spec.to_xml())
        bodies = {body.get("name"): body for body in root.iter("body")}
        ET.SubElement(bodies["tcp"], "site", name="tcp_site", size=".001")
        for body in root.iter("body"):
            body_name = body.get("name")
            if body_name is None:
                continue
            for index, geom in enumerate(body.findall("geom")):
                if geom.get("name") is None:
                    geom.set("name", f"{body_name}_collision_{index}")
        assets = {}
        for element in root.iter():
            filename = element.get("file")
            if filename is None:
                continue
            if element.tag == "mesh":
                # MjSpec may omit this inferred attribute after its process-wide
                # asset cache is warm. The explicit file suffix is sufficient;
                # normalize it so identical sources have identical model bytes.
                element.attrib.pop("content_type", None)
            data = Path(filename).read_bytes()
            target = (
                f"assets/{hashlib.sha256(data).hexdigest()}"
                f"{Path(filename).suffix.lower()}"
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
            "collision_proxy": proxy,
            "geometry_scope": (
                "Bounded collision geometry from every pinned source link "
                "with the hand fixed at its declared maximum opening; "
                "no scene, props or other arm."
            ),
        },
    )


__all__ = ["reference_model_sources"]
