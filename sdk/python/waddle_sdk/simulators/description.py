"""Shared loader for the packaged, hash-pinned manufacturer URDF assemblies.

This is the reference robots' internal importer, not another public scene
format. All engines consume the same geometry, inertias, joints and hand map.
"""

from __future__ import annotations

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, replace
from functools import lru_cache
from importlib.resources import files
from pathlib import Path

import numpy as np

from .model import Link, Shape
from .scene import profile, rotation, transform


def vector(node: ET.Element | None, key: str, default="0 0 0") -> tuple[float, ...]:
    return tuple(
        map(float, (default if node is None else node.get(key, default)).split())
    )


def axis_rotation(axis, angle):
    x, y, z = np.asarray(axis) / np.linalg.norm(axis)
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * (cross @ cross)


@lru_cache(maxsize=128)
def mesh_triangles(filename: str) -> np.ndarray:
    """Packaged meshes are binary STL in metres, converted only at vendoring."""
    data = Path(filename).read_bytes()
    count = int.from_bytes(data[80:84], "little")
    if len(data) != 84 + count * 50 or not count:
        raise ValueError("reference mesh must be a nonempty binary STL")
    rows = np.frombuffer(
        data,
        dtype=np.dtype(
            [("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")]
        ),
        offset=84,
        count=count,
    )
    result = rows["vertices"].astype(float)
    if not np.isfinite(result).all():
        raise ValueError("reference mesh has non-finite vertices")
    result.setflags(write=False)
    return result


@dataclass
class Description:
    name: str
    links: list[Link]
    hand_names: tuple[str, ...]
    hand_limits: tuple[tuple[float, float], ...]
    exclusions: tuple[tuple[str, str], ...]

    def native_links(self):
        # Menagerie's xArm hand uses two ball-joint loop closures. Only the
        # opposing driver is an angle mimic; the linkage joints are passive.
        if self.name != "xarm7":
            return self.links
        return [
            replace(link, mimic=None)
            if link.mimic and link.joint != "right_outer_knuckle_joint"
            else link
            for link in self.links
        ]

    def closures(self):
        if self.name != "xarm7":
            return ()
        # Menagerie finger-local anchors; derive the other frame from the
        # manufacturer's zero-pose URDF rather than rounding its geometry.
        poses = {}
        for link in self.links:
            poses[link.name] = (
                np.eye(4) if link.parent is None else poses[link.parent]
            ) @ transform(link.xyz, link.rpy)
        result = []
        for side, sign in (("left", -1), ("right", 1)):
            first, second = f"{side}_finger", f"{side}_inner_knuckle"
            anchor = (0.0, sign * 0.015, 0.015)
            other = np.linalg.inv(poses[second]) @ poses[first] @ np.r_[anchor, 1.0]
            result.append((first, anchor, second, tuple(other[:3])))
        return result

    def servo(self, joint: str) -> tuple[float, float, float]:
        """Position gain, velocity gain, and force limit from upstream references.

        I2RT 1.3.5 supplies YAM motor gains; Menagerie supplies xArm's native
        position drives. Mimic joints are passive, with one motor per hand.
        Sources and unit conversions are recorded in data/README.md.
        """
        if joint in self.hand_names:
            if joint != self.hand_names[0]:
                return 0.0, 0.0, 0.0
            if self.name == "yam":
                # I2RT linear_4310: 6.57 rad motor stroke / 96 mm jaw stroke.
                # One driven slide travels half the jaw separation.
                ratio = 0.096 / (2 * 6.57)
                return 20 / ratio**2, 0.5 / ratio**2, 0.5 / ratio
            # UFACTORY rates G2 at 50 N gripping force, not 50 N m hinge
            # torque. Virtual work gives tau = F * abs(dwidth/dangle).
            # Its minimum transmission over [0, .85] is 2 * .042039 m/rad
            # (the manufacturer's finger-link Z offset at the open stop).
            # A constant native torque bound respects the quasi-static rating
            # throughout the stroke without a custom per-step force callback.
            return 100.0, 10.0, 50.0 * 2 * 0.042039
        index = profile(self.name).names.index(joint)
        if self.name == "yam":
            return (80.0, 5.0, 28.0) if index < 3 else (10.0, 1.5, 10.0)
        return (
            (1500.0, 150.0, 50.0)
            if index < 2
            else (1000.0, 100.0, 30.0)
            if index < 5
            else (800.0, 80.0, 20.0)
        )

    def armature(self, joint: str) -> float:
        """Reflected rotor inertia, following Menagerie and mjlab conventions."""
        if joint in self.hand_names:
            if joint != self.hand_names[0]:
                return 0.0
            return (0.0018 / (0.096 / (2 * 6.57)) ** 2) if self.name == "yam" else 0.005
        return (
            (0.032 if profile(self.name).names.index(joint) < 3 else 0.0018)
            if self.name == "yam"
            else 0.1
        )

    def hand_position(self, opening: float) -> float:
        if self.name == "yam":
            return opening * profile(self.name).opening / 2
        # G2 parallelogram: outer knuckle pivot (y,z)=(.035465,.042039).
        # Opening is jaw metres / .084, not revolute angle / angle limit.
        a, b = 0.035465, 0.042039
        closed = a * math.cos(0.85) - b * math.sin(0.85)
        rhs = closed + opening * profile(self.name).opening / 2
        return math.acos(float(np.clip(rhs / math.hypot(a, b), -1, 1))) - math.atan2(
            b, a
        )

    def hand_state(self, q: float, dq: float) -> tuple[float, float]:
        width = profile(self.name).opening
        if self.name == "yam":
            return q * 2 / width, dq * 2 / width
        a, b = 0.035465, 0.042039
        closed = a * math.cos(0.85) - b * math.sin(0.85)
        return (
            2 * (a * math.cos(q) - b * math.sin(q) - closed) / width,
            -2 * (a * math.sin(q) + b * math.cos(q)) * dq / width,
        )

    def expand(self, q) -> tuple[float, ...]:
        return tuple(q[:-1]) + (self.hand_position(q[-1]),) * len(self.hand_names)

    def poses(self, q) -> dict[str, np.ndarray]:
        p = profile(self.name)
        p.poses(q)  # public vector validation
        values = dict(zip(p.names[:-1] + self.hand_names, self.expand(q), strict=True))
        result = {}
        for link in self.links:
            pose = transform(link.xyz, link.rpy)
            if link.joint:
                motion = np.eye(4)
                if link.kind == "prismatic":
                    motion[:3, 3] = np.asarray(link.axis) * values[link.joint]
                else:
                    motion[:3, :3] = axis_rotation(link.axis, values[link.joint])
                pose = pose @ motion
            result[link.name] = (
                pose if link.parent is None else result[link.parent] @ pose
            )
        return result


@lru_cache(maxsize=2)
def description(name: str) -> Description:
    p = profile(name)
    data = Path(str(files(__package__).joinpath("data")))
    manifest = json.loads((data / "models.json").read_text())["files"]
    for relative, digest in manifest.items():
        if relative.startswith(name + "/"):
            path = (data / relative).resolve()
            path.relative_to(data.resolve())
            if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
                raise ValueError(f"reference robot asset hash mismatch: {relative}")
    source = data / name / "robot.urdf"
    root = ET.parse(source).getroot()
    aliases = (
        {"base_link": "base", "grasp_link": "tcp"}
        if name == "yam"
        else {"link_base": "base", "link_tcp": "tcp"}
    )
    materials = {
        m.get("name"): vector(m.find("color"), "rgba", "0.8 0.8 0.8 1")
        for m in root.findall("material")
    }
    links = {}
    for raw in root.findall("link"):
        link_name = aliases.get(raw.get("name"), raw.get("name"))
        link = Link(link_name, mass=1e-6, inertia=(1e-9, 1e-9, 1e-9, 0, 0, 0))
        inertial = raw.find("inertial")
        if inertial is not None:
            link.mass = float(inertial.find("mass").get("value"))
            link.com = vector(inertial.find("origin"), "xyz")
            row = inertial.find("inertia")
            xx, yy, zz, xy, xz, yz = (
                float(row.get(k)) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
            )
            matrix = np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
            r = rotation(vector(inertial.find("origin"), "rpy"))
            matrix = r @ matrix @ r.T
            link.inertia = tuple(
                matrix[i, j]
                for i, j in ((0, 0), (1, 1), (2, 2), (0, 1), (0, 2), (1, 2))
            )
        for kind in ("visual", "collision"):
            for geom in raw.findall(kind):
                mesh = geom.find("geometry/mesh")
                if mesh is None:
                    raise ValueError(
                        "reference robot geometry must be a manufacturer mesh"
                    )
                path = (source.parent / mesh.get("filename")).resolve()
                relative = str(path.relative_to(data.resolve()))
                if relative not in manifest:
                    raise ValueError(
                        "reference URDF mesh is absent from asset manifest"
                    )
                material = geom.find("material")
                color = (
                    materials.get(material.get("name"), (0.8, 0.8, 0.8, 1))
                    if material is not None
                    else (0.25, 0.27, 0.3, 1)
                )
                if material is not None and material.find("color") is not None:
                    color = vector(material.find("color"), "rgba")
                link.shapes.append(
                    Shape(
                        "mesh",
                        vector(mesh, "scale", "1 1 1"),
                        vector(geom.find("origin"), "xyz"),
                        vector(geom.find("origin"), "rpy"),
                        color,
                        str(path),
                        kind == "visual",
                        kind == "collision",
                    )
                )
        links[link_name] = link
    for joint in root.findall("joint"):
        child = joint.find("child").get("link")
        link = links[aliases.get(child, child)]
        parent = joint.find("parent").get("link")
        link.parent = aliases.get(parent, parent)
        link.xyz, link.rpy = (
            vector(joint.find("origin"), "xyz"),
            vector(joint.find("origin"), "rpy"),
        )
        link.kind = joint.get("type")
        if link.kind != "fixed":
            if link.kind not in ("prismatic", "revolute"):
                raise ValueError("reference robot needs bounded scalar joints")
            link.joint = joint.get("name")
            if name == "xarm7" and link.joint in {f"joint{i}" for i in range(1, 8)}:
                link.joint = f"joint_{link.joint[5:]}"
            link.axis = vector(joint.find("axis"), "xyz", "1 0 0")
            limit = joint.find("limit")
            link.limits = tuple(float(limit.get(k)) for k in ("lower", "upper"))
            if link.joint in p.names[:-1]:
                link.limits = p.limits[p.names.index(link.joint)]
            mimic = joint.find("mimic")
            if mimic is not None:
                link.mimic = (
                    mimic.get("joint"),
                    float(mimic.get("multiplier", "1")),
                    float(mimic.get("offset", "0")),
                )
    ordered = []
    remaining = dict(links)
    while remaining:
        ready = [n for n, link in remaining.items() if link.parent not in remaining]
        if not ready:
            raise ValueError("reference URDF contains a joint cycle")
        ordered.extend(remaining.pop(n) for n in ready)
    hand = tuple(
        link for link in ordered if link.joint and link.joint not in p.names[:-1]
    )
    # Mechanical connections inside the hand that a URDF tree cannot express
    # as parent/child: opposing linear fingers and the G2 four-bar loop pins.
    exclusions = (
        (("tip_left", "tip_right"),)
        if name == "yam"
        else (
            ("left_finger", "left_inner_knuckle"),
            ("right_finger", "right_inner_knuckle"),
            ("left_outer_knuckle", "left_inner_knuckle"),
            ("right_outer_knuckle", "right_inner_knuckle"),
            ("xarm_gripper_base_link", "left_finger"),
            ("xarm_gripper_base_link", "right_finger"),
        )
    )
    # Fixed frame links do not create a new physical body. Match native welded
    # body adjacency even when a loader retains those frame-only links.
    groups = {}
    for link in ordered:
        groups[link.name] = (
            groups[link.parent] if link.parent and link.kind == "fixed" else link.name
        )
    adjacent = {
        frozenset((groups[link.name], groups[link.parent]))
        for link in ordered
        if link.parent
    }
    extra = set(exclusions)
    physical = [link for link in ordered if any(s.collision for s in link.shapes)]
    for i, first in enumerate(physical):
        for second in physical[i + 1 :]:
            if first.parent == second.name or second.parent == first.name:
                continue
            pair = frozenset((groups[first.name], groups[second.name]))
            if len(pair) == 1 or pair in adjacent:
                extra.add((first.name, second.name))
    return Description(
        name,
        ordered,
        tuple(link.joint for link in hand),
        tuple(link.limits for link in hand),
        tuple(sorted(extra)),
    )


@lru_cache(maxsize=2)
def collision_bounds(name: str):
    """Conservative sphere cover of complete mesh triangles, in each link frame."""
    result = []
    for link in description(name).links:
        surfaces = []
        for shape in link.shapes:
            if not shape.collision:
                continue
            triangles = mesh_triangles(shape.mesh) * np.asarray(shape.size)
            triangles = triangles @ rotation(shape.rpy).T + np.asarray(shape.xyz)
            surfaces.append(triangles)
        if not surfaces:
            continue
        # Cover a physical link, independent of the native importer's convex
        # partition. Covering each overlapping piece separately multiplies the
        # public planning geometry without adding useful spatial resolution.
        triangles = np.concatenate(surfaces)
        # Split along the longest dimension. Assign entire triangles, never
        # just vertices, so every surface point is enclosed by a sphere.
        axis = int(np.argmax(np.ptp(triangles.reshape(-1, 3), axis=0)))
        centers = triangles.mean(axis=1)[:, axis]
        bins = np.floor((centers - centers.min()) / 0.04).astype(int)
        for bucket in np.unique(bins):
            vertices = triangles[bins == bucket].reshape(-1, 3)
            center = (vertices.min(axis=0) + vertices.max(axis=0)) / 2
            radius = float(np.linalg.norm(vertices - center, axis=1).max()) + 1e-6
            result.append((f"{link.name}_{bucket}", link.name, center, radius))
    return tuple(result)
