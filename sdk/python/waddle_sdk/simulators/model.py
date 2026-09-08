"""One primitive geometry model exported into each simulator's native format."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from .scene import Profile, quaternion, rotation


def numbers(values: Any) -> str:
    return " ".join(str(float(x)) for x in values)


@dataclass
class Shape:
    kind: str
    size: tuple[float, ...]
    xyz: tuple[float, ...] = (0.0, 0.0, 0.0)
    rpy: tuple[float, ...] = (0.0, 0.0, 0.0)
    color: tuple[float, ...] = (0.3, 0.35, 0.4, 1.0)


@dataclass
class Link:
    name: str
    parent: str | None = None
    xyz: tuple[float, ...] = (0.0, 0.0, 0.0)
    rpy: tuple[float, ...] = (0.0, 0.0, 0.0)
    joint: str | None = None
    kind: str = "fixed"
    axis: tuple[float, ...] = (0.0, 0.0, 1.0)
    limits: tuple[float, float] = (0.0, 0.0)
    mass: float = 0.3
    shapes: list[Shape] = field(default_factory=list)


def segment(endpoint: Any, radius: float) -> list[Shape]:
    """Capsule represented by URDF-compatible cylinder plus endpoint spheres."""
    p = np.asarray(endpoint, dtype=float)
    length = float(np.linalg.norm(p))
    if length < 1e-5:
        return []
    if length < 3 * radius:
        return [Shape("sphere", (length / 4,), tuple(p / 2))]
    rpy = (0.0, math.acos(float(p[2] / length)), math.atan2(float(p[1]), float(p[0])))
    return [
        Shape("cylinder", (radius, length - 2.2 * radius), tuple(p / 2), rpy),
        Shape("sphere", (radius,), tuple(p * (1.1 * radius / length))),
        Shape("sphere", (radius,), tuple(p * (1 - 1.1 * radius / length))),
    ]


def robot_links(p: Profile) -> list[Link]:
    radius = 0.02 if p.name == "yam" else 0.025
    result = [Link("base", shapes=segment(p.origins[0], radius))]
    for i in range(p.dof):
        endpoint = (
            p.origins[i + 1] if i + 1 < p.dof else tuple(np.asarray(p.tool_xyz) * 0.48)
        )
        result.append(
            Link(
                f"link{i + 1}",
                "base" if i == 0 else f"link{i}",
                p.origins[i],
                p.rpys[i],
                p.names[i],
                "revolute",
                limits=p.limits[i],
                shapes=segment(endpoint, radius),
            )
        )
    result.append(Link("tcp", f"link{p.dof}", p.tool_xyz, p.tool_rpy, mass=0.001))
    for side, sign in (("left", 1), ("right", -1)):
        axis = np.asarray(p.closing_axis) * sign
        # Inner jaw surfaces meet at the declared pinch point at action=0.
        # Pads straddle that point so a surface-target grasp contacts the sides.
        xyz = np.asarray(p.pinch_offset) + axis * 0.004
        size = (0.008, 0.04, 0.06) if p.closing_axis[0] else (0.04, 0.008, 0.06)
        result.append(
            Link(
                f"{side}_pad",
                "tcp",
                joint=f"{side}_finger",
                kind="prismatic",
                axis=tuple(axis),
                limits=(0.0, p.opening / 2),
                mass=0.08,
                shapes=[Shape("box", size, tuple(xyz), color=(0.08, 0.08, 0.08, 1.0))],
            )
        )
    return result


def objects(environment: str) -> list[list[Link]]:
    table = [
        Link(
            "table",
            xyz=(0.35, 0.0, -0.035),
            shapes=[Shape("box", (1.2, 1.0, 0.07), color=(0.55, 0.4, 0.25, 1.0))],
        )
    ]
    if environment == "two_cubes":
        return [
            table,
            *[
                [
                    Link(
                        f"cube_{i}",
                        xyz=(0.32, y, 0.026),
                        kind="free",
                        mass=0.06,
                        shapes=[Shape("box", (0.05, 0.05, 0.05), color=color)],
                    )
                ]
                for i, y, color in (
                    (1, -0.10, (0.08, 0.8, 0.1, 1.0)),
                    (2, 0.1, (0.1, 0.35, 0.9, 1.0)),
                )
            ],
        ]
    if environment == "drawer":
        cabinet = Link(
            "cabinet",
            xyz=(0.46, 0.0, 0.0),
            shapes=[
                Shape("box", (0.3, 0.32, 0.018), (0.02, 0.0, 0.009)),
                Shape("box", (0.3, 0.018, 0.24), (0.02, -0.151, 0.12)),
                Shape("box", (0.3, 0.018, 0.24), (0.02, 0.151, 0.12)),
                Shape("box", (0.018, 0.32, 0.24), (0.161, 0.0, 0.12)),
                Shape("box", (0.3, 0.32, 0.018), (0.02, 0.0, 0.24)),
            ],
        )
        drawer = Link(
            "drawer",
            "cabinet",
            xyz=(0.0, 0.0, 0.04),
            joint="drawer_slide",
            kind="prismatic",
            axis=(-1.0, 0.0, 0.0),
            limits=(0.0, 0.22),
            mass=0.5,
            shapes=[
                Shape(
                    "box",
                    (0.26, 0.27, 0.012),
                    (0.0, 0.0, 0.01),
                    color=(0.6, 0.5, 0.4, 1.0),
                ),
                Shape("box", (0.016, 0.28, 0.16), (-0.13, 0.0, 0.075)),
                Shape(
                    "box",
                    (0.024, 0.09, 0.018),
                    (-0.157, 0.0, 0.1),
                    color=(0.85, 0.85, 0.85, 1.0),
                ),
            ],
        )
        return [table, [cabinet, drawer]]
    if environment != "bottle_cap":
        raise ValueError("unknown reference environment")
    # A fixture holds the bottle so a single arm can unscrew it. The cap has
    # passive twist and axial travel coupled by a screw constraint, never an
    # agent-accessible command that declares the task complete.
    bottle = Link(
        "bottle",
        xyz=(0.34, 0.0, 0.085),
        shapes=[
            Shape("cylinder", (0.033, 0.17), color=(0.1, 0.6, 0.35, 1.0)),
            Shape(
                "cylinder",
                (0.018, 0.035),
                (0.0, 0.0, 0.095),
                color=(0.1, 0.6, 0.35, 1.0),
            ),
        ],
    )
    twist = Link(
        "cap_twist",
        "bottle",
        xyz=(0.0, 0.0, 0.125),
        joint="cap_rotation",
        kind="revolute",
        limits=(-0.2, 6 * math.pi),
        mass=0.001,
    )
    cap = Link(
        "cap",
        "cap_twist",
        joint="cap_lift",
        kind="prismatic",
        limits=(-0.001, 0.04),
        mass=0.025,
        shapes=[Shape("cylinder", (0.025, 0.022), color=(0.9, 0.2, 0.12, 1.0))],
    )
    return [table, [bottle, twist, cap]]


SCREW_PITCH = 0.005 / (2 * math.pi)


def screw_force(q: Any, dq: Any) -> tuple[float, float]:
    # Metre/radian spring constraint; equal and opposite generalized forces
    # exchange work. Engines use this same law, including the damping term.
    theta, z = q
    omega, dz = dq
    force = -5000 * (z - SCREW_PITCH * theta) - 20 * (dz - SCREW_PITCH * omega)
    return -SCREW_PITCH * force, force


def urdf(links: list[Link], name: str) -> str:
    root = ET.Element("robot", name=name)
    for link in links:
        node = ET.SubElement(root, "link", name=link.name)
        inertial = ET.SubElement(node, "inertial")
        ET.SubElement(inertial, "mass", value=str(link.mass))
        inertia = max(link.mass * 0.003, 1e-6)
        ET.SubElement(
            inertial,
            "inertia",
            ixx=str(inertia),
            iyy=str(inertia),
            izz=str(inertia),
            ixy="0",
            ixz="0",
            iyz="0",
        )
        for i, shape in enumerate(link.shapes):
            for kind in ("visual", "collision"):
                geom = ET.SubElement(node, kind)
                ET.SubElement(
                    geom, "origin", xyz=numbers(shape.xyz), rpy=numbers(shape.rpy)
                )
                geometry = ET.SubElement(geom, "geometry")
                attrs = (
                    {"size": numbers(shape.size)}
                    if shape.kind == "box"
                    else {"radius": str(shape.size[0])}
                )
                if shape.kind == "cylinder":
                    attrs["length"] = str(shape.size[1])
                ET.SubElement(geometry, shape.kind, **attrs)
                if kind == "visual":
                    material = ET.SubElement(geom, "material", name=f"{link.name}_{i}")
                    ET.SubElement(material, "color", rgba=numbers(shape.color))
        if link.parent is not None:
            joint = ET.SubElement(
                root, "joint", name=link.joint or f"{link.name}_fixed", type=link.kind
            )
            ET.SubElement(joint, "parent", link=link.parent)
            ET.SubElement(joint, "child", link=link.name)
            ET.SubElement(joint, "origin", xyz=numbers(link.xyz), rpy=numbers(link.rpy))
            if link.joint:
                ET.SubElement(joint, "axis", xyz=numbers(link.axis))
                ET.SubElement(
                    joint,
                    "limit",
                    lower=str(link.limits[0]),
                    upper=str(link.limits[1]),
                    effort="50",
                    velocity="3",
                )
                ET.SubElement(joint, "dynamics", damping=".05", friction=".01")
    return ET.tostring(root, encoding="unicode")


def mjcf(p: Profile, config: dict) -> str:
    root = ET.Element("mujoco", model=p.name)
    ET.SubElement(root, "compiler", angle="radian", autolimits="true")
    ET.SubElement(
        root, "option", timestep=str(config["timestep"]), integrator="implicitfast"
    )
    visual = ET.SubElement(root, "visual")
    ET.SubElement(
        visual,
        "global",
        offwidth=str(max(c["stream"]["width"] for c in config["cameras"].values())),
        offheight=str(max(c["stream"]["height"] for c in config["cameras"].values())),
    )
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", pos="0 -1 2", dir="0 0 -1", diffuse=".8 .8 .8")
    bodies = {}
    contact = ET.SubElement(root, "contact")
    for group in [robot_links(p), *objects(config["environment"])]:
        for link in group:
            parent = world if link.parent is None else bodies[link.parent]
            body = ET.SubElement(
                parent,
                "body",
                name=link.name,
                pos=numbers(link.xyz),
                quat=numbers(quaternion(rotation(link.rpy))),
            )
            bodies[link.name] = body
            if group[0].name == "base":
                body.set("gravcomp", "1")
            if link.kind == "free":
                ET.SubElement(body, "freejoint")
            elif link.joint:
                ET.SubElement(
                    body,
                    "joint",
                    name=link.joint,
                    type="slide" if link.kind == "prismatic" else "hinge",
                    axis=numbers(link.axis),
                    range=numbers(link.limits),
                    damping=".1",
                    armature=".01",
                )
            ET.SubElement(
                body,
                "inertial",
                pos="0 0 0",
                mass=str(link.mass),
                diaginertia=numbers([max(link.mass * 0.003, 1e-6)] * 3),
            )
            for shape in link.shapes:
                size = (
                    tuple(np.array(shape.size) / 2)
                    if shape.kind == "box"
                    else (shape.size[0], shape.size[1] / 2)
                    if shape.kind == "cylinder"
                    else shape.size
                )
                ET.SubElement(
                    body,
                    "geom",
                    type=shape.kind,
                    size=numbers(size),
                    pos=numbers(shape.xyz),
                    quat=numbers(quaternion(rotation(shape.rpy))),
                    rgba=numbers(shape.color),
                    friction="1.2 .01 .001",
                    condim="4",
                )
            if link.parent:
                ET.SubElement(contact, "exclude", body1=link.parent, body2=link.name)
    # Fixed TCP links are fused by PhysX; explicitly match that adjacent-link
    # exclusion in MuJoCo (pads still collide with each other and with props).
    for side in ("left", "right"):
        ET.SubElement(contact, "exclude", body1=f"link{p.dof}", body2=f"{side}_pad")
    actuators = ET.SubElement(root, "actuator")
    ET.SubElement(bodies["tcp"], "site", name="tcp_site", size=".001")
    for name, limits in zip(p.names[:-1], p.limits[:-1], strict=True):
        ET.SubElement(
            actuators,
            "position",
            name=name,
            joint=name,
            kp="5000",
            kv="70",
            ctrlrange=numbers(limits),
            forcerange="-50 50",
        )
    for name in ("left_finger", "right_finger"):
        ET.SubElement(
            actuators,
            "position",
            name=name,
            joint=name,
            kp="5000",
            kv="30",
            ctrlrange=f"0 {p.opening / 2}",
            forcerange="-20 20",
        )
    for name, row in config["cameras"].items():
        t = np.asarray(row["transform"])
        gl_rotation = t[:3, :3] @ np.diag([1, -1, -1])
        intr, stream = row["intrinsics"], row["stream"]
        # MuJoCo principal pixel offsets are measured relative to image center.
        ET.SubElement(
            bodies["tcp"] if row["mount"]["kind"] == "wrist" else world,
            "camera",
            name=name,
            pos=numbers(t[:3, 3]),
            quat=numbers(quaternion(gl_rotation)),
            resolution=f"{stream['width']} {stream['height']}",
            sensorsize="1 1",
            focalpixel=f"{intr['fx']} {intr['fy']}",
            principalpixel=f"{intr['cx'] - stream['width'] / 2} {intr['cy'] - stream['height'] / 2}",
        )
    return ET.tostring(root, encoding="unicode")
