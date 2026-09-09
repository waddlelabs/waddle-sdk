"""Manufacturer robot descriptions and reference props in native scene formats."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field, replace
from importlib.resources import files
from typing import Any

import numpy as np

from .scene import Profile, quaternion


def numbers(values: Any) -> str:
    return " ".join(str(float(x)) for x in values)


@dataclass
class Shape:
    kind: str
    size: tuple[float, ...]
    xyz: tuple[float, ...] = (0.0, 0.0, 0.0)
    rpy: tuple[float, ...] = (0.0, 0.0, 0.0)
    color: tuple[float, ...] = (0.3, 0.35, 0.4, 1.0)
    mesh: str | None = None
    visual: bool = True
    collision: bool = True
    texture: str | None = None


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
    com: tuple[float, ...] = (0.0, 0.0, 0.0)
    inertia: tuple[float, ...] | None = None  # xx, yy, zz, xy, xz, yz in link frame
    mimic: tuple[str, float, float] | None = None
    damping: float = 0.0  # passive joint resistance in SI units


def robot_links(p: Profile) -> list[Link]:
    from .description import description

    return description(p.name).native_links()


def objects(environment: str) -> list[list[Link]]:
    table = [
        Link(
            "table",
            xyz=(0.35, 0.0, -0.035),
            shapes=[
                Shape(
                    "box",
                    (1.2, 1.0, 0.07),
                    color=(1.0, 1.0, 1.0, 1.0),
                    texture=str(
                        files(__package__).joinpath(
                            "data/appearance/wood_table_001_diff_1k.png"
                        )
                    ),
                )
            ],
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
                        # Uniform 50 mm cube: I = mass * side**2 / 6.
                        inertia=(0.000025, 0.000025, 0.000025, 0.0, 0.0, 0.0),
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
            # Keep the closed front clear of the robot's starting hand. The
            # handle travels from x=.503 to .283 m as the drawer opens.
            xyz=(0.66, 0.0, 0.0),
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
            damping=5.0,
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
        # Leave the central jog region clear; the cap remains on the tabletop
        # within the robot's reach. Contacts use the normal engine solver.
        xyz=(0.34, -0.16, 0.085),
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
    carriage = Link(
        "cap_carriage",
        "bottle",
        xyz=(0.0, 0.0, 0.125),
        joint="cap_lift",
        kind="prismatic",
        limits=(-0.001, 0.04),
        mass=0.001,
    )
    cap = Link(
        "cap",
        "cap_carriage",
        joint="cap_rotation",
        kind="revolute",
        limits=(-0.2, 6 * math.pi),
        mass=0.025,
        # Rotation = axial travel / pitch. Both motions share the Z axis.
        mimic=("cap_lift", 1 / SCREW_PITCH, 0.0),
        shapes=[Shape("cylinder", (0.025, 0.022), color=(0.9, 0.2, 0.12, 1.0))],
    )
    return [table, [bottle, carriage, cap]]


SCREW_PITCH = 0.005 / (2 * math.pi)
SCREW_RESISTANCE = 0.001  # N m; reference thread resistance, not measured seal torque.


def urdf(links: list[Link], name: str) -> str:
    root = ET.Element("robot", name=name)
    for link in links:
        node = ET.SubElement(root, "link", name=link.name)
        # Standalone static scenery has no dynamic mass properties. Keep
        # inertials for every articulation link, including its fixed frames.
        if len(links) > 1 or link.kind != "fixed":
            inertial = ET.SubElement(node, "inertial")
            ET.SubElement(inertial, "origin", xyz=numbers(link.com), rpy="0 0 0")
            ET.SubElement(inertial, "mass", value=str(link.mass))
            inertia = max(link.mass * 0.003, 1e-6)
            tensor = link.inertia or (inertia, inertia, inertia, 0, 0, 0)
            ET.SubElement(
                inertial,
                "inertia",
                **dict(
                    zip(("ixx", "iyy", "izz", "ixy", "ixz", "iyz"), map(str, tensor))
                ),
            )
        for i, shape in enumerate(link.shapes):
            for kind in ("visual", "collision"):
                if not getattr(shape, kind):
                    continue
                geom = ET.SubElement(node, kind)
                ET.SubElement(
                    geom, "origin", xyz=numbers(shape.xyz), rpy=numbers(shape.rpy)
                )
                geometry = ET.SubElement(geom, "geometry")
                attrs = (
                    {"filename": shape.mesh, "scale": numbers(shape.size)}
                    if shape.mesh
                    else (
                        {"size": numbers(shape.size)}
                        if shape.kind == "box"
                        else {"radius": str(shape.size[0])}
                    )
                )
                if shape.kind == "cylinder":
                    attrs["length"] = str(shape.size[1])
                ET.SubElement(geometry, shape.kind, **attrs)
                if kind == "visual":
                    material = ET.SubElement(geom, "material", name=f"{link.name}_{i}")
                    ET.SubElement(material, "color", rgba=numbers(shape.color))
                    if shape.texture:
                        ET.SubElement(material, "texture", filename=shape.texture)
        if link.parent is not None:
            joint = ET.SubElement(
                root, "joint", name=link.joint or f"{link.name}_fixed", type=link.kind
            )
            ET.SubElement(joint, "parent", link=link.parent)
            ET.SubElement(joint, "child", link=link.name)
            ET.SubElement(joint, "origin", xyz=numbers(link.xyz), rpy=numbers(link.rpy))
            if link.joint:
                ET.SubElement(joint, "axis", xyz=numbers(link.axis))
                if link.damping:
                    ET.SubElement(
                        joint, "dynamics", damping=str(link.damping), friction="0"
                    )
                ET.SubElement(
                    joint,
                    "limit",
                    lower=str(link.limits[0]),
                    upper=str(link.limits[1]),
                    effort="50",
                    velocity="3",
                )
                if link.mimic:
                    target, multiplier, offset = link.mimic
                    ET.SubElement(
                        joint,
                        "mimic",
                        joint=target,
                        multiplier=str(multiplier),
                        offset=str(offset),
                    )
    return ET.tostring(root, encoding="unicode")


def mjcf(p: Profile, config: dict) -> str:
    from .description import description

    robot = description(p.name)
    import mujoco

    # MuJoCo's maintained URDF importer owns geometry and full inertias. Keep
    # fixed frames for the same camera/TCP names used by the other backends.
    props = objects(config["environment"])
    groups = [robot.native_links(), *props]
    master = robot.hand_names[0]
    hand_drives = [master] + [
        link.joint for link in groups[0] if link.mimic == (master, 1.0, 0.0)
    ]
    scene_links = [Link("scene_root")]
    for group in groups:
        scene_links.extend(
            [replace(group[0], parent="scene_root", kind="fixed"), *group[1:]]
        )
    source = ET.fromstring(urdf(scene_links, p.name))
    ET.SubElement(
        ET.SubElement(source, "mujoco"),
        "compiler",
        discardvisual="false",
        fusestatic="false",
        strippath="false",
    )
    native = mujoco.MjSpec.from_string(ET.tostring(source, encoding="unicode"))
    root = ET.fromstring(native.to_xml())
    # MuJoCo's documented manipulation configuration reduces soft-constraint
    # friction creep for all contacts, without changing material friction.
    ET.SubElement(
        root,
        "option",
        timestep=str(config["timestep"]),
        integrator="implicitfast",
        cone="elliptic",
        impratio="10",
        solver="Newton",
        tolerance="1e-10",
    )
    visual = ET.SubElement(root, "visual")
    samples, shadow_size = {
        "fast": (0, 1024),
        "standard": (4, 4096),
        "high": (8, 8192),
    }[config.get("render_quality", "standard")]
    ET.SubElement(
        visual, "quality", offsamples=str(samples), shadowsize=str(shadow_size)
    )
    ET.SubElement(
        visual,
        "global",
        offwidth=str(max(c["stream"]["width"] for c in config["cameras"].values())),
        offheight=str(max(c["stream"]["height"] for c in config["cameras"].values())),
    )
    world = root.find("worldbody")
    ET.SubElement(
        visual,
        "headlight",
        ambient=".18 .20 .24",
        diffuse=".25 .25 .25",
        specular=".1 .1 .1",
    )
    ET.SubElement(
        world,
        "light",
        pos="-0.5 -1 2",
        dir=".3 .4 -1",
        diffuse=".9 .85 .78",
        specular=".5 .5 .5",
    )
    ET.SubElement(
        world,
        "light",
        pos="1 1 1.5",
        dir="-.4 -.4 -1",
        diffuse=".3 .36 .45",
        castshadow="false",
    )
    asset = root.find("asset")
    ET.SubElement(
        asset,
        "texture",
        name="studio_sky",
        type="skybox",
        builtin="gradient",
        rgb1=".35 .42 .52",
        rgb2=".82 .85 .89",
        width="256",
        height="1536",
    )
    ET.SubElement(
        asset,
        "texture",
        name="table_wood",
        type="2d",
        file=props[0][0].shapes[0].texture,
    )
    ET.SubElement(
        asset,
        "material",
        name="table_finish",
        texture="table_wood",
        texrepeat="1 1",
        texuniform="true",
        specular=".25",
        shininess=".2",
    )
    for material in asset.findall("material"):
        if material.get("name") != "table_finish":
            material.set("specular", ".45")
            material.set("shininess", ".4")
    # Remove the URDF-only container so free props are native world children.
    container = world.find("body[@name='scene_root']")
    world.extend(container.findall("body"))
    world.remove(container)
    bodies = {body.get("name"): body for body in world.iter("body")}
    for link in robot.links:
        bodies[link.name].set("gravcomp", "1")
    for group in props:
        if group[0].kind == "free":
            ET.SubElement(bodies[group[0].name], "freejoint")
    if config["environment"] == "bottle_cap":
        # Native dry thread resistance in N m, above the cap's roughly
        # 0.0002 N m gravity load through the helix. This is a reference prop
        # setting, not a measured bottle seal or an active holding torque.
        bodies["cap"].find("joint").set("frictionloss", str(SCREW_RESISTANCE))
    # Collision visuals are hidden by the camera renderer; actual CAD remains.
    for geom in world.iter("geom"):
        geom.set("group", "2" if geom.get("contype") == "0" else "3")
    # Reuse Menagerie's finger-pad contact response on the manufacturer's
    # collision meshes. Default 20 ms contacts let the stiff linear hand
    # penetrate a held cube and oscillate; no material friction is increased.
    fingers = (
        ("tip_left", "tip_right")
        if p.name == "yam"
        else ("left_finger", "right_finger")
    )
    for name in fingers:
        for geom in bodies[name].findall("geom"):
            if geom.get("group") == "3":
                geom.set("solref", "0.004 1")
                geom.set("solimp", "0.95 0.99 0.001")
                geom.set("priority", "1")
    for geom in bodies["table"].findall("geom"):
        if geom.get("group") == "2":
            geom.set("material", "table_finish")
            geom.set("rgba", "1 1 1 1")
    for link in robot.links:
        if link.joint:
            bodies[link.name].find("joint").set(
                "armature",
                str(
                    robot.armature(master) / len(hand_drives)
                    if link.joint in hand_drives
                    else robot.armature(link.joint)
                ),
            )
    contact = ET.SubElement(root, "contact")
    for group in groups:
        for link in group:
            if link.parent:
                ET.SubElement(contact, "exclude", body1=link.parent, body2=link.name)
    for first, second in robot.exclusions:
        ET.SubElement(contact, "exclude", body1=first, body2=second)
    equality = ET.SubElement(root, "equality")
    for link in scene_links:
        if link.mimic:
            master, multiplier, offset = link.mimic
            coupling = ET.SubElement(
                equality,
                "joint",
                joint1=link.joint,
                joint2=master,
                polycoef=numbers((offset, multiplier, 0, 0, 0)),
            )
            if link.joint in hand_drives:
                # Match Menagerie's mechanical gripper coupling time constant.
                coupling.set("solref", "0.005 1")
    for first, anchor, second, _other in robot.closures():
        ET.SubElement(
            equality,
            "connect",
            body1=first,
            body2=second,
            anchor=numbers(anchor),
            solref="0.005 1",
        )
    # Menagerie's xArm/Robotiq pattern distributes one motor's force through a
    # fixed tendon. The mechanical equality need not transfer the entire load
    # from one jaw to the other, which otherwise shifts the pinch midpoint.
    # Keep the same total gain, force budget and reflected rotor inertia.
    tendon = ET.SubElement(ET.SubElement(root, "tendon"), "fixed", name="hand_motor")
    for name in hand_drives:
        ET.SubElement(tendon, "joint", joint=name, coef=str(1 / len(hand_drives)))
    actuators = ET.SubElement(root, "actuator")
    ET.SubElement(bodies["tcp"], "site", name="tcp_site", size=".001")
    for name, limits in zip(p.names[:-1], p.limits[:-1], strict=True):
        kp, kd, effort = robot.servo(name)
        ET.SubElement(
            actuators,
            "position",
            name=name,
            joint=name,
            kp=str(kp),
            kv=str(kd),
            ctrlrange=numbers(limits),
            forcerange=numbers((-effort, effort)),
        )
    for name, limits in zip(robot.hand_names[:1], robot.hand_limits[:1], strict=True):
        kp, kd, effort = robot.servo(name)
        ET.SubElement(
            actuators,
            "position",
            name=name,
            tendon="hand_motor",
            kp=str(kp),
            kv=str(kd),
            ctrlrange=numbers(limits),
            forcerange=numbers((-effort, effort)),
        )
    for name, row in config["cameras"].items():
        t = np.asarray(row["transform"])
        gl_rotation = t[:3, :3] @ np.diag([1, -1, -1])
        intr, stream = row["intrinsics"], row["stream"]
        # MuJoCo shifts the projection frustum, so its offsets have the
        # opposite sign to principal points in the returned optical image.
        # Optical pixel centers are integer coordinates; the first OpenGL
        # raster sample is half a pixel from the edge of the viewport.
        ET.SubElement(
            bodies["tcp"] if row["mount"]["kind"] == "wrist" else world,
            "camera",
            name=name,
            pos=numbers(t[:3, 3]),
            quat=numbers(quaternion(gl_rotation)),
            resolution=f"{stream['width']} {stream['height']}",
            sensorsize="1 1",
            focalpixel=f"{intr['fx']} {intr['fy']}",
            principalpixel=numbers(
                (
                    (stream["width"] - 1) / 2 - intr["cx"],
                    (stream["height"] - 1) / 2 - intr["cy"],
                )
            ),
        )
    return ET.tostring(root, encoding="unicode")
