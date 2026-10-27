"""Portable tabletop scene authoring for the reference simulation runtime.

Compilation is offline. The returned native scene uses the ordinary robot,
actuators, cameras and reset contract; no device is opened here.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
import shutil
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path

from .model import mjcf, numbers
from .playground_fixtures import FIXTURES, place_fixture
from .scene import profile, quaternion, rotation

CATALOG = {
    "block": {"label": "Block", "size": [0.04, 0.04, 0.04], "mass": 0.06},
    "bottle": {"label": "Bottle", "size": [0.04, 0.04, 0.09], "mass": 0.08},
    "ball": {"label": "Ball", "size": [0.04, 0.04, 0.04], "mass": 0.04},
    "cylinder": {"label": "Cylinder", "size": [0.04, 0.04, 0.06], "mass": 0.06},
    "bowl": {"label": "Bowl", "size": [0.14, 0.14, 0.06], "mass": 0.15},
    "tray": {"label": "Tray", "size": [0.18, 0.14, 0.04], "mass": 0.18},
    "target": {"label": "Target", "size": [0.10, 0.10, 0.001], "mass": 0},
}
IDENTIFIER = re.compile(r"^[A-Za-z][A-Za-z0-9_]{0,47}$")


def _vector(value, length, low, high, name):
    if (
        not isinstance(value, list)
        or len(value) != length
        or any(
            isinstance(x, bool)
            or not isinstance(x, (int, float))
            or not math.isfinite(x)
            or not low <= x <= high
            for x in value
        )
    ):
        raise ValueError(
            f"playground.{name}: expected {length} finite values in [{low}, {high}]"
        )
    return value


def validate_scene(scene):
    """Validate bounded metric scene data; never accept executable XML or paths."""
    required = {"id", "name", "objects"}
    if (
        not isinstance(scene, dict)
        or not required <= set(scene)
        or set(scene) - required - {"fixture", "seed", "instruction"}
    ):
        raise ValueError("playground.scene: expected id, name and objects")
    if "fixture" in scene and (
        not isinstance(scene["fixture"], str) or scene["fixture"] not in FIXTURES
    ):
        raise ValueError("playground.fixture: unknown reference assembly")
    if "seed" in scene and (
        "fixture" not in scene
        or type(scene["seed"]) is not int
        or not 0 <= scene["seed"] <= 999999
    ):
        raise ValueError("playground.seed: reference fixture seed must be 0..999999")
    if "instruction" in scene and (
        not isinstance(scene["instruction"], str)
        or not 1 <= len(scene["instruction"]) <= 1000
    ):
        raise ValueError("playground.instruction: expected 1 to 1000 characters")
    if not isinstance(scene["id"], str) or not IDENTIFIER.fullmatch(scene["id"]):
        raise ValueError("playground.scene_id: invalid scene identifier")
    if not isinstance(scene["name"], str) or not 1 <= len(scene["name"]) <= 80:
        raise ValueError("playground.name: expected 1 to 80 characters")
    if not isinstance(scene["objects"], list) or len(scene["objects"]) > 64:
        raise ValueError("playground.objects: at most 64 objects are supported")
    names = set()
    for obj in scene["objects"]:
        if not isinstance(obj, dict) or set(obj) != {
            "id",
            "kind",
            "position",
            "rotation",
            "size",
            "color",
        }:
            raise ValueError(
                "playground.object: expected id, kind, position, rotation, size and color"
            )
        name = obj["id"]
        if not isinstance(name, str) or not IDENTIFIER.fullmatch(name) or name in names:
            raise ValueError(
                "playground.object_id: object names must be unique identifiers"
            )
        names.add(name)
        if obj["kind"] not in CATALOG:
            raise ValueError("playground.kind: unknown object")
        position = _vector(obj["position"], 3, -0.8, 0.8, "position")
        if position[2] < 0:
            raise ValueError(
                "playground.position: object centers must be above the table"
            )
        _vector(obj["rotation"], 3, -360, 360, "rotation")
        size = _vector(obj["size"], 3, 0.001, 0.4, "size")
        if (
            obj["kind"] in {"bottle", "cylinder", "ball", "bowl", "target"}
            and size[0] != size[1]
        ):
            raise ValueError(
                "playground.size: round objects require equal X/Y dimensions"
            )
        if obj["kind"] == "ball" and size[1] != size[2]:
            raise ValueError("playground.size: balls require equal dimensions")
        _vector(obj["color"], 3, 0, 1, "color")
    return deepcopy(scene)


def _object(world, obj, easy_grasp):
    name = "prop_" + obj["id"]
    kind = obj["kind"]
    body = ET.SubElement(
        world,
        "body",
        name=name,
        pos=numbers(obj["position"]),
        quat=numbers(quaternion(rotation([math.radians(x) for x in obj["rotation"]]))),
    )
    if kind != "target":
        ET.SubElement(body, "freejoint", name=name + "_free")
    x, y, z = (v / 2 for v in obj["size"])
    shapes = []
    if kind == "block":
        shapes.append(("box", (x, y, z), (0, 0, 0)))
    elif kind in {"bottle", "cylinder", "target"}:
        shapes.append(("cylinder", (x, z), (0, 0, 0)))
    elif kind == "ball":
        shapes.append(("sphere", (x,), (0, 0, 0)))
    else:
        wall = min(0.004, x / 4, y / 4, z / 2)
        if kind == "tray":
            shapes.append(("box", (x, y, wall / 2), (0, 0, -z + wall / 2)))
            shapes.extend(
                [
                    ("box", (x, wall / 2, z), (0, y - wall / 2, 0)),
                    ("box", (x, wall / 2, z), (0, -y + wall / 2, 0)),
                    ("box", (wall / 2, y, z), (x - wall / 2, 0, 0)),
                    ("box", (wall / 2, y, z), (-x + wall / 2, 0, 0)),
                ]
            )
        else:
            shapes.append(("cylinder", (x, wall / 2), (0, 0, -z + wall / 2)))
            # A convex segment ring gives a genuinely open container.
            for i in range(16):
                angle = 2 * math.pi * i / 16
                shapes.append(
                    (
                        "box",
                        (wall / 2, x * math.tan(math.pi / 16), z),
                        (
                            (x - wall / 2) * math.cos(angle),
                            (x - wall / 2) * math.sin(angle),
                            0,
                        ),
                        angle,
                    )
                )
    names = []
    mass = CATALOG[kind]["mass"]
    for i, shape in enumerate(shapes):
        geom_name = f"{name}_geom_{i}"
        attrs = {
            "name": geom_name,
            "type": shape[0],
            "size": numbers(shape[1]),
            "pos": numbers(shape[2]),
            "rgba": numbers([*obj["color"], 1]),
            "friction": "0.8 0.001 0.0001" if easy_grasp else "1 0.005 0.0001",
            "condim": "4" if easy_grasp else "3",
            "margin": "0.000001",
            "solref": "0.004 1",
            "solimp": "0.95 0.99 0.001",
        }
        if mass:
            attrs["mass"] = str(mass / len(shapes))
            names.append(geom_name)
        else:
            attrs.update(contype="0", conaffinity="0", group="2")
        if len(shape) == 4:
            attrs["quat"] = numbers(quaternion(rotation([0, 0, shape[3]])))
        ET.SubElement(body, "geom", **attrs)
    return names


def compile_scene(config, scene, output: Path, *, revision=1, easy_grasp=True):
    """Compose a fresh immutable native scene from a reference robot configuration.

    Dimensions are full extents in metres; object rotation is XYZ degrees.
    Robot force limits and actuators are preserved. Easy grasp changes only
    contact material/solver parameters, never object attachment or outcomes.
    """
    scene = validate_scene(scene)
    fixture = scene.get("fixture")
    if fixture and (len(config["parts"]) != 2 or config["robot"] != "yam"):
        raise ValueError(
            "playground.fixture: these reference assemblies require two YAMs"
        )
    if output.exists():
        raise ValueError("playground.output: scene revision already exists")
    output.mkdir(parents=True)
    config = deepcopy(config)
    config.pop("native_scene", None)
    config.pop("_native_root", None)
    config.pop("pose_profile", None)
    config["environment"] = FIXTURES[fixture] if fixture else "two_cubes"
    root = ET.fromstring(mjcf(profile(config["robot"]), config))
    world = root.find("worldbody")
    single = len(config["parts"]) == 1
    bases = {"base"} if single else {f"{p}__base" for p in config["parts"]}
    removed = set()
    for body in list(world.findall("body")):
        if not fixture and body.get("name") not in bases | {"table"}:
            removed.update(n.get("name") for n in body.iter() if n.get("name"))
            world.remove(body)
    contact = root.find("contact")
    if contact is None:
        contact = ET.SubElement(root, "contact")
    for node in list(contact):
        if any(value in removed for value in node.attrib.values()):
            contact.remove(node)
    if fixture:
        place_fixture(world, fixture, scene.get("seed", 0))
    if single:
        part = next(iter(config["parts"]))
        robot = world.find("body[@name='base']")
        names = {
            n.get("name") for n in robot.iter() if n.get("name") and n.tag != "camera"
        }
        for section in (root.find("actuator"), root.find("tendon")):
            if section is not None:
                names.update(n.get("name") for n in section if n.get("name"))
        for node in root.iter():
            for key, value in list(node.attrib.items()):
                if value in names:
                    node.set(key, part + "__" + value)
    for node in root.iter():
        if node.get("file"):
            source = Path(node.get("file"))
            digest = hashlib.sha256(source.read_bytes()).hexdigest()
            target = output / "assets" / (digest + source.suffix)
            target.parent.mkdir(exist_ok=True)
            shutil.copyfile(source, target)
            node.set("file", target.relative_to(output).as_posix())
    props = [
        name for obj in scene["objects"] for name in _object(world, obj, easy_grasp)
    ]
    if fixture:
        props.extend(
            geom.get("name")
            for body in world.findall("body")
            if body.get("name") not in bases | {"table"}
            and not body.get("name", "").startswith("prop_")
            for geom in body.iter("geom")
            if geom.get("name") and geom.get("group") == "3"
        )
    if easy_grasp:
        root.find("option").set("noslip_iterations", "3")
        tips = {
            "yam": ("tip_left", "tip_right"),
            "so101": ("gripper_link", "moving_jaw_so101_v1_link"),
            "xarm7": ("left_finger", "right_finger"),
        }[config["robot"]]
        fingers = []
        for body in world.iter("body"):
            if any(body.get("name", "").endswith("__" + tip) for tip in tips):
                for i, geom in enumerate(body.findall("geom")):
                    if geom.get("group") == "3":
                        name = body.get("name") + f"__playground_contact_{i}"
                        geom.set("name", name)
                        fingers.append(name)
        for finger in fingers:
            for prop in props:
                if any(
                    {pair.get("geom1"), pair.get("geom2")} == {finger, prop}
                    for pair in contact.findall("pair")
                ):
                    continue
                ET.SubElement(
                    contact,
                    "pair",
                    geom1=finger,
                    geom2=prop,
                    condim="4",
                    # A small finite patch stabilizes ordinary grasps without
                    # making marginal edge contacts resist arbitrary torque.
                    friction="0.8 0.8 0.001 0.0001 0.0001",
                    margin="0.000001",
                    solref="0.004 1",
                    solimp="0.95 0.99 0.001",
                )
    keys = root.find("keyframe")
    if keys is not None:
        root.remove(keys)
    # Reference generators put their initial robot coordinates in joint refs.
    # Save the exact compiled default state as the canonical reset keyframe.
    model_path = output / "world.xml"
    model_path.write_text(ET.tostring(root, encoding="unicode"))
    import mujoco as mj

    model = mj.MjModel.from_xml_path(str(model_path))
    data = mj.MjData(model)
    from .description import description

    robot = description(config["robot"])
    p = profile(config["robot"])
    for part in config["parts"]:
        q = p.home
        for name, value in zip(
            p.names[:-1] + robot.hand_names, robot.expand(q), strict=True
        ):
            data.qpos[model.joint(part + "__" + name).qposadr] = value
        for name in p.names[:-1] + robot.hand_names[:1]:
            actuator = model.actuator(part + "__" + name)
            data.ctrl[actuator.id] = data.qpos[model.joint(part + "__" + name).qposadr][
                0
            ]
    ET.SubElement(
        ET.SubElement(root, "keyframe"),
        "key",
        name="initial",
        qpos=numbers(data.qpos),
        ctrl=numbers(data.ctrl),
    )
    model_path.write_text(ET.tostring(root, encoding="unicode"))
    config.update(environment="playground", scene_revision=f"1.0.{revision}")
    config["native_scene"] = {
        "model": "world.xml",
        "initial_keyframe": "initial",
        "files": {
            p.relative_to(output).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in [model_path, *sorted((output / "assets").glob("*"))]
        },
        "prefixes": {part: part + "__" for part in config["parts"]},
        "camera_names": {name: name for name in config["cameras"]},
        "pose_groups": [],
    }
    (output / "simulation.json").write_text(json.dumps(config, indent=2) + "\n")
    (output / "scene.json").write_text(json.dumps(scene, indent=2) + "\n")
    return config
