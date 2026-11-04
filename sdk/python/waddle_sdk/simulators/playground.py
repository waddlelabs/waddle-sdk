"""Portable tabletop scene authoring for the reference simulation runtime.

Compilation is offline. The returned native scene uses the ordinary robot,
actuators, cameras and reset contract; no device is opened here.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import xml.etree.ElementTree as ET
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .model import mjcf, numbers
from .playground_bowls import bowl_geometry
from .playground_fixtures import FIXTURES, place_fixture, validate_fixture
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


def _object(world, asset, obj, easy_grasp):
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
    elif kind == "bowl":
        visual, shapes = bowl_geometry(asset, x, 2 * z, name)
        ET.SubElement(
            body,
            "geom",
            name=name + "_visual",
            type="mesh",
            mesh=visual,
            rgba=numbers([*obj["color"], 1]),
            group="2",
            contype="0",
            conaffinity="0",
            mass="0",
        )
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
    names = []
    mass = CATALOG[kind]["mass"]
    for i, shape in enumerate(shapes):
        geom_name = f"{name}_geom_{i}"
        attrs = {
            "name": geom_name,
            "type": shape[0],
            **(
                {"mesh": shape[1]}
                if shape[0] == "mesh"
                else {"size": numbers(shape[1])}
            ),
            "pos": numbers(shape[2]),
            "rgba": numbers([*obj["color"], 1]),
            "friction": "0.8 0.001 0.0001" if easy_grasp else "1 0.005 0.0001",
            "condim": "4" if easy_grasp else "3",
            "margin": "0.000001",
            "solref": "0.004 1",
            "solimp": "0.95 0.99 0.001",
        }
        if kind == "bowl":
            attrs["group"] = "3"
        if mass:
            attrs["mass"] = str(mass / len(shapes))
            names.append(geom_name)
        else:
            attrs.update(contype="0", conaffinity="0", group="2")
        if len(shape) == 4:
            attrs["quat"] = numbers(shape[3])
        ET.SubElement(body, "geom", **attrs)
    return names


def _reference(config, scene, easy_grasp):
    fixture = scene.get("fixture")
    environment = FIXTURES[fixture] if fixture else None
    root = ET.fromstring(mjcf(profile(config["robot"]), config))
    # Compound props and two articulated hands can exceed MuJoCo's inferred
    # constraint arena during ordinary contact. Allocate bounded scratch up
    # front instead of dropping constraints under load. Explicit reference
    # size declarations remain authoritative; dynamics/contact settings do not
    # change and the new bytes enter the ordinary immutable scene identity.
    if root.find("size") is None:
        ET.SubElement(root, "size", memory="256M")
    world = root.find("worldbody")
    single = len(config["parts"]) == 1
    bases = {"base"} if single else {f"{p}__base" for p in config["parts"]}
    removed = set()
    for body in list(world.findall("body")):
        if not environment and body.get("name") not in bases | {"table"}:
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
        # Reference benchmark MJCF may leave free joints unnamed. Authored scenes
        # need stable coordinate identities for consumers of native state.
        for body in world.iter("body"):
            for joint in body.findall("freejoint"):
                if not joint.get("name"):
                    joint.set("name", body.get("name") + "_free")
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
        bases = {part + "__base"}
    if root.find("asset") is None:
        ET.SubElement(root, "asset")
    props, fingers = [], []
    if fixture:
        for body in world.findall("body"):
            if body.get("name") in bases | {"table"} or body.get("name", "").startswith(
                "prop_"
            ):
                continue
            for index, geom in enumerate(body.iter("geom")):
                if geom.get("group") != "3":
                    continue
                # Imported reference collision shapes may be unnamed. Give each
                # one a stable identity before selecting the same finger contact
                # material used by authored props, including compound children.
                name = geom.get("name")
                if not name:
                    name = f"{body.get('name')}__playground_contact_{index}"
                    geom.set("name", name)
                props.append(name)
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
    return root, fingers, props


def _pairs(root, fingers, props):
    # Repeated finger contacts share source defaults, not native contact pairs.
    # Each compiled pair keeps its original parameters and geometry identities.
    defaults = root.find("default")
    if defaults is None:
        defaults = ET.SubElement(root, "default")
    material = ET.SubElement(defaults, "default", {"class": "playground_contact"})
    ET.SubElement(
        material,
        "pair",
        condim="4",
        friction="0.8 0.8 0.001 0.0001 0.0001",
        margin="0.000001",
        solref="0.004 1",
        solimp="0.95 0.99 0.001",
    )
    contact = root.find("contact")
    if os.environ.get("WADDLE_SIMULATION_MODEL_CACHE"):
        from .playground_contacts import cache

        fragment = cache.prepare(
            contact, fingers, props, _contact_pairs, _incremental=True
        )
        root[list(root).index(contact)] = fragment.node
        return fragment
    _contact_pairs(contact, fingers, props)
    return None


def _contact_pairs(contact, fingers, props):
    existing_pairs = {
        frozenset((pair.get("geom1"), pair.get("geom2")))
        for pair in contact.findall("pair")
    }
    for finger in fingers:
        for prop in props:
            pair_key = frozenset((finger, prop))
            if pair_key in existing_pairs:
                continue
            existing_pairs.add(pair_key)
            ET.SubElement(
                contact,
                "pair",
                geom1=finger,
                geom2=prop,
                **{"class": "playground_contact"},
            )


@dataclass(frozen=True)
class ScenePreparation:
    """Fresh authoring model and the exact portable bytes it describes.

    The caller owns the mutable native model. This carries no running state or
    authority; independent runtime owners must instantiate their own model/data.
    """

    config: dict
    model: Any
    contents: dict[str, bytes]


def compile_scene(config, scene, output: Path, *, revision=1, easy_grasp=True):
    """Compose a fresh immutable native scene from a reference robot configuration.

    Dimensions are full extents in metres; object rotation is XYZ degrees.
    Robot force limits and actuators are preserved. Easy grasp changes only
    contact material/solver parameters, never object attachment or outcomes.
    """
    return prepare_scene(
        config, scene, output, revision=revision, easy_grasp=easy_grasp
    ).config


def prepare_scene(
    config, scene, output: Path, *, revision=1, easy_grasp=True, _prefer_handoff=False
):
    """Compile once and retain the exact model for trusted authoring consumers.

    The written scene has the same reset, geometry and reference robot contracts
    as :func:`compile_scene`. Source assets are captured into immutable bytes;
    the final reset key is written into both the native model and portable XML.
    A resident consumer can prefer a sealed handoff over exporting a second disk
    binary; unavailable handoff storage retains the ordinary disk-cache fallback.
    """
    scene = validate_scene(scene)
    fixture = scene.get("fixture")
    if fixture:
        validate_fixture(fixture, config["robot"], len(config["parts"]))
    if output.exists():
        raise ValueError("playground.output: scene revision already exists")
    output.mkdir(parents=True)
    config = deepcopy(config)
    config.pop("native_scene", None)
    config.pop("_native_root", None)
    config.pop("pose_profile", None)
    config["environment"] = (FIXTURES[fixture] if fixture else None) or "two_cubes"
    from . import playground_source

    if os.environ.get("WADDLE_SIMULATION_MODEL_CACHE"):
        from dataclasses import asdict

        from . import model
        from .description import description

        key = (
            mjcf,
            model.objects,
            _reference,
            _object,
            bowl_geometry,
            place_fixture,
            hashlib.sha256(
                json.dumps(
                    (
                        config,
                        asdict(profile(config["robot"])),
                        asdict(description(config["robot"])),
                        CATALOG,
                        fixture,
                        scene.get("seed", 0),
                        easy_grasp,
                    ),
                    sort_keys=True,
                    allow_nan=False,
                ).encode()
            ).digest(),
        )
        root, captured, fingers, props = playground_source.cache.prepare(
            key,
            lambda: _reference(config, scene, easy_grasp),
            _object,
            scene["objects"],
            easy_grasp,
        )
    else:
        root, fingers, fixture_props = _reference(config, scene, easy_grasp)
        captured = playground_source.capture(root)
        props = [
            name
            for obj in scene["objects"]
            for name in _object(
                root.find("worldbody"), root.find("asset"), obj, easy_grasp
            )
        ] + fixture_props
    contacts = _pairs(root, fingers, props) if easy_grasp else None
    from .playground_contacts import scene_xml

    contents, hashes = {}, {}
    sources = {}
    for source, (data, digest) in captured.items():
        relative = "assets/" + digest + source.suffix
        target = output / relative
        target.parent.mkdir(exist_ok=True)
        target.write_bytes(data)
        contents[relative], hashes[relative] = data, digest
        sources[source] = relative
    for node in root.iter():
        if name := node.get("file"):
            node.set("file", sources[Path(name)])
    keys = root.find("keyframe")
    if keys is not None:
        root.remove(keys)
    # Reference generators put their initial robot coordinates in joint refs.
    # Save the exact compiled default state as the canonical reset keyframe.
    # Reserve the key before compilation so setting its arrays below does not
    # require rebuilding topology or loading all robot assets a second time.
    initial = ET.SubElement(ET.SubElement(root, "keyframe"), "key", name="initial")
    model_path = output / "world.xml"
    contents["world.xml"] = scene_xml(root, contacts)
    import mujoco as mj

    from .compiled_model import load_model

    # Only the final scene belongs in the shared private binary cache.
    model = load_model(
        "world.xml",
        contents,
        cache="",
        _authored=True,
        _authored_tree=root,
        _authored_contacts=contacts,
    )
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
    initial.set("qpos", numbers(data.qpos))
    initial.set("ctrl", numbers(data.ctrl))
    key = model.key("initial").id
    model.key_qpos[key] = data.qpos
    model.key_ctrl[key] = data.ctrl
    contents["world.xml"] = scene_xml(root, contacts)
    hashes["world.xml"] = hashlib.sha256(contents["world.xml"]).hexdigest()
    model_path.write_bytes(contents["world.xml"])
    config.update(environment="playground", scene_revision=f"1.0.{revision}")
    config["native_scene"] = {
        "model": "world.xml",
        "initial_keyframe": "initial",
        "files": hashes,
        "prefixes": {part: part + "__" for part in config["parts"]},
        "camera_names": {name: name for name in config["cameras"]},
        "pose_groups": [],
    }
    (output / "simulation.json").write_text(json.dumps(config, indent=2) + "\n")
    (output / "scene.json").write_text(json.dumps(scene, indent=2) + "\n")
    if selected_cache := os.environ.get("WADDLE_SIMULATION_MODEL_CACHE"):
        if not Path(selected_cache).is_absolute():
            raise ValueError(
                "simulation.model_cache: select an absolute private directory"
            )
        from .prepared_model import cache
        from .source_references import cache as references

        # This is the exact tree just serialized above. Retain only its immutable
        # reference list; final publication still hashes every written file and
        # checks current confinement and manifest membership normally.
        references.read(contents["world.xml"], hashes["world.xml"], _tree=root)
        retained = cache.remember("world.xml", hashes, contents["world.xml"], model)
        if not (_prefer_handoff and retained):
            model = load_model("world.xml", contents, _prepared=model)
    return ScenePreparation(config, model, contents)
