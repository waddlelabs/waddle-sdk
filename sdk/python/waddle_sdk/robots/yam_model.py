"""Verified YAM source geometry and hardware relationships; no device lifecycle.

The optional pinned I2RT distribution supplies mesh bytes. No vendor Python code
is imported. This module returns source geometry, never collision approximations,
planner selections, workspace files or runtime authority.
"""

from __future__ import annotations

import base64
import hashlib
import json
from collections.abc import Mapping
from importlib import metadata
from pathlib import Path, PurePosixPath
from xml.etree import ElementTree as ET

import numpy as np

from . import yam
from .models import ModelSourceError, ModelSources

_ARM = "i2rt/robot_models/arm/yam/"
_HAND = "i2rt/robot_models/gripper/linear_4310/"


def _fail(detail):
    return ModelSourceError(detail)


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _pose(element, *, urdf=False):
    transform = np.eye(4)
    if element is None:
        return transform
    if urdf:
        r, p, y = (float(v) for v in element.get("rpy", "0 0 0").split())
        cr, cp, cy = np.cos([r, p, y])
        sr, sp, sy = np.sin([r, p, y])
        transform[:3, :3] = [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    else:
        q = np.asarray([float(v) for v in element.get("quat", "1 0 0 0").split()])
        w, x, y, z = q / np.linalg.norm(q)
        transform[:3, :3] = [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    transform[:3, 3] = [
        float(v) for v in element.get("xyz" if urdf else "pos", "0 0 0").split()
    ]
    if not np.all(np.isfinite(transform)):
        raise _fail("Source model contains a nonfinite transform")
    return transform


def _vendor():
    try:
        dist = metadata.distribution("i2rt")
    except metadata.PackageNotFoundError:
        raise _fail(
            "Install the SDK's pinned I2RT dependency before selecting YAM"
        ) from None
    origin = json.loads(dist.read_text("direct_url.json") or "{}")
    if (
        origin.get("url", "").removesuffix(".git") != yam.I2RT_REPO.removesuffix(".git")
        or origin.get("vcs_info", {}).get("commit_id") != yam.I2RT_PIN
    ):
        raise _fail("YAM model assets require the exact SDK I2RT Git pin")
    files = {str(item): item for item in dist.files or ()}
    provenance = {}

    def read(name):
        entry = files.get(name)
        if entry is None or entry.hash is None or entry.hash.mode != "sha256":
            raise _fail(
                "A required public model source lacks an installed SHA256 record"
            )
        with Path(dist.locate_file(entry)).open("rb") as handle:
            data = handle.read(32 * 1024 * 1024 + 1)
        digest = (
            base64.urlsafe_b64encode(hashlib.sha256(data).digest()).decode().rstrip("=")
        )
        if len(data) > 32 * 1024 * 1024 or digest != entry.hash.value:
            raise _fail(
                "An installed public model source differs from its package record"
            )
        provenance[name] = _sha(data)
        return data

    licenses = [name for name in files if name.endswith(".dist-info/licenses/LICENSE")]
    if len(licenses) != 1:
        raise _fail(
            "The pinned vendor license must be retained with copied model assets"
        )
    return read, provenance, read(licenses[0])


def model_sources(
    *, factory: str, part_name: str, part: Mapping
) -> ModelSources | None:
    """Read the exact installed vendor pin and verify every consumed asset hash.

    Requires the documented I2RT install only when explicitly called. Missing,
    changed or unverified selected sources raise :class:`ModelSourceError`; there
    is no network download, device construction or approximate fallback.
    """
    try:
        if factory != "arm":
            return None
        frame = part.get("base_frame", part.get("options", {}).get("base_frame"))
        options = part.get("options", {})
        if (
            not isinstance(frame, str)
            or not frame
            or options.get("base_frame", frame) != frame
        ):
            raise _fail(
                "Every YAM part must declare one consistent explicit base frame"
            )
        if "fk" in options:
            raise _fail("A custom SDK FK needs a customer-selected model")
        if part.get("gripper") != {
            "joint": yam.GRIPPER_JOINT_NAME,
            "closed_m": 0,
            "open_m": yam.GRIPPER_MAX_OPENING_M,
            "closed_action": 0,
            "open_action": 1,
        }:
            raise _fail(
                "The standard YAM source requires the declared physical SDK gripper mapping"
            )
        return _read_sources(part_name)
    except ModelSourceError:
        raise
    except (
        OSError,
        ValueError,
        TypeError,
        KeyError,
        AttributeError,
        StopIteration,
        ET.ParseError,
    ) as error:
        raise _fail("YAM source model metadata or structure is invalid") from error


def _read_sources(part_name):
    read, provenance, license_bytes = _vendor()
    urdf_bytes = yam.urdf_text().encode()
    hand_bytes = read(_HAND + "linear_4310.xml")
    urdf, hand = ET.fromstring(urdf_bytes), ET.fromstring(hand_bytes)
    equality = hand.findall("equality/joint")
    if len(equality) != 1 or equality[0].attrib != {
        "joint1": "joint7",
        "joint2": "joint8",
        "polycoef": "0 1 0 0 0",
    }:
        raise _fail("The pinned hand's measured finger coupling has changed")
    joints = {j.get("name"): j for j in urdf.findall("joint")}
    terminal = joints[yam.ARM_JOINT_NAMES[-1]].find("child").get("link")
    for name in yam.ARM_JOINT_NAMES:
        if joints[name].get("type") != "revolute":
            raise _fail("The public YAM joint chain changed")
    arm_assets = {}
    for link in urdf.findall("link"):
        if link.get("name") in (terminal, yam.URDF_TCP_FRAME):
            continue
        collisions = link.findall("collision")
        if len(collisions) != 1:
            raise _fail("The public arm collision mesh layout changed")
        source = PurePosixPath(collisions[0].find("geometry/mesh").get("filename"))
        if source.parent != PurePosixPath("assets") or ".." in source.parts:
            raise _fail("The public arm collision asset path changed")
        arm_assets[str(source)] = read(_ARM + str(source))
    hand_body = hand.find("worldbody/body")
    fingers = {}
    for finger in hand_body.findall("body"):
        joint, geom = finger.find("joint"), finger.find("geom")
        if (
            joint is None
            or joint.get("type") != "slide"
            or geom is None
            or len(finger.findall("joint")) != 1
            or len(finger.findall("geom")) != 1
        ):
            raise _fail("The pinned hand's linear finger structure changed")
        lower, upper = (float(v) for v in joint.get("range").split())
        if lower != 0 or upper != yam.GRIPPER_MAX_OPENING_M / 2:
            raise _fail("The pinned hand travel disagrees with the SDK opening")
        axis = np.asarray([float(v) for v in joint.get("axis").split()])
        axis /= np.linalg.norm(axis)
        displacement = _pose(geom)[:3, :3].T @ (axis * upper)
        if not np.all(np.isfinite(displacement)):
            raise _fail("The pinned hand slide axis is invalid")
        fingers[geom.get("mesh")] = tuple(float(v) for v in displacement)
    if set(fingers) != {"tip_left", "tip_right"}:
        raise _fail("Both complete public finger meshes are required")
    hand_assets = {}
    for mesh in hand.findall("asset/mesh"):
        source = PurePosixPath(mesh.get("file"))
        if len(source.parts) != 1 or source.name in (".", ".."):
            raise _fail("The public hand asset paths changed")
        hand_assets[str(source)] = read(_HAND + "assets/" + str(source))
    if len(arm_assets) != 6 or len(hand_assets) != 3:
        raise _fail("The complete pinned arm and hand geometry layout changed")
    hand_tcp = hand_body.find("site[@name='grasp_site']")
    sdk_tcp_joint = next(
        j for j in joints.values() if j.find("child").get("link") == yam.URDF_TCP_FRAME
    )
    if (
        hand_tcp is None
        or sdk_tcp_joint.get("type") != "fixed"
        or sdk_tcp_joint.find("parent").get("link") != terminal
    ):
        raise _fail("The SDK TCP attachment changed")
    transform = _pose(sdk_tcp_joint.find("origin"), urdf=True) @ np.linalg.inv(
        _pose(hand_tcp)
    )
    model, assets = _assemble(urdf, hand, arm_assets, hand_assets, terminal, transform)
    return ModelSources(
        format="mjcf",
        model=model,
        assets=assets,
        joint_names=tuple(yam.ARM_JOINT_NAMES),
        joint_units=("rad",) * len(yam.ARM_JOINT_NAMES),
        base_body=yam.URDF_BASE_LINK,
        tcp_site="grasp_site",
        tcp_frame=part_name + "_tool",
        licenses={"LICENSE.i2rt": license_bytes},
        provenance={
            "vendor_repository": yam.I2RT_REPO,
            "vendor_commit": yam.I2RT_PIN,
            "vendor_sources_sha256": provenance,
            "sdk_urdf_sha256": _sha(urdf_bytes),
            "hand_attachment": "T_sdk_link6_tcp @ inverse(T_vendor_hand_grasp_site)",
            "T_sdk_link6_vendor_hand": transform.tolist(),
            "geometry_scope": "Complete YAM arm and linear_4310 hand. No table, environment or other arm geometry.",
        },
    )


def _text(values):
    return " ".join(format(float(value), ".17g") for value in values)


def _urdf_pose(element):
    return _pose(element, urdf=True)


def _set_pose(element, transform):
    # Stable matrix-to-wxyz conversion, including 180-degree rotations.
    rotation = transform[:3, :3]
    values = (
        np.array(
            [
                [
                    rotation[0, 0] - rotation[1, 1] - rotation[2, 2],
                    rotation[1, 0] + rotation[0, 1],
                    rotation[2, 0] + rotation[0, 2],
                    rotation[2, 1] - rotation[1, 2],
                ],
                [
                    rotation[1, 0] + rotation[0, 1],
                    rotation[1, 1] - rotation[0, 0] - rotation[2, 2],
                    rotation[2, 1] + rotation[1, 2],
                    rotation[0, 2] - rotation[2, 0],
                ],
                [
                    rotation[2, 0] + rotation[0, 2],
                    rotation[2, 1] + rotation[1, 2],
                    rotation[2, 2] - rotation[0, 0] - rotation[1, 1],
                    rotation[1, 0] - rotation[0, 1],
                ],
                [
                    rotation[2, 1] - rotation[1, 2],
                    rotation[0, 2] - rotation[2, 0],
                    rotation[1, 0] - rotation[0, 1],
                    np.trace(rotation),
                ],
            ]
        )
        / 3
    )
    _, vectors = np.linalg.eigh(values)
    q = vectors[:, -1][[3, 0, 1, 2]]
    if q[0] < 0:
        q = -q
    element.set("pos", _text(transform[:3, 3]))
    element.set("quat", _text(q))


def _assemble(urdf, hand, arm_assets, hand_assets, terminal, transform):
    import copy

    links = {link.get("name"): link for link in urdf.findall("link")}
    joints = {joint.get("name"): joint for joint in urdf.findall("joint")}
    root = ET.Element("mujoco", model="sdk_yam_linear_4310")
    ET.SubElement(root, "compiler", angle="radian", fusestatic="false")
    assets = ET.SubElement(root, "asset")
    world = ET.SubElement(root, "worldbody")
    base = ET.SubElement(world, "body", name=yam.URDF_BASE_LINK)
    bodies = {yam.URDF_BASE_LINK: base}
    copied = {}
    for joint_name in yam.ARM_JOINT_NAMES:
        joint = joints[joint_name]
        if joint.get("type") != "revolute":
            raise _fail("The public YAM joint chain changed")
        name = joint.find("child").get("link")
        body = ET.SubElement(
            bodies[joint.find("parent").get("link")], "body", name=name
        )
        _set_pose(body, _urdf_pose(joint.find("origin")))
        limit = joint.find("limit")
        ET.SubElement(
            body,
            "joint",
            name=joint_name,
            type="hinge",
            axis=joint.find("axis").get("xyz"),
            range=f"{limit.get('lower')} {limit.get('upper')}",
        )
        bodies[name] = body
    for name, body in bodies.items():
        if name == terminal:
            continue
        link = links[name]
        inertial = link.find("inertial")
        inertia = inertial.find("inertia")
        origin = inertial.find("origin")
        if not np.allclose(_urdf_pose(origin)[:3, :3], np.eye(3), atol=1e-12):
            raise _fail("The public arm inertia frames changed")
        ET.SubElement(
            body,
            "inertial",
            pos=origin.get("xyz"),
            mass=inertial.find("mass").get("value"),
            fullinertia=" ".join(
                inertia.get(key) for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
            ),
        )
        collisions = link.findall("collision")
        if len(collisions) != 1:
            raise _fail("The public arm collision mesh layout changed")
        collision = collisions[0]
        source = PurePosixPath(collision.find("geometry/mesh").get("filename"))
        if source.parent != PurePosixPath("assets"):
            raise _fail("The public arm collision asset path changed")
        filename = "assets/arm_" + source.name
        copied[filename] = arm_assets[str(source)]
        ET.SubElement(assets, "mesh", name=name, file=filename)
        geom = ET.SubElement(
            body, "geom", name=name + "_collision", type="mesh", mesh=name
        )
        _set_pose(geom, _urdf_pose(collision.find("origin")))
    for mesh in hand.findall("asset/mesh"):
        element = copy.deepcopy(mesh)
        filename = "assets/hand_" + mesh.get("file")
        copied[filename] = hand_assets[mesh.get("file")]
        element.set("file", filename)
        assets.append(element)
    for child in hand.find("worldbody/body"):
        element = copy.deepcopy(child)
        _set_pose(element, transform @ _pose(child))
        bodies[terminal].append(element)
    # Preserve physical slide coordinates and their coupling in the source.
    root.append(copy.deepcopy(hand.find("equality")))
    for geom in root.iter("geom"):
        if geom.get("name") is None:
            geom.set("name", geom.get("mesh") + "_collision")
    ET.indent(root)
    return ET.tostring(root, encoding="utf-8", xml_declaration=True), copied
