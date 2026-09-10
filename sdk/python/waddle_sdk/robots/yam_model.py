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
from dataclasses import dataclass
from importlib import metadata
from pathlib import Path, PurePosixPath
from types import MappingProxyType
from xml.etree import ElementTree as ET

import numpy as np

from . import yam

_ARM = "i2rt/robot_models/arm/yam/"
_HAND = "i2rt/robot_models/gripper/linear_4310/"


class ModelSourceError(ValueError):
    """Selected source geometry is unavailable or fails provenance/structure checks."""

    code = "model_sources_invalid"


def _fail(detail):
    return ModelSourceError(detail)


def _sha(data):
    return hashlib.sha256(data).hexdigest()


@dataclass(frozen=True)
class YamModelSources:
    """Immutable, verified arm/hand source bytes and their physical relationships.

    Matrices are homogeneous transforms in metres, using column vectors.
    ``hand_attachment`` maps the vendor hand root into the terminal arm link.
    ``finger_displacement_mesh_m`` gives each coupled finger's complete slide in
    its mesh coordinates. It describes travel, not a collision approximation.
    The vendor arm URDF lacks its terminal link mesh: ``arm_assets`` deliberately
    omits it; the complete hand replaces that link's geometry at the given transform.
    Asset keys are the relative paths referenced by the respective source XML.
    """

    arm_urdf: bytes
    hand_mjcf: bytes
    arm_assets: Mapping[str, bytes]
    hand_assets: Mapping[str, bytes]
    joint_names: tuple[str, ...]
    base_link: str
    terminal_link: str
    tcp_site: str
    hand_attachment: tuple[tuple[float, ...], ...]
    finger_displacement_mesh_m: Mapping[str, tuple[float, ...]]
    vendor_repository: str
    vendor_commit: str
    vendor_sources_sha256: Mapping[str, str]
    license_bytes: bytes


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


def model_sources() -> YamModelSources:
    """Read the exact installed vendor pin and verify every consumed asset hash.

    Requires the documented I2RT install only when explicitly called. Missing,
    changed or unverified selected sources raise :class:`ModelSourceError`; there
    is no network download, device construction or approximate fallback.
    """
    try:
        return _read_sources()
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


def _read_sources():
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
    return YamModelSources(
        urdf_bytes,
        hand_bytes,
        MappingProxyType(arm_assets),
        MappingProxyType(hand_assets),
        tuple(yam.ARM_JOINT_NAMES),
        yam.URDF_BASE_LINK,
        terminal,
        "grasp_site",
        tuple(tuple(float(v) for v in row) for row in transform),
        MappingProxyType(fingers),
        yam.I2RT_REPO,
        yam.I2RT_PIN,
        MappingProxyType(provenance),
        license_bytes,
    )
