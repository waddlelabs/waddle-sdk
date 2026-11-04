"""Explicit portable MJCF selection for the reference robot runtime.

Native scene selection reuses reference actuation, sensing and supervision. It
supplies no task policy and never discovers or opens physical devices.
"""

from __future__ import annotations

import hashlib
import math
import os
import xml.etree.ElementTree as ET
from copy import copy
from pathlib import Path

import numpy as np


def replacement_signature(config, *, _contents=None, _tree=None):
    """Bind a native scene's robot assembly and its referenced model assets.

    Replacement is optional and intentionally limited to self-contained native
    XML. Included models must reopen normally. Environment bodies and their
    contact pairs may change; robot sources, drives and constraints may not.
    """
    if "native_scene" not in config:
        raise ValueError(
            "simulation.scene_contract: replacement requires native scenes"
        )
    root = (
        _tree
        if _tree is not None
        else ET.parse(resolve(config["_native_root"], config)).getroot()
        if _contents is None
        else ET.fromstring(_contents[config["native_scene"]["model"]])
    )
    if root.find(".//include") is not None:
        raise ValueError(
            "simulation.scene_contract: included native scenes require reopen"
        )
    selected = config["native_scene"]
    values = [selected["prefixes"], selected["camera_names"]]
    protected = []
    for name in ("compiler", "default", "actuator", "tendon", "equality", "extension"):
        nodes = root.findall(name)
        protected.extend(nodes)
        values.append([ET.tostring(node, encoding="unicode") for node in nodes])
    world = root.find("worldbody")
    for prefix in selected["prefixes"].values():
        body = world.find(f"body[@name='{prefix}base']")
        if body is None:
            raise ValueError(
                "simulation.scene_contract: missing reference robot assembly"
            )
        protected.append(body)
        values.append(ET.tostring(body, encoding="unicode"))

    # Tabletop props may add/remove meshes or textures. Only assets reached by
    # the preserved robot/default/drive definitions belong to its contract.
    # Follow material -> texture references as well as direct geometry assets.
    references = {"mesh", "material", "texture", "hfield", "skin"}
    assets = {(node.tag, node.get("name")): node for node in root.findall("asset/*")}
    for key, node in list(assets.items()):
        # Normalize only the comparison copy. Trusted same-call authoring may
        # supply the exact tree it serialized; never mutate that caller's tree.
        if (
            node.tag == "mesh"
            and node.get("content_type") == "model/stl"
            and node.get("file", "").endswith(".stl")
        ):
            node = copy(node)
            node.attrib = dict(node.attrib)
            node.attrib.pop("content_type")
            assets[key] = node
    pending = list(protected)
    required = {}
    while pending:
        node = pending.pop()
        for child in node.iter():
            for attribute, name in child.attrib.items():
                key = (attribute, name)
                if attribute in references and key in assets and key not in required:
                    required[key] = assets[key]
                    pending.append(assets[key])
    values.append(
        [ET.tostring(required[key], encoding="unicode") for key in sorted(required)]
    )
    compiler = root.find("compiler")
    directories = {} if compiler is None else compiler.attrib
    bound_files = {}
    for parent in [*protected, *required.values()]:
        for node in parent.iter():
            if file := node.get("file"):
                directory = directories.get(
                    {"mesh": "meshdir", "texture": "texturedir"}.get(
                        node.tag, "assetdir"
                    ),
                    directories.get("assetdir", ""),
                )
                relative = (Path(directory) / file).as_posix()
                manifested = (Path(selected["model"]).parent / relative).as_posix()
                if manifested not in selected["files"]:
                    raise ValueError(
                        "simulation.scene_contract: robot asset missing from digest manifest: "
                        + relative
                    )
                bound_files[relative] = selected["files"][manifested]
    # Byte changes to a referenced robot asset still require a fresh owner.
    values.append(bound_files)
    return values


def resolve(root, config, *, contents=None):
    """Validate confined files and optionally retain the exact verified bytes.

    A trusted consumer can compare and compile this snapshot without reopening
    mutable paths. Ordinary path-only callers retain full hash validation.
    Optional reference analysis reuse is content-bound, never file validity;
    every manifest membership and configuration check still runs on each call.
    """
    root = Path(root).resolve(strict=True)
    selected = config["native_scene"]
    if not isinstance(selected, dict) or set(selected) != {
        "model",
        "files",
        "initial_keyframe",
        "prefixes",
        "camera_names",
        "pose_groups",
    }:
        raise ValueError(
            "native_scene requires model, files, initial_keyframe, prefixes, camera_names and pose_groups"
        )
    files = selected["files"]
    if not isinstance(files, dict) or not files or len(files) > 10000:
        raise ValueError("native scene requires a bounded file digest manifest")
    directories, xml_sources = {Path("."): root}, {}
    for name, digest in files.items():
        path = Path(name)
        if path.is_absolute() or ".." in path.parts or "\\" in name or not name:
            raise ValueError("native scene asset paths must be portable")
        # Many meshes share one directory. Resolve its confinement once during
        # this pass, while checking the current leaf type and bytes every time.
        # Never retain these path resolutions across validation calls.
        directory = directories.get(path.parent)
        if directory is None:
            directory = (root / path.parent).resolve(strict=True)
            directory.relative_to(root)
            directories[path.parent] = directory
        resolved = directory / path.name
        if resolved.is_symlink() or not resolved.is_file():
            raise ValueError("native scene assets must be ordinary files")
        if contents is None and not name.endswith(".xml"):
            with resolved.open("rb") as stream:
                actual = hashlib.file_digest(stream, "sha256").hexdigest()
        else:
            data = resolved.read_bytes()
            actual = hashlib.sha256(data).hexdigest()
        if actual != digest:
            raise ValueError(f"native scene asset digest differs: {name}")
        if name.endswith(".xml"):
            xml_sources[name] = data
        if contents is not None:
            contents[name] = data
    if (
        selected["model"] not in files
        or not isinstance(selected["initial_keyframe"], str)
        or not selected["initial_keyframe"]
    ):
        raise ValueError(
            "native scene requires a manifested model and named initial keyframe"
        )
    # Validate referenced XML paths as well as the inventory. Included XML has
    # its own source directory; referenced mesh/texture directories remain local.
    from .source_references import cache, references

    for name, data in xml_sources.items():
        # Analyze the exact bytes hashed above. Only immutable XML reference
        # names can be reused; file validity and current manifest membership
        # are checked afresh, including when the same XML moves directories.
        selected_references = (
            cache.read(data, files[name])
            if os.environ.get("WADDLE_SIMULATION_MODEL_CACHE")
            else references(data)
        )
        for relative in selected_references:
            reference = (Path(name).parent / relative).as_posix()
            if reference not in files:
                raise ValueError(
                    f"native XML references an unmanifested asset: {reference}"
                )
    prefixes = selected["prefixes"]
    if (
        not isinstance(prefixes, dict)
        or set(prefixes) != set(config["parts"])
        or any(not isinstance(v, str) or not v for v in prefixes.values())
    ):
        raise ValueError("native scene needs one nonempty model prefix per robot part")
    if any(
        a != b and b.startswith(a) for a in prefixes.values() for b in prefixes.values()
    ) or len(set(prefixes.values())) != len(prefixes):
        raise ValueError("native robot prefixes must not overlap")
    names = selected["camera_names"]
    if (
        not isinstance(names, dict)
        or set(names) != set(config["cameras"])
        or any(not isinstance(v, str) or not v for v in names.values())
    ):
        raise ValueError(
            "native scene camera names must match declared camera profiles"
        )
    groups = selected["pose_groups"]
    if not isinstance(groups, list) or len(groups) > 256:
        raise ValueError("native pose groups must be a bounded list")
    seen = set()
    for group in groups:
        if not isinstance(group, dict) or set(group) != {
            "bodies",
            "translation_xy_m",
            "yaw_rad",
        }:
            raise ValueError(
                "native pose group requires bodies and explicit XY/yaw bounds"
            )
        if (
            not isinstance(group["bodies"], list)
            or not group["bodies"]
            or any(
                not isinstance(n, str) or not n or n in seen for n in group["bodies"]
            )
        ):
            raise ValueError("native pose groups require disjoint named bodies")
        if len(set(group["bodies"])) != len(group["bodies"]):
            raise ValueError("native pose group bodies must be unique")
        seen.update(group["bodies"])
        for key, upper in (("translation_xy_m", 1.0), ("yaw_rad", math.pi)):
            value = group[key]
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or not 0 <= value <= upper
            ):
                raise ValueError(
                    "native pose group bounds must be finite and nonnegative"
                )
    return root / selected["model"]


def initialize(engine):
    """Bind an imported model to the unchanged reference robot execution loop."""
    from .model import Link
    from .scene import transform

    mj, model, data, config = engine.mj, engine.model, engine.data, engine.config
    selected = config["native_scene"]
    if not np.isclose(model.opt.timestep, config["timestep"], rtol=0, atol=1e-12):
        raise ValueError("native model timestep differs from simulation configuration")
    engine._native_key = int(model.key(selected["initial_keyframe"]).id)
    mj.mj_resetDataKeyframe(model, data, engine._native_key)
    mj.mj_forward(model, data)
    robot_bodies = set()
    for part, prefix in selected["prefixes"].items():
        ids = {
            i for i in range(1, model.nbody) if model.body(i).name.startswith(prefix)
        }
        if not ids or ids & robot_bodies:
            raise ValueError("native robot body scopes overlap or are absent")
        robot_bodies.update(ids)
        expected = transform(config["parts"][part]["xyz"], config["parts"][part]["rpy"])
        base = data.body(prefix + "base")
        if not np.allclose(base.xpos, expected[:3, 3], atol=1e-6) or not np.allclose(
            base.xmat.reshape(3, 3), expected[:3, :3], atol=1e-6
        ):
            raise ValueError("native robot base differs from its declared placement")
        q = engine.read(part)[0]
        poses = engine.description.poses(q)
        for name, pose in poses.items():
            actual = data.body(prefix + name)
            world = expected @ pose
            if not np.allclose(actual.xpos, world[:3, 3], atol=2e-5) or not np.allclose(
                actual.xmat.reshape(3, 3), world[:3, :3], atol=2e-5
            ):
                raise ValueError(
                    f"native robot assembly differs from reference kinematics: {part}/{name}"
                )
    engine._native_robot_bodies = robot_bodies
    groups = []
    for index in range(1, model.nbody):
        if index in robot_bodies:
            continue
        body = model.body(index)
        if not body.name:
            raise ValueError("native environment bodies require names")
        if int(body.jntnum[0]) > 1:
            raise ValueError(
                "native environment bodies support one free or passive scalar joint"
            )
        joint = None
        kind = "fixed"
        initial = 0.0
        if int(body.jntnum[0]):
            joint_id = int(body.jntadr[0])
            joint_type = int(model.jnt_type[joint_id])
            if joint_type == mj.mjtJoint.mjJNT_FREE:
                kind = "free"
            elif joint_type in (mj.mjtJoint.mjJNT_SLIDE, mj.mjtJoint.mjJNT_HINGE):
                kind = (
                    "prismatic" if joint_type == mj.mjtJoint.mjJNT_SLIDE else "revolute"
                )
                joint = model.joint(joint_id).name
                initial = float(data.qpos[int(model.jnt_qposadr[joint_id])])
            else:
                raise ValueError("native environment ball joints are unsupported")
        groups.append([Link(body.name, kind=kind, joint=joint, initial=initial)])
    return groups


def camera_parameters(model, data, name, *, width, height, tcp_site=None):
    """Return rectified intrinsics and an optical pose from one compiled camera.

    Scene poses are world-relative; a selected TCP makes the pose TCP-relative.
    This reports configured simulator geometry, not physical calibration.
    """
    index = int(model.camera(name).id)
    sensor = model.cam_sensorsize[index]
    if np.all(sensor > 0):
        fx, fy, px, py = model.cam_intrinsic[index]
        fx, px = fx * width / sensor[0], px * width / sensor[0]
        fy, py = fy * height / sensor[1], py * height / sensor[1]
    else:
        fx = fy = (height / 2) / math.tan(math.radians(model.cam_fovy[index]) / 2)
        px = py = 0.0
    optical = np.eye(4)
    optical[:3, :3] = data.cam_xmat[index].reshape(3, 3) @ np.diag([1, -1, -1])
    optical[:3, 3] = data.cam_xpos[index]
    if tcp_site is not None:
        tcp = np.eye(4)
        tcp[:3, :3] = data.site(tcp_site).xmat.reshape(3, 3)
        tcp[:3, 3] = data.site(tcp_site).xpos
        optical = np.linalg.inv(tcp) @ optical
    return {
        "fx": float(fx),
        "fy": float(fy),
        "cx": float((width - 1) / 2 - px),
        "cy": float((height - 1) / 2 - py),
    }, optical.tolist()


def validate_cameras(engine):
    """Reject declared projection metadata that differs from the rendered model."""
    config = engine.config
    for name, row in config["cameras"].items():
        native = config["native_scene"]["camera_names"][name]
        mount = row["mount"]
        tcp = None
        if mount["kind"] == "wrist":
            tcp = config["native_scene"]["prefixes"][mount["part"]] + "tcp_site"
            if int(engine.model.cam_bodyid[engine.model.camera(native).id]) != int(
                engine.model.site_bodyid[engine.model.site(tcp).id]
            ):
                raise ValueError(
                    "native wrist camera must be rigidly mounted on the TCP body"
                )
        else:
            body = int(engine.model.cam_bodyid[engine.model.camera(native).id])
            while body:
                if engine.model.body_jntnum[body]:
                    raise ValueError(
                        "native scene camera cannot be mounted on a moving body"
                    )
                body = int(engine.model.body_parentid[body])
        intr, matrix = camera_parameters(
            engine.model,
            engine.data,
            native,
            width=row["stream"]["width"],
            height=row["stream"]["height"],
            tcp_site=tcp,
        )
        if any(
            not np.isclose(value, row["intrinsics"][key], rtol=1e-6, atol=1e-5)
            for key, value in intr.items()
        ) or not np.allclose(matrix, row["transform"], atol=1e-6):
            raise ValueError(
                f"native camera projection differs from declared profile: {name}"
            )
