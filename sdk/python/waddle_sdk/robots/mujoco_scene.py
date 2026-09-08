"""Compile portable Waddle scenes into self-contained MuJoCo bundles."""

from __future__ import annotations

import hashlib
import json
import math
import shutil
import xml.etree.ElementTree as ET
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from ..scene import (
    SceneArtifacts,
    SceneCompilerError,
    SceneConfig,
    ScenePathError,
    SceneValidationError,
)
from .mujoco import _mujoco_module


@dataclass(frozen=True)
class _Joint:
    name: str
    lower: float
    upper: float
    home: float
    velocity: float
    effort: float
    kp: float


def _vector(
    value: object | None, default: Sequence[float], *, width: int
) -> list[float]:
    selected = default if value is None else value
    values = [float(item) for item in selected]  # type: ignore[union-attr]
    if len(values) != width or not np.all(np.isfinite(values)):
        raise SceneValidationError(f"expected {width} finite numeric values")
    return values


def _pose(row: Mapping[str, Any] | None) -> tuple[list[float], list[float]]:
    value = {} if row is None else row
    position = _vector(value.get("position_m"), (0.0, 0.0, 0.0), width=3)
    quaternion = _vector(value.get("quaternion_wxyz"), (1.0, 0.0, 0.0, 0.0), width=4)
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12:
        raise SceneValidationError("a pose quaternion must be non-zero")
    return position, [value / norm for value in quaternion]


def _pose_matrix(position: Sequence[float], quaternion: Sequence[float]) -> np.ndarray:
    """Return ``parent_from_child`` for one normalized portable wxyz pose."""

    w, x, y, z = (float(value) for value in quaternion)
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    matrix[:3, 3] = np.asarray(position, dtype=np.float64)
    return matrix


def _simulation_calibration(
    *,
    config: SceneConfig,
    camera: str,
    frame_id: str,
    to_frame: str,
    matrix: np.ndarray,
    part: str | None = None,
) -> dict[str, Any]:
    """Build one deterministic Metal-compatible privileged calibration record."""

    artifact: dict[str, Any] = {
        "schema": "waddle.simulation-calibration/v1",
        "camera": camera,
        "from_frame": frame_id,
        "to_frame": to_frame,
        "matrix": np.asarray(matrix, dtype=np.float64).tolist(),
        "rms_mm": 0.0,
        "n_pairs": 0,
        "t_calibrated_unix": 0.0,
        "valid": True,
        "point_pairs_from": [],
        "point_pairs_to": [],
        "rms_unit": "mm",
        "rms_stat": "simulation_ground_truth",
        "provenance": {
            "kind": "simulation_ground_truth",
            "scene_api_version": config.document["api_version"],
            "scene_id": config.name,
            "seed": config.seed,
            "privileged": True,
        },
    }
    if part is not None:
        artifact["part"] = part
    return artifact


def _multiply_quaternion(left: Sequence[float], right: Sequence[float]) -> list[float]:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return [
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    ]


def _portable_path(root: Path, value: object, *, field: str) -> Path:
    if not isinstance(value, str) or not value or "\x00" in value or "\\" in value:
        raise ScenePathError(f"{field} must be a portable relative path")
    relative = Path(value)
    if relative.is_absolute() or any(part == ".." for part in relative.parts):
        raise ScenePathError(f"{field} must stay beneath the scene directory")
    resolved_root = root.resolve()
    resolved = (resolved_root / relative).resolve(strict=False)
    try:
        resolved.relative_to(resolved_root)
    except ValueError as exc:
        raise ScenePathError(f"{field} escapes the scene directory") from exc
    return resolved


def _resolve_urdf_asset(
    value: str,
    *,
    urdf: Path,
    scene_root: Path,
    packages: Mapping[str, object],
    field: str,
) -> Path:
    if "\x00" in value or "\\" in value:
        raise ScenePathError(f"{field} is not a portable asset path")
    if value.startswith("package://"):
        package_path = value[len("package://") :]
        package, separator, remainder = package_path.partition("/")
        if not separator or package not in packages:
            raise ScenePathError(
                f"{field} names package {package!r}; declare packages.{package}"
            )
        package_root = _portable_path(
            scene_root, packages[package], field=f"packages.{package}"
        )
        candidate = (package_root / remainder).resolve(strict=False)
        try:
            candidate.relative_to(package_root)
        except ValueError as exc:
            raise ScenePathError(f"{field} escapes package {package!r}") from exc
    else:
        path = Path(value)
        if path.is_absolute() or value.startswith("file://"):
            raise ScenePathError(
                f"{field} must be relative or use a declared package:// URI"
            )
        candidate = (urdf.parent / path).resolve(strict=False)
        try:
            candidate.relative_to(scene_root.resolve())
        except ValueError as exc:
            raise ScenePathError(f"{field} escapes the scene directory") from exc
    if not candidate.is_file():
        raise ScenePathError(f"{field} does not exist: {candidate}")
    return candidate


def _parse_urdf(*, name: str, source: Path) -> ET.Element:
    """Parse one robot source without importing or constructing a backend."""

    try:
        tree = ET.parse(source)
    except (OSError, ET.ParseError) as exc:
        raise SceneValidationError(f"robots.{name}.urdf is not valid XML") from exc
    root = tree.getroot()
    if root.tag != "robot" or not root.get("name"):
        raise SceneValidationError(f"robots.{name}.urdf must have a named <robot> root")
    return root


def _stage_urdf(
    *,
    name: str,
    source: Path,
    scene_root: Path,
    packages: Mapping[str, object],
    output_dir: Path,
) -> tuple[Path, ET.Element]:
    root = _parse_urdf(name=name, source=source)
    tree = ET.ElementTree(root)

    assets_dir = output_dir / "assets" / name
    assets_dir.mkdir(parents=True, exist_ok=True)
    asset_rows = list(root.findall(".//mesh")) + list(root.findall(".//texture"))
    for index, element in enumerate(asset_rows):
        attribute = "filename"
        raw = element.get(attribute)
        if raw is None:
            continue
        source_asset = _resolve_urdf_asset(
            raw,
            urdf=source,
            scene_root=scene_root,
            packages=packages,
            field=f"robots.{name}.urdf asset {index}",
        )
        digest = hashlib.sha256(source_asset.read_bytes()).hexdigest()[:12]
        destination = assets_dir / f"{digest}-{source_asset.name}"
        if not destination.exists():
            shutil.copyfile(source_asset, destination)
        element.set(attribute, f"../assets/{name}/{destination.name}")

    mujoco_extension = root.find("mujoco")
    if mujoco_extension is None:
        mujoco_extension = ET.SubElement(root, "mujoco")
    compiler = mujoco_extension.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(mujoco_extension, "compiler")
    compiler.set("discardvisual", "false")

    sources = output_dir / "sources"
    sources.mkdir(parents=True, exist_ok=True)
    # MjSpec 3.4 dispatches from the file suffix before inspecting the root
    # element. The staged document is rewritten XML that still has a URDF
    # <robot> root, so use the portable XML suffix accepted across 3.5+.
    staged = sources / f"{name}.xml"
    tree.write(staged, encoding="utf-8", xml_declaration=True)
    return staged, root


def _attribute_number(element: ET.Element, name: str) -> float | None:
    value = element.get(name)
    if value is None:
        return None
    try:
        result = float(value)
    except ValueError as exc:
        raise SceneValidationError(
            f"URDF joint limit {name!r} must be numeric"
        ) from exc
    if not math.isfinite(result):
        raise SceneValidationError(f"URDF joint limit {name!r} must be finite")
    return result


def _joints(
    *, name: str, root: ET.Element, robot: Mapping[str, Any]
) -> tuple[_Joint, ...]:
    overrides = robot.get("joints", {})
    by_name = {str(row.get("name")): row for row in root.findall("joint")}
    unknown = set(overrides) - set(by_name)
    if unknown:
        raise SceneValidationError(
            f"robots.{name}.joints names unknown URDF joints {sorted(unknown)!r}"
        )
    result: list[_Joint] = []
    for joint_name, row in by_name.items():
        kind = row.get("type")
        if kind == "fixed":
            if joint_name in overrides:
                raise SceneValidationError(
                    f"robots.{name}.joints.{joint_name} cannot actuate a fixed joint"
                )
            continue
        if kind not in {"revolute", "continuous", "prismatic"}:
            raise SceneValidationError(
                f"robots.{name} joint {joint_name!r} has unsupported type {kind!r}; "
                "portable scenes support scalar revolute, continuous, and "
                "prismatic joints"
            )
        if row.find("mimic") is not None:
            raise SceneValidationError(
                f"robots.{name} joint {joint_name!r} is a mimic joint; provide a "
                "backend-native scene until portable coupled joints are specified"
            )
        limit = row.find("limit")
        source_lower = None if limit is None else _attribute_number(limit, "lower")
        source_upper = None if limit is None else _attribute_number(limit, "upper")
        source_velocity = (
            None if limit is None else _attribute_number(limit, "velocity")
        )
        source_effort = None if limit is None else _attribute_number(limit, "effort")
        override = overrides.get(joint_name, {})
        lower = override.get("lower", source_lower)
        upper = override.get("upper", source_upper)
        if lower is None or upper is None:
            raise SceneValidationError(
                f"robots.{name}.joints.{joint_name} needs finite lower and upper "
                "limits (continuous joints require reviewed overrides)"
            )
        lower = float(lower)
        upper = float(upper)
        if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
            raise SceneValidationError(
                f"robots.{name}.joints.{joint_name} needs lower < upper"
            )
        if source_lower is not None and lower < source_lower:
            raise SceneValidationError(
                f"robots.{name}.joints.{joint_name}.lower widens the URDF limit"
            )
        if source_upper is not None and upper > source_upper:
            raise SceneValidationError(
                f"robots.{name}.joints.{joint_name}.upper widens the URDF limit"
            )
        velocity = override.get("max_velocity", source_velocity)
        effort = override.get("effort", source_effort)
        if velocity is None or float(velocity) <= 0.0:
            raise SceneValidationError(
                f"robots.{name}.joints.{joint_name} needs a positive velocity limit"
            )
        if effort is None or float(effort) <= 0.0:
            raise SceneValidationError(
                f"robots.{name}.joints.{joint_name} needs a positive effort limit"
            )
        if source_velocity is not None and float(velocity) > source_velocity:
            raise SceneValidationError(
                f"robots.{name}.joints.{joint_name}.max_velocity widens the URDF limit"
            )
        if source_effort is not None and float(effort) > source_effort:
            raise SceneValidationError(
                f"robots.{name}.joints.{joint_name}.effort widens the URDF limit"
            )
        default_home = 0.0 if lower <= 0.0 <= upper else (lower + upper) / 2.0
        home = float(override.get("home", default_home))
        if not lower <= home <= upper:
            raise SceneValidationError(
                f"robots.{name}.joints.{joint_name}.home is outside its limits"
            )
        kp = float(override.get("kp", robot.get("position_kp", 100.0)))
        if not math.isfinite(kp) or kp <= 0.0:
            raise SceneValidationError(
                f"robots.{name}.joints.{joint_name}.kp must be positive"
            )
        result.append(
            _Joint(
                name=joint_name,
                lower=lower,
                upper=upper,
                home=home,
                velocity=float(velocity),
                effort=float(effort),
                kp=kp,
            )
        )
    if not result:
        raise SceneValidationError(
            f"robots.{name}.urdf has no controllable scalar joints"
        )
    return tuple(result)


def _material_values(row: Mapping[str, Any] | None) -> dict[str, Any]:
    value = {} if row is None else row
    result: dict[str, Any] = {}
    if "rgba" in value:
        rgba = _vector(value["rgba"], (), width=4)
        if any(item < 0.0 or item > 1.0 for item in rgba):
            raise SceneValidationError("material rgba components must be in [0, 1]")
        result["rgba"] = rgba
    for field in ("metallic", "roughness", "emission", "reflectance"):
        if field in value:
            number = float(value[field])
            if not 0.0 <= number <= 1.0:
                raise SceneValidationError(f"material {field} must be in [0, 1]")
            result[field] = number
    return result


def _apply_appearance(spec: Any, *, name: str, robot: Mapping[str, Any]) -> None:
    appearance = robot.get("appearance", {})
    rows: list[tuple[str | None, Mapping[str, Any]]] = []
    if "default" in appearance:
        rows.append((None, appearance["default"]))
    rows.extend((str(link), row) for link, row in appearance.get("links", {}).items())
    for index, (link, row) in enumerate(rows):
        if link is not None and spec.body(link) is None:
            raise SceneValidationError(
                f"robots.{name}.appearance.links names unknown URDF link {link!r}"
            )
        values = _material_values(row)
        material_name = f"waddle_appearance_{index}"
        spec.add_material(name=material_name, **values)
        matched = 0
        for geom in spec.geoms:
            if link is None or geom.parent.name == link:
                geom.material = material_name
                if "rgba" in values:
                    geom.rgba = values["rgba"]
                matched += 1
        if link is not None and matched == 0:
            raise SceneValidationError(
                f"robots.{name}.appearance link {link!r} has no renderable geometry"
            )


def _copy_scene_mesh(
    source: Path, *, output_dir: Path, category: str, name: str
) -> tuple[Path, str]:
    digest = hashlib.sha256(source.read_bytes()).hexdigest()[:12]
    destination_dir = output_dir / "assets" / category
    destination_dir.mkdir(parents=True, exist_ok=True)
    destination = destination_dir / f"{name}-{digest}{source.suffix.lower()}"
    if not destination.exists():
        shutil.copyfile(source, destination)
    return destination, destination.relative_to(output_dir).as_posix()


def _add_material(spec: Any, *, name: str, row: Mapping[str, Any]) -> str:
    spec.add_material(name=name, **_material_values(row))
    return name


def _geom_arguments(
    mj: Any,
    *,
    spec: Any,
    geometry: Mapping[str, Any],
    scene_root: Path,
    output_dir: Path,
    asset_category: str,
    asset_name: str,
) -> dict[str, Any]:
    kind = str(geometry["kind"])
    if kind == "box":
        size = [value / 2.0 for value in _vector(geometry["size_m"], (), width=3)]
        enum = mj.mjtGeom.mjGEOM_BOX
    elif kind == "sphere":
        size = [float(geometry["radius_m"]), 0.0, 0.0]
        enum = mj.mjtGeom.mjGEOM_SPHERE
    elif kind in {"capsule", "cylinder"}:
        size = [float(geometry["radius_m"]), float(geometry["length_m"]) / 2.0, 0.0]
        enum = (
            mj.mjtGeom.mjGEOM_CAPSULE
            if kind == "capsule"
            else mj.mjtGeom.mjGEOM_CYLINDER
        )
    elif kind == "plane":
        extents = [float(value) / 2.0 for value in geometry["size_m"]]
        size = [extents[0], extents[1], 0.01]
        enum = mj.mjtGeom.mjGEOM_PLANE
    elif kind == "mesh":
        source = _portable_path(
            scene_root, geometry["path"], field=f"geometry {asset_name} mesh"
        )
        if not source.is_file():
            raise ScenePathError(f"geometry mesh does not exist: {source}")
        _destination, relative = _copy_scene_mesh(
            source,
            output_dir=output_dir,
            category=asset_category,
            name=asset_name,
        )
        mesh_name = f"{asset_category}_{asset_name}_mesh"
        spec.add_mesh(
            name=mesh_name,
            file=relative,
            scale=_vector(geometry.get("scale"), (1.0, 1.0, 1.0), width=3),
        )
        return {"type": mj.mjtGeom.mjGEOM_MESH, "meshname": mesh_name}
    else:  # pragma: no cover - held by the JSON schema
        raise AssertionError(f"unknown portable geometry {kind!r}")
    checked_width = (
        1 if kind == "sphere" else 2 if kind in {"capsule", "cylinder", "plane"} else 3
    )
    if any(not math.isfinite(value) or value <= 0.0 for value in size[:checked_width]):
        raise SceneValidationError(
            f"geometry {asset_name!r} dimensions must be positive"
        )
    return {"type": enum, "size": size}


def _add_geometry(
    mj: Any,
    *,
    spec: Any,
    body: Any,
    name: str,
    row: Mapping[str, Any],
    scene_root: Path,
    output_dir: Path,
    category: str,
) -> None:
    position, quaternion = _pose(row.get("pose"))
    material_name = _add_material(
        spec,
        name=f"{category}_{name}_material",
        row=row["material"],
    )
    arguments = _geom_arguments(
        mj,
        spec=spec,
        geometry=row["geometry"],
        scene_root=scene_root,
        output_dir=output_dir,
        asset_category=category,
        asset_name=name,
    )
    arguments.update(
        name=f"{category}/{name}",
        pos=position,
        quat=quaternion,
        material=material_name,
    )
    collision = row.get("collision")
    if collision is None:
        arguments.update(contype=0, conaffinity=0, group=2)
    else:
        arguments.update(contype=1, conaffinity=1, group=0)
        if "friction" in collision:
            arguments["friction"] = [float(value) for value in collision["friction"]]
        if "mass_kg" in collision:
            arguments["mass"] = float(collision["mass_kg"])
        if "density_kg_m3" in collision:
            arguments["density"] = float(collision["density_kg_m3"])
    body.add_geom(**arguments)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_yaml(path: Path, document: Mapping[str, Any]) -> None:
    import yaml

    path.write_text(
        yaml.safe_dump(document, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def _preflight_robot_sources(config: SceneConfig) -> None:
    """Validate backend-independent URDF constraints before loading MuJoCo."""

    for part_name, robot in config.document["robots"].items():
        base_frame = str(robot.get("base_frame", "world"))
        if base_frame != "world":
            raise SceneValidationError(
                f"robots.{part_name}.base_frame must be 'world'; the portable "
                "robot pose already places that URDF base in the scene world"
            )
        source = _portable_path(
            config.scene_root,
            robot["urdf"],
            field=f"robots.{part_name}.urdf",
        )
        root = _parse_urdf(name=str(part_name), source=source)
        _joints(name=str(part_name), root=root, robot=robot)


def compile_scene(*, config: SceneConfig, output_dir: Path) -> SceneArtifacts:
    """Compile one portable scene into MJCF, site.yaml, and replay evidence."""

    _preflight_robot_sources(config)
    mj = _mujoco_module()
    if not hasattr(mj, "MjSpec"):
        raise SceneCompilerError(
            "portable scene compilation needs MuJoCo 3.5 or newer; reinstall "
            "'waddle-sdk[mujoco]'"
        )
    output_dir.mkdir(parents=True, exist_ok=True)
    scene = mj.MjSpec()
    scene.modelname = config.name
    scene.modelfiledir = str(output_dir)
    physics = config.document.get("physics", {})
    scene.option.timestep = float(physics.get("timestep_s", 0.002))
    scene.option.gravity = _vector(
        physics.get("gravity_m_s2"), (0.0, 0.0, -9.81), width=3
    )

    packages = config.document.get("packages", {})
    part_rows: dict[str, Any] = {}
    attached_links: dict[str, set[str]] = {}
    tool_mounts: dict[str, tuple[str, np.ndarray]] = {}
    for part_name, robot in config.document["robots"].items():
        base_frame = str(robot.get("base_frame", "world"))
        if base_frame != "world":
            raise SceneValidationError(
                f"robots.{part_name}.base_frame must be 'world'; the portable "
                "robot pose already places that URDF base in the scene world"
            )
        source = _portable_path(
            config.scene_root,
            robot["urdf"],
            field=f"robots.{part_name}.urdf",
        )
        staged, urdf_root = _stage_urdf(
            name=str(part_name),
            source=source,
            scene_root=config.scene_root,
            packages=packages,
            output_dir=output_dir,
        )
        joints = _joints(name=str(part_name), root=urdf_root, robot=robot)
        try:
            child = mj.MjSpec.from_file(str(staged))
        except ValueError as exc:
            raise SceneCompilerError(
                f"MuJoCo refused robots.{part_name}.urdf ({type(exc).__name__})"
            ) from exc
        for asset in list(child.meshes) + list(child.textures):
            if asset.file:
                asset.file = f"assets/{part_name}/{Path(asset.file).name}"
        _apply_appearance(child, name=str(part_name), robot=robot)
        links = {str(body.name) for body in child.bodies if body.name}
        attached_links[str(part_name)] = links
        position, quaternion = _pose(robot.get("pose"))
        mount = scene.worldbody.add_frame(
            name=f"{part_name}/mount", pos=position, quat=quaternion
        )
        try:
            scene.attach(child, prefix=f"{part_name}/", frame=mount)
        except (AttributeError, TypeError):  # early MjSpec attachment spelling
            mount.attach(child, f"{part_name}/", "")

        actuator_names: list[str] = []
        model_joint_names: list[str] = []
        for joint in joints:
            model_joint = f"{part_name}/{joint.name}"
            actuator_name = f"{part_name}/{joint.name}_position"
            actuator = scene.add_actuator(
                name=actuator_name,
                target=model_joint,
                trntype=mj.mjtTrn.mjTRN_JOINT,
                ctrllimited=1,
                ctrlrange=[joint.lower, joint.upper],
                forcelimited=1,
                forcerange=[-joint.effort, joint.effort],
            )
            # Spell the <position> shortcut out using MjSpec fields. The
            # set_to_position convenience method was added after the supported
            # floor, so use the stable generic fields across MuJoCo 3.5+.
            actuator.dyntype = mj.mjtDyn.mjDYN_NONE
            actuator.gaintype = mj.mjtGain.mjGAIN_FIXED
            actuator.gainprm[0] = joint.kp
            actuator.biastype = mj.mjtBias.mjBIAS_AFFINE
            actuator.biasprm[1] = -joint.kp
            actuator_names.append(actuator_name)
            model_joint_names.append(model_joint)

        tool_site = None
        if "tool" in robot:
            tool = robot["tool"]
            link = str(tool["link"])
            if link not in links:
                raise SceneValidationError(
                    f"robots.{part_name}.tool.link names unknown URDF link {link!r}"
                )
            tool_site = f"{part_name}/tool"
            tool_position, tool_quaternion = _pose(tool.get("pose"))
            tool_mounts[str(part_name)] = (
                link,
                _pose_matrix(tool_position, tool_quaternion),
            )
            scene.body(f"{part_name}/{link}").add_site(
                name=tool_site,
                pos=tool_position,
                quat=tool_quaternion,
                size=[0.005, 0.0, 0.0],
            )

        for coating in robot.get("coatings", ()):
            link = str(coating["link"])
            if link not in links:
                raise SceneValidationError(
                    f"robots.{part_name}.coatings names unknown URDF link {link!r}"
                )
            _add_geometry(
                mj,
                spec=scene,
                body=scene.body(f"{part_name}/{link}"),
                name=str(coating["name"]),
                row=coating,
                scene_root=config.scene_root,
                output_dir=output_dir,
                category=f"{part_name}_coating",
            )

        collision_bodies = []
        for sphere in robot.get("collision_spheres", ()):
            link = str(sphere["link"])
            if link not in links:
                raise SceneValidationError(
                    f"robots.{part_name}.collision_spheres names unknown link {link!r}"
                )
            collision_bodies.append(
                {
                    "name": str(sphere["name"]),
                    "body": f"{part_name}/{link}",
                    "radius_m": float(sphere["radius_m"]),
                    "center_m": _vector(
                        sphere.get("center_m"), (0.0, 0.0, 0.0), width=3
                    ),
                }
            )
        for coating in robot.get("coatings", ()):
            if "collision" not in coating:
                continue
            sphere = coating["safety_sphere"]
            collision_bodies.append(
                {
                    "name": str(sphere["name"]),
                    "body": f"{part_name}/{coating['link']}",
                    "radius_m": float(sphere["radius_m"]),
                    "center_m": _vector(
                        sphere.get("center_m"), (0.0, 0.0, 0.0), width=3
                    ),
                }
            )
        minimum_velocity = min(joint.velocity for joint in joints)
        part_rows[str(part_name)] = {
            "world": "cell",
            "posture": str(robot.get("posture", "supervised")),
            "base_frame": base_frame,
            "connection": {},
            "joint_limits": {
                joint.name: [joint.lower, joint.upper] for joint in joints
            },
            "options": {
                "model_joint_names": model_joint_names,
                "actuator_names": actuator_names,
                "home": [joint.home for joint in joints],
                "rate_hz": float(robot.get("rate_hz", 100.0)),
                "max_joint_speed_per_s": minimum_velocity,
                **({} if tool_site is None else {"tool_site": tool_site}),
                **(
                    {}
                    if not collision_bodies
                    else {"collision_bodies": collision_bodies}
                ),
            },
        }

    for name, row in config.document.get("geometry", {}).items():
        _add_geometry(
            mj,
            spec=scene,
            body=scene.worldbody,
            name=str(name),
            row=row,
            scene_root=config.scene_root,
            output_dir=output_dir,
            category="scene",
        )

    frames: dict[str, Any] = {}
    for name, row in config.document.get("frames", {}).items():
        position, quaternion = _pose(row.get("pose"))
        frames[str(name)] = {
            "parent": str(row["parent"]),
            "position": position,
            "quaternion_wxyz": quaternion,
        }

    camera_rows: dict[str, Any] = {}
    calibration_rows: dict[str, dict[str, Any]] = {}
    for name, row in config.document.get("cameras", {}).items():
        position, optical_quaternion = _pose(row.get("pose"))
        # Portable camera poses use the optical convention (+Z forward, +X
        # right, +Y down). MuJoCo cameras look down -Z with +Y up.
        mujoco_quaternion = _multiply_quaternion(
            optical_quaternion, (0.0, 1.0, 0.0, 0.0)
        )
        mount = row["mount"]
        if mount["kind"] == "scene":
            body = scene.worldbody
            parent_frame = "world"
            site_mount = {"kind": "scene"}
            calibration_name = str(name)
            calibration_rows[calibration_name] = _simulation_calibration(
                config=config,
                camera=str(name),
                frame_id=str(row.get("frame_id", f"{name}_optical")),
                to_frame="world",
                matrix=_pose_matrix(position, optical_quaternion),
            )
        else:
            part = str(mount["part"])
            link = str(mount["link"])
            if link not in attached_links[part]:
                raise SceneValidationError(
                    f"cameras.{name}.mount.link names unknown {part!r} link {link!r}"
                )
            body = scene.body(f"{part}/{link}")
            parent_frame = f"{part}/{link}"
            site_mount = {"kind": "wrist", "part": part}
            tool_mount = tool_mounts.get(part)
            if tool_mount is None:
                raise SceneValidationError(
                    f"cameras.{name} is wrist-mounted but robots.{part}.tool is not declared; "
                    "privileged calibration cannot define a TCP transform"
                )
            tool_link, link_from_tcp = tool_mount
            if link != tool_link:
                raise SceneValidationError(
                    f"cameras.{name}.mount.link is {link!r}, but robots.{part}.tool.link "
                    f"is {tool_link!r}; automatic wrist calibration requires a camera "
                    "rigidly mounted on the declared tool link"
                )
            calibration_name = f"{name}_mount"
            calibration_rows[calibration_name] = _simulation_calibration(
                config=config,
                camera=str(name),
                frame_id=str(row.get("frame_id", f"{name}_optical")),
                to_frame="tcp",
                matrix=np.linalg.inv(link_from_tcp)
                @ _pose_matrix(position, optical_quaternion),
                part=part,
            )
        fovy = float(row.get("vertical_fov_deg", 58.0))
        if not 0.0 < fovy < 180.0:
            raise SceneValidationError(
                f"cameras.{name}.vertical_fov_deg must be in (0, 180)"
            )
        body.add_camera(
            name=str(name),
            pos=position,
            quat=mujoco_quaternion,
            fovy=fovy,
        )
        frame_id = str(row.get("frame_id", f"{name}_optical"))
        if not frame_id.replace("_", "a").replace("-", "a").replace(".", "a").isalnum():
            raise SceneValidationError(
                f"cameras.{name}.frame_id must be a portable frame name"
            )
        frames[frame_id] = {
            "parent": parent_frame,
            "position": position,
            "quaternion_wxyz": optical_quaternion,
        }
        camera_rows[str(name)] = {
            "world": "cell",
            "connection": {"camera": str(name)},
            "stream": {
                "width": int(row["stream"]["width"]),
                "height": int(row["stream"]["height"]),
                "fps": float(row["stream"]["fps"]),
            },
            "frame_id": frame_id,
            "mount": site_mount,
            "options": {
                "depth": bool(row.get("depth", True)),
                "depth_scale_mm": float(row.get("depth_scale_mm", 1.0)),
            },
        }

    light_types = {
        "directional": mj.mjtLightType.mjLIGHT_DIRECTIONAL,
        "point": mj.mjtLightType.mjLIGHT_POINT,
        "spot": mj.mjtLightType.mjLIGHT_SPOT,
    }
    for name, row in config.document.get("lights", {}).items():
        direction = _vector(row.get("direction"), (0.0, 0.0, -1.0), width=3)
        norm = float(np.linalg.norm(direction))
        if norm <= 1e-12:
            raise SceneValidationError(f"lights.{name}.direction must be non-zero")
        direction = [value / norm for value in direction]
        color = _vector(row["color"], (), width=3)
        if any(value < 0.0 or value > 1.0 for value in color):
            raise SceneValidationError(f"lights.{name}.color must be in [0, 1]")
        intensity = float(row["intensity"])
        if intensity < 0.0:
            raise SceneValidationError(f"lights.{name}.intensity must be non-negative")
        arguments: dict[str, Any] = {
            "name": str(name),
            "type": light_types[str(row["type"])],
            "pos": _vector(row["position_m"], (), width=3),
            "dir": direction,
            "diffuse": color,
            "specular": color,
            "intensity": intensity,
            "castshadow": int(bool(row.get("cast_shadows", True))),
        }
        if "range_m" in row:
            arguments["range"] = float(row["range_m"])
        if "spot_cutoff_deg" in row:
            arguments["cutoff"] = float(row["spot_cutoff_deg"])
        scene.worldbody.add_light(**arguments)

    try:
        scene.compile()
        world_xml = scene.to_xml()
    except ValueError as exc:
        raise SceneCompilerError(
            "MuJoCo could not compile the resolved portable scene "
            f"({type(exc).__name__})"
        ) from exc
    world_path = output_dir / "world.xml"
    world_path.write_text(world_xml, encoding="utf-8")

    site_options = config.document.get("site", {})
    calibration_value = str(site_options.get("calibration_artifacts", "calib/"))
    calibration_relative = Path(calibration_value)
    if (
        calibration_relative.is_absolute()
        or any(part == ".." for part in calibration_relative.parts)
        or "\\" in calibration_value
        or "\x00" in calibration_value
    ):
        raise ScenePathError("site.calibration_artifacts must stay beneath the build")
    calibration_dir = (output_dir / calibration_relative).resolve(strict=False)
    try:
        calibration_dir.relative_to(output_dir.resolve())
    except ValueError as exc:
        raise ScenePathError("site.calibration_artifacts escapes the build") from exc
    calibration_dir.mkdir(parents=True, exist_ok=True)
    for artifact_name, artifact in sorted(calibration_rows.items()):
        (calibration_dir / f"{artifact_name}.json").write_text(
            json.dumps(artifact, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

    site_document = {
        "api_version": "waddle.site/v1",
        "kind": "Site",
        "metadata": {"id": config.name},
        "worlds": {
            "cell": {
                "driver": "waddle_sdk.robots.mujoco:backend",
                "connection": {"model": "world.xml"},
                "options": {"evidence": "resolved-scene.json"},
            }
        },
        "parts": part_rows,
        "cameras": camera_rows,
        "frames": frames,
        "calibration": {"artifacts": calibration_value},
        "workspace_bounds": site_options.get("workspace_bounds", {}),
        "envelope": {
            "static_keepouts": site_options.get("static_keepouts", []),
            "self_collision": site_options.get("self_collision", {}),
        },
        "recording": {
            "root": str(site_options.get("recording_root", "recordings/")),
            "format": "mcap",
        },
    }
    site_path = output_dir / "site.yaml"
    _write_yaml(site_path, site_document)

    # Validate the emitted customer contract without constructing a backend.
    from ..site import load_site

    load_site(site_path)

    inventory = []
    for path in sorted(output_dir.rglob("*")):
        if path.is_file() and path.name != "resolved-scene.json":
            inventory.append(
                {
                    "path": path.relative_to(output_dir).as_posix(),
                    "size": path.stat().st_size,
                    "sha256": _sha256(path),
                }
            )
    evidence = {
        "schema": "waddle.simulation-build/v1",
        "backend": "mujoco",
        "scene_api_version": config.document["api_version"],
        "scene_id": config.name,
        "seed": config.seed,
        "resolved_scene": config.document,
        "files": inventory,
    }
    evidence_path = output_dir / "resolved-scene.json"
    evidence_path.write_text(
        json.dumps(evidence, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return SceneArtifacts(
        site_manifest="site.yaml",
        world="world.xml",
        evidence="resolved-scene.json",
    )


__all__ = ["compile_scene"]
