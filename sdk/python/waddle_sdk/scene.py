"""Portable, backend-compiled simulation scene declarations.

The scene layer is an explicit build step.  It turns a reviewed portable scene
and its local assets into backend-native files plus an ordinary ``site.yaml``.
It never opens a Site, simulator, robot, camera, transport, or renderer. Applications
consume the resulting :class:`~waddle_sdk.runtime.SdkRuntimePort` unchanged.
"""

from __future__ import annotations

import hashlib
import importlib
import inspect
import json
import math
import random
import shutil
import tempfile
import xml.etree.ElementTree as ET
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from importlib import metadata
from importlib.resources import files
from pathlib import Path
from types import MappingProxyType
from typing import Any, Protocol, runtime_checkable

SCENE_API_VERSION = "waddle.scene/v1"


class SceneError(ValueError):
    """Base class for portable scene build refusals."""


class SceneSyntaxError(SceneError):
    """A scene document is not valid JSON or YAML."""


class SceneValidationError(SceneError):
    """A scene document or compiler result violates the public contract."""


class ScenePathError(SceneError):
    """A scene-owned asset path is absolute or escapes the scene directory."""


class SceneCompilerError(SceneError):
    """A backend compiler could not be resolved or satisfy its contract."""


@dataclass(frozen=True)
class SceneConfig:
    """One fully validated and deterministically resolved compiler input."""

    name: str
    document: Mapping[str, Any]
    scene_root: Path
    seed: int


@dataclass(frozen=True)
class SceneArtifacts:
    """Portable relative paths returned by a backend scene compiler."""

    site_manifest: str
    world: str
    evidence: str


@dataclass(frozen=True)
class CompiledScene:
    """A completed backend-native scene bundle."""

    backend: str
    output_dir: Path
    artifacts: SceneArtifacts

    @property
    def site_path(self) -> Path:
        return self.output_dir / self.artifacts.site_manifest

    @property
    def world_path(self) -> Path:
        return self.output_dir / self.artifacts.world

    @property
    def evidence_path(self) -> Path:
        return self.output_dir / self.artifacts.evidence


@runtime_checkable
class SimulationSceneCompiler(Protocol):
    """Callable implemented by one installable simulator backend package."""

    def __call__(self, *, config: SceneConfig, output_dir: Path) -> SceneArtifacts: ...


_COMPILER_ALIASES = MappingProxyType(
    {"mujoco": "waddle_sdk.robots.mujoco_scene:compile_scene"}
)


def simulation_compiler_names() -> tuple[str, ...]:
    """List built-in and installed compiler short names without importing them."""

    installed = metadata.entry_points().select(group="waddle_sdk.simulation_compilers")
    return tuple(sorted({*_COMPILER_ALIASES, *(entry.name for entry in installed)}))


def _initializer_asset(
    value: str,
    *,
    urdf: Path,
    packages: Mapping[str, Path],
) -> Path:
    if "\x00" in value or "\\" in value:
        raise ScenePathError("URDF asset names must be portable paths")
    if value.startswith("package://"):
        package_path = value[len("package://") :]
        package, separator, remainder = package_path.partition("/")
        if not separator or package not in packages:
            raise ScenePathError(
                f"URDF asset names package {package!r}; pass --package {package}=PATH"
            )
        package_root = packages[package].resolve()
        candidate = (package_root / remainder).resolve(strict=False)
        try:
            candidate.relative_to(package_root)
        except ValueError as exc:
            raise ScenePathError(f"URDF asset escapes package {package!r}") from exc
    else:
        relative = Path(value)
        if (
            relative.is_absolute()
            or value.startswith("file://")
            or any(part == ".." for part in relative.parts)
        ):
            raise ScenePathError(
                "URDF assets must be relative or use an explicitly mapped "
                "package:// URI"
            )
        source_root = urdf.parent.resolve()
        candidate = (source_root / relative).resolve(strict=False)
        try:
            candidate.relative_to(source_root)
        except ValueError as exc:
            raise ScenePathError("URDF asset escapes its source directory") from exc
    if not candidate.is_file():
        raise ScenePathError(f"URDF asset does not exist: {candidate}")
    return candidate


def initialize_scene(
    urdf: str | Path,
    *,
    output_dir: str | Path,
    part_name: str = "arm",
    tool_link: str | None = None,
    rgba: Sequence[float] = (0.9, 0.55, 0.1, 1.0),
    packages: Mapping[str, str | Path] | None = None,
) -> Path:
    """Create a portable editable scene bundle from one local URDF.

    The initializer copies only assets referenced by the URDF, rewrites their
    paths into the new bundle, and refuses to overwrite an existing directory.
    It does not guess workspace bounds, collision spheres, or a tool frame.
    """

    if (
        not isinstance(part_name, str)
        or not part_name
        or not all(character.isalnum() or character in "_.-" for character in part_name)
    ):
        raise SceneValidationError("part_name must be a portable manifest name")
    source = Path(urdf).resolve(strict=False)
    if not source.is_file():
        raise ScenePathError(f"URDF does not exist: {source}")
    color = [float(value) for value in rgba]
    if len(color) != 4 or not all(
        math.isfinite(value) and 0.0 <= value <= 1.0 for value in color
    ):
        raise SceneValidationError("RGBA must contain four finite values in [0, 1]")
    package_roots = {
        str(name): Path(value).resolve(strict=False)
        for name, value in (packages or {}).items()
    }
    for name, root in package_roots.items():
        if not root.is_dir():
            raise ScenePathError(f"package {name!r} root does not exist: {root}")
    try:
        tree = ET.parse(source)
    except (OSError, ET.ParseError) as exc:
        raise SceneValidationError("URDF is not valid XML") from exc
    root = tree.getroot()
    if root.tag != "robot" or not root.get("name"):
        raise SceneValidationError("URDF must have a named <robot> root")
    links = {str(row.get("name")) for row in root.findall("link")}
    if tool_link is not None and tool_link not in links:
        raise SceneValidationError(f"tool link {tool_link!r} is not in the URDF")

    output = Path(output_dir).resolve(strict=False)
    if output.exists():
        raise FileExistsError(f"simulation scene output already exists: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(
        tempfile.mkdtemp(prefix=f".{output.name}-", dir=str(output.parent))
    )
    try:
        assets = temporary / "assets"
        assets.mkdir()
        elements = list(root.findall(".//mesh")) + list(root.findall(".//texture"))
        for element in elements:
            value = element.get("filename")
            if value is None:
                continue
            asset = _initializer_asset(
                value,
                urdf=source,
                packages=package_roots,
            )
            digest = hashlib.sha256(asset.read_bytes()).hexdigest()[:12]
            destination = assets / f"{digest}-{asset.name}"
            if not destination.exists():
                shutil.copyfile(asset, destination)
            element.set("filename", f"assets/{destination.name}")
        staged_urdf = temporary / "robot.urdf"
        tree.write(staged_urdf, encoding="utf-8", xml_declaration=True)

        robot: dict[str, Any] = {
            "urdf": "robot.urdf",
            "pose": {
                "position_m": [0.0, 0.0, 0.0],
                "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
            },
            "posture": "supervised",
            "base_frame": "world",
            "rate_hz": 100.0,
            "position_kp": 100.0,
            "appearance": {"default": {"rgba": color, "roughness": 0.6}},
            "collision_spheres": [],
        }
        if tool_link is not None:
            robot["tool"] = {
                "link": tool_link,
                "pose": {
                    "position_m": [0.0, 0.0, 0.0],
                    "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
                },
            }
        document = {
            "api_version": SCENE_API_VERSION,
            "kind": "SimulationScene",
            "metadata": {"id": f"{part_name}-simulation"},
            "physics": {
                "timestep_s": 0.002,
                "gravity_m_s2": [0.0, 0.0, -9.81],
            },
            "robots": {part_name: robot},
            "cameras": {
                "scene_camera": {
                    "mount": {"kind": "scene"},
                    "pose": {
                        "position_m": [0.0, -2.0, 0.8],
                        "quaternion_wxyz": [0.70710678, -0.70710678, 0.0, 0.0],
                    },
                    "stream": {"width": 640, "height": 480, "fps": 30},
                    "vertical_fov_deg": 58.0,
                    "depth": True,
                    "depth_scale_mm": 1.0,
                }
            },
            "lights": {
                "key": {
                    "type": "directional",
                    "position_m": [0.0, -1.0, 3.0],
                    "direction": [0.2, 0.2, -1.0],
                    "color": [1.0, 0.95, 0.9],
                    "intensity": 1.0,
                    "cast_shadows": True,
                }
            },
            "geometry": {
                "floor": {
                    "geometry": {"kind": "plane", "size_m": [4.0, 4.0]},
                    "pose": {"position_m": [0.0, 0.0, 0.0]},
                    "material": {
                        "rgba": [0.35, 0.35, 0.35, 1.0],
                        "roughness": 0.8,
                    },
                    "collision": {"friction": [0.8, 0.01, 0.001]},
                }
            },
            "site": {
                "static_keepouts": [],
                "self_collision": {},
                "calibration_artifacts": "calib/",
                "recording_root": "recordings/",
            },
            "randomization": {"seed": 0},
        }
        import yaml

        scene_path = temporary / "scene.yaml"
        scene_path.write_text(
            yaml.safe_dump(document, sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )
        _validate_semantics(_validate_schema(document), temporary)
        temporary.replace(output)
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return output / "scene.yaml"


def _load_document(path: Path) -> object:
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise SceneSyntaxError(f"cannot read scene manifest {path}: {exc}") from exc
    try:
        if path.suffix.lower() == ".json":
            return json.loads(text)
        import yaml

        return yaml.safe_load(text)
    except Exception as exc:  # parser exceptions are dependency-specific
        raise SceneSyntaxError(f"invalid scene manifest {path}: {exc}") from exc


def _schema() -> dict[str, Any]:
    resource = files("waddle_sdk").joinpath("schemas/scene-v1.schema.json")
    return json.loads(resource.read_text(encoding="utf-8"))


def _validate_schema(document: object) -> dict[str, Any]:
    try:
        import jsonschema
    except ModuleNotFoundError as exc:  # defensive for incomplete source installs
        raise RuntimeError(
            "simulation scenes require the waddle-sdk manifest dependencies; "
            "reinstall waddle-sdk"
        ) from exc
    validator = jsonschema.Draft202012Validator(_schema())
    errors = sorted(validator.iter_errors(document), key=lambda error: list(error.path))
    if errors:
        error = errors[0]
        location = ".".join(str(item) for item in error.absolute_path) or "<root>"
        raise SceneValidationError(f"{location}: {error.message}")
    assert isinstance(document, dict)
    return document


def _confined_path(root: Path, value: object, *, field: str) -> Path:
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


def _walk_numbers(value: object, path: str = "<root>") -> None:
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return
    if isinstance(value, (int, float)):
        if not math.isfinite(float(value)):
            raise SceneValidationError(f"{path}: numeric values must be finite")
        return
    if isinstance(value, Mapping):
        for name, child in value.items():
            _walk_numbers(child, f"{path}.{name}")
        return
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        for index, child in enumerate(value):
            _walk_numbers(child, f"{path}[{index}]")


def _validate_semantics(document: Mapping[str, Any], root: Path) -> None:
    _walk_numbers(document)
    robot_names = set(document["robots"])
    for name, robot in document["robots"].items():
        urdf = _confined_path(root, robot["urdf"], field=f"robots.{name}.urdf")
        if not urdf.is_file():
            raise ScenePathError(f"robots.{name}.urdf does not exist: {urdf}")
        for index, coating in enumerate(robot.get("coatings", ())):
            geometry = coating["geometry"]
            if geometry["kind"] == "mesh":
                mesh = _confined_path(
                    root,
                    geometry["path"],
                    field=f"robots.{name}.coatings[{index}].geometry.path",
                )
                if not mesh.is_file():
                    raise ScenePathError(f"coating mesh does not exist: {mesh}")
    for name, camera in document.get("cameras", {}).items():
        mount = camera["mount"]
        if mount["kind"] == "wrist" and mount["part"] not in robot_names:
            raise SceneValidationError(
                f"cameras.{name}.mount.part: unknown robot {mount['part']!r}"
            )
    for name, row in document.get("geometry", {}).items():
        geometry = row["geometry"]
        if geometry["kind"] == "mesh":
            mesh = _confined_path(
                root, geometry["path"], field=f"geometry.{name}.geometry.path"
            )
            if not mesh.is_file():
                raise ScenePathError(f"geometry mesh does not exist: {mesh}")


def _resolve_randomized(
    value: object,
    *,
    generator: random.Random,
    decisions: dict[str, object],
    path: str,
) -> object:
    if isinstance(value, Mapping):
        if set(value) == {"uniform"}:
            bounds = value["uniform"]
            assert isinstance(bounds, Sequence)
            lower, upper = float(bounds[0]), float(bounds[1])
            if lower > upper:
                raise SceneValidationError(f"{path}.uniform: lower exceeds upper")
            resolved = generator.uniform(lower, upper)
            decisions[path] = resolved
            return resolved
        if set(value) == {"choice"}:
            choices = value["choice"]
            assert isinstance(choices, Sequence)
            if not choices:
                raise SceneValidationError(f"{path}.choice: needs at least one value")
            selected = choices[generator.randrange(len(choices))]
            resolved = _resolve_randomized(
                selected,
                generator=generator,
                decisions=decisions,
                path=path,
            )
            decisions[path] = resolved
            return resolved
        return {
            str(name): _resolve_randomized(
                child,
                generator=generator,
                decisions=decisions,
                path=f"{path}.{name}",
            )
            for name, child in value.items()
        }
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        return [
            _resolve_randomized(
                child,
                generator=generator,
                decisions=decisions,
                path=f"{path}[{index}]",
            )
            for index, child in enumerate(value)
        ]
    return value


def resolve_scene(document: Mapping[str, Any], *, seed: int) -> dict[str, Any]:
    """Resolve every declared distribution with one replayable seed."""

    if (
        isinstance(seed, bool)
        or not isinstance(seed, int)
        or not 0 <= seed <= 2**63 - 1
    ):
        raise SceneValidationError(
            "randomization seed must be an integer in [0, 2^63-1]"
        )
    decisions: dict[str, object] = {}
    resolved = _resolve_randomized(
        document,
        generator=random.Random(seed),
        decisions=decisions,
        path="$",
    )
    assert isinstance(resolved, dict)
    resolved["resolution"] = {
        "seed": seed,
        "decisions": dict(sorted(decisions.items())),
    }
    _walk_numbers(resolved)
    return resolved


def resolve_scene_compiler(spec: str) -> Callable[..., object]:
    """Resolve a short backend name or ``module:callable`` compiler target."""

    target_spec = _COMPILER_ALIASES.get(spec, spec)
    if target_spec == spec and ":" not in spec and "." not in spec:
        selected = tuple(
            metadata.entry_points().select(
                group="waddle_sdk.simulation_compilers", name=spec
            )
        )
        if len(selected) > 1:
            raise SceneCompilerError(
                f"multiple installed scene compilers are named {spec!r}"
            )
        if selected:
            try:
                installed = selected[0].load()
            except (ImportError, AttributeError) as exc:
                raise SceneCompilerError(
                    f"cannot load installed scene compiler ({type(exc).__name__})"
                ) from exc
            if not callable(installed):
                raise SceneCompilerError(
                    f"installed scene compiler {spec!r} is not callable"
                )
            parameters = inspect.signature(installed).parameters
            if "config" not in parameters or "output_dir" not in parameters:
                raise SceneCompilerError(
                    f"installed scene compiler {spec!r} must accept config= and "
                    "output_dir="
                )
            return installed
    if ":" in target_spec:
        module_name, attribute = target_spec.split(":", 1)
    else:
        module_name, attribute = target_spec, "compile_scene"
    if not module_name or not attribute:
        raise SceneCompilerError(
            f"scene compiler {target_spec!r} must name module:attribute"
        )
    try:
        module = importlib.import_module(module_name)
        target = getattr(module, attribute)
    except (ImportError, AttributeError) as exc:
        raise SceneCompilerError(
            f"cannot load scene compiler ({type(exc).__name__})"
        ) from exc
    if not callable(target):
        raise SceneCompilerError(f"scene compiler {target_spec!r} is not callable")
    parameters = inspect.signature(target).parameters
    if "config" not in parameters or "output_dir" not in parameters:
        raise SceneCompilerError(
            f"scene compiler {target_spec!r} must accept config= and output_dir="
        )
    return target


def _safe_artifact(output: Path, value: str, *, field: str) -> Path:
    path = _confined_path(output, value, field=field)
    if not path.is_file():
        raise SceneCompilerError(f"compiler did not create {field}: {path}")
    return path


@dataclass(frozen=True)
class Scene:
    """A validated portable scene that has not imported a simulator runtime."""

    path: Path
    manifest: Mapping[str, Any]

    @property
    def id(self) -> str:
        return str(self.manifest["metadata"]["id"])

    def resolved(self, *, seed: int | None = None) -> SceneConfig:
        configured = self.manifest.get("randomization", {}).get("seed", 0)
        selected = configured if seed is None else seed
        if isinstance(selected, bool) or not isinstance(selected, int):
            raise SceneValidationError("randomization seed must be an integer")
        document = resolve_scene(self.manifest, seed=selected)
        return SceneConfig(
            name=self.id,
            document=document,
            scene_root=self.path.parent,
            seed=selected,
        )

    def compile(
        self,
        *,
        backend: str,
        output_dir: str | Path,
        seed: int | None = None,
    ) -> CompiledScene:
        """Compile atomically into a new directory without overwriting files."""

        output = Path(output_dir).resolve(strict=False)
        if output.exists():
            raise FileExistsError(f"simulation output already exists: {output}")
        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = Path(
            tempfile.mkdtemp(prefix=f".{output.name}-", dir=str(output.parent))
        )
        try:
            target = resolve_scene_compiler(backend)
            result = target(config=self.resolved(seed=seed), output_dir=temporary)
            if not isinstance(result, SceneArtifacts):
                raise SceneCompilerError(
                    "scene compiler must return waddle_sdk.scene.SceneArtifacts"
                )
            _safe_artifact(temporary, result.site_manifest, field="site_manifest")
            _safe_artifact(temporary, result.world, field="world")
            _safe_artifact(temporary, result.evidence, field="evidence")
            temporary.replace(output)
        except BaseException:
            shutil.rmtree(temporary, ignore_errors=True)
            raise
        return CompiledScene(backend=backend, output_dir=output, artifacts=result)


def load_scene(path: str | Path) -> Scene:
    """Load and validate a portable scene without importing a simulator."""

    source = Path(path).resolve(strict=False)
    document = _validate_schema(_load_document(source))
    if document["api_version"] != SCENE_API_VERSION:
        raise SceneValidationError(
            f"unsupported scene api_version {document['api_version']!r}"
        )
    _validate_semantics(document, source.parent)
    return Scene(path=source, manifest=document)


__all__ = [
    "SCENE_API_VERSION",
    "CompiledScene",
    "Scene",
    "SceneArtifacts",
    "SceneCompilerError",
    "SceneConfig",
    "SceneError",
    "ScenePathError",
    "SceneSyntaxError",
    "SceneValidationError",
    "SimulationSceneCompiler",
    "initialize_scene",
    "load_scene",
    "resolve_scene",
    "resolve_scene_compiler",
    "simulation_compiler_names",
]
