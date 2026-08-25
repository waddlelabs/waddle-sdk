"""Strict site manifest loading and the primary SDK lifecycle."""

from __future__ import annotations

import hashlib
import importlib
import inspect
import json
import re
import threading
import time
import warnings
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, replace
from importlib.resources import files
from pathlib import Path
from types import MappingProxyType
from typing import Any

import numpy as np

from . import descriptors
from ._session import Control, _derive_grants, create_core_session
from .cameras import CameraDriver
from .cameras.site import CameraConfig
from .robots import base
from .robots.site import PartConfig
from .runtime import (
    SUPPORT_CONTRACT_VERSION,
    BodySphere,
    FaultCode,
    JointPositionCommand,
    JSONValue,
    Observation,
    PartObservation,
    Pose,
    RuntimeEvent,
    RuntimeFault,
    SubmitResult,
    SupportFact,
    SupportMatrix,
    SupportRow,
)
from .transport import Grpc

SITE_API_VERSION = "waddle.site/v1"

_CONNECTOR_COMPATIBILITY_SCHEMA = "waddle.connector-compatibility/v1"
_CONNECTOR_COMPATIBILITY_DETAIL_MAX_BYTES = 2_048
_CONNECTOR_COMPATIBILITY_MESSAGE_MAX_CHARS = 512
_SEMVER = re.compile(
    r"(?:0|[1-9][0-9]*)\.(?:0|[1-9][0-9]*)\.(?:0|[1-9][0-9]*)"
    r"(?:-[0-9A-Za-z-]+(?:\.[0-9A-Za-z-]+)*)?"
    r"(?:\+[0-9A-Za-z-]+(?:\.[0-9A-Za-z-]+)*)?"
)
_UTC_DEADLINE = re.compile(r"[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}Z")


class ConnectorCompatibilityWarning(UserWarning):
    """The connected host recommends an SDK upgrade before a deadline."""


class ConnectorRegistrationError(RuntimeError):
    """A connector Register barrier failed with a stable public code."""

    def __init__(self, code: str, detail: str):
        self.code = code
        self.detail = detail
        super().__init__(f"{code}: {detail}")


def _exception_type(error: BaseException) -> str:
    """Return a bounded identifier without serializing an untrusted exception."""
    name = re.sub(r"[^0-9A-Za-z_.-]", "_", type(error).__name__)
    return name[:80] or "Exception"


def _operation_fault(
    operation: str,
    error: Exception,
    *,
    code: FaultCode = FaultCode.INTERNAL,
    context: Mapping[str, JSONValue] | None = None,
) -> RuntimeFault:
    """Classify an untyped implementation error at the public runtime boundary.

    Arbitrary vendor exception strings can contain device paths, URLs, or
    credentials.  Keep the original exception in Python's ``__cause__`` for
    owner-side logs, but carry only its type plus SDK-owned operation/scope
    fields over Metal's transport.
    """
    if isinstance(error, RuntimeFault):
        return error
    error_type = _exception_type(error)
    safe_context: dict[str, JSONValue] = {
        "operation": operation,
        "error_type": error_type,
    }
    if context is not None:
        safe_context.update(context)
    if isinstance(error, OSError) and error.errno is not None:
        safe_context["errno"] = int(error.errno)
    return RuntimeFault(
        code,
        f"{operation} failed ({error_type})",
        retryable=isinstance(error, TimeoutError),
        context=safe_context,
    )


def _event_fault(
    operation: str,
    error: Exception,
    *,
    context: Mapping[str, JSONValue] | None = None,
) -> dict[str, JSONValue]:
    fault = _operation_fault(operation, error, context=context)
    return fault.as_dict()


def _strict_semver(value: object) -> bool:
    if (
        not isinstance(value, str)
        or len(value) > 64
        or _SEMVER.fullmatch(value) is None
    ):
        return False
    without_build = value.split("+", 1)[0]
    if "-" not in without_build:
        return True
    prerelease = without_build.split("-", 1)[1]
    return all(
        not (
            identifier.isdigit() and len(identifier) > 1 and identifier.startswith("0")
        )
        for identifier in prerelease.split(".")
    )


def _connector_compatibility_message(detail: object, *, code: str) -> str | None:
    """Render allow-listed compatibility JSON without echoing server text."""
    if code not in {"upgrade_recommended", "upgrade_required"}:
        return None
    if not isinstance(detail, str):
        return None
    if len(detail.encode("utf-8")) > _CONNECTOR_COMPATIBILITY_DETAIL_MAX_BYTES:
        return None
    try:
        payload = json.loads(detail)
    except (TypeError, ValueError):
        return None
    if (
        not isinstance(payload, Mapping)
        or payload.get("schema") != _CONNECTOR_COMPATIBILITY_SCHEMA
        or payload.get("connector") != "waddle-sdk"
        or payload.get("code") != code
    ):
        return None

    versions: dict[str, str] = {}
    for name in ("current_version", "minimum_version", "recommended_version"):
        value = payload.get(name)
        if not _strict_semver(value):
            return None
        assert isinstance(value, str)
        versions[name] = value
    deadline = payload.get("enforcement_deadline")
    if (
        not isinstance(deadline, str)
        or len(deadline) > 32
        or _UTC_DEADLINE.fullmatch(deadline) is None
    ):
        return None

    upgrade = (
        f"python -m pip install --upgrade waddle-sdk=={versions['recommended_version']}"
    )
    if code == "upgrade_recommended":
        message = (
            f"Waddle SDK {versions['current_version']} will be rejected after "
            f"{deadline}; minimum {versions['minimum_version']}, recommended "
            f"{versions['recommended_version']}. Upgrade: {upgrade}"
        )
    else:
        message = (
            f"Waddle SDK {versions['current_version']} is no longer accepted; "
            f"minimum {versions['minimum_version']}, recommended "
            f"{versions['recommended_version']} (enforcement began {deadline}). "
            f"Upgrade: {upgrade}"
        )
    return message[:_CONNECTOR_COMPATIBILITY_MESSAGE_MAX_CHARS]


class ManifestError(ValueError):
    """Base class for site manifest refusals."""


class ManifestSyntaxError(ManifestError):
    """The manifest is not valid JSON/YAML."""


class ManifestValidationError(ManifestError):
    """The manifest does not satisfy the strict v1 schema."""


class ManifestPathError(ManifestError):
    """A site-owned path is absolute or escapes the site directory."""


def _load_document(path: Path) -> object:
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise ManifestSyntaxError(f"cannot read site manifest {path}: {exc}") from exc
    try:
        if path.suffix.lower() == ".json":
            return json.loads(text)
        import yaml

        return yaml.safe_load(text)
    except Exception as exc:  # parser exceptions are dependency-specific
        raise ManifestSyntaxError(f"invalid site manifest {path}: {exc}") from exc


def _schema() -> dict[str, Any]:
    resource = files("waddle_sdk").joinpath("schemas/site-v1.schema.json")
    return json.loads(resource.read_text(encoding="utf-8"))


def _validate(document: object) -> dict[str, Any]:
    try:
        import jsonschema
    except ModuleNotFoundError as exc:  # defensive for incomplete source installs
        raise RuntimeError(
            "site manifests require the waddle-sdk manifest dependencies; "
            "reinstall waddle-sdk"
        ) from exc
    validator = jsonschema.Draft202012Validator(_schema())
    errors = sorted(validator.iter_errors(document), key=lambda error: list(error.path))
    if errors:
        error = errors[0]
        location = ".".join(str(item) for item in error.absolute_path) or "<root>"
        raise ManifestValidationError(f"{location}: {error.message}")
    assert isinstance(document, dict)
    return document


def _validate_static_envelope(document: Mapping[str, Any]) -> None:
    envelope = document["envelope"]
    part_names = set(document["parts"])
    seen: set[str] = set()
    for index, keepout in enumerate(envelope["static_keepouts"]):
        identifier = str(keepout["id"])
        if identifier in seen:
            raise ManifestValidationError(
                f"envelope.static_keepouts[{index}].id: duplicate {identifier!r}"
            )
        seen.add(identifier)
        unknown = set(keepout.get("parts", ())) - part_names
        if unknown:
            raise ManifestValidationError(
                f"envelope.static_keepouts[{index}].parts: unknown parts "
                f"{sorted(unknown)!r}"
            )
        numeric = [float(keepout.get("margin_m", 0.0))]
        if keepout["kind"] == "box":
            lower = np.asarray(keepout["min"], dtype=float)
            upper = np.asarray(keepout["max"], dtype=float)
            numeric.extend(lower)
            numeric.extend(upper)
            if np.any(lower > upper):
                raise ManifestValidationError(
                    f"envelope.static_keepouts[{index}]: box min must not exceed max"
                )
        else:
            numeric.extend(float(value) for value in keepout["center"])
            numeric.append(float(keepout["radius_m"]))
        if not np.all(np.isfinite(numeric)):
            raise ManifestValidationError(
                f"envelope.static_keepouts[{index}]: coordinates and sizes must be finite"
            )

    self_collision = envelope["self_collision"]
    unknown = set(self_collision.get("parts", ())) - part_names
    if unknown:
        raise ManifestValidationError(
            f"envelope.self_collision.parts: unknown parts {sorted(unknown)!r}"
        )
    margin = float(self_collision.get("margin_m", 0.0))
    if not np.isfinite(margin):
        raise ManifestValidationError("envelope.self_collision.margin_m must be finite")
    for index, pair in enumerate(self_collision.get("ignore_pairs", ())):
        if pair[0] == pair[1]:
            raise ManifestValidationError(
                f"envelope.self_collision.ignore_pairs[{index}] needs distinct bodies"
            )


def _validate_gripper_geometry(document: Mapping[str, Any]) -> None:
    """Validate the optional hardware-neutral grasp geometry as one fact set."""

    geometry_fields = (
        "closing_axis_tcp",
        "pinch_offset_tcp_m",
        "pointing_down_wxyz",
    )
    for part_name, part in document["parts"].items():
        gripper = part.get("gripper")
        if not isinstance(gripper, Mapping):
            continue
        present = tuple(name for name in geometry_fields if name in gripper)
        if present and len(present) != len(geometry_fields):
            missing = sorted(set(geometry_fields) - set(present))
            raise ManifestValidationError(
                f"parts.{part_name}.gripper: grasp geometry is incomplete; "
                f"missing {missing!r}"
            )
        if not present:
            continue
        closing_axis = np.asarray(gripper["closing_axis_tcp"], dtype=float)
        pinch_offset = np.asarray(gripper["pinch_offset_tcp_m"], dtype=float)
        pointing_down = np.asarray(gripper["pointing_down_wxyz"], dtype=float)
        if not (
            np.all(np.isfinite(closing_axis))
            and np.all(np.isfinite(pinch_offset))
            and np.all(np.isfinite(pointing_down))
        ):
            raise ManifestValidationError(
                f"parts.{part_name}.gripper: grasp geometry must be finite"
            )
        if not np.isclose(float(np.linalg.norm(closing_axis)), 1.0, atol=1e-6):
            raise ManifestValidationError(
                f"parts.{part_name}.gripper.closing_axis_tcp must be a unit vector"
            )
        if not np.isclose(float(np.linalg.norm(pointing_down)), 1.0, atol=1e-6):
            raise ManifestValidationError(
                f"parts.{part_name}.gripper.pointing_down_wxyz must be a unit quaternion"
            )


def _site_path(root: Path, value: object, field_name: str) -> Path:
    if not isinstance(value, str) or not value:
        raise ManifestPathError(f"{field_name} must be a non-empty relative path")
    if "\x00" in value or "\\" in value:
        raise ManifestPathError(f"{field_name} must be a portable relative path")
    relative = Path(value)
    if relative.is_absolute() or any(part == ".." for part in relative.parts):
        raise ManifestPathError(f"{field_name} must stay beneath {root}")
    resolved_root = root.resolve()
    resolved = (root / relative).resolve(strict=False)
    try:
        resolved.relative_to(resolved_root)
    except ValueError as exc:
        raise ManifestPathError(f"{field_name} escapes {root}") from exc
    return resolved


_SECRET_KEYS = ("secret", "token", "password", "api_key", "credential")


def _validate_secret_references(value: object, path: str = "") -> None:
    if isinstance(value, Mapping):
        if set(value) == {"secret"}:
            name = value.get("secret")
            if not isinstance(name, str) or not name:
                raise ManifestValidationError(
                    f"{path or '<root>'}: secret references require a non-empty name"
                )
            return
        for key, item in value.items():
            location = f"{path}.{key}" if path else str(key)
            lowered = str(key).lower()
            if any(marker in lowered for marker in _SECRET_KEYS):
                if not (
                    isinstance(item, Mapping)
                    and set(item) == {"secret"}
                    and isinstance(item.get("secret"), str)
                    and bool(item["secret"])
                ):
                    raise ManifestValidationError(
                        f"{location}: secrets must use a named {{secret: NAME}} reference"
                    )
            _validate_secret_references(item, location)
    elif isinstance(value, list):
        for index, item in enumerate(value):
            _validate_secret_references(item, f"{path}[{index}]")


def _resolve_secrets(
    value: object,
    resolver: Mapping[str, str] | Callable[[str], str] | None,
) -> object:
    if isinstance(value, Mapping):
        if set(value) == {"secret"} and isinstance(value.get("secret"), str):
            name = value["secret"]
            if resolver is None:
                raise ManifestValidationError(
                    f"secret {name!r} is required but Site.open received no resolver"
                )
            try:
                secret = resolver(name) if callable(resolver) else resolver[name]
            except (KeyError, TypeError) as exc:
                raise ManifestValidationError(
                    f"secret {name!r} is unavailable"
                ) from exc
            if not isinstance(secret, str) or not secret:
                raise ManifestValidationError(
                    f"secret {name!r} resolved to an empty value"
                )
            return secret
        return {
            str(key): _resolve_secrets(item, resolver) for key, item in value.items()
        }
    if isinstance(value, list):
        return [_resolve_secrets(item, resolver) for item in value]
    return value


def _driver_target(spec: str, *, camera: bool = False):
    if ":" in spec:
        module_name, attribute = spec.split(":", 1)
    else:
        module_name = spec
        leaf = module_name.rsplit(".", 1)[-1]
        attribute = "".join(piece.capitalize() for piece in leaf.split("_")) + "Driver"
    if not module_name or not attribute:
        raise ManifestValidationError(f"driver {spec!r} must name module:attribute")
    try:
        module = importlib.import_module(module_name)
        try:
            target = getattr(module, attribute)
        except AttributeError:
            candidates = [
                getattr(module, name)
                for name in getattr(module, "__all__", ())
                if name.endswith("Driver") and callable(getattr(module, name, None))
            ]
            if len(candidates) != 1:
                raise
            target = candidates[0]
    except (ImportError, AttributeError) as exc:
        kind = "camera" if camera else "part"
        raise ManifestValidationError(
            f"cannot load {kind} driver {spec!r}: {exc}"
        ) from exc
    if not callable(target):
        raise ManifestValidationError(f"driver {spec!r} is not callable")
    return target


def _call_part_factory(target, config: PartConfig) -> base.Rig:
    parameters = inspect.signature(target).parameters
    if "config" in parameters:
        result = target(config=config)
    else:
        static_keepouts = config.envelope.get("static_keepouts", ())
        self_collision = config.envelope.get("self_collision", {})
        if static_keepouts or self_collision:
            raise ManifestValidationError(
                f"part {config.name!r} uses static envelope rules but its driver "
                "does not accept PartConfig; refusing rather than silently "
                "dropping owner safety"
            )
        kwargs = {
            **dict(config.options),
            **dict(config.connection),
            "posture": config.posture,
        }
        if config.joint_limits:
            limits = config.joint_limits
            kwargs["joint_limits"] = (
                list(limits.values()) if isinstance(limits, Mapping) else limits
            )
        bounds = config.workspace_bounds
        if bounds:
            kwargs["workspace"] = (bounds.get("min"), bounds.get("max"))
        try:
            result = target(**kwargs)
        except TypeError as exc:
            raise ManifestValidationError(
                f"part {config.name!r} driver arguments do not match its factory: {exc}"
            ) from exc
    if not isinstance(result, base.Rig):
        raise ManifestValidationError(
            f"part {config.name!r} driver must return waddle_sdk.robots.base.Rig"
        )
    return result


def _call_camera_factory(target, config: CameraConfig) -> CameraDriver:
    parameters = inspect.signature(target).parameters
    if "config" in parameters:
        result = target(config=config)
    else:
        kwargs = {
            **dict(config.options),
            **dict(config.connection),
            "width": int(config.stream["width"]),
            "height": int(config.stream["height"]),
            "fps": config.stream["fps"],
        }
        try:
            inspect.signature(target).bind(**kwargs)
        except TypeError as exc:
            raise ManifestValidationError(
                f"camera {config.name!r} driver arguments do not match its factory: {exc}"
            ) from exc
        result = target(**kwargs)
    if not isinstance(result, CameraDriver):
        raise ManifestValidationError(
            f"camera {config.name!r} driver must provide capture() and close()"
        )
    return result


def _camera_description(raw: Mapping[str, Any]) -> descriptors.Camera:
    stream = raw["stream"]
    intrinsics_raw = raw.get("intrinsics")
    intrinsics = None
    if intrinsics_raw is not None:
        intrinsics = descriptors.Intrinsics(
            fx=intrinsics_raw["fx"],
            fy=intrinsics_raw["fy"],
            cx=intrinsics_raw["cx"],
            cy=intrinsics_raw["cy"],
            distortion=tuple(intrinsics_raw.get("distortion", ())),
            depth_scale_mm=intrinsics_raw["depth_scale_mm"],
        )
    return descriptors.Camera(
        width=stream["width"],
        height=stream["height"],
        fps=stream["fps"],
        encoding=stream.get("encoding", "rgb8"),
        frame_id=raw.get("frame_id"),
        intrinsics=intrinsics,
        stream_policy=descriptors.StreamPolicy(
            local_full_rate=True, still_fps=stream.get("still_fps")
        ),
    )


def _combine_rigs(
    site_id: str,
    components: Mapping[str, base.Rig],
    camera_descriptions: Mapping[str, descriptors.Camera],
    camera_factories: Mapping[str, Callable[[], CameraDriver]],
    frames: Sequence[descriptors.FrameTransform],
    envelope: Mapping[str, Any],
) -> base.Rig:
    rates = {float(rig.rate_hz) for rig in components.values()}
    if len(rates) != 1:
        raise ManifestValidationError(
            "all site parts must currently declare the same control rate"
        )
    rate_hz = rates.pop()
    spaces = {name: rig.robot().action_space for name, rig in components.items()}
    nested = [
        name
        for name, space in spaces.items()
        if isinstance(space, descriptors.Composite)
    ]
    if nested:
        raise ManifestValidationError(
            "site part factories must return one bare action space, not Composite: "
            + ", ".join(nested)
        )
    component_frames = tuple(
        frame for rig in components.values() for frame in rig.robot().frames
    )

    def build_arms() -> dict[str, base.Arm]:
        opened: dict[str, base.Arm] = {}
        try:
            for name, rig in components.items():
                arms = rig.arms()
                if len(arms) != 1:
                    base.close_all(arms, report=rig.report)
                    raise ManifestValidationError(
                        f"part {name!r} factory returned a rig with {len(arms)} arms; "
                        "a site part factory must return exactly one"
                    )
                arm = next(iter(arms.values()))
                arm.part = name
                try:
                    arm.configure_static_safety(
                        static_keepouts=envelope.get("static_keepouts", ()),
                        self_collision=envelope.get("self_collision", {}),
                    )
                except (TypeError, ValueError) as exc:
                    arm.close()
                    raise ManifestValidationError(
                        f"part {name!r} cannot enforce the site envelope: {exc}"
                    ) from exc
                opened[name] = arm
            collision_frames = {
                arm.collision_frame
                for arm in opened.values()
                if arm.self_collision_enabled
            }
            if len(collision_frames) > 1:
                raise ManifestValidationError(
                    "self-collision across site parts requires one shared "
                    f"collision frame, got {sorted(collision_frames)!r}"
                )
        except BaseException:
            base.close_all(opened)
            raise
        return opened

    def build_cameras() -> dict[str, CameraDriver]:
        opened: dict[str, CameraDriver] = {}
        try:
            for name, factory in camera_factories.items():
                opened[name] = factory()
        except BaseException:
            base.close_all(opened)
            raise
        return opened

    declarations = [rig.robot() for rig in components.values()]
    kinematics = declarations[0].kinematics_urdf if len(declarations) == 1 else None
    declaration = descriptors.Robot(
        name=site_id,
        cell_id=site_id,
        action_space=descriptors.Composite(rate_hz=rate_hz, **spaces),
        cameras=dict(camera_descriptions),
        frames=component_frames + tuple(frames),
        kinematics_urdf=kinematics,
    )
    postures = {rig.posture for rig in components.values()}
    if len(postures) != 1:
        raise ManifestValidationError(
            "a site cannot mix monitor and supervised parts until the wire can "
            "declare send grants per part"
        )
    posture = postures.pop()
    return base.Rig(
        declaration=declaration,
        build_arms=build_arms,
        build_cameras=build_cameras if camera_factories else None,
        rate_hz=rate_hz,
        posture=posture,
        estop_hardware=all(rig.estop_hardware for rig in components.values()),
    )


@dataclass(frozen=True)
class Site:
    path: Path
    manifest: Mapping[str, Any]
    calibration_root: Path
    recording_root: Path | None

    @property
    def id(self) -> str:
        return self.manifest["metadata"]["id"]

    def describe(self) -> Mapping[str, JSONValue]:
        return MappingProxyType(json.loads(json.dumps(dict(self.manifest))))

    def open(
        self,
        *,
        transport=None,
        media=None,
        console: bool = True,
        secrets: Mapping[str, str] | Callable[[str], str] | None = None,
        _testing: bool = False,
        authorization_timeout_s: float = 15.0,
    ) -> "SiteSession":
        """Return an unopened session context; hardware opens in ``__enter__``."""
        return SiteSession(
            self,
            transport=transport,
            media=media,
            console=console,
            secrets=secrets,
            _testing=_testing,
            authorization_timeout_s=authorization_timeout_s,
        )

    def _rig(
        self, resolver: Mapping[str, str] | Callable[[str], str] | None
    ) -> base.Rig:
        raw = _resolve_secrets(self.manifest, resolver)
        assert isinstance(raw, Mapping)
        bounds = raw.get("workspace_bounds", {})
        envelope = raw.get("envelope", {})
        components: dict[str, base.Rig] = {}
        for name, part in raw["parts"].items():
            target = _driver_target(part["driver"])
            config = PartConfig(
                name=name,
                posture=part["posture"],
                connection=part["connection"],
                joint_limits=part.get("joint_limits", {}),
                workspace_bounds=bounds,
                envelope=envelope,
                options=part.get("options", {}),
                site_root=self.path.parent,
            )
            components[name] = _call_part_factory(target, config)

        camera_descriptions: dict[str, descriptors.Camera] = {}
        camera_factories: dict[str, Callable[[], CameraDriver]] = {}
        for name, camera in raw.get("cameras", {}).items():
            target = _driver_target(camera["driver"], camera=True)
            config = CameraConfig(
                name=name,
                connection=camera["connection"],
                stream=camera["stream"],
                frame_id=camera.get("frame_id"),
                intrinsics=camera.get("intrinsics"),
                options=camera.get("options", {}),
                site_root=self.path.parent,
            )
            camera_descriptions[name] = _camera_description(camera)
            camera_factories[name] = lambda target=target, config=config: (
                _call_camera_factory(target, config)
            )

        frame_descriptions = [
            descriptors.FrameTransform(
                parent=value["parent"],
                child=name,
                position=tuple(value.get("position", (0.0, 0.0, 0.0))),
                quaternion=tuple(value.get("quaternion_wxyz", (1.0, 0.0, 0.0, 0.0))),
            )
            for name, value in raw.get("frames", {}).items()
        ]
        return _combine_rigs(
            self.id,
            components,
            camera_descriptions,
            camera_factories,
            frame_descriptions,
            envelope,
        )


class SiteSession:
    def __init__(
        self,
        site: Site,
        *,
        transport,
        media,
        console,
        secrets,
        _testing,
        authorization_timeout_s,
    ):
        self.site = site
        self._transport = transport
        self._media = media
        self._console = console
        self._secrets = secrets
        self._testing = _testing
        if isinstance(authorization_timeout_s, bool) or not isinstance(
            authorization_timeout_s, (int, float)
        ):
            raise TypeError("authorization_timeout_s must be a positive number")
        if authorization_timeout_s <= 0:
            raise ValueError("authorization_timeout_s must be positive")
        self._authorization_timeout_s = float(authorization_timeout_s)
        self._managed: base.RigSession | None = None
        self._active: Run | None = None
        self._events: list[RuntimeEvent] = []
        self._event_lock = threading.Lock()
        self._service_stop = threading.Event()
        self._service_thread: threading.Thread | None = None

    def _authorize_connector(self, rig: base.Rig) -> None:
        transport = self._transport
        if not isinstance(transport, Grpc) or transport.customer_id is None:
            return
        if transport.authorization_only:
            raise ValueError(
                "Site.open requires a runnable connector transport, not an authorization-only probe"
            )
        probe_transport = replace(transport, authorization_only=True)
        probe = create_core_session(
            self.site.id,
            rig.robot(),
            Control(),
            transport=probe_transport,
        )
        deadline = time.monotonic() + self._authorization_timeout_s
        try:
            while time.monotonic() < deadline:
                status = dict(probe.status())
                error_code = status.get("connector_registration_error_code")
                if isinstance(error_code, str) and error_code:
                    safe_code = (
                        error_code
                        if len(error_code) <= 64
                        and re.fullmatch(r"[a-z0-9_]+", error_code) is not None
                        else "registration_failed"
                    )
                    detail = _connector_compatibility_message(
                        status.get("connector_registration_detail"),
                        code=safe_code,
                    )
                    if detail is None:
                        detail = (
                            "connector registration was rejected before hardware opened"
                        )
                    raise ConnectorRegistrationError(safe_code, detail)
                if status.get("connector_binding_refused"):
                    raise RuntimeError(
                        "the host registered without accepting waddle.v0.connector.binding; refusing to open hardware"
                    )
                if status.get("plane_registered"):
                    if not status.get("connector_binding_negotiated"):
                        raise RuntimeError(
                            "the host registered without accepting waddle.v0.connector.binding; refusing to open hardware"
                        )
                    notice = _connector_compatibility_message(
                        status.get("connector_registration_detail"),
                        code="upgrade_recommended",
                    )
                    if notice is not None:
                        warnings.warn(
                            notice,
                            ConnectorCompatibilityWarning,
                            stacklevel=3,
                        )
                    return
                time.sleep(0.02)
            raise TimeoutError(
                "connector authorization timed out before hardware was opened"
            )
        finally:
            probe.shutdown()

    def __enter__(self) -> "SiteSession":
        if self._managed is not None:
            raise RuntimeError("this SiteSession is already open")
        rig = self.site._rig(self._secrets)
        self._authorize_connector(rig)
        managed = base.RigSession(
            rig,
            self.site.id,
            transport=self._transport,
            media=self._media,
            recording_dir=self.site.recording_root,
            console=self._console,
            _testing=self._testing,
        )
        managed._open(create_core_session)
        self._managed = managed
        self._service_stop.clear()
        self._service_thread = threading.Thread(
            target=self._serve_calibration_requests,
            name=f"waddle-calibration-{self.site.id}",
            daemon=True,
        )
        self._service_thread.start()
        self._event("session.opened", {"site_id": self.site.id})
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        managed = self._managed
        try:
            if self._active is not None:
                self._active.__exit__(exc_type, exc, tb)
        finally:
            self._service_stop.set()
            service_thread = self._service_thread
            self._service_thread = None
            if service_thread is not None:
                service_thread.join(timeout=5.0)
            self._managed = None
            if managed is not None:
                managed.close(
                    interrupted=(
                        exc_type is not None and issubclass(exc_type, KeyboardInterrupt)
                    )
                )
            self._event("session.closed", {"site_id": self.site.id})
        return False

    def _serve_calibration_requests(self) -> None:
        cursor = 0
        while not self._service_stop.is_set():
            managed = self._managed
            if managed is None or managed.core is None:
                return
            try:
                rows = managed.core.calibration_measurement_requests(cursor, 250)
            except Exception as error:  # noqa: BLE001 -- keep local controls alive
                if self._service_stop.is_set():
                    return
                self._event(
                    "calibration.request_poll_failed",
                    _event_fault("poll calibration requests", error),
                )
                self._service_stop.wait(0.1)
                continue
            for row in rows:
                cursor = max(cursor, int(row["cursor"]))
                if self._service_stop.is_set():
                    return
                try:
                    self.calibration_measurement(
                        calibration_id=str(row["calibration_id"]),
                        sample_id=str(row["sample_id"]),
                        camera=str(row["camera"]),
                        frame_sequence=int(row["frame_sequence"]),
                        x=int(row["x"]),
                        y=int(row["y"]),
                    )
                except Exception as error:  # noqa: BLE001 -- one bad click is isolated
                    self._event(
                        "calibration.measurement_refused",
                        {
                            "calibration_id": str(row["calibration_id"]),
                            "sample_id": str(row["sample_id"]),
                            **_event_fault(
                                "submit calibration measurement",
                                error,
                                context={
                                    "camera": str(row["camera"]),
                                    "frame_sequence": int(row["frame_sequence"]),
                                },
                            ),
                        },
                    )

    def _require(self) -> base.RigSession:
        if self._managed is None or self._managed.core is None:
            raise RuntimeFault(FaultCode.NOT_OPEN, "SiteSession is not open")
        return self._managed

    def _event(self, kind: str, data: Mapping[str, JSONValue]) -> RuntimeEvent:
        managed = self._managed
        session_ns = 0
        if managed is not None and managed.core is not None:
            session_ns = int(managed.core.stamp().session_ns)
        with self._event_lock:
            event = RuntimeEvent(len(self._events) + 1, kind, session_ns, dict(data))
            self._events.append(event)
        return event

    def describe(self) -> Mapping[str, JSONValue]:
        description = dict(self.site.describe())
        managed = self._managed
        if managed is not None:
            try:
                description["runtime"] = dict(managed.core.status())
                description["robot"] = self._registered_robot_description(managed)
                description["support"] = self.support().as_dict()
            except RuntimeFault:
                raise
            except Exception as exc:
                raise _operation_fault("describe SDK runtime", exc) from exc
        return description

    def _registered_robot_description(
        self, managed: base.RigSession
    ) -> dict[str, JSONValue]:
        control = managed.control
        if control is None:
            raise RuntimeFault(
                FaultCode.NOT_OPEN,
                "the opened session has not registered its control verbs",
            )
        return managed.robot._compile(
            _derive_grants(control, managed.robot.action_space)
        )

    @staticmethod
    def _digest(payload: Mapping[str, object]) -> str:
        canonical = json.dumps(
            payload,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
        ).encode("utf-8")
        return hashlib.sha256(canonical).hexdigest()

    def _composite_embodiment_digest(
        self,
        robot: Mapping[str, JSONValue],
        base_frames: Mapping[str, str],
    ) -> str:
        parts = self.site.manifest["parts"]
        grippers = {
            str(name): dict(part["gripper"])
            for name, part in parts.items()
            if "gripper" in part
        }
        payload = {
            "contractVersion": SUPPORT_CONTRACT_VERSION,
            "actionSpace": robot["actionSpace"],
            "kinematicsUrdf": robot.get("kinematicsUrdf"),
            "frames": robot.get("frames"),
            "grippers": grippers,
            "cameras": robot.get("cameras", []),
            "baseFrames": dict(sorted(base_frames.items())),
        }
        return self._digest(payload)

    @staticmethod
    def _relevant_frames(
        robot: Mapping[str, JSONValue], action_space: Mapping[str, JSONValue]
    ) -> dict[str, JSONValue]:
        action_frames: set[str] = set()

        def collect(value: object) -> None:
            if isinstance(value, Mapping):
                for key, item in value.items():
                    if key == "frameId" and isinstance(item, str) and item:
                        action_frames.add(item)
                    else:
                        collect(item)
            elif isinstance(value, list):
                for item in value:
                    collect(item)

        collect(action_space)
        if not action_frames:
            return {}
        graph = robot.get("frames")
        if not isinstance(graph, Mapping):
            return {}
        transforms = graph.get("transforms", [])
        if not isinstance(transforms, list):
            return {}
        relevant_ids: set[int] = set()
        frontier = set(action_frames)
        while frontier:
            child = frontier.pop()
            for index, transform in enumerate(transforms):
                if (
                    not isinstance(transform, Mapping)
                    or transform.get("child") != child
                ):
                    continue
                if index in relevant_ids:
                    continue
                relevant_ids.add(index)
                parent = transform.get("parent")
                if isinstance(parent, str) and parent:
                    frontier.add(parent)
        relevant = [
            transform
            for index, transform in enumerate(transforms)
            if index in relevant_ids
        ]
        return {"transforms": relevant} if relevant else {}

    def _robot_embodiment_digest(
        self,
        *,
        robot: Mapping[str, JSONValue],
        action_space: Mapping[str, JSONValue],
        gripper: Mapping[str, object] | None,
        base_frame: str,
    ) -> str:
        return self._digest(
            {
                "contractVersion": SUPPORT_CONTRACT_VERSION,
                "actionSpace": action_space,
                "kinematicsUrdf": robot.get("kinematicsUrdf"),
                "frames": self._relevant_frames(robot, action_space),
                "gripper": None if gripper is None else dict(gripper),
                "baseFrame": base_frame,
            }
        )

    def _camera_embodiment_digest(
        self, camera: Mapping[str, JSONValue]
    ) -> str:
        return self._digest(
            {
                "contractVersion": SUPPORT_CONTRACT_VERSION,
                "camera": camera,
            }
        )

    @staticmethod
    def _grant_facts(grants: Sequence[Mapping[str, JSONValue]]) -> set[SupportFact]:
        by_verb = {
            "VERB_SEND": SupportFact.SEND_GRANT,
            "VERB_HOLD": SupportFact.HOLD_GRANT,
            "VERB_RESUME": SupportFact.RESUME_GRANT,
            "VERB_HOME": SupportFact.HOME_GRANT,
            "VERB_ESTOP": SupportFact.ESTOP_GRANT,
        }
        return {
            fact
            for grant in grants
            if (fact := by_verb.get(str(grant.get("verb", "")))) is not None
        }

    @staticmethod
    def _part_space_descriptions(
        action_space: Mapping[str, JSONValue],
        part_names: Sequence[str],
    ) -> dict[str, Mapping[str, JSONValue]]:
        composite = action_space.get("composite")
        if isinstance(composite, Mapping):
            raw_parts = composite.get("parts", [])
            if isinstance(raw_parts, list):
                return {
                    str(row["name"]): row["space"]
                    for row in raw_parts
                    if isinstance(row, Mapping)
                    and isinstance(row.get("name"), str)
                    and isinstance(row.get("space"), Mapping)
                }
        if len(part_names) == 1:
            return {str(part_names[0]): action_space}
        return {}

    def support(self) -> SupportMatrix:
        """Return conservative support facts for this opened hardware session.

        Facts refine the registered action space and grants; they never widen
        either one. In particular, camera depth is not advertised because v0
        has no stable aligned-depth declaration.
        """
        managed = self._require()
        robot = self._registered_robot_description(managed)
        action_space = robot["actionSpace"]
        grants = robot.get("grants", [])
        if not isinstance(action_space, Mapping) or not isinstance(grants, list):
            raise RuntimeFault(
                FaultCode.INTERNAL,
                "the registered robot description has malformed support inputs",
            )
        base_frames = {name: arm.base_frame for name, arm in managed.arms.items()}
        digest = self._composite_embodiment_digest(robot, base_frames)
        grant_facts = self._grant_facts(grants)
        part_spaces = self._part_space_descriptions(
            action_space, tuple(managed.arms)
        )
        manifest_parts = self.site.manifest["parts"]
        rows: list[SupportRow] = []
        for name, arm in managed.arms.items():
            facts = {
                SupportFact.JOINT_POSITION_OBSERVATION,
                SupportFact.JOINT_VELOCITY_OBSERVATION,
                *grant_facts,
            }
            part_space = part_spaces.get(name, {})
            joint_position = part_space.get("jointPosition")
            if isinstance(joint_position, Mapping):
                facts.add(SupportFact.JOINT_POSITION_ACTION)
                joints = joint_position.get("joints", [])
                if isinstance(joints, list) and joints:
                    if all(
                        isinstance(joint, Mapping)
                        and "minPosition" in joint
                        and "maxPosition" in joint
                        for joint in joints
                    ):
                        facts.add(SupportFact.POSITION_LIMITS)
                    if all(
                        isinstance(joint, Mapping) and "maxVelocity" in joint
                        for joint in joints
                    ):
                        facts.add(SupportFact.VELOCITY_LIMITS)
            if isinstance(arm.driver, base.PositionVelocityDriver):
                facts.add(SupportFact.VELOCITY_FEEDFORWARD)
            if arm.fk is not None:
                facts.update(
                    (SupportFact.EE_POSE_OBSERVATION, SupportFact.FORWARD_KINEMATICS)
                )
            if arm.base_frame:
                facts.add(SupportFact.BASE_FRAME)
            if arm.collision_spheres is not None and arm.collision_frame:
                facts.add(SupportFact.BODY_SPHERES)
            if arm.workspace is not None:
                facts.add(SupportFact.WORKSPACE_BOUNDS)
            if "kinematicsUrdf" in robot:
                facts.add(SupportFact.URDF_MODEL)
            manifest_part = manifest_parts.get(name, {})
            gripper = (
                manifest_part.get("gripper")
                if isinstance(manifest_part, Mapping)
                else None
            )
            if isinstance(gripper, Mapping):
                facts.add(SupportFact.GRIPPER_MAPPING)
                geometry_fields = (
                    "closing_axis_tcp",
                    "pinch_offset_tcp_m",
                    "pointing_down_wxyz",
                )
                if all(field in gripper for field in geometry_fields):
                    facts.add(SupportFact.GRIPPER_GEOMETRY)
            rows.append(
                SupportRow(
                    scope=f"robot:{name}",
                    embodiment_digest=self._robot_embodiment_digest(
                        robot=robot,
                        action_space=part_space,
                        gripper=gripper if isinstance(gripper, Mapping) else None,
                        base_frame=arm.base_frame,
                    ),
                    facts=tuple(facts),
                )
            )

        public_cameras = {
            str(camera["name"]): camera
            for camera in robot.get("cameras", [])
            if isinstance(camera, Mapping) and isinstance(camera.get("name"), str)
        }
        for name, description in managed.robot.cameras.items():
            facts = {SupportFact.CAMERA_RGB}
            if description.intrinsics is not None:
                facts.add(SupportFact.CAMERA_INTRINSICS)
            public_camera = public_cameras.get(name)
            if public_camera is None:
                raise RuntimeFault(
                    FaultCode.INTERNAL,
                    f"camera {name!r} is missing from the registered description",
                )
            rows.append(
                SupportRow(
                    scope=f"camera:{name}",
                    embodiment_digest=self._camera_embodiment_digest(public_camera),
                    facts=tuple(facts),
                )
            )
        return SupportMatrix(
            contract_version=SUPPORT_CONTRACT_VERSION,
            embodiment_digest=digest,
            action_space=action_space,
            grants=tuple(grants),
            rows=tuple(rows),
        )

    @staticmethod
    def _joint_vector(arm: base.Arm, values: Sequence[float]) -> np.ndarray:
        try:
            vector = np.asarray(values, dtype=float).reshape(-1)
        except (TypeError, ValueError) as exc:
            raise RuntimeFault(
                FaultCode.INVALID_REQUEST,
                "joint_position must be a finite declared-width vector",
            ) from exc
        if vector.size != len(arm.joint_names) or not np.all(np.isfinite(vector)):
            raise RuntimeFault(
                FaultCode.INVALID_REQUEST,
                "joint_position must be a finite declared-width vector",
                context={"expected_width": len(arm.joint_names)},
            )
        return vector

    def forward_kinematics(
        self, part: str, joint_position: Sequence[float]
    ) -> Pose:
        """Evaluate an opened part's hardware-specific FK implementation."""
        managed = self._require()
        arm = managed.arms.get(part)
        if arm is None:
            raise RuntimeFault(FaultCode.NOT_FOUND, f"part {part!r} is not declared")
        if arm.fk is None or not arm.base_frame:
            raise RuntimeFault(
                FaultCode.UNSUPPORTED,
                f"part {part!r} provides no frame-tagged forward kinematics",
            )
        vector = self._joint_vector(arm, joint_position)
        try:
            position, rotation = arm.fk(vector[: arm.arm_dof])
            position_values = np.asarray(position, dtype=float)
            rotation_values = np.asarray(rotation, dtype=float)
            if position_values.shape != (3,) or rotation_values.shape != (3, 3):
                raise ValueError("expected xyz and a 3x3 rotation matrix")
            quaternion = base.quaternion_wxyz(rotation_values)
            return Pose(
                position_m=tuple(float(value) for value in position_values),
                quaternion_wxyz=tuple(float(value) for value in quaternion),
                frame_id=arm.base_frame,
            )
        except RuntimeFault:
            raise
        except Exception as exc:
            raise _operation_fault(
                "forward kinematics",
                exc,
                context={"part": part},
            ) from exc

    def body_geometry(
        self, part: str, joint_position: Sequence[float]
    ) -> tuple[BodySphere, ...]:
        """Evaluate an opened part's conservative named body geometry."""
        managed = self._require()
        arm = managed.arms.get(part)
        if arm is None:
            raise RuntimeFault(FaultCode.NOT_FOUND, f"part {part!r} is not declared")
        if arm.collision_spheres is None or not arm.collision_frame:
            raise RuntimeFault(
                FaultCode.UNSUPPORTED,
                f"part {part!r} provides no frame-tagged body geometry",
            )
        vector = self._joint_vector(arm, joint_position)
        try:
            return tuple(
                BodySphere(
                    name=sphere.name,
                    center_m=tuple(float(value) for value in sphere.center_m),
                    radius_m=sphere.radius_m,
                    frame_id=arm.collision_frame,
                )
                for sphere in arm.collision_snapshot(vector)
            )
        except RuntimeFault:
            raise
        except Exception as exc:
            raise _operation_fault(
                "body geometry",
                exc,
                context={"part": part},
            ) from exc

    def run(self, *, task, actor) -> "Run":
        return Run(self, task=task, actor=actor)

    def begin_run(self, *, task, actor) -> "Run":
        run = self.run(task=task, actor=actor)
        run.__enter__()
        return run

    def observe(self) -> Observation:
        managed = self._require()
        parts: dict[str, PartObservation] = {}
        for name, arm in managed.arms.items():
            try:
                position, velocity = arm.state()
                pose = arm.ee_pose(position)
            except RuntimeFault:
                raise
            except Exception as exc:
                raise _operation_fault(
                    "observe robot part",
                    exc,
                    context={"part": name},
                ) from exc
            parts[name] = PartObservation(
                joint_position=np.asarray(position, dtype=np.float64),
                joint_velocity=np.asarray(velocity, dtype=np.float64),
                ee_pose_wxyz=None
                if pose is None
                else np.asarray(pose, dtype=np.float64),
                frame_id=None if pose is None else arm.base_frame,
            )
        cameras = {
            name: sample
            for name in managed.robot.cameras
            if (sample := managed.camera_sample(name)) is not None
        }
        # Stamp the composite envelope after taking its constituent snapshots.
        # Camera pumps update concurrently; stamping first can make a frame
        # captured during assembly appear to come from the observation's future.
        try:
            stamp = managed.core.stamp()
        except RuntimeFault:
            raise
        except Exception as exc:
            raise _operation_fault("stamp observation", exc) from exc
        return Observation(stamp.session_ns, stamp.unix_ns, parts, cameras)

    def submit(self, action, observation=None) -> SubmitResult:
        if self._active is None:
            raise RuntimeFault(FaultCode.NOT_OPEN, "no run is active")
        return self._active.step(action, observation)

    def hold(self, reason: str) -> None:
        managed = self._require()
        if not isinstance(reason, str) or not reason:
            raise ValueError("hold reason must be a non-empty string")
        try:
            managed.core.request_hold(reason)
        except AttributeError as exc:
            raise RuntimeFault(
                FaultCode.UNSUPPORTED,
                "the installed native core does not provide the core-owned hold path",
            ) from exc
        except RuntimeFault:
            raise
        except (TypeError, ValueError) as exc:
            raise _operation_fault(
                "request hold", exc, code=FaultCode.INVALID_REQUEST
            ) from exc
        except Exception as exc:
            raise _operation_fault("request hold", exc) from exc
        self._event("control.hold", {"reason": reason})

    def estop(self, reason: str) -> None:
        managed = self._require()
        if not isinstance(reason, str) or not reason:
            raise ValueError("e-stop reason must be a non-empty string")
        try:
            managed.core.request_estop()
        except RuntimeFault:
            raise
        except Exception as exc:
            raise _operation_fault("request e-stop", exc) from exc
        self._event("control.estop", {"reason": reason})

    def events(self, after_cursor: int = 0) -> tuple[RuntimeEvent, ...]:
        if isinstance(after_cursor, bool) or not isinstance(after_cursor, int):
            raise TypeError("after_cursor must be an integer")
        if after_cursor < 0:
            raise ValueError("after_cursor must be non-negative")
        with self._event_lock:
            return tuple(event for event in self._events if event.cursor > after_cursor)

    def calibration_measurement(
        self,
        *,
        calibration_id: str,
        sample_id: str,
        camera: str,
        frame_sequence: int,
        x: int,
        y: int,
    ) -> Mapping[str, JSONValue]:
        managed = self._require()
        if camera not in managed.robot.cameras:
            raise RuntimeFault(
                FaultCode.NOT_FOUND,
                f"camera {camera!r} is not declared",
                context={"camera": camera},
            )
        sample = managed.camera_sample(camera)
        if sample is None:
            raise RuntimeFault(FaultCode.NOT_FOUND, f"camera {camera!r} has no sample")
        try:
            point = managed.resolve_pixel(camera, x, y, frame_sequence=frame_sequence)
        except RuntimeFault:
            raise
        except (TypeError, ValueError) as exc:
            raise _operation_fault(
                "resolve calibration pixel",
                exc,
                code=FaultCode.INVALID_REQUEST,
                context={"camera": camera, "frame_sequence": frame_sequence},
            ) from exc
        except RuntimeError as exc:
            raise _operation_fault(
                "resolve calibration frame",
                exc,
                code=FaultCode.CONFLICT,
                context={"camera": camera, "frame_sequence": frame_sequence},
            ) from exc
        description = managed.robot.cameras[camera]
        frame_id = description.frame_id or camera
        try:
            if managed.core.status().get("calibration_measurements_negotiated", False):
                managed.core.calibration_measurement_submit(
                    calibration_id,
                    sample_id,
                    camera,
                    frame_sequence,
                    frame_id,
                    sample.session_ns,
                    point,
                    point[2],
                )
        except RuntimeFault:
            raise
        except (TypeError, ValueError) as exc:
            raise _operation_fault(
                "submit calibration measurement",
                exc,
                code=FaultCode.INVALID_REQUEST,
                context={"camera": camera, "frame_sequence": frame_sequence},
            ) from exc
        except Exception as exc:
            raise _operation_fault(
                "submit calibration measurement",
                exc,
                context={"camera": camera, "frame_sequence": frame_sequence},
            ) from exc
        result: dict[str, JSONValue] = {
            "calibration_id": calibration_id,
            "sample_id": sample_id,
            "camera": camera,
            "frame_sequence": frame_sequence,
            "session_ns": sample.session_ns,
            "frame_id": frame_id,
            "point_xyz": [float(value) for value in point],
            "depth_m": float(point[2]),
        }
        self._event("calibration.measurement", result)
        return MappingProxyType(result)


class Run:
    def __init__(self, session: SiteSession, *, task, actor):
        self._session = session
        self._task, self._task_metadata = _task(task, actor)
        self._episode = None

    @property
    def id(self) -> str:
        if self._episode is None:
            raise RuntimeFault(FaultCode.NOT_OPEN, "run has not started")
        return self._episode.id

    @property
    def done(self) -> bool:
        return self._episode is not None and bool(self._episode.done)

    @property
    def outcome(self) -> str | None:
        return None if self._episode is None else self._episode.outcome

    def __enter__(self) -> "Run":
        managed = self._session._require()
        if self._session._active is not None:
            raise RuntimeFault(FaultCode.BUSY, "another run is active")
        try:
            self._episode = managed.core.start_episode(
                self._task, task_metadata=self._task_metadata
            )
        except RuntimeFault:
            raise
        except (TypeError, ValueError) as exc:
            raise _operation_fault(
                "start run", exc, code=FaultCode.INVALID_REQUEST
            ) from exc
        except Exception as exc:
            raise _operation_fault("start run", exc) from exc
        self._session._active = self
        self._session._event("run.started", {"run_id": self.id, "task": self._task})
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        try:
            if self._episode is not None and not self._episode.done:
                reason = (
                    "run exited before a terminal outcome"
                    if exc_type is None
                    else f"unhandled {_exception_type(exc)}"
                )
                try:
                    self._episode.terminate("abort", reason)
                except RuntimeFault:
                    raise
                except Exception as terminate_error:
                    raise _operation_fault(
                        "abort run", terminate_error
                    ) from terminate_error
            if self._episode is not None:
                self._session._event(
                    "run.finished",
                    {"run_id": self.id, "outcome": self._episode.outcome or "abort"},
                )
        finally:
            if self._session._active is self:
                self._session._active = None
        return False

    def observe(self) -> Observation:
        if self._episode is None:
            raise RuntimeFault(FaultCode.NOT_OPEN, "run has not started")
        return self._session.observe()

    def step(self, action, observation=None) -> SubmitResult:
        if self._episode is None:
            raise RuntimeFault(FaultCode.NOT_OPEN, "run has not started")
        if self._episode.done:
            raise RuntimeFault(FaultCode.CONFLICT, "run is already terminal")
        velocity_feedforward = None
        gate_action = action
        if isinstance(action, JointPositionCommand):
            gate_action = np.asarray(action.positions, dtype=np.float64)
            if action.velocity_feedforward_rad_s is not None:
                velocity_feedforward = np.asarray(
                    action.velocity_feedforward_rad_s, dtype=np.float64
                )
        if isinstance(observation, Observation):
            obs = observation.gate_vector()
        else:
            obs = observation
        try:
            decided = self._episode.gate(gate_action, obs)
        except RuntimeFault:
            raise
        except (TypeError, ValueError) as exc:
            raise _operation_fault(
                "validate run action",
                exc,
                code=FaultCode.INVALID_REQUEST,
            ) from exc
        except Exception as exc:
            raise _operation_fault("gate run action", exc) from exc
        gate = self._episode.last_gate
        kind = "pass" if gate is None else str(gate.kind)
        part = None if gate is None else gate.part
        dispatched = decided is not None
        detail = ""
        if kind != "pass":
            # A non-pass action belongs to the core's selected stream, never
            # to the caller. Clear the caller's hint even when the selected
            # action has none, or one policy's velocity would ride another
            # policy's position target.
            velocity_feedforward = None
        if gate is not None and gate.velocity_feedforward is not None:
            velocity_feedforward = np.asarray(
                gate.velocity_feedforward, dtype=np.float64
            )
            if isinstance(decided, dict):
                velocity_feedforward = base.split_by_part(
                    self._session._require().arms, velocity_feedforward
                )
        if dispatched:
            try:
                dispatched = base.apply_decision(
                    self._session._require().arms,
                    decided,
                    # The value now belongs to the exact action the core selected:
                    # caller on pass, selected stream on substitute/anchorless
                    # blend, absent on an actually interpolated blend.
                    velocity_feedforward_rad_s=velocity_feedforward,
                )
            except RuntimeFault:
                raise
            except (TypeError, ValueError) as exc:
                raise _operation_fault(
                    "validate dispatched action",
                    exc,
                    code=FaultCode.INVALID_REQUEST,
                ) from exc
            except Exception as exc:
                raise _operation_fault("dispatch robot action", exc) from exc
            if not dispatched:
                kind = "owner_refusal"
                detail = "the owner envelope refused the complete action"
        result = SubmitResult(
            dispatched=dispatched, gate=kind, part=part, detail=detail
        )
        self._session._event(
            "run.step",
            {"run_id": self.id, "dispatched": dispatched, "gate": kind, "part": part},
        )
        return result

    def hold(self, reason: str) -> None:
        self._session.hold(reason)

    def finish(self, outcome: str, reason: str = "") -> None:
        if self._episode is None:
            raise RuntimeFault(FaultCode.NOT_OPEN, "run has not started")
        if outcome not in {"success", "failure", "abort"}:
            raise ValueError("outcome must be success, failure, or abort")
        try:
            self._episode.terminate(outcome, reason)
        except RuntimeFault:
            raise
        except Exception as exc:
            raise _operation_fault("finish run", exc) from exc


def _task(task: object, actor: object) -> tuple[str, dict[str, str]]:
    if isinstance(task, str):
        if not task:
            raise ValueError("task must be non-empty")
        label = task
        task_value: object = {"text": task}
    elif isinstance(task, Mapping):
        task_value = dict(task)
        candidate = (
            task_value.get("text") or task_value.get("name") or task_value.get("id")
        )
        label = (
            candidate if isinstance(candidate, str) and candidate else "structured task"
        )
    else:
        raise TypeError("task must be a string or mapping")
    if isinstance(actor, str):
        if not actor:
            raise ValueError("actor must be non-empty")
        actor_value: object = {"id": actor}
    elif isinstance(actor, Mapping):
        actor_value = dict(actor)
    else:
        raise TypeError("actor must be a string or mapping")
    return label, {
        "waddle.task": json.dumps(task_value, sort_keys=True, separators=(",", ":")),
        "waddle.actor": json.dumps(actor_value, sort_keys=True, separators=(",", ":")),
    }


def load_site(path: str | Path) -> Site:
    manifest_path = Path(path).expanduser().resolve()
    document = _validate(_load_document(manifest_path))
    _validate_secret_references(document)
    _validate_static_envelope(document)
    _validate_gripper_geometry(document)
    root = manifest_path.parent
    calibration_root = _site_path(
        root,
        document.get("calibration", {}).get("artifacts", "calib/"),
        "calibration.artifacts",
    )
    recording_raw = document.get("recording")
    recording_root = (
        None
        if recording_raw is None
        else _site_path(root, recording_raw.get("root", "data/"), "recording.root")
    )
    return Site(
        path=manifest_path,
        manifest=MappingProxyType(document),
        calibration_root=calibration_root,
        recording_root=recording_root,
    )


__all__ = [
    "ConnectorCompatibilityWarning",
    "ConnectorRegistrationError",
    "ManifestError",
    "ManifestPathError",
    "ManifestSyntaxError",
    "ManifestValidationError",
    "Run",
    "Site",
    "SiteSession",
    "load_site",
]
