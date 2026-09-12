"""Public contracts for manifest-selected, shared simulation worlds.

A simulation backend is an ordinary installable Python package named by a
``site.yaml`` world.  Its factory and ``part`` declarations are non-opening;
the SDK opens the world only after connector authorization and closes it after
all world-backed arms and cameras. Applications consuming the ordinary
:class:`~waddle_sdk.runtime.SdkRuntimePort` need no simulator-specific imports.
"""

from __future__ import annotations

import copy
import hashlib
import importlib
import inspect
import json
import math
import threading
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from importlib import metadata
from pathlib import Path
from types import MappingProxyType
from typing import TYPE_CHECKING, Any, Protocol, runtime_checkable

if TYPE_CHECKING:
    from .cameras import CameraDriver
    from .cameras.site import CameraConfig
    from .robots import base
    from .robots.site import PartConfig


@dataclass(frozen=True)
class WorldConfig:
    """Manifest values supplied to one lazy simulation-backend factory."""

    name: str
    connection: Mapping[str, Any]
    options: Mapping[str, Any] = field(default_factory=dict)
    site_root: Path = Path(".")


@runtime_checkable
class SimulationBackend(Protocol):
    """Lifecycle and clock surface required for every simulated world.

    ``open`` creates the backend process, physics state, or renderer.  ``step``
    is called once per SDK composite tick, regardless of how many parts share
    the world.  An asynchronously clocked backend may implement it as a
    validated no-op.  ``reset`` runs once before the ordinary per-arm reset and
    may refuse by returning ``False``.  ``close`` must be idempotent.
    """

    def open(self) -> None: ...

    def step(self, dt: float) -> None: ...

    def reset(self) -> bool: ...

    def close(self) -> None: ...


@runtime_checkable
class SimulationPartBackend(Protocol):
    """Optional world facet for declaring one simulated robot part."""

    def part(self, *, config: PartConfig) -> base.Rig: ...


@runtime_checkable
class SimulationCameraBackend(Protocol):
    """Optional world facet for opening one simulated camera."""

    def camera(self, *, config: CameraConfig) -> CameraDriver: ...


@runtime_checkable
class SimulationAdministrationBackend(Protocol):
    """Optional trusted reset and state facet for a simulation world.

    This facet is deliberately separate from robot and camera ports. Its state
    may contain privileged ground truth and must be retained by an evaluator,
    never handed to a participant runtime.
    """

    def evaluation_snapshot(self) -> Mapping[str, Any]: ...

    def evaluation_reset(self, *, seed: int) -> bool: ...


class SimulationAdministrationError(RuntimeError):
    """A trusted simulation operation was unavailable or became uncertain."""


@dataclass(frozen=True)
class SimulationSnapshot:
    """Immutable evaluator-owned state from one complete simulated site."""

    episode_revision: int
    digest: str
    _worlds_json: str = field(repr=False)

    @property
    def worlds(self) -> Mapping[str, Any]:
        return MappingProxyType(json.loads(self._worlds_json))

    def as_dict(self) -> Mapping[str, Any]:
        return MappingProxyType(
            {
                "contract": "waddle.simulation-snapshot/v1",
                "episode_revision": self.episode_revision,
                "digest": self.digest,
                "worlds": json.loads(self._worlds_json),
            }
        )


@dataclass(frozen=True)
class _AdministrationBinding:
    worlds: Mapping[str, SimulationAdministrationBackend]
    lifecycle_lock: threading.RLock
    dispatch_lock: threading.RLock
    owner: object


class SimulationAdministration:
    """Explicit capability retained only by a trusted local evaluator.

    Construct this object before opening a fully simulated :class:`Site`, pass
    it as ``simulation_administration=`` to ``Site.open``, and keep it outside
    the participant process. Calls fail before binding, after site close, for a
    partially physical site, or after an uncertain reset.

    The caller must stop participant dispatch and drain admitted operations
    before ``reset``. The SDK serializes the reset against action dispatch and
    world stepping; it cannot decide when a higher-level operation is terminal.
    """

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._binding: _AdministrationBinding | None = None
        self._episode_revision = 0
        self._failed = False

    def _bind(
        self,
        worlds: Mapping[str, SimulationBackend],
        *,
        lifecycle_lock: threading.RLock,
        dispatch_lock: threading.RLock,
        owner: object,
        fully_simulated: bool,
    ) -> None:
        with self._lock:
            if self._binding is not None or self._failed:
                raise SimulationAdministrationError(
                    "simulation administration is already bound or failed"
                )
            if not fully_simulated or not worlds:
                raise SimulationAdministrationError(
                    "simulation administration requires a fully simulated site"
                )
            unsupported = [
                name
                for name, backend in worlds.items()
                if not isinstance(backend, SimulationAdministrationBackend)
            ]
            if unsupported:
                raise SimulationAdministrationError(
                    "simulation worlds do not provide trusted administration: "
                    + ", ".join(sorted(unsupported))
                )
            self._binding = _AdministrationBinding(
                MappingProxyType(dict(worlds)),
                lifecycle_lock,
                dispatch_lock,
                owner,
            )
            self._episode_revision = 0

    def _unbind(self, owner: object) -> None:
        with self._lock:
            if self._binding is not None and self._binding.owner is owner:
                self._binding = None

    def _require(self) -> _AdministrationBinding:
        with self._lock:
            if self._binding is None:
                raise SimulationAdministrationError(
                    "simulation administration is not bound to an open site"
                )
            if self._failed:
                raise SimulationAdministrationError(
                    "simulation administration failed; reopen the site"
                )
            return self._binding

    def snapshot(self) -> SimulationSnapshot:
        """Read privileged state without exposing it through the SDK runtime."""

        with self._lock:
            binding = self._require()
            try:
                with binding.dispatch_lock, binding.lifecycle_lock:
                    worlds = {
                        name: _evaluation_document(backend.evaluation_snapshot())
                        for name, backend in binding.worlds.items()
                    }
            except Exception as error:
                raise SimulationAdministrationError(
                    "simulation snapshot failed"
                ) from error
            return self._snapshot(worlds)

    def reset(self, *, seed: int) -> SimulationSnapshot:
        """Reset only the bound simulation and return its new initial state."""

        if isinstance(seed, bool) or not isinstance(seed, int):
            raise TypeError("simulation reset seed must be an integer")
        if seed < 0 or seed > 2**63 - 1:
            raise ValueError("simulation reset seed must be between 0 and 2^63-1")
        with self._lock:
            binding = self._require()
            try:
                with binding.dispatch_lock, binding.lifecycle_lock:
                    for backend in binding.worlds.values():
                        if not backend.evaluation_reset(seed=seed):
                            raise SimulationAdministrationError(
                                "a simulation world refused reset"
                            )
                    self._episode_revision += 1
                    worlds = {
                        name: _evaluation_document(backend.evaluation_snapshot())
                        for name, backend in binding.worlds.items()
                    }
            except Exception as error:
                self._failed = True
                if isinstance(error, SimulationAdministrationError):
                    raise
                raise SimulationAdministrationError(
                    "simulation reset was not confirmed; reopen the site"
                ) from error
            return self._snapshot(worlds)

    def _snapshot(self, worlds: Mapping[str, Any]) -> SimulationSnapshot:
        worlds_json = json.dumps(
            worlds, sort_keys=True, separators=(",", ":"), ensure_ascii=True
        )
        payload = json.dumps(
            {
                "contract": "waddle.simulation-snapshot/v1",
                "episode_revision": self._episode_revision,
                "worlds": json.loads(worlds_json),
            },
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
        ).encode("utf-8")
        return SimulationSnapshot(
            self._episode_revision,
            "sha256:" + hashlib.sha256(payload).hexdigest(),
            worlds_json,
        )


def _evaluation_document(value: Mapping[str, Any]) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise TypeError("simulation evaluation state must be a mapping")
    copied = copy.deepcopy(dict(value))

    def validate(item: Any) -> None:
        if item is None or isinstance(item, (str, bool, int)):
            return
        if isinstance(item, float):
            if not math.isfinite(item):
                raise ValueError("simulation evaluation state must be finite")
            return
        if isinstance(item, list):
            for child in item:
                validate(child)
            return
        if isinstance(item, dict):
            if any(not isinstance(key, str) for key in item):
                raise TypeError("simulation evaluation state keys must be strings")
            for child in item.values():
                validate(child)
            return
        raise TypeError("simulation evaluation state must contain only JSON values")

    validate(copied)
    if len(json.dumps(copied, separators=(",", ":")).encode("utf-8")) > 16 << 20:
        raise ValueError("simulation evaluation state exceeds 16 MiB")
    return copied


class SimulationFactoryError(ValueError):
    """A world driver reference or factory does not satisfy the SDK contract."""


_BUILTIN_BACKENDS = {
    "mujoco": "waddle_sdk.robots.mujoco:backend",
    "ros2": "waddle_sdk.robots.ros2:backend",
}


def simulation_backend_names() -> tuple[str, ...]:
    """List built-in and installed backend short names without importing them."""

    installed = metadata.entry_points().select(group="waddle_sdk.simulation_backends")
    return tuple(sorted({*_BUILTIN_BACKENDS, *(entry.name for entry in installed)}))


def _installed_backend(name: str) -> Callable[..., object] | None:
    selected = tuple(
        metadata.entry_points().select(
            group="waddle_sdk.simulation_backends", name=name
        )
    )
    if len(selected) > 1:
        raise SimulationFactoryError(
            f"multiple installed simulation backends are named {name!r}"
        )
    if not selected:
        return None
    try:
        target = selected[0].load()
    except (ImportError, AttributeError) as exc:
        raise SimulationFactoryError(
            f"cannot load installed world driver ({type(exc).__name__})"
        ) from exc
    if not callable(target):
        raise SimulationFactoryError(f"installed world driver {name!r} is not callable")
    return target


def resolve_simulation_factory(spec: str) -> Callable[..., object]:
    """Resolve one lazy ``module:attribute`` simulation factory."""

    target_spec = _BUILTIN_BACKENDS.get(spec, spec)
    if target_spec == spec and ":" not in spec and "." not in spec:
        installed = _installed_backend(spec)
        if installed is not None:
            return installed
    if ":" in target_spec:
        module_name, attribute = target_spec.split(":", 1)
    else:
        module_name = target_spec
        attribute = "backend"
    if not module_name or not attribute:
        raise SimulationFactoryError(
            f"world driver {target_spec!r} must name module:attribute"
        )
    try:
        module = importlib.import_module(module_name)
        target = getattr(module, attribute)
    except (ImportError, AttributeError) as exc:
        raise SimulationFactoryError(
            f"cannot load world driver ({type(exc).__name__})"
        ) from exc
    if not callable(target):
        raise SimulationFactoryError(f"world driver {target_spec!r} is not callable")
    return target


def build_simulation_backend(
    target: Callable[..., object], config: WorldConfig
) -> SimulationBackend:
    """Build one unopened backend and validate its mandatory lifecycle."""

    parameters = inspect.signature(target).parameters
    if "config" not in parameters:
        raise SimulationFactoryError(
            f"world {config.name!r} driver must accept config=WorldConfig"
        )
    result = target(config=config)
    required = ("open", "step", "reset", "close")
    if not isinstance(result, SimulationBackend) or any(
        not callable(getattr(result, name, None)) for name in required
    ):
        raise SimulationFactoryError(
            f"world {config.name!r} driver must provide open(), step(), reset(), "
            "and close()"
        )
    return result


__all__ = [
    "SimulationAdministration",
    "SimulationAdministrationBackend",
    "SimulationAdministrationError",
    "SimulationBackend",
    "SimulationCameraBackend",
    "SimulationFactoryError",
    "SimulationPartBackend",
    "SimulationSnapshot",
    "WorldConfig",
    "build_simulation_backend",
    "resolve_simulation_factory",
    "simulation_backend_names",
]
