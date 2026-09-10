"""Public contracts for manifest-selected, shared simulation worlds.

A simulation backend is an ordinary installable Python package named by a
``site.yaml`` world.  Its factory and ``part`` declarations are non-opening;
the SDK opens the world only after connector authorization and closes it after
all world-backed arms and cameras. Applications consuming the ordinary
:class:`~waddle_sdk.runtime.SdkRuntimePort` need no simulator-specific imports.
"""

from __future__ import annotations

import importlib
import inspect
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from importlib import metadata
from pathlib import Path
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
    "SimulationBackend",
    "SimulationCameraBackend",
    "SimulationFactoryError",
    "SimulationPartBackend",
    "WorldConfig",
    "build_simulation_backend",
    "resolve_simulation_factory",
    "simulation_backend_names",
]
