"""Non-opening owner-envelope presets published by robot adapters.

Presets are configuration suggestions, never authority and never a substitute for
site review.  A robot module may expose a module-level ``safety_presets`` callable
with the signature documented by :class:`SafetyPresetProvider`.  Initializers can
then offer hardware-aware starting values without importing Metal into the SDK or
opening a device.
"""

from __future__ import annotations

import importlib
import math
import re
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, Protocol

_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")


def _freeze(value: Any) -> Any:
    if isinstance(value, Mapping):
        return MappingProxyType(
            {str(key): _freeze(item) for key, item in value.items()}
        )
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        return tuple(_freeze(item) for item in value)
    return value


def _mapping(value: Mapping[str, Any]) -> Mapping[str, Any]:
    return _freeze(value)


def _workspace_bounds(value: Mapping[str, Any]) -> Mapping[str, Any]:
    row = {str(key): item for key, item in value.items()}
    if set(row) != {"min", "max"}:
        raise ValueError("safety preset workspace_bounds require exactly min and max")
    corners: list[tuple[float, float, float]] = []
    for name in ("min", "max"):
        raw = row[name]
        if not isinstance(raw, Sequence) or isinstance(raw, (str, bytes)):
            raise TypeError(f"safety preset workspace_bounds.{name} must be xyz")
        corner = tuple(float(item) for item in raw)
        if len(corner) != 3 or not all(math.isfinite(item) for item in corner):
            raise ValueError(
                f"safety preset workspace_bounds.{name} needs three finite numbers"
            )
        corners.append(corner)  # type: ignore[arg-type]
    if any(lower > upper for lower, upper in zip(*corners, strict=True)):
        raise ValueError("safety preset workspace_bounds min must not exceed max")
    return MappingProxyType({"min": corners[0], "max": corners[1]})


@dataclass(frozen=True)
class SafetyPreset:
    """One reviewed starting point for a site's owner envelope.

    The values are copied into ``site.yaml``.  Runtime code never receives the
    preset identifier and cannot distinguish a preset from hand-entered values.
    """

    identifier: str
    label: str
    workspace_bounds: Mapping[str, Any]
    static_keepouts: Sequence[Mapping[str, Any]] = ()
    self_collision: Mapping[str, Any] = field(default_factory=dict)
    review: str = "Verify these bounds against the actual mounting and surroundings."

    def __post_init__(self) -> None:
        if not _IDENTIFIER.fullmatch(self.identifier):
            raise ValueError("safety preset identifier is invalid")
        if not self.label.strip() or not self.review.strip():
            raise ValueError("safety preset label and review note must be non-empty")
        object.__setattr__(
            self, "workspace_bounds", _workspace_bounds(self.workspace_bounds)
        )
        object.__setattr__(
            self,
            "static_keepouts",
            tuple(_mapping(row) for row in self.static_keepouts),
        )
        object.__setattr__(self, "self_collision", _mapping(self.self_collision))


class SafetyPresetProvider(Protocol):
    """The optional module-level ``safety_presets`` adapter extension.

    It receives the selected factory attribute and manifest options, performs no
    I/O, and returns zero or more immutable presets.
    """

    def __call__(
        self, *, factory: str, options: Mapping[str, Any]
    ) -> Iterable[SafetyPreset]: ...


@dataclass(frozen=True)
class SafetyPresetReport:
    presets: tuple[SafetyPreset, ...]
    warnings: tuple[str, ...] = ()


def safety_presets_for_driver(
    driver: str,
    *,
    options: Mapping[str, Any] | None = None,
) -> SafetyPresetReport:
    """Load a driver's configuration-only presets without opening hardware.

    An absent extension is ordinary capability degradation.  Import/provider
    failures are returned as secret-safe warnings so an optional adapter cannot
    make the initializer unusable.
    """

    if ":" in driver:
        module_name, factory = driver.split(":", 1)
    else:
        module_name = driver
        leaf = module_name.rsplit(".", 1)[-1]
        factory = "".join(piece.capitalize() for piece in leaf.split("_")) + "Driver"
    if not module_name or not factory:
        return SafetyPresetReport((), (f"invalid robot driver target {driver!r}",))
    try:
        module = importlib.import_module(module_name)
    except Exception as error:  # noqa: BLE001 -- optional extension is isolated
        return SafetyPresetReport(
            (),
            (f"cannot inspect robot safety presets ({type(error).__name__})",),
        )
    provider = getattr(module, "safety_presets", None)
    if provider is None:
        return SafetyPresetReport(())
    if not callable(provider):
        return SafetyPresetReport(
            (), ("robot safety_presets extension is not callable",)
        )
    try:
        rows = tuple(
            provider(
                factory=factory,
                options=MappingProxyType(dict(options or {})),
            )
        )
        if not all(isinstance(row, SafetyPreset) for row in rows):
            raise TypeError("provider returned a non-SafetyPreset value")
        identifiers = [row.identifier for row in rows]
        if len(set(identifiers)) != len(identifiers):
            raise ValueError("provider returned duplicate safety preset identifiers")
    except Exception as error:  # noqa: BLE001 -- optional extension is isolated
        return SafetyPresetReport(
            (),
            (f"robot safety preset provider failed ({type(error).__name__})",),
        )
    return SafetyPresetReport(rows)


__all__ = [
    "SafetyPreset",
    "SafetyPresetProvider",
    "SafetyPresetReport",
    "safety_presets_for_driver",
]
