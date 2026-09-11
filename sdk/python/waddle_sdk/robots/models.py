"""Optional, non-opening source-model contract shared by robot adapters.

A selected adapter module may expose ``model_sources`` matching
:class:`ModelSourceProvider`. Sources describe hardware; applications choose
compilers, approximations and execution policy. No provider opens a driver.
"""

from __future__ import annotations

import importlib
import json
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import PurePosixPath
from types import MappingProxyType
from typing import Any, Protocol


class ModelSourceError(ValueError):
    """Unavailable or invalid selected sources, with a stable scoped reason."""

    def __init__(self, detail: str, *, code: str = "model_sources_invalid"):
        super().__init__(detail)
        self.code = code


def _freeze(value):
    if isinstance(value, Mapping):
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_freeze(item) for item in value)
    return value


def _plain(value):
    if isinstance(value, Mapping):
        return {key: _plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    return value


def _files(files):
    result = {}
    for name, data in files.items():
        path = PurePosixPath(name)
        if (
            not name
            or path.is_absolute()
            or ".." in path.parts
            or str(path) != name
            or "\\" in name
            or "\x00" in name
            or not isinstance(data, bytes)
            or len(data) > 32 * 1024 * 1024
        ):
            raise ModelSourceError(
                "Assets require bounded bytes and confined relative paths"
            )
        result[name] = data
    return MappingProxyType(result)


@dataclass(frozen=True)
class ModelSources:
    """Complete source model, immutable assets and explicit named bindings.

    ``format`` declares the source language (currently the reference provider
    emits ``mjcf``). Consumers reject unsupported formats explicitly. ``assets``
    are paths relative to the source document; ``licenses`` are retained beside
    published assets. ``provenance`` contains finite JSON source evidence, never
    credentials or runtime authority. Primary joint names/order/units describe the
    controllable chain. Other model joints retain their physical relationships
    in the source document and require a supported consumer treatment.

    ``tcp_frame`` explicitly binds an application-visible frame identifier to
    ``tcp_site`` in the source model. No topology, frame, or units are inferred.
    The caller binds ``base_body`` to the configured part's base frame.
    """

    format: str
    model: bytes
    assets: Mapping[str, bytes]
    joint_names: tuple[str, ...]
    joint_units: tuple[str, ...]
    base_body: str
    tcp_site: str
    tcp_frame: str
    licenses: Mapping[str, bytes] = field(default_factory=dict)
    provenance: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        names, units = tuple(self.joint_names), tuple(self.joint_units)
        if (
            not isinstance(self.model, bytes)
            or not self.model
            or len(self.model) > 32 * 1024 * 1024
            or not 1 <= len(names) <= 128
            or len(set(names)) != len(names)
            or len(units) != len(names)
            or any(unit not in ("rad", "m") for unit in units)
            or any(
                not isinstance(name, str) or not name or len(name) > 256
                for name in (
                    self.format,
                    self.base_body,
                    self.tcp_site,
                    self.tcp_frame,
                    *names,
                )
            )
        ):
            raise ModelSourceError(
                "Source model requires bounded bytes and explicit unique joint/frame bindings"
            )
        assets, licenses = _files(self.assets), _files(self.licenses)
        if set(assets) & set(licenses) or any(
            name in {"model.xml", "SOURCE.json"} for name in (*assets, *licenses)
        ):
            raise ModelSourceError(
                "Source assets and licenses must have distinct non-reserved paths"
            )
        if (
            len(self.model)
            + sum(map(len, assets.values()))
            + sum(map(len, licenses.values()))
            > 128 * 1024 * 1024
        ):
            raise ModelSourceError("Source bundle exceeds 128 MiB")
        try:
            evidence = json.loads(json.dumps(_plain(self.provenance), allow_nan=False))
        except (TypeError, ValueError) as error:
            raise ModelSourceError(
                "Source provenance must contain finite JSON"
            ) from error
        if not isinstance(evidence, dict):
            raise ModelSourceError("Source provenance must be a mapping")
        object.__setattr__(self, "assets", assets)
        object.__setattr__(self, "licenses", licenses)
        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "joint_units", units)
        object.__setattr__(self, "provenance", _freeze(evidence))


class ModelSourceProvider(Protocol):
    """Optional module-level adapter source loader; never a hardware factory.

    ``part`` is the reviewed manifest part row, including configuration needed
    to select exact geometry. Return ``None`` when sources are unsupported.
    Once a supported source is selected, failures raise ``ModelSourceError``;
    they must never return placeholder geometry or a different model.
    """

    def __call__(
        self, *, factory: str, part_name: str, part: Mapping[str, Any]
    ) -> ModelSources | None: ...


def model_sources_for_driver(
    driver: str, *, part_name: str, part: Mapping[str, Any]
) -> ModelSources | None:
    """Resolve the same optional source contract for any explicit robot driver.

    Missing extensions are ordinary absence. Import/provider failures remain
    typed failures with secret-safe messages. Explicit external model selections
    can bypass this function entirely. No registry edits or device opening occur.
    """
    module_name, separator, factory = driver.partition(":")
    if not separator or not module_name or not factory:
        raise ModelSourceError(
            "Robot sources require a module:factory driver target",
            code="source_provider_invalid",
        )
    try:
        module = importlib.import_module(module_name)
        provider = getattr(module, "model_sources", None)
        if provider is None:
            return None
        if not callable(provider):
            raise ModelSourceError(
                "Source provider is not callable", code="source_provider_invalid"
            )
        result = provider(factory=factory, part_name=part_name, part=_freeze(part))
        if result is not None and not isinstance(result, ModelSources):
            raise ModelSourceError(
                "Source provider returned an invalid bundle",
                code="source_provider_invalid",
            )
        return result
    except ModelSourceError:
        raise
    except Exception as error:
        raise ModelSourceError(
            "Selected source provider failed", code="source_provider_failed"
        ) from error


__all__ = [
    "ModelSourceError",
    "ModelSourceProvider",
    "ModelSources",
    "model_sources_for_driver",
]
