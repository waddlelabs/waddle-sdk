"""Non-opening hardware discovery for site-configuration frontends.

Discovery reports evidence; it never constructs a driver, opens a robot bus,
starts a camera pipeline, or claims that a generic transport identifies a
particular robot. Configuration frontends use these immutable candidates as
suggestions and keep the site operator in the loop for adapter selection and
owner-envelope values.

Custom SDK integrations can publish callables in the
``waddle_sdk.hardware_discovery`` entry-point group. A provider receives no
arguments and returns an iterable of :class:`HardwareCandidate` objects.
Provider failures become report warnings so one broken optional vendor package
does not hide the rest of the machine.
"""

from __future__ import annotations

import os
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from importlib.metadata import entry_points
from pathlib import Path
from types import MappingProxyType
from typing import Any, Literal, Protocol, TypeAlias

HardwareKind: TypeAlias = Literal["camera", "robot", "transport"]
Confidence: TypeAlias = Literal["confirmed", "possible"]


def _frozen_mapping(value: Mapping[str, Any]) -> Mapping[str, Any]:
    return MappingProxyType({str(key): item for key, item in value.items()})


@dataclass(frozen=True)
class HardwareCandidate:
    """One non-opening piece of hardware evidence.

    ``driver`` is populated only when the evidence identifies an SDK adapter
    safely. A generic CAN or serial transport deliberately has no driver; a
    user or custom provider must select the robot that is actually attached.
    """

    identifier: str
    kind: HardwareKind
    label: str
    driver: str | None = None
    connection: Mapping[str, Any] = field(default_factory=dict)
    metadata: Mapping[str, Any] = field(default_factory=dict)
    confidence: Confidence = "confirmed"

    def __post_init__(self) -> None:
        if not self.identifier or not self.label:
            raise ValueError("hardware candidate identifier and label must be non-empty")
        if self.kind not in {"camera", "robot", "transport"}:
            raise ValueError(f"unsupported hardware candidate kind {self.kind!r}")
        if self.confidence not in {"confirmed", "possible"}:
            raise ValueError(f"unsupported discovery confidence {self.confidence!r}")
        if self.driver is not None and not self.driver:
            raise ValueError("hardware candidate driver must be non-empty when present")
        object.__setattr__(self, "connection", _frozen_mapping(self.connection))
        object.__setattr__(self, "metadata", _frozen_mapping(self.metadata))


class HardwareDiscoveryProvider(Protocol):
    def __call__(self) -> Iterable[HardwareCandidate]: ...


@dataclass(frozen=True)
class DiscoveryReport:
    candidates: tuple[HardwareCandidate, ...]
    warnings: tuple[str, ...] = ()


def _read(path: Path) -> str | None:
    try:
        value = path.read_text(encoding="utf-8", errors="replace").strip()
    except OSError:
        return None
    return value or None


def _device_fact(device: Path, name: str) -> str | None:
    """Read one USB fact from a video node or one of its sysfs parents."""

    current = device.resolve(strict=False)
    for candidate in (current, *current.parents):
        value = _read(candidate / name)
        if value is not None:
            return value
        if candidate.name in {"devices", "sys", ""}:
            break
    return None


def _linux_candidates(*, sys_root: Path, dev_root: Path) -> tuple[HardwareCandidate, ...]:
    candidates: list[HardwareCandidate] = []

    network = sys_root / "class" / "net"
    try:
        interfaces = sorted(network.iterdir(), key=lambda path: path.name)
    except OSError:
        interfaces = []
    for interface in interfaces:
        if _read(interface / "type") != "280":  # Linux ARPHRD_CAN
            continue
        candidates.append(
            HardwareCandidate(
                identifier=f"linux-can:{interface.name}",
                kind="transport",
                label=f"CAN interface {interface.name}",
                connection={"channel": interface.name},
                metadata={
                    "transport": "socketcan",
                    "state": _read(interface / "operstate") or "unknown",
                },
            )
        )

    video_root = sys_root / "class" / "video4linux"
    try:
        video_nodes = sorted(video_root.iterdir(), key=lambda path: path.name)
    except OSError:
        video_nodes = []
    seen_cameras: set[str] = set()
    for node in video_nodes:
        name = _read(node / "name") or node.name
        serial = _device_fact(node / "device", "serial")
        vendor = (_device_fact(node / "device", "idVendor") or "").lower()
        product = (_device_fact(node / "device", "idProduct") or "").lower()
        lowered = name.lower()
        if "realsense" in lowered or vendor == "8086":
            family = "realsense"
            driver = "waddle_sdk.cameras.realsense"
            connection: Mapping[str, Any] = {"serial": serial} if serial else {}
            label = f"Intel RealSense {serial or name}"
        elif any(marker in lowered for marker in ("orbbec", "astra", "gemini")):
            family = "orbbec"
            driver = "waddle_sdk.cameras.orbbec"
            # The current Orbbec adapter has no serial-selection parameter;
            # retain a discovered serial as evidence only.
            connection = {}
            label = f"Orbbec {serial or name}"
        else:
            family = "uvc"
            driver = "waddle_sdk.cameras.usb"
            connection = {"device": str(dev_root / node.name)}
            label = f"USB camera {name} ({node.name})"
        physical = serial or str((node / "device").resolve(strict=False))
        identity = f"linux-camera:{family}:{physical}"
        if identity in seen_cameras:
            continue
        seen_cameras.add(identity)
        candidates.append(
            HardwareCandidate(
                identifier=identity,
                kind="camera",
                label=label,
                driver=driver,
                connection=connection,
                metadata={
                    "device": str(dev_root / node.name),
                    "family": family,
                    "name": name,
                    "product_id": product,
                    "serial": serial or "",
                    "vendor_id": vendor,
                },
                confidence="confirmed" if family != "uvc" else "possible",
            )
        )

    for pattern in ("ttyACM*", "ttyUSB*"):
        try:
            serial_nodes = sorted(dev_root.glob(pattern), key=lambda path: path.name)
        except OSError:
            serial_nodes = []
        for node in serial_nodes:
            candidates.append(
                HardwareCandidate(
                    identifier=f"linux-serial:{node.name}",
                    kind="transport",
                    label=f"Serial device {node}",
                    connection={"device": str(node)},
                    metadata={"transport": "serial"},
                )
            )

    return tuple(candidates)


def _plugin_providers() -> tuple[tuple[str, HardwareDiscoveryProvider], ...]:
    selected = entry_points().select(group="waddle_sdk.hardware_discovery")
    providers: list[tuple[str, HardwareDiscoveryProvider]] = []
    for entry in selected:
        loaded = entry.load()
        if not callable(loaded):
            raise TypeError(f"hardware discovery entry point {entry.name!r} is not callable")
        providers.append((entry.name, loaded))
    return tuple(providers)


def discover_hardware(
    *,
    providers: Sequence[HardwareDiscoveryProvider] = (),
    include_plugins: bool = True,
    sys_root: str | Path = "/sys",
    dev_root: str | Path = "/dev",
) -> DiscoveryReport:
    """Scan without opening hardware and return all usable evidence."""

    warnings: list[str] = []
    found: list[HardwareCandidate] = []
    if os.name == "posix":
        found.extend(
            _linux_candidates(sys_root=Path(sys_root), dev_root=Path(dev_root))
        )

    named_providers: list[tuple[str, HardwareDiscoveryProvider]] = [
        (getattr(provider, "__name__", type(provider).__name__), provider)
        for provider in providers
    ]
    if include_plugins:
        try:
            named_providers.extend(_plugin_providers())
        except (ImportError, TypeError) as error:
            warnings.append(f"hardware discovery plugins: {type(error).__name__}: {error}")

    for name, provider in named_providers:
        try:
            rows = tuple(provider())
            if not all(isinstance(row, HardwareCandidate) for row in rows):
                raise TypeError("provider returned a non-HardwareCandidate value")
            found.extend(rows)
        except Exception as error:  # noqa: BLE001 -- optional integrations are isolated
            warnings.append(
                f"hardware discovery provider {name!r}: {type(error).__name__}: {error}"
            )

    unique: dict[str, HardwareCandidate] = {}
    for candidate in found:
        if candidate.identifier in unique:
            warnings.append(f"duplicate hardware candidate {candidate.identifier!r} ignored")
            continue
        unique[candidate.identifier] = candidate
    ordered = tuple(
        sorted(unique.values(), key=lambda row: (row.kind, row.label, row.identifier))
    )
    return DiscoveryReport(ordered, tuple(warnings))


__all__ = [
    "Confidence",
    "DiscoveryReport",
    "HardwareCandidate",
    "HardwareDiscoveryProvider",
    "HardwareKind",
    "discover_hardware",
]
