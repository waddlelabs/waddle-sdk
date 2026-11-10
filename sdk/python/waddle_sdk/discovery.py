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
import re
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from importlib import import_module
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
            raise ValueError(
                "hardware candidate identifier and label must be non-empty"
            )
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
    """Read one USB fact from a device node or one of its sysfs parents."""

    current = device.resolve(strict=False)
    for candidate in (current, *current.parents):
        value = _read(candidate / name)
        if value is not None:
            return value
        if candidate.name in {"devices", "sys", ""}:
            break
    return None


def _usb_path(device: Path) -> str:
    """Stable USB device ancestor shared by sysfs and vendor physical ports."""
    # The RSUSB backend reports bus-port-device (e.g. 2-3.4-4); the
    # V4L backend reports a sysfs path. Device numbers change after reconnect.
    compact = re.fullmatch(r"(\d+-\d+(?:\.\d+)*)-\d+", str(device))
    if compact:
        return "usb:" + compact[1]
    current = device.resolve(strict=False)
    for candidate in (current, *current.parents):
        if re.fullmatch(r"\d+-\d+(?:\.\d+)*", candidate.name):
            return "usb:" + candidate.name
    return ""


def _realsense_vendor_candidates(
    module: Any | None = None,
) -> tuple[HardwareCandidate, ...]:
    """Enumerate driver-valid RealSense identities without starting a stream."""

    if module is None:
        module = import_module("pyrealsense2")
    from .cameras.realsense import _shared_context

    rows: list[HardwareCandidate] = []
    for device in _shared_context(module).query_devices():
        serial = str(device.get_info(module.camera_info.serial_number)).strip()
        if not serial:
            continue
        try:
            name = str(device.get_info(module.camera_info.name)).strip()
        except (AttributeError, RuntimeError):
            name = "Intel RealSense"
        try:
            product = str(device.get_info(module.camera_info.product_id)).strip()
        except (AttributeError, RuntimeError):
            product = ""
        try:
            physical_port = _usb_path(
                Path(device.get_info(module.camera_info.physical_port))
            )
        except (AttributeError, RuntimeError):
            physical_port = ""
        rows.append(
            HardwareCandidate(
                identifier=f"linux-camera:realsense:{serial}",
                kind="camera",
                label=f"{name or 'Intel RealSense'} {serial}",
                driver="waddle_sdk.cameras.realsense",
                connection={"serial": serial},
                metadata={
                    "device": "",
                    "usb_path": physical_port,
                    "family": "realsense",
                    "name": name or "Intel RealSense",
                    "product_id": product,
                    "serial": serial,
                    "vendor_id": "8086",
                },
            )
        )
    return tuple(rows)


def _linux_candidates(
    *,
    sys_root: Path,
    dev_root: Path,
    realsense_resolved: tuple[HardwareCandidate, ...] = (),
) -> tuple[HardwareCandidate, ...]:
    candidates: list[HardwareCandidate] = []

    network = sys_root / "class" / "net"
    try:
        interfaces = sorted(network.iterdir(), key=lambda path: path.name)
    except OSError:
        interfaces = []
    bound_usb: set[str] = set()
    for interface in interfaces:
        if _read(interface / "type") != "280":  # Linux ARPHRD_CAN
            continue
        device = interface / "device"
        usb_path = _usb_path(device)
        attachment = (
            "usb"
            if usb_path
            else "built-in"
            if (device / "subsystem").resolve().name == "platform"
            else "unknown"
        )
        serial = _device_fact(device, "serial") or ""
        label = f"CAN interface {interface.name}"
        if usb_path:
            bound_usb.add(usb_path)
            label = f"USB CAN {serial or usb_path} · {interface.name}"
        elif attachment == "built-in":
            label = f"Built-in CAN port {interface.name}"
        candidates.append(
            HardwareCandidate(
                identifier=(
                    f"linux-can-usb:{serial}" if serial else f"linux-can-{usb_path}"
                )
                if usb_path
                else f"linux-can:{interface.name}",
                kind="transport",
                label=label,
                connection={"channel": interface.name},
                metadata={
                    "transport": "socketcan",
                    "attachment": attachment,
                    "usb_path": usb_path,
                    "serial": serial,
                    "state": _read(interface / "operstate") or "unknown",
                },
            )
        )

    # Known gs_usb adapters still need to be visible when the kernel lacks their
    # driver. USB identity alone must never supply a guessed CAN channel.
    usb_root = sys_root / "bus" / "usb" / "devices"
    try:
        usb_devices = sorted(usb_root.iterdir())
    except OSError:
        usb_devices = []
    for device in usb_devices:
        vendor = (_read(device / "idVendor") or "").lower()
        product = (_read(device / "idProduct") or "").lower()
        if (vendor, product) not in {("1d50", "606f"), ("1209", "2323")}:
            continue
        usb_path = _usb_path(device)
        if not usb_path or usb_path in bound_usb:
            continue
        serial = _read(device / "serial") or ""
        candidates.append(
            HardwareCandidate(
                identifier=f"linux-can-usb:{serial}"
                if serial
                else f"linux-can-{usb_path}",
                kind="transport",
                label=f"USB CAN {serial or usb_path}",
                metadata={
                    "transport": "socketcan",
                    "attachment": "usb",
                    "usb_path": usb_path,
                    "serial": serial,
                    "setup_error": "No CAN interface; install/enable the Linux gs_usb driver",
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
            usb_path = _usb_path(node / "device")
            if any(
                (usb_path and row.metadata.get("usb_path") == usb_path)
                or (serial and row.connection.get("serial") == serial)
                for row in realsense_resolved
            ):
                continue
            family = "realsense"
            # A USB interface's sysfs serial is not necessarily the serial that
            # librealsense accepts in config.enable_device(). Keep it as evidence,
            # but never turn it into an executable driver selector.
            driver = None
            connection: Mapping[str, Any] = {}
            label = (
                f"Intel RealSense USB device {serial or name} (SDK serial unresolved)"
            )
            metadata_serial = ""
            confidence: Confidence = "possible"
        elif any(marker in lowered for marker in ("orbbec", "astra", "gemini")):
            family = "orbbec"
            driver = "waddle_sdk.cameras.orbbec"
            connection = {"serial": serial} if serial else {}
            label = f"Orbbec {serial or name}"
            metadata_serial = serial or ""
            confidence = "confirmed"
        else:
            family = "uvc"
            driver = "waddle_sdk.cameras.usb"
            connection = {"device": str(dev_root / node.name)}
            label = f"USB camera {name} ({node.name})"
            metadata_serial = serial or ""
            confidence = "possible"
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
                    "usb_path": _usb_path(node / "device"),
                    "family": family,
                    "name": name,
                    "product_id": product,
                    "serial": metadata_serial,
                    "usb_serial_evidence": serial or "",
                    "vendor_id": vendor,
                },
                confidence=confidence,
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
            raise TypeError(
                f"hardware discovery entry point {entry.name!r} is not callable"
            )
        providers.append((entry.name, loaded))
    return tuple(providers)


def discover_hardware(
    *,
    providers: Sequence[HardwareDiscoveryProvider] = (),
    include_plugins: bool = True,
    include_vendor_enumeration: bool = True,
    sys_root: str | Path = "/sys",
    dev_root: str | Path = "/dev",
) -> DiscoveryReport:
    """Scan without opening hardware and return all usable evidence."""

    warnings: list[str] = []
    found: list[HardwareCandidate] = []
    if os.name == "posix":
        realsense: tuple[HardwareCandidate, ...] = ()
        realsense_error = None
        if include_vendor_enumeration:
            try:
                realsense = _realsense_vendor_candidates()
            except Exception as error:  # noqa: BLE001 -- optional vendor discovery
                realsense_error = (
                    f"RealSense discovery failed ({type(error).__name__}): {error}"
                )
                if (
                    isinstance(error, ModuleNotFoundError)
                    and error.name == "pyrealsense2"
                ):
                    realsense_error += (
                        ". Add or Preview installs camera support when an installer is connected; "
                        "otherwise install 'waddle-sdk[realsense]' in this Python environment."
                    )
        found.extend(
            _linux_candidates(
                sys_root=Path(sys_root),
                dev_root=Path(dev_root),
                realsense_resolved=realsense,
            )
        )
        found.extend(realsense)
        unresolved = any(
            row.metadata.get("family") == "realsense" and row.driver is None
            for row in found
        )
        if include_vendor_enumeration and unresolved:
            warnings.append(
                realsense_error
                or "RealSense is visible on USB but its SDK returned no matching device. "
                "Check USB permissions, the camera cable and the installed camera runtime."
            )

    named_providers: list[tuple[str, HardwareDiscoveryProvider]] = [
        (getattr(provider, "__name__", type(provider).__name__), provider)
        for provider in providers
    ]
    if include_plugins:
        try:
            named_providers.extend(_plugin_providers())
        except (ImportError, TypeError) as error:
            warnings.append(
                f"hardware discovery plugin loading failed ({type(error).__name__})"
            )

    for name, provider in named_providers:
        try:
            rows = tuple(provider())
            if not all(isinstance(row, HardwareCandidate) for row in rows):
                raise TypeError("provider returned a non-HardwareCandidate value")
            found.extend(rows)
        except Exception as error:  # noqa: BLE001 -- optional integrations are isolated
            warnings.append(
                f"hardware discovery provider {name!r} failed ({type(error).__name__})"
            )

    unique: dict[str, HardwareCandidate] = {}
    for candidate in found:
        if candidate.identifier in unique:
            warnings.append(
                f"duplicate hardware candidate {candidate.identifier!r} ignored"
            )
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
