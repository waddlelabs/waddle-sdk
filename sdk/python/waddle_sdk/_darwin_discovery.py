"""Read-only IORegistry evidence; no USB handle or driver is opened."""

from __future__ import annotations

import plistlib
import subprocess
from pathlib import Path

from .discovery import HardwareCandidate
from .robots.usbcan_config import SUPPORTED_DEVICE_IDS

# USB identities accepted by the Candlelight/gs_usb reference transport. A
# transport does not identify the robot attached to it.
_GS_USB_IDS = SUPPORTED_DEVICE_IDS


def _usb_registry() -> list[dict]:
    result = subprocess.run(
        ["/usr/sbin/ioreg", "-a", "-r", "-c", "IOUSBHostDevice"],
        stdin=subprocess.DEVNULL,
        capture_output=True,
        timeout=5,
        check=True,
    )
    if len(result.stdout) > 8 * 1024 * 1024:
        raise ValueError("USB registry exceeds discovery's input bound")
    rows = plistlib.loads(result.stdout)
    if not isinstance(rows, list):
        raise TypeError("USB registry must contain a list of devices")
    return rows


def candidates(
    *, dev_root: Path, realsense_resolved: bool
) -> tuple[HardwareCandidate, ...]:
    found = []
    pending = list(_usb_registry())
    seen = set()
    while pending:
        row = pending.pop()
        if not isinstance(row, dict):
            raise TypeError("USB registry device must be a mapping")
        pending.extend(row.get("IORegistryEntryChildren", []))
        vendor, product = row.get("idVendor"), row.get("idProduct")
        if type(vendor) is not int or type(product) is not int:
            continue
        name = str(
            row.get("USB Product Name", row.get("IORegistryEntryName", "USB device"))
        )
        identity = str(row.get("IORegistryEntryID", row.get("locationID", "")))
        if not identity or identity in seen:
            continue
        seen.add(identity)
        serial = str(row.get("USB Serial Number", ""))
        metadata = {
            "vendor_id": f"{vendor:04x}",
            "product_id": f"{product:04x}",
            "usb_serial_evidence": serial,
            "name": name,
            "registry_id": identity,
        }
        if (vendor, product) in _GS_USB_IDS:
            found.append(
                HardwareCandidate(
                    identifier=f"darwin-usb-can:{identity}",
                    kind="transport",
                    label=f"USB CAN adapter {name}",
                    metadata={**metadata, "transport": "gs_usb"},
                )
            )
        elif (
            vendor == 0x8086 and "realsense" in name.lower() and not realsense_resolved
        ):
            found.append(
                HardwareCandidate(
                    identifier=f"darwin-camera:realsense:{identity}",
                    kind="camera",
                    label=f"{name} (SDK serial unresolved)",
                    metadata={**metadata, "family": "realsense", "serial": ""},
                    confidence="possible",
                )
            )
    for path in sorted(dev_root.glob("cu.*")):
        found.append(
            HardwareCandidate(
                identifier=f"darwin-serial:{path.name}",
                kind="transport",
                label=f"Serial device {path}",
                connection={"device": str(path)},
                metadata={"transport": "serial"},
            )
        )
    return tuple(found)
