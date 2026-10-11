"""Device-free configuration for the optional classic USB-CAN provider.

These immutable site facts can be constructed without python-can, PyUSB, I2RT
or an attached device. Opening belongs to the selected driver's lifecycle.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

SUPPORTED_DEVICE_IDS = frozenset({(0x1D50, 0x606F), (0x1209, 0x2323)})


@dataclass(frozen=True)
class UsbCanSelector:
    """Bind serial identity AND current location; reordered devices cannot substitute."""

    vendor_id: int
    product_id: int
    bus: int
    address: int
    serial_number: str

    def __post_init__(self):
        if (
            type(self.vendor_id) is not int
            or type(self.product_id) is not int
            or (self.vendor_id, self.product_id) not in SUPPORTED_DEVICE_IDS
        ):
            raise ValueError("Select a supported Candlelight/gs_usb integer VID/PID")
        if any(
            type(value) is not int or value < 0 for value in (self.bus, self.address)
        ):
            raise ValueError("USB bus/address must be explicit non-negative integers")
        if (
            not isinstance(self.serial_number, str)
            or not self.serial_number
            or self.serial_number.strip() != self.serial_number
            or "\0" in self.serial_number
        ):
            raise ValueError("USB-CAN requires an exact non-empty serial number")

    @property
    def channel(self) -> str:
        """An explicit vendor log identity; never a SocketCAN interface lookup."""
        return f"gs_usb:{self.serial_number}@{self.bus}:{self.address}"


@dataclass(frozen=True)
class UsbCanConfig:
    """Explicit YAM transport choice, without changing its controller or limits.

    ``libusb_library`` optionally binds an absolute native library path. The
    declaration performs no library lookup or device access. A reconnect that
    changes location requires a site operator to update the exact selector.
    """

    selector: UsbCanSelector
    libusb_library: str | None = None

    def __post_init__(self):
        if not isinstance(self.selector, UsbCanSelector):
            raise TypeError("USB-CAN selector must be UsbCanSelector")
        library = self.libusb_library
        if library is not None and (
            not isinstance(library, str)
            or "\0" in library
            or not Path(library).is_absolute()
        ):
            raise ValueError("libusb_library must be an absolute library path or None")

    @classmethod
    def from_value(cls, value: UsbCanConfig | Mapping) -> UsbCanConfig:
        """Freeze the ordinary Site manifest mapping; reject ambiguous fields."""
        if isinstance(value, cls):
            return value
        if not isinstance(value, Mapping):
            raise TypeError("usb_can must be UsbCanConfig or a configuration mapping")
        if set(value) - {"selector", "libusb_library"} or "selector" not in value:
            raise ValueError("usb_can requires selector and optional libusb_library")
        selector = value["selector"]
        if isinstance(selector, Mapping):
            try:
                selector = UsbCanSelector(**dict(selector))
            except TypeError as error:
                raise ValueError(
                    "usb_can.selector requires exact USB identity fields"
                ) from error
        return cls(selector, value.get("libusb_library"))


__all__ = ["SUPPORTED_DEVICE_IDS", "UsbCanConfig", "UsbCanSelector"]
