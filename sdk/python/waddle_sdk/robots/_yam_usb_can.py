"""Bind the reviewed YAM motor interface to one explicitly selected USB bus."""

from __future__ import annotations

from ._i2rt_runtime import verify_transport_runtime


def prepare(config):
    """Verify source before optional/vendor imports; constructing opens no bus.

    Each interface owns a fresh bus. The reviewed I2RT implementation retains
    motor codecs, feedback, gains, gravity compensation and gripper behavior.
    """
    source = verify_transport_runtime()
    from i2rt.motor_drivers.dm_driver import DMSingleMotorCanInterface

    from .usbcan import UsbCanBus

    def bus_factory(*, bustype, channel, bitrate):
        if channel != config.selector.channel or bitrate != 1_000_000:
            raise ValueError(
                "Reviewed USB YAM requires its selected channel at 1000000 bit/s"
            )
        return UsbCanBus(
            config.selector, bitrate=bitrate, libusb_library=config.libusb_library
        )

    def motor_interface_factory(**kwargs):
        return DMSingleMotorCanInterface(
            **kwargs, bustype="gs_usb", bus_factory=bus_factory
        )

    return motor_interface_factory, source
