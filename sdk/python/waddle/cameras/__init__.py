"""Camera capture contracts and optional vendor adapters.

Importing this package never imports a vendor SDK.  Install and import the
adapter you need explicitly (``waddle.cameras.realsense`` or
``waddle.cameras.orbbec``); each adapter loads its vendor package only when a
driver is constructed.
"""

from .base import CameraDriver, CameraFrame, CameraSample

__all__ = ["CameraDriver", "CameraFrame", "CameraSample"]
