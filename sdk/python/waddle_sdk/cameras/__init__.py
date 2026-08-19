"""Camera capture contracts and optional vendor adapters.

Importing this package never imports a vendor SDK.  Install and import the
adapter you need explicitly (``waddle_sdk.cameras.depthai``,
``waddle_sdk.cameras.realsense``, or ``waddle_sdk.cameras.orbbec``); each
adapter loads its vendor package only when a
driver is constructed.
"""

from .base import CameraCalibrationDriver, CameraDriver, CameraFrame, CameraSample

__all__ = ["CameraCalibrationDriver", "CameraDriver", "CameraFrame", "CameraSample"]
