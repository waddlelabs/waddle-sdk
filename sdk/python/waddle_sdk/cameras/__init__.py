"""Camera capture contracts and optional vendor adapters.

Importing this package never imports a vendor SDK.  Install and import the
adapter you need explicitly (``waddle_sdk.cameras.realsense`` or
``waddle_sdk.cameras.orbbec``); each adapter loads its vendor package only when a
driver is constructed.
"""

from .base import CameraCalibrationDriver, CameraDriver, CameraFrame, CameraSample
from .inspection import (
    CameraInspection,
    CameraInspectionError,
    CameraInspectionFrame,
    CameraInspectionSession,
    CameraInspectionSpec,
    inspect_cameras,
)
from .site import CameraConfig, CameraMount

__all__ = [
    "CameraCalibrationDriver",
    "CameraConfig",
    "CameraDriver",
    "CameraFrame",
    "CameraInspection",
    "CameraInspectionError",
    "CameraInspectionFrame",
    "CameraInspectionSession",
    "CameraInspectionSpec",
    "CameraMount",
    "CameraSample",
    "inspect_cameras",
]
