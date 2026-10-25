"""Camera capture contracts and optional vendor adapters.

Importing this package never imports a vendor SDK.  Install and import the
adapter you need explicitly (``waddle_sdk.cameras.realsense`` or
``waddle_sdk.cameras.orbbec``); each adapter loads its vendor package only when a
driver is constructed.
"""

from .base import (
    CameraCalibrationDriver,
    CameraCaptureStateDriver,
    CameraDriver,
    CameraFrame,
    CameraSample,
)
from .capture_state import CameraCaptureState
from .inspection import (
    CameraInspection,
    CameraInspectionError,
    CameraInspectionFrame,
    CameraInspectionSession,
    CameraInspectionSpec,
    inspect_cameras,
)
from .site import CameraConfig, CameraMount
from .timing import CameraContentTiming

__all__ = [
    "CameraCalibrationDriver",
    "CameraCaptureState",
    "CameraCaptureStateDriver",
    "CameraConfig",
    "CameraContentTiming",
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
