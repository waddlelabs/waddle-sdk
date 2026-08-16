"""Waddle SDK: site-owned hardware, safety, timing, and recording.

The root package intentionally exposes only the strict Site lifecycle,
transport selection, outcomes, and manifest errors. Driver extension APIs
live in waddle_sdk.robots and waddle_sdk.cameras.

There is no process-global execution path. A SiteSession owns exactly one
native session and closes it deterministically with its context.
"""

from __future__ import annotations

import enum

from ._native import core
from .site import (
    ManifestError,
    ManifestPathError,
    ManifestSyntaxError,
    ManifestValidationError,
    Run,
    Site,
    SiteSession,
    load_site,
)
from .transport import Grpc, LiveKit

__version__: str = core.__version__


class Outcome(str, enum.Enum):
    """Terminal run outcomes."""

    SUCCESS = "success"
    FAILURE = "failure"
    ABORT = "abort"


__all__ = [
    "Grpc",
    "LiveKit",
    "ManifestError",
    "ManifestPathError",
    "ManifestSyntaxError",
    "ManifestValidationError",
    "Outcome",
    "Run",
    "Site",
    "SiteSession",
    "load_site",
]
