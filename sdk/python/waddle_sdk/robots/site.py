"""Manifest-facing robot driver extension contract."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class PartConfig:
    name: str
    posture: str
    connection: Mapping[str, Any]
    joint_limits: object
    workspace_bounds: Mapping[str, Any]
    envelope: Mapping[str, Any]
    options: Mapping[str, Any] = field(default_factory=dict)
    site_root: Path = Path(".")


__all__ = ["PartConfig"]
