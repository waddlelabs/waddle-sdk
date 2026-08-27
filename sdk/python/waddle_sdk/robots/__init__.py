"""Robot modules: what a customer imports instead of writing a driver.

`waddle_sdk` itself is deliberately robot-agnostic — it declares, gates and
records whatever machine you describe to it. This subpackage is the other
half of that bargain for people who own a machine somebody else has already
described: `waddle_sdk.robots.<vendor>` carries that robot's facts and its
driver, and `waddle_sdk.robots.base` carries everything about a robot module that
is not a vendor fact.

Nothing here is imported by `import waddle_sdk`, and nothing here decides
anything Waddle decides: a robot module builds a declaration, opens a driver,
and enforces the OWNER's envelope on the way to it. Claims, leases, handoffs
and timelines stay in waddle-core, exactly as they do for a program that
writes its own driver.

Every layer is usable alone, and that is the design rather than an accident:
take the declaration and wire `the Site lifecycle` yourself, bring your own driver,
bring your own envelope, or run your own loop. The rig factory each vendor
module exposes is composition sugar over those pieces — never a wall around
them.
"""

from __future__ import annotations

from . import base
from .base import Driver, PositionVelocityDriver
from .safety import (
    SafetyPreset,
    SafetyPresetProvider,
    SafetyPresetReport,
    safety_presets_for_driver,
)
from .site import PartConfig

__all__ = [
    "Driver",
    "PartConfig",
    "PositionVelocityDriver",
    "SafetyPreset",
    "SafetyPresetProvider",
    "SafetyPresetReport",
    "base",
    "safety_presets_for_driver",
]
