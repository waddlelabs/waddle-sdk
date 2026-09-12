"""Optional physics environments implementing ordinary robot/camera contracts.

Importing this package never imports a physics engine or opens a device.
"""

from .scene import (
    BACKENDS,
    DUAL_ARM_TASK_ENVIRONMENTS,
    ENVIRONMENTS,
    RENDER_QUALITIES,
    ROBOTS,
    SINGLE_ARM_TASK_ENVIRONMENTS,
    TASK_ENVIRONMENTS,
    load_scene,
    make_site,
)
from .sources import reference_model_sources

__all__ = [
    "BACKENDS",
    "DUAL_ARM_TASK_ENVIRONMENTS",
    "ENVIRONMENTS",
    "RENDER_QUALITIES",
    "ROBOTS",
    "SINGLE_ARM_TASK_ENVIRONMENTS",
    "TASK_ENVIRONMENTS",
    "load_scene",
    "make_site",
    "reference_model_sources",
]
