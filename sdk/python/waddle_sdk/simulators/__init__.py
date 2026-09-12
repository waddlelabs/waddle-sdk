"""Optional physics environments implementing ordinary robot/camera contracts.

Importing this package never imports a physics engine or opens a device.
"""

from .scene import (
    BACKENDS,
    ENVIRONMENTS,
    RENDER_QUALITIES,
    ROBOTS,
    load_scene,
    make_site,
)
from .sources import reference_model_sources

__all__ = [
    "BACKENDS",
    "ENVIRONMENTS",
    "RENDER_QUALITIES",
    "ROBOTS",
    "load_scene",
    "make_site",
    "reference_model_sources",
]
