"""Optional physics environments implementing ordinary robot/camera contracts.

Importing this package never imports a physics engine or opens a device.
"""

from .scene import BACKENDS, ENVIRONMENTS, RENDER_QUALITIES, ROBOTS, make_site

__all__ = ["BACKENDS", "ENVIRONMENTS", "RENDER_QUALITIES", "ROBOTS", "make_site"]
