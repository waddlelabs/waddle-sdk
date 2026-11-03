"""Rigid brush, open dustpan and loose pieces for native tabletop cleanup.

All movement and containment follow ordinary contact dynamics. The brush has a
raised handle and a finite rigid sweeping pad, not simulated flexible bristles.
"""

import math
import xml.etree.ElementTree as ET

from .model import numbers


def _body(world, name, position, *, free=False):
    body = ET.SubElement(world, "body", name=name, pos=numbers(position))
    if free:
        ET.SubElement(body, "freejoint", name=name + "_free")
    return body


def _box(body, name, size, position, color, *, mass=None, pitch=0):
    attributes = {
        "type": "box",
        "size": numbers([value / 2 for value in size]),
        "pos": numbers(position),
        "rgba": numbers([*color, 1]),
        "quat": numbers([math.cos(pitch / 2), 0, math.sin(pitch / 2), 0]),
    }
    ET.SubElement(
        body,
        "geom",
        **attributes,
        name=name + "_visual",
        group="2",
        contype="0",
        conaffinity="0",
        mass="0",
    )
    collision = ET.SubElement(
        body,
        "geom",
        **attributes,
        name=name + "_collision",
        group="3",
        friction="0.6 0.001 0.0001",
        solref="0.004 1",
    )
    if mass is not None:
        collision.set("mass", str(mass))


def add_cleanup_station(world):
    """Add a graspable tool and fixed pan with a shallow entrance ramp."""
    brush = _body(world, "brush", (0.25, 0.21, 0.0125), free=True)
    _box(
        brush,
        "brush_handle",
        (0.11, 0.024, 0.024),
        (-0.025, 0, 0.015),
        (0.85, 0.50, 0.20),
        mass=0.045,
    )
    _box(
        brush,
        "brush_pad",
        (0.035, 0.10, 0.024),
        (0.055, 0, 0),
        (0.18, 0.27, 0.40),
        mass=0.025,
    )
    _box(
        brush,
        "brush_heel",
        (0.015, 0.024, 0.024),
        (-0.07, 0, 0),
        (0.85, 0.50, 0.20),
        mass=0.010,
    )

    pan = _body(world, "dustpan", (0.44, 0.08, 0))
    blue = (0.20, 0.48, 0.82)
    _box(pan, "dustpan_floor", (0.14, 0.18, 0.003), (0.01, 0, 0.0015), blue)
    _box(
        pan,
        "dustpan_ramp",
        (0.04, 0.18, 0.001),
        (-0.08, 0, 0.0005),
        blue,
        pitch=-math.atan(0.003 / 0.04),
    )
    for sign in (-1, 1):
        _box(
            pan,
            f"dustpan_side_{sign}",
            (0.18, 0.006, 0.03),
            (-0.01, sign * 0.093, 0.015),
            blue,
        )
    _box(pan, "dustpan_back", (0.006, 0.192, 0.03), (0.083, 0, 0.015), blue)

    for index, (x, y) in enumerate(
        (
            (0.25, -0.04),
            (0.29, -0.02),
            (0.32, 0.03),
            (0.25, 0.07),
            (0.29, 0.10),
            (0.32, 0.13),
        )
    ):
        body = _body(world, f"debris_{index}", (x, y, 0.0095), free=True)
        _box(
            body,
            f"debris_{index}",
            (0.018, 0.018, 0.018),
            (0, 0, 0),
            (0.94, 0.45 + 0.04 * (index % 3), 0.17),
            mass=0.006,
        )
