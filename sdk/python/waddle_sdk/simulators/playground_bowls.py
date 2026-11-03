"""Rounded hollow bowls with smooth visuals and separate convex wall contacts."""

import math
import xml.etree.ElementTree as ET
from itertools import pairwise

from .model import numbers


def _profile(radius, height, steps):
    wall = min(0.0035, radius / 12, height / 12)
    floor = min(0.004, height / 10)
    bottom = -height / 2

    # An ellipsoidal side flares from a small flat foot into a deep open rim.
    def side(t):
        return math.sqrt(0.28**2 + (1 - 0.28**2) * (2 * t - t * t))

    outer = [
        (radius * side(i / steps), bottom + floor + (height - floor) * i / steps)
        for i in range(steps + 1)
    ]
    inner = [
        (
            (radius - wall) * side(i / steps),
            bottom + floor + (height - floor) * i / steps,
        )
        for i in range(steps + 1)
    ]
    return outer, inner, floor


def _lathe(points, segments=80):
    vertices, rings, faces = [], [], []
    for radius, z in points:
        ring = []
        for i in range(segments if radius else 1):
            angle = 2 * math.pi * i / segments
            ring.append(len(vertices))
            vertices.append((radius * math.cos(angle), radius * math.sin(angle), z))
        rings.append(ring)
    for first, second in pairwise(rings):
        for i in range(segments):
            j = (i + 1) % segments
            if len(first) == 1:
                faces.append((first[0], second[j], second[i]))
            elif len(second) == 1:
                faces.append((first[i], first[j], second[0]))
            else:
                faces.extend(
                    ((first[i], first[j], second[i]), (first[j], second[j], second[i]))
                )
    return vertices, faces


def bowl_geometry(asset, radius, height, name):
    """Return a closed visual shell and curved bands of analytic contacts that leave the cavity open.

    Each wall band is a separate convex solid; a single mesh collider would fill
    the bowl's opening with its convex hull. The floor alone contacts the table
    when upright. Native contacts, finite mass and ordinary release stay intact.
    """
    outer, inner, floor = _profile(radius, height, 24)
    vertices, faces = _lathe(
        [
            (0, -height / 2),
            (outer[0][0], -height / 2),
            *outer,
            *reversed(inner),
            (0, -height / 2 + floor),
        ]
    )
    visual = name + "_shell"
    ET.SubElement(
        asset,
        "mesh",
        name=visual,
        vertex=numbers([v for p in vertices for v in p]),
        face=" ".join(str(v) for p in faces for v in p),
    )
    shapes = [("cylinder", (outer[0][0], floor / 2), (0, 0, -height / 2 + floor / 2))]
    outer, inner, _ = _profile(radius, height, 6)
    for band in range(6):
        for sector in range(16):
            vertices = []
            for angle in (2 * math.pi * sector / 16, 2 * math.pi * (sector + 1) / 16):
                for r, z in (
                    outer[band],
                    outer[band + 1],
                    inner[band],
                    inner[band + 1],
                ):
                    vertices.extend((r * math.cos(angle), r * math.sin(angle), z))
            mesh = f"{name}_wall_{band}_{sector}"
            ET.SubElement(asset, "mesh", name=mesh, vertex=numbers(vertices))
            shapes.append(("mesh", mesh, (0, 0, 0)))
    return visual, shapes
