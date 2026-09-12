"""Manufacturer robot descriptions and reference props in native scene formats."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field, replace
from importlib.resources import files
from typing import Any

import numpy as np

from .scene import Profile, quaternion


def numbers(values: Any) -> str:
    return " ".join(str(float(x)) for x in values)


@dataclass
class Shape:
    kind: str
    size: tuple[float, ...]
    xyz: tuple[float, ...] = (0.0, 0.0, 0.0)
    rpy: tuple[float, ...] = (0.0, 0.0, 0.0)
    color: tuple[float, ...] = (0.3, 0.35, 0.4, 1.0)
    mesh: str | None = None
    visual: bool = True
    collision: bool = True
    texture: str | None = None
    friction: tuple[float, float, float] | None = None


@dataclass
class Link:
    name: str
    parent: str | None = None
    xyz: tuple[float, ...] = (0.0, 0.0, 0.0)
    rpy: tuple[float, ...] = (0.0, 0.0, 0.0)
    joint: str | None = None
    kind: str = "fixed"
    axis: tuple[float, ...] = (0.0, 0.0, 1.0)
    limits: tuple[float, float] = (0.0, 0.0)
    mass: float = 0.3
    shapes: list[Shape] = field(default_factory=list)
    com: tuple[float, ...] = (0.0, 0.0, 0.0)
    inertia: tuple[float, ...] | None = None  # xx, yy, zz, xy, xz, yz in link frame
    mimic: tuple[str, float, float] | None = None
    damping: float = 0.0  # passive joint resistance in SI units
    initial: float = 0.0  # passive joint initial position in metres/radians
    stiffness: float = 0.0  # passive joint spring in SI units
    spring_reference: float = 0.0  # spring rest position in metres/radians


def robot_links(p: Profile) -> list[Link]:
    from .description import description

    return description(p.name).native_links()


def objects(environment: str, *, robot: str | None = None) -> list[list[Link]]:
    table = [
        Link(
            "table",
            xyz=(0.35, 0.0, -0.035),
            shapes=[
                Shape(
                    "box",
                    (1.2, 1.0, 0.07),
                    color=(1.0, 1.0, 1.0, 1.0),
                    texture=str(
                        files(__package__).joinpath(
                            "data/appearance/wood_table_001_diff_1k.png"
                        )
                    ),
                )
            ],
        )
    ]

    def free_box(name, xyz, color, *, size=0.046, mass=0.05):
        inertia = mass * size**2 / 6
        return Link(
            name,
            xyz=xyz,
            kind="free",
            mass=mass,
            inertia=(inertia, inertia, inertia, 0.0, 0.0, 0.0),
            shapes=[Shape("box", (size, size, size), color=color)],
        )

    def free_cylinder(
        name,
        xyz,
        color,
        *,
        radius=0.018,
        length=0.10,
        mass=0.05,
        rpy=(0.0, 0.0, 0.0),
    ):
        transverse = mass * (3 * radius**2 + length**2) / 12
        axial = mass * radius**2 / 2
        return Link(
            name,
            xyz=xyz,
            rpy=rpy,
            kind="free",
            mass=mass,
            inertia=(transverse, transverse, axial, 0.0, 0.0, 0.0),
            shapes=[Shape("cylinder", (radius, length), color=color)],
        )

    def marked_region(name, xyz, *, size=(0.16, 0.16)):
        return Link(
            name,
            xyz=xyz,
            shapes=[
                Shape(
                    "box",
                    (*size, 0.001),
                    color=(0.12, 0.35, 0.95, 1.0),
                    collision=False,
                )
            ],
        )

    def collar_shapes(*, radius=0.016, height=0.025, color=(0.1, 0.3, 0.85, 1.0)):
        shapes = []
        for index in range(12):
            angle = 2 * math.pi * index / 12
            shapes.append(
                Shape(
                    "box",
                    (0.012, 0.008, height),
                    (radius * math.cos(angle), radius * math.sin(angle), height / 2),
                    (0.0, 0.0, angle + math.pi / 2),
                    color=color,
                )
            )
        return shapes

    def tray_fixture(name, *, xyz, loaded=False):
        tray = Link(
            name,
            xyz=xyz,
            kind="free",
            mass=0.5,
            inertia=(0.0045, 0.0032, 0.0072, 0.0, 0.0, 0.0),
            shapes=[
                Shape("box", (0.26, 0.32, 0.02), (0.0, 0.0, 0.01)),
                Shape("box", (0.26, 0.012, 0.05), (0.0, -0.154, 0.035)),
                Shape("box", (0.26, 0.012, 0.05), (0.0, 0.154, 0.035)),
                Shape("box", (0.012, 0.296, 0.05), (-0.124, 0.0, 0.035)),
                Shape("box", (0.012, 0.296, 0.05), (0.124, 0.0, 0.035)),
            ],
        )
        handles = [
            Link(
                f"{name}_handle_{side}",
                parent=name,
                xyz=(0.0, sign * 0.20, 0.055),
                mass=1e-6,
                inertia=(1e-9, 1e-9, 1e-9, 0.0, 0.0, 0.0),
                shapes=[
                    Shape(
                        "box",
                        (0.12, 0.022, 0.022),
                        color=(0.95, 0.45, 0.08, 1.0),
                    )
                ],
            )
            for side, sign in (("left", -1.0), ("right", 1.0))
        ]
        if not loaded:
            return [tray, *handles], []
        contents = [
            free_box(
                f"tray_content_{index}",
                (xyz[0] + x, xyz[1] + y, 0.045),
                color,
                size=0.04,
                mass=0.04,
            )
            for index, (x, y, color) in enumerate(
                (
                    (-0.055, -0.06, (0.1, 0.72, 0.2, 1.0)),
                    (0.055, 0.06, (0.68, 0.16, 0.82, 1.0)),
                ),
                start=1,
            )
        ]
        return [tray, *handles], contents

    def open_bin(
        name,
        xyz,
        color=(0.1, 0.3, 0.85, 1.0),
        *,
        kind="fixed",
        mass=0.3,
    ):
        return Link(
            name,
            xyz=xyz,
            kind=kind,
            mass=mass,
            inertia=(0.0017, 0.0017, 0.0017, 0.0, 0.0, 0.0),
            shapes=[
                Shape("box", (0.18, 0.18, 0.01), (0.0, 0.0, 0.005), color=color),
                Shape("box", (0.01, 0.18, 0.08), (-0.085, 0.0, 0.04), color=color),
                Shape("box", (0.01, 0.18, 0.08), (0.085, 0.0, 0.04), color=color),
                Shape("box", (0.16, 0.01, 0.08), (0.0, -0.085, 0.04), color=color),
                Shape("box", (0.16, 0.01, 0.08), (0.0, 0.085, 0.04), color=color),
            ],
        )

    def drawer_fixture(
        *,
        initial=0.0,
        cabinet_kind="fixed",
        cabinet_name="cabinet",
        include_interior=False,
    ):
        cabinet = Link(
            cabinet_name,
            xyz=(0.62 if robot == "so101" else 0.66, 0.0, 0.0),
            kind=cabinet_kind,
            mass=1.5,
            inertia=(0.02, 0.025, 0.015, 0.0, 0.0, 0.0),
            shapes=[
                Shape("box", (0.3, 0.32, 0.018), (0.02, 0.0, 0.009)),
                Shape("box", (0.3, 0.018, 0.24), (0.02, -0.151, 0.12)),
                Shape("box", (0.3, 0.018, 0.24), (0.02, 0.151, 0.12)),
                Shape("box", (0.018, 0.32, 0.24), (0.161, 0.0, 0.12)),
                Shape("box", (0.3, 0.32, 0.018), (0.02, 0.0, 0.24)),
            ],
        )
        drawer = Link(
            "drawer",
            cabinet_name,
            xyz=(0.0, 0.0, 0.04),
            joint="drawer_slide",
            kind="prismatic",
            axis=(-1.0, 0.0, 0.0),
            limits=(0.0, 0.22),
            mass=0.5,
            damping=5.0,
            initial=initial,
            shapes=[
                Shape(
                    "box",
                    (0.26, 0.27, 0.012),
                    (0.0, 0.0, 0.01),
                    color=(0.6, 0.5, 0.4, 1.0),
                ),
                Shape("box", (0.016, 0.28, 0.16), (-0.13, 0.0, 0.075)),
                Shape(
                    "box",
                    (0.024, 0.09, 0.018),
                    (-0.157, 0.0, 0.1),
                    color=(0.85, 0.85, 0.85, 1.0),
                ),
            ],
        )
        fixture = [cabinet, drawer]
        if include_interior:
            fixture.append(
                Link(
                    "drawer_interior",
                    parent="drawer",
                    xyz=(0.0, 0.0, 0.055),
                    mass=1e-6,
                    inertia=(1e-9, 1e-9, 1e-9, 0.0, 0.0, 0.0),
                )
            )
        return fixture

    if environment == "two_cubes":
        return [
            table,
            *[
                [
                    Link(
                        f"cube_{i}",
                        xyz=(0.32, y, 0.026),
                        kind="free",
                        mass=0.06,
                        # Uniform 50 mm cube: I = mass * side**2 / 6.
                        inertia=(0.000025, 0.000025, 0.000025, 0.0, 0.0, 0.0),
                        shapes=[Shape("box", (0.05, 0.05, 0.05), color=color)],
                    )
                ]
                for i, y, color in (
                    (1, -0.10, (0.08, 0.8, 0.1, 1.0)),
                    (2, 0.1, (0.1, 0.35, 0.9, 1.0)),
                )
            ],
        ]
    if environment == "split_workspace_sorting":
        orange = (0.95, 0.35, 0.05, 1.0)
        green = (0.08, 0.72, 0.18, 1.0)

        def cube(name, y, color):
            return Link(
                name,
                xyz=(0.26, y, 0.02),
                kind="free",
                mass=0.04,
                inertia=(0.0000107, 0.0000107, 0.0000107, 0.0, 0.0, 0.0),
                shapes=[Shape("box", (0.04, 0.04, 0.04), color=color)],
            )

        def goal_bin(name, y, color):
            return Link(
                name,
                xyz=(0.32, y, 0.0),
                shapes=[
                    Shape("box", (0.11, 0.11, 0.01), (0.0, 0.0, 0.005), color=color),
                    Shape(
                        "box", (0.008, 0.11, 0.07), (-0.051, 0.0, 0.035), color=color
                    ),
                    Shape("box", (0.008, 0.11, 0.07), (0.051, 0.0, 0.035), color=color),
                    Shape(
                        "box", (0.094, 0.008, 0.07), (0.0, -0.051, 0.035), color=color
                    ),
                    Shape(
                        "box", (0.094, 0.008, 0.07), (0.0, 0.051, 0.035), color=color
                    ),
                ],
            )

        return [
            table,
            [cube("left_object", -0.44, orange)],
            [cube("right_object", 0.44, green)],
            [goal_bin("right_goal_bin", 0.065, orange)],
            [goal_bin("left_goal_bin", -0.065, green)],
        ]
    if environment == "handover-block":
        return [
            table,
            [
                free_box(
                    "handover_block",
                    (0.28, -0.30, 0.023),
                    (0.95, 0.45, 0.08, 1.0),
                )
            ],
            [
                Link(
                    "goal_region",
                    xyz=(0.32, 0.30, 0.0005),
                    shapes=[
                        Shape(
                            "box",
                            (0.16, 0.16, 0.001),
                            color=(0.12, 0.35, 0.95, 1.0),
                            collision=False,
                        )
                    ],
                )
            ],
        ]
    if environment == "stabilize-open-drawer":
        return [
            table,
            drawer_fixture(
                cabinet_kind="free",
                cabinet_name="movable_cabinet",
            ),
        ]
    if environment == "hold-container-place":
        return [
            table,
            [
                open_bin(
                    "movable_container",
                    (0.38, -0.12, 0.0),
                    kind="free",
                    mass=0.3,
                )
            ],
            [
                free_box(
                    "target_object",
                    (0.30, 0.20, 0.023),
                    (0.95, 0.45, 0.08, 1.0),
                )
            ],
        ]
    if environment == "stabilize-remove-lid":
        box = Link(
            "movable_box",
            xyz=(0.36, -0.08, 0.0),
            kind="free",
            mass=0.35,
            inertia=(0.0012, 0.0012, 0.0015, 0.0, 0.0, 0.0),
            shapes=[
                Shape("box", (0.16, 0.16, 0.01), (0.0, 0.0, 0.005)),
                Shape("box", (0.01, 0.16, 0.08), (-0.075, 0.0, 0.04)),
                Shape("box", (0.01, 0.16, 0.08), (0.075, 0.0, 0.04)),
                Shape("box", (0.14, 0.01, 0.08), (0.0, -0.075, 0.04)),
                Shape("box", (0.14, 0.01, 0.08), (0.0, 0.075, 0.04)),
            ],
        )
        lid_grips = [
            Link(
                f"lid_grip_{side}",
                parent="movable_box",
                xyz=(sign * 0.081, 0.0, 0.06),
                joint=f"lid_grip_{side}",
                kind="prismatic",
                axis=(-sign, 0.0, 0.0),
                limits=(0.0, 0.01),
                mass=0.01,
                damping=0.2,
                initial=0.0044,
                stiffness=400.0,
                inertia=(0.000002, 0.0000002, 0.000002, 0.0, 0.0, 0.0),
                shapes=[
                    Shape(
                        "box",
                        (0.01, 0.05, 0.02),
                        color=(0.72, 0.75, 0.8, 1.0),
                    )
                ],
            )
            for side, sign in (("left", -1.0), ("right", 1.0))
        ]
        lid = Link(
            "box_lid",
            xyz=(0.36, -0.08, 0.0875),
            kind="free",
            mass=0.12,
            inertia=(0.00033, 0.00033, 0.00064, 0.0, 0.0, 0.0),
            shapes=[
                Shape(
                    "box",
                    (0.18, 0.18, 0.015),
                    color=(0.95, 0.45, 0.08, 1.0),
                ),
                Shape(
                    "box",
                    (0.06, 0.03, 0.025),
                    (0.0, 0.0, 0.02),
                    color=(0.95, 0.45, 0.08, 1.0),
                ),
                # The skirt clears the rigid box walls but compresses the two
                # passive spring pads. Their finite normal load supplies the
                # friction fit without requiring either rigid body to deform.
                Shape(
                    "box",
                    (0.008, 0.18, 0.03),
                    (-0.085, 0.0, -0.015),
                    color=(0.95, 0.45, 0.08, 1.0),
                ),
                Shape(
                    "box",
                    (0.008, 0.18, 0.03),
                    (0.085, 0.0, -0.015),
                    color=(0.95, 0.45, 0.08, 1.0),
                ),
                Shape(
                    "box",
                    (0.158, 0.008, 0.03),
                    (0.0, -0.085, -0.015),
                    color=(0.95, 0.45, 0.08, 1.0),
                ),
                Shape(
                    "box",
                    (0.158, 0.008, 0.03),
                    (0.0, 0.085, -0.015),
                    color=(0.95, 0.45, 0.08, 1.0),
                ),
            ],
        )
        goal = Link(
            "lid_goal_region",
            xyz=(0.36, 0.25, 0.0005),
            shapes=[
                Shape(
                    "box",
                    (0.22, 0.20, 0.001),
                    color=(0.12, 0.35, 0.95, 1.0),
                    collision=False,
                )
            ],
        )
        return [table, [box, *lid_grips], [lid], [goal]]
    if environment == "oriented-tool-handover":
        tool = Link(
            "handled_tool",
            xyz=(0.30, -0.30, 0.012),
            kind="free",
            mass=0.16,
            inertia=(0.00035, 0.0012, 0.0012, 0.0, 0.0, 0.0),
            shapes=[
                Shape(
                    "box",
                    (0.16, 0.022, 0.022),
                    color=(0.95, 0.45, 0.08, 1.0),
                ),
                Shape(
                    "box",
                    (0.045, 0.11, 0.035),
                    (0.065, 0.0, 0.012),
                    color=(0.72, 0.75, 0.8, 1.0),
                ),
            ],
        )
        marker = Link(
            "tool_goal_pose",
            xyz=(0.32, 0.30, 0.0005),
            shapes=[
                Shape(
                    "box",
                    (0.18, 0.03, 0.001),
                    color=(0.12, 0.35, 0.95, 1.0),
                    collision=False,
                ),
                Shape(
                    "box",
                    (0.045, 0.12, 0.001),
                    (0.065, 0.0, 0.0),
                    color=(0.12, 0.35, 0.95, 1.0),
                    collision=False,
                ),
            ],
        )
        return [table, [tool], [marker]]
    if environment == "two-arm-peg-insertion":
        receiver = Link(
            "receiving_part",
            xyz=(0.39, -0.18, 0.0),
            kind="free",
            mass=0.30,
            inertia=(0.0008, 0.0008, 0.0012, 0.0, 0.0, 0.0),
            shapes=[
                Shape(
                    "box",
                    (0.12, 0.12, 0.012),
                    (0.0, 0.0, 0.006),
                    color=(0.12, 0.35, 0.95, 1.0),
                )
            ],
        )
        socket = Link(
            "target_socket",
            parent="receiving_part",
            xyz=(0.0, 0.0, 0.012),
            mass=1e-6,
            inertia=(1e-9, 1e-9, 1e-9, 0.0, 0.0, 0.0),
            shapes=collar_shapes(radius=0.023, height=0.04),
        )
        peg = free_cylinder(
            "target_peg",
            (0.29, 0.20, 0.012),
            (0.95, 0.45, 0.08, 1.0),
            radius=0.012,
            length=0.08,
            rpy=(0.0, math.pi / 2, 0.0),
        )
        peg.damping = 0.001
        peg.shapes[0].friction = (1.0, 0.05, 0.01)
        return [table, [receiver, socket], [peg]]
    if environment == "joint-lift":
        tray, _ = tray_fixture("two_handle_tray", xyz=(0.40, 0.0, 0.0))
        return [table, tray]
    if environment == "loaded-tray-transport":
        tray, contents = tray_fixture(
            "loaded_tray", xyz=(0.39, -0.14, 0.0), loaded=True
        )
        return [
            table,
            tray,
            *[[content] for content in contents],
            [marked_region("goal_region", (0.34, 0.24, 0.0005), size=(0.32, 0.36))],
        ]
    if environment == "uncap-return-test-tube":
        tube = free_cylinder(
            "target_test_tube",
            (0.38, 0.0, 0.06),
            (0.55, 0.88, 0.96, 0.38),
            radius=0.009,
            length=0.10,
            mass=0.015,
        )
        cap_grips = [
            Link(
                f"tube_cap_grip_{side}",
                parent="target_test_tube",
                xyz=(sign * 0.010, 0.0, 0.045),
                joint=f"tube_cap_grip_{side}",
                kind="prismatic",
                axis=(-sign, 0.0, 0.0),
                limits=(0.0, 0.006),
                mass=0.004,
                damping=0.1,
                initial=0.002,
                stiffness=180.0,
                inertia=(2e-7, 2e-8, 2e-7, 0.0, 0.0, 0.0),
                shapes=[
                    Shape(
                        "box",
                        (0.006, 0.012, 0.012),
                        color=(0.72, 0.75, 0.8, 1.0),
                    )
                ],
            )
            for side, sign in (("left", -1.0), ("right", 1.0))
        ]
        cap_shapes = [
            Shape(
                "cylinder",
                (0.014, 0.020),
                color=(0.95, 0.45, 0.08, 1.0),
            )
        ]
        for index in range(12):
            angle = 2 * math.pi * index / 12
            cap_shapes.append(
                Shape(
                    "box",
                    (0.010, 0.005, 0.025),
                    (0.012 * math.cos(angle), 0.012 * math.sin(angle), -0.0125),
                    (0.0, 0.0, angle + math.pi / 2),
                    color=(0.95, 0.45, 0.08, 1.0),
                )
            )
        cap = Link(
            "target_tube_cap",
            xyz=(0.38, 0.0, 0.125),
            kind="free",
            mass=0.012,
            inertia=(0.000001, 0.000001, 0.000001, 0.0, 0.0, 0.0),
            shapes=cap_shapes,
        )
        rack = Link(
            "tube_rack",
            xyz=(0.38, 0.0, 0.0),
            shapes=[
                Shape(
                    "box",
                    (0.10, 0.10, 0.01),
                    (0.0, 0.0, 0.005),
                    color=(0.10, 0.30, 0.85, 1.0),
                )
            ],
        )
        slot = Link(
            "assigned_rack_slot",
            parent="tube_rack",
            xyz=(0.0, 0.0, 0.01),
            mass=1e-6,
            inertia=(1e-9, 1e-9, 1e-9, 0.0, 0.0, 0.0),
            shapes=collar_shapes(),
        )
        return [
            table,
            [rack, slot],
            [tube, *cap_grips],
            [cap],
            [marked_region("cap_table_region", (0.29, 0.18, 0.0005))],
        ]
    if environment == "retrieve-bottle-clutter":
        bottles = [
            free_cylinder(
                name,
                (x, y, 0.065),
                color,
                radius=0.016,
                length=0.11,
                mass=0.06,
            )
            for name, x, y, color in (
                ("target_bottle", 0.42, -0.025, (0.95, 0.45, 0.08, 1.0)),
                ("other_bottle_1", 0.46, -0.035, (0.1, 0.72, 0.2, 1.0)),
                ("other_bottle_2", 0.415, 0.025, (0.68, 0.16, 0.82, 1.0)),
                ("other_bottle_3", 0.46, 0.030, (0.15, 0.35, 0.95, 1.0)),
            )
        ]
        return [
            table,
            [open_bin("bottle_bin", (0.44, 0.0, 0.0))],
            *[[bottle] for bottle in bottles],
            [marked_region("table_goal_region", (0.29, 0.26, 0.0005))],
        ]
    if environment == "touch_target":
        pads = (
            ("distractor_pad_left", -0.12, (0.15, 0.35, 0.95, 1.0)),
            ("target_pad", 0.0, (0.95, 0.18, 0.12, 1.0)),
            ("distractor_pad_right", 0.12, (0.15, 0.35, 0.95, 1.0)),
        )
        return [
            table,
            *[
                [
                    Link(
                        name,
                        xyz=(0.40, y, 0.16),
                        shapes=[Shape("box", (0.015, 0.08, 0.08), color=color)],
                    )
                ]
                for name, y, color in pads
            ],
        ]
    if environment == "pick_lift":
        return [
            table,
            [
                Link(
                    "target_cube",
                    xyz=(0.32, 0.0, 0.023),
                    kind="free",
                    mass=0.05,
                    inertia=(0.0000176, 0.0000176, 0.0000176, 0.0, 0.0, 0.0),
                    shapes=[
                        Shape("box", (0.046, 0.046, 0.046), color=(0.1, 0.72, 0.2, 1.0))
                    ],
                )
            ],
        ]
    if environment == "place_in_bin":
        target = Link(
            "target_cube",
            xyz=(0.28, -0.12, 0.023),
            kind="free",
            mass=0.05,
            inertia=(0.0000176, 0.0000176, 0.0000176, 0.0, 0.0, 0.0),
            shapes=[Shape("box", (0.046, 0.046, 0.046), color=(0.95, 0.45, 0.08, 1.0))],
        )
        goal = Link(
            "goal_bin",
            xyz=(0.42, 0.10, 0.0),
            shapes=[
                Shape(
                    "box",
                    (0.18, 0.18, 0.01),
                    (0.0, 0.0, 0.005),
                    color=(0.1, 0.3, 0.85, 1.0),
                ),
                Shape(
                    "box",
                    (0.01, 0.18, 0.08),
                    (-0.085, 0.0, 0.04),
                    color=(0.1, 0.3, 0.85, 1.0),
                ),
                Shape(
                    "box",
                    (0.01, 0.18, 0.08),
                    (0.085, 0.0, 0.04),
                    color=(0.1, 0.3, 0.85, 1.0),
                ),
                Shape(
                    "box",
                    (0.16, 0.01, 0.08),
                    (0.0, -0.085, 0.04),
                    color=(0.1, 0.3, 0.85, 1.0),
                ),
                Shape(
                    "box",
                    (0.16, 0.01, 0.08),
                    (0.0, 0.085, 0.04),
                    color=(0.1, 0.3, 0.85, 1.0),
                ),
            ],
        )
        return [table, [target], [goal]]
    if environment == "push_to_region":
        target = Link(
            "target_cube",
            xyz=(0.28, -0.10, 0.023),
            kind="free",
            mass=0.05,
            inertia=(0.0000176, 0.0000176, 0.0000176, 0.0, 0.0, 0.0),
            shapes=[Shape("box", (0.046, 0.046, 0.046), color=(0.95, 0.65, 0.08, 1.0))],
        )
        region = Link(
            "goal_region",
            xyz=(0.46, 0.10, 0.0005),
            shapes=[
                Shape(
                    "box",
                    (0.14, 0.14, 0.001),
                    color=(0.12, 0.35, 0.95, 1.0),
                    collision=False,
                )
            ],
        )
        return [table, [target], [region]]
    if environment == "operate_control":
        panel = Link(
            "control_panel",
            xyz=(0.45, 0.0, 0.0),
            shapes=[
                Shape(
                    "box",
                    (0.03, 0.34, 0.28),
                    (0.0, 0.0, 0.14),
                    color=(0.18, 0.2, 0.24, 1.0),
                )
            ],
        )
        buttons = []
        for name, y, color in (
            ("distractor_control_left", -0.105, (0.15, 0.35, 0.95, 1.0)),
            ("target_control", 0.0, (0.95, 0.16, 0.1, 1.0)),
            ("distractor_control_right", 0.105, (0.15, 0.35, 0.95, 1.0)),
        ):
            buttons.append(
                Link(
                    name,
                    parent="control_panel",
                    xyz=(-0.025, y, 0.14),
                    joint=name,
                    kind="prismatic",
                    axis=(1.0, 0.0, 0.0),
                    limits=(0.0, 0.012),
                    mass=0.02,
                    damping=1.0,
                    shapes=[
                        Shape(
                            "cylinder",
                            (0.026, 0.02),
                            rpy=(0.0, math.pi / 2, 0.0),
                            color=color,
                        )
                    ],
                )
            )
        return [table, [panel, *buttons]]
    if environment == "select-distractors":
        return [
            table,
            [
                free_box(
                    "target_object",
                    (0.26, -0.14, 0.023),
                    (0.95, 0.45, 0.08, 1.0),
                )
            ],
            [
                free_box(
                    "distractor_object_1",
                    (0.34, -0.03, 0.023),
                    (0.1, 0.72, 0.2, 1.0),
                )
            ],
            [
                free_box(
                    "distractor_object_2",
                    (0.25, 0.09, 0.023),
                    (0.68, 0.16, 0.82, 1.0),
                )
            ],
            [open_bin("goal_bin", (0.43, 0.13, 0.0))],
        ]
    if environment == "close-drawer":
        return [table, drawer_fixture(initial=0.15)]
    if environment == "stack-two-cubes":
        return [
            table,
            [free_box("cube_bottom", (0.27, -0.10, 0.023), (0.1, 0.72, 0.2, 1.0))],
            [free_box("cube_top", (0.35, 0.08, 0.023), (0.15, 0.35, 0.95, 1.0))],
        ]
    if environment == "ring-on-peg":
        segments = []
        segment_count = 12
        radius = 0.033
        for index in range(segment_count):
            angle = 2 * math.pi * index / segment_count
            segments.append(
                Shape(
                    "box",
                    (0.018, 0.012, 0.014),
                    (radius * math.cos(angle), radius * math.sin(angle), 0.0),
                    (0.0, 0.0, angle + math.pi / 2),
                    color=(0.95, 0.25, 0.08, 1.0),
                )
            )
        ring = Link(
            "target_ring",
            xyz=(0.28, -0.10, 0.008),
            kind="free",
            mass=0.04,
            inertia=(0.000022, 0.000022, 0.00004, 0.0, 0.0, 0.0),
            shapes=segments,
        )
        peg = Link(
            "target_peg",
            xyz=(0.43, 0.10, 0.0),
            shapes=[
                Shape(
                    "cylinder",
                    (0.012, 0.10),
                    (0.0, 0.0, 0.05),
                    color=(0.12, 0.35, 0.95, 1.0),
                )
            ],
        )
        return [table, [ring], [peg]]
    if environment == "use-hook":
        hook = Link(
            "hook",
            xyz=(0.27, -0.13, 0.009),
            kind="free",
            mass=0.05,
            inertia=(0.00008, 0.00008, 0.00012, 0.0, 0.0, 0.0),
            shapes=[
                Shape("box", (0.13, 0.014, 0.014), color=(0.95, 0.45, 0.08, 1.0)),
                Shape(
                    "box",
                    (0.014, 0.06, 0.014),
                    (0.058, 0.023, 0.0),
                    color=(0.95, 0.45, 0.08, 1.0),
                ),
                Shape(
                    "box",
                    (0.035, 0.014, 0.014),
                    (0.048, 0.052, 0.0),
                    color=(0.95, 0.45, 0.08, 1.0),
                ),
            ],
        )
        target = Link(
            "target_object",
            xyz=(0.44, 0.04, 0.015),
            kind="free",
            mass=0.06,
            inertia=(0.00002, 0.00002, 0.00003, 0.0, 0.0, 0.0),
            shapes=[
                Shape(
                    "cylinder",
                    (0.03, 0.03),
                    color=(0.1, 0.72, 0.2, 1.0),
                )
            ],
        )
        region = Link(
            "goal_region",
            xyz=(0.28, 0.15, 0.0005),
            shapes=[
                Shape(
                    "box",
                    (0.14, 0.12, 0.001),
                    color=(0.12, 0.35, 0.95, 1.0),
                    collision=False,
                )
            ],
        )
        return [table, [hook], [target], [region]]
    if environment == "open-hinged-door":
        frame = Link(
            "door_frame",
            xyz=(0.43, -0.15, 0.0),
            shapes=[
                Shape("box", (0.05, 0.05, 0.34), (0.025, -0.025, 0.17)),
                Shape("box", (0.05, 0.05, 0.34), (0.04, 0.305, 0.17)),
                Shape("box", (0.05, 0.38, 0.04), (0.025, 0.14, 0.34)),
                # The negative-x strike lip captures the extended bolt. A
                # 10 mm throat leaves the closed bolt unloaded but blocks the
                # door as soon as it rotates toward the robot.
                Shape(
                    "box",
                    (0.012, 0.055, 0.06),
                    (-0.025, 0.292, 0.17),
                    color=(0.72, 0.75, 0.8, 1.0),
                ),
            ],
        )
        door = Link(
            "door",
            parent="door_frame",
            joint="door_hinge",
            kind="revolute",
            axis=(0.0, 0.0, 1.0),
            limits=(0.0, 1.35),
            mass=0.8,
            damping=0.35,
            inertia=(0.011633, 0.006428, 0.005248, 0.0, 0.0, 0.0),
            com=(0.0, 0.14, 0.165),
            shapes=[
                Shape(
                    "box",
                    (0.018, 0.28, 0.31),
                    (0.0, 0.14, 0.165),
                    color=(0.18, 0.42, 0.72, 1.0),
                )
            ],
        )
        lever = Link(
            "door_lever",
            parent="door",
            xyz=(-0.02, 0.205, 0.18),
            joint="door_lever",
            kind="revolute",
            axis=(1.0, 0.0, 0.0),
            limits=(0.0, 1.05),
            mass=0.08,
            damping=0.08,
            inertia=(0.00008, 0.00008, 0.00002, 0.0, 0.0, 0.0),
            com=(-0.02, 0.0, -0.035),
            shapes=[
                Shape(
                    "cylinder",
                    (0.014, 0.05),
                    (-0.025, 0.0, 0.0),
                    (0.0, math.pi / 2, 0.0),
                    color=(0.95, 0.48, 0.08, 1.0),
                ),
                Shape(
                    "box",
                    (0.018, 0.018, 0.09),
                    (-0.052, 0.0, -0.04),
                    color=(0.95, 0.48, 0.08, 1.0),
                ),
            ],
        )
        latch = Link(
            "latch_bolt",
            parent="door",
            xyz=(0.0, 0.245, 0.17),
            joint="door_latch",
            kind="prismatic",
            axis=(0.0, -1.0, 0.0),
            limits=(0.0, 0.06),
            mass=0.06,
            damping=0.1,
            inertia=(0.0000261, 0.00000324, 0.0000261, 0.0, 0.0, 0.0),
            mimic=("door_lever", 0.06 / 1.05, 0.0),
            shapes=[
                Shape(
                    "box",
                    (0.018, 0.07, 0.018),
                    (0.0, 0.035, 0.0),
                    color=(0.72, 0.75, 0.8, 1.0),
                )
            ],
        )
        return [table, [frame, door, lever, latch]]
    if environment == "stack-three-cubes":
        return [
            table,
            [free_box("cube_bottom", (0.25, -0.13, 0.023), (0.1, 0.72, 0.2, 1.0))],
            [free_box("cube_middle", (0.35, -0.01, 0.023), (0.15, 0.35, 0.95, 1.0))],
            [free_box("cube_top", (0.27, 0.13, 0.023), (0.95, 0.45, 0.08, 1.0))],
        ]
    if environment == "insert-peg":
        peg = Link(
            "target_peg",
            xyz=(0.27, -0.12, 0.012),
            rpy=(0.0, math.pi / 2, 0.0),
            kind="free",
            mass=0.05,
            inertia=(0.000027, 0.000027, 0.0000036, 0.0, 0.0, 0.0),
            shapes=[
                Shape(
                    "cylinder",
                    (0.012, 0.08),
                    color=(0.95, 0.45, 0.08, 1.0),
                )
            ],
        )
        peg.damping = 0.001
        peg.shapes[0].friction = (1.0, 0.05, 0.01)
        socket_shapes = [
            Shape(
                "box",
                (0.10, 0.10, 0.006),
                (0.0, 0.0, 0.003),
                color=(0.12, 0.35, 0.95, 1.0),
            )
        ]
        for index in range(16):
            angle = 2 * math.pi * index / 16
            socket_shapes.append(
                Shape(
                    "box",
                    (0.020, 0.012, 0.04),
                    (0.023 * math.cos(angle), 0.023 * math.sin(angle), 0.023),
                    (0.0, 0.0, angle + math.pi / 2),
                    color=(0.12, 0.35, 0.95, 1.0),
                )
            )
        socket = Link("target_hole", xyz=(0.40, 0.10, 0.0), shapes=socket_shapes)
        return [table, [peg], [socket]]
    if environment == "retrieve-from-drawer":
        target = free_box(
            "target_object",
            (0.57, 0.07, 0.0735),
            (0.95, 0.45, 0.08, 1.0),
            size=0.035,
            mass=0.03,
        )
        goal = Link(
            "goal_region",
            xyz=(0.30, 0.18, 0.0005),
            shapes=[
                Shape(
                    "box",
                    (0.14, 0.14, 0.001),
                    color=(0.12, 0.35, 0.95, 1.0),
                    collision=False,
                )
            ],
        )
        return [table, drawer_fixture(), [target], [goal]]
    if environment == "store-in-drawer":
        target = free_box(
            "target_object",
            (0.29, -0.15, 0.02),
            (0.95, 0.45, 0.08, 1.0),
            size=0.04,
            mass=0.04,
        )
        return [
            table,
            drawer_fixture(initial=0.15, include_interior=True),
            [target],
        ]
    if environment == "insert-usb":
        connector = Link(
            "usb_connector",
            xyz=(0.28, -0.12, 0.009),
            kind="free",
            mass=0.025,
            inertia=(0.000006, 0.000025, 0.000025, 0.0, 0.0, 0.0),
            shapes=[
                Shape(
                    "box",
                    (0.06, 0.04, 0.018),
                    (-0.03, 0.0, 0.0),
                    color=(0.95, 0.45, 0.08, 1.0),
                ),
                Shape(
                    "box",
                    (0.035, 0.028, 0.004),
                    (0.0175, 0.0, -0.0025),
                    color=(0.72, 0.75, 0.8, 1.0),
                ),
                # The upper-left rail leaves the upper-right quadrant empty.
                # A matching port key fills that quadrant, so the lower plate
                # collides with it after a 180-degree roll.
                Shape(
                    "box",
                    (0.035, 0.006, 0.005),
                    (0.0175, -0.011, 0.002),
                    color=(0.72, 0.75, 0.8, 1.0),
                ),
            ],
        )
        port = Link(
            "usb_port",
            xyz=(0.42, 0.10, 0.12),
            shapes=[
                Shape("box", (0.04, 0.065, 0.16), (0.0, -0.0475, 0.0)),
                Shape("box", (0.04, 0.065, 0.16), (0.0, 0.0475, 0.0)),
                Shape("box", (0.04, 0.03, 0.075), (0.0, 0.0, -0.0425)),
                Shape("box", (0.04, 0.03, 0.075), (0.0, 0.0, 0.0425)),
                Shape(
                    "box",
                    (0.025, 0.010, 0.005),
                    (0.0075, 0.008, 0.002),
                    color=(0.72, 0.75, 0.8, 1.0),
                ),
                Shape(
                    "box",
                    (0.006, 0.03, 0.011),
                    (0.033, 0.0, 0.0),
                    color=(0.72, 0.75, 0.8, 1.0),
                ),
            ],
        )
        return [table, [connector], [port]]
    if environment == "load-clear-test-tubes":
        tube_color = (0.55, 0.88, 0.96, 0.38)
        tubes = [
            [
                Link(
                    f"clear_test_tube_{index}",
                    xyz=(x, y, 0.009),
                    rpy=(0.0, math.pi / 2, 0.0),
                    kind="free",
                    mass=0.015,
                    inertia=(0.0000128, 0.0000128, 0.00000061, 0.0, 0.0, 0.0),
                    shapes=[Shape("cylinder", (0.009, 0.10), color=tube_color)],
                )
            ]
            for index, (x, y) in enumerate(
                ((0.25, -0.18), (0.30, -0.07), (0.24, 0.05), (0.25, 0.16)),
                start=1,
            )
        ]
        for (tube,) in tubes:
            tube.damping = 0.001
            tube.shapes[0].friction = (1.0, 0.05, 0.01)
        rack = Link(
            "blue_rack",
            xyz=(0.39, 0.10, 0.0),
            shapes=[
                Shape(
                    "box",
                    (0.16, 0.18, 0.01),
                    (0.0, 0.0, 0.005),
                    color=(0.10, 0.30, 0.85, 1.0),
                )
            ],
        )
        slots = []
        for row, y in (("front", -0.04), ("back", 0.04)):
            for column, x in (("left", -0.04), ("right", 0.04)):
                segments = []
                for index in range(12):
                    angle = 2 * math.pi * index / 12
                    segments.append(
                        Shape(
                            "box",
                            (0.012, 0.008, 0.025),
                            (0.016 * math.cos(angle), 0.016 * math.sin(angle), 0.0225),
                            (0.0, 0.0, angle + math.pi / 2),
                            color=(0.10, 0.30, 0.85, 1.0),
                        )
                    )
                slots.append(
                    Link(
                        f"rack_slot_{row}_{column}",
                        parent="blue_rack",
                        xyz=(x, y, 0.0),
                        mass=1e-6,
                        inertia=(1e-9, 1e-9, 1e-9, 0.0, 0.0, 0.0),
                        shapes=segments,
                    )
                )
        return [table, *tubes, [rack, *slots]]
    if environment == "drawer":
        # Keep the closed front clear of the robot's starting hand. The
        # handle travels from x=.503 to .283 m as the drawer opens.
        return [table, drawer_fixture()]
    if environment != "bottle_cap":
        raise ValueError("unknown reference environment")
    # A fixture holds the bottle so a single arm can unscrew it. Each engine
    # loads the same free-cap/thread assembly from the packaged native assets.
    bottle = Link(
        "bottle",
        # Leave the central jog region clear; the cap remains on the tabletop
        # within the robot's reach. Contacts use the normal engine solver.
        xyz=(0.34, -0.16, 0.085),
        shapes=[
            Shape("cylinder", (0.033, 0.17), color=(0.1, 0.6, 0.35, 1.0)),
        ],
    )
    return [table, [bottle]]


def urdf(links: list[Link], name: str) -> str:
    root = ET.Element("robot", name=name)
    for link in links:
        node = ET.SubElement(root, "link", name=link.name)
        # Standalone static scenery has no dynamic mass properties. Keep
        # inertials for every articulation link, including its fixed frames.
        if len(links) > 1 or link.kind != "fixed":
            inertial = ET.SubElement(node, "inertial")
            ET.SubElement(inertial, "origin", xyz=numbers(link.com), rpy="0 0 0")
            ET.SubElement(inertial, "mass", value=str(link.mass))
            inertia = max(link.mass * 0.003, 1e-6)
            tensor = link.inertia or (inertia, inertia, inertia, 0, 0, 0)
            ET.SubElement(
                inertial,
                "inertia",
                **dict(
                    zip(("ixx", "iyy", "izz", "ixy", "ixz", "iyz"), map(str, tensor))
                ),
            )
        for i, shape in enumerate(link.shapes):
            for kind in ("visual", "collision"):
                if not getattr(shape, kind):
                    continue
                geom = ET.SubElement(node, kind)
                ET.SubElement(
                    geom, "origin", xyz=numbers(shape.xyz), rpy=numbers(shape.rpy)
                )
                geometry = ET.SubElement(geom, "geometry")
                attrs = (
                    {"filename": shape.mesh, "scale": numbers(shape.size)}
                    if shape.mesh
                    else (
                        {"size": numbers(shape.size)}
                        if shape.kind == "box"
                        else {"radius": str(shape.size[0])}
                    )
                )
                if shape.kind == "cylinder":
                    attrs["length"] = str(shape.size[1])
                ET.SubElement(geometry, shape.kind, **attrs)
                if kind == "visual":
                    material = ET.SubElement(geom, "material", name=f"{link.name}_{i}")
                    ET.SubElement(material, "color", rgba=numbers(shape.color))
                    if shape.texture:
                        ET.SubElement(material, "texture", filename=shape.texture)
        if link.parent is not None:
            joint = ET.SubElement(
                root, "joint", name=link.joint or f"{link.name}_fixed", type=link.kind
            )
            ET.SubElement(joint, "parent", link=link.parent)
            ET.SubElement(joint, "child", link=link.name)
            ET.SubElement(joint, "origin", xyz=numbers(link.xyz), rpy=numbers(link.rpy))
            if link.joint:
                ET.SubElement(joint, "axis", xyz=numbers(link.axis))
                if link.damping:
                    ET.SubElement(
                        joint, "dynamics", damping=str(link.damping), friction="0"
                    )
                ET.SubElement(
                    joint,
                    "limit",
                    lower=str(link.limits[0]),
                    upper=str(link.limits[1]),
                    effort="50",
                    velocity="3",
                )
                if link.mimic:
                    target, multiplier, offset = link.mimic
                    ET.SubElement(
                        joint,
                        "mimic",
                        joint=target,
                        multiplier=str(multiplier),
                        offset=str(offset),
                    )
    return ET.tostring(root, encoding="unicode")


def _robot_instance(links: list[Link], prefix: str, placement: dict) -> list[Link]:
    """Namespace one robot while retaining its manufacturer-local geometry."""

    result = []
    for link in links:
        root = link.parent is None
        result.append(
            replace(
                link,
                name=prefix + link.name,
                parent=None if root else prefix + link.parent,
                xyz=tuple(placement["xyz"]) if root else link.xyz,
                rpy=tuple(placement["rpy"]) if root else link.rpy,
                joint=None if link.joint is None else prefix + link.joint,
                mimic=(
                    None
                    if link.mimic is None
                    else (prefix + link.mimic[0], *link.mimic[1:])
                ),
            )
        )
    return result


def mjcf(p: Profile, config: dict) -> str:
    from .description import description

    robot = description(p.name)
    import mujoco

    # MuJoCo's maintained URDF importer owns geometry and full inertias. Keep
    # fixed frames for the same camera/TCP names used by the other backends.
    props = objects(config["environment"], robot=config["robot"])
    prefixes = {
        part: "" if len(config["parts"]) == 1 else f"{part}__"
        for part in config["parts"]
    }
    robot_groups = {
        part: _robot_instance(robot.native_links(), prefixes[part], placement)
        for part, placement in config["parts"].items()
    }
    groups = [*robot_groups.values(), *props]
    masters = {part: f"{prefixes[part]}{robot.hand_names[0]}" for part in robot_groups}
    hand_drives = {
        part: [masters[part]]
        + [link.joint for link in links if link.mimic == (masters[part], 1.0, 0.0)]
        for part, links in robot_groups.items()
    }
    scene_links = [Link("scene_root")]
    for group in groups:
        scene_links.extend(
            [replace(group[0], parent="scene_root", kind="fixed"), *group[1:]]
        )
    source = ET.fromstring(urdf(scene_links, p.name))
    ET.SubElement(
        ET.SubElement(source, "mujoco"),
        "compiler",
        discardvisual="false",
        fusestatic="false",
        strippath="false",
    )
    native = mujoco.MjSpec.from_string(ET.tostring(source, encoding="unicode"))
    root = ET.fromstring(native.to_xml())
    # MuJoCo's documented manipulation configuration reduces soft-constraint
    # friction creep for all contacts, without changing material friction.
    ET.SubElement(
        root,
        "option",
        timestep=str(config["timestep"]),
        integrator="implicitfast",
        cone="elliptic",
        impratio="10",
        solver="Newton",
        tolerance="1e-10",
    )
    visual = ET.SubElement(root, "visual")
    samples, shadow_size = {
        "fast": (0, 1024),
        "standard": (4, 4096),
        "high": (8, 8192),
    }[config.get("render_quality", "standard")]
    ET.SubElement(
        visual, "quality", offsamples=str(samples), shadowsize=str(shadow_size)
    )
    ET.SubElement(
        visual,
        "global",
        offwidth=str(max(c["stream"]["width"] for c in config["cameras"].values())),
        offheight=str(max(c["stream"]["height"] for c in config["cameras"].values())),
    )
    world = root.find("worldbody")
    ET.SubElement(
        visual,
        "headlight",
        ambient=".18 .20 .24",
        diffuse=".25 .25 .25",
        specular=".1 .1 .1",
    )
    ET.SubElement(
        world,
        "light",
        pos="-0.5 -1 2",
        dir=".3 .4 -1",
        diffuse=".9 .85 .78",
        specular=".5 .5 .5",
    )
    ET.SubElement(
        world,
        "light",
        pos="1 1 1.5",
        dir="-.4 -.4 -1",
        diffuse=".3 .36 .45",
        castshadow="false",
    )
    asset = root.find("asset")
    ET.SubElement(
        asset,
        "texture",
        name="studio_sky",
        type="skybox",
        builtin="gradient",
        rgb1=".35 .42 .52",
        rgb2=".82 .85 .89",
        width="256",
        height="1536",
    )
    ET.SubElement(
        asset,
        "texture",
        name="table_wood",
        type="2d",
        file=props[0][0].shapes[0].texture,
    )
    ET.SubElement(
        asset,
        "material",
        name="table_finish",
        texture="table_wood",
        texrepeat="1 1",
        texuniform="true",
        specular=".25",
        shininess=".2",
    )
    for material in asset.findall("material"):
        if material.get("name") != "table_finish":
            material.set("specular", ".45")
            material.set("shininess", ".4")
    # Remove the URDF-only container so free props are native world children.
    container = world.find("body[@name='scene_root']")
    world.extend(container.findall("body"))
    world.remove(container)
    bodies = {body.get("name"): body for body in world.iter("body")}
    for links in robot_groups.values():
        for link in links:
            bodies[link.name].set("gravcomp", "1")
    for group in props:
        if group[0].kind == "free":
            ET.SubElement(bodies[group[0].name], "freejoint")
        for link in group:
            if not link.joint or not link.stiffness:
                continue
            joint = bodies[link.name].find("joint")
            joint.set("stiffness", str(link.stiffness))
            joint.set("springref", str(link.spring_reference))
    # Collision visuals are hidden by the camera renderer; actual CAD remains.
    for geom in world.iter("geom"):
        geom.set("group", "2" if geom.get("contype") == "0" else "3")
    for group in props:
        for link in group:
            if not any(shape.friction is not None for shape in link.shapes):
                continue
            collision_shapes = [shape for shape in link.shapes if shape.collision]
            collision_geometries = [
                geom
                for geom in bodies[link.name].findall("geom")
                if geom.get("group") == "3"
            ]
            if len(collision_shapes) != len(collision_geometries):
                raise RuntimeError(
                    f"Native collision geometry count changed for {link.name!r}"
                )
            for shape, geometry in zip(
                collision_shapes, collision_geometries, strict=True
            ):
                if shape.friction is not None:
                    geometry.set("friction", numbers(shape.friction))
    # Reuse Menagerie's finger-pad contact response on the manufacturer's
    # collision meshes. Default 20 ms contacts let the stiff linear hand
    # penetrate a held cube and oscillate; no material friction is increased.
    fingers = {
        "so101": ("gripper_link", "moving_jaw_so101_v1_link"),
        "yam": ("tip_left", "tip_right"),
        "xarm7": ("left_finger", "right_finger"),
    }[p.name]
    for part in robot_groups:
        for name in fingers:
            for geom in bodies[f"{prefixes[part]}{name}"].findall("geom"):
                if geom.get("group") == "3":
                    geom.set("solref", "0.004 1")
                    geom.set("solimp", "0.95 0.99 0.001")
                    geom.set("priority", "1")
    for geom in bodies["table"].findall("geom"):
        if geom.get("group") == "2":
            geom.set("material", "table_finish")
            geom.set("rgba", "1 1 1 1")
    for part, links in robot_groups.items():
        drives = hand_drives[part]
        master = robot.hand_names[0]
        for native, link in zip(robot.native_links(), links, strict=True):
            if not native.joint:
                continue
            joint = bodies[link.name].find("joint")
            joint.set(
                "armature",
                str(
                    robot.armature(master) / len(drives)
                    if link.joint in drives
                    else robot.armature(native.joint)
                ),
            )
            if p.name == "xarm7" and native.joint in robot.hand_names:
                # Menagerie's follower/spring-link classes inherit the .1
                # joint armature. Dropping it leaves the soft four-bar closure
                # supported only by tiny CAD link inertias, so a loaded hand
                # folds instead of retaining its physical jaw opening.
                if link.joint not in drives:
                    joint.set("armature", "0.1")
                if not native.joint.endswith("inner_knuckle_joint"):
                    joint.set("solreflimit", "0.005 1")
    contact = ET.SubElement(root, "contact")
    for group in groups:
        for link in group:
            if link.parent:
                ET.SubElement(contact, "exclude", body1=link.parent, body2=link.name)
    for part in robot_groups:
        for first, second in robot.exclusions:
            ET.SubElement(
                contact,
                "exclude",
                body1=f"{prefixes[part]}{first}",
                body2=f"{prefixes[part]}{second}",
            )
    equality = ET.SubElement(root, "equality")
    all_hand_drives = {name for names in hand_drives.values() for name in names}
    for link in scene_links:
        if link.mimic:
            master, multiplier, offset = link.mimic
            coupling = ET.SubElement(
                equality,
                "joint",
                joint1=link.joint,
                joint2=master,
                polycoef=numbers((offset, multiplier, 0, 0, 0)),
            )
            if link.joint in all_hand_drives:
                # Match Menagerie's mechanical gripper coupling time constant.
                coupling.set("solref", "0.005 1")
    for part in robot_groups:
        for first, anchor, second, _other in robot.closures():
            ET.SubElement(
                equality,
                "connect",
                body1=f"{prefixes[part]}{first}",
                body2=f"{prefixes[part]}{second}",
                anchor=numbers(anchor),
                solref="0.005 1",
            )
    # Menagerie's xArm/Robotiq pattern distributes one motor's force through a
    # fixed tendon. The mechanical equality need not transfer the entire load
    # from one jaw to the other, which otherwise shifts the pinch midpoint.
    # Keep the same total gain, force budget and reflected rotor inertia.
    tendons = ET.SubElement(root, "tendon")
    for part, drives in hand_drives.items():
        tendon = ET.SubElement(tendons, "fixed", name=f"{prefixes[part]}hand_motor")
        for name in drives:
            ET.SubElement(tendon, "joint", joint=name, coef=str(1 / len(drives)))
    actuators = ET.SubElement(root, "actuator")
    for part in robot_groups:
        ET.SubElement(
            bodies[f"{prefixes[part]}tcp"],
            "site",
            name=f"{prefixes[part]}tcp_site",
            size=".001",
        )
        for name, limits in zip(p.names[:-1], p.limits[:-1], strict=True):
            kp, kd, effort = robot.servo(name)
            ET.SubElement(
                actuators,
                "position",
                name=f"{prefixes[part]}{name}",
                joint=f"{prefixes[part]}{name}",
                kp=str(kp),
                kv=str(kd),
                ctrlrange=numbers(limits),
                forcerange=numbers((-effort, effort)),
            )
        for name, limits in zip(
            robot.hand_names[:1], robot.hand_limits[:1], strict=True
        ):
            kp, kd, effort = robot.servo(name)
            ET.SubElement(
                actuators,
                "position",
                name=f"{prefixes[part]}{name}",
                tendon=f"{prefixes[part]}hand_motor",
                kp=str(kp),
                kv=str(kd),
                ctrlrange=numbers(limits),
                forcerange=numbers((-effort, effort)),
            )
    for name, row in config["cameras"].items():
        t = np.asarray(row["transform"])
        gl_rotation = t[:3, :3] @ np.diag([1, -1, -1])
        intr, stream = row["intrinsics"], row["stream"]
        # MuJoCo shifts the projection frustum, so its offsets have the
        # opposite sign to principal points in the returned optical image.
        # Optical pixel centers are integer coordinates; the first OpenGL
        # raster sample is half a pixel from the edge of the viewport.
        ET.SubElement(
            bodies[f"{prefixes[row['mount']['part']]}tcp"]
            if row["mount"]["kind"] == "wrist"
            else world,
            "camera",
            name=name,
            pos=numbers(t[:3, 3]),
            quat=numbers(quaternion(gl_rotation)),
            resolution=f"{stream['width']} {stream['height']}",
            sensorsize="1 1",
            focalpixel=f"{intr['fx']} {intr['fy']}",
            principalpixel=numbers(
                (
                    (stream["width"] - 1) / 2 - intr["cx"],
                    (stream["height"] - 1) / 2 - intr["cy"],
                )
            ),
        )
    if config["environment"] == "bottle_cap":
        from .thread import append_mjcf

        append_mjcf(root)
    return ET.tostring(root, encoding="unicode")
