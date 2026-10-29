"""Finite reference assemblies for authored two-arm scenes.

Only geometry and initial placement live here. Task instructions and scoring
belong to consumers of the normal simulation observation contracts.
"""

import math

import numpy as np

from .model import numbers

FIXTURES = {
    "handover": "handover-block",
    "hold_container": "hold-container-place",
    "peg_socket": "two-arm-peg-insertion",
    "loaded_tray": "loaded-tray-transport",
}


def place_fixture(world, kind, seed=0):
    """Place existing reference assets in the shared YAM work area.

    A seed changes only bounded XY placement. Assembled bodies and tray contents
    move together, retaining their original geometry, mass and native contacts.
    Seed zero is canonical. Robot placement and camera optics are untouched.
    """
    placements = {
        "handover": {
            "handover_block": (0.28, -0.20),
            "goal_region": (0.34, 0.20),
        },
        "hold_container": {
            "movable_container": (0.30, -0.10),
            "target_object": (0.30, 0.20),
        },
        "peg_socket": {
            "receiving_part": (0.32, -0.12),
            "target_peg": (0.30, 0.18),
        },
        "loaded_tray": {
            "loaded_tray": (0.27, 0.0),
            "tray_content_1": (0.215, -0.06),
            "tray_content_2": (0.325, 0.06),
            "goal_region": (0.43, 0.0),
        },
    }[kind]
    rng = np.random.default_rng(seed)
    offset = rng.uniform(-0.012, 0.012, size=2) if seed else np.zeros(2)
    for name, xy in placements.items():
        body = world.find(f"body[@name='{name}']")
        if body is None:
            raise ValueError(f"playground.fixture: missing reference body {name}")
        xyz = [float(v) for v in body.get("pos", "0 0 0").split()]
        xyz[:2] = np.asarray(xy) + offset
        if not all(math.isfinite(v) for v in xyz):
            raise ValueError("playground.fixture: nonfinite reference placement")
        body.set("pos", numbers(xyz))
