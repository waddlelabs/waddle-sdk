"""One-for-one SDK coverage for the retired Waddle body-bounds implementation.

The historical Pinocchio loader and teleoperation clamp are deliberately not
recreated. Driver adapters supply deterministic conservative spheres; the SDK
checks those bodies and the TCP against the owner workspace and rejects a whole
command. These tests preserve every historical function name while exercising
that replacement or an explicit hard-cut disposition.
"""

from __future__ import annotations

import time
from importlib import resources
from pathlib import Path
from typing import Callable, Sequence

import numpy as np
import pytest

from waddle_sdk.robots import base


class _Driver:
    kind = "sim"

    def __init__(self, current: Sequence[float] = (0.0, 0.0)) -> None:
        self.current = np.asarray(current, dtype=float)
        self.writes: list[np.ndarray] = []
        self.holds = 0
        self._estopped = False

    @property
    def estopped(self) -> bool:
        return self._estopped

    def read(self) -> tuple[np.ndarray, np.ndarray]:
        return self.current.copy(), np.zeros_like(self.current)

    def write(self, target: Sequence[float]) -> None:
        value = np.asarray(target, dtype=float)
        self.writes.append(value.copy())
        self.current = value.copy()

    def hold(self) -> None:
        self.holds += 1

    def estop(self) -> None:
        self._estopped = True

    def re_enable(self) -> None:
        self._estopped = False

    def step(self, dt: float) -> None:
        return None

    def home(self, values: Sequence[float]) -> bool:
        self.current = np.asarray(values, dtype=float)
        return True

    def close(self) -> None:
        return None


def _fk(q: Sequence[float]) -> tuple[np.ndarray, np.ndarray]:
    return np.array([float(q[0]), float(q[1]), 0.5]), np.eye(3)


def _tool_spheres(q: Sequence[float]) -> tuple[base.CollisionSphere, ...]:
    return (
        base.CollisionSphere("arm", (float(q[0]) - 0.20, float(q[1]), 0.5), 0.05),
        base.CollisionSphere("gripper", (float(q[0]), float(q[1]), 0.5), 0.05),
    )


def _arm(
    *,
    driver: _Driver | None = None,
    workspace: tuple[tuple[float, float, float], tuple[float, float, float]]
    | None = ((0.0, -1.0, 0.0), (1.0, 1.0, 1.0)),
    spheres: Callable[[Sequence[float]], Sequence[base.CollisionSphere]] | None = (
        _tool_spheres
    ),
    self_collision: bool = False,
    margin_m: float = 0.0,
    ignore_pairs: Sequence[Sequence[str]] = (),
    static_keepouts: Sequence[dict[str, object]] = (),
    collision_frame: str = "site",
) -> base.Arm:
    return base.Arm(
        part="arm",
        driver=driver or _Driver(),
        joint_names=("x", "y"),
        joint_limits=((-2.0, 2.0), (-2.0, 2.0)),
        step_caps=(10.0, 10.0),
        workspace=workspace,
        fk=_fk,
        collision_spheres=spheres,
        collision_frame=collision_frame,
        static_keepouts=static_keepouts,
        self_collision_enabled=self_collision,
        self_collision_margin_m=margin_m,
        self_collision_ignore_pairs=ignore_pairs,
        report=lambda _line: None,
    )


def test_load_derives_spheres_and_gripper_set() -> None:
    snapshot = _arm(workspace=None).collision_snapshot((0.5, 0.0))
    assert [sphere.name for sphere in snapshot] == ["arm/arm", "arm/gripper"]
    assert all(isinstance(sphere, base.CollisionSphere) for sphere in snapshot)


def test_margin_inflates_radii() -> None:
    def separated(_q: Sequence[float]) -> tuple[base.CollisionSphere, ...]:
        return (
            base.CollisionSphere("one", (0.0, 0.0, 0.0), 0.10),
            base.CollisionSphere("two", (0.21, 0.0, 0.0), 0.10),
        )

    assert _arm(workspace=None, spheres=separated, self_collision=True).command((0.0, 0.0))
    widened = _arm(
        workspace=None,
        spheres=separated,
        self_collision=True,
        margin_m=0.02,
    )
    assert not widened.command((0.0, 0.0))


def test_joints_reject_when_min_z_above_gripper() -> None:
    arm = _arm(workspace=((0.0, -1.0, 0.48), (1.0, 1.0, 1.0)))
    assert not arm.command((0.5, 0.0))


def test_joints_accept_in_big_box() -> None:
    arm = _arm(workspace=((-2.0, -2.0, -2.0), (2.0, 2.0, 2.0)))
    assert arm.command((0.5, 0.0))


def test_pose_reject_gripper_below_floor() -> None:
    def low(q: Sequence[float]) -> tuple[base.CollisionSphere, ...]:
        return (base.CollisionSphere("gripper", (float(q[0]), 0.0, 0.04), 0.05),)

    assert not _arm(spheres=low).command((0.5, 0.0))


def test_pose_accept_high_in_big_box() -> None:
    assert _arm(workspace=((-2.0, -2.0, -2.0), (2.0, 2.0, 2.0))).command(
        (0.5, 0.0)
    )


def test_gripper_sweep_covered_by_inflated_endpoints() -> None:
    queried: list[tuple[float, ...]] = []

    def conservative(q: Sequence[float]) -> tuple[base.CollisionSphere, ...]:
        queried.append(tuple(float(value) for value in q))
        return (base.CollisionSphere("swept_gripper", (0.5, 0.0, 0.5), 0.20),)

    arm = _arm(spheres=conservative)
    assert arm.command((0.37, 0.0))
    assert queried == [(0.37, 0.0)]


def test_clamp_lifts_gripper_to_floor_minimally() -> None:
    driver = _Driver()
    arm = _arm(driver=driver, workspace=((0.0, -1.0, 0.48), (1.0, 1.0, 1.0)))
    assert not arm.command((0.5, 0.0))
    assert driver.writes == [] and driver.holds == 1


def test_clamp_none_when_in_bounds() -> None:
    driver = _Driver()
    arm = _arm(driver=driver, workspace=((-2.0, -2.0, -2.0), (2.0, 2.0, 2.0)))
    assert arm.command((0.5, 0.0))
    np.testing.assert_array_equal(driver.writes[0], [0.5, 0.0])


def test_clamp_max_face_pushes_down() -> None:
    def high(_q: Sequence[float]) -> tuple[base.CollisionSphere, ...]:
        return (base.CollisionSphere("gripper", (0.5, 0.0, 0.98), 0.05),)

    driver = _Driver()
    arm = _arm(driver=driver, spheres=high)
    assert not arm.command((0.5, 0.0))
    assert driver.writes == []


def test_check_cost_under_budget() -> None:
    arm = _arm(workspace=((-2.0, -2.0, -2.0), (2.0, 2.0, 2.0)))
    started = time.perf_counter()
    for _ in range(250):
        assert arm.check(np.array([0.5, 0.0]), np.array([0.0, 0.0])) is None
    assert (time.perf_counter() - started) / 250 < 0.001


def test_base_frame_guard_rejects_non_root_base() -> None:
    keepout = {
        "id": "fixture",
        "kind": "sphere",
        "frame": "other",
        "center": (0.0, 0.0, 0.0),
        "radius_m": 0.1,
    }
    with pytest.raises(ValueError, match="collision frame"):
        _arm(workspace=None, static_keepouts=(keepout,), collision_frame="site")


def test_fixed_base_link_excluded() -> None:
    names = {sphere.name for sphere in _arm(workspace=None).collision_snapshot((0.5, 0.0))}
    assert "arm/fixed_base" not in names


def test_fixed_base_no_longer_perpetually_violates() -> None:
    assert _arm().command((0.5, 0.0))


def test_spheres_conservatively_contain_geom_aabbs() -> None:
    half_extents = np.array([0.1, 0.2, 0.3])
    sphere = base.CollisionSphere("box", (0.0, 0.0, 0.0), np.linalg.norm(half_extents))
    corners = np.array(
        [
            (x, y, z)
            for x in (-half_extents[0], half_extents[0])
            for y in (-half_extents[1], half_extents[1])
            for z in (-half_extents[2], half_extents[2])
        ]
    )
    assert np.all(np.linalg.norm(corners, axis=1) <= sphere.radius_m)


def test_violations_list_and_ignore_faces() -> None:
    arm = _arm(workspace=((0.0, -1.0, 0.48), (1.0, 1.0, 1.0)))
    assert "workspace_ignore_faces" not in arm.__dataclass_fields__
    assert not arm.command((0.5, 0.0))


def test_violations_dedup_keeps_max_overshoot() -> None:
    def two_bad(_q: Sequence[float]) -> tuple[base.CollisionSphere, ...]:
        return (
            base.CollisionSphere("slightly_low", (0.5, 0.0, 0.02), 0.05),
            base.CollisionSphere("very_low", (0.5, 0.0, -0.20), 0.05),
        )

    driver = _Driver()
    arm = _arm(driver=driver, spheres=two_bad)
    assert not arm.command((0.5, 0.0))
    assert driver.writes == [] and driver.holds == 1


def test_urdf_relative_meshes_resolve_without_package_dirs() -> None:
    model = resources.files("waddle_sdk.robots").joinpath("yam_data", "yam.urdf")
    assert model.is_file() and "grasp_link" in model.read_text(encoding="utf-8")


def test_tcp_bounds_fallback_loads_without_meshes() -> None:
    arm = _arm(spheres=None)
    assert arm.collision_snapshot((0.5, 0.0)) == ()
    assert arm.command((0.5, 0.0))


def test_tcp_fallback_joints_check_tracks_the_tcp_point() -> None:
    driver = _Driver()
    arm = _arm(driver=driver, spheres=None)
    assert not arm.command((-0.1, 0.0))
    assert arm.command((0.2, 0.0))


def test_tcp_fallback_pose_check_and_clamp_are_single_point() -> None:
    driver = _Driver()
    arm = _arm(driver=driver, spheres=None)
    assert not arm.command((-0.1, 0.0))
    assert driver.writes == [] and driver.holds == 1


def test_tcp_fallback_rejects_unknown_tcp_frame() -> None:
    with pytest.raises(ValueError, match="declared no `fk`"):
        base.Arm(
            part="arm",
            driver=_Driver(),
            joint_names=("x", "y"),
            joint_limits=((-2.0, 2.0), (-2.0, 2.0)),
            step_caps=(10.0, 10.0),
            workspace=((0.0, 0.0, 0.0), (1.0, 1.0, 1.0)),
        )


def test_body_wb_split_gripper_vs_arm() -> None:
    def arm_outside(_q: Sequence[float]) -> tuple[base.CollisionSphere, ...]:
        return (
            base.CollisionSphere("arm_link", (-0.1, 0.0, 0.5), 0.05),
            base.CollisionSphere("gripper", (0.5, 0.0, 0.5), 0.05),
        )

    assert not _arm(spheres=arm_outside).command((0.5, 0.0))


def test_joints_inward_recovery() -> None:
    driver = _Driver(current=(-0.5, 0.0))
    arm = _arm(driver=driver)
    assert arm.command((0.5, 0.0))


def test_recovery_rejects_new_face_violation() -> None:
    driver = _Driver(current=(-0.5, 0.0))
    arm = _arm(driver=driver)
    assert not arm.command((1.5, 0.0))
    assert driver.writes == []


def test_ee_only_skips_arm_link_against_tcp_envelope() -> None:
    def arm_outside(_q: Sequence[float]) -> tuple[base.CollisionSphere, ...]:
        return (base.CollisionSphere("arm_link", (-0.1, 0.0, 0.5), 0.05),)

    arm = _arm(spheres=arm_outside)
    assert "workspace_ee_only" not in arm.__dataclass_fields__
    assert not arm.command((0.5, 0.0))


def test_ee_only_still_blocks_a_genuine_tcp_envelope_violation() -> None:
    arm = _arm(spheres=None)
    assert not arm.command((-0.1, 0.0))


def test_ee_only_and_body_wb_split_are_independent_knobs() -> None:
    fields = _arm().__dataclass_fields__
    assert "workspace_ee_only" not in fields
    assert "body_workspace" not in fields
    assert not _arm().command((0.01, 0.0))
