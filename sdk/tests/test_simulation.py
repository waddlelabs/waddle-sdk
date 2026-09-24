"""Physics/SDK conformance, using real engines when their extras are installed."""

from __future__ import annotations

import hashlib
import importlib
import json
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from importlib.resources import files
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml
from waddle_sdk import load_site
from waddle_sdk.robots import yam
from waddle_sdk.simulation import SimulationAdministration
from waddle_sdk.simulators import reference_model_sources
from waddle_sdk.simulators.adapters import World
from waddle_sdk.simulators.description import (
    collision_bounds,
    description,
    mesh_triangles,
)
from waddle_sdk.simulators.scene import (
    BACKENDS,
    DUAL_ARM_TASK_ENVIRONMENTS,
    ENVIRONMENTS,
    REFERENCE_ENVIRONMENTS,
    RENDER_QUALITIES,
    ROBOTS,
    TASK_ENVIRONMENTS,
    depth_z16,
    load_scene,
    make_site,
    profile,
    quaternion,
    rotation,
    transform,
)

ENVIRONMENT_BACKENDS = tuple(
    (backend, environment)
    for environment in REFERENCE_ENVIRONMENTS
    for backend in BACKENDS
) + tuple(
    ("mujoco", environment)
    for environment in TASK_ENVIRONMENTS
    if environment not in DUAL_ARM_TASK_ENVIRONMENTS
)

DUAL_ARM_MEDIUM_TASK_ENVIRONMENTS = (
    "handover-block",
    "stabilize-open-drawer",
    "hold-container-place",
    "stabilize-remove-lid",
)
DUAL_ARM_HARD_TASK_ENVIRONMENTS = (
    "oriented-tool-handover",
    "two-arm-peg-insertion",
    "joint-lift",
    "loaded-tray-transport",
    "uncap-return-test-tube",
    "retrieve-bottle-clutter",
)


def test_every_task_environment_has_native_mechanics_acceptance():
    single_arm = {
        environment
        for backend, environment in ENVIRONMENT_BACKENDS
        if backend == "mujoco" and environment in TASK_ENVIRONMENTS
    }
    dual_arm = {
        "split_workspace_sorting",
        *DUAL_ARM_MEDIUM_TASK_ENVIRONMENTS,
        *DUAL_ARM_HARD_TASK_ENVIRONMENTS,
    }
    assert single_arm == set(TASK_ENVIRONMENTS) - set(DUAL_ARM_TASK_ENVIRONMENTS)
    assert dual_arm == set(DUAL_ARM_TASK_ENVIRONMENTS)


def documents(
    root: Path,
    backend="mujoco",
    robot="yam",
    environment="two_cubes",
    *,
    worker_python=None,
    arms=1,
):
    site, sim = make_site(
        "physics-test",
        backend=backend,
        robot=robot,
        environment=environment,
        width=192,
        height=144,
        worker_python=worker_python,
        arms=arms,
    )
    (root / "simulation.json").write_text(json.dumps(sim))
    (root / "site.yaml").write_text(yaml.safe_dump(site))
    return site, sim


@pytest.mark.parametrize("robot", ROBOTS)
@pytest.mark.parametrize(("backend", "environment"), ENVIRONMENT_BACKENDS)
def test_all_reference_declarations_validate_without_opening(
    tmp_path, backend, robot, environment
):
    site, sim = documents(tmp_path, backend, robot, environment)
    loaded = load_site(tmp_path / "site.yaml")
    assert loaded.id == "physics-test"
    assert load_scene(tmp_path, "simulation.json")[1] == sim
    expected_scene_revision = {
        "candy-bin-transfer": "1.2.0",
        "pick_lift": "1.1.0",
        "chocolate-packing": "1.0.1",
    }.get(environment, "1.0.0")
    assert sim["scene_revision"] == expected_scene_revision
    assert sim["asset_revision"] == (
        "1.2.0" if environment == "candy-bin-transfer" else "1.0.0"
    )
    assert sim["embodiment_revision"] == "1.0.0"
    assert set(site["cameras"]) == {"scene", "wrist"}
    assert site["parts"]["arm"]["gripper"]["open_m"] == profile(robot).opening
    assembly = loaded._assembly(None)
    action_space = assembly.rig.robot().action_space.parts["arm"]
    assert action_space.rate_hz == site["parts"]["arm"]["options"]["rate_hz"]
    assert assembly.rig.rate_hz == action_space.rate_hz


@pytest.mark.parametrize("robot", ROBOTS)
def test_dual_task_declarations_have_two_parts_and_three_cameras(tmp_path, robot):
    site, sim = documents(
        tmp_path,
        "mujoco",
        robot,
        "split_workspace_sorting",
        arms=2,
    )
    loaded = load_site(tmp_path / "site.yaml")
    assert loaded.id == "physics-test"
    assert set(site["parts"]) == {"left", "right"}
    assert set(site["cameras"]) == {"scene", "left_wrist", "right_wrist"}
    assert set(sim["parts"]) == {"left", "right"}
    assert sim["environment"] == "split_workspace_sorting"


def test_dual_task_environment_requires_two_arms():
    with pytest.raises(ValueError, match="require arms=2"):
        make_site(
            "invalid-dual-task",
            backend="mujoco",
            robot="so101",
            environment="split_workspace_sorting",
        )


def test_yam_pick_lift_near_base_pose_distribution_is_default(tmp_path, monkeypatch):
    pytest.importorskip("mujoco")
    monkeypatch.setenv("MUJOCO_GL", "egl")
    from waddle_sdk.simulators.mujoco import Engine

    _, config = documents(tmp_path, "mujoco", "yam", "pick_lift")
    assert "pose_profile" not in config
    (tmp_path / "simulation.json").write_text(json.dumps(config))
    assert load_scene(tmp_path, "simulation.json")[1] == config
    engine = Engine(config, tmp_path)
    try:
        positions = []
        for seed in range(1, 65):
            assert engine.evaluation_reset(seed=seed) is True
            positions.append(engine.data.body("target_cube").xpos[:2].copy())
        xy = np.asarray(positions)
        assert np.all((0.25 <= xy[:, 0]) & (xy[:, 0] <= 0.33))
        assert np.all((-0.06 <= xy[:, 1]) & (xy[:, 1] <= 0.06))
        assert np.ptp(xy[:, 0]) > 0.06
        assert np.ptp(xy[:, 1]) > 0.09
        assert engine.evaluation_reset(seed=0) is True
        np.testing.assert_allclose(engine.data.body("target_cube").xpos[:2], (0.32, 0))
    finally:
        engine.close()


def test_yam_pick_lift_pose_profile_rejects_far_or_foreign_ranges(tmp_path):
    _, config = documents(tmp_path, "mujoco", "yam", "pick_lift")
    config["pose_profile"] = {
        "x_offset_m": -0.03,
        "x_half_range_m": 0.10,
        "y_half_range_m": 0.06,
    }
    (tmp_path / "simulation.json").write_text(json.dumps(config))
    with pytest.raises(ValueError, match="near-base XY range"):
        load_scene(tmp_path, "simulation.json")
    config["pose_profile"]["x_half_range_m"] = 0.04
    config["robot"] = "so101"
    (tmp_path / "simulation.json").write_text(json.dumps(config))
    with pytest.raises(ValueError, match="YAM pick_lift pose profile"):
        load_scene(tmp_path, "simulation.json")


@pytest.mark.parametrize("backend", ("isaac", "sapien"))
@pytest.mark.parametrize("environment", TASK_ENVIRONMENTS)
def test_development_task_environments_reject_unvalidated_backends(
    backend, environment
):
    with pytest.raises(ValueError, match="task environments.*MuJoCo"):
        make_site(
            "unvalidated-task-backend",
            backend=backend,
            robot="so101",
            environment=environment,
        )


@pytest.mark.parametrize(
    ("robot", "depth", "wrist_model"),
    [
        ("so101", False, "rgb"),
        ("yam", True, "realsense_d405"),
        ("xarm7", True, "realsense_d435"),
    ],
)
def test_reference_camera_capabilities_match_each_embodiment(
    tmp_path, robot, depth, wrist_model
):
    site, simulation = documents(tmp_path, robot=robot)
    for document in (site, simulation):
        assert document["cameras"]["scene"]["options"] == {
            "sensor_model": "rgb" if robot == "so101" else "realsense_d435",
            "depth": depth,
        }
        assert document["cameras"]["wrist"]["options"] == {
            "sensor_model": wrist_model,
            "depth": depth,
        }
        for camera in document["cameras"].values():
            assert ("depth_scale_mm" in camera["intrinsics"]) is depth


def test_physical_camera_profiles_replace_every_simulated_optical_contract(tmp_path):
    profiles = {
        "scene": {
            "stream": {"width": 848, "height": 480, "fps": 30},
            "intrinsics": {
                "fx": 431.2,
                "fy": 430.8,
                "cx": 421.7,
                "cy": 238.9,
                "depth_scale_mm": 0.1,
            },
            "transform": np.eye(4).tolist(),
        },
        "wrist": {
            "stream": {"width": 640, "height": 480, "fps": 15},
            "intrinsics": {
                "fx": 387.1,
                "fy": 386.9,
                "cx": 318.4,
                "cy": 241.1,
                "depth_scale_mm": 0.1,
            },
            "transform": transform((0.02, 0.01, -0.04), (0.0, 0.1, 0.0)).tolist(),
        },
    }
    site, simulation = make_site(
        "calibrated",
        backend="mujoco",
        robot="yam",
        environment="two_cubes",
        camera_profiles=profiles,
    )
    (tmp_path / "simulation.json").write_text(json.dumps(simulation))
    (tmp_path / "site.yaml").write_text(yaml.safe_dump(site))
    load_site(tmp_path / "site.yaml")
    assert load_scene(tmp_path, "simulation.json")[1] == simulation
    for name, profile_row in profiles.items():
        assert site["cameras"][name]["stream"] == profile_row["stream"]
        assert site["cameras"][name]["intrinsics"] == profile_row["intrinsics"]
        assert simulation["cameras"][name]["stream"] == profile_row["stream"]
        assert simulation["cameras"][name]["intrinsics"] == profile_row["intrinsics"]
        assert simulation["cameras"][name]["transform"] == profile_row["transform"]


def test_physical_camera_profiles_require_exact_names_and_depth_capability():
    with pytest.raises(ValueError, match="exactly"):
        make_site(
            "incomplete-calibration",
            backend="mujoco",
            robot="yam",
            environment="two_cubes",
            camera_profiles={},
        )
    rgb_profile = {
        name: {
            "stream": {"width": 640, "height": 480, "fps": 30},
            "intrinsics": {
                "fx": 400.0,
                "fy": 400.0,
                "cx": 319.5,
                "cy": 239.5,
                "depth_scale_mm": 1.0,
            },
            "transform": np.eye(4).tolist(),
        }
        for name in ("scene", "wrist")
    }
    with pytest.raises(ValueError, match="depth capability"):
        make_site(
            "wrong-depth",
            backend="mujoco",
            robot="so101",
            environment="two_cubes",
            camera_profiles=rgb_profile,
        )


@pytest.mark.parametrize("robot", ROBOTS)
def test_two_arm_mujoco_declaration_has_independent_parts_and_wrist_cameras(
    tmp_path, robot
):
    site, simulation = documents(tmp_path, robot=robot, arms=2)
    loaded = load_site(tmp_path / "site.yaml")
    assembly = loaded._assembly(None)
    assert set(site["parts"]) == set(simulation["parts"]) == {"left", "right"}
    assert set(site["cameras"]) == {"scene", "left_wrist", "right_wrist"}
    assert set(assembly.rig.robot().action_space.parts) == {"left", "right"}
    assert site["parts"]["left"]["base_frame"] != site["parts"]["right"]["base_frame"]
    assert simulation["parts"]["left"]["xyz"][1] == pytest.approx(-0.28)
    assert simulation["parts"]["right"]["xyz"][1] == pytest.approx(0.28)


def test_non_mujoco_two_arm_reference_scene_is_rejected():
    with pytest.raises(ValueError, match="two-arm.*MuJoCo"):
        make_site(
            "dual",
            backend="sapien",
            robot="yam",
            environment="two_cubes",
            arms=2,
        )


@pytest.mark.parametrize("robot", ROBOTS)
def test_reference_planning_source_retains_arm_contract_without_scene_joints(
    tmp_path, robot
):
    mujoco = pytest.importorskip("mujoco")
    bundle = reference_model_sources(robot, part_name="left")
    repeated = reference_model_sources(robot, part_name="left")
    assert repeated.model == bundle.model
    assert dict(repeated.assets) == dict(bundle.assets)
    assert len(bundle.assets) <= 127
    assert all(len(content) <= 32 * 1024 * 1024 for content in bundle.assets.values())
    model_path = tmp_path / "model.xml"
    model_path.write_bytes(bundle.model)
    for name, content in bundle.assets.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)
    model = mujoco.MjModel.from_xml_path(str(model_path))
    joint_names = tuple(
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index)
        for index in range(model.njnt)
    )
    assert joint_names == profile(robot).names[:-1] == bundle.joint_names
    assert bundle.joint_units == ("rad",) * profile(robot).dof
    assert bundle.base_body == "base"
    assert bundle.tcp_site == "tcp_site"
    assert bundle.tcp_frame == "left_tool"
    assert model.ngeom <= 256 and model.nmeshvert <= 200_000
    geom_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index)
        for index in range(model.ngeom)
    ]
    assert all(geom_names)
    assert len(set(geom_names)) == len(geom_names)
    proxy = bundle.provenance["collision_proxy"]
    assert proxy["planner_geometry_count"] == model.ngeom
    assert proxy["source_piece_count"] >= proxy["planner_geometry_count"]
    assert proxy["method"] == "spatial_convex_hulls_of_complete_source_collision_pieces"
    assert proxy["bucket_width_m"] == 0.02


def _minimum_body_distance(mujoco, model, data, first, second):
    body_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        for name in (first, second)
    ]
    geoms = [
        [
            index
            for index, body in enumerate(model.geom_bodyid)
            if int(body) == body_id and int(model.geom_contype[index]) != 0
        ]
        for body_id in body_ids
    ]
    assert all(geoms)
    distances = [
        float(mujoco.mj_geomDistance(model, data, a, b, 1.0, None))
        for a in geoms[0]
        for b in geoms[1]
    ]
    penetrations = [distance for distance in distances if distance < 0]
    if penetrations:
        return min(penetrations)
    # MuJoCo 3.11 reports zero for a degenerate convex-mesh query even when
    # there is no contact. Positive queries on the remaining complete pieces
    # provide the physical clearance this regression compares.
    clearances = [distance for distance in distances if distance > 0]
    assert clearances
    return min(clearances)


def test_xarm_planner_source_matches_valid_pull_and_retains_true_collision(tmp_path):
    """The bounded planning proxy keeps the runtime collision classification."""

    mujoco = pytest.importorskip("mujoco")
    bundle = reference_model_sources("xarm7", part_name="arm")
    planner_root = tmp_path / "planner"
    planner_root.mkdir()
    model_path = planner_root / "model.xml"
    model_path.write_bytes(bundle.model)
    for name, content in bundle.assets.items():
        path = planner_root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)
    planning_model = mujoco.MjModel.from_xml_path(str(model_path))
    planning_data = mujoco.MjData(planning_model)

    from waddle_sdk.simulators.mujoco import Engine

    _, config = make_site(
        "xarm-planner-parity",
        backend="mujoco",
        robot="xarm7",
        environment="drawer",
        width=32,
        height=24,
    )
    runtime_root = tmp_path / "runtime"
    runtime_root.mkdir()
    runtime = Engine(config, runtime_root)

    def compare(q, bodies):
        runtime.home((*q, 1.0))
        runtime_distance = _minimum_body_distance(
            mujoco, runtime.model, runtime.data, *bodies
        )
        for name, value in zip(profile("xarm7").names[:-1], q, strict=True):
            planning_data.qpos[int(planning_model.joint(name).qposadr[0])] = value
        mujoco.mj_forward(planning_model, planning_data)
        planning_distance = _minimum_body_distance(
            mujoco, planning_model, planning_data, *bodies
        )
        return runtime_distance, planning_distance

    try:
        pull_75 = (
            -0.47238,
            0.0921975,
            0.4527625,
            0.13521,
            -3.1627225,
            1.5215025,
            1.5433175,
        )
        full_pull = (
            -0.56731,
            0.04854,
            0.54599,
            0.05310,
            -3.16345,
            1.55916,
            1.54585,
        )
        for q in (pull_75, full_pull):
            runtime_distance, planning_distance = compare(q, ("link2", "link4"))
            assert runtime_distance > 0.03
            assert planning_distance == pytest.approx(runtime_distance, abs=1e-5)

        true_collision = (
            2.9480633482992573,
            -1.071559859638074,
            -3.500871524453849,
            -0.0746701755359535,
            -2.9156410161234247,
            0.49064709922779604,
            4.719526272546258,
        )
        runtime_distance, planning_distance = compare(true_collision, ("base", "link5"))
        assert runtime_distance < -0.02
        assert planning_distance < -0.02
    finally:
        runtime.close()


def test_yam_planner_source_accepts_native_clearance_and_retains_true_collision(
    tmp_path,
):
    """Concave source meshes must not become one false convex planning hull."""

    mujoco = pytest.importorskip("mujoco")
    bundle = reference_model_sources("yam", part_name="arm")
    planner_root = tmp_path / "planner-yam"
    planner_root.mkdir()
    model_path = planner_root / "model.xml"
    model_path.write_bytes(bundle.model)
    for name, content in bundle.assets.items():
        path = planner_root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)
    planning_model = mujoco.MjModel.from_xml_path(str(model_path))
    planning_data = mujoco.MjData(planning_model)

    from waddle_sdk.simulators.mujoco import Engine

    _, config = make_site(
        "yam-planner-parity",
        backend="mujoco",
        robot="yam",
        environment="two_cubes",
        width=32,
        height=24,
    )
    runtime_root = tmp_path / "runtime-yam"
    runtime_root.mkdir()
    runtime = Engine(config, runtime_root)

    def compare(q, bodies):
        runtime.home((*q, 1.0))
        runtime_distance = _minimum_body_distance(
            mujoco, runtime.model, runtime.data, *bodies
        )
        for name, value in zip(profile("yam").names[:-1], q, strict=True):
            planning_data.qpos[int(planning_model.joint(name).qposadr[0])] = value
        mujoco.mj_forward(planning_model, planning_data)
        planning_distance = _minimum_body_distance(
            mujoco, planning_model, planning_data, *bodies
        )
        return runtime_distance, planning_distance

    try:
        clear_targets = (
            (
                (
                    -0.1645073941,
                    0.8466620062,
                    0.3973416481,
                    -0.3213553208,
                    0.0978822564,
                    -0.5,
                ),
                (0.26, -0.06, 0.08),
            ),
            (
                (
                    0.8728566265,
                    1.9691198157,
                    1.1706587770,
                    0.3026730298,
                    -0.2423370779,
                    -0.5,
                ),
                (0.32, 0.445, 0.10),
            ),
        )
        for q, expected_tcp in clear_targets:
            runtime_distance, planning_distance = compare(q, ("base", "link_2"))
            assert runtime_distance > 0.001
            assert planning_distance > 0.001
            np.testing.assert_allclose(
                runtime.native_tcp()[0], expected_tcp, atol=3e-5, rtol=0
            )

        true_collision = (
            1.6142213825,
            0.0750332025,
            0.6630894444,
            -1.4551047427,
            0.5088703553,
            1.8968663964,
        )
        runtime_distance, planning_distance = compare(
            true_collision, ("link_1", "tip_right")
        )
        assert runtime_distance < -0.003
        assert planning_distance < -0.003
    finally:
        runtime.close()


@pytest.mark.parametrize("robot", ROBOTS)
def test_native_two_arm_mujoco_keeps_commands_and_cameras_part_scoped(
    tmp_path, monkeypatch, robot
):
    pytest.importorskip("mujoco")
    monkeypatch.setenv("MUJOCO_GL", "egl")
    from waddle_sdk.simulators.mujoco import Engine

    _, config = make_site(
        "dual-native",
        backend="mujoco",
        robot=robot,
        environment="two_cubes",
        width=96,
        height=72,
        arms=2,
    )
    engine = Engine(config, tmp_path)
    try:
        left, right = engine.read("left")[0], engine.read("right")[0]
        left_tcp = engine.native_tcp("left")[0]
        right_tcp = engine.native_tcp("right")[0]
        assert right_tcp[1] - left_tcp[1] == pytest.approx(0.56)
        target = left.copy()
        target[0] += 0.05
        engine.write("left", target)
        for _ in range(800):
            engine.step()
        assert engine.read("left")[0][0] > left[0] + 0.03
        np.testing.assert_allclose(engine.read("right")[0], right, atol=0.01)
        for camera in ("left_wrist", "right_wrist"):
            rgb, depth = engine.capture(camera)
            assert rgb.shape == (72, 96, 3)
            assert (depth is None) is (robot == "so101")
    finally:
        engine.close()


def test_yam_fk_matches_live_adapter_at_multiple_configurations():
    p = profile("yam")
    rng = np.random.default_rng(12)
    for _ in range(20):
        q = np.array([rng.uniform(lo, hi) for lo, hi in p.limits])
        expected_pos, expected_rot = yam.forward_kinematics(q[:-1])
        pose = p.poses(q)[-1]
        np.testing.assert_allclose(pose[:3, 3], expected_pos, atol=1e-9)
        np.testing.assert_allclose(pose[:3, :3], expected_rot, atol=1e-9)


def test_yam_reference_home_is_in_the_tabletop_working_region():
    p = profile("yam")
    position, orientation = yam.forward_kinematics(p.home[:-1])
    assert 0.27 < position[0] < 0.29
    assert abs(position[1]) < 0.02
    assert 0.15 < position[2] < 0.17
    assert orientation[0, 2] > 0.5 and orientation[2, 2] < -0.5
    assert all(
        lo < q < hi for q, (lo, hi) in zip(p.home[:-1], p.limits[:-1], strict=True)
    )


@pytest.mark.parametrize("robot", ROBOTS)
def test_manufacturer_assets_are_complete_and_hash_bound(robot):
    root = Path(str(files("waddle_sdk.simulators").joinpath("data")))
    manifest = json.loads((root / "models.json").read_text())["files"]
    model = description(robot)
    assert len(model.links) > profile(robot).dof + 2
    assert (root / robot / "LICENSE").is_file()
    for relative, digest in manifest.items():
        if relative.startswith(robot + "/"):
            assert hashlib.sha256((root / relative).read_bytes()).hexdigest() == digest
    for link in model.links:
        for shape in link.shapes:
            assert shape.kind == "mesh"
            assert str(Path(shape.mesh).relative_to(root)) in manifest
    # Geometry has real robot scale and masses, not the previous .3 kg links.
    minimum_mass = {"so101": 0.6, "yam": 4.0, "xarm7": 10.0}[robot]
    assert sum(link.mass for link in model.links) > minimum_mass
    finger_names = {
        "so101": ("gripper_link", "moving_jaw_so101_v1_link"),
        "yam": ("tip_left", "tip_right"),
        "xarm7": ("left_finger", "right_finger"),
    }[robot]
    fingers = [link for link in model.links if link.name in finger_names]
    assert len(fingers) == 2
    assert all(
        1 <= sum(shape.collision for shape in link.shapes) <= 32 for link in fingers
    )
    if robot == "xarm7":
        housing = next(
            link for link in model.links if link.name == "xarm_gripper_base_link"
        )
        assert 1 < sum(shape.collision for shape in housing.shapes) <= 32


@pytest.mark.parametrize("robot", ROBOTS)
def test_urdf_fk_and_physical_jaw_travel_match_public_coordinates(robot):
    model, p = description(robot), profile(robot)
    rng = np.random.default_rng(43)
    for _ in range(10):
        q = [rng.uniform(lo, hi) for lo, hi in p.limits]
        np.testing.assert_allclose(model.poses(q)["tcp"], p.poses(q)[-1], atol=1e-9)
    left, right = {
        "so101": ("moving_jaw_so101_v1_link", "gripper_link"),
        "yam": ("tip_left", "tip_right"),
        "xarm7": ("left_finger", "right_finger"),
    }[robot]
    widths = []
    for opening in (0.0, 0.25, 0.7, 1.0):
        q = list(p.home)
        q[-1] = opening
        poses = model.poses(q)
        axis = poses["tcp"][:3, :3] @ p.closing_axis
        widths.append(float(axis @ (poses[left][:3, 3] - poses[right][:3, 3])))
        hand = model.hand_position(opening)
        measured, velocity = model.hand_state(hand, 0.0)
        assert measured == pytest.approx(opening)
        assert velocity == 0
        assert all(lo - 1e-12 <= hand <= hi + 1e-12 for lo, hi in model.hand_limits)
    if robot == "so101":
        # The real Feetech adapter exposes motor range percentage. SO-101's
        # rotating jaw therefore has nonlinear physical aperture over that
        # normalized action, unlike the two parallel-jaw mechanisms.
        assert model.hand_position(0.0) == pytest.approx(model.hand_limits[0][0])
        assert model.hand_position(1.0) == pytest.approx(model.hand_limits[0][1])
    else:
        np.testing.assert_allclose(
            np.array(widths) - widths[0],
            np.array([0, 0.25, 0.7, 1]) * p.opening,
            atol=1e-6,
        )


@pytest.mark.parametrize("robot", ROBOTS)
def test_mesh_sphere_cover_contains_collision_triangles(robot):
    bounds = collision_bounds(robot)
    for link in description(robot).links:
        spheres = [(c, r) for _name, owner, c, r in bounds if owner == link.name]
        for shape in link.shapes:
            if not shape.collision:
                continue
            triangles = mesh_triangles(shape.mesh) * shape.size
            triangles = triangles @ rotation(shape.rpy).T + shape.xyz
            # Every sampled whole triangle must fit one sphere, not merely have
            # its vertices in unrelated spheres with an uncovered middle.
            for triangle in triangles[:: max(1, len(triangles) // 100)]:
                assert any(
                    np.linalg.norm(triangle - center, axis=1).max() <= radius
                    for center, radius in spheres
                )


def test_mesh_sphere_cover_is_independent_of_collision_partition(monkeypatch):
    module = importlib.import_module("waddle_sdk.simulators.description")
    triangles = np.array(
        [[[x, 0, 0], [x + 0.01, 0.01, 0], [x, 0, 0.01]] for x in (0, 0.02, 0.1, 0.12)]
    )
    meshes = {"whole": triangles, "a": triangles[::2], "b": triangles[1::2]}

    def shape(mesh):
        return SimpleNamespace(
            collision=True, mesh=mesh, size=(1, 2, 1), rpy=(0, 0, 0.5), xyz=(0.1, 0, 0)
        )

    link = SimpleNamespace(name="link", shapes=[shape("whole")])
    monkeypatch.setattr(module, "description", lambda _: SimpleNamespace(links=[link]))
    monkeypatch.setattr(module, "mesh_triangles", meshes.__getitem__)
    collision_bounds.cache_clear()
    try:
        whole = collision_bounds("fixture")
        link.shapes = [shape("a"), shape("b")]
        collision_bounds.cache_clear()
        split = collision_bounds("fixture")
        assert len(split) == len(whole)
        for expected, actual in zip(whole, split):
            assert actual[:2] == expected[:2]
            np.testing.assert_allclose(actual[2], expected[2], atol=1e-12)
            assert actual[3] == pytest.approx(expected[3], abs=1e-12)
    finally:
        collision_bounds.cache_clear()


def test_yam_arm_assembly_retains_the_shipped_urdf_inertials():
    original = ET.fromstring(yam.urdf_text())
    assembled = ET.parse(files("waddle_sdk.simulators").joinpath("data/yam/robot.urdf"))
    for name in ("base_link", *(f"link_{i}" for i in range(1, 6))):
        expected = original.find(f"link[@name='{name}']/inertial")
        actual = assembled.find(f"link[@name='{name}']/inertial")
        for tag in ("origin", "mass", "inertia"):
            assert actual.find(tag).attrib == expected.find(tag).attrib


def test_depth_units_and_invalid_returns():
    raw = depth_z16([[1.0, 0.001, np.nan, np.inf, -1.0, 70.0]], 1.0)
    assert raw.dtype == np.uint16
    assert raw.tolist() == [[1000, 1, 0, 0, 0, 0]]
    assert depth_z16([[1.0]], 0.5).item() == 2000


@pytest.mark.parametrize("backend", BACKENDS)
def test_render_quality_preserves_site_and_scene_contract(tmp_path, backend):
    baseline_site, baseline = make_site(
        "render-test", backend=backend, robot="yam", environment="two_cubes"
    )
    assert baseline["render_quality"] == "standard"
    for quality in RENDER_QUALITIES:
        site, scene = make_site(
            "render-test",
            backend=backend,
            robot="yam",
            environment="two_cubes",
            render_quality=quality,
        )
        assert site == baseline_site
        assert scene == {**baseline, "render_quality": quality}
        (tmp_path / "simulation.json").write_text(json.dumps(scene))
        assert load_scene(tmp_path, "simulation.json")[1] == scene
    # Existing workspaces need no scene rewrite or calibration-hash migration.
    del baseline["render_quality"]
    (tmp_path / "simulation.json").write_text(json.dumps(baseline))
    assert load_scene(tmp_path, "simulation.json")[1] == baseline


@pytest.mark.parametrize("quality", ["ultra", "", None, 3, {}])
def test_invalid_render_quality_is_rejected_before_opening(tmp_path, quality):
    with pytest.raises(ValueError, match="render_quality"):
        make_site(
            "render-test",
            backend="mujoco",
            robot="yam",
            environment="two_cubes",
            render_quality=quality,
        )
    _, scene = documents(tmp_path)
    scene["render_quality"] = quality
    (tmp_path / "simulation.json").write_text(json.dumps(scene))
    with pytest.raises(ValueError, match="render_quality"):
        load_scene(tmp_path, "simulation.json")


@pytest.mark.parametrize(
    "field", ["scene_revision", "asset_revision", "embodiment_revision"]
)
@pytest.mark.parametrize("revision", ["1", "v1.0.0", "", None, 1])
def test_invalid_reference_revision_is_rejected(tmp_path, field, revision):
    _, scene = documents(tmp_path)
    scene[field] = revision
    (tmp_path / "simulation.json").write_text(json.dumps(scene))
    with pytest.raises(ValueError, match="semantic revision"):
        load_scene(tmp_path, "simulation.json")


@pytest.mark.parametrize(
    "relative", ["../scene.json", "/tmp/scene.json", "a\\scene.json"]
)
def test_scene_paths_are_confined(tmp_path, relative):
    with pytest.raises(ValueError):
        load_scene(tmp_path, relative)


def test_scene_symlink_escape_and_invalid_rotation(tmp_path):
    documents(tmp_path)
    (tmp_path / "escape").symlink_to(tmp_path.parent, target_is_directory=True)
    with pytest.raises((ValueError, FileNotFoundError)):
        load_scene(tmp_path, "escape/simulation.json")
    _, sim = documents(tmp_path)
    sim["cameras"]["scene"]["transform"][0][0] = 3
    (tmp_path / "simulation.json").write_text(json.dumps(sim))
    with pytest.raises(ValueError, match="orthonormal"):
        load_scene(tmp_path, "simulation.json")


def _native_python(backend, environment):
    if backend == "isaac":
        interpreter = os.environ.get("WADDLE_ISAAC_TEST_PYTHON")
        if interpreter is None:
            pytest.skip(
                "set WADDLE_ISAAC_TEST_PYTHON to a licensed Isaac Sim interpreter"
            )
    elif backend == "sapien" and environment == "bottle_cap":
        interpreter = os.environ.get("WADDLE_SAPIEN_GPU_TEST_PYTHON")
        if interpreter is None:
            pytest.skip("set WADDLE_SAPIEN_GPU_TEST_PYTHON to a CUDA/SAPIEN worker")
    else:
        pytest.importorskip(backend)
        interpreter = sys.executable
    return interpreter


@pytest.mark.parametrize("robot", ROBOTS)
@pytest.mark.parametrize(("backend", "environment"), ENVIRONMENT_BACKENDS)
def test_native_models_rgbd_and_bounded_joint_motion(
    tmp_path, monkeypatch, backend, robot, environment
):
    interpreter = _native_python(backend, environment)
    monkeypatch.setenv("MUJOCO_GL", "egl")
    monkeypatch.setenv(
        "PYTHONPATH", str(Path(__file__).resolve().parents[1] / "python")
    )
    # Physics engines own process-global graphics runtimes. Match the production
    # worker isolation, including when this suite runs after other engine tests.
    script = (
        "import runpy; from pathlib import Path; "
        f"ns = runpy.run_path({str(Path(__file__).resolve())!r}); "
        f"ns['_native_conformance'](Path({str(tmp_path)!r}), "
        f"{backend!r}, {robot!r}, {environment!r})"
    )
    result = subprocess.run(
        [interpreter, "-c", script], capture_output=True, text=True, timeout=300
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_native_sapien_gpu_prop_placement(tmp_path, monkeypatch):
    interpreter = os.environ.get("WADDLE_SAPIEN_GPU_TEST_PYTHON")
    if interpreter is None:
        pytest.skip(
            "set WADDLE_SAPIEN_GPU_TEST_PYTHON to a CUDA/SAPIEN/Torch interpreter"
        )
    monkeypatch.setenv(
        "PYTHONPATH", str(Path(__file__).resolve().parents[1] / "python")
    )
    script = (
        "import runpy; from pathlib import Path; "
        f"ns = runpy.run_path({str(Path(__file__).resolve())!r}); "
        f"ns['_native_sapien_gpu_prop_placement'](Path({str(tmp_path)!r}))"
    )
    result = subprocess.run(
        [interpreter, "-c", script],
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def _native_sapien_gpu_prop_placement(tmp_path):
    import sapien
    import torch
    from waddle_sdk.simulators.model import objects
    from waddle_sdk.simulators.sapien import Engine

    sapien.physx.enable_gpu()
    physics = sapien.physx.PhysxGpuSystem()
    physics.gpu_set_cuda_stream(torch.cuda.current_stream().cuda_stream)
    # Exercise the shared importer directly, including the cube fixtures that
    # ordinarily use CPU physics. GPU insertion has different pose semantics.
    engine = Engine.__new__(Engine)
    engine.sp = sapien
    engine.scene = sapien.Scene([physics, sapien.render.RenderSystem()])
    engine.scene.set_timestep(0.002)
    groups = objects("two_cubes")
    props = [engine._load(group, group[0].name, tmp_path) for group in groups]
    physics.gpu_init()
    physics.gpu_fetch_rigid_dynamic_data()
    native = physics.cuda_rigid_dynamic_data.torch().cpu().numpy()
    for entity, group in zip(props, groups, strict=True):
        if group[0].kind != "free":
            continue
        body = entity.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent)
        # GPU initialization performs one native warm-up step. Gravity may
        # move a correctly placed cube by 0.04 mm, not relocate it to the origin.
        np.testing.assert_allclose(
            native[body.gpu_pose_index, :3], group[0].xyz, atol=0.0001
        )


@pytest.mark.parametrize("backend", BACKENDS)
@pytest.mark.parametrize("quality", ["fast", "high"])
def test_native_render_presets_preserve_rgbd_and_motion(
    tmp_path, monkeypatch, backend, quality
):
    interpreter = _native_python(backend, "two_cubes")
    monkeypatch.setenv("MUJOCO_GL", "egl")
    monkeypatch.setenv(
        "PYTHONPATH", str(Path(__file__).resolve().parents[1] / "python")
    )
    script = (
        "import runpy; from pathlib import Path; "
        f"ns = runpy.run_path({str(Path(__file__).resolve())!r}); "
        f"ns['_native_conformance'](Path({str(tmp_path)!r}), "
        f"{backend!r}, 'yam', 'two_cubes', {quality!r})"
    )
    result = subprocess.run(
        [interpreter, "-c", script], capture_output=True, text=True, timeout=300
    )
    assert result.returncode == 0, result.stdout + result.stderr


def test_xarm_gripper_motor_budget_respects_rated_jaw_force():
    # G2's published 50 N rating is a gripping force, not hinge torque. Use
    # the actual jaw transmission over its whole travel to check virtual work.
    robot = description("xarm7")
    p = profile("xarm7")
    effort = robot.servo(robot.hand_names[0])[2]
    for opening in np.linspace(0, 1, 31):
        angle = robot.hand_position(opening)
        _, derivative = robot.hand_state(angle, 1.0)
        jaw_force = effort / abs(derivative * p.opening)
        assert 10 <= jaw_force <= 50.001


@pytest.mark.parametrize("backend", ["mujoco", "sapien"])
def test_native_xarm_gripper_preserves_linkage_under_contact(
    tmp_path, monkeypatch, backend
):
    interpreter = _native_python(backend, "bottle_cap")
    monkeypatch.setenv("MUJOCO_GL", "egl")
    monkeypatch.setenv(
        "PYTHONPATH", str(Path(__file__).resolve().parents[1] / "python")
    )
    script = (
        "import runpy; from pathlib import Path; "
        f"ns = runpy.run_path({str(Path(__file__).resolve())!r}); "
        f"ns['_native_xarm_gripper_contact'](Path({str(tmp_path)!r}), {backend!r})"
    )
    result = subprocess.run(
        [interpreter, "-c", script],
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("robot", ROBOTS)
def test_native_mujoco_cap_retains_axial_load_and_unscrews_freely(
    tmp_path, monkeypatch, robot
):
    pytest.importorskip("mujoco")
    monkeypatch.setenv("MUJOCO_GL", "egl")
    monkeypatch.setenv(
        "PYTHONPATH", str(Path(__file__).resolve().parents[1] / "python")
    )
    script = (
        "import runpy; from pathlib import Path; "
        f"ns = runpy.run_path({str(Path(__file__).resolve())!r}); "
        f"ns['_native_free_cap'](Path({str(tmp_path)!r}), {robot!r})"
    )
    result = subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("backend", ["sapien", "isaac"])
@pytest.mark.parametrize("robot", ROBOTS)
def test_native_physx_cap_retains_axial_load_and_unscrews_freely(
    tmp_path, monkeypatch, backend, robot
):
    interpreter = _native_python(backend, "bottle_cap")
    monkeypatch.setenv(
        "PYTHONPATH", str(Path(__file__).resolve().parents[1] / "python")
    )
    script = (
        "import runpy; from pathlib import Path; "
        f"ns = runpy.run_path({str(Path(__file__).resolve())!r}); "
        f"ns['_native_physx_free_cap'](Path({str(tmp_path)!r}), {backend!r}, {robot!r})"
    )
    result = subprocess.run(
        [interpreter, "-c", script],
        capture_output=True,
        text=True,
        timeout=300,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def _native_physx_free_cap(tmp_path, backend, robot):
    _, config = documents(tmp_path, backend, robot, "bottle_cap")
    engine = importlib.import_module(f"waddle_sdk.simulators.{backend}").Engine(
        config, tmp_path
    )
    try:
        if backend == "sapien":
            px = engine.sp.physx
            physics = engine.gpu.physics
            entity = engine.props[-1]
            assert entity.name == "cap"
            assert (
                entity.find_component_by_type(px.PhysxArticulationLinkComponent) is None
            )
            body = entity.find_component_by_type(px.PhysxRigidDynamicComponent)
            assert body.mass == pytest.approx(0.025)
            np.testing.assert_allclose(
                body.inertia,
                [9.733922509e-6, 7.021903874e-6, 5.859590435e-6],
                rtol=1e-6,
            )
            state = physics.cuda_rigid_dynamic_data.torch()
            forces = physics.cuda_rigid_dynamic_force.torch()
            torques = physics.cuda_rigid_dynamic_torque.torch()
            index = body.gpu_pose_index

            def read():
                physics.gpu_fetch_rigid_dynamic_data()
                return state[index].cpu().numpy().copy()

            def apply(force, torque):
                forces.zero_()
                torques.zero_()
                forces[index, 2], torques[index, 2] = force, torque
                physics.gpu_apply_rigid_dynamic_force()
                physics.gpu_apply_rigid_dynamic_torque()

            bolt = engine.props[-2]
            bolt_body = bolt.find_component_by_type(px.PhysxRigidStaticComponent)
            shape = bolt_body.collision_shapes[0]
            transform = bolt.pose.to_transformation_matrix()
            vertices = np.asarray(shape.vertices) * shape.scale
            top = np.max(vertices @ transform[:3, :3].T + transform[:3, 3], axis=0)[2]
        else:
            from pxr import Usd, UsdGeom, UsdPhysics

            cap = engine.props[-1]
            context = engine.world.get_physics_context()
            assert context.is_gpu_dynamics_enabled()
            assert context.get_broadphase_type() == "GPU"
            root = engine.stage.GetPrimAtPath("/World/threaded_cap")
            assert not any(prim.IsA(UsdPhysics.Joint) for prim in Usd.PrimRange(root))
            assert cap.get_mass() == pytest.approx(0.025)

            def read():
                position, orientation = cap.get_world_pose()
                return np.r_[
                    position,
                    orientation,
                    cap.get_linear_velocity(),
                    cap.get_angular_velocity(),
                ]

            def apply(force, torque):
                cap._rigid_prim_view.apply_forces_and_torques_at_pos(
                    forces=np.array([[0, 0, force]], dtype=np.float32),
                    torques=np.array([[0, 0, torque]], dtype=np.float32),
                    is_global=True,
                )

            bolt = UsdGeom.Mesh.Get(
                engine.stage, "/World/threaded_cap/bottle_thread/thread"
            )
            transform = np.asarray(
                bolt.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            )
            vertices = np.asarray(bolt.GetPointsAttr().Get())
            top = np.max(vertices @ transform[:3, :3] + transform[3, :3], axis=0)[2]

        initial = read()
        clear = np.array(engine.profile.home)
        clear[0] = np.pi / 2
        engine.home(clear)

        def advance(seconds, force=0.0, torque=0.0):
            heights = []
            for _ in range(round(seconds / config["timestep"])):
                apply(force, torque)
                engine.step()
                heights.append(read()[2])
            return np.asarray(heights)

        advance(3)
        height = read()[2]
        assert np.max(np.abs(advance(2, force=2.0) - height)) < 0.001
        advance(1)

        def angle(value):
            w, x, y, z = value[3:7]
            return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

        # A 40 mm sphere encloses the cap including its offset roof. Clearing
        # the bolt's actual mesh bounds by that radius proves contact-free exit.
        last, total = angle(read()), 0.0
        samples = []
        for _ in range(round(8 / config["timestep"])):
            apply(0.5, 0.01)
            engine.step()
            current = read()
            value = angle(current)
            total += np.arctan2(np.sin(value - last), np.cos(value - last))
            last = value
            turns = total / (2 * np.pi)
            samples.append((turns, current[2]))
            if current[2] - 0.04 > top + 0.01:
                break
        assert current[2] - 0.04 > top + 0.01
        engaged = np.array([row for row in samples if 0.5 < row[0] < 4])
        assert len(engaged) > 20
        slope, intercept = np.polyfit(engaged[:, 0], engaged[:, 1], 1)
        assert slope == pytest.approx(0.05 / 12, abs=0.0002)
        assert np.max(abs(engaged[:, 1] - slope * engaged[:, 0] - intercept)) < 0.001
        vertical_speed = read()[9]  # native center-of-mass linear velocity
        advance(0.05)
        assert read()[9] == pytest.approx(vertical_speed - 9.81 * 0.05, abs=0.005)
        engine.reset()
        np.testing.assert_allclose(read(), initial, atol=1e-7)
        np.testing.assert_allclose(engine.read()[0], engine.profile.home, atol=1e-6)
    finally:
        engine.close()


def test_native_thread_assets_are_complete_and_hash_bound():
    from waddle_sdk.simulators.thread import assets

    root = assets()
    manifest = json.loads((root / "manifest.json").read_text())
    meshes = ET.parse(root / "cap.xml").findall("asset/mesh[@file]")
    required = {
        "cap.xml",
        "cap.usdc",
        "metric_thread.cc",
        "LICENSE",
        "nut-physx.stl",
        "bolt-physx.stl",
    }
    required.update(mesh.get("file") for mesh in meshes)
    assert required <= manifest["files"].keys()
    assert meshes


def test_native_thread_compiler_is_required_only_for_bottle_cap(tmp_path, monkeypatch):
    pytest.importorskip("mujoco")
    monkeypatch.setenv("CXX", str(tmp_path / "absent-compiler"))
    monkeypatch.setenv(
        "PYTHONPATH", str(Path(__file__).resolve().parents[1] / "python")
    )
    script = (
        "import runpy; from pathlib import Path; "
        f"ns = runpy.run_path({str(Path(__file__).resolve())!r}); "
        f"ns['_native_thread_compiler_dependency'](Path({str(tmp_path)!r}))"
    )
    result = subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True,
        text=True,
        timeout=60,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr


def _native_thread_compiler_dependency(tmp_path):
    from waddle_sdk.simulators.mujoco import Engine

    for environment in ENVIRONMENTS:
        dual = environment in DUAL_ARM_TASK_ENVIRONMENTS
        _, config = documents(
            tmp_path,
            "mujoco",
            "yam",
            environment,
            arms=2 if dual else 1,
        )
        if environment == "bottle_cap":
            with pytest.raises(RuntimeError, match="requires a C\\+\\+17 compiler"):
                Engine(config, tmp_path)
        else:
            engine = Engine(config, tmp_path)
            try:
                engine.step()
                assert np.isfinite(engine.read("left" if dual else None)[0]).all()
            finally:
                engine.close()


def _native_free_cap(tmp_path, robot):
    from waddle_sdk.simulators.mujoco import Engine

    _, config = documents(tmp_path, "mujoco", robot, "bottle_cap")
    engine = Engine(config, tmp_path)
    try:
        model, data, mj = engine.model, engine.data, engine.mj
        joint = model.joint("cap_free")
        assert joint.type[0] == mj.mjtJoint.mjJNT_FREE
        cap = model.body("cap")
        assert cap.mass[0] == pytest.approx(0.025)
        assert cap.mocapid[0] == -1
        qa = int(joint.qposadr[0])
        clear = np.array(engine.profile.home)
        clear[0] = np.pi / 2
        engine.home(clear)

        def advance(seconds):
            for _ in range(round(seconds / config["timestep"])):
                engine.step()

        def angle():
            w, x, y, z = data.qpos[qa + 3 : qa + 7]
            return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

        def vertical_bounds(body_id):
            corners = []
            for gid in np.flatnonzero(model.geom_bodyid == body_id):
                bounds = model.geom_aabb[gid]
                local = np.array(
                    [
                        bounds[:3] + bounds[3:] * [x, y, z]
                        for x in (-1, 1)
                        for y in (-1, 1)
                        for z in (-1, 1)
                    ]
                )
                corners.extend(
                    local @ data.geom_xmat[gid].reshape(3, 3).T + data.geom_xpos[gid]
                )
            return np.min(corners, axis=0)[2], np.max(corners, axis=0)[2]

        advance(2)
        initial_height, initial_angle = float(data.qpos[qa + 2]), angle()
        # A pull alone must not back-drive the cap through an ideal screw
        # guide. Only native contact/friction supplies thread self-locking.
        data.xfrc_applied[cap.id, 2] = 5
        advance(2)
        assert abs(data.qpos[qa + 2] - initial_height) < 0.001
        assert abs(angle() - initial_angle) < 0.02
        data.xfrc_applied[cap.id] = 0
        advance(1)
        assert data.qpos[qa + 2] == pytest.approx(initial_height, abs=0.0001)

        # A native wrench is a model diagnostic, not task acceptance. Follow
        # the actual turns and require natural release into free-body motion.
        data.xfrc_applied[cap.id] = [0, 0, 0.5, 0, 0, 0.01]
        last_angle, total_angle = angle(), 0.0
        samples = []
        for _ in range(round(8 / config["timestep"])):
            engine.step()
            value = angle()
            total_angle += np.arctan2(
                np.sin(value - last_angle), np.cos(value - last_angle)
            )
            last_angle = value
            turns = total_angle / (2 * np.pi)
            samples.append((turns, float(data.qpos[qa + 2])))
            cap_low = vertical_bounds(cap.id)[0]
            bottle_top = vertical_bounds(model.body("bottle_thread").id)[1]
            if cap_low > bottle_top + 0.01:
                break
        assert 4.5 < turns < 7.0, turns
        assert cap_low > bottle_top + 0.01
        engaged = np.array([row for row in samples if 0.5 < row[0] < 4])
        slope, intercept = np.polyfit(engaged[:, 0], engaged[:, 1], 1)
        assert slope == pytest.approx(0.05 / 12, abs=0.0002)
        assert np.max(abs(engaged[:, 1] - (slope * engaged[:, 0] + intercept))) < 0.001
        data.xfrc_applied[cap.id] = 0

        def com_vertical_speed():
            # A spinning free body's joint origin can orbit its offset COM.
            # Gravity makes the center of mass ballistic, not that origin.
            mj.mj_forward(model, data)
            mj.mj_subtreeVel(model, data)
            return float(data.subtree_linvel[cap.id, 2])

        vertical_speed = com_vertical_speed()
        advance(0.05)
        measured_speed = com_vertical_speed()
        assert measured_speed == pytest.approx(
            vertical_speed + 0.05 * model.opt.gravity[2],
            # Allow 1% integration error for the freely tumbling offset COM.
            abs=0.01 * abs(0.05 * model.opt.gravity[2]),
        ), (vertical_speed, measured_speed, data.qpos[qa : qa + 7].tolist())
        engine.reset()
        assert data.time == 0
        assert data.qpos[qa + 2] == pytest.approx(0.1941)
    finally:
        engine.close()


def _native_xarm_gripper_contact(tmp_path, backend):
    _, config = documents(tmp_path, backend, "xarm7", "bottle_cap")
    engine = importlib.import_module(f"waddle_sdk.simulators.{backend}").Engine(
        config, tmp_path
    )
    # TCP at (.34, -.16, .205) m, pointing down over the 50 mm cap. This
    # fixture loads the manufacturer fingers without moving the arm target.
    grasp = np.array(
        [
            0.25804668,
            -0.39298376,
            -0.62978786,
            0.78923564,
            -0.25263697,
            1.12517513,
            -2.84166090,
            1.0,
        ]
    )
    try:
        engine.home(grasp)
        for _ in range(round(1 / config["timestep"])):
            engine.step()
        grasp[-1] = 0.0
        engine.write(grasp)
        for _ in range(round(3 / config["timestep"])):
            engine.step()
        readings = []
        for _ in range(round(1 / config["timestep"])):
            engine.step()
            readings.append(engine.read()[0])
        readings = np.array(readings)
        widths = readings[:, -1] * engine.profile.opening
        if backend == "sapien":
            # The free cap's roof overhangs its 50 mm hexagonal shell. At this
            # fixture pose the long manufacturer pads span that wider roof.
            # Verify against actual collision geometry, not the old cylinder.
            px = engine.sp.physx
            cap = engine.props[-1].find_component_by_type(px.PhysxRigidDynamicComponent)
            roof = next(
                s
                for s in cap.collision_shapes
                if isinstance(s, px.PhysxCollisionShapeCylinder)
            )
            np.testing.assert_allclose(widths, 2 * roof.radius, atol=0.003, rtol=0)
            engine.gpu.sync()
            links = {link.name: link for link in engine.robot.get_links()}
            for first, anchor, second, other in engine.description.closures():
                a = links[first].get_entity_pose().to_transformation_matrix() @ [
                    *anchor,
                    1,
                ]
                b = links[second].get_entity_pose().to_transformation_matrix() @ [
                    *other,
                    1,
                ]
                assert np.linalg.norm(a - b) < 0.001
        else:
            assert 0.047 < min(widths) <= max(widths) < 0.055, widths[-1]
        assert np.ptp(widths) < 0.0002
        np.testing.assert_allclose(readings[-1, :-1], grasp[:-1], atol=0.004)
        if backend == "mujoco":
            model, data = engine.model, engine.data
            for name in engine.description.hand_names:
                joint = model.joint(name)
                value = data.qpos[int(joint.qposadr[0])]
                assert joint.range[0] - 0.01 < value < joint.range[1] + 0.01
            for index in range(data.nefc):
                if int(data.efc_type[index]) == int(
                    engine.mj.mjtConstraint.mjCNSTR_EQUALITY
                ):
                    assert abs(data.efc_pos[index]) < 0.001
        grasp[-1] = 1.0
        engine.write(grasp)
        for _ in range(round(3 / config["timestep"])):
            engine.step()
        assert engine.read()[0][-1] == pytest.approx(1.0, abs=0.01)
    finally:
        engine.close()


@pytest.mark.parametrize(
    "width,height,fx,fy,cx,cy",
    [
        (192, 144, 174.0, 161.0, 109.0, 62.0),
        (641, 479, 511.0, 497.0, 301.25, 260.75),
        (640, 480, 576.0, 576.0, 319.5, 239.5),
    ],
)
def test_isaac_usd_camera_projects_with_declared_intrinsics(
    width, height, fx, fy, cx, cy
):
    # OpenUSD's real projection matrix is available without Kit or an Isaac
    # license. Native rendered RGB-D acceptance remains a separate requirement.
    usd = pytest.importorskip("pxr.Usd", reason="requires standalone usd-core")
    from pxr import Gf
    from waddle_sdk.simulators.isaac import _camera

    stage = usd.Stage.CreateInMemory()
    camera = _camera(
        stage,
        "/camera",
        {
            "stream": {"width": width, "height": height},
            "intrinsics": {"fx": fx, "fy": fy, "cx": cx, "cy": cy},
        },
    )
    projection = camera.GetCamera(
        usd.TimeCode.Default()
    ).frustum.ComputeProjectionMatrix()
    for x, y, z in [(0, 0, 1), (0.1, -0.1, 0.5), (-0.1, 0.1, 2)]:
        # Site optical coordinates are +Z forward/+Y down; USD uses -Z/+Y up.
        ndc = projection.Transform(Gf.Vec3d(x, -y, -z))
        u, v = (ndc[0] + 1) * width / 2, (1 - ndc[1]) * height / 2
        np.testing.assert_allclose(
            [u, v], [fx * x / z + cx, fy * y / z + cy], atol=1e-5, rtol=0
        )
        np.testing.assert_allclose(
            [(u - cx) * z / fx, (v - cy) * z / fy, z],
            [x, y, z],
            atol=1e-7,
            rtol=0,
        )


@pytest.mark.parametrize("robot", ROBOTS)
def test_isaac_usd_robot_preserves_manufacturer_inertias_and_joint_tree(
    tmp_path, robot
):
    if importlib.util.find_spec("urdf_usd_converter") is None:
        pytest.skip("requires standalone urdf-usd-converter")
    # USD freezes its schema registry on first use. Match worker isolation so
    # camera tests cannot freeze it before the converter registers its schemas.
    script = (
        "import runpy; from pathlib import Path; "
        f"ns = runpy.run_path({str(Path(__file__).resolve())!r}); "
        f"ns['_usd_model_conformance'](Path({str(tmp_path)!r}), {robot!r})"
    )
    result = subprocess.run(
        [sys.executable, "-c", script], capture_output=True, text=True, timeout=60
    )
    assert result.returncode == 0, result.stdout + result.stderr


def _usd_model_conformance(tmp_path, robot):
    import urdf_usd_converter as converter
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
    from waddle_sdk.simulators.isaac import _closures
    from waddle_sdk.simulators.model import urdf

    model = description(robot)
    path = tmp_path / f"{robot}.urdf"
    path.write_text(urdf(model.native_links(), robot))
    asset = converter.Converter(layer_structure=False, scene=False).convert(
        str(path), str(tmp_path / "usd")
    )
    stage = Usd.Stage.Open(asset.path)
    assert UsdGeom.GetStageMetersPerUnit(stage) == 1.0
    assert UsdGeom.GetStageUpAxis(stage) == "Z"
    bodies = {
        prim.GetName(): prim
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.RigidBodyAPI)
    }
    assert set(bodies) == {link.name for link in model.links}
    for link in model.links:
        mass = UsdPhysics.MassAPI(bodies[link.name])
        assert mass.GetMassAttr().Get() == pytest.approx(link.mass, abs=1e-6)
        np.testing.assert_allclose(
            mass.GetCenterOfMassAttr().Get(), link.com, atol=1e-7
        )
        # Gf uses row vectors. Reconstruct the body-frame tensor from the
        # imported principal axes, independently of the converter's eigenbasis.
        axes = np.asarray(Gf.Matrix3d(mass.GetPrincipalAxesAttr().Get()))
        tensor = axes.T @ np.diag(mass.GetDiagonalInertiaAttr().Get()) @ axes
        xx, yy, zz, xy, xz, yz = link.inertia
        np.testing.assert_allclose(
            tensor,
            [[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]],
            atol=1e-8,
            rtol=0,
            err_msg=link.name,
        )
    root_path = str(stage.GetDefaultPrim().GetPath())
    _closures(stage, root_path, bodies, model)
    parsed = UsdPhysics.UsdPhysicsLoadStageFromPrimRange(stage, [Sdf.Path("/")])
    joints = [
        desc
        for _paths, descriptions in parsed.values()
        for desc in descriptions
        if isinstance(desc, UsdPhysics.JointDesc)
    ]
    # All manufacturer joints plus the world attachment must still form a
    # connected, acyclic articulation. Loop constraints stay active outside it.
    parent = {prim.GetPath(): prim.GetPath() for prim in bodies.values()}
    parent[Sdf.Path.emptyPath] = Sdf.Path.emptyPath

    def component(path):
        while parent[path] != path:
            path = parent[path]
        return path

    closures = []
    for joint in joints:
        assert joint.isValid and joint.jointEnabled
        if joint.excludeFromArticulation:
            closures.append(joint)
            continue
        first, second = component(joint.body0), component(joint.body1)
        assert first != second, f"articulation cycle at {joint.primPath}"
        parent[second] = first
    assert len({component(path) for path in parent}) == 1
    assert len(closures) == (2 if robot == "xarm7" else 0)
    transforms = UsdGeom.XformCache()
    for joint in closures:
        assert joint.type == UsdPhysics.ObjectType.SphericalJoint
        first = transforms.GetLocalToWorldTransform(stage.GetPrimAtPath(joint.body0))
        second = transforms.GetLocalToWorldTransform(stage.GetPrimAtPath(joint.body1))
        np.testing.assert_allclose(
            first.Transform(Gf.Vec3d(joint.localPose0Position)),
            second.Transform(Gf.Vec3d(joint.localPose1Position)),
            atol=1e-7,
            rtol=0,
        )


def _assert_no_robot_workcell_penetration(engine):
    """Reject initial robot penetration into a task prop or the tabletop."""

    model = engine.model
    prop_bodies = set(engine._prop_body_ids)
    workcell_bodies = set(prop_bodies)
    workcell_bodies.add(int(model.body("table").id))
    robot_bodies = set(range(1, model.nbody)) - workcell_bodies
    for contact in engine.data.contact:
        bodies = tuple(int(model.geom(int(geom)).bodyid[0]) for geom in contact.geom)
        if (bodies[0] in robot_bodies) != (bodies[1] in robot_bodies):
            assert contact.dist >= 0, (
                tuple(model.body(body).name for body in bodies),
                contact.dist,
            )


def _native_conformance(
    tmp_path, backend, robot, environment, render_quality="standard"
):
    _, config = documents(tmp_path, backend, robot, environment)
    config["render_quality"] = render_quality
    # Real RGB-D cameras have off-center principal points and unequal focal
    # lengths. Centered defaults conceal renderer convention mistakes.
    config["cameras"]["scene"]["intrinsics"].update(
        fx=174.0, fy=161.0, cx=109.0, cy=62.0
    )
    if config["cameras"]["scene"]["options"]["depth"]:
        config["cameras"]["scene"]["intrinsics"]["depth_scale_mm"] = 0.1
    p = profile(robot)
    engine = importlib.import_module(f"waddle_sdk.simulators.{backend}").Engine(
        config, tmp_path
    )

    def advance(seconds):
        for _ in range(round(seconds / config["timestep"])):
            engine.step()

    try:
        if environment == "drawer" and robot != "so101":
            # The reference view must expose the handle's front face, not just
            # its top/side silhouette behind the cabinet. Verify rendered RGB-D
            # against the physical front plane while the robot is at home.
            camera = config["cameras"]["scene"]
            t = np.array(camera["transform"])
            intr = camera["intrinsics"]
            point = np.linalg.inv(t) @ [0.491, 0.0, 0.14, 1.0]
            u = round(intr["fx"] * point[0] / point[2] + intr["cx"])
            v = round(intr["fy"] * point[1] / point[2] + intr["cy"])
            rgb, depth = engine.capture("scene")
            assert 0 <= u < rgb.shape[1] and 0 <= v < rgb.shape[0]
            z = depth[v, u] * intr["depth_scale_mm"] / 1000
            world = t @ [
                z * (u - intr["cx"]) / intr["fx"],
                z * (v - intr["cy"]) / intr["fy"],
                z,
                1.0,
            ]
            assert world[0] == pytest.approx(0.491, abs=0.004), world
            assert min(rgb[v, u]) > 100, rgb[v, u]
        if environment == "two_cubes":
            # A uniform 60 g, 50 mm cube has I = m * side**2 / 6. Inspect
            # imported native bodies: setting mass alone can leave the inertia
            # computed for the builder's default density behind.
            for index in (1, 2):
                if backend == "mujoco":
                    body = engine.model.body(f"cube_{index}")
                    mass, inertia = float(body.mass[0]), body.inertia
                elif backend == "sapien":
                    body = engine.props[index].find_component_by_type(
                        engine.sp.physx.PhysxRigidDynamicComponent
                    )
                    mass, inertia = body.mass, body.inertia
                else:
                    body = engine.props[index]
                    mass = float(body.get_mass())
                    inertia = np.diag(
                        body._rigid_prim_view.get_inertias()[0].reshape(3, 3)
                    )
                assert mass == pytest.approx(0.06)
                np.testing.assert_allclose(inertia, np.full(3, 0.000025), rtol=1e-5)
        if backend == "mujoco":
            # Reference scenes must start without the hand embedded in a prop.
            # Check native contacts, independently of the layout declarations.
            _assert_no_robot_workcell_penetration(engine)
            # Check the compiled engine model, not just the source URDF: the
            # exporter must retain COM and the complete non-diagonal inertia.
            for link in description(robot).links:
                body = engine.model.body(link.name)
                assert float(body.mass[0]) == pytest.approx(link.mass)
                # MuJoCo's canonical XML writer uses six significant digits.
                np.testing.assert_allclose(body.ipos, link.com, atol=1e-7)
                orientation = np.empty(9)
                engine.mj.mju_quat2Mat(orientation, body.iquat)
                r = orientation.reshape(3, 3)
                tensor = r @ np.diag(body.inertia) @ r.T
                xx, yy, zz, xy, xz, yz = link.inertia
                np.testing.assert_allclose(
                    tensor, [[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]], atol=1e-8
                )
        for q in (p.home, tuple(np.asarray(p.home) + np.r_[0.03, np.zeros(p.dof)])):
            engine.home(q)
            native_pos, native_rot = engine.native_tcp()
            expected = p.poses(q)[-1]
            np.testing.assert_allclose(native_pos, expected[:3, 3], atol=2e-6)
            np.testing.assert_allclose(native_rot, expected[:3, :3], atol=2e-6)
        target = np.array(p.home)
        target[0] += 0.1
        target[-1] = 0.4
        engine.write(target)
        advance(4.0)
        if backend == "mujoco":
            assert engine.data.time == pytest.approx(4.0)
        measured, velocity = engine.read()
        np.testing.assert_allclose(measured, target, atol=0.015)
        assert np.isfinite(velocity).all()
        # Known trajectory velocity reaches the native PD velocity target.
        # End the trajectory explicitly, as a position-only command does on hardware.
        feedforward = np.zeros_like(target)
        feedforward[0] = 0.25
        moving = target.copy()
        moving[0] += 0.01
        engine.write(moving)
        advance(0.02)
        ordinary = engine.read()[0][0]
        engine.home(target)
        engine.write(moving, feedforward)
        advance(0.02)
        assert engine.read()[0][0] > ordinary + 0.00005
        engine.write(moving)
        advance(0.8)
        assert engine.read()[0][0] == pytest.approx(moving[0], abs=0.001)
        engine.write(target)
        advance(0.8)
        assert engine.read()[0][0] == pytest.approx(target[0], abs=0.001)
        for name in config["cameras"]:
            rgb, depth = engine.capture(name)
            assert rgb.shape == (144, 192, 3) and rgb.dtype == np.uint8
            if robot == "so101":
                assert depth is None
            else:
                assert depth.shape == (144, 192) and depth.dtype == np.uint16
                assert np.isfinite(depth).all()
        # A known unoccluded tabletop pixel deprojects onto z=0. This detects
        # optical-axis, depth-range, principal-point, and scale errors together.
        camera = config["cameras"]["scene"]
        t = np.array(camera["transform"])
        intr = camera["intrinsics"]
        if robot != "so101":
            # The pocket-overhead view puts the old near-base witness behind
            # xArm's tall upper arm. Use clear tabletop beside the source tray.
            witness_x = 0.3 if environment == "chocolate-packing" else 0.1
            point = np.linalg.inv(t) @ np.array([witness_x, -0.32, 0.0, 1.0])
            u = round(intr["fx"] * point[0] / point[2] + intr["cx"])
            v = round(intr["fy"] * point[1] / point[2] + intr["cy"])
            _, depth = engine.capture("scene")
            z = depth[v, u] * intr["depth_scale_mm"] / 1000
            assert z > 0
            world = t @ [
                z * (u - intr["cx"]) / intr["fx"],
                z * (v - intr["cy"]) / intr["fy"],
                z,
                1.0,
            ]
            assert abs(world[2]) < 0.004
        if environment == "two_cubes":
            # Check RGB against an independent world-space witness too: the
            # top-center of the green cube must project into its green pixels.
            point = np.linalg.inv(t) @ [0.32, -0.10, 0.05, 1.0]
            u = round(intr["fx"] * point[0] / point[2] + intr["cx"])
            v = round(intr["fy"] * point[1] / point[2] + intr["cy"])
            rgb, _ = engine.capture("scene")
            red, green, blue = map(int, rgb[v, u])
            assert green > 2 * max(red, blue), (u, v, rgb[v, u])
        if environment == "drawer":
            # Test the passive slide with the arm clear of its swept volume.
            # A tabletop home can otherwise physically stop the opening drawer.
            clear = engine.read()[0].copy()
            clear[0] = np.pi / 2
            if robot == "so101":
                engine.home(clear)
            else:
                engine.write(clear)
                advance(2.0)
            assert engine.read()[0][0] == pytest.approx(clear[0], abs=0.015)
            if backend == "mujoco":
                joint = engine.model.joint("drawer_slide")

                def drawer_force(value):
                    engine.data.qfrc_applied[int(joint.dofadr[0])] = value

                def drawer_state():
                    return (
                        float(engine.data.qpos[int(joint.qposadr[0])]),
                        float(engine.data.qvel[int(joint.dofadr[0])]),
                    )
            elif backend == "sapien":

                def drawer_force(value):
                    engine.props[-1].set_qf(np.array([value]))

                def drawer_state():
                    return engine.props[-1].get_qpos()[0], engine.props[-1].get_qvel()[
                        0
                    ]
            else:

                def drawer_force(value):
                    engine.props[-1].set_joint_efforts(np.array([value]))

                def drawer_state():
                    return (
                        engine.props[-1].get_joint_positions()[0],
                        engine.props[-1].get_joint_velocities()[0],
                    )

            # A short pull must dissipate motion, not coast to a limit and
            # bounce closed indefinitely after the gripper has let go.
            drawer_force(0.5)
            advance(0.1)
            drawer_force(0.0)
            advance(0.5)
            resting_position, speed = drawer_state()
            assert resting_position > 0.001
            assert abs(speed) < 0.001
            advance(2.0)
            assert drawer_state()[0] == pytest.approx(resting_position, abs=0.0001)
            drawer_force(5.0)
            advance(2.0)
            travel, _speed = drawer_state()
            assert 0.20 < travel < 0.225, travel
        if backend == "mujoco" and environment in {
            "touch_target",
            "pick_lift",
            "place_in_bin",
            "push_to_region",
            "operate_control",
        }:
            assert engine.reset() is True
            _wave_a_prop_conformance(engine, advance, config, environment)
        if backend == "mujoco" and environment in {
            "select-distractors",
            "close-drawer",
            "stack-two-cubes",
            "ring-on-peg",
            "use-hook",
            "open-hinged-door",
        }:
            assert engine.reset() is True
            _wave_b_single_prop_conformance(engine, advance, config, environment)
        if backend == "mujoco" and environment in {
            "stack-three-cubes",
            "insert-peg",
            "retrieve-from-drawer",
            "store-in-drawer",
            "insert-usb",
            "load-clear-test-tubes",
            "candy-bin-transfer",
        }:
            assert engine.reset() is True
            _wave_c_single_prop_conformance(engine, advance, config, environment)
        assert engine.reset() is True
        np.testing.assert_allclose(engine.read()[0], p.home, atol=1e-6)
        if backend == "mujoco":
            assert engine.data.time == 0
            if robot == "yam":
                # In this folded wrist pose, a single convex hull fills a
                # forearm recess and creates contact 22 mm from the source
                # collision surface. Preserve that clearance in the importer.
                folded = np.array(
                    [-0.880189, 1.551672, 0.660299, 0.932776, -1.264037, 0.039450, 1.0]
                )
                engine.home(folded)
                for contact in engine.data.contact:
                    names = {
                        engine.model.body(int(engine.model.geom(int(g)).bodyid[0])).name
                        for g in contact.geom
                    }
                    if names == {"link_4", "gripper"}:
                        assert contact.dist > -0.0001
                engine.home(p.home)
        if environment == "drawer":
            travel = (
                float(engine.data.qpos[int(joint.qposadr[0])])
                if backend == "mujoco"
                else float(engine.props[-1].get_qpos()[0])
                if backend == "sapien"
                else float(engine.props[-1].get_joint_positions()[0])
            )
            assert travel == pytest.approx(0, abs=1e-6)
            if backend in {"mujoco", "sapien"} and robot == "yam":
                # A horizontal grasp around the fixed-height handle loads both
                # jaws. Mechanical coupling must preserve their agreement
                # and the vertical pinch midpoint under asymmetric load.
                grasp = np.array(
                    [
                        -0.00030024084,
                        1.95616480086,
                        0.74226724928,
                        1.21389020595,
                        -0.00029410866,
                        1.57080267467,
                        1.0,
                    ]
                )
                engine.home(grasp)
                expected = p.poses(grasp)[-1][:3, 3]
                grasp[-1] = 0.0
                engine.write(grasp)
                advance(2.0)
                if backend == "mujoco":
                    joints = (engine.model.joint(n) for n in ("joint7", "joint8"))
                    jaws = [float(engine.data.qpos[int(j.qposadr[0])]) for j in joints]
                else:
                    jaws = engine.robot.get_qpos()[engine.finger_indices]
                assert min(jaws) > 0.001  # physical handle stops closing
                assert abs(jaws[0] - jaws[1]) < 0.0005
                actual = engine.native_tcp()[0]
                if backend == "mujoco":
                    np.testing.assert_allclose(actual, expected, atol=0.001)
                else:
                    # The drawer is free along X; this coupling regression
                    # checks the vertical deflection from unequal jaw forces.
                    assert actual[2] == pytest.approx(expected[2], abs=0.001)
            elif backend == "mujoco" and robot == "xarm7":
                # Retain an ordinary handle grasp under load. This catches
                # gradual friction-cone creep despite ample normal force.
                grasp = np.array(
                    [
                        -0.18759,
                        0.22317,
                        0.17308,
                        0.38154,
                        -3.16054,
                        1.40853,
                        1.53572,
                        1.0,
                    ]
                )
                pulled = np.array(
                    [
                        -0.56731,
                        0.04854,
                        0.54599,
                        0.05310,
                        -3.16345,
                        1.55916,
                        1.54585,
                        0.0,
                    ]
                )
                engine.home(grasp)
                grasp[-1] = 0.0
                engine.write(grasp)
                advance(2.0)
                for fraction in np.linspace(0.01, 1.0, 100):
                    engine.write(grasp + fraction * (pulled - grasp))
                    advance(0.02)
                advance(1.0)
                opened, _ = drawer_state()
                assert opened > 0.09
                advance(5.0)
                assert drawer_state()[0] == pytest.approx(opened, abs=0.002)
        if (
            environment == "two_cubes"
            and backend in {"mujoco", "sapien"}
            and robot == "yam"
        ):
            # Approach the 50 mm cube through two tabletop grasp waypoints.
            # Jaw motion must settle under contact, not penetrate the cube or
            # oscillate enough to defeat ordinary measured-stall detection.
            waypoints = (
                [-0.271499, 1.758647, 1.410958, -1.140441, -0.000006, -1.675033, 1.0],
                [-0.303437, 1.849018, 1.325780, -1.032856, -0.000018, -1.872019, 1.0],
            )
            start = engine.read()[0]
            for values in waypoints:
                target = np.array(values)
                for fraction in np.linspace(0.01, 1.0, 100):
                    engine.write(start + fraction * (target - start))
                    advance(0.01)
                start = target
            # This fixture tests a stationary grasp. Allow the position servo
            # to finish the approach before the fingers close around the cube.
            advance(1.0)
            np.testing.assert_allclose(engine.read()[0][:-1], target[:-1], atol=0.003)
            target[-1] = 0.0
            engine.write(target)
            advance(2.0)
            opening = []
            for _ in range(500):
                engine.step()
                opening.append(engine.read()[0][-1] * p.opening)
            if backend == "mujoco":
                assert 0.049 < min(opening) <= max(opening) < 0.053
            else:
                # The cube can tilt between the manufacturer finger meshes.
                # Check actual load-bearing contacts on both fingers instead
                # of assuming its faces remain parallel to the closing axis.
                touching = set()
                for contact in engine.scene.get_contacts():
                    names = {body.name for body in contact.bodies}
                    if "cube_1" in names and any(
                        np.linalg.norm(point.impulse) > 1e-6 for point in contact.points
                    ):
                        touching.update(names - {"cube_1"})
                        assert (
                            min(point.separation for point in contact.points) > -0.001
                        )
                assert {"tip_left", "tip_right"} <= touching
            assert np.ptp(opening) < 0.0002
        if environment == "two_cubes" and backend == "sapien" and robot == "xarm7":
            # A closed reference hand needs local contact candidates, not the
            # >1,000 distant pairs produced by centimetre margins between its
            # convex pieces. Bound native work without a wall-clock race.
            engine.home(p.home)
            closed = np.array(p.home)
            closed[-1] = 0.0
            engine.write(closed)
            advance(2.0)
            assert len(engine.scene.get_contacts()) < 100
    finally:
        engine.close()


def _wave_a_prop_conformance(engine, advance, config, environment):
    """Probe task-object mechanics directly, without supplying a robot route."""

    model, data = engine.model, engine.data
    witnesses = {
        "touch_target": (
            ((0.392, 0.0, 0.16), "red"),
            ((0.392, -0.12, 0.16), "blue"),
            ((0.392, 0.12, 0.16), "blue"),
        ),
        "pick_lift": (((0.32, 0.0, 0.046), "green"),),
        "place_in_bin": (
            ((0.28, -0.12, 0.046), "orange"),
            ((0.42, 0.10, 0.012), "blue"),
        ),
        "push_to_region": (
            ((0.28, -0.10, 0.046), "yellow"),
            ((0.46, 0.10, 0.002), "blue"),
        ),
        "operate_control": (
            ((0.414, 0.0, 0.14), "red"),
            ((0.414, -0.105, 0.14), "blue"),
            ((0.414, 0.105, 0.14), "blue"),
        ),
    }
    classifiers = {
        "red": lambda r, g, b: r > 3 * max(g, b),
        "green": lambda r, g, b: g > 3 * max(r, b),
        "blue": lambda r, g, b: b > 2 * max(r, g),
        "orange": lambda r, g, b: r > 1.5 * g and g > 2 * b,
        "yellow": lambda r, g, b: r > 1.2 * g and g > 3 * b,
    }
    camera = config["cameras"]["scene"]
    world_from_camera = np.asarray(camera["transform"])
    intrinsics = camera["intrinsics"]
    rgb, _ = engine.capture("scene")
    for world_point, expected_color in witnesses[environment]:
        point = np.linalg.inv(world_from_camera) @ [*world_point, 1.0]
        u = round(intrinsics["fx"] * point[0] / point[2] + intrinsics["cx"])
        v = round(intrinsics["fy"] * point[1] / point[2] + intrinsics["cy"])
        assert 0 <= u < rgb.shape[1] and 0 <= v < rgb.shape[0]
        pixels = rgb[max(0, v - 2) : v + 3, max(0, u - 2) : u + 3]
        assert any(
            classifiers[expected_color](*map(int, pixel))
            for pixel in pixels.reshape(-1, 3)
        ), (world_point, expected_color, rgb[v, u])

    if environment == "touch_target":
        target = model.body("target_pad")
        distractor = model.body("distractor_pad_left")
        target_color = model.geom_rgba[int(target.geomadr[0])]
        distractor_color = model.geom_rgba[int(distractor.geomadr[0])]
        assert target_color[0] > 3 * target_color[2]
        assert distractor_color[2] > 3 * distractor_color[0]
        return

    if environment in {"pick_lift", "place_in_bin", "push_to_region"}:
        body = model.body("target_cube")
        body_id = int(body.id)
        if environment == "pick_lift":
            initial_z = float(data.xpos[body_id, 2])
            data.xfrc_applied[body_id, 2] = 1.0
            advance(0.1)
            data.xfrc_applied[body_id] = 0
            assert data.xpos[body_id, 2] > initial_z + 0.02
            return
        if environment == "place_in_bin":
            joint_id = int(body.jntadr[0])
            qpos = int(model.jnt_qposadr[joint_id])
            dof = int(model.jnt_dofadr[joint_id])
            data.qpos[qpos : qpos + 3] = (0.42, 0.10, 0.08)
            data.qvel[dof : dof + 6] = 0
            engine.mj.mj_forward(model, data)
            advance(1.0)
            np.testing.assert_allclose(data.xpos[body_id, :2], (0.42, 0.10), atol=0.01)
            assert 0.03 < data.xpos[body_id, 2] < 0.05
            return
        initial_xy = data.xpos[body_id, :2].copy()
        data.xfrc_applied[body_id, :2] = (0.6, 0.6)
        advance(0.1)
        data.xfrc_applied[body_id] = 0
        assert np.linalg.norm(data.xpos[body_id, :2] - initial_xy) > 0.005
        return

    assert environment == "operate_control"
    target = model.joint("target_control")
    target_qpos, target_dof = int(target.qposadr[0]), int(target.dofadr[0])
    distractors = [
        model.joint(name)
        for name in (
            "distractor_control_left",
            "distractor_control_right",
        )
    ]
    assert abs(data.qpos[target_qpos]) < 1e-5
    data.qfrc_applied[target_dof] = 1.0
    advance(0.1)
    data.qfrc_applied[target_dof] = 0
    assert data.qpos[target_qpos] > 0.008
    assert all(abs(data.qpos[int(joint.qposadr[0])]) < 1e-5 for joint in distractors)


def _wave_b_single_prop_conformance(engine, advance, config, environment):
    """Probe medium single-arm task mechanics without supplying a robot route."""

    model, data = engine.model, engine.data
    witnesses = {
        "select-distractors": (
            ((0.26, -0.14, 0.046), "orange"),
            ((0.34, -0.03, 0.046), "green"),
            ((0.25, 0.09, 0.046), "purple"),
            ((0.43, 0.13, 0.012), "blue"),
        ),
        "close-drawer": (
            (
                (0.313 if config["robot"] == "so101" else 0.353, 0.0, 0.14),
                "bright",
            ),
        ),
        "stack-two-cubes": (
            ((0.27, -0.10, 0.046), "green"),
            ((0.35, 0.08, 0.046), "blue"),
        ),
        "ring-on-peg": (
            ((0.313, -0.10, 0.015), "orange"),
            ((0.43, 0.10, 0.10), "blue"),
        ),
        "use-hook": (
            ((0.27, -0.13, 0.016), "orange"),
            ((0.44, 0.04, 0.03), "green"),
            ((0.28, 0.15, 0.002), "blue"),
        ),
        "open-hinged-door": (
            ((0.421, -0.01, 0.165), "blue"),
            ((0.358, 0.055, 0.14), "orange"),
        ),
    }
    classifiers = {
        "green": lambda r, g, b: g > 2 * max(r, b),
        "blue": lambda r, g, b: b > 1.5 * max(r, g),
        "orange": lambda r, g, b: r > 1.4 * g and g > 1.5 * b,
        "purple": lambda r, g, b: r > 1.5 * g and b > 1.5 * g,
        "bright": lambda r, g, b: min(r, g, b) > 100,
    }
    camera = config["cameras"]["scene"]
    world_from_camera = np.asarray(camera["transform"])
    intrinsics = camera["intrinsics"]
    rgb, _ = engine.capture("scene")
    for world_point, expected_color in witnesses[environment]:
        point = np.linalg.inv(world_from_camera) @ [*world_point, 1.0]
        u = round(intrinsics["fx"] * point[0] / point[2] + intrinsics["cx"])
        v = round(intrinsics["fy"] * point[1] / point[2] + intrinsics["cy"])
        assert 0 <= u < rgb.shape[1] and 0 <= v < rgb.shape[0]
        pixels = rgb[max(0, v - 3) : v + 4, max(0, u - 3) : u + 4]
        assert any(
            classifiers[expected_color](*map(int, pixel))
            for pixel in pixels.reshape(-1, 3)
        ), (world_point, expected_color, rgb[v, u])

    def place_free(name, xyz):
        body = model.body(name)
        joint_id = int(body.jntadr[0])
        assert model.jnt_type[joint_id] == engine.mj.mjtJoint.mjJNT_FREE
        qpos = int(model.jnt_qposadr[joint_id])
        dof = int(model.jnt_dofadr[joint_id])
        data.qpos[qpos : qpos + 7] = (*xyz, 1.0, 0.0, 0.0, 0.0)
        data.qvel[dof : dof + 6] = 0.0
        engine.mj.mj_forward(model, data)
        return int(body.id)

    if environment == "select-distractors":
        target = place_free("target_object", (0.43, 0.13, 0.08))
        initial_distractors = {
            name: data.body(name).xpos.copy()
            for name in ("distractor_object_1", "distractor_object_2")
        }
        advance(1.0)
        np.testing.assert_allclose(data.xpos[target, :2], (0.43, 0.13), atol=0.01)
        assert 0.03 < data.xpos[target, 2] < 0.05
        for name, initial in initial_distractors.items():
            np.testing.assert_allclose(data.body(name).xpos, initial, atol=0.003)
        return

    if environment == "close-drawer":
        clear = engine.read()[0].copy()
        clear[0] = np.pi / 2
        engine.home(clear)
        joint = model.joint("drawer_slide")
        qpos, dof = int(joint.qposadr[0]), int(joint.dofadr[0])
        assert data.qpos[qpos] == pytest.approx(0.15)
        data.qfrc_applied[dof] = -5.0
        advance(1.0)
        data.qfrc_applied[dof] = 0.0
        advance(0.3)
        assert data.qpos[qpos] < 0.005
        assert abs(data.qvel[dof]) < 0.001
        assert engine.reset() is True
        assert data.qpos[qpos] == pytest.approx(0.15)
        return

    if environment == "stack-two-cubes":
        bottom = place_free("cube_bottom", (0.31, 0.0, 0.023))
        top = place_free("cube_top", (0.31, 0.0, 0.09))
        advance(1.0)
        assert data.xpos[top, 2] - data.xpos[bottom, 2] == pytest.approx(
            0.046, abs=0.004
        )
        assert np.linalg.norm(data.xpos[top, :2] - data.xpos[bottom, :2]) < 0.008
        return

    if environment == "ring-on-peg":
        ring = place_free("target_ring", (0.43, 0.10, 0.13))
        advance(1.5)
        np.testing.assert_allclose(data.xpos[ring, :2], (0.43, 0.10), atol=0.006)
        assert data.xpos[ring, 2] < 0.025
        return

    if environment == "open-hinged-door":
        clear = engine.read()[0].copy()
        clear[0] = np.pi / 2
        engine.home(clear)
        hinge = model.joint("door_hinge")
        lever = model.joint("door_lever")
        latch = model.joint("door_latch")
        hinge_qpos, hinge_dof = int(hinge.qposadr[0]), int(hinge.dofadr[0])
        lever_qpos, lever_dof = int(lever.qposadr[0]), int(lever.dofadr[0])
        latch_qpos = int(latch.qposadr[0])
        assert not any(
            {
                model.body(int(model.geom(int(geom)).bodyid[0])).name
                for geom in contact.geom
            }
            == {"door_frame", "latch_bolt"}
            for contact in data.contact
        )

        # The extended bolt reaches the fixed strike and stops the same opening
        # load that moves the door after the lever physically retracts it.
        data.qfrc_applied[hinge_dof] = 2.0
        advance(1.0)
        assert 0.02 < data.qpos[hinge_qpos] < 0.05
        assert data.qpos[latch_qpos] < 0.002

        assert engine.reset() is True
        engine.home(clear)
        data.qfrc_applied[lever_dof] = 0.08
        advance(1.5)
        assert data.qpos[lever_qpos] > 1.0
        assert data.qpos[latch_qpos] > 0.055
        data.qfrc_applied[hinge_dof] = 2.0
        advance(1.5)
        data.qfrc_applied[:] = 0.0
        advance(0.5)
        assert data.qpos[hinge_qpos] > 0.8
        assert abs(data.qvel[hinge_dof]) < 0.05

        assert engine.reset() is True
        engine.home(clear)
        np.testing.assert_allclose(
            data.qpos[[hinge_qpos, lever_qpos, latch_qpos]], 0.0, atol=1e-8
        )
        return

    assert environment == "use-hook"
    hook = place_free("hook", (0.40, 0.014, 0.009))
    target = place_free("target_object", (0.44, 0.04, 0.018))
    contacted = False
    data.xfrc_applied[hook, 0] = -1.0
    initial_target_x = float(data.xpos[target, 0])
    for _ in range(round(0.5 / config["timestep"])):
        engine.step()
        for contact in data.contact:
            names = {
                model.body(int(model.geom(int(geom)).bodyid[0])).name
                for geom in contact.geom
            }
            contacted |= names == {"hook", "target_object"}
    data.xfrc_applied[hook] = 0.0
    assert contacted
    assert data.xpos[target, 0] < initial_target_x - 0.01


def _wave_c_single_prop_conformance(engine, advance, config, environment):
    """Probe hard single-arm mechanics without adding routes to the engine."""

    model, data = engine.model, engine.data
    witnesses = {
        "stack-three-cubes": (
            ((0.25, -0.13, 0.046), "green"),
            ((0.35, -0.01, 0.046), "blue"),
            ((0.27, 0.13, 0.046), "orange"),
        ),
        "insert-peg": (
            ((0.27, -0.12, 0.018), "orange"),
            ((0.423, 0.10, 0.043), "blue"),
        ),
        "retrieve-from-drawer": (
            (
                (
                    0.463 if config["robot"] == "so101" else 0.503,
                    0.0,
                    0.14,
                ),
                "bright",
            ),
            ((0.30, 0.18, 0.002), "blue"),
        ),
        "store-in-drawer": (
            ((0.29, -0.15, 0.04), "orange"),
            (
                (
                    0.313 if config["robot"] == "so101" else 0.353,
                    0.0,
                    0.14,
                ),
                "bright",
            ),
        ),
        "insert-usb": (
            ((0.25, -0.12, 0.018), "orange"),
            ((0.40, 0.10, 0.16), "dark"),
        ),
        "load-clear-test-tubes": (
            ((0.25, -0.18, 0.018), "cyan"),
            ((0.30, -0.07, 0.018), "cyan"),
            ((0.24, 0.05, 0.018), "cyan"),
            ((0.25, 0.16, 0.018), "cyan"),
            ((0.39, 0.19, 0.023), "blue"),
        ),
        "candy-bin-transfer": (
            ((0.229, -0.248, 0.022), "orange"),
            ((0.316, -0.244, 0.022), "orange"),
            ((0.361, -0.168, 0.022), "orange"),
            ((0.42, -0.03, 0.07), "blue"),
        ),
    }
    classifiers = {
        "green": lambda r, g, b: g > 2 * max(r, b),
        "blue": lambda r, g, b: b > 1.5 * max(r, g),
        "orange": lambda r, g, b: r > 1.4 * g and g > 1.5 * b,
        "bright": lambda r, g, b: min(r, g, b) > 100,
        "dark": lambda r, g, b: max(r, g, b) < 100,
        "cyan": lambda r, g, b: b > 1.04 * r and g > 1.03 * r,
        "red": lambda r, g, b: r > 2 * max(g, b),
        "purple": lambda r, g, b: r > 1.4 * g and b > 1.4 * g,
        "yellow": lambda r, g, b: min(r, g) > 2 * b,
    }
    camera = config["cameras"]["scene"]
    world_from_camera = np.asarray(camera["transform"])
    intrinsics = camera["intrinsics"]

    def assert_visible(rgb, point, expected_color):
        camera_point = np.linalg.inv(world_from_camera) @ [*point, 1.0]
        u = round(
            intrinsics["fx"] * camera_point[0] / camera_point[2] + intrinsics["cx"]
        )
        v = round(
            intrinsics["fy"] * camera_point[1] / camera_point[2] + intrinsics["cy"]
        )
        assert 0 <= u < rgb.shape[1] and 0 <= v < rgb.shape[0]
        pixels = rgb[max(0, v - 4) : v + 5, max(0, u - 4) : u + 5]
        assert any(
            classifiers[expected_color](*map(int, pixel))
            for pixel in pixels.reshape(-1, 3)
        ), (point, expected_color, rgb[v, u])

    rgb, _ = engine.capture("scene")
    for point, expected_color in witnesses[environment]:
        assert_visible(rgb, point, expected_color)

    def place_free(name, xyz, quaternion=(1.0, 0.0, 0.0, 0.0)):
        body = model.body(name)
        joint_id = int(body.jntadr[0])
        assert model.jnt_type[joint_id] == engine.mj.mjtJoint.mjJNT_FREE
        qpos = int(model.jnt_qposadr[joint_id])
        dof = int(model.jnt_dofadr[joint_id])
        data.qpos[qpos : qpos + 7] = (*xyz, *quaternion)
        data.qvel[dof : dof + 6] = 0.0
        engine.mj.mj_forward(model, data)
        return int(body.id)

    if environment == "stack-three-cubes":
        names = ("cube_bottom", "cube_middle", "cube_top")
        for name, z in zip(names, (0.023, 0.085, 0.147), strict=True):
            place_free(name, (0.31, 0.0, z))
        advance(1.5)
        positions = [data.body(name).xpos.copy() for name in names]
        for lower, upper in zip(positions[:-1], positions[1:], strict=True):
            assert upper[2] - lower[2] == pytest.approx(0.046, abs=0.004)
            assert np.linalg.norm(upper[:2] - lower[:2]) < 0.008
        return

    if environment == "insert-peg":
        peg_start = data.body("target_peg").xpos.copy()
        advance(6.0)
        np.testing.assert_allclose(
            data.body("target_peg").xpos[:2], peg_start[:2], atol=0.004
        )
        assert np.linalg.norm(data.body("target_peg").cvel[3:]) < 0.01
        assert engine.reset() is True
        peg = place_free("target_peg", (0.40, 0.10, 0.12))
        advance(1.5)
        np.testing.assert_allclose(data.xpos[peg, :2], (0.40, 0.10), atol=0.004)
        assert data.xpos[peg, 2] < 0.055
        assert data.body("target_peg").xmat.reshape(3, 3)[2, 2] > 0.995

        assert engine.reset() is True
        peg = place_free("target_peg", (0.42, 0.10, 0.12))
        advance(1.5)
        assert data.xpos[peg, 2] > 0.075
        return

    if environment == "retrieve-from-drawer":
        drawer = model.joint("drawer_slide")
        drawer_dof = int(drawer.dofadr[0])
        data.qfrc_applied[drawer_dof] = 5.0
        advance(1.0)
        data.qfrc_applied[drawer_dof] = 0.0
        assert data.joint("drawer_slide").qpos[0] > 0.09
        assert data.body("target_object").xpos[0] < 0.55
        opened_target = data.body("target_object").xpos.copy()
        rgb, _ = engine.capture("scene")
        assert_visible(rgb, (*opened_target[:2], opened_target[2] + 0.0175), "orange")

        target = place_free("target_object", (0.30, 0.18, 0.06))
        advance(1.0)
        np.testing.assert_allclose(data.xpos[target, :2], (0.30, 0.18), atol=0.005)
        assert 0.012 < data.xpos[target, 2] < 0.023
        return

    if environment == "insert-usb":
        connector = place_free("usb_connector", (0.35, 0.10, 0.12))
        data.xfrc_applied[connector, 0] = 3.0
        max_normal_force = 0.0
        for _ in range(round(1.0 / config["timestep"])):
            engine.step()
            for index in range(data.ncon):
                force = np.zeros(6)
                engine.mj.mj_contactForce(model, data, index, force)
                max_normal_force = max(max_normal_force, float(force[0]))
        data.xfrc_applied[connector] = 0.0
        correct_x = float(data.xpos[connector, 0])
        assert correct_x > 0.404
        assert data.body("usb_connector").xmat.reshape(3, 3)[0, 0] > 0.995
        assert max_normal_force < 15.0

        assert engine.reset() is True
        connector = place_free(
            "usb_connector", (0.35, 0.10, 0.12), (0.0, 1.0, 0.0, 0.0)
        )
        data.xfrc_applied[connector, 0] = 3.0
        advance(1.0)
        data.xfrc_applied[connector] = 0.0
        assert data.xpos[connector, 0] < 0.402
        assert correct_x - data.xpos[connector, 0] > 0.005
        return

    if environment == "load-clear-test-tubes":
        tube_names = tuple(f"clear_test_tube_{index}" for index in range(1, 5))
        tube_starts = {}
        for name in tube_names:
            axis = data.body(name).xmat.reshape(3, 3)[:, 2]
            assert abs(axis[2]) < 0.05
            tube_starts[name] = data.body(name).xpos.copy()
        advance(6.0)
        for name in tube_names:
            np.testing.assert_allclose(
                data.body(name).xpos[:2], tube_starts[name][:2], atol=0.004
            )
            assert np.linalg.norm(data.body(name).cvel[3:]) < 0.01
        assert engine.reset() is True
        slots = (
            (0.35, 0.14),
            (0.43, 0.14),
            (0.35, 0.06),
            (0.43, 0.06),
        )
        for name, (x, y) in zip(tube_names, slots, strict=True):
            place_free(name, (x, y, 0.12))
            advance(1.0)
        for name, (x, y) in zip(tube_names, slots, strict=True):
            np.testing.assert_allclose(data.body(name).xpos[:2], (x, y), atol=0.004)
            assert data.body(name).xpos[2] == pytest.approx(0.06, abs=0.004)
            axis = data.body(name).xmat.reshape(3, 3)[:, 2]
            assert axis[2] > 0.985
        return

    if environment == "candy-bin-transfer":
        advance(3.0)
        for index in range(1, 19):
            candy_body = data.body(f"candy_{index}")
            position = candy_body.xpos
            assert abs(position[0] - 0.30) <= 0.074
            assert abs(position[1] + 0.20) <= 0.055
            assert np.linalg.norm(candy_body.cvel[3:]) < 0.01
        assert engine.reset() is True
        candy = place_free("candy_1", (0.42, -0.03, 0.08))
        advance(1.5)
        np.testing.assert_allclose(data.xpos[candy, :2], (0.42, -0.03), atol=0.004)
        assert data.xpos[candy, 2] == pytest.approx(0.022, abs=0.004)

        # A centered physical pinch must carry the small rigid body upward.
        # This probes robot/candy contacts rather than inferring mechanics from
        # gripper closure or an external force applied directly to the prop.
        assert engine.reset() is True
        robot = config["robot"]
        grasp, lift = {
            "so101": (
                (
                    0.6111116363,
                    0.2058768679,
                    0.2887797362,
                    -0.47794756798,
                    -1.6836794621,
                    0.15,
                ),
                (
                    0.6111028479,
                    0.0851971464,
                    -0.1595262347,
                    0.0910381109,
                    -1.6865021762,
                    0.0,
                ),
            ),
            "yam": (
                (
                    -0.5073011758,
                    1.9527587169,
                    1.3864839788,
                    -1.0045904466,
                    -0.0003258847,
                    0.0162585578,
                    0.3999989982,
                ),
                (
                    -0.5073832518,
                    1.9063926729,
                    1.5698738174,
                    -1.2342777578,
                    -0.00000725,
                    0.0162300983,
                    0.0,
                ),
            ),
            "xarm7": (
                (
                    0.3545473497,
                    0.0280239344,
                    -0.7897804566,
                    0.5556825795,
                    0.0389480318,
                    0.5362834975,
                    -3.0865124481,
                    0.25,
                ),
                (
                    0.3118233049,
                    -0.3061679102,
                    -0.7296058080,
                    0.6683255177,
                    -0.2561315532,
                    0.9158303896,
                    -2.85398900596,
                    0.0,
                ),
            ),
        }[robot]
        grasp, lift = np.asarray(grasp), np.asarray(lift)
        engine.home(grasp)
        p = profile(robot)
        tcp = p.poses(grasp)[-1]
        center = tcp[:3, 3] + tcp[:3, :3] @ np.asarray(p.pinch_offset)
        if robot == "xarm7":
            # Its TCP is at the fingertip plane; load the long pads above it.
            center += (0.0, 0.0, 0.03)
        center[2] = max(center[2], 0.022)
        closing = tcp[:3, :3] @ np.asarray(p.closing_axis)
        closing /= np.linalg.norm(closing)
        vertical = np.asarray((0.0, 0.0, 1.0))
        lengthwise = np.cross(closing, vertical)
        lengthwise /= np.linalg.norm(lengthwise)
        vertical = np.cross(lengthwise, closing)
        candy = place_free(
            "candy_1",
            center,
            quaternion(np.column_stack((lengthwise, closing, vertical))),
        )
        initial_z = float(data.xpos[candy, 2])
        closed = grasp.copy()
        closed[-1] = 0.0
        engine.write(closed)
        advance(1.5)
        maximum_z = float(data.xpos[candy, 2])
        for fraction in np.linspace(0.01, 1.0, 150):
            engine.write(closed + fraction * (lift - closed))
            for _ in range(round(0.01 / config["timestep"])):
                engine.step()
                maximum_z = max(maximum_z, float(data.xpos[candy, 2]))
        advance(0.5)
        assert maximum_z > initial_z + 0.045
        lifted_tcp = p.poses(lift)[-1]
        retained_center = lifted_tcp[:3, 3] + lifted_tcp[:3, :3] @ np.asarray(
            p.pinch_offset
        )
        if robot == "xarm7":
            retained_center += (0.0, 0.0, 0.03)
        assert np.linalg.norm(data.xpos[candy] - retained_center) < 0.04
        assert data.xpos[candy, 2] > initial_z + 0.04
        fingers = {
            "so101": {"gripper_link", "moving_jaw_so101_v1_link"},
            "yam": {"tip_left", "tip_right"},
            "xarm7": {"left_finger", "right_finger"},
        }[robot]
        touched = set()
        for event in engine.evaluation_snapshot()["contact_events"]["pairs"]:
            bodies = {event["first"]["body"], event["second"]["body"]}
            if "candy_1" in bodies and event["maximum_normal_force_n"] > 0:
                touched.update(bodies & fingers)
        assert touched == fingers
        current_touch = set()
        for contact in data.contact:
            bodies = {
                model.body(int(model.geom(int(geom)).bodyid[0])).name
                for geom in contact.geom
            }
            if "candy_1" in bodies:
                current_touch.update(bodies & fingers)
        assert current_touch == fingers
        return

    assert environment == "store-in-drawer"
    assert data.joint("drawer_slide").qpos[0] == pytest.approx(0.15)
    interior = data.body("drawer_interior").xpos.copy()
    target = place_free("target_object", (*interior[:2], 0.10))
    advance(1.0)
    interior = data.body("drawer_interior").xpos.copy()
    assert np.linalg.norm(data.xpos[target, :2] - interior[:2]) < 0.03
    drawer = model.joint("drawer_slide")
    drawer_dof = int(drawer.dofadr[0])
    data.qfrc_applied[drawer_dof] = -5.0
    advance(1.0)
    data.qfrc_applied[drawer_dof] = 0.0
    assert abs(data.joint("drawer_slide").qpos[0]) < 0.005
    interior = data.body("drawer_interior").xpos
    assert abs(data.xpos[target, 0] - interior[0]) < 0.10
    assert abs(data.xpos[target, 1] - interior[1]) < 0.10


@pytest.mark.parametrize("robot", ROBOTS)
def test_native_split_workspace_sorting_scene(tmp_path, monkeypatch, robot):
    mujoco = pytest.importorskip("mujoco")
    monkeypatch.setenv("MUJOCO_GL", "egl")
    from waddle_sdk.simulators.mujoco import Engine

    _, config = documents(
        tmp_path,
        "mujoco",
        robot,
        "split_workspace_sorting",
        arms=2,
    )
    engine = Engine(config, tmp_path)
    model, data = engine.model, engine.data

    def advance(seconds):
        for _ in range(round(seconds / config["timestep"])):
            engine.step()

    def place(name, xyz):
        body = model.body(name)
        joint_id = int(body.jntadr[0])
        assert model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE
        qpos = int(model.jnt_qposadr[joint_id])
        dof = int(model.jnt_dofadr[joint_id])
        data.qpos[qpos : qpos + 7] = (*xyz, 1.0, 0.0, 0.0, 0.0)
        data.qvel[dof : dof + 6] = 0

    try:
        _assert_no_robot_workcell_penetration(engine)
        assert np.isfinite(engine.read("left")[0]).all()
        assert np.isfinite(engine.read("right")[0]).all()
        for contact in data.contact:
            bodies = [
                model.body(int(model.geom(int(geom)).bodyid[0])).name
                for geom in contact.geom
            ]
            assert not (
                bodies[0].startswith("left__")
                and bodies[1].startswith("right__")
                or bodies[0].startswith("right__")
                and bodies[1].startswith("left__")
            ), bodies

        camera = config["cameras"]["scene"]
        world_from_camera = np.asarray(camera["transform"])
        intrinsics = camera["intrinsics"]
        rgb, _ = engine.capture("scene")
        for point, classify in (
            (
                (0.26, -0.44, 0.04),
                lambda red, green, blue: red > 1.5 * green and green > 2 * blue,
            ),
            (
                (0.26, 0.44, 0.04),
                lambda red, green, blue: green > 3 * max(red, blue),
            ),
        ):
            camera_point = np.linalg.inv(world_from_camera) @ [*point, 1.0]
            u = round(
                intrinsics["fx"] * camera_point[0] / camera_point[2] + intrinsics["cx"]
            )
            v = round(
                intrinsics["fy"] * camera_point[1] / camera_point[2] + intrinsics["cy"]
            )
            assert 0 <= u < rgb.shape[1] and 0 <= v < rgb.shape[0]
            pixels = rgb[max(0, v - 2) : v + 3, max(0, u - 2) : u + 3]
            assert any(classify(*map(int, pixel)) for pixel in pixels.reshape(-1, 3)), (
                point,
                rgb[v, u],
            )

        place("left_object", (0.32, 0.065, 0.06))
        place("right_object", (0.32, -0.065, 0.06))
        mujoco.mj_forward(model, data)
        advance(1.0)
        np.testing.assert_allclose(
            data.xpos[int(model.body("left_object").id), :2],
            (0.32, 0.065),
            atol=0.005,
        )
        np.testing.assert_allclose(
            data.xpos[int(model.body("right_object").id), :2],
            (0.32, -0.065),
            atol=0.005,
        )
        for name in ("left_object", "right_object"):
            assert 0.025 < data.xpos[int(model.body(name).id), 2] < 0.04
        assert engine.evaluation_snapshot()["identity"]["arm_count"] == 2

        assert engine.evaluation_reset(seed=7)
        for name, canonical in (
            ("left_object", (0.26, -0.44, 0.02)),
            ("right_object", (0.26, 0.44, 0.02)),
        ):
            randomized = data.xpos[int(model.body(name).id)]
            assert np.all(np.abs(randomized[:2] - canonical[:2]) <= 0.012)
            assert randomized[2] == pytest.approx(canonical[2])
    finally:
        engine.close()


@pytest.mark.parametrize("robot", ROBOTS)
@pytest.mark.parametrize(
    "environment",
    DUAL_ARM_MEDIUM_TASK_ENVIRONMENTS,
)
def test_native_medium_dual_arm_task_scenes(tmp_path, monkeypatch, robot, environment):
    mujoco = pytest.importorskip("mujoco")
    monkeypatch.setenv("MUJOCO_GL", "egl")
    from waddle_sdk.simulators.mujoco import Engine

    _, config = documents(
        tmp_path,
        "mujoco",
        robot,
        environment,
        arms=2,
    )
    engine = Engine(config, tmp_path)
    model, data = engine.model, engine.data

    def advance(seconds):
        for _ in range(round(seconds / config["timestep"])):
            engine.step()

    def place_free(name, xyz):
        body = model.body(name)
        joint_id = int(body.jntadr[0])
        assert model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE
        qpos = int(model.jnt_qposadr[joint_id])
        dof = int(model.jnt_dofadr[joint_id])
        data.qpos[qpos : qpos + 7] = (*xyz, 1.0, 0.0, 0.0, 0.0)
        data.qvel[dof : dof + 6] = 0.0
        mujoco.mj_forward(model, data)
        return int(body.id)

    witnesses = {
        "handover-block": (
            ((0.28, -0.30, 0.046), "orange"),
            ((0.32, 0.30, 0.002), "blue"),
        ),
        "stabilize-open-drawer": (
            (
                (0.463 if robot == "so101" else 0.503, 0.0, 0.14),
                "bright",
            ),
        ),
        "hold-container-place": (
            ((0.30, 0.20, 0.046), "orange"),
            ((0.38, -0.035, 0.06), "blue"),
        ),
        "stabilize-remove-lid": (
            ((0.36, -0.08, 0.115), "orange"),
            ((0.36, 0.25, 0.002), "blue"),
        ),
    }
    classifiers = {
        "blue": lambda r, g, b: b > 1.5 * max(r, g),
        "orange": lambda r, g, b: r > 1.4 * g and g > 1.5 * b,
        "bright": lambda r, g, b: min(r, g, b) > 100,
    }

    try:
        _assert_no_robot_workcell_penetration(engine)
        assert np.isfinite(engine.read("left")[0]).all()
        assert np.isfinite(engine.read("right")[0]).all()
        for contact in data.contact:
            first, second = (
                model.body(int(model.geom(int(geom)).bodyid[0])).name
                for geom in contact.geom
            )
            assert not (
                first.startswith("left__")
                and second.startswith("right__")
                or first.startswith("right__")
                and second.startswith("left__")
            ), (first, second)

        camera = config["cameras"]["scene"]
        world_from_camera = np.asarray(camera["transform"])
        intrinsics = camera["intrinsics"]
        rgb, _ = engine.capture("scene")
        for point, expected_color in witnesses[environment]:
            camera_point = np.linalg.inv(world_from_camera) @ [*point, 1.0]
            u = round(
                intrinsics["fx"] * camera_point[0] / camera_point[2] + intrinsics["cx"]
            )
            v = round(
                intrinsics["fy"] * camera_point[1] / camera_point[2] + intrinsics["cy"]
            )
            assert 0 <= u < rgb.shape[1] and 0 <= v < rgb.shape[0]
            pixels = rgb[max(0, v - 3) : v + 4, max(0, u - 3) : u + 4]
            assert any(
                classifiers[expected_color](*map(int, pixel))
                for pixel in pixels.reshape(-1, 3)
            ), (point, expected_color, rgb[v, u])
        assert engine.evaluation_snapshot()["identity"]["arm_count"] == 2

        if environment == "handover-block":
            block = place_free("handover_block", (0.32, 0.30, 0.08))
            advance(1.0)
            np.testing.assert_allclose(data.xpos[block, :2], (0.32, 0.30), atol=0.005)
            assert 0.02 < data.xpos[block, 2] < 0.03
        elif environment == "stabilize-open-drawer":
            cabinet = model.body("movable_cabinet")
            drawer = model.body("drawer")
            assert model.jnt_type[int(cabinet.jntadr[0])] == mujoco.mjtJoint.mjJNT_FREE
            initial = data.body("movable_cabinet").xpos.copy()
            data.xfrc_applied[int(drawer.id), 0] = -8.0
            advance(1.0)
            assert data.joint("drawer_slide").qpos[0] > 0.09
            assert (
                np.linalg.norm(data.body("movable_cabinet").xpos[:2] - initial[:2])
                > 0.01
            )
        elif environment == "hold-container-place":
            container = model.body("movable_container")
            assert (
                model.jnt_type[int(container.jntadr[0])] == mujoco.mjtJoint.mjJNT_FREE
            )
            initial = data.body("movable_container").xpos.copy()
            data.xfrc_applied[int(container.id), 1] = 3.0
            advance(0.5)
            assert data.body("movable_container").xpos[1] > initial[1] + 0.01
            assert engine.reset() is True
            target = place_free("target_object", (0.38, -0.12, 0.12))
            advance(1.0)
            np.testing.assert_allclose(data.xpos[target, :2], (0.38, -0.12), atol=0.005)
            assert 0.025 < data.xpos[target, 2] < 0.04
        else:
            assert environment == "stabilize-remove-lid"
            box = model.body("movable_box")
            lid = model.body("box_lid")
            advance(0.5)
            for name in ("lid_grip_left", "lid_grip_right"):
                assert 0.003 < data.joint(name).qpos[0] < 0.006

            initial_box_z = float(data.body("movable_box").xpos[2])
            data.xfrc_applied[int(lid.id), 2] = 5.0
            advance(0.5)
            assert data.body("movable_box").xpos[2] > initial_box_z + 0.02

            assert engine.reset() is True
            initial_separation = float(
                data.body("box_lid").xpos[2] - data.body("movable_box").xpos[2]
            )
            data.xfrc_applied[int(box.id), 2] = -10.0
            data.xfrc_applied[int(lid.id), 2] = 6.0
            for _ in range(round(0.5 / config["timestep"])):
                engine.step()
                if (
                    data.body("box_lid").xpos[2] - data.body("movable_box").xpos[2]
                    > initial_separation + 0.05
                ):
                    break
            data.xfrc_applied[:] = 0.0
            assert (
                data.body("box_lid").xpos[2] - data.body("movable_box").xpos[2]
                > initial_separation + 0.05
            )
            lid_id = place_free("box_lid", (0.36, 0.25, 0.08))
            advance(1.0)
            np.testing.assert_allclose(data.xpos[lid_id, :2], (0.36, 0.25), atol=0.006)
            assert 0.025 < data.xpos[lid_id, 2] < 0.035

        assert engine.evaluation_reset(seed=7)
    finally:
        engine.close()


@pytest.mark.parametrize("robot", ROBOTS)
@pytest.mark.parametrize(
    "environment",
    DUAL_ARM_HARD_TASK_ENVIRONMENTS,
)
def test_native_hard_dual_arm_task_scenes(tmp_path, monkeypatch, robot, environment):
    mujoco = pytest.importorskip("mujoco")
    monkeypatch.setenv("MUJOCO_GL", "egl")
    from waddle_sdk.simulators.mujoco import Engine

    _, config = documents(
        tmp_path,
        "mujoco",
        robot,
        environment,
        arms=2,
    )
    engine = Engine(config, tmp_path)
    model, data = engine.model, engine.data

    def advance(seconds):
        for _ in range(round(seconds / config["timestep"])):
            engine.step()

    def place_free(name, xyz, quaternion=(1.0, 0.0, 0.0, 0.0)):
        body = model.body(name)
        joint_id = int(body.jntadr[0])
        assert model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE
        qpos = int(model.jnt_qposadr[joint_id])
        dof = int(model.jnt_dofadr[joint_id])
        data.qpos[qpos : qpos + 7] = (*xyz, *quaternion)
        data.qvel[dof : dof + 6] = 0.0
        mujoco.mj_forward(model, data)
        return int(body.id)

    def park_arms_outward():
        for part, angle in (("left", -1.2), ("right", 1.2)):
            parked = np.asarray(profile(robot).home).copy()
            parked[0] = angle
            engine.home(part, parked)

    witnesses = {
        "oriented-tool-handover": (
            ((0.25, -0.30, 0.025), "orange"),
            ((0.32, 0.30, 0.002), "blue"),
        ),
        "two-arm-peg-insertion": (
            ((0.29, 0.20, 0.018), "orange"),
            ((0.39, -0.157, 0.052), "blue"),
        ),
        "joint-lift": (
            ((0.36, -0.20, 0.055), "orange"),
            ((0.36, 0.20, 0.055), "orange"),
        ),
        "loaded-tray-transport": (
            ((0.285, -0.20, 0.065), "green"),
            ((0.395, -0.08, 0.065), "purple"),
            ((0.34, 0.24, 0.002), "blue"),
        ),
        "uncap-return-test-tube": (
            ((0.38, 0.0, 0.135), "orange"),
            ((0.38, 0.0, 0.07), "cyan"),
            ((0.29, 0.18, 0.002), "blue"),
        ),
        "retrieve-bottle-clutter": (
            ((0.42, -0.025, 0.12), "orange"),
            ((0.46, -0.035, 0.12), "green"),
            ((0.29, 0.26, 0.002), "blue"),
        ),
    }
    classifiers = {
        "green": lambda r, g, b: g > 2 * max(r, b),
        "blue": lambda r, g, b: b > 1.5 * max(r, g),
        "orange": lambda r, g, b: r > 1.4 * g and g > 1.5 * b,
        "purple": lambda r, g, b: r > 1.5 * g and b > 1.5 * g,
        "cyan": lambda r, g, b: b > 1.04 * r and g > 1.03 * r,
    }

    try:
        _assert_no_robot_workcell_penetration(engine)
        assert np.isfinite(engine.read("left")[0]).all()
        assert np.isfinite(engine.read("right")[0]).all()
        camera = config["cameras"]["scene"]
        world_from_camera = np.asarray(camera["transform"])
        intrinsics = camera["intrinsics"]
        rgb, _ = engine.capture("scene")
        for point, expected_color in witnesses[environment]:
            camera_point = np.linalg.inv(world_from_camera) @ [*point, 1.0]
            u = round(
                intrinsics["fx"] * camera_point[0] / camera_point[2] + intrinsics["cx"]
            )
            v = round(
                intrinsics["fy"] * camera_point[1] / camera_point[2] + intrinsics["cy"]
            )
            assert 0 <= u < rgb.shape[1] and 0 <= v < rgb.shape[0]
            pixels = rgb[max(0, v - 4) : v + 5, max(0, u - 4) : u + 5]
            assert any(
                classifiers[expected_color](*map(int, pixel))
                for pixel in pixels.reshape(-1, 3)
            ), (point, expected_color, rgb[v, u])
        assert engine.evaluation_snapshot()["identity"]["arm_count"] == 2

        if environment == "oriented-tool-handover":
            tool = place_free("handled_tool", (0.32, 0.30, 0.10))
            advance(1.0)
            np.testing.assert_allclose(data.xpos[tool, :2], (0.32, 0.30), atol=0.006)
            assert data.body("handled_tool").xmat.reshape(3, 3)[0, 0] > 0.98
        elif environment == "two-arm-peg-insertion":
            peg_start = data.body("target_peg").xpos.copy()
            advance(6.0)
            np.testing.assert_allclose(
                data.body("target_peg").xpos[:2], peg_start[:2], atol=0.004
            )
            assert np.linalg.norm(data.body("target_peg").cvel[3:]) < 0.01
            assert engine.reset() is True
            receiver = model.body("receiving_part")
            initial = data.body("receiving_part").xpos.copy()
            data.xfrc_applied[int(receiver.id), 1] = 3.0
            advance(0.5)
            assert data.body("receiving_part").xpos[1] > initial[1] + 0.008

            assert engine.reset() is True
            receiver_start = data.body("receiving_part").xpos.copy()
            peg = place_free("target_peg", (0.39, -0.18, 0.14))
            advance(1.5)
            np.testing.assert_allclose(data.xpos[peg, :2], (0.39, -0.18), atol=0.004)
            assert data.xpos[peg, 2] < 0.06
            assert data.body("target_peg").xmat.reshape(3, 3)[2, 2] > 0.995
            assert (
                np.linalg.norm(
                    data.body("receiving_part").xpos[:2] - receiver_start[:2]
                )
                < 0.008
            )
        elif environment == "joint-lift":
            park_arms_outward()
            tray_start = data.body("two_handle_tray").xpos.copy()
            for name in (
                "two_handle_tray_handle_left",
                "two_handle_tray_handle_right",
            ):
                data.xfrc_applied[int(model.body(name).id), 2] = 4.0
            advance(0.2)
            data.xfrc_applied[:] = 0.0
            assert data.body("two_handle_tray").xpos[2] > tray_start[2] + 0.08
            assert data.body("two_handle_tray").xmat.reshape(3, 3)[2, 2] > 0.99

            assert engine.reset() is True
            park_arms_outward()
            left = model.body("two_handle_tray_handle_left")
            data.xfrc_applied[int(left.id), 2] = 4.0
            advance(0.2)
            assert data.body("two_handle_tray").xmat.reshape(3, 3)[2, 2] < 0.9
        elif environment == "loaded-tray-transport":
            park_arms_outward()
            tray = place_free("loaded_tray", (0.34, 0.24, 0.08))
            first = place_free("tray_content_1", (0.285, 0.18, 0.145))
            second = place_free("tray_content_2", (0.395, 0.30, 0.145))
            advance(1.5)
            np.testing.assert_allclose(data.xpos[tray, :2], (0.34, 0.24), atol=0.006)
            assert data.body("loaded_tray").xmat.reshape(3, 3)[2, 2] > 0.99
            for content, expected in (
                (first, (0.285, 0.18)),
                (second, (0.395, 0.30)),
            ):
                np.testing.assert_allclose(data.xpos[content, :2], expected, atol=0.006)
                assert 0.035 < data.xpos[content, 2] < 0.045
        elif environment == "uncap-return-test-tube":
            advance(0.5)
            for name in ("tube_cap_grip_left", "tube_cap_grip_right"):
                assert 0.003 < data.joint(name).qpos[0] < 0.0055
            tube = model.body("target_test_tube")
            cap = model.body("target_tube_cap")
            initial_separation = float(
                data.xpos[int(cap.id), 2] - data.xpos[int(tube.id), 2]
            )
            data.xfrc_applied[int(cap.id), 2] = 2.0
            data.xfrc_applied[int(tube.id), 2] = -5.0
            advance(0.5)
            assert (
                data.xpos[int(cap.id), 2] - data.xpos[int(tube.id), 2]
                < initial_separation + 0.01
            )

            assert engine.reset() is True
            initial_separation = float(
                data.xpos[int(cap.id), 2] - data.xpos[int(tube.id), 2]
            )
            data.xfrc_applied[int(cap.id), 2] = 3.0
            data.xfrc_applied[int(tube.id), 2] = -5.0
            for _ in range(round(0.5 / config["timestep"])):
                engine.step()
                if (
                    data.xpos[int(cap.id), 2] - data.xpos[int(tube.id), 2]
                    > initial_separation + 0.04
                ):
                    break
            data.xfrc_applied[:] = 0.0
            assert (
                data.xpos[int(cap.id), 2] - data.xpos[int(tube.id), 2]
                > initial_separation + 0.04
            )
            cap_id = place_free("target_tube_cap", (0.29, 0.18, 0.06))
            tube_id = place_free("target_test_tube", (0.38, 0.0, 0.12))
            advance(1.0)
            np.testing.assert_allclose(data.xpos[cap_id, :2], (0.29, 0.18), atol=0.006)
            assert 0.02 < data.xpos[cap_id, 2] < 0.03
            np.testing.assert_allclose(data.xpos[tube_id, :2], (0.38, 0.0), atol=0.004)
            assert data.body("target_test_tube").xmat.reshape(3, 3)[2, 2] > 0.985
        else:
            assert environment == "retrieve-bottle-clutter"
            distractors = {
                name: data.body(name).xpos.copy()
                for name in ("other_bottle_1", "other_bottle_2", "other_bottle_3")
            }
            target = place_free("target_bottle", (0.29, 0.26, 0.12))
            advance(1.5)
            np.testing.assert_allclose(data.xpos[target, :2], (0.29, 0.26), atol=0.006)
            assert data.body("target_bottle").xmat.reshape(3, 3)[2, 2] > 0.99
            for name, initial in distractors.items():
                np.testing.assert_allclose(data.body(name).xpos, initial, atol=0.003)
                assert abs(data.body(name).xpos[0] - 0.44) < 0.075
                assert abs(data.body(name).xpos[1]) < 0.075

        assert engine.evaluation_reset(seed=7)
    finally:
        engine.close()


@pytest.mark.parametrize("durations", [[0.1], [1 / 60] * 6, [0.0005] * 200])
def test_explicit_worker_advances_only_on_the_shared_sdk_clock(monkeypatch, durations):
    from waddle_sdk.simulators import worker

    state = SimpleNamespace(steps=0, reads=[])
    config = {"backend": "mujoco", "cameras": {}, "timestep": 0.002}

    class Engine:
        def __init__(self, config, scratch):
            pass

        def step(self):
            state.steps += 1

        def capture(self, name):
            pass

        def read(self):
            state.reads.append(state.steps)

        def hold(self):
            pass

        def close(self):
            pass

    class Connection:
        pending = [
            config,
            ("capture", ["scene"]),
            ("read", []),
            *[("step", [dt]) for dt in durations],
            ("read", []),
            ("close", []),
        ]

        def recv(self):
            return self.pending.pop(0)

        def send(self, result):
            assert result[0], result

        def close(self):
            pass

    monkeypatch.setattr(
        worker,
        "importlib",
        SimpleNamespace(import_module=lambda name: SimpleNamespace(Engine=Engine)),
    )
    worker.serve(Connection())
    assert state.reads == [0, 50]
    assert state.steps == 50


@pytest.mark.parametrize("operation", ["read", "write", "hold"])
def test_world_control_is_served_before_a_second_queued_camera(operation):
    import threading
    from concurrent.futures import ThreadPoolExecutor

    world = World({})
    capture_started = threading.Event()
    release_capture = threading.Event()
    calls = []

    class Connection:
        def send(self, request):
            calls.append(request[0])

        def poll(self, timeout):
            if len(calls) == 1:
                capture_started.set()
                assert release_capture.wait(2)
            return True

        def recv(self):
            return True, calls[-1]

    world._connection = Connection()

    def wait_queued(count):
        deadline = time.monotonic() + 2
        while True:
            with world._requests:
                if len(world._pending) == count:
                    return
            assert time.monotonic() < deadline, "request did not queue"
            time.sleep(0.001)

    with ThreadPoolExecutor(max_workers=3) as pool:
        first = pool.submit(world.call, "capture", "scene")
        try:
            assert capture_started.wait(2)
            second = pool.submit(world.call, "capture", "wrist")
            wait_queued(1)
            control = pool.submit(world.call, operation)
            wait_queued(2)
        finally:
            release_capture.set()
        assert first.result(timeout=2) == "capture"
        assert control.result(timeout=2) == operation
        assert second.result(timeout=2) == "capture"
    assert calls == ["capture", operation, "capture"]


def test_world_close_waits_for_active_io_and_unblocks_queued_camera():
    import threading
    from concurrent.futures import ThreadPoolExecutor

    world = World({})
    capture_started = threading.Event()
    release_capture = threading.Event()
    calls = []

    class Connection:
        def send(self, request):
            calls.append(request[0])

        def poll(self, timeout):
            if len(calls) == 1:
                capture_started.set()
                assert release_capture.wait(2)
            return True

        def recv(self):
            return True, None

        def close(self):
            calls.append("disconnected")

    world._connection = Connection()
    with ThreadPoolExecutor(max_workers=3) as pool:
        first = pool.submit(world.call, "capture", "scene")
        try:
            assert capture_started.wait(2)
            closing = pool.submit(world.close)
            second = pool.submit(world.call, "capture", "wrist")
            deadline = time.monotonic() + 2
            while True:
                with world._requests:
                    if len(world._pending) == 2:
                        break
                assert time.monotonic() < deadline
                time.sleep(0.001)
        finally:
            release_capture.set()
        first.result(timeout=2)
        closing.result(timeout=2)
        with pytest.raises(RuntimeError, match="unavailable"):
            second.result(timeout=2)
    assert calls == ["capture", "close", "disconnected"]
    assert world._pending == [] and not world._busy


@pytest.mark.parametrize("backend", BACKENDS)
@pytest.mark.parametrize("reset_on_episode", [False, True])
def test_episode_boundary_preserves_state_unless_reset_requested(
    tmp_path, monkeypatch, backend, reset_on_episode
):
    interpreter = _native_python(backend, "two_cubes")
    monkeypatch.setenv("MUJOCO_GL", "egl")
    site, _ = documents(tmp_path, backend=backend, worker_python=interpreter)
    if reset_on_episode:
        site["worlds"]["cell"]["options"] = {"reset_on_episode": True}
    (tmp_path / "site.yaml").write_text(yaml.safe_dump(site))
    with load_site(tmp_path / "site.yaml").open(
        console=False, _testing=True
    ) as session:
        owner = session._managed.arms["arm"].driver.world
        moved = np.array(profile("yam").home)
        moved[0] += 0.08
        # Establish a different physical pose without depending on a controller.
        owner.call("home", moved)
        with session.run(task="continue", actor="test"):
            actual = session.observe().parts["arm"].joint_position
            expected = profile("yam").home if reset_on_episode else moved
            np.testing.assert_allclose(actual, expected, atol=0.002)


def test_mujoco_administration_reports_ground_truth_and_resets_inside_run(
    tmp_path, monkeypatch
):
    interpreter = _native_python("mujoco", "drawer")
    monkeypatch.setenv("MUJOCO_GL", "egl")
    documents(
        tmp_path,
        backend="mujoco",
        environment="drawer",
        worker_python=interpreter,
    )
    administration = SimulationAdministration()
    with load_site(tmp_path / "site.yaml").open(
        console=False,
        _testing=True,
        simulation_administration=administration,
    ) as session:
        run = session.begin_run(task="open the drawer", actor="test")
        run_id = run.id
        try:
            initial = administration.snapshot()
            world = initial.worlds["cell"]
            assert world["schema"] == "waddle.simulation-state/mujoco-v1"
            assert world["identity"] == {
                "provider": "mujoco",
                "provider_revision": "3.11.0",
                "robot_family": "yam",
                "embodiment_revision": "1.0.0",
                "arm_count": 1,
                "environment_id": "drawer",
                "scene_revision": "1.0.0",
                "asset_revision": "1.0.0",
            }
            assert "drawer_slide" in world["joints"]
            assert "drawer" in world["bodies"]
            assert world["joints"]["drawer_slide"]["type"] == "slide"

            reset = administration.reset(seed=23)
            assert reset.episode_revision == 1
            assert reset.worlds["cell"]["joints"]["drawer_slide"]["qpos"] == [0.0]
            assert run.id == run_id and not run.done
        finally:
            run.__exit__(None, None, None)


@pytest.mark.parametrize("invalid", ["false", 0, 1, None])
def test_episode_reset_setting_requires_a_boolean(invalid):
    with pytest.raises(ValueError, match="reset_on_episode must be a boolean"):
        World({}, reset_on_episode=invalid)


@pytest.mark.parametrize("backend", BACKENDS)
def test_site_owns_one_world_and_reopens_an_independent_world(
    tmp_path, monkeypatch, backend
):
    interpreter = _native_python(backend, "two_cubes")
    monkeypatch.setenv("MUJOCO_GL", "egl")
    documents(tmp_path, backend=backend, worker_python=interpreter)
    site = load_site(tmp_path / "site.yaml")
    processes = []
    for _ in range(2):
        with site.open(console=False, _testing=True) as session:
            managed = session._managed
            owner = managed.arms["arm"].driver.world
            assert all(c.world is owner for c in managed.cameras.values())
            assert isinstance(owner, World)
            owner.step(0.0)
            assert owner.reset() is True
            np.testing.assert_allclose(
                owner.call("read")[0], profile("yam").home, atol=0.01
            )
            processes.append(owner._process)
            assert len(session.observe().parts["arm"].joint_position) == 7
            session.estop("test")
            deadline = time.monotonic() + 2
            while (
                not managed.arms["arm"].driver.estopped and time.monotonic() < deadline
            ):
                time.sleep(0.01)
            assert managed.arms["arm"].driver.estopped
            with pytest.raises(RuntimeError, match="e-stopped"):
                managed.arms["arm"].driver.write(profile("yam").home)
        assert owner._process is None
        assert processes[-1].poll() is not None
    assert processes[0].pid != processes[1].pid


def test_camera_profile_mismatch_fails_before_open(tmp_path):
    pytest.importorskip("mujoco")
    site, _ = documents(tmp_path)
    site["cameras"]["scene"]["intrinsics"]["fx"] += 1
    (tmp_path / "site.yaml").write_text(yaml.safe_dump(site))
    with pytest.raises(ValueError, match="differs"):
        with load_site(tmp_path / "site.yaml").open(console=False, _testing=True):
            pass


@pytest.mark.parametrize("backend", BACKENDS)
def test_site_preserves_custom_part_camera_and_base_frame_names(
    tmp_path, monkeypatch, backend
):
    interpreter = _native_python(backend, "two_cubes")
    monkeypatch.setenv("MUJOCO_GL", "egl")
    site, simulation = documents(tmp_path, backend=backend, worker_python=interpreter)
    site["parts"]["left"] = site["parts"].pop("arm")
    simulation["parts"]["left"] = simulation["parts"].pop("arm")
    site["parts"]["left"]["base_frame"] = "left_base"
    for old, new in (("scene", "bench_rgbd"), ("wrist", "left_rgbd")):
        for cameras in (site["cameras"], simulation["cameras"]):
            cameras[new] = cameras.pop(old)
            cameras[new]["frame_id"] = new
            if old == "wrist":
                cameras[new]["mount"]["part"] = "left"
    (tmp_path / "site.yaml").write_text(yaml.safe_dump(site))
    (tmp_path / "simulation.json").write_text(json.dumps(simulation))
    with load_site(tmp_path / "site.yaml").open(
        console=False, _testing=True
    ) as session:
        observed = session.observe()
        deadline = time.monotonic() + 5
        while set(observed.cameras) != {"bench_rgbd", "left_rgbd"}:
            assert time.monotonic() < deadline, "both named cameras must produce frames"
            time.sleep(0.01)
            observed = session.observe()
        assert set(observed.parts) == {"left"}
        assert observed.parts["left"].frame_id == "left_base"
        assert observed.parts["left"].ee_pose_wxyz is not None
        assert set(observed.cameras) == {"bench_rgbd", "left_rgbd"}
        arm = session._managed.arms["left"]
        assert arm.collision_frame == "left_base"
        with session.run(task="hold measured pose", actor="test") as run:
            assert run.step(observed.gate_vector(), observed).dispatched
            run.finish("success")


def test_reference_world_rejects_site_part_absent_from_simulation(tmp_path):
    site, _ = documents(tmp_path)
    site["parts"]["other"] = dict(site["parts"]["arm"])
    (tmp_path / "site.yaml").write_text(yaml.safe_dump(site))
    with pytest.raises(ValueError, match="absent from the simulation"):
        load_site(tmp_path / "site.yaml")._assembly(None)


def test_realtime_worker_accounts_for_render_delay_before_changing_targets(monkeypatch):
    from waddle_sdk.simulators import worker

    clock = SimpleNamespace(now=0.0)
    state = SimpleNamespace(target=0, steps=[], reads=[])
    config = {"backend": "mujoco", "cameras": {}, "timestep": 0.002, "_real_time": True}

    class Engine:
        def __init__(self, config, scratch):
            pass

        def step(self):
            state.steps.append(state.target)

        def write(self, target):
            state.target = target

        def capture(self, name):
            clock.now += 0.08

        def read(self):
            state.reads.append(len(state.steps))

        def hold(self):
            state.target = 0

        def close(self):
            pass

    class Connection:
        pending = [
            (0.0, config),
            (0.0, ("write", [1])),
            (0.01, ("step", [0.01])),
            (0.02, ("capture", ["scene"])),
            (0.1, ("write", [2])),
            (0.11, ("read", [])),
            (0.11, ("close", [])),
        ]

        def recv(self):
            clock.now, value = self.pending.pop(0)
            return value

        def send(self, result):
            assert result[0], result

        def close(self):
            pass

    monkeypatch.setattr(worker.time, "monotonic", lambda: clock.now)
    monkeypatch.setattr(
        worker,
        "importlib",
        SimpleNamespace(import_module=lambda _: SimpleNamespace(Engine=Engine)),
    )
    worker.serve(Connection())
    assert state.steps == [1] * 50 + [2] * 5
    assert state.reads == [55]


@pytest.mark.parametrize("real_time", [True, False])
def test_worker_slow_physics_keeps_controls_live_and_explicit_steps_complete(
    monkeypatch, real_time, caplog
):
    from waddle_sdk.simulators import worker

    clock = SimpleNamespace(now=0.0)
    state = SimpleNamespace(target=0, steps=[], reads=[])
    config = {
        "backend": "mujoco",
        "cameras": {},
        "timestep": 0.002,
        "_real_time": real_time,
    }

    class Engine:
        def __init__(self, config, scratch):
            pass

        def step(self):
            state.steps.append(state.target)
            clock.now += 0.007  # Slower than real time, without a wall-clock race.

        def write(self, target):
            state.target = target

        def read(self):
            state.reads.append(len(state.steps))

        def hold(self):
            pass

        def close(self):
            pass

    class Connection:
        def __init__(self):
            self.pending = iter(
                [
                    (0.0, config),
                    (0.0, ("write", [1])),
                    (0.1, ("read", [])),
                    (0.0, ("write", [2])),
                    (0.01, ("read", [])),
                    (0.0, ("step", [0.02])),
                    (0.0, ("read", [])),
                    (0.0, ("close", [])),
                ]
            )

        def recv(self):
            delay, value = next(self.pending)
            clock.now += delay
            return value

        def send(self, result):
            assert result[0], result

        def close(self):
            pass

    monkeypatch.setattr(worker.time, "monotonic", lambda: clock.now)
    monkeypatch.setattr(
        worker,
        "importlib",
        SimpleNamespace(import_module=lambda _: SimpleNamespace(Engine=Engine)),
    )
    worker.serve(Connection())
    if real_time:
        assert state.steps == [1] * 3 + [2] * 3
        assert state.reads == [3, 6, 6]
        assert caplog.text.count("Physics cannot keep up") == 1
    else:
        assert state.steps == [2] * 10
        assert state.reads == [0, 0, 10]
        assert "Physics cannot keep up" not in caplog.text


@pytest.mark.parametrize("invalid", [1, "true", None])
def test_reference_real_time_mode_requires_boolean(invalid):
    with pytest.raises(ValueError, match="real_time must be a boolean"):
        World({}, real_time=invalid)


@pytest.mark.parametrize("robot", ROBOTS)
def test_reference_control_defaults_match_nonopening_physical_declaration(robot):
    from waddle_sdk.robots import xarm
    from waddle_sdk.robots.site import PartConfig

    if robot == "so101":
        site, _ = make_site(
            "parity", backend="sapien", robot=robot, environment="two_cubes"
        )
        options = site["parts"]["arm"]["options"]
        # LeRobot's SO-101 follower exposes degrees for joints, a 0..100
        # gripper range and the maintained policy/control rate of 30 Hz.
        assert options == {
            "rate_hz": 30.0,
            "max_joint_speed_rad_s": 1.0,
            "max_gripper_speed_per_s": 1.0,
        }
        return
    if robot == "yam":
        physical = (
            yam.arm(workspace=None, channel="unused-test-bus").robot().action_space
        )
    else:
        physical = (
            xarm.arm(
                config=PartConfig(
                    name="arm",
                    posture="supervised",
                    connection={},
                    joint_limits={},
                    workspace_bounds={},
                    envelope={},
                    options={"model": robot},
                )
            )
            .robot()
            .action_space
        )
    site, _ = make_site(
        "parity", backend="sapien", robot=robot, environment="two_cubes"
    )
    options = site["parts"]["arm"]["options"]
    assert options["rate_hz"] == physical.rate_hz
    assert options["max_joint_speed_rad_s"] == physical.joints[0].max_velocity
    assert options["max_gripper_speed_per_s"] == physical.joints[-1].max_velocity


@pytest.mark.parametrize("robot", ROBOTS)
def test_chocolate_packing_camera_resolves_pocket_floors(tmp_path, robot):
    pytest.importorskip("mujoco")
    from waddle_sdk.simulators.mujoco import Engine

    _, config = make_site(
        "chocolate-pocket-view",
        backend="mujoco",
        robot=robot,
        environment="chocolate-packing",
        width=640,
        height=480,
    )
    engine = Engine(config, tmp_path)
    try:
        engine.evaluation_reset(seed=0)
        rgb, depth = engine.capture("scene")
        camera = config["cameras"]["scene"]
        world_from_camera = np.asarray(camera["transform"])
        camera_from_world = np.linalg.inv(world_from_camera)
        intr = camera["intrinsics"]
        for index in range(1, 7):
            center = engine.data.body(f"packing_slot_{index}").xpos.copy()
            # A center selected from the visible pocket mouth must see the blue
            # floor, not the gold collar. Test actual native pixels and depth.
            center[2] += 0.024
            point = camera_from_world @ np.r_[center, 1.0]
            u = round(intr["fx"] * point[0] / point[2] + intr["cx"])
            v = round(intr["fy"] * point[1] / point[2] + intr["cy"])
            assert 0 <= u < 640 and 0 <= v < 480
            red, _green, blue = map(int, rgb[v, u])
            assert blue > red * 1.5
            if depth is not None:
                z = float(depth[v, u]) * intr["depth_scale_mm"] / 1000
                projected = world_from_camera @ [
                    z * (u - intr["cx"]) / intr["fx"],
                    z * (v - intr["cy"]) / intr["fy"],
                    z,
                    1.0,
                ]
                assert abs(projected[2] - 0.004) < 0.002
                assert np.linalg.norm(projected[:2] - center[:2]) < 0.0025
    finally:
        engine.close()


@pytest.mark.parametrize("robot", ROBOTS)
def test_chocolate_packing_native_slots_and_source_continuation(tmp_path, robot):
    pytest.importorskip("mujoco")
    from waddle_sdk.simulators.mujoco import Engine

    _, config = make_site(
        "chocolate-native",
        backend="mujoco",
        robot=robot,
        environment="chocolate-packing",
        width=320,
        height=240,
    )
    engine = Engine(config, tmp_path)
    try:
        assert engine.evaluation_reset(seed=0)
        for _ in range(1000):
            engine.step()
        names = tuple(f"chocolate_{index:02d}" for index in range(1, 21))
        positions = np.array([engine.data.body(name).xpos.copy() for name in names])
        assert np.all(np.abs(positions[:, 2] - 0.011) < 0.002)
        assert (
            min(
                np.linalg.norm(first[:2] - second[:2])
                for index, first in enumerate(positions)
                for second in positions[index + 1 :]
            )
            > 0.025
        )

        slot = engine.data.body("packing_slot_1").xpos.copy()
        body = engine.model.body(names[0])
        joint = engine.model.joint(int(body.jntadr[0]))
        qpos = int(joint.qposadr[0])
        dof = int(joint.dofadr[0])
        engine.data.qpos[qpos : qpos + 7] = (*slot[:2], 0.07, 1, 0, 0, 0)
        engine.data.qvel[dof : dof + 6] = 0
        for _ in range(750):
            engine.step()
        seated = engine.data.body(names[0]).xpos.copy()
        assert np.linalg.norm(seated[:2] - slot[:2]) < 0.0025
        assert 0.009 <= seated[2] <= 0.014
        snapshot = engine.evaluation_snapshot()
        support = sum(
            row["normal_force_n"]
            for row in snapshot["contacts"]
            if {row["first"]["body"], row["second"]["body"]}
            == {names[0], "packing_box"}
        )
        assert support > 0.03

        source_before = {name: engine.data.body(name).xpos.copy() for name in names[1:]}
        assert engine.evaluation_reset(
            seed=0,
            preserve_free_bodies=names[1:],
            retire_free_bodies=(names[0],),
        )
        assert engine.evaluation_snapshot()["retired_free_bodies"] == [names[0]]
        for name, before in source_before.items():
            np.testing.assert_allclose(engine.data.body(name).xpos, before, atol=1e-9)
        assert engine.data.body(names[0]).xpos[2] < -0.4
        geom = int(engine.model.body_geomadr[int(body.id)])
        assert engine.model.geom_contype[geom] == 0
        assert engine.model.geom_conaffinity[geom] == 0
        assert engine.model.geom_rgba[geom, 3] == 0
        assert engine.evaluation_reset(
            seed=0,
            preserve_free_bodies=names[1:],
            retire_free_bodies=(names[0],),
        )
        assert engine.evaluation_snapshot()["retired_free_bodies"] == [names[0]]
    finally:
        engine.close()


@pytest.mark.parametrize("robot", ROBOTS)
def test_chocolate_packing_native_bilateral_grasp_lifts(tmp_path, robot):
    pytest.importorskip("mujoco")
    from waddle_sdk.simulators.mujoco import Engine

    _, config = make_site(
        "chocolate-grasp",
        backend="mujoco",
        robot=robot,
        environment="chocolate-packing",
        width=320,
        height=240,
    )
    grasp, lift = {
        "so101": (
            (
                0.6111116363,
                0.2058768679,
                0.2887797362,
                -0.47794756798,
                -1.6836794621,
                0.15,
            ),
            (
                0.6111028479,
                0.0851971464,
                -0.1595262347,
                0.0910381109,
                -1.6865021762,
                0.0,
            ),
        ),
        "yam": (
            (
                -0.5073011758,
                1.9527587169,
                1.3864839788,
                -1.0045904466,
                -0.0003258847,
                0.0162585578,
                0.3999989982,
            ),
            (
                -0.5073832518,
                1.9063926729,
                1.5698738174,
                -1.2342777578,
                -0.00000725,
                0.0162300983,
                0.0,
            ),
        ),
        "xarm7": (
            (
                0.3545473497,
                0.0280239344,
                -0.7897804566,
                0.5556825795,
                0.0389480318,
                0.5362834975,
                -3.0865124481,
                0.25,
            ),
            (
                0.3118233049,
                -0.3061679102,
                -0.7296058080,
                0.6683255177,
                -0.2561315532,
                0.9158303896,
                -2.85398900596,
                0.0,
            ),
        ),
    }[robot]
    engine = Engine(config, tmp_path)
    try:
        grasp, lift = np.asarray(grasp), np.asarray(lift)
        engine.home(grasp)
        target = "chocolate_13" if robot == "yam" else "chocolate_01"
        if robot != "yam":
            tcp = profile(robot).poses(grasp)[-1]
            center = tcp[:3, 3] + tcp[:3, :3] @ np.asarray(profile(robot).pinch_offset)
            if robot == "xarm7":
                center += (0, 0, 0.03)
            body = engine.model.body(target)
            joint = engine.model.joint(int(body.jntadr[0]))
            engine.data.qpos[int(joint.qposadr[0]) : int(joint.qposadr[0]) + 7] = (
                *center,
                1,
                0,
                0,
                0,
            )
            engine.mj.mj_forward(engine.model, engine.data)
        start = float(engine.data.body(target).xpos[2])
        closed = grasp.copy()
        closed[-1] = 0
        engine.write(closed)
        for _ in range(750):
            engine.step()
        for fraction in np.linspace(0.01, 1.0, 150):
            engine.write(closed + fraction * (lift - closed))
            for _ in range(5):
                engine.step()
        for _ in range(250):
            engine.step()
        assert engine.data.body(target).xpos[2] > start + 0.04
        fingers = {
            "so101": {"gripper_link", "moving_jaw_so101_v1_link"},
            "yam": {"tip_left", "tip_right"},
            "xarm7": {"left_finger", "right_finger"},
        }[robot]
        touched = set()
        for row in engine.evaluation_snapshot()["contact_events"]["pairs"]:
            bodies = {row["first"]["body"], row["second"]["body"]}
            if target in bodies and row["maximum_normal_force_n"] > 0:
                touched.update(bodies & fingers)
        assert touched == fingers
    finally:
        engine.close()
