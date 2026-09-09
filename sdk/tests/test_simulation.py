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
from waddle_sdk.simulators.adapters import World
from waddle_sdk.simulators.description import (
    collision_bounds,
    description,
    mesh_triangles,
)
from waddle_sdk.simulators.model import SCREW_PITCH
from waddle_sdk.simulators.scene import (
    BACKENDS,
    ENVIRONMENTS,
    RENDER_QUALITIES,
    ROBOTS,
    depth_z16,
    load_scene,
    make_site,
    profile,
    rotation,
)


def documents(root: Path, backend="mujoco", robot="yam", environment="two_cubes"):
    site, sim = make_site(
        "physics-test",
        backend=backend,
        robot=robot,
        environment=environment,
        width=192,
        height=144,
    )
    (root / "simulation.json").write_text(json.dumps(sim))
    (root / "site.yaml").write_text(yaml.safe_dump(site))
    return site, sim


@pytest.mark.parametrize("backend", BACKENDS)
@pytest.mark.parametrize("robot", ROBOTS)
@pytest.mark.parametrize("environment", ENVIRONMENTS)
def test_all_reference_declarations_validate_without_opening(
    tmp_path, backend, robot, environment
):
    site, sim = documents(tmp_path, backend, robot, environment)
    loaded = load_site(tmp_path / "site.yaml")
    assert loaded.id == "physics-test"
    assert load_scene(tmp_path, "simulation.json")[1] == sim
    assert set(site["cameras"]) == {"scene", "wrist"}
    assert site["parts"]["arm"]["gripper"]["open_m"] == profile(robot).opening
    assembly = loaded._assembly(None)
    action_space = assembly.rig.robot().action_space.parts["arm"]
    assert action_space.rate_hz == site["parts"]["arm"]["options"]["rate_hz"]
    assert assembly.rig.rate_hz == max(100.0, 2 * action_space.rate_hz)


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
    assert 0.34 < position[0] < 0.38
    assert abs(position[1]) < 0.02
    assert 0.13 < position[2] < 0.15
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
    assert sum(link.mass for link in model.links) > (4 if robot == "yam" else 10)
    fingers = [
        link
        for link in model.links
        if link.name in ("tip_left", "tip_right", "left_finger", "right_finger")
    ]
    assert len(fingers) == 2
    assert all(
        1 < sum(shape.collision for shape in link.shapes) <= 32 for link in fingers
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
    left, right = (
        ("tip_left", "tip_right") if robot == "yam" else ("left_finger", "right_finger")
    )
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
    np.testing.assert_allclose(
        np.array(widths) - widths[0], np.array([0, 0.25, 0.7, 1]) * p.opening, atol=1e-6
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


@pytest.mark.parametrize("backend", BACKENDS)
@pytest.mark.parametrize("robot", ROBOTS)
@pytest.mark.parametrize("environment", ENVIRONMENTS)
def test_native_models_rgbd_and_bounded_joint_motion(
    tmp_path, monkeypatch, backend, robot, environment
):
    interpreter = sys.executable
    if backend == "isaac":
        interpreter = os.environ.get("WADDLE_ISAAC_TEST_PYTHON")
        if interpreter is None:
            pytest.skip(
                "set WADDLE_ISAAC_TEST_PYTHON to a licensed Isaac Sim interpreter"
            )
    else:
        pytest.importorskip(backend)
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


@pytest.mark.parametrize("backend", ["mujoco", "sapien"])
@pytest.mark.parametrize("quality", ["fast", "high"])
def test_native_render_presets_preserve_rgbd_and_motion(
    tmp_path, monkeypatch, backend, quality
):
    pytest.importorskip(backend)
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
        [sys.executable, "-c", script], capture_output=True, text=True, timeout=300
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
    pytest.importorskip(backend)
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
        [sys.executable, "-c", script],
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


def test_native_thread_assets_are_complete_and_hash_bound():
    from waddle_sdk.simulators.thread import assets

    root = assets()
    manifest = json.loads((root / "manifest.json").read_text())
    meshes = ET.parse(root / "cap.xml").findall("asset/mesh[@file]")
    required = {"cap.xml", "metric_thread.cc", "LICENSE"}
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
        _, config = documents(tmp_path, "mujoco", "yam", environment)
        if environment == "bottle_cap":
            with pytest.raises(RuntimeError, match="requires a C\\+\\+17 compiler"):
                Engine(config, tmp_path)
        else:
            engine = Engine(config, tmp_path)
            try:
                engine.step()
                assert np.isfinite(engine.read()[0]).all()
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


def _native_conformance(
    tmp_path, backend, robot, environment, render_quality="standard"
):
    _, config = documents(tmp_path, backend, robot, environment)
    config["render_quality"] = render_quality
    # Real RGB-D cameras have off-center principal points and unequal focal
    # lengths. Centered defaults conceal renderer convention mistakes.
    config["cameras"]["scene"]["intrinsics"].update(
        fx=174.0, fy=161.0, cx=109.0, cy=62.0, depth_scale_mm=0.1
    )
    p = profile(robot)
    engine = importlib.import_module(f"waddle_sdk.simulators.{backend}").Engine(
        config, tmp_path
    )

    def advance(seconds):
        for _ in range(round(seconds / config["timestep"])):
            engine.step()

    try:
        if environment == "two_cubes" and backend in {"mujoco", "sapien"}:
            # A uniform 60 g, 50 mm cube has I = m * side**2 / 6. Inspect
            # imported native bodies: setting mass alone can leave the inertia
            # computed for the builder's default density behind.
            for index in (1, 2):
                if backend == "mujoco":
                    body = engine.model.body(f"cube_{index}")
                    mass, inertia = float(body.mass[0]), body.inertia
                else:
                    body = engine.props[index].find_component_by_type(
                        engine.sp.physx.PhysxRigidDynamicComponent
                    )
                    mass, inertia = body.mass, body.inertia
                assert mass == pytest.approx(0.06)
                np.testing.assert_allclose(inertia, np.full(3, 0.000025), rtol=1e-5)
        if backend == "mujoco":
            robot_bodies = {link.name for link in description(robot).links}
            # Reference scenes must start without the hand embedded in a prop.
            # Check native contacts, independently of the layout declarations.
            for contact in engine.data.contact:
                bodies = [
                    engine.model.body(engine.model.geom(int(g)).bodyid[0]).name
                    for g in contact.geom
                ]
                if (bodies[0] in robot_bodies) != (bodies[1] in robot_bodies):
                    assert contact.dist >= 0, (bodies, contact.dist)
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
            assert depth.shape == (144, 192) and depth.dtype == np.uint16
            assert np.isfinite(depth).all()
        # A known unoccluded tabletop pixel deprojects onto z=0. This detects
        # optical-axis, depth-range, principal-point, and scale errors together.
        camera = config["cameras"]["scene"]
        t = np.array(camera["transform"])
        point = np.linalg.inv(t) @ np.array([0.1, -0.32, 0.0, 1.0])
        intr = camera["intrinsics"]
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
        elif environment == "bottle_cap" and backend != "mujoco":
            # Apply a native generalized torque as an external load on the cap.
            # These backends retain the passive guide. MuJoCo's contact-based
            # free cap is checked by _native_free_cap instead.
            names = (
                [j.name for j in engine._screw.get_active_joints()]
                if backend == "sapien"
                else list(engine._screw.dof_names)
            )

            def cap_force(value):
                force = np.zeros(len(names))
                force[names.index("cap_rotation")] = value
                if backend == "sapien":
                    engine._screw.set_qf(force)
                else:
                    engine._screw.set_joint_efforts(force)

            def cap_state():
                values = (
                    engine._screw.get_qpos()
                    if backend == "sapien"
                    else engine._screw.get_joint_positions()
                )
                return values[[names.index(n) for n in ("cap_rotation", "cap_lift")]]

            def cap_release(angle, speed):
                scales = np.array(
                    [1 if n == "cap_rotation" else SCREW_PITCH for n in names]
                )
                if backend == "sapien":
                    engine._screw.set_qpos(angle * scales)
                    engine._screw.set_qvel(speed * scales)
                else:
                    engine._screw.set_joint_positions(angle * scales)
                    engine._screw.set_joint_velocities(speed * scales)

            # An end-stop torque test can hide a back-driving thread. Release
            # mid-travel with a small unwinding velocity, as fingers disengage.
            # Native resistance must arrest motion without locking the cap:
            # ordinary applied torque must still turn it in either direction.
            cap_release(np.pi / 2, -0.1)
            advance(0.1)
            released = cap_state()
            advance(5.0)
            drift = np.abs(cap_state() - released)
            assert drift[0] < 0.02 and drift[1] < 0.0001, drift
            cap_force(0.01)
            advance(0.25)
            turned = cap_state()[0]
            assert turned > released[0] + 0.02
            cap_force(-0.01)
            advance(0.5)
            assert cap_state()[0] < turned - 0.02
            cap_force(0.0)
            engine.reset()

            cap_force(0.3)
            advance(4.0)
            q = cap_state()
            assert q[0] > 3.0 and q[1] > 0.002, q
            assert abs(q[1] - SCREW_PITCH * q[0]) < 0.003
            cap_force(0.0)
            advance(2.0)
            released = cap_state()
            advance(5.0)
            assert cap_state()[0] > 3.0
            assert cap_state()[1] == pytest.approx(released[1], abs=0.0001)
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


@pytest.mark.parametrize("backend", ["mujoco", "sapien"])
@pytest.mark.parametrize("reset_on_episode", [False, True])
def test_episode_boundary_preserves_state_unless_reset_requested(
    tmp_path, monkeypatch, backend, reset_on_episode
):
    pytest.importorskip(backend)
    monkeypatch.setenv("MUJOCO_GL", "egl")
    site, _ = documents(tmp_path, backend=backend)
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


@pytest.mark.parametrize("invalid", ["false", 0, 1, None])
def test_episode_reset_setting_requires_a_boolean(invalid):
    with pytest.raises(ValueError, match="reset_on_episode must be a boolean"):
        World({}, reset_on_episode=invalid)


def test_site_owns_one_world_and_reopens_an_independent_world(tmp_path, monkeypatch):
    pytest.importorskip("mujoco")
    monkeypatch.setenv("MUJOCO_GL", "egl")
    documents(tmp_path)
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
    site, _ = documents(tmp_path)
    site["cameras"]["scene"]["intrinsics"]["fx"] += 1
    (tmp_path / "site.yaml").write_text(yaml.safe_dump(site))
    with pytest.raises(ValueError, match="differs"):
        with load_site(tmp_path / "site.yaml").open(console=False, _testing=True):
            pass


@pytest.mark.parametrize("backend", ["mujoco", "sapien"])
def test_site_preserves_custom_part_camera_and_base_frame_names(
    tmp_path, monkeypatch, backend
):
    pytest.importorskip(backend)
    monkeypatch.setenv("MUJOCO_GL", "egl")
    site, simulation = documents(tmp_path, backend=backend)
    site["parts"]["left"] = site["parts"].pop("arm")
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


def test_reference_world_rejects_aliasing_two_parts_to_one_robot(tmp_path):
    site, _ = documents(tmp_path)
    site["parts"]["other"] = dict(site["parts"]["arm"])
    (tmp_path / "site.yaml").write_text(yaml.safe_dump(site))
    with pytest.raises(ValueError, match="one robot part"):
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


@pytest.mark.parametrize("invalid", [1, "true", None])
def test_reference_real_time_mode_requires_boolean(invalid):
    with pytest.raises(ValueError, match="real_time must be a boolean"):
        World({}, real_time=invalid)


@pytest.mark.parametrize("robot", ROBOTS)
def test_reference_control_defaults_match_nonopening_physical_declaration(robot):
    from waddle_sdk.robots import xarm
    from waddle_sdk.robots.site import PartConfig

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
