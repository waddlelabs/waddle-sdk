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


def _native_conformance(tmp_path, backend, robot, environment):
    _, config = documents(tmp_path, backend, robot, environment)
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
                engine.data.qfrc_applied[int(joint.dofadr[0])] = 5.0
            elif backend == "sapien":
                engine.props[-1].set_qf(np.array([5.0]))
            else:
                engine.props[-1].set_joint_efforts(np.array([5.0]))
            advance(2.0)
            travel = (
                float(engine.data.qpos[int(joint.qposadr[0])])
                if backend == "mujoco"
                else float(engine.props[-1].get_qpos()[0])
                if backend == "sapien"
                else float(engine.props[-1].get_joint_positions()[0])
            )
            assert 0.20 < travel < 0.225, travel
        elif environment == "bottle_cap":
            # Apply a native generalized torque as an external load on the cap.
            # Upward travel must come from the passive thread constraint.
            if backend == "mujoco":
                engine.data.qfrc_applied[int(engine._screw[0].dofadr[0])] = 0.3
            else:
                names = (
                    [j.name for j in engine._screw.get_active_joints()]
                    if backend == "sapien"
                    else list(engine._screw.dof_names)
                )
                force = np.zeros(len(names))
                force[names.index("cap_rotation")] = 0.3
                if backend == "sapien":
                    engine._screw.set_qf(force)
                else:
                    engine._screw.set_joint_efforts(force)
            advance(4.0)
            if backend == "mujoco":
                q = [float(engine.data.qpos[int(j.qposadr[0])]) for j in engine._screw]
            elif backend == "sapien":
                names = [j.name for j in engine._screw.get_active_joints()]
                q = engine._screw.get_qpos()[
                    [names.index(n) for n in ("cap_rotation", "cap_lift")]
                ]
            else:
                names = list(engine._screw.dof_names)
                q = engine._screw.get_joint_positions()[
                    [names.index(n) for n in ("cap_rotation", "cap_lift")]
                ]
            assert q[0] > 3.0 and q[1] > 0.002, q
            assert abs(q[1] - SCREW_PITCH * q[0]) < 0.003
        assert engine.reset() is True
        np.testing.assert_allclose(engine.read()[0], p.home, atol=1e-6)
        if backend == "mujoco":
            assert engine.data.time == 0
        if environment == "drawer":
            travel = (
                float(engine.data.qpos[int(joint.qposadr[0])])
                if backend == "mujoco"
                else float(engine.props[-1].get_qpos()[0])
                if backend == "sapien"
                else float(engine.props[-1].get_joint_positions()[0])
            )
            assert travel == pytest.approx(0, abs=1e-6)
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
        physical = yam.arm(workspace=None, channel="unused-test-bus").robot().action_space
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
