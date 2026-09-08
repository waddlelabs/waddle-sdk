"""Physics/SDK conformance, using real engines when their extras are installed."""

from __future__ import annotations

import importlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pytest
import yaml
from waddle_sdk import load_site
from waddle_sdk.robots import yam
from waddle_sdk.simulators.adapters import World
from waddle_sdk.simulators.model import SCREW_PITCH, screw_force
from waddle_sdk.simulators.scene import (
    BACKENDS,
    ENVIRONMENTS,
    ROBOTS,
    depth_z16,
    load_scene,
    make_site,
    profile,
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


def test_yam_fk_matches_live_adapter_at_multiple_configurations():
    p = profile("yam")
    rng = np.random.default_rng(12)
    for _ in range(20):
        q = np.array([rng.uniform(lo, hi) for lo, hi in p.limits])
        expected_pos, expected_rot = yam.forward_kinematics(q[:-1])
        pose = p.poses(q)[-1]
        np.testing.assert_allclose(pose[:3, 3], expected_pos, atol=1e-9)
        np.testing.assert_allclose(pose[:3, :3], expected_rot, atol=1e-9)


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


def test_screw_coupling_exchanges_work_without_teleporting():
    torque, force = screw_force((1.0, 0.0), (0.0, 0.0))
    assert force > 0
    assert torque == pytest.approx(-SCREW_PITCH * force)
    assert screw_force((2.0, 2 * SCREW_PITCH), (3.0, 3 * SCREW_PITCH)) == (0.0, 0.0)


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
    p = profile(robot)
    engine = importlib.import_module(f"waddle_sdk.simulators.{backend}").Engine(
        config, tmp_path
    )
    try:
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
        for _ in range(1000):
            engine.step()
        measured, velocity = engine.read()
        np.testing.assert_allclose(measured, target, atol=0.015)
        assert np.isfinite(velocity).all()
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
        z = depth[v, u] / 1000
        assert z > 0
        world = t @ [
            z * (u - intr["cx"]) / intr["fx"],
            z * (v - intr["cy"]) / intr["fy"],
            z,
            1.0,
        ]
        assert abs(world[2]) < 0.004
        if environment == "drawer":
            if backend == "mujoco":
                joint = engine.model.joint("drawer_slide")
                engine.data.qfrc_applied[int(joint.dofadr[0])] = 5.0
            elif backend == "sapien":
                engine.props[-1].set_qf(np.array([5.0]))
            else:
                engine.props[-1].set_joint_efforts(np.array([-5.0]))
            for _ in range(1000):
                engine.step()
            travel = (
                float(engine.data.qpos[int(joint.qposadr[0])])
                if backend == "mujoco"
                else float(engine.props[-1].get_qpos()[0])
                if backend == "sapien"
                else -float(engine.props[-1].get_joint_positions()[0])
            )
            assert 0.20 < travel < 0.225
        elif environment == "bottle_cap":
            # Apply a native generalized torque as an external load on the cap.
            # Upward travel must come from the passive thread constraint.
            module = importlib.import_module(f"waddle_sdk.simulators.{backend}")
            law = module.screw_force
            module.screw_force = lambda q, dq: (law(q, dq)[0] + 0.3, law(q, dq)[1])
            for _ in range(2000):
                engine.step()
            module.screw_force = law
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
    finally:
        engine.close()


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
