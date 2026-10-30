"""Authored scenes retain reference feedback, camera and reset behavior."""

from copy import deepcopy

import numpy as np
import pytest
from waddle_sdk.simulators.playground import CATALOG, compile_scene, validate_scene
from waddle_sdk.simulators.scene import load_scene, make_site, profile


@pytest.mark.parametrize(
    "fixture", ["handover", "hold_container", "peg_socket", "loaded_tray"]
)
@pytest.mark.parametrize("seed", [0, 1, 9])
def test_bimanual_assemblies_are_stable_visible_and_resettable(tmp_path, fixture, seed):
    pytest.importorskip("mujoco")
    from waddle_sdk.simulators.mujoco import Engine

    _, config = make_site(
        "assemblies",
        backend="mujoco",
        robot="yam",
        arms=2,
        environment="two_cubes",
        width=96,
        height=72,
    )
    authored = {
        "id": "task",
        "name": "Task",
        "objects": [],
        "fixture": fixture,
        "seed": seed,
    }
    output = tmp_path / "world"
    compile_scene(config, authored, output)
    _, compiled = load_scene(output, "simulation.json")
    engine = Engine(compiled, output)
    try:
        initial = engine.data.qpos.copy()
        for _ in range(500):
            engine.step()
        assert np.isfinite(engine.data.qpos).all()
        for contact in engine.data.contact:
            names = [
                engine.model.body(int(engine.model.geom(g).bodyid[0])).name
                for g in contact.geom
            ]
            robot_contact = any(
                name.startswith(("left__", "right__")) for name in names
            )
            same_arm = any(
                all(name.startswith(part + "__") for name in names)
                for part in ("left", "right")
            )
            assert not (robot_contact and not same_arm and contact.dist < -0.002), names
        expected = {
            "handover": ["handover_block"],
            "hold_container": ["movable_container", "target_object"],
            "peg_socket": ["receiving_part", "target_peg"],
            "loaded_tray": ["loaded_tray", "tray_content_1", "tray_content_2"],
        }[fixture]
        state = engine.evaluation_snapshot()
        for name in expected:
            body = state["bodies"][name]
            assert body["position_m"][2] > -0.005
            assert np.linalg.norm(body["linear_velocity_m_s"]) < 0.02, (name, body)
        for camera in ("scene", "left_wrist", "right_wrist"):
            rgb, depth = engine.capture(camera)
            assert rgb.shape == (72, 96, 3) and np.ptp(rgb) > 20
            assert depth.shape == (72, 96) and (depth > 0).any()
        engine.reset()
        np.testing.assert_array_equal(engine.data.qpos, initial)
    finally:
        engine.close()


def test_reference_fixture_requires_its_declared_embodiment(tmp_path):
    _, config = make_site(
        "unsupported", backend="mujoco", robot="yam", arms=1, environment="two_cubes"
    )
    authored = {"id": "task", "name": "Task", "objects": [], "fixture": "handover"}
    with pytest.raises(ValueError, match="require two YAMs"):
        compile_scene(config, authored, tmp_path / "world")
    assert not (tmp_path / "world").exists()


def scene():
    return {
        "id": "test",
        "name": "Test scene",
        "objects": [
            {
                "id": kind,
                "kind": kind,
                "position": [0.3, -0.2 + 0.09 * i, row["size"][2] / 2 + 0.001],
                "rotation": [0, 0, 0],
                "size": row["size"],
                "color": [0.3, 0.6, 0.8],
            }
            for i, (kind, row) in enumerate(CATALOG.items())
        ],
    }


@pytest.mark.parametrize("robot", ["yam", "so101", "xarm7"])
@pytest.mark.parametrize("arms", [1, 2])
def test_compiled_scene_preserves_robot_camera_and_canonical_reset(
    tmp_path, robot, arms
):
    pytest.importorskip("mujoco")
    from waddle_sdk.simulators.mujoco import Engine

    _, config = make_site(
        "editable",
        backend="mujoco",
        robot=robot,
        environment="two_cubes",
        arms=arms,
        width=64,
        height=48,
    )
    output = tmp_path / "world"
    compile_scene(config, scene(), output)
    _, compiled = load_scene(output, "simulation.json")
    engine = Engine(compiled, output)
    try:
        original = engine.data.qpos.copy()
        for part in engine.parts:
            np.testing.assert_allclose(
                engine.read(part)[0], engine.profile.home, atol=1e-8
            )
        for _ in range(100):
            engine.step()
        for camera in config["cameras"]:
            rgb, depth = engine.capture(camera)
            assert rgb.shape == (48, 64, 3) and np.ptp(rgb) > 20
            if config["cameras"][camera]["options"]["depth"]:
                assert depth.shape == (48, 64) and (depth > 0).any()
            else:
                assert depth is None
        engine.reset()
        np.testing.assert_array_equal(engine.data.qpos, original)
        assert not engine._force_modes
    finally:
        engine.close()


def test_editing_creates_new_bytes_and_preserves_original_scene(tmp_path):
    pytest.importorskip("mujoco")
    _, config = make_site(
        "editable", backend="mujoco", robot="yam", environment="two_cubes"
    )
    first = compile_scene(config, scene(), tmp_path / "first")
    original = (tmp_path / "first/world.xml").read_bytes()
    changed = scene()
    changed["objects"][0]["position"][0] += 0.04
    second = compile_scene(config, changed, tmp_path / "second", revision=2)
    assert (
        second["native_scene"]["files"]["world.xml"]
        != first["native_scene"]["files"]["world.xml"]
    )
    assert (tmp_path / "first/world.xml").read_bytes() == original
    with pytest.raises(ValueError, match="already exists"):
        compile_scene(config, changed, tmp_path / "first")


@pytest.mark.parametrize(
    "mutation",
    [
        lambda s: s["objects"][0].update(id="../escape"),
        lambda s: s["objects"][0].update(position=[float("nan"), 0, 0]),
        lambda s: s["objects"][0].update(size=[-1, 1, 1]),
        lambda s: s["objects"][0].update(kind="executable.py"),
        lambda s: s["objects"].append(deepcopy(s["objects"][0])),
    ],
)
def test_scene_rejects_invalid_geometry_before_compiling(mutation):
    value = scene()
    mutation(value)
    with pytest.raises(ValueError, match="playground"):
        validate_scene(value)


def test_scene_replacement_keeps_ports_and_native_processes_and_fences_old_images(
    tmp_path,
):
    pytest.importorskip("mujoco")
    import json

    import yaml
    from waddle_sdk import load_site
    from waddle_sdk.simulation import (
        SimulationAdministration,
        SimulationAdministrationError,
        WorldConfig,
    )

    site, config = make_site(
        "editable",
        backend="mujoco",
        robot="yam",
        environment="two_cubes",
        width=96,
        height=72,
    )
    site["worlds"]["cell"]["options"] = {"real_time": False}
    first, second = tmp_path / "first", tmp_path / "second"
    empty = {"id": "empty", "name": "Empty", "objects": []}
    changed = deepcopy(empty)
    changed["objects"] = [
        {
            "id": "block",
            "kind": "block",
            "position": [0.3, -0.1, 0.021],
            "rotation": [0, 0, 0],
            "size": [0.04, 0.04, 0.04],
            "color": [1, 0, 0],
        }
    ]
    compile_scene(config, empty, first)
    compile_scene(config, changed, second, revision=2)
    (first / "site.yaml").write_text(yaml.safe_dump(site))
    admin = SimulationAdministration()
    with load_site(first / "site.yaml").open(
        console=False, _testing=True, simulation_administration=admin
    ) as session:
        world = session._managed.arms["arm"].driver.world
        pids = world._process.pid, world._renderer.process.pid
        assert admin.can_replace_scene
        before = world.capture("scene")[0]
        # Replacement must also restore robot state, not replay a previous target.
        pose = np.array(world.call("read_part", "arm")[0])
        pose[0] += 0.1
        world.call("home_part", "arm", pose)
        result = admin.replace_scene(
            {
                "cell": WorldConfig(
                    "cell", {"simulation": "simulation.json"}, site_root=second
                )
            }
        )
        assert result.episode_revision == 1
        assert "prop_block" in result.worlds["cell"]["bodies"]
        assert (world._process.pid, world._renderer.process.pid) == pids
        np.testing.assert_allclose(
            world.call("read_part", "arm")[0], profile("yam").home, atol=1e-8
        )
        after, _, snapshot, _ = world.capture("scene")
        assert snapshot["generation"] == world._generation == 1
        assert np.abs(after.astype(float) - before).max() > 20
        assert "prop_block" in admin.reset(seed=0).worlds["cell"]["bodies"]
        # An admitted contract violation cannot leave plans or admin state usable.
        invalid = json.loads((second / "simulation.json").read_text())
        invalid["cameras"]["scene"]["stream"]["width"] += 1
        (second / "simulation.json").write_text(json.dumps(invalid))
        with pytest.raises(SimulationAdministrationError, match="not confirmed"):
            admin.replace_scene(
                {
                    "cell": WorldConfig(
                        "cell", {"simulation": "simulation.json"}, site_root=second
                    )
                }
            )
        with pytest.raises(SimulationAdministrationError, match="failed; reopen"):
            admin.snapshot()
