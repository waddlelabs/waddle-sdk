"""Portable scene and MuJoCo compiler contract tests."""

from __future__ import annotations

import json
import textwrap
from pathlib import Path

import pytest
import waddle_sdk
import waddle_sdk.scene as scene_api
import yaml
from waddle_sdk import cli
from waddle_sdk.scene import (
    ScenePathError,
    SceneValidationError,
    initialize_scene,
    load_scene,
)

SDK = Path(__file__).resolve().parents[1]


def _write_urdf(tmp_path):
    path = tmp_path / "robot.urdf"
    path.write_text(
        textwrap.dedent(
            """\
            <?xml version="1.0"?>
            <robot name="portable_arm">
              <link name="base">
                <inertial>
                  <origin xyz="0 0 0" rpy="0 0 0"/>
                  <mass value="1"/>
                  <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
                </inertial>
                <visual><geometry><box size="0.2 0.2 0.2"/></geometry></visual>
                <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
              </link>
              <link name="tool_link">
                <inertial>
                  <origin xyz="0 0 0.25" rpy="0 0 0"/>
                  <mass value="0.5"/>
                  <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.005"/>
                </inertial>
                <visual>
                  <origin xyz="0 0 0.25" rpy="0 0 0"/>
                  <geometry><cylinder radius="0.04" length="0.5"/></geometry>
                </visual>
                <collision>
                  <origin xyz="0 0 0.25" rpy="0 0 0"/>
                  <geometry><cylinder radius="0.04" length="0.5"/></geometry>
                </collision>
              </link>
              <joint name="shoulder" type="revolute">
                <parent link="base"/>
                <child link="tool_link"/>
                <origin xyz="0 0 0.1" rpy="0 0 0"/>
                <axis xyz="0 1 0"/>
                <limit lower="-1.5" upper="1.5" velocity="2.0" effort="8.0"/>
              </joint>
            </robot>
            """
        ),
        encoding="utf-8",
    )
    return path


def _write_scene(tmp_path, *, lower=-1.0):
    _write_urdf(tmp_path)
    path = tmp_path / "scene.yaml"
    path.write_text(
        textwrap.dedent(
            f"""\
            api_version: waddle.scene/v1
            kind: SimulationScene
            metadata:
              id: portable-cell
            physics:
              timestep_s: 0.005
              gravity_m_s2: [0.0, 0.0, -9.81]
            robots:
              arm:
                urdf: robot.urdf
                pose:
                  position_m: [0.0, 0.0, 0.1]
                  quaternion_wxyz: [1.0, 0.0, 0.0, 0.0]
                base_frame: world
                rate_hz: 100
                joints:
                  shoulder:
                    lower: {lower}
                    upper: 1.0
                    home: 0.0
                    max_velocity: 1.2
                    effort: 6.0
                    kp: 80.0
                tool:
                  link: tool_link
                  pose:
                    position_m: [0.0, 0.0, 0.5]
                appearance:
                  default:
                    rgba: [{{uniform: [0.2, 0.3]}}, 0.4, 0.8, 1.0]
                    roughness: 0.6
                coatings:
                  - name: soft_tip
                    link: tool_link
                    pose:
                      position_m: [0.0, 0.0, 0.51]
                    geometry:
                      kind: sphere
                      radius_m: 0.03
                    material:
                      rgba: [0.1, 0.1, 0.1, 1.0]
                collision_spheres:
                  - {{name: tool, link: tool_link, radius_m: 0.08}}
            cameras:
              scene_camera:
                mount: {{kind: scene}}
                pose:
                  position_m: [0.0, -2.0, 1.0]
                  quaternion_wxyz: [0.70710678, 0.70710678, 0.0, 0.0]
                stream: {{width: 64, height: 48, fps: 20}}
                vertical_fov_deg: {{choice: [55.0, 60.0]}}
                depth: true
                depth_scale_mm: 1.0
            lights:
              key:
                type: directional
                position_m: [0.0, -1.0, 3.0]
                direction: [0.0, 0.2, -1.0]
                color: [1.0, 0.95, 0.9]
                intensity: {{uniform: [0.8, 1.2]}}
                cast_shadows: true
            geometry:
              floor:
                geometry: {{kind: plane, size_m: [4.0, 4.0]}}
                pose: {{position_m: [0.0, 0.0, 0.0]}}
                material: {{rgba: [0.3, 0.3, 0.3, 1.0], roughness: 0.8}}
                collision: {{friction: [0.8, 0.01, 0.001]}}
            site:
              workspace_bounds:
                min: [-1.0, -1.0, 0.0]
                max: [1.0, 1.0, 1.5]
              static_keepouts: []
              self_collision: {{}}
            randomization:
              seed: 7
            """
        ),
        encoding="utf-8",
    )
    return path


def test_portable_scene_validation_is_nonopening_and_seeded(tmp_path, monkeypatch):
    path = _write_scene(tmp_path)
    imported = []

    def fail_import():
        imported.append("mujoco")
        raise AssertionError("validation must not import MuJoCo")

    monkeypatch.setattr("waddle_sdk.robots.mujoco._mujoco_module", fail_import)
    scene = load_scene(path)
    first = scene.resolved().document
    second = scene.resolved().document
    assert first == second
    assert first["resolution"]["seed"] == 7
    assert first["resolution"]["decisions"]
    assert imported == []
    with pytest.raises(SceneValidationError, match="seed must be an integer"):
        scene.resolved(seed=True)


def test_mujoco_compiler_emits_complete_ordinary_site(tmp_path):
    pytest.importorskip("mujoco")
    build = load_scene(_write_scene(tmp_path)).compile(
        backend="mujoco", output_dir=tmp_path / "build"
    )
    assert build.world_path.is_file()
    assert build.site_path.is_file()
    assert build.evidence_path.is_file()

    manifest = yaml.safe_load(build.site_path.read_text())
    assert manifest["worlds"]["cell"]["driver"] == "waddle_sdk.robots.mujoco:backend"
    assert manifest["parts"]["arm"]["joint_limits"] == {"shoulder": [-1.0, 1.0]}
    assert manifest["parts"]["arm"]["options"]["model_joint_names"] == ["arm/shoulder"]
    assert manifest["cameras"]["scene_camera"]["options"]["depth"] is True
    assert manifest["frames"]["scene_camera_optical"]["parent"] == "world"
    assert manifest["calibration"] == {"artifacts": "calib/"}
    assert waddle_sdk.load_site(build.site_path).id == "portable-cell"

    calibration = json.loads((build.output_dir / "calib" / "scene_camera.json").read_text())
    assert calibration["schema"] == "waddle.simulation-calibration/v1"
    assert calibration["from_frame"] == "scene_camera_optical"
    assert calibration["to_frame"] == "world"
    assert calibration["rms_mm"] == 0.0
    assert calibration["valid"] is True
    assert calibration["provenance"] == {
        "kind": "simulation_ground_truth",
        "privileged": True,
        "scene_api_version": "waddle.scene/v1",
        "scene_id": "portable-cell",
        "seed": 7,
    }
    assert [row[3] for row in calibration["matrix"]] == pytest.approx([0.0, -2.0, 1.0, 1.0])

    world = build.world_path.read_text()
    assert 'camera name="scene_camera"' in world
    assert 'light name="key"' in world
    assert 'name="arm/shoulder_position"' in world
    assert "soft_tip" in world

    evidence = json.loads(build.evidence_path.read_text())
    assert evidence["schema"] == "waddle.simulation-build/v1"
    assert evidence["seed"] == 7
    assert {row["path"] for row in evidence["files"]} >= {
        "site.yaml",
        "world.xml",
        "sources/arm.xml",
        "calib/scene_camera.json",
    }
    with pytest.raises(FileExistsError):
        load_scene(_write_scene(tmp_path)).compile(
            backend="mujoco", output_dir=tmp_path / "build"
        )


def test_compiled_scene_opens_as_an_ordinary_rgbd_sdk_site(tmp_path):
    pytest.importorskip("mujoco")
    build = load_scene(_write_scene(tmp_path)).compile(
        backend="mujoco", output_dir=tmp_path / "runtime-build"
    )
    with waddle_sdk.load_site(build.site_path).open(
        console=False, _testing=True
    ) as session:
        sample = session._require().wait_camera("scene_camera", timeout_s=2.0)
        assert sample is not None
        assert sample.rgb.shape == (48, 64, 3)
        assert sample.depth is not None
        assert sample.depth.shape == (48, 64)
        assert session.describe()["robot"]["name"] == "portable-cell"


def test_compiler_refuses_envelope_widening_and_escaping_assets(tmp_path):
    with pytest.raises(SceneValidationError, match="widens the URDF limit"):
        load_scene(_write_scene(tmp_path, lower=-2.0)).compile(
            backend="mujoco", output_dir=tmp_path / "bad-build"
        )

    path = _write_scene(tmp_path)
    path.write_text(path.read_text().replace("robot.urdf", "../robot.urdf"))
    with pytest.raises((ScenePathError, SceneValidationError), match="beneath|match"):
        load_scene(path)


def test_initializer_refuses_relative_assets_that_escape_the_urdf_directory(
    tmp_path,
):
    source_dir = tmp_path / "source"
    source_dir.mkdir()
    (tmp_path / "outside.stl").write_bytes(b"not a mesh")
    urdf = source_dir / "robot.urdf"
    urdf.write_text(
        """<robot name="escape"><link name="base"><visual><geometry>
        <mesh filename="../outside.stl"/></geometry></visual></link></robot>""",
        encoding="utf-8",
    )
    with pytest.raises(ScenePathError, match="relative|escapes"):
        initialize_scene(urdf, output_dir=tmp_path / "initialized")


def test_mujoco_compiler_refuses_a_false_non_world_base_frame(tmp_path):
    path = _write_scene(tmp_path)
    path.write_text(path.read_text().replace("base_frame: world", "base_frame: base"))
    with pytest.raises(SceneValidationError, match="base_frame must be 'world'"):
        load_scene(path).compile(backend="mujoco", output_dir=tmp_path / "bad-base")


def test_sim_cli_validates_and_compiles(tmp_path, capsys):
    pytest.importorskip("mujoco")
    scene = _write_scene(tmp_path)
    assert cli.main(["sim", "validate", str(scene), "--json"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["scene_id"] == "portable-cell"
    assert payload["cameras"] == ["scene_camera"]

    output = tmp_path / "cli-build"
    assert (
        cli.main(
            [
                "sim",
                "compile",
                str(scene),
                "--backend",
                "mujoco",
                "--output",
                str(output),
                "--seed",
                "19",
            ]
        )
        == 0
    )
    assert (output / "site.yaml").is_file()
    evidence = json.loads((output / "resolved-scene.json").read_text())
    assert evidence["seed"] == 19


def test_sim_init_turns_a_urdf_into_an_editable_compilable_bundle(tmp_path, capsys):
    pytest.importorskip("mujoco")
    source = _write_urdf(tmp_path)
    initialized = tmp_path / "initialized"
    assert (
        cli.main(
            [
                "sim",
                "init",
                str(source),
                "--output",
                str(initialized),
                "--part",
                "arm",
                "--tool-link",
                "tool_link",
                "--color",
                "0.1",
                "0.2",
                "0.3",
                "1.0",
            ]
        )
        == 0
    )
    assert "next: waddle-sdk sim compile" in capsys.readouterr().out
    scene_path = initialized / "scene.yaml"
    initialized_document = yaml.safe_load(scene_path.read_text())
    assert initialized_document["robots"]["arm"]["appearance"]["default"]["rgba"] == [
        0.1,
        0.2,
        0.3,
        1.0,
    ]
    assert initialized_document["robots"]["arm"]["tool"]["link"] == "tool_link"
    build = load_scene(scene_path).compile(
        backend="mujoco", output_dir=tmp_path / "initialized-build"
    )
    assert build.site_path.is_file()


def test_shipped_portable_rgbd_example_compiles(tmp_path):
    pytest.importorskip("mujoco")
    source = SDK / "examples" / "portable-simulation" / "scene.yaml"
    scene = load_scene(source)
    assert set(scene.manifest["cameras"]) == {"scene", "wrist"}
    build = scene.compile(backend="mujoco", output_dir=tmp_path / "example-build")
    site = waddle_sdk.load_site(build.site_path)
    assert site.manifest["cameras"]["wrist"]["mount"] == {
        "kind": "wrist",
        "part": "arm",
    }
    scene_artifact = json.loads((build.output_dir / "calib" / "scene.json").read_text())
    wrist_artifact = json.loads(
        (build.output_dir / "calib" / "wrist_mount.json").read_text()
    )
    assert scene_artifact["to_frame"] == "world"
    assert wrist_artifact["camera"] == "wrist"
    assert wrist_artifact["part"] == "arm"
    assert wrist_artifact["from_frame"] == "wrist_optical"
    assert wrist_artifact["to_frame"] == "tcp"
    assert [row[3] for row in wrist_artifact["matrix"]] == pytest.approx(
        [0.0, 0.0, 0.04, 1.0]
    )


def test_installed_scene_compiler_short_name_uses_public_entry_points(monkeypatch):
    def compiler(*, config, output_dir):
        del config, output_dir

    class EntryPoint:
        def load(self):
            return compiler

    class EntryPoints:
        def select(self, *, group, name):
            assert group == "waddle_sdk.simulation_compilers"
            assert name == "external_sim"
            return (EntryPoint(),)

    monkeypatch.setattr(scene_api.metadata, "entry_points", EntryPoints)
    assert scene_api.resolve_scene_compiler("external_sim") is compiler


def test_sim_backend_listing_does_not_import_installed_plugins(monkeypatch, capsys):
    class EntryPoint:
        name = "external_sim"

        def load(self):
            raise AssertionError("listing must not load an entry point")

    class EntryPoints:
        def select(self, *, group):
            assert group in {
                "waddle_sdk.simulation_backends",
                "waddle_sdk.simulation_compilers",
            }
            return (EntryPoint(),)

    monkeypatch.setattr(scene_api.metadata, "entry_points", EntryPoints)
    monkeypatch.setattr(
        "waddle_sdk.simulation.metadata.entry_points",
        EntryPoints,
    )
    assert cli.main(["sim", "backends", "--json"]) == 0
    rows = json.loads(capsys.readouterr().out)["backends"]
    assert rows == [
        {"compiler": True, "name": "external_sim", "runtime": True},
        {"compiler": True, "name": "mujoco", "runtime": True},
        {"compiler": False, "name": "ros2", "runtime": True},
    ]
