"""Imported worlds use reference actuation, cameras and deterministic resets."""

import hashlib
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
from waddle_sdk.simulators.mujoco import Engine
from waddle_sdk.simulators.native_scene import camera_parameters
from waddle_sdk.simulators.scene import load_scene, make_site


def native_bundle(tmp_path):
    mj = pytest.importorskip("mujoco")
    _, config = make_site(
        "native-import",
        backend="mujoco",
        robot="yam",
        environment="two_cubes",
        arms=2,
        width=160,
        height=120,
    )
    scratch = tmp_path / "reference"
    scratch.mkdir()
    engine = Engine(config, scratch)
    root = ET.parse(scratch / "scene.xml").getroot()
    assets = tmp_path / "assets"
    assets.mkdir()
    for element in root.iter():
        filename = element.get("file")
        if filename:
            path = Path(filename)
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            dest = assets / (digest + path.suffix)
            shutil.copyfile(path, dest)
            element.set("file", str(dest.relative_to(tmp_path)))
    keys = root.find("keyframe")
    if keys is None:
        keys = ET.SubElement(root, "keyframe")
    ET.SubElement(
        keys,
        "key",
        name="initial",
        qpos=" ".join(map(str, engine.data.qpos)),
        ctrl=" ".join(map(str, engine.data.ctrl)),
    )
    (tmp_path / "world.xml").write_text(ET.tostring(root, encoding="unicode"))
    engine.close()
    config["environment"] = "imported-cell"
    config["native_scene"] = {
        "model": "world.xml",
        "initial_keyframe": "initial",
        "files": {
            p.relative_to(tmp_path).as_posix(): hashlib.sha256(
                p.read_bytes()
            ).hexdigest()
            for p in [tmp_path / "world.xml", *assets.iterdir()]
        },
        "prefixes": {"left": "left__", "right": "right__"},
        "camera_names": {name: name for name in config["cameras"]},
        "pose_groups": [
            {"bodies": [name], "translation_xy_m": 0.015, "yaw_rad": 0.1}
            for name in ("cube_1", "cube_2")
        ],
    }
    model = mj.MjModel.from_xml_path(str(tmp_path / "world.xml"))
    data = mj.MjData(model)
    mj.mj_resetDataKeyframe(model, data, model.key("initial").id)
    mj.mj_forward(model, data)
    for name, row in config["cameras"].items():
        tcp = (
            None
            if row["mount"]["kind"] == "scene"
            else row["mount"]["part"] + "__tcp_site"
        )
        intr, pose = camera_parameters(
            model, data, name, width=160, height=120, tcp_site=tcp
        )
        row["intrinsics"].update(intr)
        row["transform"] = pose
    (tmp_path / "simulation.json").write_text(json.dumps(config))
    return config


def test_native_world_preserves_motion_camera_and_seeded_reset(tmp_path):
    native_bundle(tmp_path)
    _, config = load_scene(tmp_path, "simulation.json")
    engine = Engine(config, tmp_path)
    try:
        initial = engine.data.qpos.copy()
        robot, _ = engine.read("left")
        goal = robot.copy()
        goal[0] += 0.1
        engine.write("left", goal)
        for _ in range(500):
            engine.step()
        assert engine.read("left")[0][0] == pytest.approx(goal[0], abs=0.01)
        rgb, depth = engine.capture("left_wrist")
        assert rgb.shape == (120, 160, 3) and depth.shape == (120, 160)
        assert depth.max() > 0
        engine.reset()
        np.testing.assert_array_equal(engine.data.qpos, initial)
        assert engine.evaluation_reset(seed=27)
        first = engine.data.qpos.copy()
        assert engine.evaluation_reset(seed=27)
        np.testing.assert_array_equal(engine.data.qpos, first)
        assert not np.array_equal(first, initial)
        assert engine.evaluation_reset(seed=0)
        np.testing.assert_array_equal(engine.data.qpos, initial)
    finally:
        engine.close()


def test_native_world_refuses_a_motor_in_a_position_control_mapping(tmp_path):
    config = native_bundle(tmp_path)
    path = tmp_path / "world.xml"
    root = ET.parse(path).getroot()
    actuator = root.find("actuator/position[@name='left__joint2']")
    actuator.tag = "motor"
    actuator.attrib.pop("kp")
    actuator.attrib.pop("kv")
    path.write_text(ET.tostring(root, encoding="unicode"))
    config["native_scene"]["files"]["world.xml"] = hashlib.sha256(
        path.read_bytes()
    ).hexdigest()
    (tmp_path / "simulation.json").write_text(json.dumps(config))
    _, loaded = load_scene(tmp_path, "simulation.json")
    with pytest.raises(
        ValueError, match="native actuator left__joint2.*position servo"
    ):
        Engine(loaded, tmp_path)


def test_changed_native_files_and_projection_are_refused(tmp_path):
    config = native_bundle(tmp_path)
    _, loaded = load_scene(tmp_path, "simulation.json")
    loaded["cameras"]["scene"]["intrinsics"]["fx"] += 10
    with pytest.raises(ValueError, match="projection differs"):
        Engine(loaded, tmp_path)
    (tmp_path / config["native_scene"]["model"]).write_text("changed")
    with pytest.raises(ValueError, match="digest differs"):
        load_scene(tmp_path, "simulation.json")


@pytest.mark.parametrize("imported_gains", [False, True])
def test_native_velocity_hint_matches_each_compiled_servo(tmp_path, imported_gains):
    config = native_bundle(tmp_path)
    gains = {"left": (150.0, 5.0), "right": (240.0, 3.0)}
    if imported_gains:
        path = tmp_path / "world.xml"
        root = ET.parse(path).getroot()
        for part, (kp, kd) in gains.items():
            actuator = root.find(f"actuator/position[@name='{part}__joint2']")
            actuator.set("kp", str(kp))
            actuator.set("kv", str(kd))
        path.write_text(ET.tostring(root, encoding="unicode"))
        config["native_scene"]["files"]["world.xml"] = hashlib.sha256(
            path.read_bytes()
        ).hexdigest()
        (tmp_path / "simulation.json").write_text(json.dumps(config))
    _, loaded = load_scene(tmp_path, "simulation.json")
    engine = Engine(loaded, tmp_path)
    try:
        for part in gains:
            position, _ = engine.read(part)
            goal = position.copy()
            goal[1] += 0.01
            velocity = np.zeros_like(goal)
            velocity[1] = 0.4
            velocity[-1] = 0.5  # Hand velocity is deliberately not forwarded.
            actuator = engine.model.actuator(part + "__joint2")
            kp, kd = float(actuator.gainprm[0]), float(-actuator.biasprm[2])
            engine.data.qvel[engine._dofs[part][1]] = 0.1
            engine.write(part, goal)
            engine.mj.mj_forward(engine.model, engine.data)
            baseline = float(engine.data.actuator(part + "__joint2").force[0])
            hand_control = engine.data.actuator(part + "__joint7").ctrl.copy()
            engine.write(part, goal, velocity)
            engine.mj.mj_forward(engine.model, engine.data)
            actual = float(engine.data.actuator(part + "__joint2").force[0])
            assert actual - baseline == pytest.approx(kd * velocity[1], abs=1e-10)
            assert actual == pytest.approx(kp * 0.01 + kd * (0.4 - 0.1))
            np.testing.assert_array_equal(
                engine.data.actuator(part + "__joint7").ctrl, hand_control
            )
            engine.write(part, goal)
            engine.mj.mj_forward(engine.model, engine.data)
            assert engine.data.actuator(part + "__joint2").force[0] == pytest.approx(
                baseline
            )
    finally:
        engine.close()
