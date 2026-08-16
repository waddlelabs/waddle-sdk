"""Fake-runtime contract tests for the manifest-native MuJoCo adapter."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
from typing import ClassVar

import numpy as np
import pytest
from waddle_sdk.robots import mujoco
from waddle_sdk.robots.site import PartConfig


class _ObjectKind:
    mjOBJ_JOINT = 1
    mjOBJ_ACTUATOR = 2
    mjOBJ_SITE = 3
    mjOBJ_BODY = 4


class _JointKind:
    mjJNT_FREE = 0
    mjJNT_SLIDE = 2
    mjJNT_HINGE = 3


class FakeModel:
    loaded_paths: ClassVar[list[str]] = []
    mutate = None

    def __init__(self) -> None:
        self.jnt_qposadr = np.asarray([0, 1])
        self.jnt_dofadr = np.asarray([0, 1])
        self.jnt_type = np.asarray([_JointKind.mjJNT_HINGE] * 2)
        self.jnt_limited = np.asarray([True, True])
        self.jnt_range = np.asarray([[-1.0, 1.0], [-2.0, 2.0]])
        self.opt = SimpleNamespace(timestep=0.01)
        self.names = {
            _ObjectKind.mjOBJ_JOINT: {"shoulder": 0, "elbow": 1},
            _ObjectKind.mjOBJ_ACTUATOR: {"shoulder_motor": 0, "elbow_motor": 1},
            _ObjectKind.mjOBJ_SITE: {"tool": 0},
            _ObjectKind.mjOBJ_BODY: {"upper": 0, "forearm": 1},
        }

    @classmethod
    def from_xml_path(cls, path: str):
        cls.loaded_paths.append(path)
        model = cls()
        if cls.mutate is not None:
            cls.mutate(model)
        return model


class FakeData:
    def __init__(self, _model: FakeModel) -> None:
        self.qpos = np.zeros(2)
        self.qvel = np.zeros(2)
        self.ctrl = np.zeros(2)
        self.site_xpos = np.zeros((1, 3))
        self.site_xmat = np.asarray([np.eye(3).reshape(-1)])
        self.xpos = np.zeros((2, 3))


class FakeMujoco:
    MjModel = FakeModel
    MjData = FakeData
    mjtObj = _ObjectKind
    mjtJoint = _JointKind
    steps = 0

    @staticmethod
    def mj_name2id(model: FakeModel, kind: int, name: str) -> int:
        return model.names.get(kind, {}).get(name, -1)

    @staticmethod
    def mj_forward(_model: FakeModel, data: FakeData) -> None:
        shoulder, elbow = data.qpos
        data.site_xpos[0] = (shoulder, elbow, shoulder + elbow)
        data.xpos[0] = (shoulder, 0.0, 0.0)
        data.xpos[1] = (shoulder, elbow, 0.0)

    @classmethod
    def mj_step(cls, model: FakeModel, data: FakeData) -> None:
        cls.steps += 1
        data.qvel[:] = data.ctrl - data.qpos
        data.qpos[:] = data.ctrl
        cls.mj_forward(model, data)


@pytest.fixture(autouse=True)
def _reset_fake_runtime(monkeypatch):
    FakeModel.loaded_paths.clear()
    FakeModel.mutate = None
    FakeMujoco.steps = 0
    monkeypatch.setattr(mujoco, "_mujoco_module", lambda: FakeMujoco)


def _config(tmp_path: Path, **changes) -> PartConfig:
    model = tmp_path / "models" / "cell.xml"
    model.parent.mkdir(exist_ok=True)
    model.write_text("<mujoco/>", encoding="utf-8")
    values = {
        "name": "simulation",
        "posture": "supervised",
        "connection": {"model": "models/cell.xml"},
        "joint_limits": {"shoulder": [-0.8, 0.8], "elbow": [-1.5, 1.5]},
        "workspace_bounds": {},
        "envelope": {"static_keepouts": [], "self_collision": {}},
        "options": {
            "actuator_names": ["shoulder_motor", "elbow_motor"],
            "home": [0.2, -0.2],
            "rate_hz": 100,
            "max_joint_speed_per_s": 10,
            "tool_site": "tool",
            "collision_frame": "site",
            "collision_bodies": [
                {"name": "upper", "body": "upper", "radius_m": 0.08},
                {"name": "forearm", "body": "forearm", "radius_m": 0.06},
            ],
        },
        "site_root": tmp_path,
    }
    values.update(changes)
    return PartConfig(**values)


def test_factory_is_lazy_and_steps_explicit_joint_targets(tmp_path):
    rig = mujoco.arm(config=_config(tmp_path))
    assert FakeModel.loaded_paths == []
    assert [joint.name for joint in rig.robot().action_space.joints] == [
        "shoulder",
        "elbow",
    ]

    driver = rig.arms()[""].driver
    assert FakeModel.loaded_paths == [str(tmp_path / "models" / "cell.xml")]
    position, velocity = driver.read()
    assert position.tolist() == [0.2, -0.2]
    assert velocity.tolist() == [0.0, 0.0]

    driver.write(np.asarray([0.4, 0.3]))
    driver.step(0.02)
    assert FakeMujoco.steps == 2
    assert driver.read()[0].tolist() == [0.4, 0.3]
    driver.step(0.0)
    assert FakeMujoco.steps == 2


def test_scratch_fk_and_body_spheres_do_not_mutate_live_state(tmp_path):
    arm = mujoco.arm(config=_config(tmp_path)).arms()[""]
    driver = arm.driver
    position, rotation = driver.forward_kinematics([0.5, 0.25])
    assert position.tolist() == [0.5, 0.25, 0.75]
    assert rotation.tolist() == np.eye(3).tolist()
    spheres = driver.collision_spheres([0.5, 0.25])
    assert [(row.name, row.center_m, row.radius_m) for row in spheres] == [
        ("upper", (0.5, 0.0, 0.0), 0.08),
        ("forearm", (0.5, 0.25, 0.0), 0.06),
    ]
    assert driver.read()[0].tolist() == [0.2, -0.2]


def test_estop_latches_and_invalid_durations_fail(tmp_path):
    driver = mujoco.arm(config=_config(tmp_path)).arms()[""].driver
    driver.estop()
    assert driver.estopped
    with pytest.raises(RuntimeError, match="e-stopped"):
        driver.write(np.zeros(2))
    assert not driver.home([0.0, 0.0])
    driver.re_enable()
    assert not driver.estopped
    with pytest.raises(ValueError, match="non-negative"):
        driver.step(-0.01)
    driver.close()
    driver.close()
    with pytest.raises(RuntimeError, match="closed"):
        driver.read()


def test_compiled_model_refuses_wider_or_non_scalar_declarations(tmp_path):
    config = _config(
        tmp_path,
        joint_limits={"shoulder": [-1.1, 0.8], "elbow": [-1.5, 1.5]},
    )
    with pytest.raises(ValueError, match="widen model limits"):
        mujoco.arm(config=config).arms()

    FakeModel.mutate = lambda model: model.jnt_type.__setitem__(
        0, _JointKind.mjJNT_FREE
    )
    with pytest.raises(ValueError, match="not a scalar"):
        mujoco.arm(config=_config(tmp_path)).arms()


@pytest.mark.parametrize("model", ["../cell.xml", "/tmp/cell.xml", "bad\\path.xml"])
def test_model_paths_are_portable_and_beneath_site_root(tmp_path, model):

    original = _config(tmp_path)
    config = PartConfig(**{**original.__dict__, "connection": {"model": model}})
    with pytest.raises(ValueError, match="relative path|site root"):
        mujoco.arm(config=config)
    assert FakeModel.loaded_paths == []


def test_unknown_model_objects_fail_during_lazy_open(tmp_path):
    config = _config(tmp_path)
    config = PartConfig(
        **{
            **config.__dict__,
            "options": {**config.options, "tool_site": "missing"},
        }
    )
    rig = mujoco.arm(config=config)
    with pytest.raises(ValueError, match="no object named 'missing'"):
        rig.arms()
