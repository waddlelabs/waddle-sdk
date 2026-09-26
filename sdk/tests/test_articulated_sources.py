"""Portable robot assemblies retain native geometry, hand motion and drives."""

import xml.etree.ElementTree as ET

import numpy as np
import pytest
from waddle_sdk.simulators import articulated_model_sources
from waddle_sdk.simulators.description import description
from waddle_sdk.simulators.model import mjcf
from waddle_sdk.simulators.scene import make_site, profile


@pytest.mark.parametrize("robot", ["so101", "yam", "xarm7"])
def test_portable_articulated_assembly_matches_reference_robot(tmp_path, robot):
    mj = pytest.importorskip("mujoco")
    bundle = articulated_model_sources(robot, part_name="arm")
    repeated = articulated_model_sources(robot, part_name="arm")
    assert bundle.model == repeated.model and bundle.assets == repeated.assets
    assert bundle.licenses and bundle.provenance["hand_joints"]
    for name, data in {
        "model.xml": bundle.model,
        **bundle.assets,
        **bundle.licenses,
    }.items():
        target = tmp_path / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
    xml = ET.fromstring(bundle.model)
    assert not xml.findall(".//camera") and not xml.findall(".//light")
    for node in xml.iter():
        if node.get("file"):
            assert node.get("file") in bundle.assets
    exported = mj.MjModel.from_xml_path(str(tmp_path / "model.xml"))
    _, config = make_site(
        "source-test", backend="mujoco", robot=robot, environment="two_cubes"
    )
    part = next(iter(config["parts"]))
    config["parts"][part] = {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}
    full = mj.MjModel.from_xml_string(mjcf(profile(robot), config))
    native = description(robot)
    links = native.native_links()
    assert exported.nq == len(bundle.joint_names) + len(native.hand_names)
    assert exported.nu == full.nu
    assert exported.neq == full.neq and exported.ntendon == full.ntendon
    assert all(
        exported.body(i).name not in {"table", "cube_a", "cube_b"}
        for i in range(exported.nbody)
    )
    for position in (0.2, 0.8):
        states = []
        for model in (exported, full):
            data = mj.MjData(model)
            for name, q in zip(
                bundle.joint_names, profile(robot).home[:-1], strict=True
            ):
                data.joint(name).qpos[0] = q
            for name in native.hand_names:
                data.joint(name).qpos[0] = native.hand_position(position)
            mj.mj_forward(model, data)
            states.append(data)
        for link in links:
            a, b = exported.body(link.name), full.body(link.name)
            np.testing.assert_allclose(
                states[0].body(link.name).xpos,
                states[1].body(link.name).xpos,
                atol=1e-12,
            )
            np.testing.assert_allclose(
                states[0].body(link.name).xmat,
                states[1].body(link.name).xmat,
                atol=1e-12,
            )
            np.testing.assert_allclose(a.mass, b.mass, atol=1e-12)
            np.testing.assert_allclose(a.inertia, b.inertia, atol=1e-12)
            assert a.geomnum == b.geomnum
        np.testing.assert_allclose(
            states[0].site(bundle.tcp_site).xpos,
            states[1].site(bundle.tcp_site).xpos,
            atol=1e-12,
        )
    for i in range(exported.nu):
        name = exported.actuator(i).name
        for field in ("gainprm", "biasprm", "forcerange", "ctrlrange", "trntype"):
            np.testing.assert_allclose(
                getattr(exported.actuator(name), field),
                getattr(full.actuator(name), field),
            )
