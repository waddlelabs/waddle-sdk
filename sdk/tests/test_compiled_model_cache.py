"""Verified compiled models avoid repeated parsing without sharing mutable state."""

import numpy as np
import pytest

mj = pytest.importorskip("mujoco")

from waddle_sdk.simulators.compiled_model import load_model


def inputs(x=0):
    return {
        "model.xml": f'''<mujoco><worldbody><body pos="{x} 0 1">
      <freejoint/><geom type="box" size=".1 .2 .3" mass="2"/>
    </body></worldbody></mujoco>'''.encode()
    }


@pytest.mark.parametrize("legacy", [False, True])
def test_unchanged_sources_reuse_compilation_with_independent_model_and_state(
    tmp_path, monkeypatch, legacy
):
    if legacy:
        monkeypatch.delattr(mj, "MjVfs", raising=False)
    first_timing, second_timing = {}, {}
    first = load_model("model.xml", inputs(), cache=tmp_path, timings=first_timing)
    first.geom_rgba[:] = 0
    first.body_pos[1, 0] = 99
    a = mj.MjData(first)
    a.qpos[0] = 42
    second = load_model("model.xml", inputs(), cache=tmp_path, timings=second_timing)
    assert not first_timing["cache_hit"] and first_timing["native_compile_s"] > 0
    assert second_timing["cache_hit"] and second_timing["native_compile_s"] == 0
    assert second is not first and second.body_pos[1, 0] == 0
    assert second.geom_rgba[0, 3] == 1
    b = mj.MjData(second)
    assert b.qpos[0] == 0
    mj.mj_step(second, b)
    assert b.qpos[2] < second.qpos0[2]
    assert a.qpos[0] == 42


def test_changed_xml_and_mesh_bytes_cannot_reuse_stale_geometry(tmp_path):
    first = load_model("model.xml", inputs(0), cache=tmp_path)
    second = load_model("model.xml", inputs(1), cache=tmp_path)
    assert first.body_pos[1, 0] == 0 and second.body_pos[1, 0] == 1
    xml = b'<mujoco><asset><mesh name="m" file="mesh.obj"/></asset><worldbody><geom type="mesh" mesh="m"/></worldbody></mujoco>'

    def mesh(scale):
        return f"v 0 0 0\nv {scale} 0 0\nv 0 1 0\nv 0 0 1\nf 1 2 3\nf 1 4 2\nf 1 3 4\nf 2 4 3\n".encode()

    small = load_model(
        "model.xml", {"model.xml": xml, "mesh.obj": mesh(1)}, cache=tmp_path
    )
    large = load_model(
        "model.xml", {"model.xml": xml, "mesh.obj": mesh(3)}, cache=tmp_path
    )
    assert large.geom_rbound[0] > small.geom_rbound[0]


def test_virtual_files_preserve_nested_source_geometry_and_native_dynamics(monkeypatch):
    if not hasattr(mj, "MjVfs"):
        pytest.skip("explicit native virtual files unavailable")
    sources = {
        "scene.xml": b'<mujoco><include file="parts/body.xml"/></mujoco>',
        "parts/body.xml": b"""<mujoco>
          <asset><mesh name="shape" file="shape.obj"/></asset>
          <worldbody><body pos="0 0 1"><freejoint/>
            <geom type="mesh" mesh="shape" mass="2"/>
          </body></worldbody></mujoco>""",
        "parts/shape.obj": b"v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 2 3\nf 1 4 2\nf 1 3 4\nf 2 4 3\n",
    }
    current = load_model("scene.xml", sources, cache="")
    with monkeypatch.context() as old:
        old.delattr(mj, "MjVfs")
        legacy = load_model("scene.xml", sources, cache="")
    for name in dir(legacy):
        value = getattr(legacy, name)
        if isinstance(value, np.ndarray):
            np.testing.assert_array_equal(getattr(current, name), value, err_msg=name)
    a, b = mj.MjData(current), mj.MjData(legacy)
    for _ in range(10):
        mj.mj_step(current, a)
        mj.mj_step(legacy, b)
    np.testing.assert_array_equal(a.qpos, b.qpos)
    np.testing.assert_array_equal(a.qvel, b.qvel)


def test_private_cache_corruption_recompiles_original_sources(tmp_path):
    first = load_model("model.xml", inputs(), cache=tmp_path)
    path = next(tmp_path.glob("*.mjb"))
    path.write_bytes(b"not a native compiled model")
    second = load_model("model.xml", inputs(), cache=tmp_path)
    np.testing.assert_array_equal(first.body_pos, second.body_pos)
    assert path.read_bytes() != b"not a native compiled model"


def test_portable_copy_reuses_model_and_missing_assets_keep_original_compile_failure(
    tmp_path,
):
    first = load_model(
        "scene/world.xml", {"scene/world.xml": inputs()["model.xml"]}, cache=tmp_path
    )
    second = load_model("model.xml", inputs(), cache=tmp_path)
    assert len(list(tmp_path.glob("*.mjb"))) == 1
    np.testing.assert_array_equal(first.geom_size, second.geom_size)
    with pytest.raises(ValueError, match="not found|Error opening|could not open"):
        load_model(
            "model.xml",
            {"model.xml": b'<mujoco><include file="missing.xml"/></mujoco>'},
            cache=tmp_path,
        )


def test_native_child_uses_private_compilation_cache_and_keeps_fresh_state(
    tmp_path, monkeypatch
):
    from test_native_scene import native_bundle
    from waddle_sdk.simulators.adapters import World
    from waddle_sdk.simulators.scene import load_scene

    source = tmp_path / "source"
    source.mkdir()
    native_bundle(source)
    _, config = load_scene(source, "simulation.json")
    cache = tmp_path / "cache"
    monkeypatch.setenv("WADDLE_SIMULATION_MODEL_CACHE", str(cache))
    for _ in range(2):
        world = World(config, real_time=False)
        try:
            world.open()
            assert len(list(cache.glob("*.mjb"))) == 1
            positions, _ = world.call("read", "left")
            assert np.isfinite(positions).all()
        finally:
            world.close()
