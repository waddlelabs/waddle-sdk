"""Changing scene roots can reuse immutable virtual assets without stale models."""

from concurrent.futures import ThreadPoolExecutor

import numpy as np
import pytest

mj = pytest.importorskip("mujoco")

from waddle_sdk.simulators import source_buffers
from waddle_sdk.simulators.compiled_model import load_model


def inputs(x=0, scale=1):
    return {
        "model.xml": f'''<mujoco><asset><mesh name="m" file="mesh.obj"/></asset>
          <worldbody><body pos="{x} 0 1"><freejoint/>
          <geom type="mesh" mesh="m" mass="2"/></body></worldbody></mujoco>'''.encode(),
        "mesh.obj": f"v 0 0 0\nv {scale} 0 0\nv 0 1 0\nv 0 0 1\nf 1 2 3\nf 1 4 2\nf 1 3 4\nf 2 4 3\n".encode(),
    }


@pytest.fixture
def buffers(tmp_path, monkeypatch):
    if not hasattr(mj, "MjVfs"):
        pytest.skip("explicit virtual files unavailable")
    cache = source_buffers.SourceBuffers(max_entries=1)
    monkeypatch.setattr(source_buffers, "cache", cache)
    monkeypatch.setenv("WADDLE_SIMULATION_MODEL_CACHE", str(tmp_path / "cache"))
    yield cache
    cache.close()


def test_revised_roots_and_assets_keep_independent_geometry_and_state(buffers):
    first = load_model("model.xml", inputs(), cache="")
    old = mj.MjData(first)
    old.qpos[0] = 42
    first.geom_rgba[:] = 0
    second = load_model("model.xml", inputs(x=3), cache="")
    assert first is not second
    assert second.qpos0[0] == 3 and second.geom_rgba[0, 3] == 1
    assert old.qpos[0] == 42
    assert len(buffers.entries) == 1
    previous = next(iter(buffers.entries.values()))[1]
    third = load_model("model.xml", inputs(x=5, scale=2), cache="")
    assert third.qpos0[0] == 5
    assert third.geom_rbound[0] > second.geom_rbound[0]
    with pytest.raises(RuntimeError, match="closed"):
        previous.__contains__("mesh.obj")
    data = mj.MjData(second)
    mj.mj_step(second, data)
    assert data.qpos[2] < second.qpos0[2]


def test_failed_compile_discards_virtual_buffers_and_next_valid_scene_still_loads(
    buffers,
):
    reference = load_model("model.xml", inputs(), cache="")
    broken = inputs()
    broken["model.xml"] = broken["model.xml"].replace(b'mesh="m"', b'mesh="absent"')
    with pytest.raises(ValueError, match="absent"):
        load_model("model.xml", broken, cache="")
    assert not buffers.entries
    restored = load_model("model.xml", inputs(), cache="")
    np.testing.assert_array_equal(restored.body_mass, reference.body_mass)
    np.testing.assert_array_equal(restored.geom_size, reference.geom_size)


def test_oversized_inputs_compile_without_retaining_virtual_buffers(buffers):
    buffers.max_bytes = 1
    model = load_model("model.xml", inputs(x=2), cache="")
    assert model.qpos0[0] == 2 and not buffers.entries


def test_concurrent_revisions_cannot_swap_each_others_root(buffers):
    def compile(index):
        model = load_model("model.xml", inputs(x=index), cache="")
        return model.qpos0[0]

    with ThreadPoolExecutor(max_workers=4) as workers:
        assert list(workers.map(compile, range(12))) == list(range(12))
