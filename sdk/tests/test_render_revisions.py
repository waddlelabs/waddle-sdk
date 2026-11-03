"""Scene revisions preserve compatible GL resources and acquire new pixels."""

import pickle
from itertools import pairwise

import numpy as np
import pytest


def model_for(mj, *, samples=4, object_xml=""):
    return mj.MjModel.from_xml_string(
        f"""<mujoco>
        <statistic extent="2" center="0 0 0"/>
        <visual><quality offsamples="{samples}"/></visual>
        <asset>
          <texture name="checker" type="2d" builtin="checker"
                   width="16" height="16" rgb1=".7 .7 .7" rgb2=".2 .2 .2"/>
          <material name="floor" texture="checker"/>
          <mesh name="tetra" vertex="0 0 0  .2 0 0  0 .2 0  0 0 .2"/>
        </asset>
        <worldbody>
          <light pos="0 0 3"/>
          <camera name="view" pos="0 0 2"/>
          <geom type="plane" size="2 2 .1" material="floor"/>
          <geom type="mesh" mesh="tetra" pos="-.5 0 .1"/>
          {object_xml}
        </worldbody></mujoco>"""
    )


def snapshot(mj, model, generation):
    data = mj.MjData(model)
    if model.nq:
        # Captured live state differs from the authored body position.
        data.qpos[0] += 0.1
    data.time = generation + 0.25
    state = np.empty(mj.mj_stateSize(model, mj.mjtState.mjSTATE_INTEGRATION))
    mj.mj_getState(model, data, state, mj.mjtState.mjSTATE_INTEGRATION)
    return {"model": pickle.dumps(model), "state": state, "generation": generation}


def renderer_config(depth):
    return {
        "cameras": {
            "view": {
                "stream": {"width": 64, "height": 48},
                "options": {"depth": depth},
                "intrinsics": {"depth_scale_mm": 1},
            }
        }
    }


@pytest.mark.parametrize("samples,depth", [(0, True), (4, True), (4, False)])
def test_primitive_revisions_keep_contexts_and_match_fresh_rgb_depth(samples, depth):
    mj = pytest.importorskip("mujoco")
    from waddle_sdk.simulators.render_worker import SnapshotRenderer

    renderer = SnapshotRenderer(renderer_config(depth))
    retained = None
    pictures = []
    try:
        for generation, (position, size, rgba) in enumerate(
            [
                (None, 0.15, "1 0 0 1"),
                ("0 0 .2", 0.15, "1 0 0 1"),
                (".3 0 .2", 0.15, "1 0 0 1"),
                (".3 0 .2", 0.25, "1 0 0 1"),
                (".3 0 .2", 0.25, "0 1 0 1"),
                (None, 0.15, "1 0 0 1"),
            ]
        ):
            prop = (
                ""
                if position is None
                else (
                    f'<body pos="{position}"><freejoint/><geom type="box" '
                    f'size="{size} {size} .15" rgba="{rgba}"/></body>'
                )
            )
            model = model_for(mj, samples=samples, object_xml=prop)
            current = snapshot(mj, model, generation)
            renderer.restore(current)
            actual = renderer.capture("view")
            pictures.append(actual[0])
            if retained is None:
                retained = renderer.renderers["view"]
            else:
                assert renderer.renderers["view"] is retained
                assert not retained.closed
            assert renderer.data.time == generation + 0.25
            assert retained.rgb._model is renderer.model
            if retained.depth is not None:
                assert retained.depth._model is renderer.model
            fresh = SnapshotRenderer(renderer_config(depth))
            try:
                fresh.restore(current)
                expected = fresh.capture("view")
                for observed, reference in zip(actual, expected, strict=True):
                    np.testing.assert_array_equal(observed, reference)
            finally:
                fresh.close()
        assert all(not np.array_equal(a, b) for a, b in pairwise(pictures))
        np.testing.assert_array_equal(pictures[0], pictures[-1])
    finally:
        renderer.close()
    assert retained.closed


@pytest.mark.parametrize(
    "changed",
    [
        "mesh_vert",
        "tex_data",
        "geom_size",
        "geom_matid",
        "mat_rgba",
        "extent",
        "offsamples",
    ],
)
def test_changed_uploaded_resources_rebuild_and_match_fresh_render(changed):
    mj = pytest.importorskip("mujoco")
    from waddle_sdk.simulators.render_worker import SnapshotRenderer

    renderer = SnapshotRenderer(renderer_config(True))
    fresh = SnapshotRenderer(renderer_config(True))
    try:
        model = model_for(mj)
        renderer.restore(snapshot(mj, model, 0))
        renderer.capture("view")
        previous = renderer.renderers["view"]
        if changed == "extent":
            model.stat.extent = 3
        elif changed == "offsamples":
            model.vis.quality.offsamples = 0
        elif changed == "geom_matid":
            model.geom_matid[0] = -1
        else:
            getattr(model, changed).flat[0] += 1
        current = snapshot(mj, model, 1)
        renderer.restore(current)
        actual = renderer.capture("view")
        assert previous.closed
        assert renderer.renderers["view"] is not previous
        fresh.restore(current)
        for observed, reference in zip(actual, fresh.capture("view"), strict=True):
            np.testing.assert_array_equal(observed, reference)
    finally:
        renderer.close()
        fresh.close()
