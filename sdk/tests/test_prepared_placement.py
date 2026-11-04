"""Reposition private scene preparations without recompiling unchanged geometry."""

from copy import deepcopy

import numpy as np
import pytest

mj = pytest.importorskip("mujoco")

from waddle_sdk.simulators import source_buffers
from waddle_sdk.simulators.compiled_model import load_model


def source(
    position=".2 0 .3",
    *,
    extra="",
    body_extra="",
    body_joint="<freejoint name='object_free'/>",
):
    return {
        "world.xml": f"""<mujoco><compiler angle="radian"/><asset/>
          {extra}<worldbody>
          <body name="robot"><joint name="hinge"/><geom size=".1"/></body>
          <body name="prop_object" pos="{position}" quat=".7 .1 .2 .3">
            {body_joint}<geom name="object_geom" type="box" size=".04 .06 .02"/>
            {body_extra}</body></worldbody><contact/>
          <keyframe><key name="initial"/></keyframe></mujoco>""".encode()
    }


@pytest.fixture
def buffers(tmp_path, monkeypatch):
    cache = source_buffers.SourceBuffers()
    monkeypatch.setattr(source_buffers, "cache", cache)
    monkeypatch.setenv("WADDLE_SIMULATION_MODEL_CACHE", str(tmp_path / "cache"))
    yield cache
    cache.close()


def compile_source(contents):
    return load_model("world.xml", contents, cache="", _authored=True)


def same_model(actual, expected):
    for name in dir(expected):
        value = getattr(expected, name)
        if isinstance(value, np.ndarray):
            np.testing.assert_array_equal(getattr(actual, name), value, err_msg=name)
    for name in ("center", "extent", "meaninertia", "meanmass", "meansize"):
        np.testing.assert_array_equal(
            getattr(actual.stat, name), getattr(expected.stat, name)
        )
    a, b = mj.MjData(actual), mj.MjData(expected)
    for _ in range(10):
        mj.mj_step(actual, a)
        mj.mj_step(expected, b)
        for name in ("qpos", "qvel", "qacc", "qfrc_constraint", "xpos", "xquat"):
            np.testing.assert_array_equal(
                getattr(a, name), getattr(b, name), err_msg=name
            )


def test_placement_reuses_whole_model_and_isolated_source_state(buffers, monkeypatch):
    original = compile_source(source())
    # The returned model belongs to its caller; neither its state nor later
    # authoring of the initial key can mutate the retained preparation.
    original.geom_rgba[:] = 0
    original.qpos0[:] = 3
    original.key_qpos[:] = 9
    saved = deepcopy(original)

    def unexpected_compile(*args, **kwargs):
        pytest.fail("placing an unchanged free object compiled the scene again")

    for position in (".2 0 .3", ".6 -.2 .15", "-.75 .75 .8", ".02 -.75 .02"):
        contents = source(position)
        expected = mj.MjModel.from_xml_string(contents["world.xml"].decode())
        with monkeypatch.context() as cached:
            cached.setattr(mj.MjSpec, "compile", unexpected_compile)
            actual = compile_source(contents)
        same_model(actual, expected)
        np.testing.assert_array_equal(original.geom_rgba, saved.geom_rgba)
        np.testing.assert_array_equal(original.qpos0, saved.qpos0)
        np.testing.assert_array_equal(original.key_qpos, saved.key_qpos)


def test_identical_linked_scene_reuses_private_template(buffers, monkeypatch):
    contents = source(
        extra='<statistic extent="8"/><equality><weld body1="robot" body2="prop_object"/></equality>'
    )
    compile_source(contents)
    expected = mj.MjModel.from_xml_string(contents["world.xml"].decode())

    def unexpected_compile(*args, **kwargs):
        pytest.fail("identical complete scene preparation was compiled again")

    monkeypatch.setattr(mj.MjSpec, "compile", unexpected_compile)
    same_model(compile_source(contents), expected)


@pytest.mark.parametrize(
    "options",
    [
        {"extra": '<statistic extent="8" center="1 2 3"/>'},
        {"extra": '<equality><weld body1="robot" body2="prop_object"/></equality>'},
        {"body_extra": '<site name="object_site"/>'},
        {"body_extra": '<body name="child"><geom size=".01"/></body>'},
        {"body_joint": '<joint name="object_free" type="slide"/>'},
        {"body_joint": ""},
        {
            "extra": '<actuator><general joint="object_free" gear="1 0 0 0 0 0"/></actuator>'
        },
    ],
)
def test_other_dependencies_retain_full_native_preparation(
    buffers, monkeypatch, options
):
    compile_source(source(**options))
    contents = source(".5 -.3 .2", **options)
    expected = mj.MjModel.from_xml_string(contents["world.xml"].decode())
    calls = []
    compile_model = mj.MjSpec.compile

    def recorded(*args, **kwargs):
        calls.append(True)
        return compile_model(*args, **kwargs)

    monkeypatch.setattr(mj.MjSpec, "compile", recorded)
    actual = compile_source(contents)
    assert calls
    same_model(actual, expected)


def test_model_payload_counts_towards_the_existing_cache_bound(buffers, monkeypatch):
    inputs = source()
    buffers.max_bytes = (
        sum(len(name.encode()) + len(data) for name, data in inputs.items()) + 512
    )
    compile_source(inputs)
    calls = []
    compile_model = mj.MjSpec.compile

    def recorded(*args, **kwargs):
        calls.append(True)
        return compile_model(*args, **kwargs)

    monkeypatch.setattr(mj.MjSpec, "compile", recorded)
    actual = compile_source(source(".6 .2 .1"))
    assert calls
    same_model(
        actual, mj.MjModel.from_xml_string(source(".6 .2 .1")["world.xml"].decode())
    )
    assert sum(entry[0] for entry in buffers.entries.values()) <= buffers.max_bytes
