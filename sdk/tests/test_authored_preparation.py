"""Authored edits reuse preparation without sharing a running native model."""

from copy import deepcopy

import numpy as np
import pytest

mj = pytest.importorskip("mujoco")

from test_playground import scene
from waddle_sdk.simulators import source_buffers
from waddle_sdk.simulators.compiled_model import load_model
from waddle_sdk.simulators.playground import prepare_scene
from waddle_sdk.simulators.scene import make_site


def same_model(actual, expected):
    for name in dir(expected):
        value = getattr(expected, name)
        if isinstance(value, np.ndarray):
            np.testing.assert_array_equal(getattr(actual, name), value, err_msg=name)
    a, b = mj.MjData(actual), mj.MjData(expected)
    for _ in range(10):
        mj.mj_step(actual, a)
        mj.mj_step(expected, b)
    np.testing.assert_array_equal(a.qpos, b.qpos)
    np.testing.assert_array_equal(a.qvel, b.qvel)


@pytest.mark.parametrize(
    "robot,arms,kind,compound",
    [
        ("yam", 1, kind, False)
        for kind in ("block", "bottle", "ball", "cylinder", "bowl", "tray", "target")
    ]
    + [
        ("yam", 1, "block", True),
        ("yam", 2, "bottle", False),
        ("so101", 1, "ball", False),
        ("xarm7", 1, "cylinder", False),
    ],
)
def test_added_object_matches_fresh_preparation_and_keeps_old_model(
    tmp_path, monkeypatch, robot, arms, kind, compound
):
    if not hasattr(mj, "MjVfs"):
        pytest.skip("explicit virtual files unavailable")
    buffers = source_buffers.SourceBuffers()
    monkeypatch.setattr(source_buffers, "cache", buffers)
    monkeypatch.setenv("WADDLE_SIMULATION_MODEL_CACHE", str(tmp_path / "cache"))
    _, config = make_site(
        "prepared",
        backend="mujoco",
        robot=robot,
        arms=arms,
        environment="two_cubes",
        width=96,
        height=72,
    )
    config["render_quality"] = "fast"
    original = scene()
    if not compound:
        original["objects"] = original["objects"][:1]
    first = prepare_scene(config, original, tmp_path / "first", _prefer_handoff=True)
    old_mass = first.model.body_mass.copy()
    first.model.geom_rgba[:] = 0
    edited = deepcopy(original)
    added = next(obj for obj in scene()["objects"] if obj["kind"] == kind)
    added.update(id="added", position=[0.4, 0.1, 0.1], rotation=[10, 20, 30])
    edited["objects"].append(added)

    def unexpected_full_parse(*args, **kwargs):
        pytest.fail("unchanged robot and existing objects were parsed again")

    with monkeypatch.context() as cached:
        cached.setattr(mj.MjSpec, "from_file", unexpected_full_parse)
        second = prepare_scene(
            config, edited, tmp_path / "second", _prefer_handoff=True
        )
    np.testing.assert_array_equal(first.model.body_mass, old_mass)
    assert np.all(first.model.geom_rgba == 0)
    # An independent compiler is the semantic oracle, including every native
    # array, canonical reset, contact parameters and ordinary simulation steps.
    with monkeypatch.context() as uncached:
        uncached.delenv("WADDLE_SIMULATION_MODEL_CACHE")
        expected = prepare_scene(config, edited, tmp_path / "fresh")
    same_model(second.model, expected.model)
    buffers.close()
    assert not buffers.prepared


def test_changed_assets_and_invalid_additions_do_not_reuse_old_preparation(
    tmp_path, monkeypatch
):
    from test_source_buffers import inputs

    if not hasattr(mj, "MjVfs"):
        pytest.skip("explicit virtual files unavailable")
    buffers = source_buffers.SourceBuffers(max_entries=1)
    monkeypatch.setattr(source_buffers, "cache", buffers)
    monkeypatch.setenv("WADDLE_SIMULATION_MODEL_CACHE", str(tmp_path / "cache"))

    def source(scale=1, extra=""):
        data = inputs(scale=scale)
        data["model.xml"] = f"""<mujoco><compiler angle="radian"/>
          <asset><mesh name="m" file="mesh.obj"/></asset>
          <worldbody><body name="prop_a" pos="0 0 1"><freejoint/>
            <geom name="a" type="mesh" mesh="m" mass="2"/></body>{extra}</worldbody>
          <contact/><keyframe><key name="initial"/></keyframe></mujoco>""".encode()
        return data

    first = load_model("model.xml", source(), cache="", _authored=True)
    addition = '<body name="prop_b"><geom type="sphere" name="b" size=".1"/></body>'
    changed = load_model("model.xml", source(2, addition), cache="", _authored=True)
    assert changed.geom_rbound[0] > first.geom_rbound[0]
    invalid = addition.replace('type="sphere"', 'type="mesh" mesh="missing"')
    with pytest.raises(ValueError, match="missing"):
        load_model("model.xml", source(2, invalid), cache="", _authored=True)
    assert not buffers.entries and not buffers.prepared
    fresh = load_model("model.xml", source(2, addition), cache="", _authored=True)
    same_model(changed, fresh)
    buffers.close()


def test_existing_edits_match_full_rebuild_without_reparsing_unchanged_bowl(
    tmp_path, monkeypatch
):
    if not hasattr(mj, "MjVfs"):
        pytest.skip("explicit virtual files unavailable")
    buffers = source_buffers.SourceBuffers()
    monkeypatch.setattr(source_buffers, "cache", buffers)
    monkeypatch.setenv("WADDLE_SIMULATION_MODEL_CACHE", str(tmp_path / "cache"))
    _, config = make_site(
        "edits",
        backend="mujoco",
        robot="yam",
        arms=1,
        environment="two_cubes",
        width=96,
        height=72,
    )
    config["render_quality"] = "fast"
    authored = scene()
    first = prepare_scene(config, authored, tmp_path / "first", _prefer_handoff=True)
    original = first.model.body_pos.copy()

    def unexpected_full_parse(*args, **kwargs):
        pytest.fail("unchanged scene was parsed again")

    for index, (kind, field, value) in enumerate(
        [
            ("block", "position", [0.35, -0.18, 0.1]),
            ("bowl", "rotation", [10, 20, 30]),
            ("block", "size", [0.06, 0.07, 0.08]),
            ("ball", "size", [0.03, 0.03, 0.03]),
            ("cylinder", "color", [0.8, 0.2, 0.3]),
            ("block", None, None),
        ]
    ):
        obj = next(obj for obj in authored["objects"] if obj["kind"] == kind)
        if field is None:
            authored["objects"].remove(obj)
        else:
            obj[field] = value
        with monkeypatch.context() as cached:
            cached.setattr(mj.MjSpec, "from_file", unexpected_full_parse)
            actual = prepare_scene(
                config, authored, tmp_path / f"edit-{index}", _prefer_handoff=True
            )
        with monkeypatch.context() as uncached:
            uncached.delenv("WADDLE_SIMULATION_MODEL_CACHE")
            expected = prepare_scene(config, authored, tmp_path / f"fresh-{index}")
        same_model(actual.model, expected.model)
        np.testing.assert_array_equal(first.model.body_pos, original)
    # Mesh topology/asset changes deliberately use the authoritative full parser.
    bowl = next(obj for obj in authored["objects"] if obj["kind"] == "bowl")
    bowl["size"] = [0.18, 0.18, 0.07]
    actual = prepare_scene(config, authored, tmp_path / "resized-bowl")
    with monkeypatch.context() as uncached:
        uncached.delenv("WADDLE_SIMULATION_MODEL_CACHE")
        expected = prepare_scene(config, authored, tmp_path / "fresh-bowl")
    same_model(actual.model, expected.model)
    buffers.close()


def test_repeated_add_remove_bounds_native_preparation_and_matches_full_parser(
    tmp_path, monkeypatch
):
    if not hasattr(mj, "MjVfs"):
        pytest.skip("explicit virtual files unavailable")
    buffers = source_buffers.SourceBuffers()
    monkeypatch.setattr(source_buffers, "cache", buffers)
    monkeypatch.setenv("WADDLE_SIMULATION_MODEL_CACHE", str(tmp_path / "cache"))
    parse = mj.MjSpec.from_file
    parses = []

    def full_parse(*args, **kwargs):
        parses.append(args[0])
        return parse(*args, **kwargs)

    monkeypatch.setattr(mj.MjSpec, "from_file", full_parse)
    for index in range(70):
        addition = (
            '<body name="prop_b" pos="0 0 2"><freejoint/>'
            '<geom name="b" type="sphere" size=".2"/></body>'
            if index % 2
            else ""
        )
        pair = '<pair geom1="a" geom2="b"/>' if addition else ""
        source = f"""<mujoco><compiler angle="radian"/><asset/>
            <worldbody><body name="prop_a" pos="0 0 1"><freejoint/>
            <geom name="a" type="sphere" size=".1"/></body>{addition}</worldbody>
            <contact>{pair}</contact><keyframe><key name="initial"/></keyframe>
            </mujoco>"""
        model = load_model(
            "world.xml", {"world.xml": source.encode()}, cache="", _authored=True
        )
        same_model(model, mj.MjModel.from_xml_string(source))
    # Reuse happens, but long edit sequences periodically discard their native
    # attachment bookkeeping instead of accumulating it for the session lifetime.
    assert 1 < len(parses) < 70
    buffers.close()
