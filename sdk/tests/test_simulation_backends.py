"""Contract tests for manifest-selected, shared simulation backends."""

from __future__ import annotations

import textwrap

import pytest
import simulation_fixtures
import waddle_sdk
from waddle_sdk import simulation
from waddle_sdk.runtime import SdkRuntimePort, SupportFact


def _write_site(
    tmp_path,
    *,
    backend: str = "simulation_fixtures:backend",
    second_part: bool = False,
):
    additional_part = (
        """  arm2:
    world: cell
    posture: supervised
    base_frame: cell
    connection: {entity: robot2}
    joint_limits:
      j0: [-1.0, 1.0]
      j1: [-1.0, 1.0]
"""
        if second_part
        else ""
    )
    path = tmp_path / "site.yaml"
    content = textwrap.dedent(
        f"""
            api_version: waddle.site/v1
            kind: Site
            metadata:
              id: modular-sim
            worlds:
              cell:
                driver: {backend}
                connection: {{scene: scenes/cell.usd}}
                options: {{renderer: test}}
            parts:
              arm:
                world: cell
                posture: supervised
                base_frame: cell
                connection: {{entity: robot}}
                joint_limits:
                  j0: [-1.0, 1.0]
                  j1: [-1.0, 1.0]
            __ADDITIONAL_PART__
            cameras:
              overhead:
                world: cell
                connection: {{entity: overhead_camera}}
                stream: {{width: 3, height: 2, fps: 20}}
                frame_id: overhead_optical
                mount: {{kind: scene}}
                intrinsics:
                  fx: 100.0
                  fy: 100.0
                  cx: 1.0
                  cy: 0.5
                  depth_scale_mm: 1.0
            frames: {{}}
            calibration:
              artifacts: calib/
            workspace_bounds: {{}}
            envelope:
              static_keepouts: []
              self_collision: {{}}
            recording:
              root: recordings/
              format: mcap
            """
    )
    path.write_text(
        content.replace("__ADDITIONAL_PART__\n", additional_part),
        encoding="utf-8",
    )
    return path


@pytest.fixture(autouse=True)
def _reset_backend():
    simulation_fixtures.reset()
    simulation_fixtures.worlds.clear()
    yield
    simulation_fixtures.reset()
    simulation_fixtures.worlds.clear()


def test_world_backend_is_lazy_and_runtime_is_the_ordinary_sdk_port(tmp_path):
    site = waddle_sdk.load_site(_write_site(tmp_path))
    assert simulation_fixtures.events == []

    context = site.open(console=False, _testing=True)
    assert simulation_fixtures.events == []
    with context as session:
        assert isinstance(session, SdkRuntimePort)
        assert simulation_fixtures.events[:5] == [
            "world.factory",
            "world.part",
            "world.open",
            "camera.open",
            "arm.open",
        ]

        sample = session._require().wait_camera("overhead", timeout_s=1.0)
        assert sample is not None
        assert sample.rgb.shape == (2, 3, 3)
        assert sample.depth is not None
        assert sample.depth.tolist() == [[750, 750, 750], [750, 750, 750]]

        rows = {row.scope: row for row in session.support().rows}
        assert SupportFact.JOINT_POSITION_ACTION in rows["robot:arm"].facts
        assert SupportFact.CAMERA_RGB in rows["camera:overhead"].facts
        assert SupportFact.CAMERA_INTRINSICS in rows["camera:overhead"].facts
        assert set(session.observe().parts) == {"arm"}
        assert set(session.observe().cameras) == {"overhead"}

    assert simulation_fixtures.events[-3:] == [
        "camera.close",
        "arm.close",
        "world.close",
    ]


def test_each_shared_world_advances_once_per_composite_sdk_tick(tmp_path):
    with waddle_sdk.load_site(_write_site(tmp_path, second_part=True)).open(
        console=False, _testing=True
    ) as session:
        world = simulation_fixtures.worlds[0]
        managed = session._require()
        assert set(managed.arms) == {"arm", "arm2"}
        assert managed.pump is not None
        managed.pump.stop()
        before = world.steps
        managed.pump._tick(0.05)
        assert world.steps == before + 1
        # The fixture arm's Driver.step is deliberately a no-op. Progress proves
        # that the SDK advances the shared world once rather than once per arm.


def test_world_reset_is_composed_before_the_normal_arm_reset(tmp_path):
    with waddle_sdk.load_site(_write_site(tmp_path)).open(
        console=False, _testing=True
    ) as session:
        run = session.begin_run(task="reset the test world", actor="test")
        try:
            assert simulation_fixtures.worlds[0].resets == 1
        finally:
            run.__exit__(None, None, None)


def test_world_references_and_optional_facets_fail_closed(tmp_path):
    path = _write_site(tmp_path)
    path.write_text(path.read_text().replace("world: cell", "world: missing", 1))
    with pytest.raises(waddle_sdk.ManifestValidationError, match="unknown world"):
        waddle_sdk.load_site(path)

    path = _write_site(tmp_path, backend="simulation_fixtures:no_camera_backend")
    with (
        pytest.raises(
            waddle_sdk.ManifestValidationError, match="does not provide camera"
        ),
        waddle_sdk.load_site(path).open(console=False, _testing=True),
    ):
        pass


def test_camera_open_failure_never_opens_arm_and_closes_world(tmp_path):
    path = _write_site(tmp_path, backend="simulation_fixtures:broken_camera_backend")
    with (
        pytest.raises(RuntimeError, match="renderer failed"),
        waddle_sdk.load_site(path).open(console=False, _testing=True),
    ):
        pass

    assert simulation_fixtures.events == [
        "world.factory",
        "world.part",
        "world.open",
        "camera.open.failed",
        "world.close",
    ]


def test_wrist_camera_cannot_cross_simulation_worlds(tmp_path):
    path = _write_site(tmp_path)
    text = path.read_text()
    text = text.replace(
        "options: {renderer: test}",
        (
            "options: {renderer: test}\n"
            "  other:\n"
            "    driver: simulation_fixtures:backend\n"
            "    connection: {scene: scenes/other.usd}\n"
            "    options: {renderer: test}"
        ),
    )
    text = text.replace(
        "world: cell\n    connection: {entity: overhead_camera}",
        "world: other\n    connection: {entity: overhead_camera}",
    )
    text = text.replace("mount: {kind: scene}", "mount: {kind: wrist, part: arm}")
    path.write_text(text)

    with pytest.raises(waddle_sdk.ManifestValidationError, match="same world"):
        waddle_sdk.load_site(path)


def test_installed_backend_short_names_use_the_public_entry_point_group(monkeypatch):
    class EntryPoint:
        def load(self):
            return simulation_fixtures.backend

    class EntryPoints:
        def select(self, *, group, name):
            assert group == "waddle_sdk.simulation_backends"
            assert name == "external_sim"
            return (EntryPoint(),)

    monkeypatch.setattr(simulation.metadata, "entry_points", EntryPoints)
    assert (
        simulation.resolve_simulation_factory("external_sim")
        is simulation_fixtures.backend
    )
    assert (
        simulation.resolve_simulation_factory("mujoco").__module__
        == "waddle_sdk.robots.mujoco"
    )


def test_backend_listing_is_nonopening_and_includes_installed_names(monkeypatch):
    class EntryPoint:
        name = "external_sim"

        def load(self):
            raise AssertionError("listing must not load an entry point")

    class EntryPoints:
        def select(self, *, group):
            assert group == "waddle_sdk.simulation_backends"
            return (EntryPoint(),)

    monkeypatch.setattr(simulation.metadata, "entry_points", EntryPoints)
    assert simulation.simulation_backend_names() == (
        "external_sim",
        "mujoco",
        "ros2",
    )
