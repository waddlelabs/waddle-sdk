"""Manifest and hard-safety tests for the dependency-free mock robot."""

from __future__ import annotations

import textwrap

import waddle_sdk


def _site(tmp_path, *, envelope: str) -> waddle_sdk.Site:
    path = tmp_path / "site.yaml"
    document = textwrap.dedent(
        """
        api_version: waddle.site/v1
        kind: Site
        metadata: {id: mock-cell}
        parts:
          arm:
            driver: waddle_sdk.robots.mock:arm
            posture: supervised
            connection: {}
            joint_limits:
              shoulder: [-1.0, 1.0]
              elbow: [-1.0, 1.0]
            options:
              rate_hz: 20
              step_caps: [0.2, 0.2]
              home: [0.0, 0.0]
              collision_frame: site
        cameras: {}
        frames: {}
        calibration: {artifacts: calib/}
        workspace_bounds: {}
        envelope:
        __ENVELOPE__
        recording: {root: data/, format: mcap}
        """
    ).replace(
        "__ENVELOPE__",
        textwrap.indent(textwrap.dedent(envelope).strip(), "  "),
    )
    path.write_text(document, encoding="utf-8")
    return waddle_sdk.load_site(path)


def test_mock_robot_constructs_lazily_and_runs_through_site(tmp_path):
    site = _site(
        tmp_path,
        envelope="""
        static_keepouts: []
        self_collision: {}
        """,
    )
    context = site.open(console=False, _testing=True)

    with context as session, session.run(task="mock move", actor="test") as run:
        observation = run.observe()
        assert tuple(observation.parts) == ("arm",)
        result = run.step([0.1, -0.1], observation)
        assert result.dispatched
        assert session._managed.accepted == 1


def test_mock_robot_supplies_geometry_for_manifest_keepouts(tmp_path):
    site = _site(
        tmp_path,
        envelope="""
        static_keepouts:
          - id: fixture
            kind: sphere
            frame: site
            center: [0.1, 0.0, 0.0]
            radius_m: 0.03
        self_collision: {}
        """,
    )

    with (
        site.open(console=False, _testing=True) as session,
        session.run(task="mock keepout", actor="test") as run,
    ):
        result = run.step([0.0, 0.0], run.observe())
        assert not result.dispatched
        assert result.gate == "owner_refusal"
        assert session._managed.accepted == 0
        assert session._managed.rejected == 1
