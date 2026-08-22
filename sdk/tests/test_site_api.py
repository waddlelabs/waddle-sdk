"""Contract tests for the hard-cut Site/SiteSession/Run SDK surface."""

from __future__ import annotations

import textwrap
import time

import numpy as np
import pytest

import site_fixtures
import waddle_sdk
import waddle_sdk.site as site_api
from waddle_sdk.runtime import (
    FaultCode,
    JointPositionCommand,
    RuntimeFault,
    SdkGeometryPort,
    SdkKinematicsPort,
    SdkRuntimePort,
    SdkSupportPort,
    SupportFact,
    SupportMatrix,
    SupportRow,
)


def _write_site(tmp_path, extra: str = ""):
    path = tmp_path / "site.yaml"
    path.write_text(
        textwrap.dedent(
            f"""
            api_version: waddle.site/v1
            kind: Site
            metadata:
              id: test-cell
            parts:
              arm:
                driver: site_fixtures:part
                posture: supervised
                connection: {{}}
                joint_limits: {{}}
            cameras:
              overhead:
                driver: site_fixtures:camera
                connection: {{}}
                stream: {{width: 2, height: 2, fps: 20}}
                frame_id: overhead_optical
                intrinsics:
                  fx: 100.0
                  fy: 100.0
                  cx: 0.0
                  cy: 0.0
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
            {extra}
            """
        ),
        encoding="utf-8",
    )
    return path


@pytest.fixture(autouse=True)
def _reset_fixtures():
    site_fixtures.reset()
    yield
    site_fixtures.reset()


def test_manifest_is_strict_and_paths_are_site_relative(tmp_path):
    site = waddle_sdk.load_site(_write_site(tmp_path))
    assert site.id == "test-cell"
    assert site.calibration_root == tmp_path / "calib"
    assert site.recording_root == tmp_path / "recordings"

    invalid = _write_site(tmp_path, "unexpected: true")
    with pytest.raises(waddle_sdk.ManifestValidationError, match="unexpected"):
        waddle_sdk.load_site(invalid)

    unsupported_keepout = _write_site(tmp_path)
    unsupported_keepout.write_text(
        unsupported_keepout.read_text().replace(
            "static_keepouts: []", "static_keepouts: [{kind: box}]"
        ),
        encoding="utf-8",
    )
    with pytest.raises(waddle_sdk.ManifestValidationError, match="static_keepouts"):
        waddle_sdk.load_site(unsupported_keepout)

    escaping = _write_site(tmp_path)
    escaping.write_text(
        escaping.read_text().replace("artifacts: calib/", "artifacts: ../calib/"),
        encoding="utf-8",
    )
    with pytest.raises(waddle_sdk.ManifestPathError, match="stay beneath"):
        waddle_sdk.load_site(escaping)


def test_camera_factory_internal_type_error_is_not_mislabeled_as_signature(tmp_path):
    path = _write_site(tmp_path)
    path.write_text(
        path.read_text().replace(
            "driver: site_fixtures:camera",
            "driver: site_fixtures:camera_internal_type_error",
        ),
        encoding="utf-8",
    )

    with (
        pytest.raises(TypeError, match="vendor rejected the selected stream"),
        waddle_sdk.load_site(path).open(console=False, _testing=True),
    ):
        pass


def test_driver_neutral_gripper_mapping_is_strict_and_not_a_factory_option(tmp_path):
    path = _write_site(tmp_path)
    path.write_text(
        path.read_text().replace(
            "joint_limits: {}",
            (
                "joint_limits: {}\n"
                "    gripper: {joint: j1, closed_m: 0.0, open_m: 0.095, "
                "closed_action: -1.0, open_action: 1.0}"
            ),
            1,
        ),
        encoding="utf-8",
    )
    site = waddle_sdk.load_site(path)
    assert site.describe()["parts"]["arm"]["gripper"]["open_m"] == 0.095

    with site.open(console=False, _testing=True):
        assert site_fixtures.opened["arms"] == 1

    path.write_text(
        path.read_text().replace("open_action: 1.0", "open_action: 1.0, surprise: true"),
        encoding="utf-8",
    )
    with pytest.raises(waddle_sdk.ManifestValidationError, match="surprise"):
        waddle_sdk.load_site(path)


def test_gripper_grasp_geometry_is_hardware_neutral_and_complete(tmp_path):
    path = _write_site(tmp_path)
    path.write_text(
        path.read_text().replace(
            "joint_limits: {}",
            (
                "joint_limits: {}\n"
                "    gripper:\n"
                "      joint: j1\n"
                "      closed_m: 0.0\n"
                "      open_m: 0.095\n"
                "      closed_action: -1.0\n"
                "      open_action: 1.0\n"
                "      closing_axis_tcp: [0.0, 1.0, 0.0]\n"
                "      pinch_offset_tcp_m: [0.044, 0.0, -0.0049]\n"
                "      pointing_down_wxyz: [0.0, 0.0, 1.0, 0.0]"
            ),
            1,
        ),
        encoding="utf-8",
    )

    geometry = waddle_sdk.load_site(path).describe()["parts"]["arm"]["gripper"]
    assert geometry["closing_axis_tcp"] == [0.0, 1.0, 0.0]
    assert geometry["pinch_offset_tcp_m"] == [0.044, 0.0, -0.0049]
    assert geometry["pointing_down_wxyz"] == [0.0, 0.0, 1.0, 0.0]

    path.write_text(
        path.read_text().replace(
            "      pinch_offset_tcp_m: [0.044, 0.0, -0.0049]\n",
            "",
        ),
        encoding="utf-8",
    )
    with pytest.raises(waddle_sdk.ManifestValidationError, match="geometry is incomplete"):
        waddle_sdk.load_site(path)


def test_static_keepout_rejects_complete_action_before_driver_write(tmp_path):
    path = _write_site(tmp_path)
    path.write_text(
        path.read_text().replace(
            "static_keepouts: []",
            (
                "static_keepouts: [{id: table, kind: box, frame: cell, "
                "min: [0.19, -0.03, -0.03], max: [0.30, 0.03, 0.03]}]"
            ),
        ),
        encoding="utf-8",
    )
    site = waddle_sdk.load_site(path)

    with (
        site.open(console=False, _testing=True) as session,
        session.run(task="keepout", actor="test") as run,
    ):
        refused = run.step([0.18, 0.0], run.observe())
        assert not refused.dispatched
        assert refused.gate == "owner_refusal"
        assert session._managed.rejected == 1


def test_self_collision_rejects_unless_named_pair_is_ignored(tmp_path):
    path = _write_site(tmp_path)
    path.write_text(
        path.read_text().replace(
            "self_collision: {}",
            "self_collision: {enabled: true, margin_m: 0.0}",
        ),
        encoding="utf-8",
    )
    site = waddle_sdk.load_site(path)

    with site.open(console=False, _testing=True) as session:
        with session.run(task="self collision", actor="test") as run:
            refused = run.step([0.1, -0.1], run.observe())
            assert not refused.dispatched
            assert refused.gate == "owner_refusal"
            assert session._managed.accepted == 0

    path.write_text(
        path.read_text().replace(
            "self_collision: {enabled: true, margin_m: 0.0}",
            (
                "self_collision: {enabled: true, margin_m: 0.0, "
                "ignore_pairs: [[arm/link_0, arm/link_1]]}"
            ),
        ),
        encoding="utf-8",
    )
    ignored_site = waddle_sdk.load_site(path)
    with ignored_site.open(console=False, _testing=True) as session:
        with session.run(task="ignored adjacent bodies", actor="test") as run:
            accepted = run.step([0.1, -0.1], run.observe())
            assert accepted.dispatched
            assert session._managed.accepted == 1


def test_secret_values_require_named_references(tmp_path):
    path = _write_site(tmp_path)
    path.write_text(
        path.read_text().replace("connection: {}", "connection: {api_token: raw}", 1),
        encoding="utf-8",
    )
    with pytest.raises(waddle_sdk.ManifestValidationError, match="named"):
        waddle_sdk.load_site(path)


def test_hardware_opens_only_inside_context_and_closes_once(tmp_path):
    site = waddle_sdk.load_site(_write_site(tmp_path))
    context = site.open(console=False, _testing=True)
    assert site_fixtures.opened == {"arms": 0, "cameras": 0}

    with context as session:
        assert isinstance(session, SdkRuntimePort)
        assert site_fixtures.opened == {"arms": 1, "cameras": 1}
        sample = session._managed.wait_camera("overhead", timeout_s=2.0)
        assert sample is not None
        observation = session.observe()
        assert tuple(observation.parts) == ("arm",)
        assert "overhead" in observation.cameras
        robot = session.describe()["robot"]
        (part,) = robot["actionSpace"]["composite"]["parts"]
        assert part["name"] == "arm"
        assert [joint["name"] for joint in part["space"]["jointPosition"]["joints"]] == [
            "j0",
            "j1",
        ]

    assert site_fixtures.closed == {"arms": 1, "cameras": 1}


def test_open_session_exposes_immutable_support_and_optional_sdk_facets(tmp_path):
    site = waddle_sdk.load_site(_write_site(tmp_path))
    with site.open(console=False, _testing=True) as session:
        assert isinstance(session, SdkSupportPort)
        assert isinstance(session, SdkKinematicsPort)
        assert isinstance(session, SdkGeometryPort)

        matrix = session.support()
        description = session.describe()
        assert description["support"] == matrix.as_dict()
        assert description["support"]["contractVersion"] == "waddle.sdk.support/v1"
        assert description["support"]["actionSpace"] == description["robot"][
            "actionSpace"
        ]
        assert description["support"]["grants"] == description["robot"]["grants"]
        assert len(description["support"]["embodimentDigest"]) == 64

        rows = {row.scope: row for row in matrix.rows}
        arm_facts = set(rows["robot:arm"].facts)
        assert SupportFact.JOINT_POSITION_OBSERVATION in arm_facts
        assert SupportFact.JOINT_VELOCITY_OBSERVATION in arm_facts
        assert SupportFact.JOINT_POSITION_ACTION in arm_facts
        assert SupportFact.SEND_GRANT in arm_facts
        assert SupportFact.HOLD_GRANT in arm_facts
        assert SupportFact.ESTOP_GRANT in arm_facts
        assert SupportFact.VELOCITY_FEEDFORWARD in arm_facts
        assert SupportFact.BODY_SPHERES in arm_facts
        assert SupportFact.FORWARD_KINEMATICS not in arm_facts

        camera_facts = set(rows["camera:overhead"].facts)
        assert camera_facts == {
            SupportFact.CAMERA_RGB,
            SupportFact.CAMERA_INTRINSICS,
        }
        assert all("camera.depth" not in fact.value for fact in camera_facts)

        spheres = session.body_geometry("arm", [0.0, 0.0])
        assert [sphere.name for sphere in spheres] == ["arm/link_0", "arm/link_1"]
        assert all(sphere.frame_id == "cell" for sphere in spheres)

        with pytest.raises(TypeError):
            matrix.action_space["rateHz"] = 50.0
        with pytest.raises(RuntimeFault) as fault:
            session.forward_kinematics("arm", [0.0, 0.0])
        assert fault.value.code is FaultCode.UNSUPPORTED

        arm = session._managed.arms["arm"]
        arm.base_frame = "cell"
        base_frame_matrix = session.support()
        original_rows = {row.scope: row for row in matrix.rows}
        base_frame_rows = {row.scope: row for row in base_frame_matrix.rows}
        assert base_frame_matrix.embodiment_digest != matrix.embodiment_digest
        assert (
            base_frame_rows["robot:arm"].embodiment_digest
            != original_rows["robot:arm"].embodiment_digest
        )
        assert (
            base_frame_rows["camera:overhead"].embodiment_digest
            == original_rows["camera:overhead"].embodiment_digest
        )

        arm.fk = lambda _q: (
            np.array([0.1, 0.2, 0.3]),
            np.eye(3),
        )
        pose = session.forward_kinematics("arm", [0.0, 0.0])
        assert pose.position_m == (0.1, 0.2, 0.3)
        assert pose.quaternion_wxyz == (1.0, 0.0, 0.0, 0.0)
        assert pose.frame_id == "cell"
        assert SupportFact.FORWARD_KINEMATICS in {
            fact
            for row in session.support().rows
            if row.scope == "robot:arm"
            for fact in row.facts
        }


@pytest.mark.parametrize("invalid", ["A" * 64, "a" * 63, "g" * 64])
def test_support_dtos_require_exact_lowercase_sha256(invalid: str):
    valid = "a" * 64
    with pytest.raises(ValueError, match="lowercase sha256"):
        SupportRow(scope="robot:arm", embodiment_digest=invalid, facts=())

    row = SupportRow(scope="robot:arm", embodiment_digest=valid, facts=())
    with pytest.raises(ValueError, match="lowercase sha256"):
        SupportMatrix(
            contract_version="waddle.sdk.support/v1",
            embodiment_digest=invalid,
            action_space={},
            grants=(),
            rows=(row,),
        )


def test_support_digests_are_scoped_to_relevant_public_embodiment(tmp_path):
    def digests(path):
        with waddle_sdk.load_site(path).open(
            console=False, _testing=True
        ) as session:
            matrix = session.support()
            return matrix.embodiment_digest, {
                row.scope: row.embodiment_digest for row in matrix.rows
            }

    baseline_root = tmp_path / "baseline"
    baseline_root.mkdir()
    baseline = _write_site(baseline_root)

    camera_root = tmp_path / "camera-change"
    camera_root.mkdir()
    camera_change = _write_site(camera_root)
    camera_change.write_text(
        camera_change.read_text()
        .replace("fps: 20}", "fps: 21}")
        .replace("frame_id: overhead_optical", "frame_id: overhead_new"),
        encoding="utf-8",
    )

    gripper_root = tmp_path / "gripper-change"
    gripper_root.mkdir()
    gripper_change = _write_site(gripper_root)
    gripper_change.write_text(
        gripper_change.read_text().replace(
            "joint_limits: {}",
            (
                "joint_limits: {}\n"
                "    gripper: {joint: j1, closed_m: 0.0, open_m: 0.095, "
                "closed_action: -1.0, open_action: 1.0}"
            ),
            1,
        ),
        encoding="utf-8",
    )

    baseline_matrix, baseline_rows = digests(baseline)
    camera_matrix, camera_rows = digests(camera_change)
    gripper_matrix, gripper_rows = digests(gripper_change)

    assert camera_matrix != baseline_matrix
    assert camera_rows["camera:overhead"] != baseline_rows["camera:overhead"]
    assert camera_rows["robot:arm"] == baseline_rows["robot:arm"]

    assert gripper_matrix != baseline_matrix
    assert gripper_rows["robot:arm"] != baseline_rows["robot:arm"]
    assert gripper_rows["camera:overhead"] == baseline_rows["camera:overhead"]


def test_observation_envelope_is_stamped_after_camera_snapshot(tmp_path):
    site = waddle_sdk.load_site(_write_site(tmp_path))
    with site.open(console=False, _testing=True) as session:
        managed = session._managed
        assert managed.wait_camera("overhead", timeout_s=2.0) is not None
        events: list[str] = []
        core = managed.core

        class _CoreOrderProbe:
            def stamp(self):
                events.append("stamp")
                return core.stamp()

            def __getattr__(self, name):
                return getattr(core, name)

        camera_sample = managed.camera_sample

        def observed_camera(name):
            events.append(f"camera:{name}")
            return camera_sample(name)

        managed.core = _CoreOrderProbe()
        managed.camera_sample = observed_camera
        observation = session.observe()

        assert events == ["camera:overhead", "stamp"]
        assert observation.session_ns >= observation.cameras["overhead"].session_ns


def test_local_calibration_measurement_does_not_require_remote_feature(tmp_path):
    site = waddle_sdk.load_site(_write_site(tmp_path))
    with site.open(console=False, _testing=True) as session:
        sample = session._managed.wait_camera("overhead", timeout_s=2.0)
        assert sample is not None
        result = session.calibration_measurement(
            calibration_id="cal-local",
            sample_id="sample-local",
            camera="overhead",
            frame_sequence=sample.frame_sequence,
            x=0,
            y=0,
        )

    assert result["frame_id"] == "overhead_optical"
    assert result["point_xyz"] == [0.0, 0.0, 1.0]
    assert result["depth_m"] == 1.0


def test_run_routes_gate_decision_through_owner_envelope(tmp_path):
    site = waddle_sdk.load_site(_write_site(tmp_path))
    with site.open(console=False, _testing=True) as session:
        with session.run(task={"id": "move"}, actor={"id": "test"}) as run:
            observation = run.observe()
            accepted = run.step([0.1, -0.1], observation)
            assert accepted.dispatched
            assert session._managed.accepted == 1

            refused = run.step([0.9, -0.1], observation)
            assert not refused.dispatched
            assert refused.gate == "owner_refusal"
            assert session._managed.rejected == 1
            run.finish("success")

        assert run.outcome == "success"
        events = session.events()
        assert [event.cursor for event in events] == list(range(1, len(events) + 1))
        assert any(event.kind == "run.step" for event in events)


def test_run_carries_a_known_velocity_only_for_the_unchanged_gate_action(tmp_path):
    site = waddle_sdk.load_site(_write_site(tmp_path))
    with site.open(console=False, _testing=True) as session:
        with session.run(task="known ramp", actor="test") as run:
            command = JointPositionCommand(
                [0.1, -0.1], velocity_feedforward_rad_s=[0.5, -0.5]
            )
            result = run.step(command, run.observe())

    assert result.dispatched is True
    [(position, velocity)] = site_fixtures.velocity_commands
    assert position.tolist() == pytest.approx([0.1, -0.1])
    assert velocity.tolist() == pytest.approx([0.5, -0.5])


def test_substitution_uses_the_selected_streams_velocity_never_the_callers(tmp_path):
    site = waddle_sdk.load_site(_write_site(tmp_path))
    with site.open(console=False, _testing=True) as session:
        with session.run(task="selected ramp", actor="test") as run:
            run.step([0.0, 0.0], run.observe())  # READY -> RUNNING
            core = session._managed.core
            core._testing_engage("claim-feedforward", "agent")
            deadline = time.monotonic() + 5.0
            while core.status()["gate_mode"] != "Intervention":
                assert time.monotonic() < deadline, "claim never engaged"
                time.sleep(0.005)

            core._testing_push_chunk(
                [0.1, -0.1], velocity_feedforward=[0.25, -0.25]
            )
            caller = JointPositionCommand(
                [0.02, -0.02], velocity_feedforward_rad_s=[0.9, -0.9]
            )
            deadline = time.monotonic() + 5.0
            while True:
                selected = run.step(caller, run.observe())
                if selected.gate == "substitute":
                    break
                assert time.monotonic() < deadline, "chunk never substituted"
                time.sleep(0.005)

            [(position, velocity)] = site_fixtures.velocity_commands
            assert position.tolist() == pytest.approx([0.1, -0.1])
            assert velocity.tolist() == pytest.approx([0.25, -0.25])

            core._testing_push_chunk([0.12, -0.12])
            deadline = time.monotonic() + 5.0
            while True:
                selected = run.step(caller, run.observe())
                if selected.gate == "substitute":
                    break
                assert time.monotonic() < deadline, "position-only chunk never substituted"
                time.sleep(0.005)
            assert len(site_fixtures.velocity_commands) == 1, (
                "the caller's feedforward crossed onto the selected stream's position"
            )


def test_joint_position_command_rejects_malformed_velocity_hints():
    with pytest.raises(ValueError, match="same width"):
        JointPositionCommand([0.1, 0.2], velocity_feedforward_rad_s=[0.3])
    with pytest.raises(ValueError, match="finite"):
        JointPositionCommand([0.1, 0.2], velocity_feedforward_rad_s=[0.3, float("nan")])


def test_root_exports_only_primary_surface():
    assert set(waddle_sdk.__all__) == {
        "Grpc",
        "LiveKit",
        "ManifestError",
        "ManifestPathError",
        "ManifestSyntaxError",
        "ManifestValidationError",
        "Outcome",
        "Run",
        "Site",
        "SiteSession",
        "load_site",
    }
    assert "Control" not in waddle_sdk.__all__
    assert "agent" not in waddle_sdk.__all__
    assert "ui" not in waddle_sdk.__all__


class _AuthorizationProbe:
    def __init__(self, *, accepted: bool, refused: bool = False):
        self.accepted = accepted
        self.refused = refused
        self.closed = False

    def status(self):
        assert site_fixtures.opened == {"arms": 0, "cameras": 0}
        return {
            "plane_registered": not self.refused,
            "connector_binding_negotiated": self.accepted,
            "connector_binding_refused": self.refused,
        }

    def shutdown(self):
        self.closed = True


def _connector_transport():
    return waddle_sdk.Grpc(
        "https://connector.example:443",
        "secret",
        customer_id="customer",
        project_id="project",
        workspace_id="workspace",
    )


def test_connector_binding_is_all_or_none():
    with pytest.raises(ValueError, match="must all"):
        waddle_sdk.Grpc(
            "https://connector.example:443",
            customer_id="customer",
        )
    with pytest.raises(ValueError, match="requires a connector binding"):
        waddle_sdk.Grpc(
            "https://connector.example:443",
            authorization_only=True,
        )


def test_connector_refusal_never_opens_hardware(tmp_path, monkeypatch):
    probe = _AuthorizationProbe(accepted=False, refused=True)
    monkeypatch.setattr(site_api, "create_core_session", lambda *a, **k: probe)
    site = waddle_sdk.load_site(_write_site(tmp_path))

    with pytest.raises(RuntimeError, match="connector.binding"):
        with site.open(transport=_connector_transport(), console=False):
            pass

    assert probe.closed
    assert site_fixtures.opened == {"arms": 0, "cameras": 0}


def test_connector_authorizes_before_opening_hardware(tmp_path, monkeypatch):
    original = site_api.create_core_session
    probe = _AuthorizationProbe(accepted=True)

    def create(*args, **kwargs):
        transport = kwargs.get("transport")
        if transport is not None and transport.authorization_only:
            return probe
        kwargs["transport"] = None
        kwargs["_testing"] = True
        return original(*args, **kwargs)

    monkeypatch.setattr(site_api, "create_core_session", create)
    site = waddle_sdk.load_site(_write_site(tmp_path))

    with site.open(transport=_connector_transport(), console=False):
        assert probe.closed
        assert site_fixtures.opened == {"arms": 1, "cameras": 1}

    assert site_fixtures.closed == {"arms": 1, "cameras": 1}
