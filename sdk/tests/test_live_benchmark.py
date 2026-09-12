"""Behavioral checks for live measurement arithmetic without opening hardware."""

import json
import threading
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from live.sdk_live import config as bench_config
from live.sdk_live import paired, session, vendor
from waddle_sdk.runtime import FaultCode, RuntimeFault, SubmitResult

CASE = {
    "part": "left",
    "case_id": "reference",
    "joint_names": ["joint1"],
    "start_rad": [0.0],
    "target_rad": [0.04],
    "velocity_rad_s": 0.2,
    "acceleration_rad_s2": 0.4,
    "jerk_rad_s3": 0.8,
    "joint_tolerance_rad": 0.02,
    "tcp_tolerance_m": 0.012,
    "settle_s": 0.5,
    "min_displacement_m": 0,
}


def measured_bench(monkeypatch, *, bias=0.0):
    """A deterministic plant that follows each command with optional static bias."""
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr(session.time, "monotonic", lambda: clock.now)
    monkeypatch.setattr(
        session.time,
        "sleep",
        lambda seconds: setattr(clock, "now", clock.now + seconds),
    )
    owner = object.__new__(session.Bench)
    owner.config = {"max_tracking_error_rad": 0.12}
    owner.site = SimpleNamespace(manifest={"parts": {"left": {"base_frame": "base"}}})
    owner.spaces = {"left": {"jointPosition": {"joints": [{"name": "joint1"}]}}}
    owner.period = 0.1
    owner.report = {"trials": []}
    owner.q = np.zeros(1)
    owner.command_count = 0
    owner.held = False
    owner.save = lambda: None
    owner.observe = lambda part: (
        owner.q.copy(),
        {"source": "deterministic test plant"},
    )
    owner.positions = lambda part: owner.q.copy()
    owner.pose = lambda part, q: np.array([q[0] * 0.1, 0, 0])

    def command(part, q, velocity):
        owner.command_count += 1
        owner.q = q + bias

    owner.command = command
    owner.hold = lambda: setattr(owner, "held", True)
    return owner


def test_measured_arrival_distinguishes_static_nonarrival_from_fault(monkeypatch):
    owner = measured_bench(monkeypatch, bias=0.024376)
    result = owner.move("left", CASE["target_rad"], CASE)
    assert result["outcome"] == "not_arrived"
    assert result["joint_error_rad"] == pytest.approx(0.024376)
    assert result["sample_count"] > 5
    assert result["elapsed_s"] >= result["trajectory_duration_s"] + CASE["settle_s"]
    assert owner.held and owner.report["trials"] == [result]

    fault = RuntimeFault(
        FaultCode.SAFETY_REFUSAL,
        "joint3 target -0.0017 outside [0, 3.14]",
        context={"part": "left", "joint": "joint3", "target": -0.0017},
    )
    owner = measured_bench(monkeypatch)
    owner._observation = SimpleNamespace(
        parts={"left": SimpleNamespace(joint_position=[0.0])}
    )
    owner.run = SimpleNamespace(
        step=lambda *args: SubmitResult(
            dispatched=False, gate="owner_refusal", part="left", fault=fault
        )
    )
    owner.command = lambda *args: session.Bench.command(owner, *args)
    with pytest.raises(RuntimeFault) as captured:
        owner.move("left", CASE["target_rad"], CASE)
    assert captured.value is fault
    assert owner.held
    assert owner.report["trials"][0]["outcome"] == "failed"
    assert owner.report["trials"][0]["error"] == fault.as_dict()


def test_rest_arrival_is_measured_without_becoming_a_comparison_trial(monkeypatch):
    owner = measured_bench(monkeypatch)
    result = owner.move("left", CASE["target_rad"], CASE, report_key="rest_trials")
    assert result["outcome"] == "arrived"
    assert result["settled_samples"] == 3
    assert result["joint_error_rad"] == pytest.approx(0)
    assert result["displacement_m"] == pytest.approx(0.004)
    assert owner.report["rest_trials"] == [result] and not owner.report["trials"]


def test_background_failure_interrupts_stream_before_another_command(monkeypatch):
    owner = measured_bench(monkeypatch)
    origin = RuntimeFault(FaultCode.MOTOR_FAILURE, "CAN writer stopped")
    command = owner.command

    def first_command_then_failure(*args):
        command(*args)
        owner._background_error = origin

    owner.command = first_command_then_failure
    with pytest.raises(RuntimeFault) as captured:
        owner.move("left", CASE["target_rad"], CASE)
    assert captured.value is origin
    assert owner.command_count == 1 and owner.held
    assert owner.report["trials"][0]["error"] == origin.as_dict()


def test_arrival_timing_and_endpoint_exclude_later_hold_delay_and_drift(monkeypatch):
    owner = measured_bench(monkeypatch)

    def delayed_hold():
        session.time.sleep(0.7)
        owner.q = np.array([0.0])

    owner.hold = delayed_hold
    result = owner.move("left", CASE["target_rad"], CASE)
    assert result["outcome"] == "arrived"
    assert result["joint_error_rad"] == pytest.approx(0)
    assert result["arrival_elapsed_s"] == result["samples"][-1]["elapsed_s"]
    assert result["elapsed_s"] == pytest.approx(result["arrival_elapsed_s"])
    assert result["hold_elapsed_s"] == pytest.approx(0.7)
    assert result["total_elapsed_s"] - result["elapsed_s"] == pytest.approx(0.7)
    assert result["post_hold_measured_rad"] == [0.0]


@pytest.mark.parametrize("outcome", ["arrived", "not_arrived"])
def test_backend_returns_to_configured_rest_before_close_without_hiding_trial(
    monkeypatch, tmp_path, outcome
):
    calls = []
    rest = {"joint_names": ["joint1"], "position_rad": [-0.03]}

    class Owner:
        def __init__(self, config, part):
            self.report = {"trials": []}

        def __enter__(self):
            return self

        def __exit__(self, *args):
            calls.append("closed")

        def move(self, part, target, case, *, report_key="trials"):
            calls.append(case["case_id"])
            parking = report_key == "rest_trials"
            if parking:
                assert target == rest["position_rad"]
                assert case["velocity_rad_s"] == CASE["velocity_rad_s"]
                assert case["joint_tolerance_rad"] == CASE["joint_tolerance_rad"]
            result = {
                "outcome": "arrived" if parking else outcome,
                "measured_rad": target,
            }
            self.report.setdefault(report_key, []).append(result)
            return result

    monkeypatch.setattr(paired, "Bench", Owner)
    result = paired.run_backend(
        {"rest_positions": {"left": rest}},
        "left",
        CASE,
        "sdk",
        tmp_path / "result.json",
        reference_only=True,
    )
    assert calls == ["reference-reference", "reference-rest", "closed"]
    assert len(result["trials"]) == 1
    assert result["trials"][0]["outcome"] == outcome
    assert result["rest_trials"][0]["outcome"] == "arrived"


@pytest.mark.parametrize("failure_phase", ["motion", "background", "rest"])
def test_backend_stops_after_fault_and_preserves_motion_and_rest_evidence(
    monkeypatch, tmp_path, failure_phase
):
    calls = []
    origin = RuntimeFault(
        FaultCode.MOTOR_FAILURE, "motor 3 failed", context={"motor": 3}
    )
    monkeypatch.setattr(threading, "excepthook", lambda args: None)

    class Owner:
        def __init__(self, config, part):
            self.report = {"trials": []}

        def __enter__(self):
            return self

        def __exit__(self, *args):
            calls.append("close")

        def move(self, part, target, case, *, report_key="trials"):
            calls.append(report_key)
            result = {"outcome": "not_arrived"}
            self.report.setdefault(report_key, []).append(result)
            if failure_phase == "motion" or report_key == "rest_trials":
                result.update(outcome="failed", error=origin.as_dict())
                raise origin
            if failure_phase == "background":
                threading.excepthook(
                    SimpleNamespace(
                        thread=SimpleNamespace(name="can-writer"), exc_value=origin
                    )
                )
            return result

    monkeypatch.setattr(paired, "Bench", Owner)
    output = tmp_path / "fault.json"
    with pytest.raises(RuntimeFault) as captured:
        paired.run_backend(
            {
                "rest_positions": {
                    "left": {"joint_names": ["joint1"], "position_rad": [-0.03]}
                }
            },
            "left",
            CASE,
            "sdk",
            output,
        )
    assert captured.value is origin
    assert calls == (
        ["trials", "rest_trials", "close"]
        if failure_phase == "rest"
        else ["trials", "close"]
    )
    report = json.loads(output.read_text())
    assert report["error"] == origin.as_dict()
    assert len(report["trials"]) == 1
    if failure_phase == "rest":
        assert report["trials"][0]["outcome"] == "not_arrived"
        assert report["rest_trials"][0]["error"] == origin.as_dict()


@pytest.mark.parametrize("position", [[float("nan")], [True], [], [0, 1]])
def test_invalid_rest_target_fails_during_configuration_before_hardware(
    tmp_path, position
):
    path = tmp_path / "bench.json"
    path.write_text(
        json.dumps(
            {
                "schema": "waddle.live-bench/v1",
                "site": "site.yaml",
                "evidence_directory": "evidence",
                "parts": ["left"],
                "cases": [CASE],
                "max_tracking_error_rad": 0.12,
                "rest_positions": {
                    "left": {"joint_names": ["joint1"], "position_rad": position}
                },
            }
        )
    )
    with pytest.raises(ValueError, match="rest_positions.left"):
        bench_config.load(path)


def test_setup_failure_preserves_origin_before_an_owner_exists(monkeypatch, tmp_path):
    origin = TypeError("mappingproxy is not JSON serializable")

    def broken_owner(*args):
        raise origin

    monkeypatch.setattr(paired, "Bench", broken_owner)
    output = tmp_path / "setup-failure.json"
    with pytest.raises(TypeError) as captured:
        paired.run_backend({}, "left", CASE, "sdk", output)
    assert captured.value is origin
    assert (
        json.loads(output.read_text())["error"]
        == RuntimeFault.from_exception(origin).as_dict()
    )


def test_real_manifest_projection_is_json_safe_and_opens_no_driver(
    tmp_path, monkeypatch
):
    def forbidden(*args, **kwargs):
        raise AssertionError("offline projection opened hardware")

    monkeypatch.setattr(vendor.yam, "LiveDriver", forbidden)
    site_path = tmp_path / "site.yaml"
    site_path.write_text(
        json.dumps(
            {
                "api_version": "waddle.site/v1",
                "kind": "Site",
                "metadata": {"id": "paired-test"},
                "parts": {
                    "left": {
                        "driver": "waddle_sdk.robots.yam:arm",
                        "posture": "supervised",
                        "base_frame": "left_base",
                        "connection": {"channel": "can_left"},
                    },
                    "right": {
                        "driver": "uninstalled_driver:arm",
                        "posture": "supervised",
                        "base_frame": "right_base",
                        "connection": {"channel": "can_right"},
                    },
                },
                "cameras": {},
                "frames": {},
                "calibration": {"artifacts": "calib/"},
                "workspace_bounds": {"min": [-0.5, -0.5, 0], "max": [0.5, 0.5, 1]},
                "envelope": {"static_keepouts": [], "self_collision": {}},
                "recording": {"root": "recordings/", "format": "mcap"},
            }
        )
    )
    config = {"site": str(site_path)}
    owners = [session.Bench(config, "left"), vendor.VendorBench(config, "left")]
    for owner in owners:
        assert owner.site.id == "paired-test"
        assert list(owner.site.manifest["parts"]) == ["left"]
        json.dumps(owner.report, allow_nan=False)
    assert (
        owners[0].report["source_manifest_sha256"]
        == owners[1].report["source_manifest_sha256"]
    )


def test_raw_close_joins_workers_before_closing_socket(monkeypatch):
    events = []
    chain = SimpleNamespace(running=True)
    server = SimpleNamespace(
        join=lambda timeout: events.append("server joined"), is_alive=lambda: False
    )

    def join_writer(timeout):
        assert not chain.running
        events.append("writer joined")

    writer = SimpleNamespace(
        _target=SimpleNamespace(
            __self__=chain, __name__="_set_torques_and_update_state"
        ),
        join=join_writer,
        is_alive=lambda: False,
    )
    robot = SimpleNamespace(
        _stop_event=SimpleNamespace(set=lambda: events.append("stop server")),
        _server_thread=server,
        motor_chain=chain,
        close=lambda: events.append("close socket"),
    )
    monkeypatch.setattr(vendor.threading, "enumerate", lambda: [writer])
    vendor.close_vendor(robot)
    assert events == ["stop server", "server joined", "writer joined", "close socket"]

    events.clear()
    server.is_alive = lambda: True
    with pytest.raises(RuntimeError, match="server did not stop"):
        vendor.close_vendor(robot)
    assert "close socket" not in events


def test_controller_evidence_distinguishes_requested_from_encoded_gains():
    result = vendor.settings(
        SimpleNamespace(get_robot_info=lambda: {"kp": [320, 40], "kd": [20, 6]})
    )
    assert result["kp"] == [320, 40] and result["kd"] == [20, 6]
    assert result["encoded_kp"] == pytest.approx([319.9023199023199, 39.92673992673993])
    assert result["encoded_kd"] == [5, 5]


def test_pristine_vendor_applies_explicit_arm_gains_and_preserves_hand(monkeypatch):
    get_robot = pytest.importorskip("i2rt.robots.get_robot")
    codec = pytest.importorskip("i2rt.motor_drivers.utils")
    get_utils = pytest.importorskip("i2rt.robots.utils")
    # Verify the non-opening preflight data against the installed pinned vendor.
    stock = get_utils._load_arm_config(get_utils.ArmType.YAM)
    hand_kp, hand_kd = get_utils.GripperType.LINEAR_4310.get_motor_kp_kd(
        get_utils.ArmType.YAM
    )
    info = {"kp": np.append(stock.kp, hand_kp), "kd": np.append(stock.kd, hand_kd)}
    preflight = vendor.yam._gain_vectors()
    assert preflight[0] == pytest.approx(info["kp"])
    assert preflight[1] == pytest.approx(info["kd"])
    robot = SimpleNamespace(
        get_robot_info=lambda: info,
        update_kp_kd=lambda kp, kd: info.update(kp=kp, kd=kd),
    )
    opened = []
    monkeypatch.setattr(
        get_robot, "get_yam_robot", lambda **kwargs: opened.append(kwargs) or robot
    )
    monkeypatch.setattr(vendor, "_SiteLock", lambda site_id: object())
    monkeypatch.setattr(vendor, "wait_for_startup", lambda *args, **kwargs: None)
    owner = object.__new__(vendor.VendorBench)
    owner.config = {"torque_release_authorized": True}
    owner.part = "left"
    gains = {"kp": [100, 120, 140, 10, 12, 14], "kd": [5, 5, 5, 1, 1.5, 2]}
    owner.site = SimpleNamespace(
        id="fake-gain-owner",
        manifest={
            "parts": {
                "left": {
                    "connection": {"channel": "fake_can"},
                    "options": {"arm_gains": gains},
                }
            }
        },
    )
    owner.period = 0.1
    owner.report = {}
    owner.observe = lambda part: None
    owner.save = lambda: None
    owner.__enter__()
    assert len(opened) == 1
    assert info["kp"] == pytest.approx(gains["kp"] + [hand_kp])
    assert info["kd"] == pytest.approx(gains["kd"] + [hand_kd])
    constants = codec.MotorType.get_motor_constants(codec.MotorType.DM4340)
    for name in ("kp", "kd"):
        maximum = getattr(constants, name.upper() + "_MAX")
        expected = [
            codec.uint_to_float(
                codec.float_to_uint(value, 0, maximum, 12), 0, maximum, 12
            )
            for value in info[name]
        ]
        assert owner.report["control_settings"]["encoded_" + name] == pytest.approx(
            expected
        )


@pytest.mark.parametrize("failure", [None, "cache", "host", "writer", "background"])
def test_startup_waits_for_hold_can_feedback_and_host_ingestion_without_pose_gate(
    monkeypatch, failure
):
    clock = SimpleNamespace(now=0.0, tick=0)
    owner = SimpleNamespace(report={}, _background_error=None)
    origin = RuntimeFault(FaultCode.MOTOR_FAILURE, "original CAN receive error")
    # Even an out-of-envelope position must finish readiness after fresh replies;
    # the unchanged envelope is checked separately before trajectory commands.
    position = np.array([0, 0, -0.0017, 0, 0, 0, 0.5])
    active = SimpleNamespace(pos=position, kp=np.ones(7), kd=np.ones(7))
    zero = SimpleNamespace(pos=0, kp=0, kd=0)
    chain = SimpleNamespace(
        running=True,
        command_lock=threading.Lock(),
        state_lock=threading.Lock(),
        commands=[zero] * 7,
        state=[object()],
    )
    robot = SimpleNamespace(
        motor_chain=chain,
        _server_thread=SimpleNamespace(is_alive=lambda: True),
        _command_lock=threading.Lock(),
        _state_lock=threading.Lock(),
        _commands=active,
        _joint_state=SimpleNamespace(pos=position),
        _last_motor_torques=None,
    )
    robot.get_observations = lambda: {
        "joint_pos": robot._joint_state.pos[:6],
        "gripper_pos": robot._joint_state.pos[6:],
    }

    def advance(seconds):
        clock.now += seconds
        clock.tick += 1
        if clock.tick == 1:
            robot._joint_state = SimpleNamespace(pos=position)
        if clock.tick == 2:
            robot._last_motor_torques = np.ones(7)
            chain.commands = [SimpleNamespace(pos=q, kp=1, kd=1) for q in position]
        if clock.tick in (3, 4) and failure != "cache":
            chain.state = [object()]
        if clock.tick == 5 and failure != "host":
            robot._joint_state = SimpleNamespace(pos=position)
        if failure == "writer":
            chain.running = False
        if failure == "background":
            owner._background_error = origin

    monkeypatch.setattr(vendor.time, "monotonic", lambda: clock.now)
    monkeypatch.setattr(vendor.time, "sleep", advance)
    if failure:
        with pytest.raises(RuntimeFault) as captured:
            vendor.wait_for_startup(owner, robot, "fake_can", timeout_s=0.02)
        assert captured.value.code == FaultCode.MOTOR_FAILURE
        if failure == "background":
            assert captured.value is origin
        else:
            assert captured.value.context["channel"] == "fake_can"
        assert not owner.report["startup_evidence"]["ready"]
        assert owner.report["startup_evidence"]["error"] == captured.value.as_dict()
    else:
        vendor.wait_for_startup(owner, robot, "fake_can", timeout_s=0.02)
        evidence = owner.report["startup_evidence"]
        assert evidence["ready"] and evidence["can_generations"] == 2
        assert clock.tick == 5
        assert evidence["position_rad"][2] == -0.0017


def backend_result(mode, *, outcome="arrived", error=0.001):
    trials = []
    for case_id in (
        CASE["case_id"] + "-reference",
        CASE["case_id"],
        CASE["case_id"] + "-return",
    ):
        trials.append(
            {
                "mode": mode,
                "part": "left",
                "case_id": case_id,
                "outcome": outcome,
                "start_rad": [0.0],
                "joint_error_rad": error,
                "tcp_error_m": error * 0.1,
                "displacement_m": 0.004,
                "elapsed_s": 1.2,
                "trajectory_duration_s": 1.0,
            }
        )
    return {
        "mode": mode,
        "trials": trials,
        "vendor_pristine": mode == "vendor",
        "velocity_feedforward": False,
        "control_settings": {"kp": [80], "gripper_limits": [0.04, -1.8]},
        "command_period_s": 0.1,
    }


COMPARISON = {
    "vendor": "i2rt",
    "initial_tolerance_rad": 0.02,
    "joint_error_margin_rad": 0.002,
    "tcp_error_margin_m": 0.002,
    "settling_margin_s": 0.1,
}


def test_pair_preserves_nonarrival_runs_other_backend_and_reuses_calibration(
    monkeypatch, tmp_path
):
    config = {
        "evidence_directory": str(tmp_path),
        "torque_release_authorized": True,
        "comparison": COMPARISON,
    }
    modes = []

    def child(command, **kwargs):
        mode = command[command.index("--backend") + 1]
        modes.append(mode)
        child_config = json.loads(
            Path(command[command.index("--config") + 1]).read_text()
        )
        if mode == "sdk":
            assert "--reference-only" in command
            assert child_config["part_options"]["left"]["gripper_limits"] == [
                0.04,
                -1.8,
            ]
        result = backend_result(mode, outcome="not_arrived", error=0.024376)
        result["trials"] = result["trials"][:1]
        Path(command[command.index("--output") + 1]).write_text(json.dumps(result))
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(paired.subprocess, "run", child)
    monkeypatch.setattr(paired, "reject_ci", lambda: None)
    report = paired.paired(config, CASE)
    assert modes == ["vendor", "sdk"]
    assert not report["verdict"]["passed"]
    assert [trial["joint_error_rad"] for trial in report["trials"]] == [0.024376] * 2
    assert json.loads((tmp_path / "paired-report.json").read_text()) == report


@pytest.mark.parametrize("failure", ["motor", "rest"])
def test_pair_aborts_after_motor_or_parking_failure_instead_of_opening_second_owner(
    monkeypatch, tmp_path, failure
):
    modes = []
    origin = RuntimeFault(
        FaultCode.MOTOR_FAILURE, "motor 3 on can_left stopped", context={"motor": 3}
    )

    def child(command, **kwargs):
        mode = command[command.index("--backend") + 1]
        modes.append(mode)
        result = backend_result(mode)
        if failure == "motor":
            result.update(error=origin.as_dict(), trials=[])
        else:
            result["rest_trials"] = [{"outcome": "not_arrived"}]
        Path(command[command.index("--output") + 1]).write_text(json.dumps(result))
        return SimpleNamespace(returncode=1 if failure == "motor" else 0)

    monkeypatch.setattr(paired.subprocess, "run", child)
    monkeypatch.setattr(paired, "reject_ci", lambda: None)
    report = paired.paired(
        {
            "evidence_directory": str(tmp_path),
            "torque_release_authorized": True,
            "rest_positions": {
                "left": {"joint_names": ["joint1"], "position_rad": [-0.03]}
            },
        },
        CASE,
        reference_only=True,
    )
    assert modes == ["vendor"]
    if failure == "motor":
        assert report["runs"][0]["error"] == origin.as_dict()
    else:
        assert report["runs"][0]["rest_trials"][0]["outcome"] == "not_arrived"
    assert not report["verdict"]["passed"]


def test_absolute_arrival_alone_cannot_pass_vendor_regression_or_mismatched_setup():
    baseline = {
        "config": {"comparison": COMPARISON},
        "runs": [backend_result("vendor"), backend_result("sdk")],
    }
    assert paired.compare(baseline, CASE)["passed"]

    worse = deepcopy(baseline)
    worse["runs"][1]["trials"][1]["joint_error_rad"] = 0.019
    verdict = paired.compare(worse, CASE)
    assert not verdict["passed"]
    assert (
        "SDK regression exceeds comparison.joint_error_margin_rad" in verdict["reasons"]
    )

    mismatch = deepcopy(baseline)
    mismatch["runs"][1]["trials"][1]["start_rad"] = [0.03]
    assert not paired.compare(mismatch, CASE)["passed"]
    mismatch = deepcopy(baseline)
    mismatch["runs"][1]["control_settings"]["kp"] = [100]
    assert not paired.compare(mismatch, CASE)["passed"]


def test_raw_measurements_reject_stalled_can_cache_without_trusting_read_timestamps(
    monkeypatch,
):
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr(vendor.time, "monotonic", lambda: clock.now)
    monkeypatch.setattr(vendor, "diagnostics", lambda robot: {})
    owner = object.__new__(vendor.VendorBench)
    owner.channel = "can_left"
    owner._cache = owner._cache_changed = None
    chain = SimpleNamespace(running=True, state_lock=threading.Lock(), state=[object()])
    owner.robot = SimpleNamespace(
        motor_chain=chain,
        _server_thread=SimpleNamespace(is_alive=lambda: True),
        get_observations=lambda: {
            "joint_pos": np.zeros(6),
            "joint_vel": np.zeros(6),
            "gripper_pos": np.zeros(1),
            "gripper_vel": np.zeros(1),
        },
    )
    assert owner.observe("left")[0].shape == (7,)
    clock.now = 0.51
    with pytest.raises(RuntimeFault, match="cache stalled") as captured:
        owner.observe("left")
    assert captured.value.code == FaultCode.MOTOR_FAILURE
    assert captured.value.context["channel"] == "can_left"
    chain.state = [object()]
    assert owner.observe("left")[0].shape == (7,)
    chain.running = False
    with pytest.raises(RuntimeFault, match="worker stopped"):
        owner.observe("left")
