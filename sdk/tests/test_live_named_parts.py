"""Deterministic checks of measured live verdicts; no hardware or discovery."""

from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest
from live import discovery
from live.sdk_live import named, named_config, named_trials
from test_live_benchmark import CASE
from test_live_selection import Config
from waddle_sdk.runtime import (
    FaultCode,
    JointPositionCommand,
    Observation,
    PartObservation,
    RuntimeEvent,
    RuntimeFault,
    SubmitResult,
)


def configuration():
    first = {**CASE, "case_id": "first", "target_rad": [0.08], "settle_s": 1.0}
    neighbor = {
        **first,
        "case_id": "neighbor",
        "part": "right",
        "target_rad": [0.5],
        "velocity_rad_s": 0.1,
    }
    repeat = {**first, "case_id": "repeat", "start_rad": [0.08], "target_rad": [0.0]}
    return {
        "parts": ["left", "right"],
        "cases": [first, neighbor, repeat],
        "max_tracking_error_rad": 0.2,
        "rest_positions": {
            p: {"joint_names": ["joint1"], "position_rad": [0.0]}
            for p in ("left", "right")
        },
        "named_parts": {
            "parts": ["left", "right"],
            "observation_s": 0.6,
            "max_observation_latency_s": 0.02,
            "max_feedback_gap_s": 0.2,
            "mapping_tolerance_rad": 0.005,
            "max_command_latency_s": 0.02,
            "max_command_gap_s": 0.15,
            "min_progress_rad": 0.1,
            "motion_cases": [
                {"part": c["part"], "case_id": c["case_id"]}
                for c in (first, neighbor, repeat)
            ],
            "envelope": {
                "part": "left",
                "joint_name": "joint1",
                "rejected_position_rad": 2.0,
                "observe_s": 0.6,
                "max_drift_rad": 0.01,
            },
            "stops": [
                {
                    "method": m,
                    "supported": True,
                    "after_s": 0.7,
                    "response_s": 0.2,
                    "observe_s": 0.6,
                    "min_velocity_rad_s": 0.002,
                    "max_velocity_rad_s": 0.001,
                    "max_drift_rad": 0.01,
                }
                for m in ("hold", "estop")
            ],
        },
    }


def plant(monkeypatch):
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr(named.time, "monotonic", lambda: clock.now)
    monkeypatch.setattr(
        named.time, "sleep", lambda s: setattr(clock, "now", clock.now + s)
    )
    bench = object.__new__(named.NamedBench)
    bench.config = configuration()
    bench.profile = bench.config["named_parts"]
    bench.parts = bench.profile["parts"]
    bench.site = SimpleNamespace(
        manifest={"parts": {p: {"base_frame": p + "_base"} for p in bench.parts}}
    )
    # Distinct widths and nontrivial order catch flattening or routing mistakes.
    axes = {"left": ["joint2", "joint1"], "right": ["joint1", "joint3", "joint2"]}
    bench.spaces = {
        p: {
            "jointPosition": {
                "joints": [
                    {"name": n, "minPosition": -1, "maxPosition": 1} for n in names
                ]
            }
        }
        for p, names in axes.items()
    }
    bench.report = {"named": {"samples": [], "submissions": [], "arrivals": []}}
    bench._event_cursor = 0
    bench._part_faults = {}
    bench.period = 0.05
    bench.fresh, bench.last_commands = {}, {}
    bench.save = lambda: None
    bench.pose = lambda p, q: np.array([q[axes[p].index("joint1")] * 0.1, 0, 0])
    bench.rotation = lambda *_: None
    q = {p: np.zeros(len(names)) for p, names in axes.items()}
    target = {p: v.copy() for p, v in q.items()}
    velocity = {p: v.copy() for p, v in q.items()}
    state = SimpleNamespace(
        last=0.0,
        stop_at=None,
        stops=[],
        frozen=set(),
        fault=None,
        stale=False,
        bad_frame=False,
        latency=0.0,
        runaway=False,
        events=[],
    )

    def advance():
        dt = clock.now - state.last
        if dt <= 0:
            return
        stopped = state.stop_at is not None and clock.now >= state.stop_at
        for p, positions in q.items():
            delta = np.clip(target[p] - positions, -0.5 * dt, 0.5 * dt)
            if state.runaway:
                delta = np.full_like(delta, 0.5 * dt)
            if stopped or p in state.frozen:
                delta *= 0
            q[p] += delta
            velocity[p] = delta / dt
        state.last = clock.now

    def probe(p):
        advance()
        return {
            "source": "deterministic plant only",
            "generation": 0 if state.stale else round(clock.now / bench.period),
            "ingestion": round(clock.now / bench.period),
            "joint_names": axes[p],
            "position_rad": q[p].tolist(),
            "command_position_rad": target[p].tolist(),
        }

    bench.probes = {p: SimpleNamespace(sample=lambda p=p: probe(p)) for p in q}

    def read(parts):
        advance()
        clock.now += state.latency
        return Observation(
            round(clock.now * 1e9),
            0,
            {
                p: PartObservation(
                    q[p].copy(),
                    velocity[p].copy(),
                    frame_id="wrong" if state.bad_frame else p + "_base",
                )
                for p in parts
                if state.fault is None or p != "left"
            },
            {},
            {"left": state.fault}
            if state.fault is not None and "left" in parts
            else {},
        )

    def submit(commands, observed):
        result = {}
        for p, c in commands.items():
            if np.max(np.abs(c.positions)) > 1:
                error = RuntimeFault(
                    FaultCode.SAFETY_REFUSAL,
                    "joint1 outside [-1, 1]",
                    context={"part": p, "joint": "joint1"},
                )
                result[p] = SubmitResult(False, "owner_refusal", p, fault=error)
            else:
                target[p] = np.asarray(c.positions).copy()
                result[p] = SubmitResult(True, "pass", p)
        return result

    def stop(method):
        state.stops.append(method)
        state.stop_at = clock.now + bench.period

    bench.session = SimpleNamespace(
        observe_parts=read,
        hold=lambda _: stop("hold"),
        estop=lambda _: stop("estop"),
        events=lambda after: tuple(
            event for event in state.events if event.cursor > after
        ),
    )
    bench.run = SimpleNamespace(step_parts=submit)
    return bench, state


def test_parallel_verdict_requires_independent_measured_arrival_and_reuse(monkeypatch):
    bench, state = plant(monkeypatch)
    named_trials.parallel(bench, named_config.cases(bench.config))
    evidence = bench.report["named"]
    assert evidence["reuse"]["neighbor_error_rad"] > CASE["joint_tolerance_rad"]
    assert evidence["reuse"]["neighbor_progress_rad"] >= 0.1
    assert any(set(s["commands"]) == {"left", "right"} for s in evidence["submissions"])
    assert any(set(s["commands"]) == {"left"} for s in evidence["submissions"])
    assert len(evidence["arrivals"]) == 7  # Two references, three motions, two rests.
    assert not state.stops  # A settled arm never invokes shared stopping.


def test_neighbor_freeze_cannot_pass_on_accepted_commands(monkeypatch):
    bench, state = plant(monkeypatch)
    state.frozen.add("right")
    with pytest.raises(AssertionError, match="tracking error|arrival timed out"):
        named_trials.parallel(bench, named_config.cases(bench.config))
    assert len(bench.report["named"]["arrivals"]) < 7


@pytest.mark.parametrize("failure", ["stale", "bad_frame", "latency", "read_fault"])
def test_observation_verdict_rejects_bad_feedback_and_preserves_fault(
    monkeypatch, failure
):
    bench, state = plant(monkeypatch)
    origin = RuntimeFault(
        FaultCode.MOTOR_FAILURE, "left motor 2 stopped", context={"motor": 2}
    )
    if failure == "read_fault":
        state.fault = origin
    else:
        setattr(state, failure, 0.1 if failure == "latency" else True)
    with pytest.raises((AssertionError, RuntimeFault)) as captured:
        named_trials.observations(bench)
    if failure == "read_fault":
        assert captured.value is origin
        assert (
            bench.report["named"]["samples"][-1]["faults"]["left"] == origin.as_dict()
        )


def test_observations_and_envelope_use_exact_subsets_and_measured_drift(monkeypatch):
    bench, _ = plant(monkeypatch)
    named_trials.observations(bench)
    assert {tuple(row["parts"]) for row in bench.report["named"]["samples"]} == {
        ("left",),
        ("right",),
        ("left", "right"),
    }
    named_trials.envelope(bench, bench.profile["envelope"])
    assert (
        bench.report["named"]["envelope_refusal"]["detail"] == "joint1 outside [-1, 1]"
    )
    assert len(bench.report["named"]["submissions"]) == 1


@pytest.mark.parametrize("method", ["hold", "estop"])
@pytest.mark.parametrize("effective", [True, False])
def test_stop_requires_hardware_response_after_both_arms_move(
    monkeypatch, method, effective
):
    bench, state = plant(monkeypatch)
    if not effective:
        setattr(bench.session, method, lambda _: setattr(state, "runaway", True))
    profile = next(p for p in bench.profile["stops"] if p["method"] == method)
    if effective:
        named_trials.stop(bench, named_config.cases(bench.config), profile)
        assert state.stops == [method]
        assert set(bench.report["named"]["stop"]["response_s"]) == {"left", "right"}
    else:
        with pytest.raises(AssertionError, match="failed to settle|excess motion"):
            named_trials.stop(bench, named_config.cases(bench.config), profile)


@pytest.mark.parametrize(
    "change",
    [
        "unselected",
        "broken_chain",
        "missing_rest",
        "missing_limits",
        "unsupported_stop",
    ],
)
def test_unreviewed_profiles_are_rejected_before_opening_hardware(change):
    config = configuration()
    named_config.validate(config)
    if change == "unselected":
        config["named_parts"]["parts"] = ["left", "third"]
    elif change == "broken_chain":
        config["cases"][2]["start_rad"] = [0.2]
    elif change == "missing_rest":
        config.pop("rest_positions")
    elif change == "missing_limits":
        config["named_parts"].pop("max_command_gap_s")
    else:
        config["named_parts"]["stops"][0]["supported"] = False
    with pytest.raises(ValueError):
        named_config.validate(config)


def test_missing_second_arm_or_profile_only_disables_dependent_tests(monkeypatch):
    config = Config(live=True)
    config.live_bench = configuration()
    config.live_bench["site"] = "unused.yaml"
    config.live_missing = {"parts:right": "right absent"}
    markers = {"named_parts"}
    item = SimpleNamespace(
        config=config,
        get_closest_marker=lambda name: name if name in markers else None,
        callspec=SimpleNamespace(params={}),
    )
    assert discovery.reason(item) == "right absent"
    markers.clear()
    item.callspec.params = {"part": "left"}
    assert discovery.reason(item) is None
    markers.add("named_parts")
    config.live_missing = {}
    monkeypatch.setattr(
        discovery,
        "load_site",
        lambda _: SimpleNamespace(
            manifest={
                "parts": {
                    p: {"driver": "waddle_sdk.robots.yam:arm"}
                    for p in ("left", "right")
                }
            }
        ),
    )
    assert discovery.reason(item) is None
    item.callspec.params = {"named_stop": None}
    assert "reviewed stops" in discovery.reason(item)
    item.callspec.params = {}
    markers.add("named_motion")
    config.live_bench["named_parts"].pop("motion_cases")
    assert "motion_cases" in discovery.reason(item)


def test_projected_pair_retains_identity_without_opening_other_parts(tmp_path):
    import yaml
    from test_site_api import _write_site

    path = _write_site(tmp_path)
    document = yaml.safe_load(path.read_text())
    document["parts"] = {
        p: deepcopy(document["parts"]["arm"]) for p in ("left", "right", "unselected")
    }
    path.write_text(yaml.safe_dump(document))
    config = {**configuration(), "site": str(path)}
    bench = named.NamedBench(config)
    assert bench.site.id == "test-cell"
    assert set(bench.site.manifest["parts"]) == {"left", "right"}
    assert not bench.site.manifest["cameras"] and bench.session is None


@pytest.mark.parametrize("historical", [False, True])
def test_submit_fault_is_saved_without_rewrapping_or_followup_command(
    monkeypatch, historical
):
    bench, state = plant(monkeypatch)
    origin = RuntimeFault(
        FaultCode.MOTOR_FAILURE, "motor 2 write failed", context={"part": "left"}
    )
    observed = bench.read()
    if historical:
        state.events.append(
            RuntimeEvent(
                1, "robot.part_fault", 42, {"part": "left", "fault": origin.as_dict()}
            )
        )
    else:
        bench.run.step_parts = lambda *_: {
            "left": SubmitResult(False, "owner_refusal", "left", fault=origin)
        }
    with pytest.raises(RuntimeFault) as captured:
        bench.submit({"left": JointPositionCommand([0, 0])}, observed)
    if historical:
        assert captured.value.as_dict() == origin.as_dict()
        assert not bench.report["named"]["submissions"]
        assert bench.report["part_faults"][0]["fault"] == origin.as_dict()
        return
    assert captured.value is origin
    assert (
        bench.report["named"]["submissions"][0]["receipts"]["left"]["fault"]
        == origin.as_dict()
    )


@pytest.mark.parametrize("delayed", ["command", "cadence"])
def test_timing_bounds_fail_even_when_driver_accepts_targets(monkeypatch, delayed):
    bench, _ = plant(monkeypatch)
    commands = {
        p: JointPositionCommand(np.zeros(len(bench.names(p)))) for p in bench.parts
    }
    bench.submit(commands, bench.read())
    observed = bench.read()
    if delayed == "cadence":
        named.time.sleep(2 * bench.profile["max_command_gap_s"])
    else:
        submit = bench.run.step_parts

        def slow(*args):
            named.time.sleep(2 * bench.profile["max_command_latency_s"])
            return submit(*args)

        bench.run.step_parts = slow
    with pytest.raises(AssertionError):
        bench.submit(commands, observed)
    assert len(bench.report["named"]["submissions"]) == 2
    receipts = bench.report["named"]["submissions"][-1]["receipts"]
    assert all(row["dispatched"] for row in receipts.values())


class MockFeedback:
    """Native mock adapter evidence; explicitly not device acquisition proof."""

    def __init__(self, bench, part):
        self.driver = bench.session._managed.arms[part].driver
        self.names = bench.names(part)
        self.generation = 0

    def sample(self):
        self.generation += 1
        return {
            "source": "native mock simulation only",
            "generation": self.generation,
            "ingestion": self.generation,
            "joint_names": self.names,
            "position_rad": self.driver.read()[0].tolist(),
            "command_position_rad": self.driver._target.tolist(),
        }


def test_native_sdk_named_harness_records_real_receipts_and_closes_site(tmp_path):
    import json

    import yaml
    from test_site_api import _write_site

    path = _write_site(tmp_path)
    document = yaml.safe_load(path.read_text())
    document["parts"] = {
        p: {
            **document["parts"]["arm"],
            "driver": "waddle_sdk.robots.mock:arm",
            "options": {"joint_count": 2},
        }
        for p in ("left", "right")
    }
    path.write_text(yaml.safe_dump(document))
    config = {
        **configuration(),
        "site": str(path),
        "evidence_directory": str(tmp_path / "evidence"),
        "torque_release_authorized": True,
    }
    config["named_parts"]["feedback_probes"] = {
        p: "test_live_named_parts:MockFeedback" for p in ("left", "right")
    }
    config["named_parts"]["envelope"]["rejected_position_rad"] = 10.0
    with named.NamedBench(config) as bench:
        bench.require(action=True)
        bench.profile["envelope"]["joint_name"] = bench.names("left")[0]
        observed = bench.read()
        receipt = bench.submit({"right": JointPositionCommand([0.001, 0.0])}, observed)
        assert receipt["right"].dispatched
        named_trials.envelope(bench, bench.profile["envelope"])
    report = json.loads((tmp_path / "evidence/sdk-named-report.json").read_text())
    assert report["shutdown_errors"] == []
    assert report["named"]["envelope_refusal"]["code"] == "safety_refusal"
    assert report["site_id"] == "test-cell"


def test_yam_probe_counts_device_replacement_not_reads_or_envelope_stamps():
    from threading import Lock

    from live.sdk_live.feedback import YamFeedback

    chain = SimpleNamespace(running=True, state=object(), state_lock=Lock())
    robot = SimpleNamespace(
        motor_chain=chain,
        _server_thread=SimpleNamespace(is_alive=lambda: True),
        _state_lock=Lock(),
        _command_lock=Lock(),
        _joint_state=SimpleNamespace(pos=np.zeros(7)),
        _commands=SimpleNamespace(pos=np.ones(7)),
    )
    owner = SimpleNamespace(
        session=SimpleNamespace(
            _managed=SimpleNamespace(
                arms={"left": SimpleNamespace(driver=SimpleNamespace(_robot=robot))}
            )
        ),
        profile={"max_feedback_gap_s": 0.01},
    )
    probe = YamFeedback(owner, "left")
    initial = probe.sample()
    assert probe.sample() == initial
    chain.state = object()
    after_can = probe.sample()
    assert after_can["generation"] > initial["generation"]
    assert after_can["ingestion"] == initial["ingestion"]
    robot._joint_state = SimpleNamespace(pos=np.full(7, 0.2))
    ingested = probe.sample()
    assert ingested["ingestion"] > after_can["ingestion"]
    assert ingested["position_rad"] == [0.2] * 6
    assert ingested["command_position_rad"] == [1.0] * 6
    chain.running = False
    with pytest.raises(RuntimeFault) as captured:
        probe.sample()
    assert captured.value.context["part"] == "left"


@pytest.mark.parametrize("failure", [False, True])
def test_pair_startup_evidence_keeps_each_arm_including_original_failure(
    monkeypatch, tmp_path, failure
):
    import yaml
    from live.sdk_live import session, vendor
    from test_site_api import _write_site

    path = _write_site(tmp_path)
    document = yaml.safe_load(path.read_text())
    document["parts"] = {
        p: {**document["parts"]["arm"], "driver": "waddle_sdk.robots.yam:arm"}
        for p in ("left", "right")
    }
    path.write_text(yaml.safe_dump(document))
    owner = session.Bench(
        {"site": str(path), "torque_release_authorized": True}, parts=["left", "right"]
    )
    origin = RuntimeFault(
        FaultCode.MOTOR_FAILURE, "right startup failed", context={"part": "right"}
    )
    fake = SimpleNamespace(
        _managed=SimpleNamespace(
            arms={
                p: SimpleNamespace(driver=SimpleNamespace(_robot=p, channel=p))
                for p in ("left", "right")
            }
        ),
        run=lambda **_: SimpleNamespace(__enter__=lambda: None),
        describe=dict,
        close=lambda **_: None,
        events=lambda after_cursor: (),
    )
    owner.site = SimpleNamespace(
        manifest=owner.site.manifest,
        open=lambda **_: SimpleNamespace(__enter__=lambda: fake),
    )
    monkeypatch.setattr(
        session,
        "part_action_spaces",
        lambda _: {p: {"rateHz": 100} for p in ("left", "right")},
    )
    monkeypatch.setattr(vendor, "settings", lambda robot: {"robot": robot})

    def startup(bench, robot, channel):
        bench.report["startup_evidence"] = {
            "channel": channel,
            "ready": not (failure and channel == "right"),
        }
        if failure and channel == "right":
            bench.report["startup_evidence"]["error"] = origin.as_dict()
            raise origin

    monkeypatch.setattr(vendor, "wait_for_startup", startup)
    owner.save = lambda: None
    if failure:
        with pytest.raises(RuntimeFault) as captured:
            owner.__enter__()
        assert captured.value is origin
    else:
        with owner:
            pass
    evidence = owner.report["part_startup"]
    assert evidence["left"]["startup_evidence"]["ready"]
    assert evidence["right"]["startup_evidence"]["ready"] is not failure
    assert evidence["left"]["control_settings"] == {"robot": "left"}
    assert not owner.report["shutdown_errors"]
