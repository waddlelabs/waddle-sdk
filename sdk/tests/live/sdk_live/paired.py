"""Sequential fresh-process SDK/I2RT trials sharing one measured motion loop."""

import argparse
import json
import os
import subprocess
import sys
import threading
from copy import deepcopy
from pathlib import Path
from uuid import uuid4

import numpy as np
from waddle_sdk.robots import yam
from waddle_sdk.runtime import RuntimeFault

from .config import load, reject_ci
from .metrics import assert_arrived, write_report
from .session import Bench
from .vendor import VENDOR_GRAVITY_DEFAULT, VendorBench


def reference(case):
    result = {
        **case,
        "case_id": case["case_id"] + "-reference",
        "min_displacement_m": 0,
        "target_rad": case["start_rad"],
    }
    if "minimum_orientation_rad" in case:
        result["minimum_orientation_rad"] = 0
    return result


def rest_failed(report, config, part):
    """A configured rest must arrive before another backend can own the arm."""
    if part not in config.get("rest_positions", {}):
        return False
    trials = report.get("rest_trials", [])
    return len(trials) != 1 or trials[0]["outcome"] != "arrived"


def run_backend(config, part, case, mode, output, *, reference_only=False):
    """Only the child calls this. Bounded nonarrival is data, faults still raise."""
    try:
        owner = (VendorBench if mode == "vendor" else Bench)(config, part)
    except BaseException as error:
        write_report(
            output,
            {
                "schema": "waddle.live-benchmark/v1",
                "mode": mode,
                "part": part,
                "case_id": case["case_id"],
                "trials": [],
                "error": RuntimeFault.from_exception(error).as_dict(),
            },
        )
        raise
    owner.report["comparison_defaults"] = {
        "vendor_gravity_comp_factor": VENDOR_GRAVITY_DEFAULT,
        "sdk_gravity_comp_factor": list(yam.DEFAULT_GRAVITY_COMP_FACTOR),
        "effective_policy": "both use the site option or the unchanged SDK default",
    }
    owner.report["case_id"] = case["case_id"]
    owner.report["process_id"] = os.getpid()
    owner.report["background_errors"] = []
    owner._background_error = None
    previous_hook = threading.excepthook

    def record_thread_error(args):
        if owner._background_error is None:
            owner._background_error = args.exc_value
        owner.report["background_errors"].append(
            {
                "thread": args.thread.name,
                "error": RuntimeFault.from_exception(args.exc_value).as_dict(),
            }
        )
        previous_hook(args)

    def move(target, motion_case, **kwargs):
        if owner._background_error is not None:
            raise owner._background_error
        result = owner.move(part, target, motion_case, **kwargs)
        if owner._background_error is not None:
            raise owner._background_error
        return result

    threading.excepthook = record_thread_error
    try:
        with owner:
            ref = reference(case)
            trial = move(case["start_rad"], ref)
            # Do not advance after a failed reference. The parent can still run
            # the other backend and retain both bounded nonarrival results.
            if not reference_only and trial["outcome"] == "arrived":
                move(case["target_rad"], case)
                move(
                    case["start_rad"],
                    {**ref, "case_id": case["case_id"] + "-return"},
                )
            rest = config.get("rest_positions", {}).get(part)
            if rest is not None:
                rest_case = {
                    **ref,
                    "case_id": case["case_id"] + "-rest",
                    "joint_names": rest["joint_names"],
                    "target_rad": rest["position_rad"],
                }
                try:
                    move(rest["position_rad"], rest_case, report_key="rest_trials")
                except BaseException as error:
                    owner.report["rest_error"] = RuntimeFault.from_exception(
                        error
                    ).as_dict()
                    raise
    except BaseException as error:
        owner.report["error"] = RuntimeFault.from_exception(error).as_dict()
        write_report(output, owner.report)
        raise
    finally:
        threading.excepthook = previous_hook
    write_report(output, owner.report)
    return owner.report


def compare(report, case, *, reference_only=False):
    """Absolute success and comparison eligibility are independent verdicts."""
    reasons = []
    comparison = report["config"].get("comparison")
    if not reference_only and comparison is None:
        reasons.append("Full comparison requires explicit noninferiority margins")
    runs = report["runs"]
    for run in runs:
        if rest_failed(run, report["config"], case["part"]):
            reasons.append(f"{run['mode']} did not reach its configured rest position")
    if len(runs) != 2:
        reasons.append("Both backends must complete their bounded trial")
    else:
        vendor, sdk = runs
        for key in ("control_settings", "command_period_s", "velocity_feedforward"):
            if key not in vendor or key not in sdk or vendor[key] != sdk[key]:
                reasons.append(f"Backend settings differ: {key}")
        if not vendor.get("vendor_pristine"):
            reasons.append("Vendor process was not pristine")
        for run in runs:
            if (
                run.get("error")
                or run.get("shutdown_errors")
                or run.get("background_errors")
            ):
                reasons.append(f"{run['mode']} had an execution or shutdown fault")
            expected = 1 if reference_only else 3
            if len(run["trials"]) != expected:
                reasons.append(f"{run['mode']} stopped before all requested arrivals")
            for trial in run["trials"]:
                limits = (
                    case if trial["case_id"] == case["case_id"] else reference(case)
                )
                try:
                    assert_arrived(trial, limits)
                except AssertionError:
                    reasons.append(
                        f"{run['mode']} {trial['case_id']}: {trial['outcome']}; "
                        f"joint error {trial.get('joint_error_rad')} rad"
                    )
        matching = {trial["case_id"]: trial for trial in vendor["trials"]}
        differences = []
        for trial in sdk["trials"]:
            other = matching.get(trial["case_id"])
            if other is None or any(
                key not in row
                for row in (trial, other)
                for key in (
                    "start_rad",
                    "joint_error_rad",
                    "tcp_error_m",
                    "elapsed_s",
                    "trajectory_duration_s",
                )
            ):
                continue
            difference = float(
                np.max(np.abs(np.asarray(trial["start_rad"]) - other["start_rad"]))
            )
            row = {
                "case_id": trial["case_id"],
                "max_initial_difference_rad": difference,
                "sdk_minus_vendor_joint_error_rad": trial["joint_error_rad"]
                - other["joint_error_rad"],
                "sdk_minus_vendor_tcp_error_m": trial["tcp_error_m"]
                - other["tcp_error_m"],
                "sdk_minus_vendor_elapsed_s": trial["elapsed_s"] - other["elapsed_s"],
                "sdk_minus_vendor_settling_s": (
                    trial["elapsed_s"] - trial["trajectory_duration_s"]
                )
                - (other["elapsed_s"] - other["trajectory_duration_s"]),
                "comparable_initial_state": difference
                <= (
                    comparison["initial_tolerance_rad"]
                    if comparison
                    else case["joint_tolerance_rad"]
                ),
            }
            differences.append(row)
            # References document reacquisition from independently measured
            # starting states. Only matched target trials measure regression.
            if trial["case_id"] == case["case_id"]:
                if not row["comparable_initial_state"]:
                    reasons.append(
                        "Target trial initial states differ beyond the declared tolerance"
                    )
                if (
                    abs(trial["trajectory_duration_s"] - other["trajectory_duration_s"])
                    > sdk["command_period_s"]
                ):
                    reasons.append(
                        "Target trajectory durations differ by more than one command period"
                    )
                if comparison:
                    for metric, limit in (
                        ("joint_error_rad", "joint_error_margin_rad"),
                        ("tcp_error_m", "tcp_error_margin_m"),
                        ("settling_s", "settling_margin_s"),
                    ):
                        if row["sdk_minus_vendor_" + metric] > comparison[limit]:
                            reasons.append(f"SDK regression exceeds comparison.{limit}")
        report["differences"] = differences
    return {
        "passed": not reasons,
        "reasons": reasons,
        "scope": "matched external command cadence; not maximum native vendor throughput",
    }


def paired(config, case, *, reference_only=False):
    reject_ci()
    if config.get("torque_release_authorized") is not True:
        raise ValueError("Paired trials require explicit torque_release_authorized")
    if not reference_only and "comparison" not in config:
        raise ValueError("Full paired trials require explicit comparison margins")
    directory = Path(config["evidence_directory"])
    run_id = uuid4().hex
    directory.mkdir(parents=True, exist_ok=True)
    report = {
        "schema": "waddle.live-benchmark/v1",
        "run_id": run_id,
        "config": config,
        "runs": [],
        "trials": [],
        "case_id": case["case_id"],
        "reference_only": reference_only,
    }
    # Pass the resolved profile, including command-line site overrides. JSON is
    # an argument file, never interpolated as shell code.
    config_path = directory / f"paired-{run_id}-config.json"
    child_config = deepcopy(config)
    write_report(config_path, child_config)
    env = dict(os.environ)
    tests_root = str(Path(__file__).resolve().parents[2])
    env["PYTHONPATH"] = tests_root + os.pathsep + env.get("PYTHONPATH", "")
    for mode in ("vendor", "sdk"):
        output = directory / f"paired-{run_id}-{mode}.json"
        log = directory / f"paired-{run_id}-{mode}.log"
        command = [
            sys.executable,
            "-m",
            "live.sdk_live.paired",
            "--config",
            str(config_path),
            "--part",
            case["part"],
            "--case",
            case["case_id"],
            "--backend",
            mode,
            "--output",
            str(output),
        ]
        preceding_reference_failed = bool(report["runs"]) and (
            report["runs"][0]["trials"][0]["outcome"] != "arrived"
        )
        if reference_only or preceding_reference_failed:
            command.append("--reference-only")
        print(
            f"Starting {mode} {case['part']}/{case['case_id']}; evidence {output}",
            flush=True,
        )
        # No timeout kill: process death while holding a robot is not teardown.
        # Each motion has its own bounded deadline and explicit close policy.
        with log.open("w") as stream:
            result = subprocess.run(
                command, env=env, stdout=stream, stderr=subprocess.STDOUT, check=False
            )
        child = (
            json.loads(output.read_text())
            if output.exists()
            else {
                "mode": mode,
                "trials": [],
                "error": {"detail": "Child exited without a report"},
            }
        )
        child["process_returncode"] = result.returncode
        child["log_path"] = str(log)
        report["runs"].append(child)
        report["trials"].extend(child["trials"])
        report["verdict"] = compare(report, case, reference_only=reference_only)
        write_report(directory / f"paired-{run_id}.json", report)
        write_report(directory / "paired-report.json", report)
        if (
            result.returncode
            or child.get("error")
            or child.get("shutdown_errors")
            or child.get("background_errors")
            or rest_failed(child, config, case["part"])
        ):
            # A real fault blocks the next owner. A healthy but inaccurate
            # bounded trial above deliberately does not trigger this branch.
            break
        if mode == "vendor" and "gripper_limits" in child.get("control_settings", {}):
            # The vendor just measured its own motor-unit jaw calibration. Reuse
            # those exact limits for the SDK child, avoiding a second moving
            # calibration and a different normalized hand mapping in the pair.
            options = child_config.setdefault("part_options", {}).setdefault(
                case["part"], {}
            )
            options["gripper_limits"] = child["control_settings"]["gripper_limits"]
            child_config["gripper_limits_source"] = str(output)
            write_report(config_path, child_config)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--part", required=True)
    parser.add_argument("--case", required=True)
    parser.add_argument("--reference-only", action="store_true")
    parser.add_argument("--backend", choices=("vendor", "sdk"))
    parser.add_argument("--output")
    args = parser.parse_args()
    reject_ci()
    config = load(args.config)
    case = next(
        row
        for row in config["cases"]
        if row["part"] == args.part and row["case_id"] == args.case
    )
    if args.backend:
        if not args.output:
            parser.error("--backend requires --output")
        run_backend(
            config,
            args.part,
            case,
            args.backend,
            args.output,
            reference_only=args.reference_only,
        )
        return 0
    report = paired(config, case, reference_only=args.reference_only)
    print(json.dumps(report["verdict"], indent=2))
    return 0 if report["verdict"]["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
