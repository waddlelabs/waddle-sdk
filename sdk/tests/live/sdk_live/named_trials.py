"""Hardware acceptance scenarios; also exercised against deterministic plants."""

import time

import numpy as np
from waddle_sdk.runtime import FaultCode, JointPositionCommand


def observations(bench):
    deadline = time.monotonic() + bench.profile["observation_s"]
    while time.monotonic() < deadline:
        started = time.monotonic()
        bench.read()
        for part in bench.parts:
            bench.read([part])  # Exact subsets, with the other arm still open.
        bench.pause(started)
    bench.check_fresh()


def parallel(bench, cases):
    _first, neighbor, repeat = cases
    bench.references(cases[:2])
    initial = bench.read()
    motions = {case["part"]: bench.motion(case, initial) for case in cases[:2]}
    left, right = bench.parts
    assert motions[left]["duration_s"] < motions[right]["duration_s"], (
        "Profile needs a longer neighbor trajectory"
    )
    reused = False
    reuse_position = None
    while motions:
        started = time.monotonic()
        measured, arrived = bench.tick(motions)
        if not reused:
            assert right not in arrived, (
                "Neighbor arrived before the first arm could be reused"
            )
            if left in arrived:
                reuse_position = measured.parts[right].joint_position.copy()
                remaining = float(
                    np.max(np.abs(reuse_position - motions[right]["target"]))
                )
                assert remaining > neighbor["joint_tolerance_rad"], (
                    "Neighbor already inside arrival tolerance at reuse"
                )
                motions[left] = bench.motion(repeat, measured)
                before = bench.probes[right].sample()
                expected = next(
                    row["commands"][right]
                    for row in reversed(bench.report["named"]["submissions"])
                    if right in row["commands"]
                )
                indices = [
                    bench.names(right).index(name) for name in before["joint_names"]
                ]
                np.testing.assert_allclose(
                    before["command_position_rad"],
                    np.asarray(expected)[indices],
                    atol=bench.profile["mapping_tolerance_rad"],
                    rtol=0,
                )
                # Send ONLY the reused arm here. The existing neighbor stream
                # continues on subsequent ticks, under the same gate and site.
                bench.submit(
                    {left: JointPositionCommand(motions[left]["initial"])}, measured
                )
                after = bench.probes[right].sample()
                assert before["joint_names"] == after["joint_names"]
                np.testing.assert_allclose(
                    after["command_position_rad"],
                    before["command_position_rad"],
                    atol=bench.profile["mapping_tolerance_rad"],
                    rtol=0,
                )
                bench.report["named"]["reuse"] = {
                    "time_s": time.monotonic(),
                    "neighbor_position_rad": reuse_position.tolist(),
                    "neighbor_target_rad": motions[right]["target"].tolist(),
                    "neighbor_error_rad": remaining,
                    "neighbor_reference_before": before,
                    "neighbor_reference_after": after,
                }
                arrived.remove(left)
                reused = True
        if reused and right in arrived:
            progress = float(
                np.max(np.abs(measured.parts[right].joint_position - reuse_position))
            )
            bench.report["named"]["reuse"]["neighbor_progress_rad"] = progress
            assert progress >= bench.profile["min_progress_rad"], (
                "Neighbor made insufficient measured progress after sparse reuse"
            )
        for part in arrived:
            del motions[part]
        bench.pause(started)
    assert reused
    bench.check_fresh()
    bench.rest(cases[:2])  # Healthy trials only; failures go directly to teardown.


def envelope(bench, profile):
    observed = bench.read()
    part = profile["part"]
    joints = bench.spaces[part]["jointPosition"]["joints"]
    index = bench.names(part).index(profile["joint_name"])
    row = joints[index]
    target = profile["rejected_position_rad"]
    assert target < row["minPosition"] or target > row["maxPosition"], (
        "Reviewed refusal target must lie outside the declared joint envelope"
    )
    command = observed.parts[part].joint_position.copy()
    command[index] = target
    receipts = bench.submit(
        {part: JointPositionCommand(command)}, observed, refusal=True
    )
    receipt = receipts[part]
    assert receipt.part == part, "Refusal lost its addressed part"
    assert not receipt.dispatched and receipt.fault is not None, (
        "Out-of-envelope target was not refused"
    )
    assert receipt.fault.code == FaultCode.SAFETY_REFUSAL, receipt.fault.as_dict()
    # Preserve the exact refusal. No subsequent trajectory or parking command.
    bench.report["named"]["envelope_refusal"] = receipt.fault.as_dict()
    check_drift(bench, observed, profile["observe_s"], profile["max_drift_rad"])
    bench.check_fresh()


def check_drift(bench, baseline, seconds, maximum):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        started = time.monotonic()
        measured = bench.read()
        for part in bench.parts:
            drift = float(
                np.max(
                    np.abs(
                        measured.parts[part].joint_position
                        - baseline.parts[part].joint_position
                    )
                )
            )
            assert drift <= maximum, f"{part}: drift {drift} exceeds {maximum}"
        bench.pause(started)


def stop(bench, cases, profile):
    bench.references(cases[:2])
    initial = bench.read()
    motions = {case["part"]: bench.motion(case, initial) for case in cases[:2]}
    assert all(profile["after_s"] < m["duration_s"] for m in motions.values()), (
        "Stop must interrupt both trajectories"
    )
    deadline = time.monotonic() + profile["after_s"]
    while time.monotonic() < deadline:
        started = time.monotonic()
        _, arrived = bench.tick(motions)
        assert not arrived, "Stop trial unexpectedly completed a trajectory"
        bench.pause(started)
    moving = bench.read()
    for part in bench.parts:
        velocity = float(np.max(np.abs(moving.parts[part].joint_velocity)))
        assert velocity >= profile["min_velocity_rad_s"], (
            f"{part}: insufficient measured velocity to test stopping"
        )
    began = time.monotonic()
    evidence = bench.report["named"]["stop"] = {
        "method": profile["method"],
        "requested_s": began,
        "profile": profile,
        "response_s": {},
        "samples": [],
    }
    getattr(bench.session, profile["method"])("reviewed live stop acceptance")
    evidence["rpc_elapsed_s"] = time.monotonic() - began
    # No new targets or re-enable after this point. A software e-stop may release
    # torque; supported=true is explicit reviewed physical support for both arms.
    while time.monotonic() - began < profile["observe_s"]:
        started = time.monotonic()
        measured = bench.read()
        elapsed = time.monotonic() - began
        row = {"elapsed_s": elapsed, "parts": {}}
        evidence["samples"].append(row)
        for part in bench.parts:
            state = measured.parts[part]
            speed = float(np.max(np.abs(state.joint_velocity)))
            drift = float(
                np.max(np.abs(state.joint_position - moving.parts[part].joint_position))
            )
            row["parts"][part] = {"velocity_rad_s": speed, "drift_rad": drift}
            assert drift <= profile["max_drift_rad"], (
                f"{part}: excess motion after {profile['method']}"
            )
            if speed <= profile["max_velocity_rad_s"]:
                evidence["response_s"].setdefault(part, elapsed)
            else:
                evidence["response_s"].pop(part, None)
            if elapsed >= profile["response_s"]:
                assert speed <= profile["max_velocity_rad_s"], (
                    f"{part}: {profile['method']} failed to settle within response_s"
                )
        bench.pause(started)
    assert set(evidence["response_s"]) == set(bench.parts), evidence
    assert all(t <= profile["response_s"] for t in evidence["response_s"].values()), (
        evidence
    )
    assert (
        evidence["samples"]
        and evidence["samples"][-1]["elapsed_s"] >= profile["response_s"]
    ), evidence
    bench.check_fresh()
