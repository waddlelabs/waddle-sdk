"""The published example (`examples/toy_robot.py`) has to keep working.

It is the program we point customers at and the robot half of the
cross-repo end-to-end proof, so it is tested the way it is run: as a real
subprocess, offline, with no transport in the environment.

The subprocess test cannot see everything, though. Offline, `publish_frame`
is a documented no-op that returns before it validates anything — so a
camera declaration that disagrees with the rendered frame, or an uplink
policy core would reject, stays invisible until the day someone points the
example at a real plane. The tests that follow it run the example's own
loop in-process against the loopback media plane, which is where the frames
it publishes, the proprioception it reports, and the actions it sends under
an intervention all become observable.
"""

import importlib.util
import json
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest
from mcap.reader import make_reader
from mcap_protobuf.decoder import DecoderFactory

import waddle_sdk
import waddle_sdk._testing

EXAMPLE = Path(__file__).resolve().parents[1] / "examples" / "toy_robot.py"


def _load_example():
    """Import `examples/toy_robot.py` as a module (it is not on the path,
    and it is deliberately not part of the installed package). It has to
    land in `sys.modules` before it executes — `@dataclass` resolves its
    own annotations through there."""
    name = "waddle_toy_robot_example"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, EXAMPLE)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(autouse=True)
def _clean_session():
    yield
    waddle_sdk.shutdown()


def _until(predicate, what: str, timeout: float = 20.0, tick=None):
    """Wait for something the example does to become observable, doing
    `tick()` (if given) while waiting. Bounded only so a broken build fails
    instead of hanging — the wait always ends on the observation, never on
    the clock."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        got = predicate()
        if got:
            return got
        if tick is not None:
            tick()
        time.sleep(0.005)
    pytest.fail(what)


def _decoded_observations(mcap_path):
    """Every `/waddle/observations` message, decoded through the schema the
    MCAP embeds (see tests/test_e2e.py)."""
    with open(mcap_path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        return [
            msg
            for _, channel, _, msg in reader.iter_decoded_messages()
            if channel.topic == "/waddle/observations"
        ]


def test_example_runs_offline_and_records_every_episode(tmp_path):
    env = dict(os.environ)
    # Offline is the point of this run: never inherit a developer's plane.
    for leaked in (
        "WADDLE_TOY_TRANSPORT",
        "WADDLE_TOY_TOKEN",
        "WADDLE_TOY_MEDIA",
        "WADDLE_TOY_MEDIA_TOKEN",
        "WADDLE_TOY_MODE",
    ):
        env.pop(leaked, None)
    env["WADDLE_TOY_EPISODES"] = "2"
    env["WADDLE_TOY_EPISODE_SECONDS"] = "0.5"
    env["WADDLE_TOY_RECORDING_DIR"] = str(tmp_path)

    done = subprocess.run(
        [sys.executable, str(EXAMPLE)],
        env=env,
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert done.returncode == 0, f"stdout:\n{done.stdout}\nstderr:\n{done.stderr}"

    # The status lines another process waits on.
    assert "[toy] session up " in done.stdout
    assert "[toy] rollout 1 done success" in done.stdout
    assert "[toy] rollout 2 done success" in done.stdout

    sidecars = sorted(tmp_path.glob("*.sidecar.json"))
    mcaps = sorted(tmp_path.glob("*.mcap"))
    assert len(sidecars) == 2, "one sidecar per episode"
    assert len(mcaps) == 2, "one MCAP per episode"
    assert (tmp_path / "manifest.jsonl").exists()

    for sidecar in sidecars:
        assert json.loads(sidecar.read_text())["outcome"] == "TERMINAL_OUTCOME_SUCCESS"

    # The recordings have to hold the episode, not just exist: ~20 Hz for
    # half a second, so at least a handful of gated actions and the
    # observations that go with them.
    with open(mcaps[0], "rb") as f:
        counts: dict[str, int] = {}
        for _, channel, _ in make_reader(f).iter_messages():
            counts[channel.topic] = counts.get(channel.topic, 0) + 1
    assert counts.get("/waddle/actions", 0) >= 5
    assert counts.get("/waddle/observations", 0) >= 5


def test_the_examples_declared_camera_matches_the_frames_it_renders():
    example = _load_example()
    arm = example.ToyArm()

    session = waddle_sdk.init(
        "py-toy-example",
        example.robot_description(),
        waddle_sdk.Control(send=lambda chunk: None, hold=arm.hold, estop=arm.estop),
        _testing=True,
    )

    # `render` is deterministic in (pose, gripper, frame index), so a
    # pristine twin renders exactly what `arm` is about to publish — and
    # rendering it here does not advance `arm`'s own frame index.
    expected = example.ToyArm().render()
    assert expected.dtype == np.uint8
    assert expected.shape == (example.CAMERA_H, example.CAMERA_W, 3)

    # The example's OWN housekeeping tick, not a re-typed copy of it: this
    # test is worth nothing if it keeps passing while `robot_tick` grows a
    # different notion of what to publish. Nothing has commanded the arm, so
    # the step moves it nowhere.
    example.robot_tick(session, arm, 1.0 / example.CONTROL_HZ)

    deadline = time.monotonic() + 5.0
    got: list[bytes] = []
    while time.monotonic() < deadline:
        got = waddle_sdk._testing.frames(session, example.CAMERA_NAME)
        if got:
            break
        time.sleep(0.005)
    assert got, "the declared camera never accepted the example's own frame"
    assert got[-1] == expected.tobytes()


def test_the_examples_loop_publishes_frames_and_reports_proprio(tmp_path):
    # The brief's "publishes frames and proprio at 20 Hz from the main
    # loop" — asserted against the example's OWN rollout, not against a
    # call this test makes on its behalf. Ripping `publish_frame` or
    # `report_proprio` out of `robot_tick` (or the `robot_tick` call out of
    # the loop) has to fail here: offline `publish_frame` leaves no trace at
    # all, and the gate's own obs records go on arriving without a single
    # `report_proprio`, so nothing else in this file would notice.
    example = _load_example()
    arm = example.ToyArm()
    session = waddle_sdk.init(
        "py-toy-loop",
        example.robot_description(),
        waddle_sdk.Control(send=lambda chunk: None, hold=arm.hold, estop=arm.estop),
        recording_dir=tmp_path,
        _testing=True,
    )

    outcome = example.run_rollout(session, arm, 1, "publish while you roll", 0.5)
    assert outcome == "success"

    frames = _until(
        lambda: waddle_sdk._testing.frames(session, example.CAMERA_NAME),
        "the example's loop never published a frame",
    )
    assert len(frames[-1]) == example.CAMERA_W * example.CAMERA_H * 3

    waddle_sdk.shutdown()
    observations = _decoded_observations(next(iter(tmp_path.glob("*.mcap"))))
    assert observations, "the episode recorded no observations"
    last = observations[-1].proprio
    # joint_pos rides the gate; everything below can only have come from
    # the loop's own `report_proprio`.
    assert len(last.joint_pos) == len(example._CHAIN)
    assert len(last.joint_vel) == len(example._CHAIN)
    assert last.ee_pose.frame_id == example._TOOL_FRAME
    # A real FK pose, not a default-initialized one: the tool sits ~0.77 m
    # up the chain at home and never reaches the base plane.
    assert last.ee_pose.position.z > 0.1


def test_the_example_sends_an_intervening_claimants_gripper(tmp_path):
    # The claimant's gripper does NOT ride `gate()`'s return value — it
    # arrives on `last_gate.gripper`, already mapped out of the normalized
    # 0..1 wire into this robot's declared metres. An example that sent its
    # own scripted gripper alongside the substituted joints would leave a
    # teleoperator's grasp silently unexecuted, so the loop is run for real
    # and the values it sends are recorded.
    example = _load_example()

    class RecordingArm(example.ToyArm):
        def __init__(self) -> None:
            super().__init__()
            self.gripper_commands: list[float | None] = []

        def command(self, joints, gripper=None) -> None:
            self.gripper_commands.append(gripper)
            super().command(joints, gripper)

    arm = RecordingArm()
    session = waddle_sdk.init(
        "py-toy-intervention",
        example.robot_description(),
        waddle_sdk.Control(send=lambda chunk: None, hold=arm.hold, estop=arm.estop),
        recording_dir=tmp_path,
        _testing=True,
    )

    # Normalized 0.5 lands halfway between the declared 0.0 open and 0.04
    # closed. The scripted policy only ever commands the endpoints, so
    # 0.02 m can have come from nowhere but the claimant — through core's
    # own unit conversion.
    claimant_normalized = 0.5
    expected_m = 0.02

    failed: list[BaseException] = []

    def drive() -> None:
        # A long episode: it ends when this test has its answer, not when a
        # timer runs out.
        try:
            example.run_rollout(session, arm, 1, "intervene on me", 60.0)
        except BaseException as exc:  # re-raised on the main thread
            failed.append(exc)

    loop = threading.Thread(target=drive, name="toy-rollout", daemon=True)
    loop.start()
    try:
        _until(lambda: arm.gripper_commands, "the example's loop never dispatched an action")
        waddle_sdk._testing.engage(session, "claim-1", "teleop")
        _until(
            lambda: any(g == pytest.approx(expected_m) for g in list(arm.gripper_commands)),
            "the example never sent the claimant's gripper (it sent its own)",
            tick=lambda: waddle_sdk._testing.push_teleop(
                session, [0.3, 0.0, 0.0, 0.0, 0.0, 0.0], claimant_normalized
            ),
        )
    finally:
        # End the episode the instant the answer is in (or the instant this
        # test gives up): the loop breaks on `ep.done`, so nothing here ever
        # waits out those 60 seconds. Skipped if the loop already died — its
        # own exception is the interesting one, not "no live episode".
        if loop.is_alive() and not failed:
            waddle_sdk._testing.mark_done(session, "success")
        loop.join(timeout=10.0)
    if failed:
        raise failed[0]
    assert not loop.is_alive(), "the example's loop ignored a plane-ended episode"
    # The scripted endpoints are still what it sends when nobody overrides.
    assert example.GRIPPER_OPEN_M in arm.gripper_commands


def test_the_examples_agent_run_keeps_the_robot_running_while_it_blocks(tmp_path, monkeypatch):
    # Agent mode's whole shape is that the main thread is blocked inside
    # `waddle_sdk.agent()` while the robot's own housekeeping runs elsewhere:
    # the arm keeps integrating the agent's commands and the camera keeps
    # feeding the stills the agent perceives through. Every other test in
    # this file drives the example's loop from the main thread, so a
    # background loop that never runs would be invisible here — and it is
    # the SDK's loop now (`waddle_sdk.robots.base.RobotPump`, ticked by the
    # example's own `robot_tick`), which is exactly the seam a migration
    # can get wrong.
    example = _load_example()
    arm = example.ToyArm()
    # HOLD_FIRST: the handoff holds before the claimant drives, so the hold
    # verb firing IS the agent's claim landing — which is also the proof
    # that the invited episode is live and can be ended below.
    engaged = threading.Event()

    def hold() -> None:
        engaged.set()
        arm.hold()

    session = waddle_sdk.init(
        "py-toy-agent",
        example.robot_description(),
        waddle_sdk.Control(send=lambda chunk: None, hold=hold, estop=arm.estop),
        recording_dir=tmp_path,
        _testing=True,
    )

    # The invite line is the example's own announcement that the warm-up
    # rollout is over: agent mode runs one first, and an "agent" claim
    # engaged while THAT episode is live lands on the warm-up instead — so
    # the claim below has to wait for this, not race it.
    invited = threading.Event()
    printed = example.status

    def status(message: str) -> None:
        if message.startswith("agent invite "):
            invited.set()
        printed(message)

    monkeypatch.setattr(example, "status", status)
    args = example.parse_args(
        [
            "--mode", "agent",
            "--prompt", "stack the cups",
            "--episode-seconds", "0.2",
            "--agent-timeout", "60",
        ]
    )

    box: dict = {}

    def run() -> None:
        try:
            box["code"] = example.run_agent_mode(session, arm, args)
        except BaseException as exc:  # re-raised on the main thread below
            box["error"] = exc

    caller = threading.Thread(target=run, name="pytest-toy-agent", daemon=True)
    caller.start()
    try:
        _until(invited.is_set, "the example never got as far as its agent invite")
        _until(
            engaged.is_set,
            "the agent's claim never engaged",
            tick=lambda: waddle_sdk._testing.engage(session, "agent-claim", "agent"),
        )
        # The caller is inside `waddle_sdk.agent()` from here on, so nothing it
        # does can publish a frame: every frame after this one came from the
        # background loop.
        published = len(waddle_sdk._testing.frames(session, example.CAMERA_NAME))
        _until(
            lambda: len(waddle_sdk._testing.frames(session, example.CAMERA_NAME)) > published,
            "the robot stopped publishing while the caller was blocked in agent()",
        )
        # What ends an agent run: the outcome arrives from outside the
        # customer's loop, and the blocked caller holds no episode handle.
        waddle_sdk._testing.mark_done(session, "success", "the agent is done")
    finally:
        caller.join(timeout=20.0)

    assert not caller.is_alive(), "waddle_sdk.agent() never returned"
    assert "error" not in box, f"the example's agent mode raised: {box.get('error')!r}"
    assert box["code"] == 0


def test_the_examples_estop_survives_the_scene_reset():
    # `pre_reset` runs the scene reset before EVERY episode. A latched
    # e-stop is the owner's envelope; if the reset cleared it, every e-stop
    # Waddle ever asked for would be undone by the supervision flow itself,
    # with no human in the loop.
    example = _load_example()
    arm = example.ToyArm()
    target = np.full(len(example._CHAIN), 0.3)

    arm.command(target)
    arm.step(0.05)
    assert np.any(arm.joint_positions() != 0.0), "the arm never moved to begin with"

    arm.estop()
    frozen = arm.joint_positions()
    arm.command(-target)
    arm.step(0.05)
    assert np.array_equal(arm.joint_positions(), frozen), "a latched arm moved"

    # The reset declines rather than pretending — which is what makes the
    # example's `pre_reset` return False and keep the episode out of
    # RESETTING.
    assert arm.home() is False
    assert np.array_equal(arm.joint_positions(), frozen), "the reset moved a latched arm"
    arm.command(target)
    arm.step(0.05)
    assert np.array_equal(arm.joint_positions(), frozen), "the reset cleared the latch"

    # Only the human at the machine clears it.
    arm.clear_estop()
    assert arm.home() is True
    arm.command(target)
    arm.step(0.05)
    assert np.any(arm.joint_positions() != 0.0), "the arm never recovered"


def test_empty_environment_variables_mean_unset(monkeypatch):
    # `VAR=${MAYBE_UNSET}` is how a harness parameterizes a child, and this
    # program's job is to be driven by one. Empty has to mean unset, not a
    # traceback out of `int("")` or an empty credential the SDK refuses.
    example = _load_example()
    for name in (
        "WADDLE_TOY_MODE",
        "WADDLE_TOY_TRANSPORT",
        "WADDLE_TOY_TOKEN",
        "WADDLE_TOY_MEDIA",
        "WADDLE_TOY_MEDIA_TOKEN",
        "WADDLE_TOY_PROMPT",
        "WADDLE_TOY_EPISODES",
        "WADDLE_TOY_EPISODE_SECONDS",
        "WADDLE_TOY_AGENT_TIMEOUT",
        "WADDLE_TOY_RECORDING_DIR",
    ):
        monkeypatch.setenv(name, "")

    args = example.parse_args([])
    assert args.mode == "loop"
    assert (args.transport, args.token, args.media, args.media_token) == (None, None, None, None)
    assert args.episodes == 0
    assert args.episode_seconds == 4.0
    assert args.agent_timeout == 120.0
    assert args.recording_dir == "toy-recordings"
    assert args.prompt

    # Same rule for an explicitly empty flag, and the promise it keeps: a
    # plane that asks for no credential is configured by leaving the token
    # empty (examples/README.md), so that has to build a real transport.
    args = example.parse_args(["--transport", "http://127.0.0.1:9", "--token", ""])
    assert args.token is None
    waddle_sdk.Grpc(args.transport, args.token)


def test_the_examples_success_criterion_is_not_free():
    # The example decides its own outcome, so the criterion has to be able
    # to say "no". An episode in which nothing ever dispatched leaves the
    # arm at home while the script has moved on — that must read as a
    # failure, not as the success a "did the arm reach its own last target"
    # test would happily report.
    example = _load_example()
    frozen = example.ToyArm()
    for tick in range(200):
        scripted, _gripper = example.scripted_policy(tick)
        assert frozen.error_against(scripted) > 0.15, f"a frozen arm passed at tick {tick}"


def test_the_examples_forward_kinematics_reports_a_real_orientation():
    # The example reports `ee_pose` from its own FK, and the wire takes the
    # orientation as a unit wxyz quaternion. A non-orthonormal chain or a
    # transposed quaternion would both be silent on the wire and wrong in
    # every downstream corpus.
    example = _load_example()
    rng = np.random.default_rng(20260731)
    for _ in range(16):
        q = rng.uniform(-1.5, 1.5, size=len(example._CHAIN))
        position, rotation = example.forward_kinematics(q)
        assert position.shape == (3,)
        assert np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-9)
        quat = np.array(example.quaternion_wxyz(rotation))
        assert quat.shape == (4,)
        assert np.isclose(np.linalg.norm(quat), 1.0, atol=1e-9)
    # Home is the identity orientation, so w is 1 and xyz are 0 — the
    # ordering itself, not just the norm.
    _, home_rotation = example.forward_kinematics(np.zeros(len(example._CHAIN)))
    assert np.allclose(example.quaternion_wxyz(home_rotation), (1.0, 0.0, 0.0, 0.0))
