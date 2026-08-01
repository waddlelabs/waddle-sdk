"""The published example (`examples/toy_robot.py`) has to keep working.

It is the program we point customers at and the robot half of the
cross-repo end-to-end proof, so it is tested the way it is run: as a real
subprocess, offline, with no transport in the environment.

The subprocess test cannot see everything, though. Offline, `publish_frame`
is a documented no-op that returns before it validates anything — so a
camera declaration that disagrees with the rendered frame, or an uplink
policy core would reject, stays invisible until the day someone points the
example at a real plane. The second test wires the in-process loopback
media plane and pushes one real rendered frame through, which is where
that disagreement surfaces.
"""

import importlib.util
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pytest
from mcap.reader import make_reader

import waddle
import waddle._testing

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
    waddle.shutdown()


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

    session = waddle.init(
        "py-toy-example",
        example.robot_description(),
        waddle.Control(send=lambda chunk: None, hold=arm.hold, estop=arm.estop),
        _testing=True,
    )

    frame = arm.render()
    assert frame.dtype == np.uint8
    assert frame.shape == (example.CAMERA_H, example.CAMERA_W, 3)
    session.publish_frame(example.CAMERA_NAME, frame)
    # The 7-value (xyz + wxyz) pose the example reports every tick.
    session.report_proprio(
        joint_vel=arm.joint_velocities(),
        ee_pose=arm.ee_pose(),
        ee_pose_frame=example._TOOL_FRAME,
        gripper=arm.gripper(),
    )

    deadline = time.monotonic() + 5.0
    got: list[bytes] = []
    while time.monotonic() < deadline:
        got = waddle._testing.frames(session, example.CAMERA_NAME)
        if got:
            break
        time.sleep(0.005)
    assert got, "the declared camera never accepted the example's own frame"
    assert got[-1] == frame.tobytes()


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
