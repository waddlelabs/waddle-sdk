"""End-to-end: the tutorial loop against the real core over Local
recording — the Python-side proof that obs logging (Part A) and gate-record
persistence (Part B) work through the whole stack — plus the intervention
path over the loopback media plane."""

import json
import time

import numpy as np
import pytest
from mcap.reader import make_reader
from mcap_protobuf.decoder import DecoderFactory

import waddle_sdk
import waddle_sdk._testing


@pytest.fixture(autouse=True)
def _clean_session():
    yield
    waddle_sdk.shutdown()


def _robot(n_joints: int = 3) -> waddle_sdk.Robot:
    return waddle_sdk.Robot(
        name="pytest-bot",
        robot_id="py-01",
        cell_id="cell-py",
        action_space=waddle_sdk.JointSpace(
            joints=[f"j{i}" for i in range(n_joints)], rate_hz=50
        ),
    )


def _control(log: list | None = None) -> waddle_sdk.Control:
    def send(chunk):
        if log is not None:
            log.append(chunk)

    return waddle_sdk.Control(send=send, hold=lambda: None, resume=lambda: None)


def _topic_counts(mcap_path):
    with open(mcap_path, "rb") as f:
        reader = make_reader(f)
        counts: dict[str, int] = {}
        for _, channel, _ in reader.iter_messages():
            counts[channel.topic] = counts.get(channel.topic, 0) + 1
    return counts


def _decoded_observations(mcap_path):
    """Every `/waddle/observations` message, decoded via the channel's own
    embedded `FileDescriptorSet` schema (`mcap-protobuf-support`) — the same
    schema-driven decode any external MCAP reader would use, per
    waddle-sidecar's mcaprec.rs doc ("any MCAP reader can decode the
    messages without this repo checked out")."""
    with open(mcap_path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        return [
            msg
            for _, channel, _, msg in reader.iter_decoded_messages()
            if channel.topic == "/waddle/observations"
        ]


def test_nominal_episode(tmp_path):
    session = waddle_sdk.init("py-e2e", _robot(), _control(), recording_dir=tmp_path)

    with waddle_sdk.rollout(task="stack the blocks") as ep:
        episode_id = ep.id
        action = np.array([0.1, 0.2, 0.3])
        obs = np.array([0.9, 0.8, 0.7])
        out = ep.gate(action, obs)
        assert out is action  # Pass returns the caller's exact object

        # report_proprio: a richer sample than the bare joint_pos every
        # gate(obs=...) call already records, merged into every subsequent
        # tick's recorded ProprioSample.
        session.report_proprio(
            joint_vel=[0.01, 0.02, 0.03],
            ee_pose=np.array([1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0]),
            ee_pose_frame="ee",
            gripper=0.5,
        )
        time.sleep(0.1)  # settle past the reducer's <=20ms drain cadence

        for _ in range(49):
            out = ep.gate(action, obs)
            assert out is action  # Pass returns the caller's exact object
        assert ep.last_gate.kind == "pass"
        assert ep.last_gate.provenance == "policy"
        ep.terminate("success", "test done")
    assert ep.done
    assert ep.outcome == waddle_sdk.Outcome.SUCCESS
    waddle_sdk.shutdown()

    sidecar_path = tmp_path / f"{episode_id}.sidecar.json"
    sidecar = json.loads(sidecar_path.read_text())
    assert sidecar["outcome"] == "TERMINAL_OUTCOME_SUCCESS"
    assert sidecar["task"] == "stack the blocks"
    assert (tmp_path / "manifest.jsonl").exists()

    # The end-to-end proof of obs logging + persistence through Python.
    mcap_path = tmp_path / f"{episode_id}.mcap"
    counts = _topic_counts(mcap_path)
    assert counts.get("/waddle/actions", 0) >= 50
    assert counts.get("/waddle/observations", 0) >= 50

    # report_proprio's merge, read back through the actual decoded MCAP
    # content (not just topic counts): the LAST recorded observation must
    # carry both the gate's own joint_pos and the reported extras.
    observations = _decoded_observations(mcap_path)
    last = observations[-1].proprio
    assert list(last.joint_pos) == list(obs)
    assert list(last.joint_vel) == [0.01, 0.02, 0.03]
    assert last.gripper == pytest.approx(0.5)
    assert last.ee_pose.frame_id == "ee"
    assert (last.ee_pose.position.x, last.ee_pose.position.y, last.ee_pose.position.z) == (
        1.0,
        2.0,
        3.0,
    )
    assert (
        last.ee_pose.rotation.w,
        last.ee_pose.rotation.x,
        last.ee_pose.rotation.y,
        last.ee_pose.rotation.z,
    ) == (1.0, 0.0, 0.0, 0.0)


def test_intervention(tmp_path):
    # A 6-joint action space: the open runtime carries the RAW teleop stream
    # (a twist flattens to exactly 6 values — linear xyz + angular xyz,
    # `pumps::flatten_packet`) and media intake's dims validation
    # drops anything that doesn't match the declared action space's width —
    # retargeting into whatever the real robot's space is is the closed
    # side's job. A 3-joint robot here would have every pushed packet
    # rejected at intake before it ever reached the gate.
    session = waddle_sdk.init(
        "py-intervention", _robot(n_joints=6), _control(), recording_dir=tmp_path, _testing=True
    )

    with waddle_sdk.rollout(task="towel") as ep:
        a = np.zeros(6)
        for _ in range(5):
            assert ep.gate(a, a) is a

        waddle_sdk._testing.engage(session, "claim-1", "teleop")

        # Keep ticking; once the claim engages and the stream's playout
        # delay elapses, the gate substitutes the teleop action.
        deadline = time.monotonic() + 5.0
        substituted = None
        while time.monotonic() < deadline:
            waddle_sdk._testing.push_teleop(session, [0.7, 0.0, 0.0])
            out = ep.gate(a, a)
            if out is not None and out is not a:
                substituted = out
                break
            time.sleep(0.005)
        assert substituted is not None, "teleop stream never substituted"
        assert isinstance(substituted, np.ndarray)
        assert substituted.dtype == np.float64
        assert substituted.shape == (6,)
        assert ep.last_gate.kind in ("substitute", "blend")
        assert ep.last_gate.provenance == "teleop"
        # A whole-robot command names no part: `part` is how a caller tells
        # this array apart from one addressing a single declared part.
        assert ep.last_gate.part is None

        waddle_sdk._testing.release(session, "claim-1")
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if ep.gate(a, a) is a:
                break
            time.sleep(0.005)
        else:
            pytest.fail("passthrough never resumed after release")

        episode_id = ep.id
        ep.terminate("success")
    waddle_sdk.shutdown()

    sidecar = json.loads((tmp_path / f"{episode_id}.sidecar.json").read_text())
    assert sidecar["claims"], "claim span recorded"
    assert sidecar["interventions"], "intervention span recorded"


def test_rollout_exit_aborts(tmp_path):
    waddle_sdk.init("py-abort", _robot(), _control(), recording_dir=tmp_path)

    with waddle_sdk.rollout(task="left early") as ep:
        ep.gate([0.0, 0.0, 0.0])
        episode_id = ep.id
    assert ep.done
    assert ep.outcome == waddle_sdk.Outcome.ABORT
    waddle_sdk.shutdown()

    sidecar = json.loads((tmp_path / f"{episode_id}.sidecar.json").read_text())
    assert sidecar["outcome"] == "TERMINAL_OUTCOME_ABORT"

    # An exception is never swallowed — and still aborts the episode.
    waddle_sdk.init("py-abort-2", _robot(), _control(), recording_dir=tmp_path)
    with pytest.raises(RuntimeError, match="boom"):
        with waddle_sdk.rollout(task="raises") as ep:
            raise RuntimeError("boom")
    assert ep.outcome == waddle_sdk.Outcome.ABORT


def test_gate_accepts_lists(tmp_path):
    waddle_sdk.init("py-lists", _robot(), _control())

    with waddle_sdk.rollout(task="lists") as ep:
        action = [0.1, 0.2, 0.3]
        out = ep.gate(action, [0.4, 0.5, 0.6], gripper=0.5)
        assert out is action  # identity-preserved even for lists
        ep.terminate("success")


def test_report_proprio_validates_ee_pose_shape(tmp_path):
    session = waddle_sdk.init("py-proprio-shape", _robot(), _control())

    with waddle_sdk.rollout(task="task"):
        # numpy or list accepted for both joint_vel and ee_pose.
        session.report_proprio(joint_vel=[0.1, 0.2, 0.3])
        session.report_proprio(joint_vel=np.array([0.1, 0.2, 0.3]))
        session.report_proprio(gripper=0.3)  # every field is optional
        with pytest.raises(ValueError, match="exactly 7 values"):
            session.report_proprio(ee_pose=[1.0, 2.0, 3.0])
        with pytest.raises(ValueError, match="exactly 7 values"):
            session.report_proprio(ee_pose=np.zeros(6))


def test_init_twice_raises(tmp_path):
    waddle_sdk.init("py-twice", _robot(), _control())
    with pytest.raises(RuntimeError, match="shutdown"):
        waddle_sdk.init("py-twice-2", _robot(), _control())


def test_missing_hold_verb_under_hold_first_raises_actionable_error(tmp_path):
    # No `hold`: the build-time verb-registration check must
    # surface as a clear, actionable Python exception naming both the
    # missing verb and the fix — not a 10s engage timeout with nothing to
    # diagnose it.
    control = waddle_sdk.Control(send=lambda chunk: None)
    with pytest.raises(RuntimeError) as exc_info:
        waddle_sdk.init("py-missing-hold", _robot(), control, recording_dir=tmp_path)
    message = str(exc_info.value)
    assert "hold" in message
    assert "HOLD_FIRST" in message
    assert "choose a different handoff policy" in message


def test_a_recording_directory_that_does_not_exist_yet_is_created(tmp_path):
    """A program that names a recording directory gets one. Every file the
    recorder writes — sidecar, MCAP, manifest — lives INSIDE that directory,
    so a missing one used to take the whole archive with it while the session
    opened clean, ran, and looked no different."""
    recordings = tmp_path / "recordings" / "run-1"
    assert not recordings.exists()

    waddle_sdk.init("py-missing-dir", _robot(), _control(), recording_dir=recordings)
    with waddle_sdk.rollout(task="keep this one") as ep:
        episode_id = ep.id
        ep.gate([0.1, 0.2, 0.3], [0.9, 0.8, 0.7])
        ep.terminate("success", "recorded")
    waddle_sdk.shutdown()

    assert (recordings / f"{episode_id}.sidecar.json").exists()
    assert (recordings / f"{episode_id}.mcap").exists()
    assert (recordings / "manifest.jsonl").exists()


def test_a_recording_dir_that_cannot_be_a_directory_refuses(tmp_path):
    """The residue a `mkdir -p` cannot fix — here a path that is already a
    file — fails `init` by name, rather than opening a session that records
    nothing."""
    occupied = tmp_path / "recordings"
    occupied.write_text("not a directory")

    with pytest.raises(RuntimeError) as exc_info:
        waddle_sdk.init("py-bad-dir", _robot(), _control(), recording_dir=occupied)
    message = str(exc_info.value)
    assert "recording_dir" in message
    assert "recordings" in message


def test_nested_rollout_raises(tmp_path):
    waddle_sdk.init("py-nested", _robot(), _control())
    with waddle_sdk.rollout(task="outer") as ep:
        ep.gate([0.0, 0.0, 0.0])
        # One active episode per session: the guard errors instead of
        # destroying the live episode's recording.
        with pytest.raises(RuntimeError, match="already active"):
            waddle_sdk.rollout(task="inner")
        assert not ep.done
        ep.terminate("success")
