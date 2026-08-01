"""`waddle.agent(prompt)` — "Waddle, drive this one" (flag
`waddle.v0.agent`), one layer up from `test_agent.py`'s `_core` tests, the
way `test_reset_api.py` sits above `test_reset.py`.

The centrepiece is one whole agent-driven episode over the `_testing`
loopback rig: the invited agent claims through the ordinary intervention
machinery, its stream reaches the customer's OWN registered `send` while
the caller sits blocked in `waddle.agent()`, and a plane-shaped MARK_DONE
ends the run. Everything asserted here is core's decision arriving intact
in Python — there is no agent logic in this package to test.
"""

from __future__ import annotations

import threading
import time

import pytest

import waddle
import waddle._native
import waddle._testing


@pytest.fixture(autouse=True)
def _clean_session():
    yield
    waddle.shutdown()


def _robot(n_joints: int = 6) -> waddle.Robot:
    return waddle.Robot(
        name="pytest-agent-api-bot",
        robot_id="py-agent-api-01",
        cell_id="cell-py-agent-api",
        action_space=waddle.JointSpace(
            joints=[f"j{i}" for i in range(n_joints)], rate_hz=50
        ),
    )


# --- Refusals before anything is marshalled --------------------------------


def test_agent_without_a_session_raises():
    with pytest.raises(RuntimeError, match="waddle.init"):
        waddle.agent("stack the cups")


def test_agent_without_a_declared_plane_refuses():
    """A session that declared no plane has nobody to ask: the invite could
    only ever run out to its deadline. Say so at the call, naming the fix,
    rather than blocking for ten minutes and returning an abort."""
    waddle.init(
        "py-agent-api-offline",
        _robot(),
        waddle.Control(send=lambda chunk: None, hold=lambda: None),
    )
    with pytest.raises(RuntimeError, match="transport=waddle.Grpc"):
        waddle.agent("stack the cups")


@pytest.mark.skipif(
    "grpc" not in waddle._native.FEATURES,
    reason="needs a declared plane, and this build has no control transport",
)
def test_agent_without_a_way_to_actuate_surfaces_cores_refusal():
    """Core's own precondition, surfaced verbatim: an invite is a live
    engage path, so a session that registered no control at all would
    accept the ask and then stall undiagnosably at the agent's first
    engage. (A plane that never negotiated `waddle.v0.agent` is a different
    story — it just never answers, and the deadline returns an ordinary
    abort.) The declaration-only session below is the one shape that gets
    this far: wire any of send/hold/media and `init` itself has opinions."""
    waddle.init(
        "py-agent-api-no-verbs",
        _robot(),
        waddle.Control(),
        transport=waddle.Grpc("http://127.0.0.1:9"),
    )
    with pytest.raises(RuntimeError, match="requires a registered") as excinfo:
        waddle.agent("stack the cups", timeout_s=1.0)
    assert "an agent invite is a live engage path" in str(excinfo.value)


def test_agent_validates_its_own_arguments():
    with pytest.raises(ValueError, match="non-empty prompt"):
        waddle.agent("")
    with pytest.raises(ValueError, match="positive number of seconds"):
        waddle.agent("stack the cups", timeout_s=0)
    with pytest.raises(TypeError, match="positive number of seconds"):
        waddle.agent("stack the cups", timeout_s="600")


# --- Marshalling: core's words, verbatim -----------------------------------


class _StubCoreResult:
    outcome = "success"
    episode_id = "ep-stub"
    recording_ref = "waddle://recordings/stub"
    detail = "cups stacked"


class _StubCoreSession:
    """Stands in for `_core.Session` so the marshalling is observable on its
    own: what crosses into core, and that every field of core's answer
    reaches the caller unchanged."""

    def __init__(self) -> None:
        self.calls: list[tuple[str, int]] = []

    def agent(self, prompt: str, timeout_ns: int) -> _StubCoreResult:
        self.calls.append((prompt, timeout_ns))
        return _StubCoreResult()


def test_agent_marshals_the_prompt_in_and_every_field_out(monkeypatch):
    stub = _StubCoreSession()
    monkeypatch.setattr(waddle, "_session", stub)
    monkeypatch.setattr(waddle, "_session_has_plane", True)

    result = waddle.agent("stack the cups", timeout_s=1.5)

    assert stub.calls == [("stack the cups", 1_500_000_000)]
    assert isinstance(result, waddle.AgentResult)
    assert (result.outcome, result.episode_id) == ("success", "ep-stub")
    assert result.recording_ref == "waddle://recordings/stub"
    assert result.detail == "cups stacked"


# --- One whole agent-driven episode over the loopback ----------------------


def test_agent_drives_a_whole_episode_over_the_loopback(tmp_path):
    sent: list = []
    engaged = threading.Event()

    def hold() -> None:
        # HOLD_FIRST: the handoff holds before the claimant drives, so this
        # firing IS the engage landing — the one thing this test needs to
        # observe without reaching into core's mirror.
        engaged.set()

    control = waddle.Control(send=sent.append, hold=hold, resume=lambda: None)
    session = waddle.init(
        "py-agent-api-loopback",
        _robot(),
        control,
        recording_dir=tmp_path,
        _testing=True,
    )

    box: dict = {}

    def run() -> None:
        try:
            box["result"] = waddle.agent("stack the cups", timeout_s=30.0)
        except BaseException as exc:  # reported on the main thread below
            box["error"] = exc

    caller = threading.Thread(target=run, name="pytest-agent-caller")
    caller.start()
    try:
        # `run_agent` opens the episode and drives it to RUNNING itself (it
        # injects Start — the blocked caller never ticks E6), so the invited
        # agent's claim is admissible almost immediately; C8 admits
        # ACTOR_KIND_AGENT and nothing else on an agent-invited episode.
        time.sleep(0.5)
        waddle._testing.engage(session, "agent-claim-1", "agent")
        assert engaged.wait(timeout=5.0), "the agent's claim never engaged"

        # The caller is blocked in `agent()` and never ticks, so the
        # claimed-while-stalled BYPASS pump is what carries the agent's
        # stream to the customer's own registered `send`.
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            waddle._testing.push_teleop(session, [0.7, 0.0, 0.0])
            if any(chunk.provenance == "agent" for chunk in sent):
                break
            time.sleep(0.01)
        assert any(chunk.provenance == "agent" for chunk in sent), (
            "the agent's actions never reached the registered `send`"
        )

        # What a plane `EpisodeDirective{MARK_DONE}` does: the outcome is
        # decided from outside the customer's loop, and THAT is what
        # unblocks `waddle.agent()`.
        waddle._testing.mark_done(session, "success", "agent task complete")
    finally:
        caller.join(timeout=15.0)

    assert not caller.is_alive(), "waddle.agent() never returned"
    assert "error" not in box, f"the agent run raised: {box.get('error')!r}"
    result = box["result"]
    assert isinstance(result, waddle.AgentResult)
    assert result.outcome == "success"
    assert result.episode_id.startswith("ep-")
    # No plane here, so no `AgentTaskUpdate` ever arrived: the two fields
    # that only a plane can fill stay empty rather than being invented.
    assert result.recording_ref is None
    assert result.detail == ""

    waddle.shutdown()
    assert (tmp_path / f"{result.episode_id}.sidecar.json").exists(), (
        "the agent-driven episode must be recorded like any other"
    )


def test_agent_invite_deadline_comes_back_as_an_outcome():
    """Nobody can ever engage over a loopback with no script, so the invite
    deadline (FSM.md E25) closes the episode — and that is a RESULT, not an
    exception: "the ask went unanswered" is an answer."""
    waddle.init(
        "py-agent-api-timeout",
        _robot(),
        waddle.Control(send=lambda chunk: None, hold=lambda: None),
        _testing=True,
    )
    started = time.monotonic()
    result = waddle.agent("nobody home", timeout_s=0.15)
    elapsed = time.monotonic() - started

    assert result.outcome == "abort"
    assert result.episode_id.startswith("ep-")
    assert result.recording_ref is None
    assert result.detail == ""
    assert elapsed < 5.0, "the invite deadline, not a hang"
