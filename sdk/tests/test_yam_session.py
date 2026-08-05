"""The session a rig opens — and the guarantee that opening it by hand gets
you exactly the same one.

`rig.session(...)` is composition sugar over pieces that are each usable
alone: the arms, the verbs, `waddle.init`, the console recovery, the
reporting pump, the hold a finished mission takes on live hardware, and the
shutdown. This file gates three things about it.

* **It finalizes.** The recording is finalized by the context manager's exit,
  not by a `finally:` the customer remembered to write — the footgun test
  below raises inside the body and then reads the recording back.
* **It is sugar, not a wall.** The hand-wired composition — `yam.declaration`,
  drivers, `base.Arm`, `waddle.Control`, a plain `waddle.init`, `RobotPump` —
  opens a session byte-identical to the one `rig.session()` opens: same
  registered robot JSON, same everything else `create_session` is handed. And
  a customer's own `send` callable still REPLACES the shipped envelope when it
  goes through the sugar.
* **It knows what closing costs.** On drivers this layer reads as METAL —
  anything whose `kind` is not `sim` — a finished mission holds, still
  streaming and still holding its pose, until a human says the machine is
  parked. A twin never does, and a Ctrl-C never does (the operator who typed
  it is already standing there).

The park/console tests build their rig out of twins that answer
`kind == "live"`, because that word is the whole of what the closing path
reads off a driver, and it reads it off the object that has the property.
Nothing here opens a bus.
"""

from __future__ import annotations

import json
import threading
import time

import numpy as np
import pytest
from mcap.reader import make_reader
from mcap_protobuf.decoder import DecoderFactory

import waddle
import waddle._testing
from waddle.robots import base, yam

#: The compiled core, captured before any test shims `waddle.core` (the
#: golden below records what `waddle.init` hands it).
_CORE = waddle.core

#: Bounded only so a broken build fails instead of hanging; every wait below
#: ends on an observation, never on the clock.
PATIENCE_S = 20.0

#: How often an intervention chunk may be re-offered. Offering is repeated
#: because the claim engages on the core's own schedule and a chunk offered
#: before it does is correctly dropped — and SPACED because this declaration
#: replans IMMEDIATE, so a new chunk supersedes the still-pending steps of the
#: one before it: offering faster than the playout delay replaces, every pass,
#: the step that was about to play. A property of the offer schedule, not a
#: race — nothing here asserts on WHEN a step lands, only that one does.
OFFER_INTERVAL_S = 0.1


@pytest.fixture(autouse=True)
def _clean_session():
    yield
    waddle.shutdown()


@pytest.fixture(autouse=True)
def _no_console(monkeypatch):
    """No test may take the developer's own terminal.

    `start_console_recovery` starts a reader thread only when stdin is a
    foreground TTY — which it IS under `pytest -s`, where that thread would
    then sit in `for line in sys.stdin` eating keystrokes for the rest of the
    run. The condition is decided here; the one test about the console
    overrides it with a stdin of its own."""
    monkeypatch.setattr(base, "console_is_at_the_machine", lambda: False)


# --------------------------------------------------------------------------
# The reference rig's SITE facts (see test_robots_yam.py — same numbers,
# stated again rather than imported)
# --------------------------------------------------------------------------

WORKSPACE_BOX_M = ((0.05, -0.45, 0.05), (0.60, 0.45, 0.70))
GRIPPER_LIMITS_MOTOR_RAD = (0.1, 1.7)
CROSS_ARM_XYZ = (0.0, -0.60, 0.0)
CROSS_ARM_RPY = (0.0, 0.0, -0.15)

#: 1.0 rad/s and 2.5 units/s at 10 Hz — what both factories derive.
STEP_CAPS = (0.1,) * yam.ARM_JOINT_COUNT + (0.25,)


def _cross_arm() -> base.CrossArm:
    return base.CrossArm(xyz=CROSS_ARM_XYZ, rpy=CROSS_ARM_RPY)


def _bimanual(**overrides) -> base.Rig:
    kwargs = dict(
        workspace=WORKSPACE_BOX_M,
        gripper_limits=GRIPPER_LIMITS_MOTOR_RAD,
        cross_arm=_cross_arm(),
        sim=True,
    )
    kwargs.update(overrides)
    return yam.bimanual(**kwargs)


# --------------------------------------------------------------------------
# Twins that answer "live", and the bookkeeping the tests observe
# --------------------------------------------------------------------------


class _LiveTwin(base.SimDriver):
    """A twin that answers ``kind == "live"``.

    Closing a live unit drops all torque, and that is the whole reason a
    finished mission holds before it closes. The question is asked of the
    DRIVER, so a driver that answers it differently is all a test needs — no
    bus, no vendor package."""

    kind = "live"

    def __init__(self, part: str, closed: list[str], **kwargs) -> None:
        super().__init__(**kwargs)
        self._part = part
        self._closed = closed

    def close(self) -> None:
        self._closed.append(f"closed:{self._part}")


def _live_rig(
    closed: list[str],
    *,
    posture: str = "supervised",
    estopped: bool = False,
    report=base.status,
) -> base.Rig:
    """A YAM-shaped rig whose drivers answer ``live`` — declaration and
    envelope from the module, drivers from this file."""

    def build() -> dict[str, base.Arm]:
        arms: dict[str, base.Arm] = {}
        for part, frame, home in (
            (yam.LEFT_PART, yam.LEFT_BASE_FRAME, yam.DEFAULT_SIM_HOME[0]),
            (yam.RIGHT_PART, yam.RIGHT_BASE_FRAME, yam.DEFAULT_SIM_HOME[1]),
        ):
            driver = _LiveTwin(
                part,
                closed,
                home=home,
                lower=[lo for lo, _ in yam.JOINT_LIMITS],
                upper=[hi for _, hi in yam.JOINT_LIMITS],
                step_caps=STEP_CAPS,
                rate_hz=yam.DEFAULT_RATE_HZ,
            )
            if estopped:
                driver.estop()
            arms[part] = base.Arm(
                part=part,
                driver=driver,
                joint_names=yam.JOINT_NAMES,
                joint_limits=yam.JOINT_LIMITS,
                step_caps=STEP_CAPS,
                base_frame=frame,
                arm_dof=yam.ARM_JOINT_COUNT,
                rate_hz=yam.DEFAULT_RATE_HZ,
                report=report,
            )
        return arms

    return base.Rig(
        declaration=yam.declaration(parts=(yam.LEFT_PART, yam.RIGHT_PART)),
        build_arms=build,
        rate_hz=yam.DEFAULT_RATE_HZ,
        posture=posture,
        report=report,
    )


def _count_ticks(arms) -> dict[str, list[int]]:
    """Count what the pump does to each part. The pump calls
    ``arm.step(dt)`` -> ``driver.step(dt)`` once per period per part, so
    wrapping the driver's own method is the observable "it is running"."""
    counts: dict[str, list[int]] = {}
    for part, arm in arms.items():
        counts[part] = [0]

        def wrapper(dt, _inner=arm.driver.step, _n=counts[part]) -> None:
            _n[0] += 1
            _inner(dt)

        arm.driver.step = wrapper
    return counts


def _watch(arm, name: str, seen) -> None:
    """Wrap one of a driver's verbs so a test can see it fire."""
    inner = getattr(arm.driver, name)

    def wrapper(*args, **kwargs):
        seen()
        return inner(*args, **kwargs)

    setattr(arm.driver, name, wrapper)


class _Offer:
    """One intervention chunk, offered until the core takes it (see
    :data:`OFFER_INTERVAL_S`)."""

    def __init__(self, session, values, part=None) -> None:
        self._push = (session, [float(v) for v in values], part)
        self._next = 0.0

    def __call__(self) -> None:
        now = time.monotonic()
        if now >= self._next:
            session, values, part = self._push
            waddle._testing.push_chunk(session, values, part=part)
            self._next = now + OFFER_INTERVAL_S


def _until(predicate, what: str, timeout: float = PATIENCE_S, tick=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        got = predicate()
        if got:
            return got
        if tick is not None:
            tick()
        time.sleep(0.005)
    pytest.fail(what)


def _observations(mcap_path):
    with open(mcap_path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        return [
            msg
            for _, channel, _, msg in reader.iter_decoded_messages()
            if channel.topic == "/waddle/observations"
        ]


def _reported_parts(recording_dir, episode_id: str) -> set[str]:
    samples = [o.proprio for o in _observations(recording_dir / f"{episode_id}.mcap")]
    return {s.part for s in samples if len(s.joint_pos) == yam.JOINT_COUNT}


class _ParkWatchdog:
    """Insurance, never an assertion: releases the park hold if one ever
    begins, so a test that expects NO hold fails on its assertion instead of
    hanging forever."""

    def __init__(self, park: base.ParkGate) -> None:
        self._park = park
        self._done = threading.Event()
        self.held = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._done.is_set():
            if self._park.wait_holding(timeout=0.05):
                self.held.set()
                self._park.confirm()
                return

    def stop(self) -> None:
        self._done.set()
        self._thread.join(timeout=PATIENCE_S)


# --------------------------------------------------------------------------
# What the session is while it is open
# --------------------------------------------------------------------------


def test_the_session_opens_the_arms_and_keeps_every_part_reporting(tmp_path):
    """The pump is ALWAYS on inside a session, not only under `agent()`: a
    program's own loop then only gates and applies, and there is no
    interleaved robot tick left to forget."""
    rig = _bimanual()
    with rig.session("yam-session", recording_dir=tmp_path) as s:
        assert set(s.arms) == {"left_arm", "right_arm"}
        assert s.robot is rig.robot()
        assert s.core is not None
        assert (s.accepted, s.rejected) == (0, 0)
        ticks = _count_ticks(s.arms)
        with waddle.rollout(task="hold still while the parts report") as ep:
            episode_id = ep.id
            _until(
                lambda: all(n[0] >= 2 for n in ticks.values()),
                "the session's pump never reported every part",
            )
            ep.terminate("success")

    assert _reported_parts(tmp_path, episode_id) == {"left_arm", "right_arm"}


def test_the_program_only_gates_and_applies(tmp_path):
    rig = _bimanual()
    with rig.session("yam-drive", recording_dir=tmp_path) as s:
        with waddle.rollout(task="one commanded step") as ep:
            position = np.concatenate(
                [s.arms[p].state()[0] for p in ("left_arm", "right_arm")]
            )
            decided = ep.gate(position + 0.01, position)
            assert decided is not None
            base.apply_decision(s.arms, decided)
            ep.terminate("success")
        assert (s.accepted, s.rejected) == (2, 0)


# --------------------------------------------------------------------------
# The footgun: finalization is the context manager's, not the customer's
# --------------------------------------------------------------------------


def test_an_exception_inside_the_body_still_finalizes_the_recording(tmp_path):
    """THE footgun this runner exists to retire. A program that raises
    mid-mission used to leave the recorder unflushed and the arms energized
    unless it had remembered a `finally:`; here the exit does both, and the
    exception is still the news."""
    rig = _bimanual()
    closed: list[str] = []
    episode: list[str] = []

    with pytest.raises(RuntimeError, match="the policy blew up"):
        with rig.session("yam-footgun", recording_dir=tmp_path) as s:
            for part, arm in s.arms.items():
                _watch(arm, "close", lambda part=part: closed.append(part))
            with waddle.rollout(task="raise in the middle of a rollout") as ep:
                episode.append(ep.id)
                raise RuntimeError("the policy blew up")

    episode_id = episode[0]
    sidecar = json.loads((tmp_path / f"{episode_id}.sidecar.json").read_text())
    assert sidecar["task"] == "raise in the middle of a rollout"
    # The run is recorded as what it was. A rollout that died mid-episode is
    # never a success, and the recording says so rather than saying nothing.
    assert sidecar["outcome"] == "TERMINAL_OUTCOME_ABORT"
    assert (tmp_path / f"{episode_id}.mcap").exists()
    # Readable, i.e. the recorder was flushed and closed rather than left
    # holding a half-written file.
    _observations(tmp_path / f"{episode_id}.mcap")
    assert sorted(closed) == ["left_arm", "right_arm"]
    assert waddle._session is None, "the session was left open by an unwinding exit"


def test_the_recording_directory_a_program_names_is_the_one_it_gets(tmp_path):
    """The shipped five-line program passes `recording_dir="recordings"` and
    makes no directory: nothing in a program that short can. It gets one —
    created where it asked for it — and the episode lands in it. Anything
    less is the worst shape of failure this layer has: a session that opens,
    reports, drives an episode, and leaves no archive at all."""
    recordings = tmp_path / "recordings"
    assert not recordings.exists()

    rig = _bimanual()
    with rig.session("yam-recordings", recording_dir=recordings):
        with waddle.rollout(task="land in a directory nobody made") as ep:
            episode_id = ep.id
            ep.terminate("success")

    assert (recordings / f"{episode_id}.mcap").exists()
    assert (recordings / "manifest.jsonl").exists()
    sidecar = json.loads((recordings / f"{episode_id}.sidecar.json").read_text())
    assert sidecar["task"] == "land in a directory nobody made"
    # Readable, i.e. a finalized archive rather than a file that merely exists.
    _observations(recordings / f"{episode_id}.mcap")


def test_a_finished_session_lets_the_next_one_open(tmp_path):
    """One session per process is the module's rule; the runner has to leave
    the process able to open another one, or a program that ran two missions
    would need a shutdown the sugar was supposed to own."""
    rig = _bimanual()
    with rig.session("yam-first", recording_dir=tmp_path) as first:
        assert first.core is not None
    with rig.session("yam-second", recording_dir=tmp_path) as second:
        assert second.core is not None
        assert second.arms is not first.arms


# --------------------------------------------------------------------------
# The envelope is still a REPLACEABLE default through the sugar
# --------------------------------------------------------------------------


def test_a_customers_own_send_is_the_whole_envelope(tmp_path):
    """THE AMENDMENT, item 4, through the composition sugar: the session
    passes a customer's `send` straight to the verbs, and the shipped envelope
    is REPLACED — not consulted alongside it. Driven the way a teleoperator
    drives: an engaged claim with the caller's loop stalled, which is the
    bypass path that reaches the registered `send`."""
    chunks: list = []

    def my_send(chunk) -> None:
        chunks.append(chunk)

    rig = _bimanual()
    with rig.session(
        "yam-own-envelope", recording_dir=tmp_path, _testing=True, send=my_send
    ) as s:
        assert s.control.send is my_send
        with waddle.rollout(task="a teleoperator drives") as ep:
            # One tick of the program's own loop drives the episode to
            # RUNNING; after it the loop stalls, which is the BYPASS path — the
            # one that reaches the registered `send` verb.
            ep.gate(np.zeros(2 * yam.JOINT_COUNT))
            waddle._testing.engage(s.core, "claim-envelope", "teleop")
            _until(
                lambda: chunks,
                "the customer's own send never received the intervention",
                tick=_Offer(s.core, np.full(yam.JOINT_COUNT, 0.05), "left_arm"),
            )
            waddle._testing.release(s.core, "claim-envelope")
            ep.terminate("success")

        assert (s.accepted, s.rejected) == (0, 0), (
            "the customer's callable IS the envelope — the shipped default is "
            "replaced, not consulted alongside it"
        )


# --------------------------------------------------------------------------
# What closing costs: the hold a finished mission takes on live hardware
# --------------------------------------------------------------------------


def test_a_finished_mission_holds_live_arms_until_a_human_says_they_are_parked(
    tmp_path,
):
    """Returning from the body goes on to close the drivers, which on metal
    stops the vendor's command re-send and drops all torque — from wherever
    the mission left the arms. So the program does not decide that moment: the
    site operator does, with the same console gesture that clears an e-stop.

    The ordering is the assertion: nothing closed before the gesture."""
    order: list[str] = []
    lines: list[str] = []
    rig = _live_rig(order, report=lines.append)

    def confirm() -> None:
        if s.park.wait_holding(timeout=PATIENCE_S):
            order.append("parked-gesture")
            base.apply_console_gesture("parked", s.arms, s.park, report=lines.append)

    gesture = threading.Thread(target=confirm, name="pytest-site-operator")
    with rig.session("live-park", recording_dir=tmp_path) as s:
        gesture.start()
    gesture.join(timeout=PATIENCE_S)

    assert order == ["parked-gesture", "closed:left_arm", "closed:right_arm"]
    assert any("STILL HOLDING" in line for line in lines)
    assert any("parked" in line and "closing" in line for line in lines)


def test_a_twin_is_never_held_on_the_way_out(tmp_path):
    """A twin has nothing to hold and nothing to sag, and a harness must be
    able to wait for a sim program to exit. Read off the driver, never off the
    flag that built it."""
    lines: list[str] = []
    rig = _bimanual(report=lines.append)
    with rig.session("sim-no-park", recording_dir=tmp_path) as s:
        watchdog = _ParkWatchdog(s.park)
    watchdog.stop()

    assert not watchdog.held.is_set(), "a twin asked to be parked"
    assert not any("STILL HOLDING" in line for line in lines)


def test_an_interrupted_mission_is_not_asked_to_park_itself(tmp_path):
    """Every park warning this layer has is attached to a Ctrl-C the operator
    TYPED. An interrupt is that operator, standing at the machine already —
    holding them there again would be asking a question they have answered."""
    order: list[str] = []
    lines: list[str] = []
    rig = _live_rig(order, report=lines.append)

    with pytest.raises(KeyboardInterrupt):
        with rig.session("live-interrupt", recording_dir=tmp_path) as s:
            watchdog = _ParkWatchdog(s.park)
            raise KeyboardInterrupt
    watchdog.stop()

    assert not watchdog.held.is_set(), "an interrupted mission held for a gesture"
    assert order == ["closed:left_arm", "closed:right_arm"]
    assert waddle._session is None


# --------------------------------------------------------------------------
# The console, and the arms that cannot open
# --------------------------------------------------------------------------


def test_the_console_clears_a_latch_on_a_running_session(terminal, tmp_path):
    """The ONE path that clears an e-stop latch is a word typed at the
    machine, and the session starts that reader itself. The rig here opens
    already latched, which is what a program restarted after an e-stop
    finds."""
    lines: list[str] = []
    rig = _live_rig([], estopped=True, report=lines.append)

    with rig.session("live-latched", recording_dir=tmp_path) as s:
        assert s.console is not None
        # These drivers answer `live`, so the exit holds for a park gesture
        # this test never types.
        watchdog = _ParkWatchdog(s.park)
        terminal.type("resume\n")
        _until(
            lambda: not base.latched_parts(s.arms),
            "the session's console never cleared the latch",
        )
    watchdog.stop()
    assert any("resume part=left_arm" in line for line in lines)


def test_a_finished_session_leaves_no_reader_holding_its_arms(terminal, tmp_path):
    """Sequential sessions in one process are a supported path (see above),
    and stdin is ONE stream. A reader left aimed at a finished session would
    compete with the next one for every word typed at the machine — and the
    word at stake is `resume`, the ONE path that clears an owner's e-stop
    latch. Worse on metal: `resume` on a closed session calls `re_enable` on a
    driver whose bus is already torn down.

    So the session retires its reader on the way out, and the next session
    re-aims that same reader at its own arms."""
    first_rig = _live_rig([], estopped=True)
    with first_rig.session("live-first", recording_dir=tmp_path) as first:
        watchdog = _ParkWatchdog(first.park)
        assert first.console is not None and first.console.listening is True
    watchdog.stop()
    assert first.console.listening is False, (
        "the finished session's reader still holds its arms and its ParkGate"
    )

    second_rig = _live_rig([], estopped=True)
    with second_rig.session("live-second", recording_dir=tmp_path) as second:
        watchdog = _ParkWatchdog(second.park)
        terminal.type("resume\n")
        _until(
            lambda: not base.latched_parts(second.arms),
            "the gesture never reached the session that is running",
        )
    watchdog.stop()
    assert base.latched_parts(first.arms) == ["left_arm", "right_arm"], (
        "the finished session's reader answered the word typed at the machine"
    )


def test_a_session_told_not_to_take_the_terminal_offers_no_gesture(terminal, tmp_path):
    """`console=False` is for a program whose stdin belongs to something else
    — a REPL, a supervising harness, another library reading it — and it means
    no reader is started even though a terminal is right there.

    The trap this pins is the terminal being the wrong question: a foreground
    TTY exists here and nothing is reading it, so the hold must not send a
    site operator to type a word nothing will receive."""
    lines: list[str] = []
    rig = _live_rig([], report=lines.append)

    with rig.session("live-no-console", recording_dir=tmp_path, console=False) as s:
        assert s.console is None
        watchdog = _ParkWatchdog(s.park)
    watchdog.stop()

    assert watchdog.held.is_set(), "live arms were closed without a hold"
    assert any("STILL HOLDING" in line for line in lines)
    assert not any(f"type `{base.PARK_WORD}`" in line for line in lines), (
        "a gesture was offered at a terminal this session is not reading"
    )
    assert any("signalled" in line for line in lines)


def test_a_session_that_cannot_open_closes_the_arms_it_opened(tmp_path):
    """`__enter__` opened the hardware; if the session then refuses to build,
    a context manager whose `__enter__` raises never gets an `__exit__`, so
    the unwind is this one's own. A monitor posture with a media plane is the
    reachable case: waddle-core reads a wired media plane as a live engage
    path and refuses a session that offers no verb to follow it.

    The refusal names the VERB, not the posture, and nothing in
    `waddle.robots` rephrases it — an engage-path rule has exactly one home,
    and the posture's own documentation is where that cost is written down.
    The refusal is still what the caller sees."""
    closed: list[str] = []
    rig = _live_rig(closed, posture="monitor")

    with pytest.raises(RuntimeError, match="`hold` verb"):
        with rig.session("monitor-media", recording_dir=tmp_path, _testing=True):
            pytest.fail("the session opened with no way to actuate")

    assert closed == ["closed:left_arm", "closed:right_arm"]
    assert waddle._session is None


def test_a_monitor_session_records_without_a_plane(tmp_path):
    """The other half of the same rule: watching is undiminished offline —
    the local recorder keeps the full-rate archive and the parts still
    report, with one verb (the owner's stop) on the wire."""
    closed: list[str] = []
    rig = _live_rig(closed, posture="monitor")
    with rig.session("monitor-offline", recording_dir=tmp_path) as s:
        assert s.control.send is None and s.control.hold is None
        assert s.control.estop is not None
        # These drivers answer `live`, so the mission still holds on the way
        # out — a monitor session drops torque on close exactly like any
        # other. Something has to make the gesture.
        watchdog = _ParkWatchdog(s.park)
        ticks = _count_ticks(s.arms)
        with waddle.rollout(task="watch only") as ep:
            episode_id = ep.id
            _until(
                lambda: all(n[0] >= 1 for n in ticks.values()),
                "a monitor session stopped reporting",
            )
            ep.terminate("success")
    watchdog.stop()

    assert watchdog.held.is_set(), "live arms were closed without a gesture"
    assert closed == ["closed:left_arm", "closed:right_arm"]
    assert _reported_parts(tmp_path, episode_id) == {"left_arm", "right_arm"}


# --------------------------------------------------------------------------
# THE AMENDMENT, item 7: the hand-wired composition IS the session
# --------------------------------------------------------------------------


def _shape(kwargs: dict) -> dict:
    """What `create_session` was handed, comparable across two runs: the
    callables are different objects by construction (each composition builds
    its own), so what is compared is WHICH of them are wired."""
    out = {}
    for key, value in kwargs.items():
        if value is None or isinstance(value, (str, int, float, bool)):
            out[key] = value
        elif callable(value):
            out[key] = "<callable>"
        else:
            out[key] = repr(value)
    return out


def _record_sessions(monkeypatch) -> list[dict]:
    """Capture every argument `waddle.init` hands the core."""
    calls: list[dict] = []

    class _Recorder:
        def create_session(self, **kwargs):
            calls.append(dict(kwargs))
            return _CORE.create_session(**kwargs)

    monkeypatch.setattr(waddle, "core", _Recorder())
    return calls


def _one_commanded_step(arms) -> dict[str, tuple[int, int]]:
    """The same script on either composition."""
    with waddle.rollout(task="one commanded step") as ep:
        position = np.concatenate(
            [arms[p].state()[0] for p in ("left_arm", "right_arm")]
        )
        decided = ep.gate(position + 0.01, position)
        assert decided is not None
        base.apply_decision(arms, decided)
        ep.terminate("success")
    return {part: (arm.accepted, arm.rejected) for part, arm in arms.items()}


def test_the_hand_wired_composition_is_the_session_the_rig_opens(
    monkeypatch, tmp_path
):
    """THE AMENDMENT, item 7. Every block is a first-class product and the
    session is sugar over them — so a program that wires `yam.declaration()`,
    its own drivers, `base.Arm`, `waddle.Control`, `waddle.init`, the console
    recovery and a `RobotPump` by hand must get the SAME session, down to the
    bytes of the robot JSON that registers it.

    Written out below the way a customer would write it: this is also the
    reference for a program that wants one piece of the sugar and none of the
    rest."""
    calls = _record_sessions(monkeypatch)

    # --- by hand -----------------------------------------------------------
    robot = yam.declaration(
        parts=(yam.LEFT_PART, yam.RIGHT_PART),
        name="yam-bimanual",
        rate_hz=yam.DEFAULT_RATE_HZ,
        max_joint_speed_rad_s=yam.DEFAULT_MAX_JOINT_SPEED_RAD_S,
        frames=(_cross_arm().transform(yam.LEFT_BASE_FRAME, yam.RIGHT_BASE_FRAME),),
    )
    arms: dict[str, base.Arm] = {}
    for part, frame, home in (
        (yam.LEFT_PART, yam.LEFT_BASE_FRAME, yam.DEFAULT_SIM_HOME[0]),
        (yam.RIGHT_PART, yam.RIGHT_BASE_FRAME, yam.DEFAULT_SIM_HOME[1]),
    ):
        arms[part] = base.Arm(
            part=part,
            driver=base.SimDriver(
                home,
                lower=[lo for lo, _ in yam.JOINT_LIMITS],
                upper=[hi for _, hi in yam.JOINT_LIMITS],
                step_caps=STEP_CAPS,
                rate_hz=yam.DEFAULT_RATE_HZ,
            ),
            joint_names=yam.JOINT_NAMES,
            joint_limits=yam.JOINT_LIMITS,
            step_caps=STEP_CAPS,
            base_frame=frame,
            workspace=WORKSPACE_BOX_M,
            fk=yam.forward_kinematics,
            arm_dof=yam.ARM_JOINT_COUNT,
            home_values=home,
            rate_hz=yam.DEFAULT_RATE_HZ,
        )
    verbs = waddle.Control(
        send=base.chunk_sender(arms),
        hold=lambda: base.hold_all(arms),
        estop=lambda: base.estop_all(arms),
    )
    session = waddle.init(
        "yam-golden",
        robot,
        verbs,
        recording_dir=tmp_path,
        pre_reset=base.scene_reset(arms),
    )
    park = base.ParkGate()
    console = base.start_console_recovery(arms, park)
    pump = base.RobotPump(base.proprio_tick(session, arms), yam.DEFAULT_RATE_HZ)
    pump.start()
    try:
        by_hand = _one_commanded_step(arms)
    finally:
        # The same order the sugar unwinds in, and for the same reasons: the
        # reader goes first because it is the one thing left that could still
        # drive these arms, and closing is last because it is what drops the
        # torque.
        if console is not None:
            console.retire()
        pump.stop()
        waddle.shutdown()
        base.close_all(arms)

    # --- the same thing, as sugar -----------------------------------------
    rig = _bimanual()
    with rig.session("yam-golden", recording_dir=tmp_path) as s:
        as_sugar = _one_commanded_step(s.arms)

    assert len(calls) == 2, "each composition opens exactly one session"
    hand, sugar = calls
    assert hand["robot_json"] == sugar["robot_json"], (
        "the sugar registers a different robot than the hand-wired program "
        "declares"
    )
    assert _shape(hand) == _shape(sugar)
    assert by_hand == as_sugar == {
        "left_arm": (1, 0),
        "right_arm": (1, 0),
    }


# --------------------------------------------------------------------------
# The reason the pump is always on
# --------------------------------------------------------------------------


def test_the_parts_keep_reporting_while_the_caller_is_blocked_in_agent(tmp_path):
    """`waddle.agent()` blocks the calling thread for the whole run, so the
    robot's own loop has to be somewhere else — and a customer who has to
    remember to start it is a customer whose agent run reports nothing. The
    session owns it."""
    rig = _bimanual()
    engaged = threading.Event()
    box: dict = {}

    def run() -> None:
        try:
            box["result"] = waddle.agent("stack the cups", timeout_s=60.0)
        except BaseException as exc:  # re-raised on the main thread below
            box["error"] = exc

    with rig.session("yam-agent", recording_dir=tmp_path, _testing=True) as s:
        for arm in s.arms.values():
            # HOLD_FIRST: the handoff holds before the claimant drives, so the
            # hold verb firing IS the engage landing.
            _watch(arm, "hold", engaged.set)
        ticks = _count_ticks(s.arms)
        caller = threading.Thread(target=run, name="pytest-agent-caller")
        caller.start()
        try:
            _until(
                engaged.is_set,
                "the agent's claim never engaged",
                tick=lambda: waddle._testing.engage(s.core, "agent-claim", "agent"),
            )
            before = {part: n[0] for part, n in ticks.items()}
            _until(
                lambda: all(n[0] > before[part] for part, n in ticks.items()),
                "the parts stopped reporting while the caller was blocked",
            )
            waddle._testing.mark_done(s.core, "success", "the agent is done")
        finally:
            caller.join(timeout=PATIENCE_S)

    assert "error" not in box, f"the agent run raised: {box.get('error')!r}"
    assert box["result"].outcome == "success"
    assert _reported_parts(tmp_path, box["result"].episode_id) == {
        "left_arm",
        "right_arm",
    }
