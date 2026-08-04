"""The vendor-neutral half of a robot module: `waddle.robots.base`.

A robot module is what a customer imports instead of writing a driver, an
envelope and a report loop from scratch. Everything in it that is not a
VENDOR FACT lives here — the twin, the envelope seam, the e-stop latch's
console recovery, the loop that reports proprioception — so a second vendor's
module is its facts, its driver, and a factory, and nothing else.

That claim is a test, not a comment: `toy_vendor.py`-in-a-docstring is at the
bottom of this file. A ~30-line toy vendor (a facts dict, the shipped
`SimDriver`, one factory) is built through this layer and driven end to end
against a real session — declaration, envelope, gate, proprio, recording —
with no vendor-specific code in `base` to help it. If the base layer ever
stops carrying all of the behaviour, that test is what fails.

Every layer here is also usable ALONE, which several tests below pin
directly: a driver of your own satisfies `Driver`; an `Arm` built without
forward kinematics reports joint positions and says so; `RobotPump` runs any
tick you hand it; the envelope `Arm` provides is a DEFAULT — a customer who
brings their own send callable keeps it.
"""

from __future__ import annotations

import io
import sys
import threading
import time

import numpy as np
import pytest
from mcap.reader import make_reader
from mcap_protobuf.decoder import DecoderFactory

import waddle
from waddle.robots import base


@pytest.fixture(autouse=True)
def _clean_session():
    yield
    waddle.shutdown()

# How long a poll for another thread's work may run before the test gives up.
# Never a deadline an assertion depends on: every wait below ends on an
# observable event and this only bounds the failure.
PATIENCE_S = 5.0

# One toy part: three rows, the last one a gripper in normalized units. Wide
# enough to have a layout, narrow enough to read in a failure message.
JOINTS = ("lift", "swing", "grip")
LIMITS = ((-1.0, 1.0), (-1.5, 1.5), (0.0, 1.0))
STEPS = (0.10, 0.10, 0.25)
RATE_HZ = 20.0
HOME = (0.0, 0.0, 1.0)


def _twin(home=HOME) -> base.SimDriver:
    return base.SimDriver(
        home,
        lower=[lo for lo, _ in LIMITS],
        upper=[hi for _, hi in LIMITS],
        step_caps=STEPS,
        rate_hz=RATE_HZ,
    )


def _arm(driver=None, *, lines=None, **overrides) -> base.Arm:
    kwargs: dict = dict(
        part="toy",
        driver=driver if driver is not None else _twin(),
        joint_names=JOINTS,
        joint_limits=LIMITS,
        step_caps=STEPS,
        rate_hz=RATE_HZ,
    )
    if lines is not None:
        kwargs["report"] = lines.append
    kwargs.update(overrides)
    return base.Arm(**kwargs)


def _flat_fk(q):
    """A stand-in chain: the tool sits at (lift, 0, 0), unrotated. Enough to
    have a TCP, cheap enough to reason about in a workspace assertion."""
    return np.array([float(q[0]), 0.0, 0.0]), np.eye(3)


class _CountingDriver:
    """A driver written by hand, in a test, satisfying nothing but the
    protocol — which is the point: `Driver` is a shape, not a base class, so
    a customer's own driver slots into the same `Arm`."""

    kind = "sim"

    def __init__(self, home=HOME) -> None:
        self.holds = 0
        self.writes: list[np.ndarray] = []
        self._estopped = False
        self._q = np.array(home, dtype=float)

    @property
    def estopped(self) -> bool:
        return self._estopped

    def read(self):
        return self._q.copy(), np.zeros(self._q.size)

    def write(self, target) -> None:
        self.writes.append(np.asarray(target, dtype=float))

    def hold(self) -> None:
        self.holds += 1

    def estop(self) -> None:
        self._estopped = True

    def re_enable(self) -> None:
        self._estopped = False

    def step(self, dt: float) -> None:
        return None

    def home(self, values) -> bool:
        self._q = np.array(values, dtype=float)
        return True

    def close(self) -> None:
        return None


class _LiveLikeDriver(_CountingDriver):
    """A driver that says it is metal: it integrates itself and has no home to
    snap to. `kind` is what the scene reset and the shutdown question read."""

    kind = "live"

    def home(self, values) -> bool:
        return False


# ---------------------------------------------------------------------------
# The driver protocol
# ---------------------------------------------------------------------------


def test_the_shipped_twin_satisfies_the_driver_protocol():
    assert isinstance(_twin(), base.Driver)


def test_a_driver_written_by_hand_satisfies_the_driver_protocol():
    """Swappable drivers are the point of the protocol: a customer's own
    object is admitted on its members, never on its ancestry."""
    assert isinstance(_CountingDriver(), base.Driver)


def test_an_object_missing_a_verb_is_not_a_driver():
    class NoHold:
        kind = "sim"
        estopped = False

        def read(self):
            return np.zeros(3), np.zeros(3)

    assert not isinstance(NoHold(), base.Driver)


# ---------------------------------------------------------------------------
# The twin
# ---------------------------------------------------------------------------


def test_the_twin_walks_one_accepted_commands_worth_of_travel_per_period():
    """The twin may never move faster than a single accepted command allows,
    which is what makes a sim run a rehearsal of the live one rather than a
    faster, easier version of it."""
    driver = _twin()
    driver.write([1.0, 1.5, 0.0])
    driver.step(1.0 / RATE_HZ)

    position, velocity = driver.read()
    assert list(position) == pytest.approx([0.10, 0.10, 0.75])
    assert list(velocity) == pytest.approx([2.0, 2.0, -5.0])


def test_the_twin_clamps_its_own_state_to_the_declared_limits():
    driver = _twin()
    driver.write([99.0, 0.0, 0.0])
    for _ in range(100):
        driver.step(1.0 / RATE_HZ)
    assert driver.read()[0][0] == pytest.approx(1.0)


def test_the_twin_snaps_home_and_refuses_to_while_latched():
    driver = _twin()
    assert driver.home((0.5, 0.5, 0.5)) is True
    assert list(driver.read()[0]) == pytest.approx([0.5, 0.5, 0.5])

    driver.estop()
    assert driver.home(HOME) is False, (
        "a reset that homed a latched arm would mean every e-stop Waddle asked "
        "for got cancelled by the next episode"
    )
    assert list(driver.read()[0]) == pytest.approx([0.5, 0.5, 0.5])

    driver.re_enable()
    assert driver.estopped is False
    assert driver.home(HOME) is True


# ---------------------------------------------------------------------------
# The envelope: reject, never clamp
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    ("target", "reason"),
    [
        ([0.0, 0.0], "width"),
        ([float("nan"), 0.0, 1.0], "non-finite"),
        ([0.0, 9.0, 1.0], "outside its declared limits"),
        ([0.5, 0.0, 1.0], "would move"),
        ([0.05, 0.0, 1.0], "outside the declared workspace"),
    ],
)
def test_the_envelope_refuses_a_command_whole_and_says_which_check_refused_it(
    target, reason
):
    """Five checks, one behaviour: the command is refused WHOLE, the arm is
    held where it is, and one line names the check. Nothing is clamped,
    narrowed, or partially applied — a clamped command is a command nobody
    wrote, executed faithfully."""
    lines: list[str] = []
    driver = _CountingDriver()
    arm = _arm(driver, lines=lines, fk=_flat_fk, workspace=((-0.02, -1.0, -1.0), (0.02, 1.0, 1.0)))

    assert arm.command(target) is False
    assert driver.writes == [], "a refused command reaches no driver"
    assert driver.holds == 1, "a refused command holds the arm where it is"
    assert arm.rejected == 1 and arm.accepted == 0
    assert len(lines) == 1 and reason in lines[0]
    assert "toy" in lines[0], "the line says which part refused"


def test_the_envelope_applies_a_command_it_admits():
    driver = _CountingDriver()
    arm = _arm(driver)

    assert arm.command([0.05, -0.05, 0.9]) is True
    assert len(driver.writes) == 1
    assert list(driver.writes[0]) == pytest.approx([0.05, -0.05, 0.9])
    assert arm.accepted == 1 and arm.rejected == 0


def test_an_empty_action_row_is_a_hold_not_a_refusal():
    """The wire's "hold this part" — a step addressing this part with no
    motion for it. Nothing is written, nothing is refused, and the arm keeps
    the target it already had."""
    lines: list[str] = []
    driver = _CountingDriver()
    arm = _arm(driver, lines=lines)

    assert arm.command([]) is True
    assert driver.writes == [] and driver.holds == 0
    assert arm.accepted == 0 and arm.rejected == 0
    assert lines == []


def test_a_latched_estop_refuses_every_command_until_it_is_cleared():
    """After the owner's stop the arm has no gains, so a command it "accepted"
    would be a command nothing executed — counted as applied, recorded as an
    action, and read downstream as a rollout that did something."""
    lines: list[str] = []
    driver = _CountingDriver()
    arm = _arm(driver, lines=lines)

    arm.estop()
    assert arm.estopped is True
    assert arm.command([0.05, 0.0, 1.0]) is False
    assert driver.writes == []
    assert arm.rejected == 1
    assert "e-stopped" in lines[0]

    arm.re_enable()
    assert arm.estopped is False
    assert arm.command([0.05, 0.0, 1.0]) is True


def test_a_driver_whose_reads_disagree_with_the_declaration_is_refused_by_name():
    """The envelope compares a target against what the driver just MEASURED,
    so the measurement is part of the arithmetic and gets checked like the
    rest of it. `Driver` is a shape a customer's own object satisfies on its
    members alone, which makes a read that has drifted from the declared joint
    list exactly the mistake this seam exists to name — refused whole, held,
    counted, and said in one line, like every other refusal here."""
    lines: list[str] = []
    driver = _CountingDriver((0.0,) * 5)  # five rows for a three-joint part
    arm = _arm(driver, lines=lines)

    assert arm.command([0.05, 0.0, 1.0]) is False
    assert driver.writes == []
    assert driver.holds == 1, "a refused command holds the arm where it is"
    assert arm.rejected == 1 and arm.accepted == 0
    assert len(lines) == 1
    assert "5" in lines[0] and "3" in lines[0], "the line names both widths"
    assert "disagree" in lines[0]


def test_an_arm_reports_joint_positions_without_forward_kinematics():
    """Forward kinematics is OPT-IN. A rig built without it is legal — it
    reports joint positions and no TCP — and the degradation is named rather
    than guessed at: `ee_pose()` answers None instead of inventing a frame."""
    arm = _arm()
    assert arm.fk is None
    assert arm.ee_pose() is None


def test_an_arm_with_forward_kinematics_reports_a_tcp_pose():
    arm = _arm(fk=_flat_fk, arm_dof=2)
    pose = arm.ee_pose()
    assert pose is not None
    assert list(pose) == pytest.approx([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])


def test_a_workspace_box_without_forward_kinematics_is_refused_at_construction():
    """A box is a statement about the TCP, and a TCP is nothing without the
    chain that produces it. Declaring one with no `fk` would silently check
    nothing."""
    with pytest.raises(ValueError, match="fk"):
        _arm(workspace=((0.0, 0.0, 0.0), (1.0, 1.0, 1.0)))


def test_an_arm_whose_tables_disagree_is_refused_at_construction():
    with pytest.raises(ValueError, match="step_caps"):
        _arm(step_caps=(0.1, 0.1))


# ---------------------------------------------------------------------------
# Bounded reporting
# ---------------------------------------------------------------------------


def test_the_reject_log_prints_at_most_one_line_a_period_and_counts_the_rest():
    """Rejections are a signal, and a signal that arrives at the control rate
    is noise. Nothing is dropped silently: the count of what was suppressed
    rides the next line."""
    now = 0.0
    lines: list[str] = []
    log = base.RejectLog("part=toy", period_s=1.0, report=lines.append, clock=lambda: now)

    log("first")
    for _ in range(5):
        log("suppressed")
    assert len(lines) == 1 and lines[0].endswith("first")

    now = 1.5
    log("second")
    assert len(lines) == 2
    assert "second" in lines[1] and "+5 more" in lines[1]

    now = 3.0
    log("third")
    assert "more" not in lines[2], "the count resets once it has been reported"


# ---------------------------------------------------------------------------
# Routing: the declared layout, and what the gate hands back
# ---------------------------------------------------------------------------


def _two_arms() -> dict[str, base.Arm]:
    return {
        "left": _arm(_CountingDriver(), part="left"),
        "right": base.Arm(
            part="right",
            driver=_CountingDriver((0.0, 0.0)),
            joint_names=("a", "b"),
            joint_limits=((-1.0, 1.0), (-1.0, 1.0)),
            step_caps=(0.5, 0.5),
        ),
    }


def test_a_whole_robot_vector_splits_by_the_declared_layout():
    """Declaration order IS the concatenated layout, and the parts need not be
    the same width. Pure arithmetic over the declaration, never a guess."""
    arms = _two_arms()
    rows = base.split_by_part(arms, [1.0, 2.0, 3.0, 4.0, 5.0])

    assert list(rows) == ["left", "right"]
    assert list(rows["left"]) == pytest.approx([1.0, 2.0, 3.0])
    assert list(rows["right"]) == pytest.approx([4.0, 5.0])


def test_a_whole_robot_vector_of_the_wrong_width_is_refused():
    with pytest.raises(ValueError, match="5 rows"):
        base.split_by_part(_two_arms(), [0.0, 0.0])


def test_a_part_keyed_intervention_commands_only_the_part_it_names():
    """"Move this part, hold the rest": the parts absent from the dict are
    commanded nothing."""
    arms = _two_arms()
    base.apply_decision(arms, {"right": np.array([0.1, 0.1])})

    assert arms["left"].driver.writes == []
    assert len(arms["right"].driver.writes) == 1


def test_a_gripper_on_the_sidechannel_refuses_the_step_whole():
    """This layer models a hand as a JOINT row, so a step carrying a gripper
    on the sidechannel has nowhere to land. It is refused whole and said out
    loud rather than applied without its hand — half a command nobody wrote is
    still a command nobody wrote. (A robot that declares a `Gripper` brings its
    own send callable; the envelope here is a default, not a wall.)"""
    lines: list[str] = []
    arms = _two_arms()
    send = base.chunk_sender(arms, report=lines.append)

    class _Chunk:
        steps = [(np.array([0.0, 0.0, 0.0]), 0.5, 0)]

    send(_Chunk())
    assert arms["left"].driver.writes == []
    assert len(lines) == 1 and "gripper" in lines[0]


def test_the_send_verb_applies_a_step_the_envelope_admits():
    arms = _two_arms()
    send = base.chunk_sender(arms)

    class _Chunk:
        steps = [({"right": np.array([0.1, 0.1])}, None, 0)]

    send(_Chunk())
    assert len(arms["right"].driver.writes) == 1


# ---------------------------------------------------------------------------
# The latch, and the human who clears it
# ---------------------------------------------------------------------------


def test_every_arm_gets_the_stop_even_when_one_of_them_raises():
    """A loop that let the first failure propagate would leave the second arm
    energized because the first one's bus write timed out — which is the exact
    shape of "the e-stop worked, mostly"."""

    class _Refuses(_CountingDriver):
        def estop(self) -> None:
            raise RuntimeError("bus timeout")

    arms = {"left": _arm(_Refuses(), part="left"), "right": _arm(_CountingDriver(), part="right")}
    with pytest.raises(RuntimeError, match="left"):
        base.estop_all(arms, report=lambda line: None)

    assert base.latched_parts(arms) == ["right"]
    assert arms["left"].estopped is False


def test_the_console_resume_gesture_clears_a_latch_and_says_so():
    lines: list[str] = []
    arms = {"left": _arm(part="left"), "right": _arm(part="right")}
    arms["left"].estop()

    base.apply_console_gesture("  Resume\n", arms, report=lines.append)
    assert base.latched_parts(arms) == []
    assert any("resume part=left" in line for line in lines)


def test_a_console_gesture_with_nothing_to_do_is_still_answered():
    """A gesture that silently did nothing would be indistinguishable, at the
    rig, from a program that had stopped listening."""
    lines: list[str] = []
    arms = {"left": _arm(part="left")}

    base.apply_console_gesture("resume", arms, report=lines.append)
    base.apply_console_gesture("wat", arms, report=lines.append)
    base.apply_console_gesture("parked", arms, report=lines.append)

    assert "nothing is e-stopped" in lines[0]
    assert "not a gesture" in lines[1]
    assert "nothing is waiting" in lines[2]


def test_a_park_gesture_is_honoured_only_while_something_is_holding():
    """Two facts rather than one, so the word cannot be typed ahead of time: a
    `parked` accepted at some quiet moment and remembered would release a hold
    the site operator was not standing at."""
    park = base.ParkGate()
    assert park.confirm() is False
    assert park.released is False

    park.begin()
    assert park.confirm() is True
    assert park.released is True

    park.end()
    assert park.confirm() is False


def test_a_pipe_is_not_a_person(monkeypatch):
    """The predicate itself, asked of a stdin this test owns. Everything below
    controls the ANSWER instead of the environment, so this is the one place
    the real function is exercised — and it is exercised against input, not
    against however the suite happened to be invoked."""
    monkeypatch.setattr(sys, "stdin", io.StringIO("resume\n"))
    assert base.console_is_at_the_machine() is False


def test_console_recovery_says_when_it_has_no_terminal_to_be_told_at(monkeypatch):
    """Under a harness there is no gesture at all, and the honest fallback is
    said once rather than discovered when someone types into a pipe.

    The condition is set here rather than inherited: run under `pytest -s`
    from a terminal, a test that asserted on the ambient stdin would fail AND
    leave a reader thread eating the developer's keystrokes for the rest of
    the run."""
    monkeypatch.setattr(base, "console_is_at_the_machine", lambda: False)
    lines: list[str] = []
    assert base.start_console_recovery({}, report=lines.append) is None
    assert len(lines) == 1 and "console: none" in lines[0]


def test_console_recovery_reads_the_gestures_typed_at_a_terminal(monkeypatch):
    """The other branch — the one that matters at the rig, and the one no
    ambient environment should decide: with a terminal there is a reader, the
    banner is said once, and a word typed at it reaches the arms.

    The reader is handed a stdin of this test's own. A thread reading the real
    one would compete with the harness and the shell for every keystroke."""
    monkeypatch.setattr(base, "console_is_at_the_machine", lambda: True)
    monkeypatch.setattr(sys, "stdin", io.StringIO("resume\n"))
    lines: list[str] = []
    arms = {"left": _arm(part="left")}
    arms["left"].estop()

    thread = base.start_console_recovery(arms, park=base.ParkGate(), report=lines.append)
    assert thread is not None
    # The reader ends when its input does — an observable end, not a deadline.
    thread.join(PATIENCE_S)
    assert not thread.is_alive(), "the reader ends with the input it was given"

    assert base.latched_parts(arms) == [], "the gesture reached the arms"
    assert sum("type `resume`" in line for line in lines) == 1, "the banner is said once"
    assert any("resume part=left" in line for line in lines)


# ---------------------------------------------------------------------------
# The scene reset
# ---------------------------------------------------------------------------


def test_the_default_scene_reset_refuses_a_latched_scene():
    """The whole point of a latch: an episode that opened anyway would be
    Waddle asking for a stop and the next rollout cancelling it."""
    lines: list[str] = []
    arms = {"left": _arm(part="left", home_values=HOME)}
    arms["left"].estop()

    assert base.scene_reset(arms, report=lines.append)("fold the towel") is False
    assert any("e-stopped" in line for line in lines)


def test_the_default_scene_reset_homes_a_twin():
    arms = {"left": _arm(part="left", home_values=(0.25, 0.25, 0.25))}
    arms["left"].command([0.1, 0.0, 1.0])

    assert base.scene_reset(arms, report=lambda line: None)("fold the towel") is True
    assert list(arms["left"].state()[0]) == pytest.approx([0.25, 0.25, 0.25])


def test_the_default_scene_reset_moves_no_live_arm_and_says_so():
    """An unattended homing motion is what a runbook forbids, so a live arm's
    reset is the site operator — vouched for on every episode rather than
    assumed."""
    lines: list[str] = []
    driver = _LiveLikeDriver()
    arms = {"left": _arm(driver, part="left", home_values=HOME)}

    assert base.scene_reset(arms, report=lines.append)("fold the towel") is True
    assert any("no motion" in line for line in lines)


def test_closing_a_live_arm_drops_torque_and_a_twin_has_none_to_drop():
    """Asked of the DRIVERS, not of the flag that built them — it is the
    difference between a mission that may exit on its own and one that must
    wait for a human."""
    assert base.closing_drops_torque({"left": _arm(_LiveLikeDriver())}) is True
    assert base.closing_drops_torque({"left": _arm()}) is False


# ---------------------------------------------------------------------------
# The loop
# ---------------------------------------------------------------------------


def test_the_pump_runs_the_tick_it_was_given_until_it_is_stopped():
    """`RobotPump` is a loop, not a robot: it runs any tick at a declared rate,
    which is why a program with its own reporting still uses this one."""
    ticked = threading.Event()
    calls: list[float] = []

    def tick(dt: float) -> None:
        calls.append(dt)
        ticked.set()

    pump = base.RobotPump(tick, 100.0)
    pump.start()
    assert ticked.wait(PATIENCE_S), "the pump never ran its tick"
    pump.stop()

    assert not pump.is_alive(), "stop() joins the thread"
    assert calls[0] == pytest.approx(0.01), "the tick is handed the declared period"


def test_the_proprio_tick_steps_every_part_and_reports_it():
    """One turn of the robot's own loop: integrate, then report every part.
    `joint_pos` is passed explicitly per part — a per-part sample cannot ride
    the gate's flat `obs` vector, whose layout no declaration describes."""
    reports: list[dict] = []

    class _Session:
        def report_proprio(self, **kwargs) -> None:
            reports.append(kwargs)

    arms = {"left": _arm(part="left"), "right": _arm(part="right", fk=_flat_fk, arm_dof=2,
                                                     base_frame="toy_base")}
    arms["left"].command([0.1, 0.0, 1.0])
    base.proprio_tick(_Session(), arms)(1.0 / RATE_HZ)

    by_part = {r["part"]: r for r in reports}
    assert sorted(by_part) == ["left", "right"]
    assert list(by_part["left"]["joint_pos"]) == pytest.approx([0.1, 0.0, 1.0])
    assert "ee_pose" not in by_part["left"], (
        "an arm with no forward kinematics reports joint positions only"
    )
    assert by_part["right"]["ee_pose_frame"] == "toy_base"
    assert len(by_part["right"]["ee_pose"]) == 7


# ---------------------------------------------------------------------------
# Kinematics helpers
# ---------------------------------------------------------------------------


def test_chain_fk_walks_the_declared_chain():
    position, rotation = base.chain_fk(
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        (0.5, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (np.pi / 2, 0.0),
    )
    assert list(position) == pytest.approx([0.0, 1.5, 0.0], abs=1e-9)
    assert list(base.quaternion_wxyz(rotation)) == pytest.approx(
        [np.cos(np.pi / 4), 0.0, 0.0, np.sin(np.pi / 4)]
    )


@pytest.mark.parametrize(
    ("rpy", "expected"),
    [
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)),
        ((np.pi, 0.0, 0.0), (0.0, 1.0, 0.0, 0.0)),
        ((0.0, np.pi, 0.0), (0.0, 0.0, 1.0, 0.0)),
        ((0.0, 0.0, np.pi), (0.0, 0.0, 0.0, 1.0)),
    ],
)
def test_quaternion_wxyz_is_w_first_on_every_branch(rpy, expected):
    """wxyz is this protocol's pinned convention, and handing it an xyzw
    quaternion is the classic silent-corruption bug — so all four branches of
    the conversion are pinned, not just the one a small rotation takes."""
    got = base.quaternion_wxyz(base.rpy_matrix(*rpy))
    assert list(got) == pytest.approx(list(expected), abs=1e-9)


def test_a_cross_arm_mounting_becomes_one_declared_edge():
    """A pair fact is measured as xyz + rpy and declared as xyz + a **wxyz**
    quaternion. The conversion happens here, once, so no caller is in a
    position to hand a declaration an xyzw quaternion — the classic silent
    corruption, and one that reads as a plausible pose."""
    edge = base.CrossArm(xyz=(0.0, -0.60, 0.0), rpy=(0.0, 0.0, -0.15)).transform(
        "left_base", "right_base"
    )
    assert (edge.parent, edge.child) == ("left_base", "right_base")
    assert list(edge.position) == pytest.approx([0.0, -0.60, 0.0])
    assert list(edge.quaternion) == pytest.approx(
        [np.cos(-0.075), 0.0, 0.0, np.sin(-0.075)], abs=1e-9
    )


def test_a_cross_arm_mounting_of_the_wrong_shape_is_refused():
    with pytest.raises(ValueError, match="CrossArm.rpy"):
        base.CrossArm(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0))
    with pytest.raises(ValueError, match="CrossArm.xyz"):
        base.CrossArm(xyz=(0.0, float("nan"), 0.0), rpy=(0.0, 0.0, 0.0))


# ---------------------------------------------------------------------------
# Posture, and the rig that composes it
# ---------------------------------------------------------------------------


def test_a_posture_decides_which_verbs_are_registered_and_nothing_else():
    """Grants are DERIVED from which verbs a session registers, so a monitor
    posture says on the wire that nothing may command this robot — instead of
    accepting motion it intends to drop. It adds no authority logic: who may
    command a robot, when, and under what claim is unchanged either way."""
    arms = {"toy": _arm()}

    supervised = base.control(arms)
    assert supervised.send is not None
    assert supervised.hold is not None
    assert supervised.estop is not None

    monitor = base.control(arms, posture="monitor")
    assert monitor.send is None
    assert monitor.hold is None, "no send verb, nothing to stop sending"
    assert monitor.estop is not None, "the owner's stop is registered either way"


def test_a_monitor_posture_refuses_a_send_callable_instead_of_dropping_it():
    with pytest.raises(ValueError, match="monitor"):
        base.control({"toy": _arm()}, posture="monitor", send=lambda chunk: None)


def test_the_registered_verbs_reach_every_arm():
    arms = {"left": _arm(_CountingDriver(), part="left"),
            "right": _arm(_CountingDriver(), part="right")}
    verbs = base.control(arms, report=lambda line: None)

    verbs.hold()
    assert [arm.driver.holds for arm in arms.values()] == [1, 1]
    verbs.estop()
    assert base.latched_parts(arms) == ["left", "right"]


def test_the_envelope_is_a_replaceable_default():
    """Owner-side doctrine: what this layer ships is a default built out of
    the owner's own numbers, never a wall. A customer's own send callable is
    the whole envelope, and everything else here still applies."""
    seen: list = []

    def my_send(chunk) -> None:
        seen.append(chunk)

    arms = {"toy": _arm(_CountingDriver())}
    verbs = base.control(arms, send=my_send)
    assert verbs.send is my_send

    class _Chunk:
        # A target the shipped envelope would refuse outright.
        steps = [(np.array([9.0, 9.0, 9.0]), None, 0)]

    verbs.send(_Chunk())
    assert len(seen) == 1
    assert arms["toy"].rejected == 0, (
        "the customer's callable IS the envelope — the default is replaced, not "
        "consulted alongside it"
    )


def test_an_unknown_posture_is_refused_by_name():
    with pytest.raises(ValueError, match="posture"):
        base.control({"toy": _arm()}, posture="observe")


# ---------------------------------------------------------------------------
# The second-vendor bar: a toy vendor module, through the same base layer
# ---------------------------------------------------------------------------
#
# Everything below the line is what a NEW vendor module contains: a facts
# table, a driver (the shipped twin here — a real one wraps a vendor SDK), and
# a factory. It is the template a customer copies, and it is a test rather
# than a docs snippet because the claim it makes — "the base layer carries all
# of the behaviour" — is only true while it keeps passing.

TOY_FACTS = {
    # The vendor's own numbers, with their provenance in the comment beside
    # them in a real module. A toy crane: two arm joints and a hand.
    "joints": ("boom", "stick", "grip"),
    "limits": ((-1.0, 1.0), (-1.5, 1.5), (0.0, 1.0)),
    "step_caps": (0.10, 0.10, 0.25),
    "max_effort_nm": 4.0,
    "rate_hz": 20.0,
    "home": (0.0, 0.0, 1.0),
}


def toy_driver() -> base.SimDriver:
    return base.SimDriver(
        TOY_FACTS["home"],
        lower=[lo for lo, _ in TOY_FACTS["limits"]],
        upper=[hi for _, hi in TOY_FACTS["limits"]],
        step_caps=TOY_FACTS["step_caps"],
        rate_hz=TOY_FACTS["rate_hz"],
    )


def toy_crane(*, posture: str = "supervised") -> base.Rig:
    """The whole of a second vendor module: declare the robot, say how to open
    it, hand back a rig."""
    space = waddle.JointSpace(
        joints=[
            waddle.Joint(name=name, min_position=lo, max_position=hi,
                         max_effort=TOY_FACTS["max_effort_nm"])
            for name, (lo, hi) in zip(TOY_FACTS["joints"], TOY_FACTS["limits"])
        ],
        rate_hz=TOY_FACTS["rate_hz"],
        chunking=waddle.Chunking(horizon=1, replan="immediate", interp="hold"),
    )

    def build_arms() -> dict[str, base.Arm]:
        return {
            "": base.Arm(
                part="",
                driver=toy_driver(),
                joint_names=TOY_FACTS["joints"],
                joint_limits=TOY_FACTS["limits"],
                step_caps=TOY_FACTS["step_caps"],
                rate_hz=TOY_FACTS["rate_hz"],
                home_values=TOY_FACTS["home"],
            )
        }

    return base.Rig(
        declaration=waddle.Robot(
            name="toy-crane", robot_id="toy-crane-01", action_space=space
        ),
        build_arms=build_arms,
        rate_hz=TOY_FACTS["rate_hz"],
        posture=posture,
    )


# --------------------------- end of the toy vendor -------------------------


def _observations(mcap_path):
    """Every decoded `/waddle/observations` message, via the channel's own
    embedded schema — the same read any external MCAP consumer would do."""
    with open(mcap_path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        return [
            msg
            for _, channel, _, msg in reader.iter_decoded_messages()
            if channel.topic == "/waddle/observations"
        ]


def test_a_rig_is_a_declaration_until_it_is_asked_for_arms():
    """A factory call opens no bus and starts no thread, so it is cheap,
    testable, and safe to make in a program that then decides not to run."""
    opened: list[int] = []

    def build_arms() -> dict[str, base.Arm]:
        opened.append(1)
        return {"toy": _arm()}

    rig = base.Rig(
        declaration=toy_crane().robot(), build_arms=build_arms, rate_hz=RATE_HZ
    )
    assert opened == []
    assert rig.arms().keys() == {"toy"}
    assert opened == [1]


def test_a_monitor_rig_declares_no_way_to_command_it():
    rig = toy_crane(posture="monitor")
    assert rig.control(rig.arms()).send is None


def test_a_monitor_rig_opens_a_session_that_offers_only_the_owners_stop(tmp_path):
    """A posture has to survive the thing it is for — a session, not just a
    `Control`. What the wire is told is derived from which verbs exist, so the
    whole grant list here is the owner's stop; and the program's own policy
    still drives the robot through the gate, which is what makes "nothing may
    command it" a statement about the SUPERVISION side rather than about the
    machine."""
    rig = toy_crane(posture="monitor")
    arms = rig.arms()
    verbs = rig.control(arms)
    assert waddle._derive_grants(verbs, rig.robot().action_space) == [
        {"verb": "VERB_ESTOP"}
    ]

    waddle.init("monitor-smoke", rig.robot(), verbs, recording_dir=tmp_path)
    try:
        with waddle.rollout(task="watch the crane") as ep:
            position = arms[""].state()[0]
            decided = ep.gate(position + np.array([0.05, 0.0, 0.0]), position)
            assert decided is not None, "the program's own action still passes through"
            base.apply_decision(arms, decided)
            ep.terminate("success")
    finally:
        waddle.shutdown()

    assert arms[""].accepted == 1 and arms[""].rejected == 0


def test_a_monitor_session_may_not_wire_a_media_plane():
    """The one wiring this posture cannot take, pinned so it is a decision
    rather than a surprise at the rig.

    The media plane carries the teleoperator's stream as well as the video, so
    wiring one IS an intervention path, and waddle-core refuses a session that
    would take motion it has no `send` verb to apply — `_testing=True` is that
    same plane, in process. Watching is undiminished without it: `transport=`
    uplinks proprioception and each camera's declared low-rate stills over the
    control plane, and `recording_dir=` keeps the full-rate archive locally.
    A session a teleoperator may take over is `posture="supervised"`.

    The refusal is the core's and stays the core's: this layer maps a posture
    to verb presence and to nothing else, so it does not grow a second copy of
    an engage-path rule to phrase the message more kindly."""
    rig = toy_crane(posture="monitor")
    with pytest.raises(RuntimeError, match="hold"):
        waddle.init("monitor-media", rig.robot(), rig.control(rig.arms()), _testing=True)


def test_a_second_vendor_rides_the_base_layer_end_to_end(tmp_path):
    """The bar every robot module has to clear: a facts table, a driver and a
    factory, composed by hand out of the pieces above — declaration, arms,
    verbs, scene reset, loop — and driven through a real session with no
    vendor-specific code in `base` to help it.

    What it proves, in one run: the declaration registers; the pump reports
    the sole part into the episode's own recording; the envelope admits the
    policy's commands and counts them; and a part with no forward kinematics
    lands as joint positions with no TCP rather than a pose nobody declared."""
    rig = toy_crane()
    arms = rig.arms()
    session = waddle.init(
        "toy-vendor-smoke",
        rig.robot(),
        rig.control(arms),
        recording_dir=tmp_path,
        pre_reset=rig.pre_reset(arms),
    )

    # The pump is usable alone: it runs the tick it is handed, and this one
    # counts the reports that landed while an episode was open. `inside` is
    # read BEFORE the report, so a tick it counts is one that was recorded —
    # the wait below then ends on that happens-before rather than on a clock.
    inner = base.proprio_tick(session, arms)
    in_episode = threading.Event()
    reported_in_episode = threading.Event()

    def tick(dt: float) -> None:
        inside = in_episode.is_set()
        inner(dt)
        if inside:
            reported_in_episode.set()

    pump = base.RobotPump(tick, rig.rate_hz)
    pump.start()
    try:
        with waddle.rollout(task="raise the boom") as ep:
            episode_id = ep.id
            in_episode.set()
            for _ in range(10):
                position = arms[""].state()[0]
                action = position + np.array([0.05, 0.0, 0.0])
                decided = ep.gate(action, position)
                if decided is not None:
                    base.apply_decision(arms, decided)
                time.sleep(0.005)
            assert reported_in_episode.wait(PATIENCE_S), "the pump never reported"
            ep.terminate("success")
    finally:
        pump.stop()
        waddle.shutdown()
        for arm in arms.values():
            arm.close()

    assert arms[""].accepted == 10, "every command was inside the declared envelope"
    assert arms[""].rejected == 0

    samples = [o.proprio for o in _observations(tmp_path / f"{episode_id}.mcap")]
    assert {s.part for s in samples} == {""}, "a sole-part robot reports under ''"
    from_the_pump = [s for s in samples if len(s.joint_vel) == len(TOY_FACTS["joints"])]
    assert from_the_pump, "no per-part sample carried the pump's velocities"
    assert not any(s.HasField("ee_pose") for s in samples), (
        "this rig declared no forward kinematics, so it reports joint positions "
        "and no TCP — the degradation is named, never filled in"
    )
