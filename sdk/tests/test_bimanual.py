"""Part-addressed control through the Python surface (flag `waddle.v0.parts`).

A bimanual cell declares a `Composite` action space, and the core declares
`waddle.v0.parts` for every such declaration — so a plane may address ONE
arm. On this distribution every payload carrying such an action into Python
therefore says which part it addressed, and it says it the same way
everywhere: **on a Composite declaration, an intervention payload is keyed by
part**. `gate()` returns `{"right": ndarray}`; a dispatched `Chunk`'s step
values are `{"right": ndarray}`; a whole-robot action is the same dict with
every declared part in it, sliced by the declared layout (declaration order
IS the concatenated action-vector layout, so the slicing is arithmetic, never
invention).

Single-part declarations are untouched: their steps and gate returns are the
bare float64 ndarray they have always been, which `test_single_part_surface_is_unchanged`
pins from this side of the change.

The core-side proofs of the same behavior are `waddle-runtime`'s
`tests/part_scoped_intake.rs` and the conformance fixtures
`bimanual_part_scoped_*`; these tests are about what crosses into Python.
"""

from __future__ import annotations

import json
import time

import numpy as np
import pytest
from mcap.reader import make_reader
from mcap_protobuf.decoder import DecoderFactory

import waddle
import waddle._testing

# Seven rows per arm — six joints plus the gripper folded in as the last row,
# which is the canonical bimanual declaration (`Gripper.parallel(dim=-1)`) and
# why this stage needs no per-part gripper sidechannel.
ARM_DIMS = 7

# How long a poll for a core-driven event may run before the test gives up.
# Never a deadline the assertion depends on: every wait below ends on an
# observable event and this only bounds the failure.
PATIENCE_S = 5.0

# How often a chunk may be re-offered. A chunk is offered repeatedly because
# the claim engages on the core's own schedule and a chunk offered before it
# does is correctly dropped (nothing is buffered without a claim) — but the
# offers are SPACED, because under the declared IMMEDIATE replan policy a new
# chunk supersedes the still-pending steps of the one before it. Offering
# faster than the stream's playout delay would therefore replace, on every
# pass, the step that was about to play. This is a property of the offer
# schedule, not a race with anything: nothing below asserts on when a step
# lands, only that one eventually does.
OFFER_INTERVAL_S = 0.1


@pytest.fixture(autouse=True)
def _clean_session():
    yield
    waddle.shutdown()


def _arm(prefix: str) -> waddle.JointSpace:
    return waddle.JointSpace(joints=[f"{prefix}{i}" for i in range(ARM_DIMS)])


def _bimanual() -> waddle.Robot:
    """Two named 7-row parts, in declaration order (which IS the concatenated
    14-row layout)."""
    return waddle.Robot(
        name="pytest-bimanual",
        robot_id="py-bimanual-01",
        cell_id="cell-py-bimanual",
        action_space=waddle.Composite(
            left=_arm("l"),
            right=_arm("r"),
            rate_hz=50,
        ),
    )


def _single_part() -> waddle.Robot:
    """A declaration with no addressable parts at all — the surface that must
    not move."""
    return waddle.Robot(
        name="pytest-one-arm",
        robot_id="py-one-arm-01",
        cell_id="cell-py-bimanual",
        action_space=waddle.JointSpace(
            joints=[f"j{i}" for i in range(ARM_DIMS)], rate_hz=50
        ),
    )


def _control(chunks: list | None = None) -> waddle.Control:
    def send(chunk):
        if chunks is not None:
            chunks.append(chunk)

    return waddle.Control(send=send, hold=lambda: None, resume=lambda: None)


class _Offer:
    """One intervention chunk, offered to the session until the core takes it
    (see `OFFER_INTERVAL_S` for why offering is repeated, and why it is
    spaced)."""

    def __init__(self, session, values, part=None, gripper=None):
        self._push = (session, list(values), part, gripper)
        self._next = 0.0

    def __call__(self) -> None:
        now = time.monotonic()
        if now >= self._next:
            session, values, part, gripper = self._push
            waddle._testing.push_chunk(session, values, part=part, gripper=gripper)
            self._next = now + OFFER_INTERVAL_S


def _bypass_step(session, chunks: list, values, part=None, gripper=None, accept=None):
    """Drive one step through the BYPASS path — a claimed session whose caller
    loop has stalled, which is the only path that reaches the customer's `send`
    verb (a ticking caller dispatches `gate()`'s own return itself) — and
    return the first dispatched step `accept` recognises (by default, the
    first step at all).

    `accept` is how a test that drives a SECOND intervention after a first one
    stays honest. An offer is repeated (see `_Offer`), so when the first
    dispatch lands there may be one more step already buffered behind it, and
    clearing the log and reading its next entry would read that leftover as
    the second intervention's. Selecting the dispatch by what it must CONTAIN
    is an observable happens-before: a leftover from the previous shape can
    never satisfy the next shape's predicate, so the wait ends on the arrival
    it is about and nothing else."""
    offer = _Offer(session, values, part, gripper)
    seen = 0
    deadline = time.monotonic() + PATIENCE_S
    while True:
        assert time.monotonic() < deadline, "the bypass pump never dispatched"
        while seen < len(chunks):
            step = chunks[seen].steps[0]
            seen += 1
            if accept is None or accept(step[0]):
                return step
        offer()
        time.sleep(0.02)


def _gated_substitute(ep, session, action, values, part=None):
    """Tick the gate against `action` until the offered chunk comes back
    through it, and return what the gate handed over."""
    offer = _Offer(session, values, part)
    deadline = time.monotonic() + PATIENCE_S
    while True:
        assert time.monotonic() < deadline, "the offered chunk never reached the gate"
        offer()
        out = ep.gate(action)
        if out is not None and out is not action:
            return out
        time.sleep(0.005)


def _validation_faults(recording_dir, episode_id: str) -> list[dict]:
    """Every VALIDATION_ERROR fault on the episode's own timeline — what an
    intervention chunk the intake REFUSED leaves behind (a dims mismatch, an
    undeclared part). Silence at the gate means one thing when the chunk was
    admitted and quite another when it was thrown away upstream, and this is
    the only place a Python caller can tell the two apart."""
    sidecar = json.loads(
        (recording_dir / f"{episode_id}.sidecar.json").read_text()
    )
    return [
        e["fault"]
        for e in sidecar.get("events", [])
        if "fault" in e and e["fault"].get("kind") == "FAULT_KIND_VALIDATION_ERROR"
    ]


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


def test_single_part_surface_is_unchanged():
    """A declaration with no parts hands `send` exactly what it always did:
    `(float64 ndarray, gripper, offset_ns)`. Dict-by-part is what a Composite
    declaration buys; it is not a tax on everyone else."""
    chunks: list = []
    session = waddle.init(
        "py-parts-single", _single_part(), _control(chunks), _testing=True
    )

    with waddle.rollout(task="one arm") as ep:
        ep.gate(np.zeros(ARM_DIMS))
        # Passthrough is a whole-robot decision like any other: no part.
        assert ep.last_gate.part is None
        waddle._testing.engage(session, "claim-single", "agent")
        values, gripper, offset_ns = _bypass_step(session, chunks, [0.25] * ARM_DIMS)

        assert isinstance(values, np.ndarray)
        assert values.dtype == np.float64
        assert values.shape == (ARM_DIMS,)
        assert list(values) == [0.25] * ARM_DIMS
        assert gripper is None
        assert offset_ns == 0

        waddle._testing.release(session, "claim-single")
        ep.terminate("success")


def test_part_scoped_substitute_returns_dict_by_part():
    """`gate()` on a Composite session returns the intervention keyed by the
    part it commands. Without this, one arm's 7 rows arrive indistinguishable
    from a 14-row whole-robot command."""
    session = waddle.init("py-parts-gate", _bimanual(), _control(), _testing=True)

    with waddle.rollout(task="right arm alone") as ep:
        whole = np.zeros(2 * ARM_DIMS)
        ep.gate(whole)
        waddle._testing.engage(session, "claim-gate", "agent")
        out = _gated_substitute(ep, session, whole, [0.75] * ARM_DIMS, part="right")

        assert isinstance(out, dict), f"a Composite session returns dict-by-part, got {out!r}"
        assert list(out) == ["right"], "only the addressed part is commanded"
        assert list(out["right"]) == [0.75] * ARM_DIMS
        assert out["right"].dtype == np.float64
        assert ep.last_gate.kind == "substitute"
        assert ep.last_gate.part == "right"

        waddle._testing.release(session, "claim-gate")
        ep.terminate("success")


def test_composite_send_receives_dict_by_part_steps():
    """The dispatched-chunk surface follows the same one rule as `gate()`: a
    part-scoped step carries only the part it addresses, a whole-robot step
    carries every declared part, sliced by the declared layout."""
    chunks: list = []
    session = waddle.init(
        "py-parts-send", _bimanual(), _control(chunks), _testing=True
    )

    with waddle.rollout(task="both arms") as ep:
        ep.gate(np.zeros(2 * ARM_DIMS))
        waddle._testing.engage(session, "claim-send", "agent")

        values, _, _ = _bypass_step(session, chunks, [1.25] * ARM_DIMS, part="right")
        assert isinstance(values, dict)
        assert list(values) == ["right"]
        assert list(values["right"]) == [1.25] * ARM_DIMS

        # The whole-robot dispatch is selected by the shape only it can have
        # (both declared parts), so a part-scoped step still buffered behind
        # the one above is skipped rather than misread as this one.
        whole = [0.1] * ARM_DIMS + [0.2] * ARM_DIMS
        values, _, _ = _bypass_step(
            session, chunks, whole, accept=lambda v: list(v) == ["left", "right"]
        )
        assert isinstance(values, dict)
        # Declaration order is the layout: `left` first, then `right`.
        assert list(values["left"]) == [0.1] * ARM_DIMS
        assert list(values["right"]) == [0.2] * ARM_DIMS

        waddle._testing.release(session, "claim-send")
        ep.terminate("success")


def test_part_scoped_gripper_only_step_is_an_empty_array_for_that_part(tmp_path):
    """The two shape rules compose. A gripper-only step — "hold the arm, move
    the gripper" — carries no arm rows, and a part-scoped one carries only the
    part it names, so a part-scoped gripper-only step is that part alone
    mapped to an EMPTY array, with the grip on the step's own `gripper` slot.
    The key set is what says which arm is being held: an empty dict would say
    "this session commanded nothing", which is not what happened.

    The empty-values shape is also the one a hand-rolled encoder gets wrong:
    an empty joint vector is a dims mismatch against a 7-row part, so a step
    marshalled as one is refused at the intake and never dispatched. The
    timeline is asserted clean for exactly that reason."""
    chunks: list = []
    session = waddle.init(
        "py-parts-grip",
        _bimanual(),
        _control(chunks),
        recording_dir=tmp_path,
        _testing=True,
    )

    with waddle.rollout(task="close the right gripper") as ep:
        episode_id = ep.id
        ep.gate(np.zeros(2 * ARM_DIMS))
        waddle._testing.engage(session, "claim-grip", "agent")

        values, gripper, _ = _bypass_step(
            session, chunks, [], part="right", gripper=0.03
        )
        assert isinstance(values, dict)
        assert list(values) == ["right"], "the held arm is named, not omitted"
        assert values["right"].dtype == np.float64
        assert values["right"].size == 0, "a gripper-only step commands no arm rows"
        assert gripper == pytest.approx(0.03)

        waddle._testing.release(session, "claim-grip")
        ep.terminate("success")
    waddle.shutdown()

    assert _validation_faults(tmp_path, episode_id) == [], (
        "a part-scoped gripper-only step is a legal action; a validation "
        "fault here means it was marshalled into a shape the intake refused"
    )


def test_report_proprio_part_round_trips_through_mcap(tmp_path):
    """A per-part sample keys its own recording row. It cannot ride the gate's
    flat `obs` vector — the observation layout is the customer's own and no
    declaration describes it, so slicing it by action parts would invent a
    mapping nobody declared — which is why `joint_pos` is a kwarg here."""
    session = waddle.init(
        "py-parts-proprio", _bimanual(), _control(), recording_dir=tmp_path
    )

    with waddle.rollout(task="report both arms") as ep:
        episode_id = ep.id
        session.report_proprio(part="left", joint_pos=[0.5] * ARM_DIMS, gripper=0.02)
        session.report_proprio(
            part="right", joint_pos=np.full(ARM_DIMS, -0.5), gripper=0.04
        )
        for _ in range(10):
            ep.gate(np.zeros(2 * ARM_DIMS), np.zeros(2 * ARM_DIMS))
            time.sleep(0.01)
        ep.terminate("success")
    waddle.shutdown()

    samples = [o.proprio for o in _observations(tmp_path / f"{episode_id}.mcap")]
    by_part = {s.part: s for s in samples}
    assert "left" in by_part and "right" in by_part, (
        f"each reported part must land as its own row, got parts {sorted(by_part)}"
    )
    assert list(by_part["left"].joint_pos) == [0.5] * ARM_DIMS
    assert by_part["left"].gripper == pytest.approx(0.02)
    assert list(by_part["right"].joint_pos) == [-0.5] * ARM_DIMS
    assert by_part["right"].gripper == pytest.approx(0.04)
    # The gate-tick stream still records the robot as declared, under "".
    assert "" in by_part


def test_report_proprio_unknown_part_raises():
    """Refused by NAME: a typo'd part is a declaration error the caller can
    fix, and reporting one arm's state under a name the robot does not have
    would put it in the corpus as fact."""
    session = waddle.init("py-parts-unknown", _bimanual(), _control())

    with waddle.rollout(task="typo") as ep:
        with pytest.raises(ValueError, match="waist"):
            session.report_proprio(part="waist", joint_pos=[0.0] * ARM_DIMS)
        # "" is the sole/default part and is always legal, on any declaration.
        session.report_proprio(part="", joint_pos=[0.0] * 2 * ARM_DIMS)
        ep.terminate("success")


def test_blend_window_holds_part_scoped(tmp_path):
    """A part-scoped action does not cross-fade in v0: the gate has no part
    layout with which to pair one arm's rows against a whole-robot anchor, so
    it holds (FSM.md §5). The window here is minutes long, so "still holding"
    is a fact about the contract and not about scheduling; the conformance
    fixture `bimanual_part_scoped_blend_holds` is what pins that substitution
    resumes once the window closes.

    A negative ("the gate never handed it over") is also what a dead rig
    produces, so two things are asserted alongside it, and it takes both:

    * a whole-robot chunk pushed into the SAME open window blends — the claim
      engaged, the window is open, and steps still play out of the stream;
    * the episode timeline carries NO validation fault — so the part-scoped
      chunk was ADMITTED by the intake, not refused before it ever reached
      the gate. That is the one this test would otherwise be missing: an
      intake that does not honour `Action.part` reads a 7-row action against
      a 14-row robot and refuses the chunk, and every assertion about gate
      silence below would then hold for the wrong reason.

    "No fault" only means "admitted" once a claim is actually engaged — a
    chunk pushed with none is dropped silently, by design, and would leave
    the same empty timeline. So the offers below start only after the gate
    has observably stopped passing the caller's action through, which is
    this side's view of the engage completing.
    """
    session = waddle.init(
        "py-parts-blend",
        _bimanual(),
        _control(),
        handoff=waddle.Handoff.IMMEDIATE(blend_ms=600_000),
        recording_dir=tmp_path,
        _testing=True,
    )

    with waddle.rollout(task="cross-fade") as ep:
        episode_id = ep.id
        whole = np.zeros(2 * ARM_DIMS)
        for _ in range(5):
            assert ep.gate(whole) is whole  # an anchor exists to fade out of

        waddle._testing.engage(session, "claim-blend", "agent")
        # Engage is the core's to complete on its own schedule; under a claim
        # the gate stops handing the caller's action back, which is the
        # observable edge every push below depends on.
        deadline = time.monotonic() + PATIENCE_S
        while ep.gate(whole) is whole:
            assert time.monotonic() < deadline, "the claim never engaged"
            time.sleep(0.005)

        offer = _Offer(session, [0.9] * ARM_DIMS, part="right")
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            offer()
            out = ep.gate(whole)
            assert out is None or out is whole, (
                f"a part-scoped action must not cross-fade out of a whole-robot "
                f"anchor; the gate returned {out!r} ({ep.last_gate!r})"
            )
            time.sleep(0.005)

        # The same window, a whole-robot action: this one has an anchor of its
        # own scope and width, so it cross-fades.
        out = _gated_substitute(ep, session, whole, [0.3] * 2 * ARM_DIMS)
        assert ep.last_gate.kind == "blend", "the cross-fade window must still be open"
        assert ep.last_gate.part is None
        assert list(out) == ["left", "right"]

        waddle._testing.release(session, "claim-blend")
        ep.terminate("success")
    waddle.shutdown()

    assert _validation_faults(tmp_path, episode_id) == [], (
        "the part-scoped chunk must have been ADMITTED: a validation fault "
        "here means the intake refused it, so nothing was ever offered to "
        "the gate and the hold above proved nothing"
    )
