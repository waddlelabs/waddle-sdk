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

import time

import numpy as np
import pytest

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


def _bypass_step(session, chunks: list, values, part=None):
    """Drive one step through the BYPASS path — a claimed session whose caller
    loop has stalled, which is the only path that reaches the customer's `send`
    verb (a ticking caller dispatches `gate()`'s own return itself) — and
    return the first step the verb was handed."""
    offer = _Offer(session, values, part)
    deadline = time.monotonic() + PATIENCE_S
    while not chunks:
        assert time.monotonic() < deadline, "the bypass pump never dispatched"
        offer()
        time.sleep(0.02)
    return chunks[0].steps[0]


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

        chunks.clear()
        whole = [0.1] * ARM_DIMS + [0.2] * ARM_DIMS
        values, _, _ = _bypass_step(session, chunks, whole)
        assert isinstance(values, dict)
        # Declaration order is the layout: `left` first, then `right`.
        assert list(values) == ["left", "right"]
        assert list(values["left"]) == [0.1] * ARM_DIMS
        assert list(values["right"]) == [0.2] * ARM_DIMS

        waddle._testing.release(session, "claim-send")
        ep.terminate("success")


def test_blend_window_holds_part_scoped():
    """A part-scoped action does not cross-fade in v0: the gate has no part
    layout with which to pair one arm's rows against a whole-robot anchor, so
    it holds (FSM.md §5). The window here is minutes long, so "still holding"
    is a fact about the contract and not about scheduling; the conformance
    fixture `bimanual_part_scoped_blend_holds` is what pins that substitution
    resumes once the window closes.

    A whole-robot chunk pushed into the SAME open window does blend, which is
    what makes the negative worth asserting: it proves the claim engaged, the
    intake accepted, and the window was open — so the part-scoped silence is
    the contract, not a dead rig.
    """
    session = waddle.init(
        "py-parts-blend",
        _bimanual(),
        _control(),
        handoff=waddle.Handoff.IMMEDIATE(blend_ms=600_000),
        _testing=True,
    )

    with waddle.rollout(task="cross-fade") as ep:
        whole = np.zeros(2 * ARM_DIMS)
        for _ in range(5):
            assert ep.gate(whole) is whole  # an anchor exists to fade out of

        waddle._testing.engage(session, "claim-blend", "agent")
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
