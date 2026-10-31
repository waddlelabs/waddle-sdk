"""Selected coordinated parts retain native gating and leave other owners alone."""

from dataclasses import replace

import pytest
import yaml
from test_coordinated_playback import _engage, _push_due_chunk, request
from test_named_parts import DRIVERS
from test_named_parts import site_path as site_path  # noqa: PLC0414
from waddle_sdk import load_site
from waddle_sdk.runtime import FaultCode, RuntimeFault, SupportFact


@pytest.fixture
def triple_site(site_path):
    data = yaml.safe_load(site_path.read_text())
    data["parts"]["unused"] = dict(data["parts"]["left"])
    site_path.write_text(yaml.safe_dump(data, sort_keys=False))
    return site_path


@pytest.mark.parametrize("paced", [False, True])
def test_subset_completion_never_commands_or_observes_unrelated_part(
    triple_site, monkeypatch, paced
):
    with (
        load_site(triple_site).open(console=False, _testing=True) as sdk,
        sdk.run(task="selected group", actor="test") as run,
    ):

        def unrelated(*_):
            pytest.fail("subset playback touched the unrelated part")

        monkeypatch.setattr(DRIVERS["unused"], "write", unrelated)
        observe = sdk.observe_parts
        selections = []

        def selected(parts=None):
            selections.append(tuple(parts))
            return observe(parts)

        # The ordinary background feedback pump still observes the whole site.
        # Record the actual playback reads without disabling that pump.
        monkeypatch.setattr(sdk, "observe_parts", selected)
        motion = request(paced=paced)
        result = run.start_playback(motion).wait()
        assert result["status"] == "arrived", result
        assert selections and set(selections) == {("left", "right")}
        assert result["feedback"]["positions"] == pytest.approx(
            motion.references[-1].command.positions
        )
        assert result["cleanup"] is None


def test_subset_support_is_separate_from_complete_group_support(
    triple_site, monkeypatch
):
    with (
        load_site(triple_site).open(console=False, _testing=True) as sdk,
        sdk.run(task="selection capability", actor="test") as run,
    ):
        support = sdk.support()
        assert all(
            SupportFact.COORDINATED_SUBSET_PLAYBACK in row.facts for row in support.rows
        )
        monkeypatch.setattr(
            sdk,
            "support",
            lambda: replace(
                support,
                rows=tuple(
                    replace(
                        row,
                        facts=tuple(
                            f
                            for f in row.facts
                            if f != SupportFact.COORDINATED_SUBSET_PLAYBACK
                        ),
                    )
                    for row in support.rows
                ),
            ),
        )
        with pytest.raises(RuntimeFault) as error:
            run.start_playback(request())
        assert error.value.code == FaultCode.UNSUPPORTED


def test_second_subset_owner_refusal_prevents_first_owner_write(
    triple_site, monkeypatch
):
    with (
        load_site(triple_site).open(console=False, _testing=True) as sdk,
        sdk.run(task="group preflight", actor="test") as run,
    ):
        arms = sdk._require().arms
        original = arms["right"].check

        def refuse(target, current):
            return (
                "selected second owner refusal"
                if abs(target[0]) > 0.001
                else original(target, current)
            )

        monkeypatch.setattr(arms["right"], "check", refuse)
        writes = []
        write = DRIVERS["left"].write

        def record(target):
            writes.append(tuple(target))
            write(target)

        monkeypatch.setattr(DRIVERS["left"], "write", record)
        result = run.start_playback(request()).wait()
        assert result["failure"]["evidence"]["code"] == "safety_refusal", result
        assert "selected second owner refusal" in result["failure"]["detail"]
        assert not any(any(row) for row in writes)
        assert result["cleanup"]["outcome"] == "acknowledged"


def test_subset_takeover_keeps_selected_hint_and_does_not_hold_new_stream(
    triple_site, monkeypatch
):
    with (
        load_site(triple_site).open(console=False, _testing=True) as sdk,
        sdk.run(task="selected takeover", actor="test") as run,
    ):
        run.step((0,) * 6)
        received = []
        driver = DRIVERS["right"]

        def write(target, velocity):
            received.append((tuple(target), tuple(velocity)))
            driver.write(target)

        monkeypatch.setattr(driver, "write_position_velocity", write, raising=False)
        native = _engage(sdk)
        _push_due_chunk(
            native, [0.02, -0.01], part="right", velocity_feedforward=[0.1, -0.05]
        )
        result = run.start_playback(request()).wait()
        assert result["failure"]["code"] == "superseded", result
        assert received == [((0.02, -0.01), (0.1, -0.05))]
        assert result["dispatched"] == 0 and result["cleanup"] is None
        assert sdk._managed.arms["left"].accepted == 1
        assert sdk._managed.arms["unused"].accepted == 1
