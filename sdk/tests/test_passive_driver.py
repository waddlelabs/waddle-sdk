"""Passive hardware reports measurements without control or torque ownership."""

from dataclasses import replace

import pytest
import site_fixtures
import waddle_sdk
import waddle_sdk.site as site_api
from test_site_api import _write_site
from waddle_sdk.robots import base
from waddle_sdk.runtime import (
    FaultCode,
    JointPositionCommand,
    RuntimeFault,
    SupportFact,
)


class PassiveDriver(site_fixtures._Driver):
    kind = "physical-observer"
    actuation_supported = False
    estop_supported = False
    close_releases_torque = False

    def write(self, target):
        raise RuntimeFault(FaultCode.UNSUPPORTED, "Observer cannot actuate")

    def write_position_velocity(self, target, velocity_feedforward_rad_s):
        self.write(target)

    def hold(self):
        raise RuntimeFault(FaultCode.UNSUPPORTED, "Observer cannot hold")

    def estop(self):
        raise RuntimeFault(FaultCode.UNSUPPORTED, "Observer cannot stop hardware")

    def re_enable(self):
        raise RuntimeFault(FaultCode.UNSUPPORTED, "Observer cannot enable hardware")


def arm_for(driver):
    return base.Arm(
        part="arm",
        driver=driver,
        joint_names=("j0", "j1"),
        joint_limits=((-1, 1), (-1, 1)),
        step_caps=(0.1, 0.1),
    )


@pytest.fixture
def passive_site(tmp_path, monkeypatch):
    site_fixtures.reset()
    original = site_fixtures.part

    def part(*, config):
        rig = original(config=replace(config, posture="supervised"))
        return replace(rig, posture=config.posture)

    monkeypatch.setattr(site_fixtures, "part", part)
    monkeypatch.setattr(site_fixtures, "_Driver", PassiveDriver)
    path = _write_site(tmp_path)
    path.write_text(path.read_text().replace("posture: supervised", "posture: monitor"))

    def unexpected_wait(_self):
        pytest.fail("A passive socket owner must not wait for torque-release approval")

    monkeypatch.setattr(base.ParkGate, "wait", unexpected_wait)
    yield waddle_sdk.load_site(path)
    site_fixtures.reset()


def test_passive_site_observes_refuses_control_and_closes_normally(passive_site):
    with passive_site.open(console=False) as session:
        assert session.observe_parts(["arm"]).parts["arm"].joint_position.tolist() == [
            0,
            0,
        ]
        facts = set(session.support().rows[0].facts)
        assert SupportFact.PART_OBSERVATION in facts
        assert SupportFact.JOINT_POSITION_OBSERVATION in facts
        assert not facts.intersection(
            {
                SupportFact.PART_ACTION,
                SupportFact.JOINT_POSITION_ACTION,
                SupportFact.VELOCITY_FEEDFORWARD,
                SupportFact.SEND_GRANT,
                SupportFact.HOLD_GRANT,
                SupportFact.ESTOP_GRANT,
            }
        )
        for method in (session.hold, session.estop):
            with pytest.raises(RuntimeFault) as failed:
                method("unsupported physical control")
            assert failed.value.code is FaultCode.UNSUPPORTED
        with session.run(task="observe only", actor={"id": "test"}) as run:
            result = run.step_parts({"arm": JointPositionCommand([0.01, 0.0])})["arm"]
            assert not result.dispatched
            assert result.fault.code is FaultCode.UNSUPPORTED
        assert not site_fixtures.velocity_commands
    assert site_fixtures.closed == {"arms": 1, "cameras": 1}
    # Ordinary teardown released the site lock and permits a fresh owner.
    with passive_site.open(console=False) as session:
        assert "arm" in session.observe_parts(["arm"]).parts
    assert site_fixtures.closed == {"arms": 2, "cameras": 2}


@pytest.mark.parametrize(
    "flag", ["actuation_supported", "estop_supported", "close_releases_torque"]
)
@pytest.mark.parametrize("value", [None, 0, "false"])
def test_driver_effect_flags_reject_ambiguous_values(flag, value):
    driver = PassiveDriver()
    setattr(driver, flag, value)
    with pytest.raises(TypeError, match=flag):
        arm_for(driver)


def test_explicit_passive_driver_cannot_register_supervised_control():
    arm = arm_for(PassiveDriver())
    with pytest.raises(ValueError, match="actuation"):
        base.control({"arm": arm}, posture="supervised")


def test_optional_close_effect_preserves_legacy_hardware_wait():
    legacy = site_fixtures._Driver()
    legacy.kind = "unfamiliar-physical-driver"
    arm = arm_for(legacy)
    assert base.closing_drops_torque({"arm": arm})
    legacy.close_releases_torque = False
    assert not base.closing_drops_torque({"arm": arm})
    legacy.close_releases_torque = True
    assert base.closing_drops_torque({"arm": arm})


def test_failed_passive_startup_closes_devices_and_releases_ownership(
    passive_site, monkeypatch
):
    create = site_api.create_core_session
    original = RuntimeFault(FaultCode.TRANSPORT_LOST, "registration failed")

    def failed_registration(*args, **kwargs):
        raise original

    monkeypatch.setattr(site_api, "create_core_session", failed_registration)
    with pytest.raises(RuntimeFault) as failed, passive_site.open(console=False):
        pass
    assert failed.value is original
    assert site_fixtures.closed == {"arms": 1, "cameras": 1}
    monkeypatch.setattr(site_api, "create_core_session", create)
    with passive_site.open(console=False) as session:
        assert "arm" in session.observe_parts().parts
    assert site_fixtures.closed == {"arms": 2, "cameras": 2}


def test_monitor_without_optional_flags_retains_supported_local_actions():
    legacy = arm_for(site_fixtures._Driver())
    control = base.control({"arm": legacy}, posture="monitor")
    assert control.send is None and control.hold is None
    assert callable(control.estop)
    # A grant-free local policy still crosses the ordinary owner envelope.
    assert legacy.command([0.01, 0.0])
    legacy.step(0.05)
    assert legacy.state()[0].tolist() == pytest.approx([0.01, 0.0])


def test_mixed_stop_support_never_advertises_an_all_parts_stop():
    arms = {
        "observer": arm_for(PassiveDriver()),
        "legacy": arm_for(site_fixtures._Driver()),
    }
    assert base.control(arms, posture="monitor").estop is None
    with pytest.raises(ValueError, match="e-stop support"):
        base.control(arms, posture="monitor", estop_hardware=True)
