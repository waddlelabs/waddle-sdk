"""Optional force actuation retains the normal owner envelope and feedback truth."""

from types import SimpleNamespace

import pytest
from waddle_sdk.robots import base, yam_force
from waddle_sdk.robots.yam_force import ForceLimiter
from waddle_sdk.runtime import GripperForce, JointPositionCommand, RuntimeFault


def _twin():
    return base.SimDriver(
        [0, 0, 1],
        lower=[-1, -1.5, 0],
        upper=[1, 1.5, 1],
        step_caps=[0.1, 0.1, 0.25],
        rate_hz=20,
    )


def _arm(driver):
    return base.Arm(
        part="toy",
        driver=driver,
        joint_names=("lift", "swing", "grip"),
        joint_limits=((-1, 1), (-1.5, 1.5), (0, 1)),
        step_caps=(0.1, 0.1, 0.25),
        rate_hz=20,
    )


class ForceDriver(base.SimDriver):
    gripper_force_supported = True
    gripper_force_limits_n = (1, 50)

    def __init__(self):
        original = _twin()
        self.__dict__.update(original.__dict__)
        self.force = 0
        self.force_writes = []

    def read_gripper_force(self):
        return GripperForce(self.force, "test.sensor", False)

    def write_gripper_force(self, target, force_n, velocity=None):
        self.force_writes.append(force_n)
        self.force = force_n
        self.write(target)

    def hold(self):
        self.force = 0
        super().hold()


def test_force_write_is_checked_before_activation_and_arm_writes_preserve_mode():
    driver = ForceDriver()
    arm = _arm(driver)
    assert base.apply_decision(
        {"toy": arm}, {"toy": [0, 0, 0.8]}, gripper_force_n={"toy": 10}
    )
    assert driver.force == 10
    assert base.apply_decision({"toy": arm}, {"toy": [0.03, 0, 0.8]})
    assert driver.force == 10
    assert not base.apply_decision(
        {"toy": arm}, {"toy": [3, 0, 0]}, gripper_force_n={"toy": 20}
    )
    assert driver.force_writes == [10] and driver.force == 0
    with pytest.raises(RuntimeFault, match="force_range"):
        base.apply_decision(
            {"toy": arm}, {"toy": [0, 0, 0.8]}, gripper_force_n={"toy": 60}
        )


@pytest.mark.parametrize("force", [-1, float("nan"), float("inf"), True])
def test_invalid_force_never_constructs_an_action(force):
    with pytest.raises(ValueError):
        JointPositionCommand([0, 1], gripper_force_n=force)


def test_yam_bounds_demand_from_first_contact_and_follows_deformation():
    original = SimpleNamespace(update=lambda state: state["target_qpos"])
    limiter = ForceLimiter(original, kp=20, kd=0.5, stroke=-6.57, max_speed=2.5)
    limiter.set_force(10)
    state = {
        "current_qpos": 2.0,
        "target_qpos": 6.0,
        "current_qvel": 0.0,
        "current_eff": 0.0,
        "current_normalized_qpos": 0.04,  # inside the final 5%: force mode from the start
        "last_command_qpos": 6.0,
    }
    reference = limiter.update(state)
    assert 20 * (reference - state["current_qpos"]) == pytest.approx(10 * 0.096 / 6.57)
    state["current_eff"] = 10 * 0.096 / 6.57
    state["last_command_qpos"] = reference
    limiter.update(state)
    assert limiter.holding
    state.update(current_qpos=2.03, target_qpos=2.0)
    assert limiter.update(state) > state["current_qpos"]
    assert limiter.measurement(state["current_eff"]).force_n == pytest.approx(10)
    limiter.set_force(0)
    assert limiter.update(state) == 2.0


def test_yam_force_grasp_approaches_from_open_then_hands_off_at_contact(monkeypatch):
    clock = [0.0]
    monkeypatch.setattr(yam_force, "time", SimpleNamespace(monotonic=lambda: clock[0]))
    original = SimpleNamespace(update=lambda state: state["target_qpos"])
    limiter = ForceLimiter(original, kp=20, kd=0.5, stroke=-6.57, max_speed=2.5)
    limiter.set_force(36)
    state = {
        "current_qpos": 0.0,
        "target_qpos": 6.57,
        "current_qvel": 0.0,
        "current_eff": 0.0,
        "current_normalized_qpos": 1.0,
        "last_command_qpos": 0.0,
    }
    assert limiter.update(state) == 0.0
    clock[0] = 0.01
    fast = limiter.update(state)
    assert fast > 36 * 0.096 / 6.57 / 20
    assert limiter.approaching
    state["current_qpos"] = fast
    state["last_command_qpos"] = fast
    state["current_eff"] = 36 * 0.096 / 6.57
    state["current_qvel"] = 0.2  # normalized jaw/s, still 1.31 motor rad/s
    clock[0] = 0.02
    moving = limiter.update(state)
    assert limiter.approaching and moving > fast
    state["current_qpos"] = moving
    state["last_command_qpos"] = moving
    state["current_qvel"] = 0.0
    clock[0] = 0.03
    contact = limiter.update(state)
    assert not limiter.approaching
    assert contact - moving <= 36 * 0.096 / 6.57 / 20 + 1e-9

    limiter.set_force(0)
    limiter.set_force(36)
    state["current_normalized_qpos"] = 0.04
    state["current_eff"] = 0.0
    clock[0] = 0.04
    limiter.update(state)
    assert not limiter.approaching


def test_yam_force_grasp_approaches_fast_from_a_partly_open_unloaded_jaw(monkeypatch):
    # From 75 mm, force mode the whole way took 3.6 s of uneven travel on the rig,
    # against 1.4 s from fully open. Load, not opening, decides the approach.
    clock = [0.0]
    monkeypatch.setattr(yam_force, "time", SimpleNamespace(monotonic=lambda: clock[0]))
    original = SimpleNamespace(update=lambda state: state["target_qpos"])

    def start(effort_n):
        limiter = ForceLimiter(original, kp=20, kd=0.5, stroke=-6.57, max_speed=2.5)
        limiter.set_force(36)
        state = {
            "current_qpos": 1.6,
            "target_qpos": 6.57,
            "current_qvel": 0.0,
            "current_eff": effort_n * 0.096 / 6.57,
            "current_normalized_qpos": 0.75,
            "last_command_qpos": 1.6,
        }
        limiter.update(state)
        clock[0] += 0.01
        return limiter, limiter.update(state), state

    free, reference, state = start(0.0)
    assert free.approaching and reference - state["current_qpos"] > 36 * 0.096 / 6.57 / 20
    loaded, reference, state = start(20.0)
    assert not loaded.approaching
    assert reference - state["current_qpos"] <= 36 * 0.096 / 6.57 / 20 + 1e-9


def test_explicit_release_cannot_reactivate_vendor_fixed_force_hold():
    legacy = SimpleNamespace(update=lambda state: state["target_qpos"] + 1)
    limiter = ForceLimiter(legacy, kp=20, kd=0.5, stroke=-6.57, max_speed=2.5)
    state = {"target_qpos": 2}
    assert limiter.update(state) == 3
    limiter.set_force(10)
    limiter.set_force(0)
    assert limiter.update(state) == 2


def test_release_waits_for_its_reference_and_failed_write_discards_pending_mode():
    limiter = ForceLimiter(
        SimpleNamespace(update=lambda state: state["target_qpos"]),
        kp=20,
        kd=0.5,
        stroke=-6.57,
        max_speed=2.5,
    )
    limiter.set_force(10)
    state = {
        "current_qpos": 2.0,
        "target_qpos": 6.0,
        "current_qvel": 0.0,
        "current_eff": 0.0,
        "current_normalized_qpos": 0.04,  # force mode, not the free approach
        "last_command_qpos": 6.0,
    }
    limiter.set_force(0, target_qpos=2.0)
    assert limiter.update(state) < 2.01  # Old copied command stays force-bounded.
    assert limiter.force_n == 10
    state["target_qpos"] = 2.0
    assert limiter.update(state) == 2.0 and limiter.force_n == 0
    limiter.set_force(20, target_qpos=6.0)
    limiter.discard_pending()
    state["target_qpos"] = 6.0
    assert limiter.update(state) == 6.0 and limiter.force_n == 0


def test_release_preserves_vendor_limiter_for_later_position_closure():
    legacy = SimpleNamespace(update=lambda state: 2.03, clog_force_threshold=0.5)
    limiter = ForceLimiter(legacy, kp=20, kd=0.5, stroke=-6.57, max_speed=2.5)
    limiter.set_force(10)
    limiter.set_force(0)
    assert limiter.update({"current_qpos": 2.0, "target_qpos": 2.0}) == 2.0
    assert limiter.update({"current_qpos": 2.0, "target_qpos": 2.2}) == 2.03


def test_yam_position_references_blend_at_motor_rate_without_direct_goal_step(
    monkeypatch,
):
    clock = [0.0]
    monkeypatch.setattr(yam_force, "time", SimpleNamespace(monotonic=lambda: clock[0]))
    legacy = SimpleNamespace(update=lambda state: state["target_qpos"])
    limiter = ForceLimiter(legacy, kp=20, kd=0.5, stroke=-6.57, max_speed=2.5)
    state = {"target_qpos": 0.2, "last_command_qpos": 0.0}
    assert limiter.update(state) == 0.0
    for tick in range(1, 9):
        clock[0] = tick * 0.005
        state["last_command_qpos"] = limiter.update(state)
        assert state["last_command_qpos"] <= 0.2
    assert state["last_command_qpos"] == pytest.approx(0.2)

    # A single direct half-stroke goal is also slew-limited rather than
    # becoming the vendor's full position step on the next motor tick.
    clock[0] += 0.005
    state["target_qpos"] = 3.285
    prior = state["last_command_qpos"]
    state["last_command_qpos"] = limiter.update(state)
    clock[0] += 0.005
    limited = limiter.update(state)
    assert 0 < limited - state["last_command_qpos"] <= 6.57 * 0.005 + 1e-9
    assert limited - prior < 0.1

    # The vendor's contact/clog correction still takes effect immediately.
    legacy.update = lambda state: state["target_qpos"] - 0.1
    assert limiter.update(state) == pytest.approx(3.185)


def test_yam_stationary_force_hold_recovers_a_small_friction_shortfall(monkeypatch):
    clock = [0.0]
    monkeypatch.setattr(yam_force, "time", SimpleNamespace(monotonic=lambda: clock[0]))
    limiter = ForceLimiter(
        SimpleNamespace(update=lambda state: state["target_qpos"]),
        kp=20,
        kd=0.5,
        stroke=-6.57,
        max_speed=2.5,
    )
    limiter.set_force(22)
    state = {
        "current_qpos": 2.0,
        "target_qpos": 6.0,
        "current_qvel": 0.0,
        "current_eff": 0.0,
        "current_normalized_qpos": 0.25,
        "last_command_qpos": 2.0,
    }
    measured = 0.0
    for _ in range(200):
        reference = limiter.update(state)
        # Stationary plant with 2 N of transmission loss.
        measured = max(0.0, (reference - 2.0) * 20 * 6.57 / 0.096 - 2.0)
        state["last_command_qpos"] = reference
        state["current_eff"] = measured * 0.096 / 6.57
        clock[0] += 0.01
    assert 21.5 <= measured <= 22.5
    state["current_qvel"] = 0.2
    limiter.update(state)
    assert limiter._force_bias_n == 0.0
