"""The I2RT YAM robot module's moving parts: its live driver and its factories.

`test_yam_facts.py` gates the NUMBERS this module states against the vendor's
own model. This file gates what it BUILDS out of them:

* the declaration — pinned against the customer program that already runs at
  the rig, compiled to JSON and compared field for field, so "the module
  declares what the program declared" is a check rather than a claim;
* `yam.declaration()` standing alone, since a customer who wires
  `waddle.init` by hand must get the same robot the factory registers;
* the live driver's refusals — an absent vendor package, an arm that reports
  a different number of joints than this module declares, a command after an
  e-stop — driven against a stand-in vendor module, because the real one is
  not installed here and CI has no CAN bus;
* forward kinematics being OPT-IN: a rig built without it is legal and
  reports joint positions only, with no TCP pose invented from nowhere.

The site numbers below are the REFERENCE rig's (the workspace box, the
bench-measured gripper motor limits, the cross-arm mounting). They are typed
out here rather than imported so that this file is a second, independent
statement of what the customer program declares — which is the whole point of
the golden below.
"""

from __future__ import annotations

import json
import sys
import types

import numpy as np
import pytest
from mcap.reader import make_reader
from mcap_protobuf.decoder import DecoderFactory

import waddle
from waddle.robots import base, yam


@pytest.fixture(autouse=True)
def _clean_session():
    yield
    waddle.shutdown()


# --------------------------------------------------------------------------
# The reference rig's SITE facts (envelope.py's site block, verbatim)
# --------------------------------------------------------------------------

WORKSPACE_BOX_M = ((0.05, -0.45, 0.05), (0.60, 0.45, 0.70))
GRIPPER_LIMITS_MOTOR_RAD = (0.1, 1.7)
CROSS_ARM_XYZ = (0.0, -0.60, 0.0)
CROSS_ARM_RPY = (0.0, 0.0, -0.15)


def _cross_arm() -> base.CrossArm:
    return base.CrossArm(xyz=CROSS_ARM_XYZ, rpy=CROSS_ARM_RPY)


def _bimanual(**overrides) -> base.Rig:
    kwargs: dict = dict(
        workspace=WORKSPACE_BOX_M,
        gripper_limits=GRIPPER_LIMITS_MOTOR_RAD,
        cross_arm=_cross_arm(),
        sim=True,
    )
    kwargs.update(overrides)
    return yam.bimanual(**kwargs)


def _arm_rig(**overrides) -> base.Rig:
    kwargs: dict = dict(
        workspace=WORKSPACE_BOX_M,
        gripper_limits=GRIPPER_LIMITS_MOTOR_RAD,
        sim=True,
    )
    kwargs.update(overrides)
    return yam.arm(**kwargs)


def _grants(rig: base.Rig) -> list[dict]:
    return waddle._derive_grants(rig.control(rig.arms()), rig.robot().action_space)


def _observations(mcap_path):
    with open(mcap_path, "rb") as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        return [
            msg
            for _, channel, _, msg in reader.iter_decoded_messages()
            if channel.topic == "/waddle/observations"
        ]


# --------------------------------------------------------------------------
# The golden: what the customer program at the rig already declares
# --------------------------------------------------------------------------
#
# A verbatim replica of the bimanual-YAM customer program's declaration, with
# every number typed out rather than imported from `waddle.robots.yam`. Two
# independent statements of one declaration: a factory that stops producing
# this shape — a renamed part, a dropped chunking policy, a joint limit
# widened, an xyzw quaternion where a wxyz one belongs — fails here.

SUBSTRATE_JOINT_NAMES = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "gripper",
)

SUBSTRATE_JOINT_LIMITS = (
    (-2.61799, 3.05433),
    (0.0, 3.65),
    (0.0, 3.13),
    (-1.5708, 1.5708),
    (-1.5708, 1.5708),
    (-2.0944, 2.0944),
    (0.0, 1.0),
)

SUBSTRATE_RATE_HZ = 10.0
SUBSTRATE_MAX_JOINT_SPEED_RAD_S = 1.0
SUBSTRATE_MAX_JOINT_EFFORT_NM = 10.0
SUBSTRATE_LEFT_BASE_FRAME = "yam_left_base"
SUBSTRATE_RIGHT_BASE_FRAME = "yam_right_base"


def _substrate_part_space() -> waddle.JointSpace:
    return waddle.JointSpace(
        joints=[
            waddle.Joint(
                name=name,
                min_position=lo,
                max_position=hi,
                max_velocity=SUBSTRATE_MAX_JOINT_SPEED_RAD_S,
                max_effort=SUBSTRATE_MAX_JOINT_EFFORT_NM,
            )
            for name, (lo, hi) in zip(
                SUBSTRATE_JOINT_NAMES, SUBSTRATE_JOINT_LIMITS, strict=True
            )
        ],
        rate_hz=SUBSTRATE_RATE_HZ,
        chunking=waddle.Chunking(horizon=1, replan="immediate", interp="hold"),
    )


def _substrate_declaration() -> waddle.Robot:
    return waddle.Robot(
        name="waddle-yam-bimanual",
        robot_id="yam-bimanual-01",
        cell_id="yam-cell",
        action_space=waddle.Composite(
            left_arm=_substrate_part_space(),
            right_arm=_substrate_part_space(),
            rate_hz=SUBSTRATE_RATE_HZ,
            chunking=waddle.Chunking(horizon=1, replan="immediate", interp="hold"),
        ),
        frames=(
            waddle.FrameTransform(
                parent=SUBSTRATE_LEFT_BASE_FRAME,
                child=SUBSTRATE_RIGHT_BASE_FRAME,
                position=CROSS_ARM_XYZ,
                quaternion=base.quaternion_wxyz(base.rpy_matrix(*CROSS_ARM_RPY)),
            ),
        ),
    )


def _as_json(robot: waddle.Robot, grants: list[dict]) -> dict:
    return json.loads(json.dumps(robot._compile(grants)))


def test_the_factory_declares_what_the_customer_program_declares(tmp_path):
    """The golden. `yam.bimanual()` must compile to the byte-shape the
    program running at the reference rig already registers — same part names
    in the same order, same per-joint limits, same rate and chunking, same
    cross-arm edge with its rpy converted to a **wxyz** quaternion."""
    rig = _bimanual(
        name="waddle-yam-bimanual",
        robot_id="yam-bimanual-01",
        cell_id="yam-cell",
    )
    grants = _grants(rig)
    assert _as_json(rig.robot(), grants) == _as_json(_substrate_declaration(), grants)


def test_the_standalone_declaration_is_the_one_the_rig_registers():
    """THE AMENDMENT, item 1: `yam.declaration()` is a first-class product.

    A customer who wants none of the rest of this module — their own driver,
    their own loop, a plain `waddle.init` — still gets exactly the robot the
    factory would have registered, or the two would drift and the hand-wired
    program would be the one that broke."""
    rig = _bimanual(
        name="waddle-yam-bimanual",
        robot_id="yam-bimanual-01",
        cell_id="yam-cell",
    )
    hand_wired = yam.declaration(
        parts=("left_arm", "right_arm"),
        name="waddle-yam-bimanual",
        robot_id="yam-bimanual-01",
        cell_id="yam-cell",
        frames=(
            _cross_arm().transform(
                SUBSTRATE_LEFT_BASE_FRAME, SUBSTRATE_RIGHT_BASE_FRAME
            ),
        ),
    )
    grants = _grants(rig)
    assert _as_json(hand_wired, grants) == _as_json(rig.robot(), grants)


def test_declaration_order_is_the_action_layout():
    """Declaration order IS the layout of the concatenated action vector, so
    it is pinned: left rows 0..6, right rows 7..13, everywhere, for
    everyone."""
    space = _bimanual().robot()._compile([])["actionSpace"]
    parts = space["composite"]["parts"]
    assert [p["name"] for p in parts] == ["left_arm", "right_arm"]
    assert all(
        len(p["space"]["jointPosition"]["joints"]) == yam.JOINT_COUNT for p in parts
    )


def test_a_rig_with_no_cross_arm_edge_declares_none():
    """No transform, no declaration — never an identity nobody measured. A
    cross-arm pose then refuses downstream instead of resolving through a
    guess."""
    rig = _bimanual(cross_arm=None)
    assert "frames" not in rig.robot()._compile([])


def test_one_arm_declares_the_shipped_model_and_two_arms_decline_to():
    """A `kinematics_urdf` field describes ONE chain. A single YAM carries the
    shipped model; a bimanual rig cannot, because naming one arm's chain would
    name the other arm's tool frame as something it is not."""
    import base64

    single = _arm_rig().robot()._compile([])
    assert base64.b64decode(single["kinematicsUrdf"]).decode() == yam.urdf_text()
    assert "kinematicsUrdf" not in _bimanual().robot()._compile([])


def test_a_declared_model_is_tied_to_the_frame_the_poses_are_reported_in():
    """The shipped model's root link is `base_link`; the arm reports its TCP
    in the site's own frame name. Declaring the model without saying they are
    the same frame would leave a consumer two unrelated trees, so the rename
    is declared as the identity edge it is."""
    frames = _arm_rig().robot()._compile([])["frames"]["transforms"]
    assert [(f["parent"], f["child"]) for f in frames] == [
        ("yam_base", yam.URDF_BASE_LINK)
    ]
    rotation = frames[0]["transform"]["rotation"]
    assert (rotation["w"], rotation.get("x", 0.0)) == (1.0, 0.0)


def test_a_declared_model_needs_no_edge_when_the_names_already_agree():
    rig = _arm_rig(base_frame=yam.URDF_BASE_LINK)
    assert "frames" not in rig.robot()._compile([])


def test_a_second_chain_may_not_carry_the_one_chain_model():
    with pytest.raises(ValueError, match="one chain"):
        yam.declaration(parts=("left_arm", "right_arm"), declare_urdf=True)


# --------------------------------------------------------------------------
# What the factories refuse
# --------------------------------------------------------------------------


def test_a_live_rig_without_a_channel_is_refused_by_argument_name():
    with pytest.raises(ValueError, match="channel"):
        _bimanual(sim=False, left=yam.ArmSite(channel="can_left"))


def test_a_single_live_arm_without_a_channel_is_refused_by_argument_name():
    with pytest.raises(ValueError, match="channel"):
        _arm_rig(sim=False)


@pytest.mark.parametrize(
    "limits",
    [(1.7, 0.1), (0.5, 0.5), (0.1,), (0.1, 1.7, 2.0), (0.1, float("nan")), 0.5],
)
def test_a_malformed_gripper_limit_pair_is_refused_even_in_sim(limits):
    """Required in sim too, so the program text is identical across the
    sim->live flip: the shape is validated either way and only the live
    driver reads the values."""
    with pytest.raises(ValueError, match="gripper_limits"):
        _bimanual(gripper_limits=limits)


@pytest.mark.parametrize(
    "box",
    [
        ((0.6, -0.45, 0.05), (0.05, 0.45, 0.70)),  # a minimum above its maximum
        ((0.05, -0.45), (0.60, 0.45)),  # a corner that is not a point
        (0.05, 0.60),  # not corners at all
    ],
)
def test_a_malformed_workspace_box_is_refused(box):
    with pytest.raises(ValueError, match="workspace"):
        _bimanual(workspace=box)


def test_an_unknown_posture_is_refused_by_name():
    with pytest.raises(ValueError, match="posture"):
        _bimanual(posture="observe")


def test_a_factory_call_opens_nothing():
    """Declaration is cheap: constructing a live rig touches no CAN bus, so a
    program can be built and then decide not to run. `arms()` is where the
    hardware opens and where a failure to open it lands."""
    rig = _bimanual(
        sim=False,
        left=yam.ArmSite(channel="can_left"),
        right=yam.ArmSite(channel="can_right"),
    )
    assert rig.robot().name == "yam-bimanual"
    with pytest.raises(RuntimeError, match="i2rt"):
        rig.arms()


# --------------------------------------------------------------------------
# The posture, at the factory
# --------------------------------------------------------------------------


def test_a_monitor_rig_registers_only_the_owners_stop():
    """THE AMENDMENT, item 6: the posture is chosen at construction and maps
    to verb presence only — no authority logic anywhere near it."""
    rig = _bimanual(posture="monitor")
    verbs = rig.control(rig.arms())
    assert verbs.send is None and verbs.hold is None
    assert waddle._derive_grants(verbs, rig.robot().action_space) == [
        {"verb": "VERB_ESTOP"}
    ]


def test_a_supervised_rig_registers_the_three_driving_verbs():
    rig = _bimanual()
    verbs = rig.control(rig.arms())
    assert verbs.send is not None and verbs.hold is not None and verbs.estop is not None


# --------------------------------------------------------------------------
# The twins, through the envelope
# --------------------------------------------------------------------------


def test_the_sim_factory_drives_its_twins_through_the_envelope(tmp_path):
    """A sim rig is a rehearsal of the live one: the same declaration, the
    same envelope arithmetic, the same seam. The step cap is DERIVED from the
    declared speed and rate (1.0 rad/s at 10 Hz = 0.10 rad per command), which
    is what makes a jump refusable at all."""
    rig = _bimanual()
    arms = rig.arms()
    assert set(arms) == {"left_arm", "right_arm"}

    waddle.init("yam-sim-smoke", rig.robot(), rig.control(arms), recording_dir=tmp_path)
    with waddle.rollout(task="nudge both arms") as ep:
        position = np.concatenate(
            [arms[p].state()[0] for p in ("left_arm", "right_arm")]
        )
        action = position + 0.01
        decided = ep.gate(action, position)
        assert decided is not None
        base.apply_decision(arms, decided)
        ep.terminate("success")

    assert arms["left_arm"].accepted == 1 and arms["left_arm"].rejected == 0
    assert arms["right_arm"].accepted == 1

    lines: list[str] = []
    arms["left_arm"].report = lines.append
    arms["left_arm"]._reject._report = lines.append
    jump = arms["left_arm"].state()[0] + 0.5
    assert arms["left_arm"].command(jump) is False
    assert arms["left_arm"].rejected == 1
    assert any("would move" in line for line in lines)


def test_the_twins_start_on_distinct_rows():
    """Two twins that started identical would be told apart only by name,
    which is the one thing a bimanual index-map bug hides behind."""
    arms = _bimanual().arms()
    left = arms["left_arm"].state()[0]
    right = arms["right_arm"].state()[0]
    assert not np.allclose(left, right)


def test_a_site_may_place_its_own_twin_home():
    rig = _bimanual(left=yam.ArmSite(sim_home=(0.0,) * yam.JOINT_COUNT))
    assert np.allclose(rig.arms()["left_arm"].state()[0], np.zeros(yam.JOINT_COUNT))


# --------------------------------------------------------------------------
# Forward kinematics: opt-in, and what opting out costs
# --------------------------------------------------------------------------


def _report_one_tick(rig: base.Rig, tmp_path, project: str):
    """Open a session, report every part once, and hand back the recorded
    proprio samples. No pump and no thread — the tick is called directly, so
    nothing here waits on a clock."""
    arms = rig.arms()
    session = waddle.init(
        project, rig.robot(), rig.control(arms), recording_dir=tmp_path
    )
    tick = base.proprio_tick(session, arms)
    with waddle.rollout(task="report once") as ep:
        episode_id = ep.id
        tick(1.0 / rig.rate_hz)
        ep.terminate("success")
    waddle.shutdown()
    return [o.proprio for o in _observations(tmp_path / f"{episode_id}.mcap")]


def test_a_rig_reports_the_tcp_the_declared_chain_produces(tmp_path):
    rig = _bimanual()
    samples = _report_one_tick(rig, tmp_path, "yam-fk")
    by_part = {s.part: s for s in samples if len(s.joint_pos) == yam.JOINT_COUNT}
    assert by_part.keys() == {"left_arm", "right_arm"}
    for part, frame in (("left_arm", "yam_left_base"), ("right_arm", "yam_right_base")):
        sample = by_part[part]
        assert sample.HasField("ee_pose")
        assert sample.ee_pose.frame_id == frame
        expected, _ = yam.forward_kinematics(
            list(sample.joint_pos)[: yam.ARM_JOINT_COUNT]
        )
        got = sample.ee_pose.position
        assert np.allclose([got.x, got.y, got.z], expected)


def test_a_rig_built_without_forward_kinematics_reports_joint_positions_only(tmp_path):
    """THE AMENDMENT, item 2: forward kinematics is OPT-IN. A rig handed none
    is legal — it reports joint positions and velocities, and the features
    that want a TCP degrade by NAME rather than by a pose invented from
    nowhere."""
    rig = _bimanual(fk=None, workspace=None)
    samples = _report_one_tick(rig, tmp_path, "yam-no-fk")
    reported = [s for s in samples if len(s.joint_pos) == yam.JOINT_COUNT]
    assert {s.part for s in reported} == {"left_arm", "right_arm"}
    assert not any(s.HasField("ee_pose") for s in reported)


def test_a_workspace_box_without_forward_kinematics_is_refused():
    """A box is a statement about a TCP, and a rig with no kinematics has
    none. Refused at construction rather than silently unenforced."""
    with pytest.raises(ValueError, match="workspace"):
        _bimanual(fk=None).arms()


# --------------------------------------------------------------------------
# The live driver, against a stand-in vendor package
# --------------------------------------------------------------------------


class _FakeYamRobot:
    """The four vendor calls this driver makes, and nothing else."""

    def __init__(self, *, dofs: int = 7, info: dict | None = None) -> None:
        self.dofs = dofs
        self.info = {"kp": [10.0] * 7, "kd": [1.0] * 7} if info is None else info
        self.commands: list[np.ndarray] = []
        self.gains: list[tuple] = []
        self.zeroed = 0
        self.closed = 0
        self.observations = {
            "joint_pos": [0.1] * 6,
            "joint_vel": [0.0] * 6,
            "gripper_pos": [0.5],
        }
        self.zero_torque_raises = False

    def num_dofs(self) -> int:
        return self.dofs

    def get_robot_info(self) -> dict:
        return self.info

    def get_observations(self) -> dict:
        return self.observations

    def command_joint_pos(self, values) -> None:
        self.commands.append(np.asarray(values, dtype=float))

    def zero_torque_mode(self) -> None:
        self.zeroed += 1
        if self.zero_torque_raises:
            raise RuntimeError("the bus write timed out")

    def update_kp_kd(self, kp, kd) -> None:
        self.gains.append((kp, kd))

    def close(self) -> None:
        self.closed += 1


class _FakeVendor:
    def __init__(self) -> None:
        self.calls: list[dict] = []
        self.robots: list[_FakeYamRobot] = []
        self.next_kwargs: dict = {}

    def get_yam_robot(self, **kwargs) -> _FakeYamRobot:
        self.calls.append(kwargs)
        robot = _FakeYamRobot(**self.next_kwargs)
        self.robots.append(robot)
        return robot


@pytest.fixture
def vendor(monkeypatch) -> _FakeVendor:
    """A stand-in for the vendor package, installed under its real names.

    The real one is a direct git dependency that is not installed here and
    needs a CAN bus to do anything; what this proves is the shape of the calls
    this driver makes and the refusals it raises around them."""
    fake = _FakeVendor()
    i2rt = types.ModuleType("i2rt")
    robots = types.ModuleType("i2rt.robots")
    get_robot = types.ModuleType("i2rt.robots.get_robot")
    utils = types.ModuleType("i2rt.robots.utils")
    get_robot.get_yam_robot = fake.get_yam_robot

    class GripperType:
        LINEAR_4310 = "LINEAR_4310"

    utils.GripperType = GripperType
    for name, module in (
        ("i2rt", i2rt),
        ("i2rt.robots", robots),
        ("i2rt.robots.get_robot", get_robot),
        ("i2rt.robots.utils", utils),
    ):
        monkeypatch.setitem(sys.modules, name, module)
    return fake


def _live(vendor: _FakeVendor, **overrides) -> yam.LiveDriver:
    kwargs: dict = dict(
        channel="can_left",
        gripper_limits=GRIPPER_LIMITS_MOTOR_RAD,
        report=lambda _: None,
    )
    kwargs.update(overrides)
    return yam.LiveDriver(**kwargs)


@pytest.mark.skipif(
    "i2rt" in sys.modules, reason="the real vendor package is importable here"
)
def test_a_missing_vendor_package_names_the_command_that_installs_it():
    """The import is lazy — inside `__init__`, so importing this module on a
    machine with no vendor package is fine — and the failure carries the exact
    command, pinned to the same commit every fact in this module is."""
    try:
        import i2rt  # noqa: F401
    except ImportError:
        pass
    else:
        pytest.skip("the real vendor package is installed")
    with pytest.raises(RuntimeError) as excinfo:
        yam.LiveDriver(channel="can_left", gripper_limits=GRIPPER_LIMITS_MOTOR_RAD)
    message = str(excinfo.value)
    assert (
        'pip install "i2rt @ git+https://github.com/i2rt-robotics/i2rt@'
        f'{yam.I2RT_PIN}"' in message
    )


def test_the_live_driver_pins_the_gripper_range_instead_of_calibrating(vendor):
    """Constructing with no override runs a physical auto-calibration that
    DRIVES THE JAWS on every connect. This module never auto-ranges a hand:
    the bench-measured pair is passed every time."""
    driver = _live(vendor)
    assert isinstance(driver, base.Driver)
    (call,) = vendor.calls
    assert call["channel"] == "can_left"
    assert call["zero_gravity_mode"] is False
    assert np.allclose(call["gripper_limits_override"], GRIPPER_LIMITS_MOTOR_RAD)


def test_an_arm_that_reports_other_joints_is_refused_not_adapted_to(vendor):
    vendor.next_kwargs = {"dofs": 6}
    with pytest.raises(RuntimeError, match="6 DOF"):
        _live(vendor)


def test_the_live_driver_reads_the_hand_as_the_seventh_row(vendor):
    driver = _live(vendor)
    position, velocity = driver.read()
    assert position.shape == (yam.JOINT_COUNT,)
    assert position[yam.ARM_JOINT_COUNT] == 0.5
    assert velocity.shape == (yam.JOINT_COUNT,)


def test_an_absent_velocity_reads_as_zero_and_an_absent_position_is_a_fault(vendor):
    """The wire has no "unknown" for a velocity, so an absent one is reported
    as zero; an absent POSITION is a fault, because guessing one would put a
    pose nobody measured into the record."""
    driver = _live(vendor)
    vendor.robots[0].observations = {"joint_pos": [0.1] * 6}
    position, velocity = driver.read()
    assert np.allclose(velocity, 0.0)
    assert position[yam.ARM_JOINT_COUNT] == 0.0
    vendor.robots[0].observations = {}
    with pytest.raises(RuntimeError, match="joint_pos"):
        driver.read()


def test_the_estop_latches_before_the_vendor_call(vendor):
    """A stop that half-happened is still a stop, and the one thing that must
    not follow it is a program that believes it can drive again."""
    driver = _live(vendor)
    vendor.robots[0].zero_torque_raises = True
    with pytest.raises(RuntimeError, match="timed out"):
        driver.estop()
    assert driver.estopped is True
    with pytest.raises(RuntimeError, match="e-stopped"):
        driver.write(np.zeros(yam.JOINT_COUNT))


def test_re_enable_restores_the_snapshotted_gains_and_holds_the_measured_pose(vendor):
    driver = _live(vendor)
    driver.estop()
    driver.re_enable()
    robot = vendor.robots[0]
    assert robot.gains == [([10.0] * 7, [1.0] * 7)]
    assert np.allclose(robot.commands[-1][: yam.ARM_JOINT_COUNT], 0.1)
    assert driver.estopped is False


def test_re_enable_refuses_to_guess_gains_it_never_snapshotted(vendor):
    """A made-up kp is how a demo arm slams. Refusing leaves the latch set and
    the arm floating, which is the state the site operator can already see."""
    vendor.next_kwargs = {"info": {}}
    driver = _live(vendor)
    driver.estop()
    with pytest.raises(RuntimeError, match="refusing to guess"):
        driver.re_enable()
    assert driver.estopped is True


def test_a_zero_gravity_driver_commands_nothing(vendor):
    """`posture="monitor"` builds the arm compliant, and this driver then
    refuses to write at all — so "nothing can command it" is a property of the
    object rather than of a flag somebody remembered to check."""
    driver = _live(vendor, zero_gravity=True)
    assert vendor.calls[0]["zero_gravity_mode"] is True
    with pytest.raises(RuntimeError, match="zero-gravity"):
        driver.write(np.zeros(yam.JOINT_COUNT))


def test_a_live_arm_has_no_home_and_integrates_itself(vendor):
    driver = _live(vendor)
    assert driver.home([0.0] * yam.JOINT_COUNT) is False
    assert driver.step(0.1) is None
    driver.close()
    assert vendor.robots[0].closed == 1


def test_the_live_factory_opens_the_channels_it_was_given(vendor):
    rig = _bimanual(
        sim=False,
        left=yam.ArmSite(channel="can_left"),
        right=yam.ArmSite(channel="can_right"),
    )
    arms = rig.arms()
    assert [call["channel"] for call in vendor.calls] == ["can_left", "can_right"]
    assert base.closing_drops_torque(arms) is True


def test_a_monitor_posture_opens_the_arms_compliant(vendor):
    """THE AMENDMENT, item 6: monitor is no send verb AND zero gravity where
    the driver supports it — one construction choice, no new authority."""
    rig = _bimanual(
        sim=False,
        posture="monitor",
        left=yam.ArmSite(channel="can_left"),
        right=yam.ArmSite(channel="can_right"),
    )
    rig.arms()
    assert [call["zero_gravity_mode"] for call in vendor.calls] == [True, True]


def test_a_site_may_measure_one_hand_differently_from_the_other(vendor):
    """Gripper limits are PER UNIT — a different pair of arms is re-measured
    at the bench — so an arm may carry its own."""
    rig = _bimanual(
        sim=False,
        left=yam.ArmSite(channel="can_left", gripper_limits=(0.2, 1.5)),
        right=yam.ArmSite(channel="can_right"),
    )
    rig.arms()
    assert np.allclose(vendor.calls[0]["gripper_limits_override"], (0.2, 1.5))
    assert np.allclose(
        vendor.calls[1]["gripper_limits_override"], GRIPPER_LIMITS_MOTOR_RAD
    )
