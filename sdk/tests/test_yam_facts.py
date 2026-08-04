"""The fact gate for `waddle.robots.yam`: every number, against its source.

A robot module is a pile of numbers somebody transcribed. Transcription is
where robot code goes quietly wrong — a limit widened by a digit, a chain
origin taken from a different hardware revision, a tool frame stated in the
flange's frame — and none of it announces itself: the arm executes the
nonsense faithfully.

So the numbers are not trusted here either. `yam.py` writes them down once,
the shipped URDF (`waddle/robots/yam_data/yam.urdf`, the pinned vendor
snapshot) states them again in the vendor's own words, and this file compares
the two. It is the open-side twin of the gate the closed repo runs against
the same URDF, and it is **directional** in exactly the same way:

* a POSITION LIMIT may be TIGHTER than the model and never looser — the
  declared interval must sit inside the URDF's, so tightening is always
  allowed and loosening past the hardware is what fails;
* an EFFORT ceiling, likewise, `<=`;
* every other fact — chain origins, rpys, axes, the tool frame — is not an
  interval and must MATCH, to a nanometre/nanoradian (which is float dust
  from the CAD export, four orders below the smallest real number in the
  table and four above the largest dust term).

What this gate cannot check, it names rather than glossing: the arm limits
are the URDF ∧ MJCF intersection, and the MJCF (MuJoCo Menagerie `i2rt_yam`)
is not shipped here, so the tightenings it contributes carry provenance
comments in `yam.py` and are cross-checked by the closed repo's gate. The one
tightening that IS visible from here — joint1's upper — is asserted below, so
the intersection cannot silently become "whatever the URDF says".
"""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from importlib.resources import files

import numpy as np
import pytest

from waddle import descriptors
from waddle.robots import base, yam

# A nanometre / nanoradian. The CAD export leaves dust as large as 1.2e-13 in
# the URDF (`joint4`'s rpy pitch), and the smallest number that means anything
# in these tables is 2.4e-05 (`joint5`'s origin z) — so this threshold sits
# ~10^4 clear of both, and no edit a human could make to a fact survives it.
EXACT = 1e-9

DATA = files("waddle.robots") / "yam_data"

#: Pointers that resolve only inside the repository this snapshot was patched
#: in. `test_the_shipped_data_points_only_at_what_a_wheel_holder_can_open`
#: refuses both classes; the data README's "patch 4" is the same rule applied
#: by hand, and this is what keeps the next re-vendor to it.
INTERNAL_POINTERS = (
    # a label from a task tracker nobody outside that repo can read
    re.compile(r"\bTask [A-Z]\d\b"),
    # a source path under it — the wheel ships no `conformance/` tree, and
    # neither does the repository this wheel is built from
    re.compile(r"\bconformance/\S+\.py\b"),
)


def _urdf_joints() -> dict[str, ET.Element]:
    """Every `<joint>` in the shipped URDF, by name."""
    root = ET.fromstring(yam.urdf_text())
    return {j.attrib["name"]: j for j in root.iter("joint")}


def _floats(element: ET.Element, tag: str, attribute: str) -> tuple[float, ...]:
    child = element.find(tag)
    assert child is not None, f"{element.attrib['name']} has no <{tag}>"
    return tuple(float(v) for v in child.attrib[attribute].split())


def _chain_from_urdf() -> list[ET.Element]:
    """Walk the URDF from `URDF_BASE_LINK` to `URDF_TCP_FRAME`, in order.

    Resolving the chain by topology rather than by document order is the
    point: the URDF lists its joints backwards (joint6 first), so a test that
    read them in file order would agree with a constants table that had been
    written backwards too.
    """
    by_parent = {
        j.find("parent").attrib["link"]: j  # type: ignore[union-attr]
        for j in _urdf_joints().values()
    }
    chain: list[ET.Element] = []
    link = yam.URDF_BASE_LINK
    while link in by_parent:
        joint = by_parent[link]
        chain.append(joint)
        link = joint.find("child").attrib["link"]  # type: ignore[union-attr]
    assert link == yam.URDF_TCP_FRAME, (
        f"the chain from {yam.URDF_BASE_LINK} ends at {link}, not at the "
        f"declared TCP frame {yam.URDF_TCP_FRAME}"
    )
    return chain


# ---------------------------------------------------------------------------
# The shipped data: what it is, where it came from, what it is not
# ---------------------------------------------------------------------------


def test_the_shipped_urdf_is_the_vendor_snapshot_the_module_pins():
    text = yam.urdf_text()
    assert yam.I2RT_PIN in text, (
        "the shipped URDF must carry the commit it was vendored from — the "
        "pin is the only thing that makes 'this is what the vendor said' "
        "checkable"
    )
    assert "github.com/i2rt-robotics/i2rt" in text
    assert ET.fromstring(text).tag == "robot"
    assert yam.I2RT_PIN in (DATA / "README.md").read_text(encoding="utf-8")


def test_the_vendors_licence_ships_beside_the_model():
    licence = (DATA / "LICENSE").read_text(encoding="utf-8")
    assert "MIT License" in licence
    assert "Permission is hereby granted, free of charge" in licence
    assert "I2RT" in licence
    readme = (DATA / "README.md").read_text(encoding="utf-8")
    assert "MIT" in readme and "LICENSE" in readme


def test_the_model_ships_as_text_with_no_meshes():
    """The wheel carries the kinematic contract, not a visual model.

    The URDF's `<mesh filename="assets/...">` references are deliberately
    unresolved: the STLs are ~megabytes against a 3.7 MB wheel, and nothing
    in this SDK renders anything. A future change that starts shipping them
    fails here, where the size decision is written down.
    """
    assert sorted(p.name for p in DATA.iterdir()) == [
        "LICENSE",
        "README.md",
        "yam.urdf",
    ]
    readme = (DATA / "README.md").read_text(encoding="utf-8")
    assert "mesh" in readme.lower(), (
        "the data README must say the meshes are absent — an unresolved "
        "mesh reference is otherwise read as a broken file"
    )


def test_the_shipped_data_points_only_at_what_a_wheel_holder_can_open():
    """Everything in this directory ships, comments included.

    A vendored file is read by somebody who has the wheel and the public
    repo and nothing else: the vendor's repository, the pinned commit, the
    files beside it and this SDK's own modules all resolve for them. A task
    label or a source path from the repository the snapshot was patched in
    resolves for nobody, and comments in vendored data are exactly where
    such a pointer survives, because nothing else ever reads them.
    """
    for path in DATA.iterdir():
        text = path.read_text(encoding="utf-8")
        for pattern in INTERNAL_POINTERS:
            found = pattern.search(text)
            assert found is None, (
                f"{path.name} names {found.group(0)!r}, which resolves only "
                f"inside the repository this snapshot was patched in — say "
                f"WHAT the patch was, never where its ticket or its test lives"
            )


# ---------------------------------------------------------------------------
# Names, arity and layout
# ---------------------------------------------------------------------------


def test_a_parts_row_layout_is_the_arm_joints_then_the_gripper():
    assert yam.JOINT_NAMES == yam.ARM_JOINT_NAMES + (yam.GRIPPER_JOINT_NAME,)
    assert yam.JOINT_LIMITS == yam.ARM_JOINT_LIMITS_RAD + (
        yam.GRIPPER_JOINT_LIMITS,
    )
    assert yam.ARM_JOINT_COUNT == len(yam.ARM_JOINT_NAMES) == 6
    assert yam.JOINT_COUNT == len(yam.JOINT_NAMES) == 7
    assert len(yam.ARM_JOINT_LIMITS_RAD) == yam.ARM_JOINT_COUNT


def test_the_declared_arm_joints_are_the_urdfs_chain_in_its_own_order():
    assert tuple(j.attrib["name"] for j in _chain_from_urdf()[:-1]) == (
        yam.ARM_JOINT_NAMES
    )


def test_the_gripper_row_is_not_a_joint_the_urdf_knows_about():
    """The seventh row is real, and the URDF has never heard of it.

    The vendored model stops at the six arm joints plus the fixed TCP frame —
    it carries no finger geometry at all — while `command_joint_pos` takes a
    seventh element. Stating that here keeps the next reader from looking for
    a `gripper` joint in the URDF and concluding the table is wrong.
    """
    joints = _urdf_joints()
    assert yam.GRIPPER_JOINT_NAME not in joints
    revolute = [n for n, j in joints.items() if j.attrib["type"] == "revolute"]
    fixed = [n for n, j in joints.items() if j.attrib["type"] == "fixed"]
    assert sorted(revolute) == sorted(yam.ARM_JOINT_NAMES)
    assert fixed == ["grasp_joint"]


# ---------------------------------------------------------------------------
# Limits — directional: tighter is always allowed, looser is the failure
# ---------------------------------------------------------------------------


def test_every_declared_limit_sits_inside_the_urdfs_own_limit():
    joints = _urdf_joints()
    for name, (lower, upper) in zip(
        yam.ARM_JOINT_NAMES, yam.ARM_JOINT_LIMITS_RAD, strict=True
    ):
        limit = joints[name].find("limit")
        assert limit is not None
        model_lower = float(limit.attrib["lower"])
        model_upper = float(limit.attrib["upper"])
        assert lower >= model_lower, (
            f"{name}: declared lower {lower} is BELOW the model's "
            f"{model_lower} — a declared limit may only ever be tighter"
        )
        assert upper <= model_upper, (
            f"{name}: declared upper {upper} is ABOVE the model's "
            f"{model_upper} — a declared limit may only ever be tighter"
        )


def test_joint1s_upper_is_tighter_than_the_urdf_because_the_mjcf_says_so():
    """The one place the MJCF half of the intersection is visible from here.

    `ARM_JOINT_LIMITS_RAD` is the URDF ∧ MJCF intersection, and the MJCF is
    not shipped in this wheel — so most of what it contributes cannot be
    machine-checked publicly (it is cross-checked by the closed repo's gate
    against the same two models). This one can: the MuJoCo Menagerie
    `i2rt_yam` model ranges `joint1` to 3.05433 where the URDF allows 3.13,
    and the declared table takes the smaller. If someone ever "fixes" the
    table to match the URDF, the containment test above would still pass and
    this one would not.
    """
    urdf_upper = float(_urdf_joints()["joint1"].find("limit").attrib["upper"])
    declared_upper = yam.ARM_JOINT_LIMITS_RAD[0][1]
    assert declared_upper < urdf_upper
    assert declared_upper == pytest.approx(3.05433, abs=EXACT)


def test_the_declared_effort_ceiling_never_exceeds_the_models():
    for name in yam.ARM_JOINT_NAMES:
        limit = _urdf_joints()[name].find("limit")
        assert yam.MAX_JOINT_EFFORT_NM <= float(limit.attrib["effort"])


def test_the_gripper_row_is_the_vendors_normalized_range():
    assert yam.GRIPPER_JOINT_LIMITS == (0.0, 1.0)


# ---------------------------------------------------------------------------
# The chain — not intervals, so these must match
# ---------------------------------------------------------------------------


def test_the_chain_origins_are_the_urdfs_own():
    for joint, xyz, rpy in zip(
        _chain_from_urdf()[:-1],
        yam.CHAIN_ORIGIN_XYZ_M,
        yam.CHAIN_ORIGIN_RPY_RAD,
        strict=True,
    ):
        name = joint.attrib["name"]
        assert _floats(joint, "origin", "xyz") == pytest.approx(
            xyz, abs=EXACT
        ), f"{name}: origin xyz"
        assert _floats(joint, "origin", "rpy") == pytest.approx(
            rpy, abs=EXACT
        ), f"{name}: origin rpy"


def test_every_arm_joint_turns_about_the_one_declared_axis():
    for joint in _chain_from_urdf()[:-1]:
        assert _floats(joint, "axis", "xyz") == pytest.approx(
            yam.CHAIN_AXIS, abs=EXACT
        ), f"{joint.attrib['name']}: axis"


def test_the_tool_frame_is_the_urdfs_fixed_grasp_joint():
    """The TCP is a tool fact, and it is stated in the tool's frame.

    `link_6` (the flange) sits 90° from `grasp_link`, which is what every
    consumer of a YAM pose speaks. Three orientation bugs on this arm came
    from stating a tool fact in the flange's frame, so the frame NAMES are
    asserted here beside the numbers.
    """
    tool = _chain_from_urdf()[-1]
    assert tool.attrib["type"] == "fixed"
    assert tool.find("child").attrib["link"] == yam.URDF_TCP_FRAME
    assert _floats(tool, "origin", "xyz") == pytest.approx(
        yam.TOOL_ORIGIN_XYZ_M, abs=EXACT
    )
    assert _floats(tool, "origin", "rpy") == pytest.approx(
        yam.TOOL_ORIGIN_RPY_RAD, abs=EXACT
    )


# ---------------------------------------------------------------------------
# The gripper's physical stroke — re-derived from the PINNED vendor
# ---------------------------------------------------------------------------


def test_the_stroke_is_two_jaws_of_the_pinned_vendors_own_travel():
    """0.095 m, and the arithmetic that produces it, pinned together.

    The pinned i2rt tree models this hand (`linear_4310.xml`) as two
    equality-coupled slide joints, each ranged `0 0.0475` along exactly
    opposed axes, so the jaw separation moves 2 × 0.0475 m end to end. The
    same tree's `linear_4310.yml` declares `gripper_stroke: 0.096`, so this
    figure is 1 mm conservative against the vendor's own number rather than
    equal to it by accident.

    Pinning the ARITHMETIC and not just the value is what makes the number
    un-editable without editing its derivation: a hand that changes must
    change `0.0475`, which is the thing a re-vendor can check.
    """
    assert yam.GRIPPER_MAX_OPENING_M == 2 * 0.0475
    assert yam.GRIPPER_MAX_OPENING_M <= 0.096, (
        "the vendor's own declared gripper_stroke is the ceiling this "
        "derivation may not exceed"
    )
    assert yam.GRIPPER_MAX_OPENING_M > 0.075, (
        "0.075 is the RETIRED figure, derived from the pre-pin Menagerie "
        "MJCF (2 × 0.037524, i2rt commit d4efb66) — one hardware revision "
        "behind this module's pin"
    )


# ---------------------------------------------------------------------------
# Forward kinematics — the constants, walked
# ---------------------------------------------------------------------------


def test_forward_kinematics_walks_the_shipped_urdfs_chain():
    """The FK a customer gets is the FK the shipped model describes.

    The element-wise tests above compare tables; this one compares the thing
    the tables are FOR, by walking the URDF's own numbers through the same
    generic chain helper and landing in the same place. A transposed row or a
    swapped pair that happened to survive an element-wise pass would move the
    tool here.
    """
    chain = _chain_from_urdf()
    origins = [_floats(j, "origin", "xyz") for j in chain[:-1]]
    rpys = [_floats(j, "origin", "rpy") for j in chain[:-1]]
    tool_xyz = _floats(chain[-1], "origin", "xyz")
    tool_rpy = _floats(chain[-1], "origin", "rpy")

    for q in (
        (0.0,) * 6,
        (0.1, 0.2, 0.3, -0.4, 0.5, -0.6),
        (-1.2, 1.0, 2.0, 1.5, -1.5, 2.0),
    ):
        position, rotation = yam.forward_kinematics(q)
        want_position, want_rotation = base.chain_fk(
            origins, rpys, tool_xyz, tool_rpy, q
        )
        assert position == pytest.approx(want_position, abs=EXACT)
        assert np.allclose(rotation, want_rotation, atol=EXACT)


def test_forward_kinematics_refuses_a_vector_that_is_not_the_arm():
    """Six arm joints, not the seven-row part vector.

    The seventh row is the gripper, and feeding it to the chain would put the
    tool somewhere nobody commanded. `chain_fk` zips strictly, so the refusal
    is structural rather than a comment asking callers to slice.
    """
    with pytest.raises(ValueError):
        yam.forward_kinematics((0.0,) * yam.JOINT_COUNT)
