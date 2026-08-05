# I2RT YAM URDF (vendored, patched) — third-party content

This directory is **not** waddle-sdk code. It is a point-in-time snapshot of
a robot description published by the robot's maker, shipped inside an
Apache-2.0 wheel under its own MIT licence.

| | |
|---|---|
| Source | <https://github.com/i2rt-robotics/i2rt> |
| Path | `robot_models/arm/yam/yam.urdf` |
| Pinned commit | `570ef66681ff12bd8298aba34084307cfecc9f05` |
| Licence | MIT — [`./LICENSE`](./LICENSE), copied verbatim from the source repo root |

`waddle.robots.yam` reads this file two ways: `yam.urdf_text()` hands it to a
single-arm declaration as `kinematics_urdf`, and `sdk/tests/test_yam_facts.py`
compares every constant in `yam.py` against the numbers written here. That
second use is the reason the file ships at all — a fact table with no second
source is a fact table nothing checks.

## Meshes are not included, on purpose

The URDF's `<mesh filename="assets/…">` references are **unresolved**: none of
the 14 STL files it names ship here. This copy is the kinematic contract — the
chain, the limits, the tool frame — not a visual model, and nothing in this
SDK renders anything. The meshes are megabytes against a wheel that is a few,
and a viewer that wants them can fetch them from the pinned commit above.

## Patches applied to `yam.urdf`

Upstream's kinematics, joint limits and inertial properties are untouched. The
three patches below are the vendored copy's, and one more (patch 4) is this
wheel's.

1. **Mesh refs**: `package://assets/` → `assets/` (relative), so the meshes
   would resolve next to this file rather than via a ROS `package://` URI —
   moot here, since they are not shipped.
2. **A fixed TCP frame appended**: `<link name="grasp_link"/>` plus
   `<joint name="grasp_joint" type="fixed">` (parent `link_6`, origin
   `xyz="0 0 0.1347" rpy="0 0 -1.5708"`), mirroring the MuJoCo Menagerie
   `grasp_site` (`pos="0 0 0.1347"`, `quat="1 0 0 -1"` in `link_6`).
   `quat="1 0 0 -1"` is MuJoCo wxyz — a −90° rotation about `link_6`'s **Z**
   axis — so the matching URDF rpy is yaw-only
   (`rpy = Rz(yaw)·Ry(pitch)·Rx(roll)`).

   The frame originally shipped as `rpy="-1.5708 0 0"` — a rotation about
   **X**. Translation does not depend on `rpy`, so `grasp_link`'s *position*
   matched exactly and the wrong axis went unnoticed until an FK-agreement
   check compared *orientations* and found 120° of divergence. This is why
   `waddle.robots.yam` states its tool fact in the tool's frame and asserts
   the frame names, rather than in the flange's.
3. **`link_6` meshes backfilled** from upstream commit
   `d4efb66d81bd8bde42909880b16591d4af82e8c0`: `link_6_visual.stl` /
   `link_6_collision.stl` do not exist at the pinned commit, though the URDF
   at that commit still references them. Only the two STL files came from the
   older commit, and neither ships here — the markup is upstream's.
4. **This copy only, and comment text only** — no element of the model
   differs:
   - the header comment's pointer to the patch writeup names `./README.md`
     (this file) instead of the path it had in the repo this snapshot was
     taken from;
   - one `--` inside the patch-2 comment became `:`. A double hyphen is
     illegal inside an XML comment, and it made the file unparseable by
     strict parsers — Python's stdlib `xml.etree` among them, which is what
     the fact gate reads it with, and what a customer handed this as
     `kinematics_urdf` is likely to reach for. Lenient parsers accepted it,
     which is why it survived;
   - two pointers that resolved only in that repo are gone from the patch-2
     comments: a task-tracker label on the correction, and the path of the
     FK-agreement check that caught it. *What* the correction was is still
     stated, in the header and again at `grasp_joint`; only the places a
     reader of this wheel could not open have been dropped.

Everything in this directory ships to everyone who installs the SDK, comments
included, so the rule behind that last item is the general one:
`sdk/tests/test_yam_facts.py` refuses an internal task label or an
unreachable source path anywhere in these files. Re-vendoring re-applies it.

## Re-vendoring

There is no generator to re-run: this is a hand-made snapshot. Re-vendor by
applying the same patches to a fresh checkout of the upstream repo at whatever
commit is then current, updating `I2RT_PIN` in `waddle/robots/yam.py`, and
running the fact gate — which is what says whether any number moved.
