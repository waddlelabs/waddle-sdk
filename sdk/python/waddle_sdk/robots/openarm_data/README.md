# OpenArm v1 source facts

This directory contains the exact text inputs used to gate
`waddle_sdk.robots.openarm` constants. They were copied from
`enactic/openarm_description` commit
`1fba2cbc05001f05b4514120b70130b4ac06f409`:

- `arm/control_gains.yaml`
- `arm/joint_limits.yaml`
- `arm/kinematics.yaml`
- `arm/kinematics_offset.yaml`
- `gripper/kinematics.yaml`
- `gripper/openarm_parallel_gripper.xacro`

The files remain under the upstream Apache-2.0 license reproduced in
`LICENSE.txt`. They are package data, not executable code. The SDK's
seven-joint ordering, axes, origins, gains, limits, hand origin, and TCP
offset are tested against these shipped bytes.
