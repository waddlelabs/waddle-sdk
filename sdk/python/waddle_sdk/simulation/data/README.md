# Model provenance

`xarm7-kinematics.yaml` is the unmodified
`xarm_description/config/kinematics/default/xarm7_default_kinematics.yaml` from
https://github.com/xArm-Developer/xarm_ros at
`aad7e1611c9c46eb719045414394bfdd42dcb0f8` (BSD-3-Clause; `xarm-LICENSE`).
The 0.172 m flange-to-TCP offset and gripper closing direction come from that
revision's `xarm_description/urdf/gripper/xarm_gripper.urdf.xacro`.
The G2 hand's 84 mm opening agrees with the SDK live adapter's normalization.
Joint names/limits are read from the SDK's non-opening xArm7 declaration.

YAM origins, rotations, joint limits, gripper stroke, and TCP come directly from
`waddle_sdk.robots.yam`, whose constants document the pinned public I2RT source.
The linear hand uses a pinch point at (0.044, 0, -0.0049) in that TCP frame.

`model.py` generates original primitive collision/visual shapes and approximate
inertias. These files contain kinematic facts; they do not claim exact CAD, motor,
friction, or inertial equivalence to a manufactured robot.
