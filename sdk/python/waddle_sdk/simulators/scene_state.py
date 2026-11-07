"""Transfer live native state by identity when a trusted editor changes topology."""

import numpy as np


def pose_edits(before, after):
    """Only explicitly changed authored translation/rotation overrides live pose."""
    old = {before.joint(i).name: i for i in range(before.njnt)}
    result = {}
    for i in range(after.njnt):
        joint = after.joint(i)
        previous = old.get(joint.name)
        if (
            previous is None
            or int(joint.type[0]) != 0
            or before.jnt_type[previous] != 0
        ):
            continue
        a, b = int(before.jnt_qposadr[previous]), int(joint.qposadr[0])
        changes = {}
        for key, span in (("position", slice(0, 3)), ("rotation", slice(3, 7))):
            first, second = before.qpos0[a : a + 7][span], after.qpos0[b : b + 7][span]
            if not np.array_equal(first, second):
                changes[key] = second.copy()
        if changes:
            result[joint.name] = changes
    return result


def transfer_state(before, data, after, target, *, edits=None):
    """Keep time, named joint state, actuator targets and external forces.

    Added joints retain their initialized state. A deliberate teleport clears
    only that object's velocity and solver warm start; resizing/recoloring does
    not teleport it. Native contacts are recomputed against the edited model.
    """
    edits = edits or {}
    old = {before.joint(i).name: i for i in range(before.njnt)}
    for i in range(after.njnt):
        joint = after.joint(i)
        previous = old.get(joint.name)
        if previous is None or before.jnt_type[previous] != after.jnt_type[i]:
            continue
        nq, nv = {0: (7, 6), 1: (4, 3), 2: (1, 1), 3: (1, 1)}[int(after.jnt_type[i])]
        aq, av = int(before.jnt_qposadr[previous]), int(before.jnt_dofadr[previous])
        bq, bv = int(after.jnt_qposadr[i]), int(after.jnt_dofadr[i])
        target.qpos[bq : bq + nq] = data.qpos[aq : aq + nq]
        for key in ("qvel", "qacc_warmstart", "qfrc_applied"):
            getattr(target, key)[bv : bv + nv] = getattr(data, key)[av : av + nv]
        if change := edits.get(joint.name):
            if "position" in change:
                target.qpos[bq : bq + 3] = change["position"]
            if "rotation" in change:
                target.qpos[bq + 3 : bq + 7] = change["rotation"]
            target.qvel[bv : bv + nv] = 0
            target.qacc_warmstart[bv : bv + nv] = 0
    old_actuators = {before.actuator(i).name: i for i in range(before.nu)}
    for i in range(after.nu):
        previous = old_actuators.get(after.actuator(i).name)
        if previous is not None:
            target.ctrl[i] = data.ctrl[previous]
            a, b = int(before.actuator_actadr[previous]), int(after.actuator_actadr[i])
            n = int(after.actuator_actnum[i])
            if a >= 0 and b >= 0 and n == before.actuator_actnum[previous]:
                target.act[b : b + n] = data.act[a : a + n]
    old_bodies = {before.body(i).name: i for i in range(before.nbody)}
    for i in range(after.nbody):
        previous = old_bodies.get(after.body(i).name)
        if previous is not None:
            target.xfrc_applied[i] = data.xfrc_applied[previous]
            a, b = int(before.body_mocapid[previous]), int(after.body_mocapid[i])
            if a >= 0 and b >= 0:
                target.mocap_pos[b] = data.mocap_pos[a]
                target.mocap_quat[b] = data.mocap_quat[a]
    target.time = data.time
