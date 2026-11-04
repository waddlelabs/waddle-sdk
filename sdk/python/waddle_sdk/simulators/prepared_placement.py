"""Reposition a private whole-scene template after exact source comparison."""

import math
from copy import copy

from .authored_preparation import _contact_delta, _encoded, _key


def _references(node, names):
    return (
        any(names.intersection(value.split()) for _, value in node[1])
        or bool(names.intersection((node[2] or "").split()))
        or any(_references(child, names) for child in node[3])
    )


def translate(model, before, after):
    """Return an independent model for isolated free-body translations, else None.

    Source buffers already bind every non-root byte. This function additionally
    compares the complete root structure, including unchanged contacts. It does
    not read files or retain source-validity decisions, data or live state.
    """
    import mujoco as mj
    import numpy as np

    if (
        model is None
        or before is None
        or after is None
        or before.static != after.static
        or _key(before.parts["asset"]) != _key(after.parts["asset"])
        or any(_contact_delta(before, after))
    ):
        return None
    old, new = before.parts["worldbody"], after.parts["worldbody"]
    if old.attrib != new.attrib or old.text != new.text or len(old) != len(new):
        return None
    if _key(old) == _key(new):
        return copy(model)
    if any(node[0] == "statistic" for node in before.static[3]) or any(
        getattr(model, name) for name in ("nplugin", "nflex", "nskin")
    ):
        return None
    updates, names, unchanged = [], set(), []
    for a, b in zip(old, new, strict=True):
        if _key(a) == _key(b):
            unchanged.append(_key(a))
            continue
        if (
            a.tag != "body"
            or b.tag != "body"
            or not a.get("name", "").startswith("prop_")
            or a.attrib.keys() != b.attrib.keys()
            or {k: v for k, v in a.attrib.items() if k != "pos"}
            != {k: v for k, v in b.attrib.items() if k != "pos"}
            or a.text != b.text
            or _encoded(a) != _encoded(b)
            or len(a.findall("freejoint")) != 1
            or any(node.tag not in {"freejoint", "geom"} or len(node) for node in a)
        ):
            return None
        position = [float(value) for value in b.get("pos", "").split()]
        previous = [float(value) for value in a.get("pos", "").split()]
        if len(position) != 3 or not all(math.isfinite(value) for value in position):
            return None
        body = model.body(a.get("name")).id
        joint = int(model.body_jntadr[body])
        if (
            model.body_parentid[body] != 0
            or model.body_mocapid[body] != -1
            or model.body_jntnum[body] != 1
            or model.jnt_type[joint] != mj.mjtJoint.mjJNT_FREE
            or not np.array_equal(model.body_pos[body], previous)
        ):
            return None
        names.update(node.get("name") for node in a.iter() if node.get("name"))
        updates.append((body, int(model.jnt_qposadr[joint]), position))
    # An external constraint, actuator, sensor or camera can depend on this
    # body's authored pose. Preserve the compiler's initialization for those.
    if (
        not updates
        or _references(before.static, names)
        or any(_references(node, names) for node in unchanged)
    ):
        return None
    result = copy(model)
    for body, qpos, position in updates:
        result.body_pos[body] = position
        result.qpos0[qpos : qpos + 3] = position
        result.qpos_spring[qpos : qpos + 3] = position
        result.key_qpos[:, qpos : qpos + 3] = position
    # Recompute all constants and statistics from the revised default pose.
    # This is disposable preparation data, never the running world's MjData.
    mj.mj_setConst(result, mj.MjData(result))
    return result
