"""Generate the shampoo scene asset by sequential native gravity drops.

Run with MuJoCo 3.13.0 from the SDK repository. Writes evidence under tmp;
inspect before explicitly replacing the shipped pile.json. The installed scene's
geometry can differ from the shipped pile's recorded authoring geometry, so this
utility does not promise bitwise reproduction of a historical asset. This offline authoring
utility has privileged state access and is not a robot task policy.
"""

import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
from waddle_sdk.simulators.mujoco import Engine
from waddle_sdk.simulators.scene import make_site

root = Path(__file__).resolve().parents[1] / "tmp/shampoo-packing"
root.mkdir(parents=True, exist_ok=True)
_, config = make_site(
    "shampoo-drop",
    backend="mujoco",
    robot="yam",
    environment="shampoo-packing",
    width=1280,
    height=960,
)
e = Engine(config, root)
try:
    import random

    from waddle_sdk.simulators.scene import quaternion, rotation

    rng = random.Random(210924)
    bodies = [e.model.body(f"shampoo_{i:02d}") for i in range(1, 21)]
    geoms = []
    for i, body in enumerate(bodies):
        j = e.model.joint(int(body.jntadr[0]))
        q = int(j.qposadr[0])
        e.data.qpos[q : q + 7] = [2 + i * 0.15, 0, 1, 1, 0, 0, 0]
        e.model.body_gravcomp[body.id] = 1
        ids = [
            g
            for g in range(e.model.ngeom)
            if e.model.geom_bodyid[g] == body.id and e.model.geom_contype[g]
        ]
        geoms.append(ids)
        e.model.geom_contype[ids] = 0
        e.model.geom_conaffinity[ids] = 0
    robot_contacts = []
    for index, body in enumerate(bodies):
        top = 0.008
        for placed in bodies[:index]:
            d = e.data.body(placed.id)
            axis = d.xmat.reshape(3, 3)[:, 2]
            top = max(
                top,
                float(
                    d.xpos[2] + 0.045 * abs(axis[2]) + 0.02 * (1 - axis[2] ** 2) ** 0.5
                ),
            )
        j = e.model.joint(int(body.jntadr[0]))
        q = int(j.qposadr[0])
        v = int(j.dofadr[0])
        rpy = [
            rng.uniform(0.6, 2.5),
            rng.uniform(-0.5, 0.5),
            rng.uniform(-math.pi, math.pi),
        ]
        e.data.qpos[q : q + 7] = [
            0.29 + rng.uniform(-0.05, 0.05),
            -0.20 + rng.uniform(-0.04, 0.04),
            top + 0.05,
            *quaternion(rotation(rpy)),
        ]
        e.data.qvel[v : v + 6] = 0
        e.model.body_gravcomp[body.id] = 0
        e.model.geom_contype[geoms[index]] = 1
        e.model.geom_conaffinity[geoms[index]] = 1
        e.mj.mj_forward(e.model, e.data)
        for _ in range(1500):
            e.step()
    for step in range(10000):
        e.step()
        for contact in e.data.contact:
            names = [
                e.model.body(int(e.model.geom_bodyid[g])).name for g in contact.geom
            ]
            if any(n.startswith("shampoo_") for n in names) and any(
                n not in e._prop_links and n not in ("table", "world") for n in names
            ):
                robot_contacts.append((step, names, float(contact.dist)))
    rows = []
    speeds = []
    tilts = []
    for i in range(1, 21):
        body = e.data.body(f"shampoo_{i:02d}")
        m = body.xmat.reshape(3, 3)
        rpy = [
            math.atan2(m[2, 1], m[2, 2]),
            math.asin(float(np.clip(-m[2, 0], -1, 1))),
            math.atan2(m[1, 0], m[0, 0]),
        ]
        rows.append({"xyz": body.xpos.tolist(), "rpy": rpy})
        j = e.model.joint(int(e.model.body(f"shampoo_{i:02d}").jntadr[0]))
        v = int(j.dofadr[0])
        speeds.append(float(np.linalg.norm(e.data.qvel[v : v + 3])))
        tilts.append(float(math.acos(abs(m[2, 2]))))
    output = {
        "schema": "waddle.gravity-pile/v1",
        "provenance": {
            "engine": "mujoco",
            "engine_version": e.mj.__version__,
            "gravity_m_s2": e.model.opt.gravity.tolist(),
            "timestep_s": float(e.model.opt.timestep),
            "settle_steps": 10000,
            "drop_interval_steps": 1500,
            "drop_method": "sequential, 50 mm above current pile top",
            "drop_seed": 210924,
            "source_wall_height_m": 0.06,
            "drop_xy_half_ranges_m": [0.05, 0.04],
            "receiving_origin_z_m": 0.06,
            "receiving_pocket_depth_m": 0.08,
            "receiving_pocket_profile": "continuous-taper",
            "max_linear_speed_m_s": max(speeds),
            "robot_contacts": len(robot_contacts),
        },
        "bottles": rows,
    }
    (root / "settled-pile.json").write_text(json.dumps(output, indent=2) + "\n")
    Image.fromarray(e.capture("scene")[0]).save(root / "shampoo-initial.png")
    print(
        json.dumps(
            {
                "max_speed": max(speeds),
                "tilts": tilts,
                "robot_contacts": robot_contacts[:4],
                "positions": [r["xyz"] for r in rows],
            },
            indent=2,
        )
    )
finally:
    e.close()
