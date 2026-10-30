"""Reference planning collision surfaces retain native linear-finger gaps."""

import numpy as np
import pytest
from waddle_sdk.simulators import reference_model_sources


def test_yam_planning_fingers_preserve_native_gaps():
    """Distances around concave finger surfaces agree with the native assembly."""
    from itertools import product
    from xml.etree import ElementTree as ET

    from waddle_sdk.simulators import articulated_model_sources
    from waddle_sdk.simulators.description import description

    mj = pytest.importorskip("mujoco")
    models, states = [], []
    for bundle in (
        articulated_model_sources("yam", part_name="arm"),
        reference_model_sources("yam", part_name="arm"),
    ):
        root = ET.fromstring(bundle.model)
        ET.SubElement(
            root.find("worldbody"), "geom", name="probe", type="sphere", size="0.0001"
        )
        model = mj.MjModel.from_xml_string(
            ET.tostring(root, encoding="unicode"), dict(bundle.assets)
        )
        data = mj.MjData(model)
        for name in description("yam").hand_names:
            if mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name) >= 0:
                data.joint(name).qpos[0] = description("yam").hand_position(1.0)
        mj.mj_kinematics(model, data)
        mj.mj_comPos(model, data)
        models.append(model)
        states.append(data)
    for body in ("tip_left", "tip_right"):
        groups = [
            [
                i
                for i in range(m.ngeom)
                if m.geom_bodyid[i] == m.body(body).id and m.geom_contype[i]
            ]
            for m in models
        ]
        data = states[0]
        centers = data.geom_xpos[groups[0]]
        low, high = centers.min(axis=0) - 0.008, centers.max(axis=0) + 0.008
        errors = []
        for point in product(
            *(np.linspace(a, b, 7) for a, b in zip(low, high, strict=True))
        ):
            distances = []
            for model, state, geoms in zip(models, states, groups, strict=True):
                probe = model.geom("probe").id
                state.geom_xpos[probe] = point
                distances.append(
                    min(
                        mj.mj_geomDistance(model, state, g, probe, 1, None)
                        for g in geoms
                    )
                )
            errors.append(abs(distances[0] - distances[1]))
        assert max(errors) < 2e-6, (body, max(errors))
