"""Compound bowls retain support, hollow walls and stack contact without duplicate soles."""

import numpy as np
import pytest
from waddle_sdk.simulators.playground import compile_scene
from waddle_sdk.simulators.scene import load_scene, make_site


@pytest.mark.parametrize("mode", ["upright", "tilted", "stacked", "filled"])
def test_bowl_support_and_wall_contacts(tmp_path, mode):
    pytest.importorskip("mujoco")
    from waddle_sdk.simulators.mujoco import Engine

    _, config = make_site(
        "bowls",
        backend="mujoco",
        robot="yam",
        arms=2,
        environment="two_cubes",
        width=32,
        height=32,
    )
    first = {
        "id": "lower",
        "kind": "bowl",
        "position": [0.35, -0.15, 0.0305],
        "rotation": [0, 0, 0],
        "size": [0.14, 0.14, 0.06],
        "color": [0.8, 0.2, 0.3],
    }
    if mode == "tilted":
        first.update(position=[0.35, -0.15, 0.08], rotation=[90, 0, 0])
    objects = [first]
    if mode == "stacked":
        objects.extend(
            [
                {**first, "id": "upper", "position": [0.35, -0.15, 0.091]},
                {**first, "id": "top", "position": [0.35, -0.15, 0.152]},
            ]
        )
    if mode == "filled":
        objects.append(
            {
                **first,
                "id": "inside",
                "kind": "block",
                "position": [0.35, -0.15, 0.04],
                "size": [0.02, 0.02, 0.02],
            }
        )
    out = tmp_path / "world"
    compile_scene(config, {"id": "bowls", "name": "Bowls", "objects": objects}, out)
    _, compiled = load_scene(out, "simulation.json")
    engine = Engine(compiled, out)
    try:
        for _ in range(1000 if mode == "stacked" else 500):
            engine.step()
        state = engine.evaluation_snapshot()
        lower = state["bodies"]["prop_lower"]
        assert np.isfinite(engine.data.qpos).all()
        assert lower["position_m"][2] > 0.02
        if mode == "upright":
            table = engine.model.body("table").id
            contacts = [
                c
                for c in engine.data.contact
                if table in engine.model.geom_bodyid[c.geom]
            ]
            assert 0 < len(contacts) <= 8, (
                "Overlapping wall soles multiply one support patch"
            )
            assert lower["position_m"][2] == pytest.approx(0.03, abs=0.002)
        elif mode == "stacked":
            upper = state["bodies"]["prop_upper"]
            # Equal curved bowls nest through their hollow interiors; the rim
            # supports the upper bowl above the lower floor, without a weld.
            assert 0.012 < upper["position_m"][2] - lower["position_m"][2] < 0.045
            np.testing.assert_allclose(
                upper["position_m"][:2], lower["position_m"][:2], atol=0.004
            )
            assert np.linalg.norm(upper["linear_velocity_m_s"]) < 0.02
            top = state["bodies"]["prop_top"]
            assert 0.012 < top["position_m"][2] - upper["position_m"][2] < 0.045
            np.testing.assert_allclose(
                top["position_m"][:2], lower["position_m"][:2], atol=0.004
            )
            assert np.linalg.norm(top["linear_velocity_m_s"]) < 0.02
        elif mode == "filled":
            inside = state["bodies"]["prop_inside"]
            assert inside["position_m"][2] == pytest.approx(0.014, abs=0.002)
        else:
            wall_ids = {
                i
                for i in range(engine.model.ngeom)
                if engine.model.geom_bodyid[i] == engine.model.body("prop_lower").id
                and engine.model.geom_group[i] == 3
                and engine.model.geom_type[i] == engine.mj.mjtGeom.mjGEOM_MESH
            }
            table = engine.model.body("table").id
            assert any(
                table in engine.model.geom_bodyid[list(pair)]
                and wall_ids.intersection(pair)
                for pair in engine._contact_events
            ), "Tilted walls must still collide with the table"
    finally:
        engine.close()
