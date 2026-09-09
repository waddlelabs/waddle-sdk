"""Check the packaged native assembly with standalone USD, without launching Kit."""

import xml.etree.ElementTree as ET

import numpy as np
import pytest

pytest.importorskip("pxr.Usd", reason="requires standalone usd-core")
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade
from waddle_sdk.simulators.description import mesh_triangles
from waddle_sdk.simulators.thread import append_isaac, assets


@pytest.fixture
def assembly():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    # Exercise the production reference path, including relocation of internal
    # material bindings and body transforms away from the asset's default root.
    cap = append_isaac(stage, "/World/fixture")
    return stage, cap


def test_cap_is_one_free_rigid_body_with_source_inertials(assembly):
    stage, cap = assembly
    assert [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)] == [cap]
    assert not any(p.IsA(UsdPhysics.Joint) for p in stage.Traverse())
    rigid = UsdPhysics.RigidBodyAPI(cap)
    assert rigid.GetRigidBodyEnabledAttr().Get()
    assert not rigid.GetKinematicEnabledAttr().Get()
    source = ET.parse(assets() / "cap.xml").find(".//body[@name='cap']/inertial")
    mass = UsdPhysics.MassAPI(cap)
    assert mass.GetMassAttr().Get() == pytest.approx(float(source.get("mass")))
    np.testing.assert_allclose(
        mass.GetCenterOfMassAttr().Get(),
        np.fromstring(source.get("pos"), sep=" "),
        atol=1e-9,
    )
    axes = np.asarray(Gf.Matrix3d(mass.GetPrincipalAxesAttr().Get()))
    actual = axes.T @ np.diag(mass.GetDiagonalInertiaAttr().Get()) @ axes
    quat = np.fromstring(source.get("quat"), sep=" ")
    expected_axes = np.asarray(
        Gf.Matrix3d(Gf.Quatd(float(quat[0]), Gf.Vec3d(*quat[1:])))
    )
    expected = (
        expected_axes.T
        @ np.diag(np.fromstring(source.get("diaginertia"), sep=" "))
        @ expected_axes
    )
    np.testing.assert_allclose(actual, expected, atol=1e-12, rtol=0)


@pytest.mark.parametrize("name,mesh_name", [("cap", "nut"), ("bottle_thread", "bolt")])
def test_usd_thread_retains_exact_source_surfaces_and_native_collision(
    assembly, name, mesh_name
):
    stage, _ = assembly
    body = stage.GetPrimAtPath(f"/World/fixture/{name}")
    mesh = UsdGeom.Mesh.Get(stage, f"{body.GetPath()}/thread")
    assert np.all(np.asarray(mesh.GetFaceVertexCountsAttr().Get()) == 3)
    points = np.asarray(mesh.GetPointsAttr().Get())
    faces = np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
    np.testing.assert_array_equal(
        points[faces], mesh_triangles(str(assets() / f"{mesh_name}-physx.stl"))
    )
    # Closed, consistently wound two-manifold topology, independently of the
    # offline mesh loader. Open or degenerate surfaces invalidate an SDF solid.
    edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    _, count = np.unique(np.sort(edges, axis=1), axis=0, return_counts=True)
    assert np.all(count == 2)
    assert len(np.unique(edges, axis=0)) == len(edges)
    triangles = points[faces]
    assert np.all(
        np.linalg.norm(
            np.cross(
                triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]
            ),
            axis=1,
        )
        > 0
    )
    assert mesh.GetSubdivisionSchemeAttr().Get() == "none"
    collision = UsdPhysics.MeshCollisionAPI(mesh.GetPrim())
    assert collision.GetApproximationAttr().Get() == (
        "sdf" if name == "cap" else "none"
    )
    if name == "cap":
        assert (
            "PhysxSDFMeshCollisionAPI"
            in mesh.GetPrim().GetMetadata("apiSchemas").GetAppliedItems()
        )
        assert (
            mesh.GetPrim().GetAttribute("physxSDFMeshCollision:sdfResolution").Get()
            == 256
        )
        assert (
            mesh.GetPrim()
            .GetAttribute("physxSDFMeshCollision:sdfSubgridResolution")
            .Get()
            == 6
        )
    source = ET.parse(assets() / "cap.xml").find(f".//body[@name='{name}']")
    pose = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    np.testing.assert_allclose(
        pose.ExtractTranslation(), np.fromstring(source.get("pos"), sep=" "), atol=1e-10
    )
    quat = np.fromstring(source.get("quat"), sep=" ")
    np.testing.assert_allclose(
        np.asarray(pose)[:3, :3],
        Gf.Matrix3d(Gf.Quatd(float(quat[0]), Gf.Vec3d(*quat[1:]))),
        atol=1e-7,
    )


def test_usd_roof_and_collision_material_survive_reference_composition(assembly):
    stage, cap = assembly
    source = Usd.Stage.Open(str(assets() / "cap.usdc"))
    assert UsdGeom.GetStageMetersPerUnit(source) == 1.0
    assert UsdGeom.GetStageUpAxis(source) == "Z"
    roof = UsdGeom.Cylinder.Get(stage, f"{cap.GetPath()}/roof")
    assert roof.GetRadiusAttr().Get() == pytest.approx(0.02887)
    assert roof.GetHeightAttr().Get() == pytest.approx(0.002)
    assert roof.GetAxisAttr().Get() == "Z"
    pose = roof.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    np.testing.assert_allclose(
        pose.ExtractTranslation(), [0.34, -0.16, 0.2201], atol=1e-9
    )
    colliders = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
    assert len(colliders) == 3
    for prim in colliders:
        assert prim.GetAttribute("physxCollision:contactOffset").Get() == pytest.approx(
            0.001
        )
        assert prim.GetAttribute("physxCollision:restOffset").Get() == 0
        material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial(
            materialPurpose="physics"
        )
        assert str(material.GetPath()) == "/World/fixture/contact_material"
        contact = UsdPhysics.MaterialAPI(material.GetPrim())
        assert contact.GetStaticFrictionAttr().Get() == pytest.approx(1.2)
        assert contact.GetDynamicFrictionAttr().Get() == 1.0
        assert contact.GetRestitutionAttr().Get() == 0
