"""Package the native PhysX cap assembly using standalone OpenUSD, without Kit.

Requires usd-core 26.8, trimesh 5.1.0 and NumPy 2.5.3. The worker only references
the resulting asset; geometry conversion is an offline authoring operation.
Use --output to review an asset before replacing the packaged file and hash.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

DATA = (
    Path(__file__).resolve().parents[1] / "sdk/python/waddle_sdk/simulators/data/thread"
)


def numbers(element, name):
    return np.fromstring(element.get(name), sep=" ")


def orientation(values):
    return Gf.Quatf(float(values[0]), Gf.Vec3f(*map(float, values[1:])))


def pose(geom, position, quaternion):
    geom.AddTranslateOp().Set(Gf.Vec3d(*map(float, position)))
    geom.AddOrientOp().Set(orientation(quaternion))


def physx_attribute(prim, name, kind, value, uniform=False):
    # Author the public schema declarations without importing a licensed Kit
    # runtime. PhysX interprets these native APIs when it opens the USD asset.
    prim.CreateAttribute(
        name,
        kind,
        custom=False,
        variability=Sdf.VariabilityUniform if uniform else Sdf.VariabilityVarying,
    ).Set(value)


def generate(destination):
    stage = Usd.Stage.CreateNew(str(destination / "cap.usdc"))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/threaded_cap")
    stage.SetDefaultPrim(root.GetPrim())
    material = UsdShade.Material.Define(stage, "/threaded_cap/contact_material")
    contact = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    contact.CreateStaticFrictionAttr(1.2)
    contact.CreateDynamicFrictionAttr(1.0)
    contact.CreateRestitutionAttr(0.0)

    def collision(geom, source):
        prim = geom.GetPrim()
        geom.CreateDisplayColorAttr(
            [Gf.Vec3f(*map(float, numbers(source, "rgba")[:3]))]
        )
        UsdPhysics.CollisionAPI.Apply(prim)
        prim.AddAppliedSchema("PhysxCollisionAPI")
        physx_attribute(
            prim, "physxCollision:contactOffset", Sdf.ValueTypeNames.Float, 0.001
        )
        physx_attribute(
            prim, "physxCollision:restOffset", Sdf.ValueTypeNames.Float, 0.0
        )
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material, materialPurpose="physics"
        )

    template = ET.parse(DATA / "cap.xml")
    for name, mesh_name in (("bottle_thread", "bolt"), ("cap", "nut")):
        source = template.find(f".//body[@name='{name}']")
        body = UsdGeom.Xform.Define(stage, f"/threaded_cap/{name}")
        pose(body, numbers(source, "pos"), numbers(source, "quat"))
        surface = trimesh.load_mesh(DATA / f"{mesh_name}-physx.stl")
        mesh = UsdGeom.Mesh.Define(stage, f"{body.GetPath()}/thread")
        mesh.CreatePointsAttr(
            Vt.Vec3fArray.FromNumpy(np.asarray(surface.vertices, dtype=np.float32))
        )
        mesh.CreateFaceVertexCountsAttr(
            Vt.IntArray.FromNumpy(np.full(len(surface.faces), 3, dtype=np.int32))
        )
        mesh.CreateFaceVertexIndicesAttr(
            Vt.IntArray.FromNumpy(np.asarray(surface.faces, dtype=np.int32).reshape(-1))
        )
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateExtentAttr([Gf.Vec3f(*map(float, row)) for row in surface.bounds])
        collision(mesh, source.find("geom[@type='sdf']"))
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(
            "sdf" if name == "cap" else "none"
        )
        if name != "cap":
            continue
        mesh.GetPrim().AddAppliedSchema("PhysxSDFMeshCollisionAPI")
        for attribute, value in (("sdfResolution", 256), ("sdfSubgridResolution", 6)):
            physx_attribute(
                mesh.GetPrim(),
                f"physxSDFMeshCollision:{attribute}",
                Sdf.ValueTypeNames.Int,
                value,
                uniform=True,
            )
        UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
        inertial = source.find("inertial")
        mass = UsdPhysics.MassAPI.Apply(body.GetPrim())
        mass.CreateMassAttr(float(inertial.get("mass")))
        mass.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, numbers(inertial, "pos"))))
        mass.CreateDiagonalInertiaAttr(
            Gf.Vec3f(*map(float, numbers(inertial, "diaginertia")))
        )
        mass.CreatePrincipalAxesAttr(orientation(numbers(inertial, "quat")))
        body.GetPrim().AddAppliedSchema("PhysxRigidBodyAPI")
        for attribute, value in (
            ("solverPositionIterationCount", 15),
            ("solverVelocityIterationCount", 1),
        ):
            physx_attribute(
                body.GetPrim(),
                f"physxRigidBody:{attribute}",
                Sdf.ValueTypeNames.Int,
                value,
            )
        roof_source = source.find("geom[@name='cap_roof']")
        radius, half_length = numbers(roof_source, "size")
        roof = UsdGeom.Cylinder.Define(stage, f"{body.GetPath()}/roof")
        roof.CreateRadiusAttr(float(radius))
        roof.CreateHeightAttr(float(2 * half_length))
        roof.CreateAxisAttr(UsdGeom.Tokens.z)
        pose(roof, numbers(roof_source, "pos"), [1, 0, 0, 0])
        collision(roof, roof_source)
    stage.GetRootLayer().Save()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DATA)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    generate(args.output)
    digest = hashlib.sha256((args.output / "cap.usdc").read_bytes()).hexdigest()
    if args.output.resolve() == DATA.resolve():
        manifest = json.loads((DATA / "manifest.json").read_text())
        manifest["files"]["cap.usdc"] = digest
        (DATA / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"asset": str(args.output / "cap.usdc"), "sha256": digest}))
