"""Isaac Sim standalone USD/PhysX adapter, isolated from the SDK interpreter."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .model import objects, robot_links, screw_force
from .scene import depth_z16, profile, quaternion, rotation, transform


class Engine:
    def __init__(self, config: dict, scratch: Path):
        del scratch
        # Isaac owns Kit and must initialize before importing Omni/pxr modules.
        # License acceptance remains the site operator's runtime setting.
        from isaacsim import SimulationApp

        self.app = SimulationApp(
            {
                "headless": True,
                "width": 640,
                "height": 480,
                "renderer": "RayTracedLighting",
            }
        )
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.types import ArticulationAction
        from pxr import Gf, PhysxSchema, UsdGeom, UsdLux, UsdPhysics, UsdShade
        import omni.replicator.core as rep

        self.Gf, self.Geom, self.Physics, self.Physx = (
            Gf,
            UsdGeom,
            UsdPhysics,
            PhysxSchema,
        )
        self.Action = ArticulationAction
        self.config = config
        self.profile = p = profile(config["robot"])
        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=config["timestep"],
            rendering_dt=1 / 30,
        )
        self.stage = self.world.stage
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        light = UsdLux.DomeLight.Define(self.stage, "/World/light")
        light.CreateIntensityAttr(1500.0)
        material = UsdShade.Material.Define(self.stage, "/World/contact_material")
        physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics_material.CreateStaticFrictionAttr(1.2)
        physics_material.CreateDynamicFrictionAttr(1.0)
        physics_material.CreateRestitutionAttr(0.0)
        self.material = material
        self.Shade = UsdShade
        self._signs = {}
        self._build(robot_links(p), "robot", robot=True)
        self.robot = self.world.scene.add(
            SingleArticulation(prim_path="/World/robot", name="robot")
        )
        self.props = []
        for links in objects(config["environment"]):
            name = links[0].name
            self._build(links, name)
            if len(links) > 1:
                item = self.world.scene.add(
                    SingleArticulation(prim_path=f"/World/{name}", name=name)
                )
                self.props.append(item)
        self._screw = self.props[-1] if config["environment"] == "bottle_cap" else None
        self.world.reset()
        self.order = list(self.robot.dof_names)
        self.arm_indices = [self.order.index(n) for n in p.names[:-1]]
        self.finger_indices = [self.order.index(n) for n in ("left_finger", "right_finger")]
        self.home(p.home)
        self.cameras = {}
        for name, row in config["cameras"].items():
            path = f"/World/camera_{name}"
            camera = UsdGeom.Camera.Define(self.stage, path)
            intr, stream = row["intrinsics"], row["stream"]
            camera.CreateFocalLengthAttr(1.0)
            camera.CreateHorizontalApertureAttr(stream["width"] / intr["fx"])
            camera.CreateVerticalApertureAttr(stream["height"] / intr["fy"])
            camera.CreateHorizontalApertureOffsetAttr(
                (intr["cx"] - stream["width"] / 2) / intr["fx"]
            )
            camera.CreateVerticalApertureOffsetAttr(
                -(intr["cy"] - stream["height"] / 2) / intr["fy"]
            )
            camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 10.0))
            self._pose(camera, np.eye(4))
            product = rep.create.render_product(path, (stream["width"], stream["height"]))
            rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            depth = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
            rgb.attach([product])
            depth.attach([product])
            self.cameras[name] = (camera, product, rgb, depth)

    def _pose(self, geom, matrix):
        xform = self.Geom.Xformable(geom)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(self.Gf.Vec3d(*map(float, matrix[:3, 3])))
        q = quaternion(matrix[:3, :3])
        xform.AddOrientOp().Set(self.Gf.Quatf(float(q[0]), self.Gf.Vec3f(*map(float, q[1:]))))

    def _build(self, links, name, robot=False):
        P, G, F, PX = self.Physics, self.Geom, self.Gf, self.Physx
        root_path = f"/World/{name}"
        root = G.Xform.Define(self.stage, root_path)
        articulated = len(links) > 1
        if articulated:
            P.ArticulationRootAPI.Apply(root.GetPrim())
            PX.PhysxArticulationAPI.Apply(root.GetPrim()).CreateEnabledSelfCollisionsAttr(True)
        poses = {}
        for link in links:
            pose = transform(link.xyz, link.rpy)
            if link.parent is not None:
                pose = poses[link.parent] @ pose
            poses[link.name] = pose
            path = f"{root_path}/{link.name}"
            body = G.Xform.Define(self.stage, path)
            self._pose(body, pose)
            if articulated or link.kind == "free":
                P.RigidBodyAPI.Apply(body.GetPrim())
                mass = P.MassAPI.Apply(body.GetPrim())
                mass.CreateMassAttr(link.mass)
                mass.CreateDiagonalInertiaAttr(F.Vec3f(*([max(link.mass * 0.003, 1e-6)] * 3)))
                if robot:
                    PX.PhysxRigidBodyAPI.Apply(body.GetPrim()).CreateDisableGravityAttr(True)
            for i, shape in enumerate(link.shapes):
                gp = f"{path}/shape_{i}"
                if shape.kind == "box":
                    geometry = G.Cube.Define(self.stage, gp)
                    geometry.CreateSizeAttr(1.0)
                elif shape.kind == "sphere":
                    geometry = G.Sphere.Define(self.stage, gp)
                    geometry.CreateRadiusAttr(shape.size[0])
                else:
                    geometry = G.Cylinder.Define(self.stage, gp)
                    geometry.CreateRadiusAttr(shape.size[0])
                    geometry.CreateHeightAttr(shape.size[1])
                    geometry.CreateAxisAttr("Z")
                self._pose(geometry, transform(shape.xyz, shape.rpy))
                if shape.kind == "box":
                    G.Xformable(geometry).AddScaleOp().Set(F.Vec3f(*map(float, shape.size)))
                geometry.CreateDisplayColorAttr([F.Vec3f(*map(float, shape.color[:3]))])
                P.CollisionAPI.Apply(geometry.GetPrim())
                PX.PhysxCollisionAPI.Apply(geometry.GetPrim()).CreateContactOffsetAttr(0.001)
                self.Shade.MaterialBindingAPI.Apply(geometry.GetPrim()).Bind(
                    self.material, materialPurpose="physics"
                )
            if not articulated:
                continue
            if link.parent is None:
                joint = P.FixedJoint.Define(self.stage, f"{root_path}/anchor")
                joint.CreateBody1Rel().SetTargets([path])
                joint.CreateLocalPos0Attr(F.Vec3f(*map(float, pose[:3, 3])))
                q = quaternion(pose[:3, :3])
                joint.CreateLocalRot0Attr(F.Quatf(float(q[0]), F.Vec3f(*map(float, q[1:]))))
                continue
            parent_path = f"{root_path}/{link.parent}"
            joint_path = f"{root_path}/joints/{link.joint or link.name + '_fixed'}"
            kind = (
                P.RevoluteJoint
                if link.kind == "revolute"
                else P.PrismaticJoint
                if link.kind == "prismatic"
                else P.FixedJoint
            )
            joint = kind.Define(self.stage, joint_path)
            joint.CreateBody0Rel().SetTargets([parent_path])
            joint.CreateBody1Rel().SetTargets([path])
            joint.CreateLocalPos0Attr(F.Vec3f(*map(float, link.xyz)))
            q = quaternion(rotation(link.rpy))
            joint.CreateLocalRot0Attr(F.Quatf(float(q[0]), F.Vec3f(*map(float, q[1:]))))
            P.FilteredPairsAPI.Apply(body.GetPrim()).CreateFilteredPairsRel().AddTarget(
                parent_path
            )
            if link.joint:
                axis = int(np.argmax(np.abs(link.axis)))
                sign = float(link.axis[axis])
                self._signs[link.joint] = sign
                joint.CreateAxisAttr("XYZ"[axis])
                limits = sorted(
                    np.asarray(link.limits)
                    * sign
                    * (180 / np.pi if link.kind == "revolute" else 1.0)
                )
                joint.CreateLowerLimitAttr(float(limits[0]))
                joint.CreateUpperLimitAttr(float(limits[1]))
                if robot:
                    finger = link.kind == "prismatic"
                    drive = P.DriveAPI.Apply(joint.GetPrim(), "linear" if finger else "angular")
                    # USD angular drive gains use degrees; the public joint
                    # control API and all SDK vectors remain in radians.
                    factor = 1.0 if finger else np.pi / 180
                    drive.CreateStiffnessAttr(5000 * factor)
                    drive.CreateDampingAttr((30 if finger else 70) * factor)
                    drive.CreateMaxForceAttr(20.0 if finger else 50.0)

    def _expand(self, q):
        values = np.zeros(len(self.order))
        values[self.arm_indices] = q[:-1]
        for i in self.finger_indices:
            values[i] = q[-1] * self.profile.opening / 2 * self._signs[self.order[i]]
        return values

    def read(self):
        q, dq = self.robot.get_joint_positions(), self.robot.get_joint_velocities()
        signs = np.array([self._signs[self.order[i]] for i in self.finger_indices])
        return np.r_[
            q[self.arm_indices],
            sum(q[self.finger_indices] * signs) / self.profile.opening,
        ], np.r_[
            dq[self.arm_indices],
            sum(dq[self.finger_indices] * signs) / self.profile.opening,
        ]

    def write(self, q):
        self.robot.apply_action(self.Action(joint_positions=self._expand(q)))

    def hold(self):
        self.write(self.read()[0])

    def home(self, q):
        self.robot.set_joint_positions(self._expand(q))
        self.robot.set_joint_velocities(np.zeros(len(self.order)))
        self.write(q)
        return True

    def step(self):
        if self._screw is not None:
            names = list(self._screw.dof_names)
            idx = [names.index(n) for n in ("cap_rotation", "cap_lift")]
            force = np.zeros(len(names))
            force[idx] = screw_force(
                self._screw.get_joint_positions()[idx],
                self._screw.get_joint_velocities()[idx],
            )
            self._screw.set_joint_efforts(force)
        self.world.step(render=False)

    def capture(self, name):
        row = self.config["cameras"][name]
        camera, _product, rgb, depth = self.cameras[name]
        matrix = np.asarray(row["transform"]).copy()
        if row["mount"]["kind"] == "wrist":
            tcp = self.Geom.Xformable(self.stage.GetPrimAtPath("/World/robot/tcp"))
            matrix = np.asarray(tcp.ComputeLocalToWorldTransform(0)).T @ matrix
        matrix[:3, :3] = matrix[:3, :3] @ np.diag([1, -1, -1])
        self._pose(camera, matrix)
        # Flush render/annotator latency at a fixed physics state. RGB and
        # axial depth come from the same render product and tick.
        for _ in range(3):
            self.world.render()
        color, z = np.asarray(rgb.get_data()), np.asarray(depth.get_data())
        shape = (row["stream"]["height"], row["stream"]["width"])
        if color.shape[:2] != shape or z.shape != shape:
            raise RuntimeError("Isaac render product has no complete RGB-D frame")
        return np.ascontiguousarray(color[..., :3], dtype=np.uint8), depth_z16(
            z, row["intrinsics"]["depth_scale_mm"]
        )

    def native_tcp(self):
        tcp = self.Geom.Xformable(self.stage.GetPrimAtPath("/World/robot/tcp"))
        pose = np.asarray(tcp.ComputeLocalToWorldTransform(0)).T
        return pose[:3, 3].copy(), pose[:3, :3].copy()

    def close(self):
        for _camera, product, rgb, depth in self.cameras.values():
            rgb.detach([product])
            depth.detach([product])
            product.destroy()
        self.world.stop()
        self.app.close()
