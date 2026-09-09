"""Isaac Sim standalone USD/PhysX adapter, isolated from the SDK interpreter."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .description import description
from .model import objects, robot_links, urdf
from .scene import depth_z16, profile, quaternion, transform


class Engine:
    def __init__(self, config: dict, scratch: Path):
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
        import omni.kit.app
        import omni.replicator.core as rep

        omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
            "isaacsim.asset.importer.urdf", True
        )
        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation, SingleRigidPrim
        from isaacsim.core.utils.types import ArticulationAction
        from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

        self.Gf, self.Geom, self.Physics, self.Physx = (
            Gf,
            UsdGeom,
            UsdPhysics,
            PhysxSchema,
        )
        self.Usd = Usd
        self.Articulation = SingleArticulation
        self.RigidPrim = SingleRigidPrim
        self.Action = ArticulationAction
        self.config = config
        self.profile = p = profile(config["robot"])
        self.description = robot = description(p.name)
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
        self.robot, robot_bodies = self._load(
            robot_links(p), "robot", scratch, robot=True
        )
        self.props = [
            self._load(links, links[0].name, scratch)[0]
            for links in objects(config["environment"])
        ]
        self._screw = self.props[-1] if config["environment"] == "bottle_cap" else None
        self.tcp = self.world.scene.add(
            SingleRigidPrim(
                prim_path=str(robot_bodies["tcp"].GetPath()),
                name="tcp_measurement",
                reset_xform_properties=False,
            )
        )
        self.world.reset()
        self.order = list(self.robot.dof_names)
        self.arm_indices = [self.order.index(n) for n in p.names[:-1]]
        self.finger_indices = [self.order.index(n) for n in robot.hand_names]
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
            product = rep.create.render_product(
                path, (stream["width"], stream["height"])
            )
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
        xform.AddOrientOp().Set(
            self.Gf.Quatf(float(q[0]), self.Gf.Vec3f(*map(float, q[1:])))
        )

    def _load(self, links, name, scratch, robot=False):
        """Use Isaac Sim's importer, as Isaac Lab's UrdfConverter does.

        The importer owns mesh conversion, inertias, joint axes, and mimic
        constraints. SDK code only configures drives and binds named bodies.
        """
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

        path = scratch / f"{name}.urdf"
        path.write_text(urdf(links, name))
        gains = {
            link.joint: self.description.servo(link.joint)
            for link in links
            if robot and link.joint
        }
        config = URDFImporterConfig(
            urdf_path=str(path),
            usd_path=str(scratch / "usd"),
            fix_base=links[0].kind != "free",
            merge_fixed_joints=False,
            collision_from_visuals=False,
            allow_self_collision=True,
            joint_drive_type="force",
            joint_target_type="position" if robot else "none",
            override_joint_stiffness={key: value[0] for key, value in gains.items()},
            override_joint_damping={key: value[1] for key, value in gains.items()},
        )
        usd_path = URDFImporter(config).import_urdf()
        if not usd_path:
            raise RuntimeError(f"Isaac Sim could not import {name}")
        root_path = f"/World/{name}"
        root = self.Geom.Xform.Define(self.stage, root_path)
        root.GetPrim().GetReferences().AddReference(usd_path)
        root.GetPrim().GetVariantSet("Physics").SetVariantSelection("physx")
        self._pose(root, transform(links[0].xyz, links[0].rpy))
        prims = list(self.Usd.PrimRange(root.GetPrim()))
        bodies = {
            prim.GetName(): prim
            for prim in prims
            if prim.HasAPI(self.Physics.RigidBodyAPI)
        }
        for prim in prims:
            if prim.HasAPI(self.Physics.CollisionAPI):
                self.Shade.MaterialBindingAPI.Apply(prim).Bind(
                    self.material, materialPurpose="physics"
                )
            if robot and prim.HasAPI(self.Physics.RigidBodyAPI):
                self.Physx.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)
            if robot and prim.GetName() in gains and prim.IsA(self.Physics.Joint):
                axis = "linear" if prim.IsA(self.Physics.PrismaticJoint) else "angular"
                self.Physics.DriveAPI.Apply(prim, axis).CreateMaxForceAttr(
                    gains[prim.GetName()][2]
                )
        if robot:
            for first, second in self.description.exclusions:
                self.Physics.FilteredPairsAPI.Apply(
                    bodies[first]
                ).CreateFilteredPairsRel().AddTarget(bodies[second].GetPath())
        if robot:
            for first, anchor, second, other in self.description.closures():
                joint = self.Physics.SphericalJoint.Define(
                    self.stage, f"{root_path}/closure_{first}"
                )
                joint.CreateBody0Rel().SetTargets([bodies[first].GetPath()])
                joint.CreateBody1Rel().SetTargets([bodies[second].GetPath()])
                joint.CreateLocalPos0Attr(self.Gf.Vec3f(*map(float, anchor)))
                joint.CreateLocalPos1Attr(self.Gf.Vec3f(*map(float, other)))
        if len(links) > 1:
            roots = [
                prim for prim in prims if prim.HasAPI(self.Physics.ArticulationRootAPI)
            ]
            if len(roots) != 1:
                raise RuntimeError(
                    f"Isaac importer produced {len(roots)} articulation roots for {name}"
                )
            item = self.Articulation(prim_path=str(roots[0].GetPath()), name=name)
        elif links[0].kind == "free":
            item = self.RigidPrim(
                prim_path=str(bodies[links[0].name].GetPath()), name=name
            )
        else:
            return None, bodies
        return self.world.scene.add(item), bodies

    def _expand(self, q):
        values = np.zeros(len(self.order))
        values[self.arm_indices] = q[:-1]
        values[self.finger_indices] = self.description.hand_position(q[-1])
        return values

    def read(self):
        q, dq = self.robot.get_joint_positions(), self.robot.get_joint_velocities()
        hand, speed = self.description.hand_state(
            float(q[self.finger_indices[0]]),
            float(dq[self.finger_indices[0]]),
        )
        return np.r_[q[self.arm_indices], hand], np.r_[dq[self.arm_indices], speed]

    def write(self, q, velocity=None):
        self._requested = self._expand(q)
        self._velocity = np.zeros(len(self.order))
        if velocity is not None:
            self._velocity[self.arm_indices] = velocity[:-1]
        self.robot.apply_action(
            self.Action(
                joint_positions=self._requested, joint_velocities=self._velocity
            )
        )

    def hold(self):
        self.write(self.read()[0])

    def home(self, q):
        self.robot.set_joint_positions(self._expand(q))
        self.robot.set_joint_velocities(np.zeros(len(self.order)))
        self.write(q)
        return True

    def step(self):
        self.world.step(render=False)

    def reset(self):
        self.world.reset()
        return self.home(self.profile.home)

    def capture(self, name):
        row = self.config["cameras"][name]
        camera, _product, rgb, depth = self.cameras[name]
        matrix = np.asarray(row["transform"]).copy()
        if row["mount"]["kind"] == "wrist":
            position, orientation = self.native_tcp()
            mount = np.eye(4)
            mount[:3, :3], mount[:3, 3] = orientation, position
            matrix = mount @ matrix
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
        # Query the physics view, because USD transforms can lag PhysX/Fabric.
        position, q = self.tcp.get_world_pose()
        quat = self.Gf.Quatd(float(q[0]), self.Gf.Vec3d(*map(float, q[1:])))
        matrix = np.asarray(self.Gf.Matrix3d(quat)).T
        return np.asarray(position).copy(), matrix

    def close(self):
        for _camera, product, rgb, depth in self.cameras.values():
            rgb.detach([product])
            depth.detach([product])
            product.destroy()
        self.world.stop()
        self.app.close()
