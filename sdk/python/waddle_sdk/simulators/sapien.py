"""SAPIEN 3 scene adapter; all Vulkan calls stay on the worker main thread."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import replace
from pathlib import Path

import numpy as np

from .description import description
from .model import SCREW_PITCH, objects, robot_links, urdf
from .scene import depth_z16, profile, quaternion, rotation


class Engine:
    def __init__(self, config: dict, scratch: Path):
        import sapien

        self.sp = sapien
        self.config = config
        self.profile = p = profile(config["robot"])
        self.description = robot = description(p.name)
        self.scene = sapien.Scene()
        self.scene.set_timestep(config["timestep"])
        self.scene.set_ambient_light([0.18, 0.20, 0.24])
        quality = config.get("render_quality", "standard")
        self.scene.add_directional_light(
            [0.3, 0.4, -1],
            [2.0, 1.85, 1.7],
            shadow=quality != "fast",
            shadow_map_size={"fast": 512, "standard": 2048, "high": 4096}[quality],
        )
        self.scene.add_directional_light([-0.4, -0.4, -1], [0.3, 0.36, 0.45])
        links = robot_links(p)
        master = robot.hand_names[0]
        self.hand_drives = {master} | {
            link.joint for link in links if link.mimic == (master, 1.0, 0.0)
        }
        # Use ManiSkill's PDJointPosMimicController pattern. SAPIEN's URDF
        # mimic tendon can oscillate under grasp contact. Share the single
        # actuator's gains, force budget and reflected inertia across its jaws;
        # passive four-bar links keep their native closure constraints.
        self.robot = self._load(
            [
                replace(link, mimic=None) if link.joint in self.hand_drives else link
                for link in links
            ],
            "robot",
            scratch,
        )
        self.robot.set_solver_position_iterations(15)
        self.robot.set_solver_velocity_iterations(1)
        self.joints = {j.name: j for j in self.robot.get_active_joints()}
        self.order = [j.name for j in self.robot.get_active_joints()]
        self.arm_indices = [self.order.index(n) for n in p.names[:-1]]
        self.finger_indices = [self.order.index(n) for n in robot.hand_names]
        for name, joint in self.joints.items():
            # PDJointPosController explicitly sets friction=0 by default;
            # SAPIEN's loader otherwise inserts joint friction=0.05.
            joint.set_friction(0.0)
            source = master if name in self.hand_drives else name
            share = len(self.hand_drives) if name in self.hand_drives else 1
            joint.set_armature([robot.armature(source) / share])
            kp, kd, effort = (value / share for value in robot.servo(source))
            joint.set_drive_properties(kp, kd, force_limit=effort)
        self.drives = []
        links = {link.name: link for link in self.robot.get_links()}
        for first, anchor, second, other in robot.closures():
            drive = self.scene.create_drive(
                links[first], sapien.Pose(anchor), links[second], sapien.Pose(other)
            )
            drive.set_limit_x(0, 0)
            drive.set_limit_y(0, 0)
            drive.set_limit_z(0, 0)
            self.drives.append(drive)
        self.home(p.home)
        self.props = [
            self._load(group, group[0].name, scratch)
            for group in objects(config["environment"])
        ]
        self._screw = self.props[-1] if config["environment"] == "bottle_cap" else None
        self._initial_state = self.scene.get_physx_system().pack()
        self.cameras = {}
        for name, row in config["cameras"].items():
            intr, stream = row["intrinsics"], row["stream"]
            camera = self.scene.add_camera(
                name, stream["width"], stream["height"], 1.0, 0.01, 10.0
            )
            camera.set_perspective_parameters(
                0.01, 10.0, intr["fx"], intr["fy"], intr["cx"], intr["cy"], 0.0
            )
            self.cameras[name] = camera

    def _load(self, links, name, scratch):
        if len(links) > 1:
            path = scratch / f"{name}.urdf"
            path.write_text(urdf(links, name))
            if name == "robot":
                srdf = ET.Element("robot", name=name)
                for first, second in self.description.exclusions:
                    ET.SubElement(
                        srdf,
                        "disable_collisions",
                        link1=first,
                        link2=second,
                        reason="Default",
                    )
                path.with_suffix(".srdf").write_text(
                    ET.tostring(srdf, encoding="unicode")
                )
            loader = self.scene.create_urdf_loader()
            loader.set_material(1.2, 1.0, 0.0)
            loader.fix_root_link = True
            # Each URDF collision element names one convex mesh; multiple
            # elements remain supported. STL has no submesh partition format.
            loader.load_multiple_collisions_from_file = False
            if name == "bottle":
                # A helical thread couples metres to radians. Use the native
                # tendon with work-conjugate force coefficients, rather than
                # the URDF loader's inverse-ratio mimic-force convention.
                builders, _, _ = loader.parse(str(path))
                entities = builders[0].build_entities(fix_root_link=True)
                chain = [
                    entity.find_component_by_type(
                        self.sp.physx.PhysxArticulationLinkComponent
                    )
                    for entity in entities
                ]
                result = chain[0].articulation
                coefficients = [0, 1, -SCREW_PITCH]
                result.create_fixed_tendon(
                    chain, coefficients, coefficients, stiffness=5000, damping=20
                )
                for entity in entities:
                    self.scene.add_entity(entity)
            else:
                result = loader.load(str(path))
            if result is None:
                raise RuntimeError(f"SAPIEN could not load {name}")
            result.set_root_pose(
                self.sp.Pose(links[0].xyz, quaternion(rotation(links[0].rpy)))
            )
            for link in result.get_links():
                visual = link.entity.find_component_by_type(
                    self.sp.render.RenderBodyComponent
                )
                if visual:
                    for shape in visual.render_shapes:
                        for part in shape.parts:
                            part.material.specular = 0.5
                            part.material.roughness = 0.35 if name == "robot" else 0.65
                            part.material.metallic = 0.1 if name == "robot" else 0.0
            return result
        link = links[0]
        builder = self.scene.create_actor_builder()
        material = self.scene.create_physical_material(1.2, 1.0, 0.0)
        for shape in link.shapes:
            pose = self.sp.Pose(shape.xyz, quaternion(rotation(shape.rpy)))
            finish = self.sp.render.RenderMaterial(
                base_color=shape.color, roughness=0.65, specular=0.3
            )
            if shape.texture:
                finish.base_color_texture = self.sp.render.RenderTexture2D(
                    shape.texture
                )
            if shape.kind == "box":
                half = np.asarray(shape.size) / 2
                builder.add_box_collision(pose=pose, half_size=half, material=material)
                builder.add_box_visual(pose=pose, half_size=half, material=finish)
            elif shape.kind == "sphere":
                builder.add_sphere_collision(
                    pose=pose, radius=shape.size[0], material=material
                )
                builder.add_sphere_visual(
                    pose=pose, radius=shape.size[0], material=finish
                )
            else:
                raise ValueError("single-body reference props use box/sphere geometry")
        result = (
            builder.build(name=name)
            if link.kind == "free"
            else builder.build_static(name=name)
        )
        result.set_pose(self.sp.Pose(link.xyz, quaternion(rotation(link.rpy))))
        if link.kind == "free":
            result.find_component_by_type(
                self.sp.physx.PhysxRigidDynamicComponent
            ).mass = link.mass
        return result

    def _expand(self, q):
        values = np.zeros(len(self.order))
        values[self.arm_indices] = q[:-1]
        values[self.finger_indices] = self.description.hand_position(q[-1])
        return values

    def read(self):
        q, dq = self.robot.get_qpos(), self.robot.get_qvel()
        hand, speed = self.description.hand_state(
            float(q[self.finger_indices[0]]),
            float(dq[self.finger_indices[0]]),
        )
        return np.r_[q[self.arm_indices], hand], np.r_[dq[self.arm_indices], speed]

    def write(self, q, velocity=None):
        self._requested = self._expand(q)
        self._velocity = (
            np.zeros(self.profile.dof)
            if velocity is None
            else np.asarray(velocity[:-1])
        )
        for i, name in enumerate(self.profile.names[:-1]):
            self.joints[name].set_drive_velocity_target(float(self._velocity[i]))
        for name, joint in self.joints.items():
            joint.set_drive_target(float(self._requested[self.order.index(name)]))

    def hold(self):
        self.write(self.read()[0])

    def home(self, q):
        self.robot.set_qpos(self._expand(q))
        self.robot.set_qvel(np.zeros(len(self.order)))
        self.write(q)
        return True

    def step(self):
        self.robot.set_qf(
            self.robot.compute_passive_force(
                gravity=True, coriolis_and_centrifugal=False
            )
        )
        self.scene.step()

    def reset(self):
        self.scene.get_physx_system().unpack(self._initial_state)
        self.write(self.profile.home)
        return True

    def capture(self, name):
        row = self.config["cameras"][name]
        matrix = np.asarray(row["transform"])
        if row["mount"]["kind"] == "wrist":
            tcp = next(link for link in self.robot.get_links() if link.name == "tcp")
            matrix = tcp.get_entity_pose().to_transformation_matrix() @ matrix
        # SAPIEN camera entity: +X forward, +Y left, +Z up.
        native = matrix[:3, :3] @ np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]])
        camera = self.cameras[name]
        camera.set_entity_pose(self.sp.Pose(matrix[:3, 3], quaternion(native)))
        self.scene.update_render()
        camera.take_picture()
        color = camera.get_picture("Color")
        position = camera.get_picture("Position")
        rgb = np.rint(np.clip(color[..., :3], 0, 1) * 255).astype(np.uint8)
        depth = -position[..., 2]
        depth[position[..., 3] >= 1] = 0
        return rgb, depth_z16(depth, row["intrinsics"]["depth_scale_mm"])

    def native_tcp(self):
        pose = (
            next(link for link in self.robot.get_links() if link.name == "tcp")
            .get_entity_pose()
            .to_transformation_matrix()
        )
        return pose[:3, 3].copy(), pose[:3, :3].copy()

    def close(self):
        self.cameras.clear()
        self.drives.clear()
        self.props.clear()
        self.robot = None
        self.scene = None
