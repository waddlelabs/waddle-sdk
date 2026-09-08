"""SAPIEN 3 scene adapter; all Vulkan calls stay on the worker main thread."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from .model import objects, robot_links, screw_force, urdf
from .scene import depth_z16, profile, quaternion, rotation


class Engine:
    def __init__(self, config: dict, scratch: Path):
        import sapien

        self.sp = sapien
        self.config = config
        self.profile = p = profile(config["robot"])
        self.scene = sapien.Scene()
        self.scene.set_timestep(config["timestep"])
        self.scene.set_ambient_light([0.5, 0.5, 0.5])
        self.scene.add_directional_light([0, -0.5, -1], [1.0, 1.0, 1.0], shadow=True)
        self.robot = self._load(robot_links(p), "robot", scratch)
        self.joints = {j.name: j for j in self.robot.get_active_joints()}
        self.order = [j.name for j in self.robot.get_active_joints()]
        self.arm_indices = [self.order.index(n) for n in p.names[:-1]]
        self.finger_indices = [
            self.order.index(n) for n in ("left_finger", "right_finger")
        ]
        for name, joint in self.joints.items():
            finger = name.endswith("_finger")
            joint.set_drive_properties(
                1500 if finger else 5000,
                15 if finger else 70,
                force_limit=20 if finger else 50,
            )
        self.home(p.home)
        self.props = [
            self._load(group, group[0].name, scratch)
            for group in objects(config["environment"])
        ]
        self._screw = self.props[-1] if config["environment"] == "bottle_cap" else None
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
            loader = self.scene.create_urdf_loader()
            loader.set_material(1.2, 1.0, 0.0)
            loader.fix_root_link = True
            loader.load_multiple_collisions_from_file = True
            result = loader.load(str(path))
            if result is None:
                raise RuntimeError(f"SAPIEN could not load {name}")
            result.set_root_pose(
                self.sp.Pose(links[0].xyz, quaternion(rotation(links[0].rpy)))
            )
            return result
        link = links[0]
        builder = self.scene.create_actor_builder()
        material = self.scene.create_physical_material(1.2, 1.0, 0.0)
        for shape in link.shapes:
            pose = self.sp.Pose(shape.xyz, quaternion(rotation(shape.rpy)))
            if shape.kind == "box":
                half = np.asarray(shape.size) / 2
                builder.add_box_collision(pose=pose, half_size=half, material=material)
                builder.add_box_visual(
                    pose=pose, half_size=half, material=shape.color[:3]
                )
            elif shape.kind == "sphere":
                builder.add_sphere_collision(
                    pose=pose, radius=shape.size[0], material=material
                )
                builder.add_sphere_visual(
                    pose=pose, radius=shape.size[0], material=shape.color[:3]
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
        values[self.finger_indices] = q[-1] * self.profile.opening / 2
        return values

    def read(self):
        q, dq = self.robot.get_qpos(), self.robot.get_qvel()
        return np.r_[
            q[self.arm_indices], sum(q[self.finger_indices]) / self.profile.opening
        ], np.r_[
            dq[self.arm_indices], sum(dq[self.finger_indices]) / self.profile.opening
        ]

    def write(self, q):
        values = self._expand(q)
        for name, joint in self.joints.items():
            joint.set_drive_target(float(values[self.order.index(name)]))

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
        if self._screw is not None:
            names = [j.name for j in self._screw.get_active_joints()]
            idx = [names.index(n) for n in ("cap_rotation", "cap_lift")]
            force = np.zeros(len(names))
            force[idx] = screw_force(
                self._screw.get_qpos()[idx], self._screw.get_qvel()[idx]
            )
            self._screw.set_qf(force)
        self.scene.step()

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
        self.props.clear()
        self.robot = None
        self.scene = None
