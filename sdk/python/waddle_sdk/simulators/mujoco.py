"""Native MuJoCo scene, position drives, and RGB-D sensors."""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

from ..robots.mujoco import _evaluation_snapshot
from .description import description
from .model import mjcf, objects
from .randomization import pose_groups, sample
from .scene import depth_z16, profile


class Engine:
    def __init__(self, config: dict, scratch: Path):
        # Selection precedes the first MuJoCo import and is local to this worker.
        if sys.platform == "linux" and not os.environ.get("DISPLAY"):
            os.environ.setdefault("MUJOCO_GL", "egl")
        import mujoco

        self.mj = mujoco
        self.config = config
        self.profile = p = profile(config["robot"])
        self.description = robot = description(p.name)
        path = scratch / "scene.xml"
        path.write_text(mjcf(p, config))
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.data = mujoco.MjData(self.model)
        self.parts = tuple(config["parts"])
        self._qpos = {}
        self._dofs = {}
        self._controls = {}
        for part in self.parts:
            prefix = "" if len(self.parts) == 1 else f"{part}__"
            joints = [
                self.model.joint(f"{prefix}{name}")
                for name in p.names[:-1] + robot.hand_names
            ]
            self._qpos[part] = [int(j.qposadr[0]) for j in joints]
            self._dofs[part] = [int(j.dofadr[0]) for j in joints]
            self._controls[part] = tuple(
                f"{prefix}{name}" for name in p.names[:-1] + robot.hand_names[:1]
            )
        native_controls = p.names[:-1] + robot.hand_names[:1]
        self._velocity_ratio = np.array(
            [robot.servo(name)[1] / robot.servo(name)[0] for name in native_controls]
        )
        object_groups = objects(config["environment"])
        self._prop_initial = {
            link.joint: float(link.initial)
            for group in object_groups
            for link in group
            if link.joint is not None
        }
        self.renderers = {}
        for part in self.parts:
            self.home(part, p.home)
        self._restore_prop_initial()
        self._pose_groups = pose_groups(config["environment"], object_groups)
        self._pose_initial = self._capture_pose_initial()

    def _restore_prop_initial(self):
        for name, value in self._prop_initial.items():
            joint = self.model.joint(name)
            self.data.qpos[int(joint.qposadr[0])] = value
            self.data.qvel[int(joint.dofadr[0])] = 0.0
        self.mj.mj_forward(self.model, self.data)

    def _part(self, part=None):
        if part is None:
            if len(self.parts) != 1:
                raise ValueError("a robot part is required in a multi-arm world")
            return self.parts[0]
        if part not in self.parts:
            raise ValueError("robot part is absent from the simulation")
        return part

    def read(self, part=None):
        part = self._part(part)
        q, dq = self.data.qpos[self._qpos[part]], self.data.qvel[self._dofs[part]]
        n = self.profile.dof
        # Hardware reports the driven joint encoder, not passive linkage motion.
        hand, speed = self.description.hand_state(float(q[n]), float(dq[n]))
        return np.r_[q[:n], hand], np.r_[dq[:n], speed]

    def write(self, part_or_q, q_or_velocity=None, velocity=None):
        if isinstance(part_or_q, str):
            part, q = self._part(part_or_q), q_or_velocity
        else:
            part, q, velocity = self._part(), part_or_q, q_or_velocity
        # Native PD(q, dq): position + kv/kp * desired velocity is exactly
        # equivalent to kp*(target-q) + kv*(desired_velocity-dq).
        target = np.r_[q[:-1], self.description.hand_position(q[-1])]
        if velocity is not None:
            target += self._velocity_ratio * np.r_[velocity[:-1], 0.0]
        for name, value in zip(self._controls[part], target, strict=True):
            self.data.actuator(name).ctrl[0] = value

    def hold(self, part=None):
        parts = self.parts if part is None else (self._part(part),)
        for name in parts:
            self.write(name, self.read(name)[0])

    def home(self, part_or_q, q=None):
        if isinstance(part_or_q, str):
            part = self._part(part_or_q)
        else:
            part, q = self._part(), part_or_q
        self.data.qpos[self._qpos[part]] = self.description.expand(q)
        self.data.qvel[self._dofs[part]] = 0
        self.write(part, q)
        self.mj.mj_forward(self.model, self.data)
        return True

    def step(self):
        self.mj.mj_step(self.model, self.data)

    def reset(self):
        if hasattr(self, "_pose_initial"):
            self._restore_pose_initial()
        self.mj.mj_resetData(self.model, self.data)
        for part in self.parts:
            self.home(part, self.profile.home)
        self._restore_prop_initial()
        return True

    def evaluation_reset(self, *, seed):
        if isinstance(seed, bool) or not isinstance(seed, int):
            raise TypeError("simulation reset seed must be an integer")
        if seed < 0 or seed > 2**63 - 1:
            raise ValueError("simulation reset seed must be between 0 and 2^63-1")
        self.reset()
        self._apply_pose_randomization(seed)
        return True

    def _capture_pose_initial(self):
        initial = {}
        for group in self._pose_groups:
            for name in group.bodies:
                body = self.model.body(name)
                joint_id = int(body.jntadr[0]) if int(body.jntnum[0]) else None
                free = joint_id is not None and int(
                    self.model.jnt_type[joint_id]
                ) == int(self.mj.mjtJoint.mjJNT_FREE)
                if free:
                    address = int(self.model.jnt_qposadr[joint_id])
                    pose = self.data.qpos[address : address + 7].copy()
                    dof_address = int(self.model.jnt_dofadr[joint_id])
                else:
                    address = None
                    pose = np.r_[body.pos.copy(), body.quat.copy()]
                    dof_address = None
                initial[name] = (free, address, dof_address, pose)
        return initial

    def _restore_pose_initial(self):
        for name, (free, address, _dof_address, pose) in self._pose_initial.items():
            if free:
                self.data.qpos[address : address + 7] = pose
            else:
                body = self.model.body(name)
                body.pos[:] = pose[:3]
                body.quat[:] = pose[3:]

    @staticmethod
    def _yaw_quaternion(angle):
        return np.array([math.cos(angle / 2), 0.0, 0.0, math.sin(angle / 2)])

    @staticmethod
    def _quaternion_multiply(first, second):
        aw, ax, ay, az = first
        bw, bx, by, bz = second
        return np.array(
            [
                aw * bw - ax * bx - ay * by - az * bz,
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
            ]
        )

    def _apply_pose_randomization(self, seed):
        environment = self.config["environment"]
        for index, group in enumerate(self._pose_groups):
            dx = sample(seed, environment, index, "x", group.translation_xy_m)
            dy = sample(seed, environment, index, "y", group.translation_xy_m)
            yaw = sample(seed, environment, index, "yaw", group.yaw_rad)
            originals = [self._pose_initial[name][3] for name in group.bodies]
            anchor = np.mean([pose[:2] for pose in originals], axis=0)
            cosine, sine = math.cos(yaw), math.sin(yaw)
            rotation = np.array([[cosine, -sine], [sine, cosine]])
            yaw_quaternion = self._yaw_quaternion(yaw)
            for name, original in zip(group.bodies, originals, strict=True):
                position = original[:3].copy()
                position[:2] = anchor + rotation @ (position[:2] - anchor) + (dx, dy)
                orientation = self._quaternion_multiply(yaw_quaternion, original[3:])
                free, address, dof_address, _pose = self._pose_initial[name]
                if free:
                    self.data.qpos[address : address + 3] = position
                    self.data.qpos[address + 3 : address + 7] = orientation
                    self.data.qvel[dof_address : dof_address + 6] = 0.0
                else:
                    body = self.model.body(name)
                    body.pos[:] = position
                    body.quat[:] = orientation
        self.mj.mj_forward(self.model, self.data)

    def evaluation_snapshot(self):
        """Return backend ground truth to the isolated trusted evaluator."""

        snapshot = _evaluation_snapshot(mj=self.mj, model=self.model, data=self.data)
        snapshot["identity"] = {
            "provider": "mujoco",
            "provider_revision": self.mj.__version__,
            "robot_family": self.config["robot"],
            "embodiment_revision": self.config.get("embodiment_revision"),
            "arm_count": len(self.parts),
            "environment_id": self.config["environment"],
            "scene_revision": self.config.get("scene_revision"),
            "asset_revision": self.config.get("asset_revision"),
        }
        return snapshot

    def capture(self, name):
        row = self.config["cameras"][name]
        if name not in self.renderers:
            self.renderers[name] = self.mj.Renderer(
                self.model, height=row["stream"]["height"], width=row["stream"]["width"]
            )
        renderer = self.renderers[name]
        option = self.mj.MjvOption()
        option.geomgroup[3] = 0  # collisions do not replace the manufacturer's visuals
        renderer.update_scene(self.data, camera=name, scene_option=option)
        renderer.disable_depth_rendering()
        rgb = renderer.render().copy()
        if not row["options"]["depth"]:
            return rgb, None
        renderer.enable_depth_rendering()
        depth = renderer.render().copy()
        renderer.disable_depth_rendering()
        far = self.model.vis.map.zfar * self.model.stat.extent
        depth[depth >= far * 0.999] = 0
        return rgb, depth_z16(depth, row["intrinsics"]["depth_scale_mm"])

    def native_tcp(self, part=None):
        part = self._part(part)
        self.mj.mj_forward(self.model, self.data)
        prefix = "" if len(self.parts) == 1 else f"{part}__"
        site = self.data.site(f"{prefix}tcp_site")
        return site.xpos.copy(), site.xmat.reshape(3, 3).copy()

    def close(self):
        for renderer in self.renderers.values():
            renderer.close()
        self.renderers.clear()
