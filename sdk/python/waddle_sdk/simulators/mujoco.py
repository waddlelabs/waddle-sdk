"""Native MuJoCo scene, position drives, and RGB-D sensors."""

from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np

from .description import description
from .model import mjcf
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
        self.renderers = {}
        for part in self.parts:
            self.home(part, p.home)

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
        self.mj.mj_resetData(self.model, self.data)
        for part in self.parts:
            self.home(part, self.profile.home)
        return True

    def evaluation_reset(self, *, seed):
        # Current canonical scenes are deterministic. The explicit seed is part
        # of the stable facet now and will drive task randomizers as they land.
        if isinstance(seed, bool) or not isinstance(seed, int):
            raise TypeError("simulation reset seed must be an integer")
        if seed < 0 or seed > 2**63 - 1:
            raise ValueError("simulation reset seed must be between 0 and 2^63-1")
        return self.reset()

    def evaluation_snapshot(self):
        """Return backend ground truth to the isolated trusted evaluator."""

        self.mj.mj_forward(self.model, self.data)
        joint_types = {
            int(self.mj.mjtJoint.mjJNT_FREE): "free",
            int(self.mj.mjtJoint.mjJNT_BALL): "ball",
            int(self.mj.mjtJoint.mjJNT_SLIDE): "slide",
            int(self.mj.mjtJoint.mjJNT_HINGE): "hinge",
        }
        joints = {}
        for index in range(self.model.njnt):
            name = self.mj.mj_id2name(self.model, self.mj.mjtObj.mjOBJ_JOINT, index)
            qpos_start = int(self.model.jnt_qposadr[index])
            qpos_end = (
                int(self.model.jnt_qposadr[index + 1])
                if index + 1 < self.model.njnt
                else self.model.nq
            )
            dof_start = int(self.model.jnt_dofadr[index])
            dof_end = (
                int(self.model.jnt_dofadr[index + 1])
                if index + 1 < self.model.njnt
                else self.model.nv
            )
            joints[name or f"joint:{index}"] = {
                "type": joint_types[int(self.model.jnt_type[index])],
                "qpos": [float(value) for value in self.data.qpos[qpos_start:qpos_end]],
                "qvel": [float(value) for value in self.data.qvel[dof_start:dof_end]],
            }

        bodies = {}
        for index in range(self.model.nbody):
            name = self.mj.mj_id2name(self.model, self.mj.mjtObj.mjOBJ_BODY, index)
            velocity = np.zeros(6, dtype=float)
            self.mj.mj_objectVelocity(
                self.model,
                self.data,
                self.mj.mjtObj.mjOBJ_BODY,
                index,
                velocity,
                0,
            )
            bodies[name or f"body:{index}"] = {
                "position_m": [float(value) for value in self.data.xpos[index]],
                "orientation_wxyz": [float(value) for value in self.data.xquat[index]],
                "angular_velocity_rad_s": [float(value) for value in velocity[:3]],
                "linear_velocity_m_s": [float(value) for value in velocity[3:]],
            }

        contacts = []
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            force = np.zeros(6, dtype=float)
            self.mj.mj_contactForce(self.model, self.data, index, force)
            first_geom, second_geom = int(contact.geom1), int(contact.geom2)

            def geom(identifier):
                name = self.mj.mj_id2name(
                    self.model, self.mj.mjtObj.mjOBJ_GEOM, identifier
                )
                body_id = int(self.model.geom_bodyid[identifier])
                body = self.mj.mj_id2name(
                    self.model, self.mj.mjtObj.mjOBJ_BODY, body_id
                )
                return {
                    "geom": name or f"geom:{identifier}",
                    "body": body or f"body:{body_id}",
                }

            contacts.append(
                {
                    "first": geom(first_geom),
                    "second": geom(second_geom),
                    "distance_m": float(contact.dist),
                    "normal_force_n": float(force[0]),
                }
            )
        return {
            "schema": "waddle.simulation-state/mujoco-v1",
            "backend": "mujoco",
            "identity": {
                "provider": "mujoco",
                "provider_revision": self.mj.__version__,
                "robot_family": self.config["robot"],
                "embodiment_revision": self.config.get("embodiment_revision"),
                "arm_count": len(self.parts),
                "environment_id": self.config["environment"],
                "scene_revision": self.config.get("scene_revision"),
                "asset_revision": self.config.get("asset_revision"),
            },
            "time_s": float(self.data.time),
            "joints": joints,
            "bodies": bodies,
            "contacts": contacts,
        }

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
