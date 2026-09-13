"""Native MuJoCo scene, position drives, and RGB-D sensors."""

from __future__ import annotations

import hashlib
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

from ..robots.mujoco import (
    _evaluation_geometry,
    _evaluation_snapshot,
)
from ..simulation import SimulationVariation
from .description import description
from .model import mjcf, objects
from .randomization import held_out_sample, pose_groups, sample, variation_sample
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
        object_groups = objects(config["environment"], robot=config["robot"])
        self._prop_links = {
            link.name: link
            for group in object_groups
            for link in group
            if link.name != "table"
        }
        self._prop_body_ids = tuple(
            sorted(int(self.model.body(name).id) for name in self._prop_links)
        )
        self._prop_geom_ids = tuple(
            index
            for body_id in self._prop_body_ids
            for index in range(
                int(self.model.body_geomadr[body_id]),
                int(
                    self.model.body_geomadr[body_id] + self.model.body_geomnum[body_id]
                ),
            )
        )
        self._prop_joint_ids = tuple(
            index
            for body_id in self._prop_body_ids
            for index in range(
                int(self.model.body_jntadr[body_id]),
                int(self.model.body_jntadr[body_id] + self.model.body_jntnum[body_id]),
            )
        )
        self._prop_dof_ids = tuple(
            dof
            for joint_id in self._prop_joint_ids
            for dof in range(
                int(self.model.jnt_dofadr[joint_id]),
                (
                    int(self.model.jnt_dofadr[joint_id + 1])
                    if joint_id + 1 < self.model.njnt
                    else self.model.nv
                ),
            )
        )
        self._prop_child_body_ids = tuple(
            int(self.model.body(name).id)
            for group in object_groups
            if group and group[0].name != "table"
            for name in (link.name for link in group[1:])
        )
        for group in object_groups:
            root = group[0]
            if root.kind != "free" or not root.damping:
                continue
            joint_id = int(self.model.body(root.name).jntadr[0])
            dof = int(self.model.jnt_dofadr[joint_id])
            self.model.dof_damping[dof : dof + 6] = root.damping
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
        self._variation_initial = self._capture_variation_initial()
        self._active_variation = dict(SimulationVariation().as_dict())
        self._active_variation_digest = self._variation_digest()
        self._contact_force_buffer = np.zeros(6, dtype=float)
        self._clear_contact_events()

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
        self._latch_contact_events()

    def _clear_contact_events(self):
        self._contact_events = {}
        self._contact_events_through_s = float(self.data.time)

    def _latch_contact_events(self):
        now = float(self.data.time)
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            self.mj.mj_contactForce(
                self.model, self.data, index, self._contact_force_buffer
            )
            force = max(0.0, float(self._contact_force_buffer[0]))
            if force == 0.0:
                continue
            key = tuple(sorted((int(contact.geom1), int(contact.geom2))))
            event = self._contact_events.get(key)
            if event is None:
                event = {
                    "first_time_s": now,
                    "last_time_s": now,
                    "minimum_distance_m": float(contact.dist),
                    "maximum_normal_force_n": force,
                    "contact_samples": 0,
                }
                self._contact_events[key] = event
            event["last_time_s"] = now
            event["minimum_distance_m"] = min(
                event["minimum_distance_m"], float(contact.dist)
            )
            event["maximum_normal_force_n"] = max(
                event["maximum_normal_force_n"], force
            )
            event["contact_samples"] += 1
        self._contact_events_through_s = now

    def reset(self):
        if hasattr(self, "_variation_initial"):
            self._restore_variation_initial()
        if hasattr(self, "_pose_initial"):
            self._restore_pose_initial()
        self.mj.mj_resetData(self.model, self.data)
        if hasattr(self, "_variation_initial"):
            self.mj.mj_setConst(self.model, self.data)
        for part in self.parts:
            self.home(part, self.profile.home)
        self._restore_prop_initial()
        self._active_variation = dict(SimulationVariation().as_dict())
        if hasattr(self, "_variation_initial"):
            self._active_variation_digest = self._variation_digest()
        if hasattr(self, "_contact_events"):
            self._clear_contact_events()
        return True

    def evaluation_reset(self, *, seed, variation=None):
        if isinstance(seed, bool) or not isinstance(seed, int):
            raise TypeError("simulation reset seed must be an integer")
        if seed < 0 or seed > 2**63 - 1:
            raise ValueError("simulation reset seed must be between 0 and 2^63-1")
        profile = (
            SimulationVariation(pose="canonical" if seed == 0 else "bounded")
            if variation is None
            else SimulationVariation.parse(variation)
        )
        if seed == 0 and profile != SimulationVariation():
            raise ValueError("noncanonical variation requires a nonzero seed")
        self.reset()
        if profile.geometry != "canonical":
            self._apply_geometry_randomization(seed, profile.geometry)
        if profile.pose != "canonical":
            self._apply_pose_randomization(seed, profile.pose)
        if profile.appearance != "canonical":
            self._apply_appearance_randomization(seed, profile.appearance)
        if profile.physics != "canonical":
            self._apply_physics_randomization(seed, profile.physics)
        self._active_variation = dict(profile.as_dict())
        self.mj.mj_forward(self.model, self.data)
        self._active_variation_digest = self._variation_digest()
        return self._initial_robot_clearance_valid()

    def _initial_robot_clearance_valid(self):
        """Reject reset states that penetrate a robot into the workcell or peer."""

        workcell_bodies = {*self._prop_body_ids, int(self.model.body("table").id)}
        robot_bodies = set(range(1, self.model.nbody)) - workcell_bodies
        for contact in self.data.contact:
            if contact.dist >= 0:
                continue
            bodies = tuple(
                int(self.model.geom(int(geom)).bodyid[0]) for geom in contact.geom
            )
            if (bodies[0] in robot_bodies) != (bodies[1] in robot_bodies):
                return False
            names = tuple(self.model.body(body).name for body in bodies)
            if (
                names[0].startswith("left__")
                and names[1].startswith("right__")
                or names[0].startswith("right__")
                and names[1].startswith("left__")
            ):
                return False
        return True

    def _capture_variation_initial(self):
        return {
            name: getattr(self.model, name).copy()
            for name in (
                "body_pos",
                "body_mass",
                "body_inertia",
                "dof_damping",
                "geom_pos",
                "geom_size",
                "geom_rgba",
                "geom_friction",
                "jnt_pos",
                "jnt_range",
                "light_ambient",
                "light_diffuse",
                "light_specular",
            )
        }

    def _restore_variation_initial(self):
        for name, value in self._variation_initial.items():
            getattr(self.model, name)[:] = value

    def _refresh_model_constants(self):
        qpos = self.data.qpos.copy()
        qvel = self.data.qvel.copy()
        act = self.data.act.copy()
        ctrl = self.data.ctrl.copy()
        time_s = float(self.data.time)
        self.mj.mj_setConst(self.model, self.data)
        self.data.qpos[:] = qpos
        self.data.qvel[:] = qvel
        self.data.act[:] = act
        self.data.ctrl[:] = ctrl
        self.data.time = time_s
        self.mj.mj_forward(self.model, self.data)

    @staticmethod
    def _level_sample(seed, environment, group, field, level, bound):
        if level == "bounded":
            return variation_sample(seed, environment, group, field, bound)
        return held_out_sample(seed, environment, f"{group}:{field}", bound, bound * 2)

    def _apply_geometry_randomization(self, seed, level):
        environment = self.config["environment"]
        delta = self._level_sample(
            seed, environment, "scene", "geometry_scale", level, 0.02
        )
        scale = 1.0 + delta
        initial = self._variation_initial
        for geom_id in self._prop_geom_ids:
            self.model.geom_size[geom_id] = initial["geom_size"][geom_id] * scale
            self.model.geom_pos[geom_id] = initial["geom_pos"][geom_id] * scale
        for body_id in self._prop_child_body_ids:
            self.model.body_pos[body_id] = initial["body_pos"][body_id] * scale
        for joint_id in self._prop_joint_ids:
            kind = int(self.model.jnt_type[joint_id])
            self.model.jnt_pos[joint_id] = initial["jnt_pos"][joint_id] * scale
            if kind == int(self.mj.mjtJoint.mjJNT_SLIDE):
                self.model.jnt_range[joint_id] = initial["jnt_range"][joint_id] * scale
                address = int(self.model.jnt_qposadr[joint_id])
                self.data.qpos[address] *= scale
        self._refresh_model_constants()

    def _apply_appearance_randomization(self, seed, level):
        environment = self.config["environment"]
        initial = self._variation_initial
        bound = 0.14
        for geom_id in self._prop_geom_ids:
            rgba = initial["geom_rgba"][geom_id].copy()
            for channel in range(3):
                delta = self._level_sample(
                    seed,
                    environment,
                    geom_id,
                    f"appearance_{channel}",
                    level,
                    bound,
                )
                rgba[channel] = np.clip(rgba[channel] * (1.0 + delta), 0.0, 1.0)
            self.model.geom_rgba[geom_id] = rgba
        lighting_delta = self._level_sample(
            seed, environment, "scene", "lighting", level, 0.12
        )
        lighting = 1.0 + lighting_delta
        for name in ("light_ambient", "light_diffuse", "light_specular"):
            getattr(self.model, name)[:] = np.clip(initial[name] * lighting, 0.0, 1.0)

    def _apply_physics_randomization(self, seed, level):
        environment = self.config["environment"]
        initial = self._variation_initial
        for body_id in self._prop_body_ids:
            delta = self._level_sample(seed, environment, body_id, "mass", level, 0.15)
            factor = 1.0 + delta
            self.model.body_mass[body_id] = initial["body_mass"][body_id] * factor
            self.model.body_inertia[body_id] = initial["body_inertia"][body_id] * factor
        for geom_id in self._prop_geom_ids:
            delta = self._level_sample(
                seed, environment, geom_id, "friction", level, 0.15
            )
            self.model.geom_friction[geom_id] = np.maximum(
                initial["geom_friction"][geom_id] * (1.0 + delta), 1e-6
            )
        prop_bodies = set(self._prop_body_ids)
        for joint_id in self._prop_joint_ids:
            address = int(self.model.jnt_dofadr[joint_id])
            next_address = (
                int(self.model.jnt_dofadr[joint_id + 1])
                if joint_id + 1 < self.model.njnt
                else self.model.nv
            )
            body_id = int(self.model.jnt_bodyid[joint_id])
            if body_id not in prop_bodies:
                continue
            delta = self._level_sample(
                seed, environment, joint_id, "damping", level, 0.15
            )
            self.model.dof_damping[address:next_address] = initial["dof_damping"][
                address:next_address
            ] * (1.0 + delta)
        self._refresh_model_constants()

    def _variation_digest(self):
        digest = hashlib.sha256(
            json.dumps(
                self._active_variation, sort_keys=True, separators=(",", ":")
            ).encode()
        )
        selections = (
            ("body_pos", self._prop_body_ids),
            ("body_mass", self._prop_body_ids),
            ("body_inertia", self._prop_body_ids),
            ("dof_damping", self._prop_dof_ids),
            ("geom_pos", self._prop_geom_ids),
            ("geom_size", self._prop_geom_ids),
            ("geom_rgba", self._prop_geom_ids),
            ("geom_friction", self._prop_geom_ids),
            ("jnt_pos", self._prop_joint_ids),
            ("jnt_range", self._prop_joint_ids),
        )
        for name, indices in selections:
            digest.update(name.encode())
            values = np.asarray(getattr(self.model, name)[list(indices)], dtype="<f8")
            digest.update(values.tobytes())
        for name in ("light_ambient", "light_diffuse", "light_specular"):
            digest.update(name.encode())
            digest.update(np.asarray(getattr(self.model, name), dtype="<f8").tobytes())
        for group in self._pose_groups:
            for name in group.bodies:
                digest.update(name.encode())
                digest.update(
                    np.asarray(self._current_pose(name), dtype="<f8").tobytes()
                )
        return "sha256:" + digest.hexdigest()

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

    def _current_pose(self, name):
        free, address, _dof_address, _pose = self._pose_initial[name]
        if free:
            return self.data.qpos[address : address + 7].copy()
        body = self.model.body(name)
        return np.r_[body.pos.copy(), body.quat.copy()]

    def _apply_pose_randomization(self, seed, level="bounded"):
        environment = self.config["environment"]
        for index, group in enumerate(self._pose_groups):
            if level == "bounded":
                dx = sample(seed, environment, index, "x", group.translation_xy_m)
                dy = sample(seed, environment, index, "y", group.translation_xy_m)
                yaw = sample(seed, environment, index, "yaw", group.yaw_rad)
            else:
                dx = held_out_sample(
                    seed,
                    environment,
                    f"pose:{index}:x",
                    group.translation_xy_m,
                    group.translation_xy_m * 1.5,
                )
                dy = held_out_sample(
                    seed,
                    environment,
                    f"pose:{index}:y",
                    group.translation_xy_m,
                    group.translation_xy_m * 1.5,
                )
                yaw = held_out_sample(
                    seed,
                    environment,
                    f"pose:{index}:yaw",
                    group.yaw_rad,
                    group.yaw_rad * 1.5,
                )
            originals = [self._current_pose(name) for name in group.bodies]
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
        snapshot["variation"] = {
            "profile": dict(self._active_variation),
            "resolved_digest": self._active_variation_digest,
        }
        snapshot["contact_events"] = {
            "schema": "waddle.simulation-contact-events/mujoco-v1",
            "through_time_s": self._contact_events_through_s,
            "pairs": [
                {
                    **self._contact_events[key],
                    "first": _evaluation_geometry(
                        mj=self.mj, model=self.model, identifier=key[0]
                    ),
                    "second": _evaluation_geometry(
                        mj=self.mj, model=self.model, identifier=key[1]
                    ),
                }
                for key in sorted(self._contact_events)
            ],
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
