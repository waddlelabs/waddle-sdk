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
