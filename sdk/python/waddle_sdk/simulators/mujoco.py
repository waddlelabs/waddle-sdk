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
        joints = [self.model.joint(name) for name in p.names[:-1] + robot.hand_names]
        self._qpos = [int(j.qposadr[0]) for j in joints]
        self._dofs = [int(j.dofadr[0]) for j in joints]
        self._controls = p.names[:-1] + robot.hand_names[:1]
        self._velocity_ratio = np.array(
            [robot.servo(name)[1] / robot.servo(name)[0] for name in self._controls]
        )
        self.renderers = {}
        self._screw = None
        if config["environment"] == "bottle_cap":
            self._screw = tuple(
                self.model.joint(name) for name in ("cap_rotation", "cap_lift")
            )
        self.home(p.home)

    def read(self):
        q, dq = self.data.qpos[self._qpos], self.data.qvel[self._dofs]
        n = self.profile.dof
        # Hardware reports the driven joint encoder, not passive linkage motion.
        hand, speed = self.description.hand_state(float(q[n]), float(dq[n]))
        return np.r_[q[:n], hand], np.r_[dq[:n], speed]

    def write(self, q, velocity=None):
        # Native PD(q, dq): position + kv/kp * desired velocity is exactly
        # equivalent to kp*(target-q) + kv*(desired_velocity-dq).
        target = np.r_[q[:-1], self.description.hand_position(q[-1])]
        if velocity is not None:
            target += self._velocity_ratio * np.r_[velocity[:-1], 0.0]
        for name, value in zip(self._controls, target, strict=True):
            self.data.actuator(name).ctrl[0] = value

    def hold(self):
        self.write(self.read()[0])

    def home(self, q):
        self.data.qpos[self._qpos] = self.description.expand(q)
        self.data.qvel[self._dofs] = 0
        self.write(q)
        self.mj.mj_forward(self.model, self.data)
        return True

    def step(self):
        self.mj.mj_step(self.model, self.data)

    def reset(self):
        self.mj.mj_resetData(self.model, self.data)
        return self.home(self.profile.home)

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
        renderer.enable_depth_rendering()
        depth = renderer.render().copy()
        renderer.disable_depth_rendering()
        far = self.model.vis.map.zfar * self.model.stat.extent
        depth[depth >= far * 0.999] = 0
        return rgb, depth_z16(depth, row["intrinsics"]["depth_scale_mm"])

    def native_tcp(self):
        self.mj.mj_forward(self.model, self.data)
        site = self.data.site("tcp_site")
        return site.xpos.copy(), site.xmat.reshape(3, 3).copy()

    def close(self):
        for renderer in self.renderers.values():
            renderer.close()
        self.renderers.clear()
