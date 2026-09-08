"""MuJoCo renderer/scene extension of the existing joint-target driver."""

from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np

from .model import mjcf, screw_force
from .scene import depth_z16, profile


class Engine:
    def __init__(self, config: dict, scratch: Path):
        # Selection precedes the first MuJoCo import and is local to this worker.
        if sys.platform == "linux" and not os.environ.get("DISPLAY"):
            os.environ.setdefault("MUJOCO_GL", "egl")
        from ..robots.mujoco import MujocoDriver

        self.config = config
        self.profile = p = profile(config["robot"])
        path = scratch / "scene.xml"
        path.write_text(mjcf(p, config))
        self.driver = MujocoDriver(
            model_path=path,
            joint_names=p.names[:-1] + ("left_finger", "right_finger"),
            joint_limits=p.limits[:-1] + ((0.0, p.opening / 2),) * 2,
            actuator_names=p.names[:-1] + ("left_finger", "right_finger"),
            home=self._expand(p.home),
            tool_site="tcp_site",
            collision_bodies=(),
        )
        self.mj, self.model, self.data = (
            self.driver._mj,
            self.driver._model,
            self.driver._data,
        )
        self.renderers = {}
        self._screw = None
        if config["environment"] == "bottle_cap":
            self._screw = tuple(self.model.joint(name) for name in ("cap_rotation", "cap_lift"))

    def _expand(self, q):
        return tuple(q[:-1]) + (q[-1] * self.profile.opening / 2,) * 2

    def read(self):
        q, dq = self.driver.read()
        return np.r_[q[:-2], (q[-2] + q[-1]) / self.profile.opening], np.r_[
            dq[:-2], (dq[-2] + dq[-1]) / self.profile.opening
        ]

    def write(self, q):
        self.driver.write(np.asarray(self._expand(q)))

    def hold(self):
        self.driver.hold()

    def home(self, q):
        return self.driver.home(self._expand(q))

    def step(self):
        if self._screw is not None:
            addresses = [int(j.qposadr[0]) for j in self._screw]
            dofs = [int(j.dofadr[0]) for j in self._screw]
            self.data.qfrc_applied[dofs] = screw_force(
                self.data.qpos[addresses], self.data.qvel[dofs]
            )
        self.driver.step(self.config["timestep"])

    def capture(self, name):
        row = self.config["cameras"][name]
        if name not in self.renderers:
            self.renderers[name] = self.mj.Renderer(
                self.model, height=row["stream"]["height"], width=row["stream"]["width"]
            )
        renderer = self.renderers[name]
        renderer.update_scene(self.data, camera=name)
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
        self.driver.close()
