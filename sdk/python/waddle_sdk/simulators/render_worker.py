"""Private read-only renderer for immutable snapshots of a simulation world."""

from __future__ import annotations

import os
import pickle
import sys
import traceback
from multiprocessing.connection import Connection

from ..cameras._mujoco_render import RGBDRenderer
from .scene import depth_z16


class SnapshotRenderer:
    def __init__(self, config):
        import mujoco

        self.mj = mujoco
        self.config = config
        self.generation = None
        self.renderers = {}

    def restore(self, snapshot):
        if snapshot["generation"] != self.generation:
            self.close()
            # Only the owning physics process supplies this private pipe.
            self.model = pickle.loads(snapshot["model"])
            self.data = self.mj.MjData(self.model)
            self.generation = snapshot["generation"]
        self.mj.mj_setState(
            self.model,
            self.data,
            snapshot["state"],
            self.mj.mjtState.mjSTATE_INTEGRATION,
        )
        self.mj.mj_forward(self.model, self.data)

    def capture(self, name):
        row = self.config["cameras"][name]
        if name not in self.renderers:
            self.renderers[name] = RGBDRenderer(
                self.mj,
                self.model,
                height=row["stream"]["height"],
                width=row["stream"]["width"],
                depth=row["options"]["depth"],
            )
        option = self.mj.MjvOption()
        option.geomgroup[3] = 0
        rgb, depth, _ = self.renderers[name].capture(
            self.data,
            camera=self.config.get("native_scene", {})
            .get("camera_names", {})
            .get(name, name),
            scene_option=option,
        )
        if depth is not None:
            far = self.model.vis.map.zfar * self.model.stat.extent
            depth[depth >= far * 0.999] = 0
            depth = depth_z16(depth, row["intrinsics"]["depth_scale_mm"])
        return rgb, depth

    def close(self):
        for renderer in self.renderers.values():
            renderer.close()
        self.renderers.clear()


def serve(connection):
    renderer = None
    try:
        if hasattr(os, "sched_getaffinity"):
            # Software GL can otherwise occupy every inherited CPU and starve
            # the independent control owner. Stay inside its allowed CPU set;
            # this is a bound on optional rendering, not a resource reservation.
            allowed = sorted(os.sched_getaffinity(0))
            os.sched_setaffinity(0, {allowed[-1]})
        os.environ["LP_NUM_THREADS"] = "1"
        config, initial = connection.recv()
        renderer = SnapshotRenderer(config)
        renderer.restore(initial)
        # Readiness still includes real RGB/depth capture for every camera.
        for name in config["cameras"]:
            renderer.capture(name)
        connection.send((True, None))
        while True:
            request = connection.recv()
            if request is None:
                break
            name, snapshot = request
            renderer.restore(snapshot)
            connection.send((True, renderer.capture(name)))
    except (EOFError, BrokenPipeError):
        pass
    except BaseException as error:  # noqa: BLE001 -- preserve native worker faults across its private pipe
        traceback.print_exc(file=sys.stderr)
        try:
            connection.send((False, f"{type(error).__name__}: {error}"))
        except (OSError, EOFError):
            pass
    finally:
        if renderer is not None:
            renderer.close()
        connection.close()


if __name__ == "__main__":
    serve(Connection(int(sys.argv[1])))
