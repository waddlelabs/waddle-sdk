"""Private RealSense capture process, independent of supervised robot threads."""

from __future__ import annotations

import subprocess
import sys
import threading
from multiprocessing import Pipe
from multiprocessing.connection import Connection

import numpy as np

from .base import CameraFrame


class IsolatedRealSenseDriver:
    """Keep vendor device opening/recovery out of the caller's interpreter."""

    _worker_module = "waddle_sdk.cameras.realsense_process"

    def __init__(self, *, width=640, height=480, fps=30, serial=None):
        self._lock = threading.Lock()
        self._closed = False
        self._connection, child = Pipe()
        self._process = None
        try:
            self._process = subprocess.Popen(
                [sys.executable, "-m", self._worker_module, str(child.fileno())],
                pass_fds=(child.fileno(),),
                stdin=subprocess.DEVNULL,
            )
            child.close()
            self._connection.send(
                {"width": width, "height": height, "fps": fps, "serial": serial}
            )
            self._metadata(self._receive())
        except BaseException:
            child.close()
            self.close()
            raise

    def _receive(self):
        if not self._connection.poll(30):
            raise RuntimeError(
                "RealSense capture process did not reply within 30 seconds"
            )
        try:
            ok, value = self._connection.recv()
        except (EOFError, ConnectionResetError) as error:
            raise RuntimeError(
                "RealSense capture process exited without a reply"
            ) from error
        if not ok:
            error, cause = value
            if cause is not None:
                error.__cause__ = cause
            raise error
        return value

    def _metadata(self, value):
        self.depth_scale_mm, self._intrinsics, self._raw_intrinsics = value

    def intrinsics(self):
        if self._intrinsics is None:
            raise RuntimeError("the active RealSense profile exposes no intrinsics")
        return self._intrinsics

    def capture(self):
        with self._lock:
            if self._closed:
                raise RuntimeError("RealSense camera is closed")
            self._connection.send("capture")
            rgb, depth, metadata = self._receive()
            self._metadata(metadata)
            resolver = None
            if self._raw_intrinsics is not None:
                # Only pure per-pixel deprojection runs here. No device/context
                # or pipeline is created in the robot owner's interpreter.
                from .realsense import _vendor_module

                rs = _vendor_module()
                intrinsics = rs.intrinsics()
                for key, value in self._raw_intrinsics.items():
                    setattr(
                        intrinsics,
                        key,
                        getattr(rs.distortion, value) if key == "model" else value,
                    )

                def resolver(x, y, depth_m):
                    return tuple(
                        rs.rs2_deproject_pixel_to_point(
                            intrinsics, [float(x), float(y)], float(depth_m)
                        )
                    )

            return CameraFrame(rgb=rgb, depth=depth, point_resolver=resolver)

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            process = self._process
            if process is not None:
                try:
                    self._connection.send(None)
                except (OSError, EOFError):
                    pass
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    # This process owns cameras only; a wedged USB cleanup must
                    # never prevent the supervising program from draining.
                    process.terminate()
                    try:
                        process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
            self._connection.close()


def _metadata(driver):
    raw = driver._active_intrinsics
    values = None
    if raw is not None:
        values = {
            key: str(raw.model).rsplit(".", 1)[-1]
            if key == "model"
            else getattr(raw, key)
            for key in ("width", "height", "ppx", "ppy", "fx", "fy", "coeffs", "model")
        }
    return driver.depth_scale_mm, driver._intrinsics, values


def serve(connection):
    from .realsense import RealSenseDriver

    driver = None
    try:
        driver = RealSenseDriver(**connection.recv())
        connection.send((True, _metadata(driver)))
        while True:
            request = connection.recv()
            if request is None:
                break
            if request != "capture":
                raise ValueError("Unknown RealSense capture request")
            frame = driver.capture()
            connection.send(
                (
                    True,
                    (np.asarray(frame.rgb), np.asarray(frame.depth), _metadata(driver)),
                )
            )
    except (EOFError, BrokenPipeError):
        pass
    except Exception as error:  # noqa: BLE001 -- retain the vendor's original failure across the private pipe
        try:
            connection.send((False, (error, error.__cause__)))
        except (OSError, EOFError):
            pass
    finally:
        if driver is not None:
            driver.close()
        connection.close()


if __name__ == "__main__":
    serve(Connection(int(sys.argv[1])))
