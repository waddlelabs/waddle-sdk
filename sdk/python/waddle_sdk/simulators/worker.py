"""Private process-local physics loop. No socket listener or control authority."""

from __future__ import annotations

import importlib
import logging
import math
import queue
import sys
import tempfile
import threading
import time
import traceback
from contextlib import nullcontext
from multiprocessing.connection import Connection
from pathlib import Path


class _CameraLoop:
    """Own native render contexts and a separate bounded camera transaction."""

    def __init__(self, engine, connection, lock, synchronize, names):
        self.engine, self.connection = engine, connection
        self.lock, self.synchronize, self.names = lock, synchronize, names
        self.transaction = threading.Lock()
        self.stop = threading.Event()
        self.ready = queue.Queue(maxsize=1)
        self.error = None
        self.cleanup_error = None
        self.thread = threading.Thread(
            target=self._run, name="waddle-simulation-camera"
        )

    def _capture(self, name, *, advance=True):
        with self.lock:
            if advance:
                self.synchronize()
            prepared = self.engine.prepare_capture(name)
        return self.engine.render_capture(prepared)

    def start(self):
        self.thread.start()
        error = self.ready.get()
        if error is not None:
            raise error

    def _run(self):
        started = False
        try:
            for name in self.names:
                self._capture(name, advance=False)
            self.ready.put(None)
            started = True
            while not self.stop.is_set():
                if not self.connection.poll(0.1):
                    continue
                operation, arguments = self.connection.recv()
                try:
                    if operation != "capture" or len(arguments) not in (1, 2):
                        raise ValueError("unsupported simulation camera operation")
                    with self.transaction:
                        result = self._capture(arguments[0])
                        timed = len(arguments) == 2 and arguments[1] is True
                        self.connection.send((True, result if timed else result[:2]))
                except Exception as error:  # noqa: BLE001 -- forward the original simulation fault
                    with self.lock:
                        try:
                            self.engine.hold()
                        except Exception:  # noqa: BLE001 -- cleanup failure cannot replace the original fault
                            traceback.print_exc(file=sys.stderr)
                    traceback.print_exc(file=sys.stderr)
                    self.connection.send((False, f"{type(error).__name__}: {error}"))
        except BaseException as error:  # noqa: BLE001 -- preserve thread/process failure for the owner
            self.error = error
            if not started:
                self.ready.put(error)
            elif not self.stop.is_set():
                with self.lock:
                    try:
                        self.engine.hold()
                    except Exception:  # noqa: BLE001 -- cleanup failure cannot replace the original fault
                        traceback.print_exc(file=sys.stderr)
        finally:
            try:
                with self.lock:
                    self.engine.close_renderers()
            except BaseException as error:  # noqa: BLE001 -- preserve thread/process failure for the owner
                self.cleanup_error = error
                traceback.print_exc(file=sys.stderr)
            self.connection.close()

    def shutdown(self):
        self.stop.set()
        self.thread.join(3.0)
        if self.thread.is_alive():
            raise RuntimeError("simulation camera worker did not drain")
        if self.error is not None:
            raise self.error
        if self.cleanup_error is not None:
            raise self.cleanup_error


def serve(connection: Connection) -> None:
    engine = None
    cameras = None
    with tempfile.TemporaryDirectory(prefix="waddle-simulation-") as scratch:
        try:
            config = connection.recv()
            engine_type = importlib.import_module(
                f"waddle_sdk.simulators.{config['backend']}"
            ).Engine
            engine = engine_type(config, Path(scratch))
            lock = threading.RLock()
            dt = config["timestep"]
            pending_time = 0.0
            real_time = config.get("_real_time", False)
            last_time = time.monotonic()

            def advance(duration, deadline=None):
                nonlocal pending_time
                pending_time += duration
                steps = math.floor(pending_time / dt + 1e-10)
                for index in range(steps):
                    engine.step()
                    if (
                        deadline is not None
                        and index + 1 < steps
                        and time.monotonic() >= deadline
                    ):
                        pending_time = 0.0
                        return False
                pending_time = max(0.0, pending_time - steps * dt)
                return True

            warned_slow = False

            def synchronize():
                nonlocal last_time, warned_slow
                if real_time:
                    # Advance OLD targets first. Preserve the original native
                    # substeps and 20-ms catch-up ceiling, including on capture.
                    now = time.monotonic()
                    caught_up = advance(now - last_time, deadline=now + 0.02)
                    last_time = now if caught_up else time.monotonic()
                    if not caught_up and not warned_slow:
                        logging.getLogger(__name__).warning(
                            "Physics cannot keep up with real time; dropping "
                            "wall-clock lag while retaining fixed native steps. "
                            "Motion and camera throughput may be slower."
                        )
                        warned_slow = True

            camera_fd = config.get("_camera_connection_fd")
            if camera_fd is not None:
                camera_connection = Connection(camera_fd)
                parallel = getattr(engine, "parallel_camera_capture", False)
                if type(parallel) is not bool:
                    raise ValueError("parallel_camera_capture must be a boolean")
                if parallel:
                    for name in (
                        "prepare_capture",
                        "render_capture",
                        "close_renderers",
                    ):
                        if not callable(getattr(engine, name, None)):
                            raise TypeError(f"parallel camera engine lacks {name}")
                    cameras = _CameraLoop(
                        engine, camera_connection, lock, synchronize, config["cameras"]
                    )
                    cameras.start()
                else:
                    camera_connection.close()
            if cameras is None:
                for name in config["cameras"]:
                    engine.capture(name)
            timing_kind = (
                getattr(engine, "content_timing_kind", None)
                if callable(getattr(engine, "capture_timed", None))
                else None
            )
            last_time = time.monotonic()
            connection.send(
                (
                    True,
                    {
                        "camera_content_timing_kind": timing_kind,
                        "parallel_camera_capture": cameras is not None,
                    },
                )
            )
            while True:
                operation, arguments = connection.recv()
                if operation == "close":
                    with lock:
                        synchronize()
                        engine.hold()
                    if cameras is not None:
                        cameras.shutdown()
                    connection.send(
                        (True, {"camera_drain_confirmed": True} if cameras else None)
                    )
                    break
                # Resets may mutate model/calibration fields. Drain a complete
                # capture transaction before changing them; ordinary control and
                # Hold need only the short physics-state lock.
                barrier = (
                    cameras.transaction
                    if cameras and operation in {"reset", "evaluation_reset"}
                    else nullcontext()
                )
                with barrier, lock:
                    synchronize()
                    if operation not in {
                        "read",
                        "write",
                        "hold",
                        "home",
                        "capture",
                        "step",
                        "reset",
                        "gripper_force",
                        "gripper_force_part",
                        "gripper_force_limits",
                        "gripper_force_limits_part",
                        "write_force",
                        "write_force_part",
                        "read_part",
                        "write_part",
                        "hold_part",
                        "home_part",
                        "evaluation_snapshot",
                        "evaluation_reset",
                    }:
                        raise ValueError("unsupported simulation operation")
                    try:
                        if operation == "step":
                            if not real_time:
                                advance(arguments[0])
                            result = None
                        elif operation == "capture" and cameras is not None:
                            raise ValueError(
                                "capture requires the dedicated camera channel"
                            )
                        elif (
                            operation == "capture"
                            and len(arguments) == 2
                            and arguments[1] is True
                        ):
                            result = engine.capture_timed(arguments[0])
                        elif operation == "evaluation_snapshot":
                            result = engine.evaluation_snapshot()
                        elif operation == "evaluation_reset":
                            variation = arguments[1] if len(arguments) > 1 else None
                            result = engine.evaluation_reset(
                                seed=arguments[0],
                                variation=variation,
                                preserve_free_bodies=arguments[2]
                                if len(arguments) > 2
                                else (),
                                retire_free_bodies=arguments[3]
                                if len(arguments) > 3
                                else (),
                            )
                            pending_time = 0.0
                            last_time = time.monotonic()
                        else:
                            method = getattr(engine, operation.removesuffix("_part"))
                            result = method(*arguments)
                            if operation == "reset":
                                pending_time = 0.0
                                last_time = time.monotonic()
                        connection.send((True, result))
                    except Exception as error:  # noqa: BLE001 -- forward the original simulation fault
                        engine.hold()
                        traceback.print_exc(file=sys.stderr)
                        connection.send((False, f"{type(error).__name__}: {error}"))
        except (EOFError, BrokenPipeError):
            pass
        except BaseException as error:  # noqa: BLE001 -- preserve thread/process failure for the owner
            traceback.print_exc(file=sys.stderr)
            try:
                connection.send((False, f"{type(error).__name__}: {error}"))
            except (OSError, EOFError):
                pass
        finally:
            renderers_drained = cameras is None
            if cameras is not None:
                try:
                    cameras.shutdown()
                    renderers_drained = True
                except BaseException:  # noqa: BLE001 -- retain uncertain render teardown
                    traceback.print_exc(file=sys.stderr)
            # A stuck/failed render owner retains its model/context references
            # until process teardown. Never destroy its contexts on this thread.
            if engine is not None and renderers_drained:
                engine.close()
            connection.close()


if __name__ == "__main__":
    serve(Connection(int(sys.argv[1])))
