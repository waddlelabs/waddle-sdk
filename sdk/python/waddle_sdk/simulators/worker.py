"""Private process-local physics loop. No socket listener or control authority."""

from __future__ import annotations

import importlib
import logging
import math
import sys
import tempfile
import time
import traceback
from multiprocessing.connection import Connection
from pathlib import Path


def serve(connection: Connection) -> None:
    engine = None
    with tempfile.TemporaryDirectory(prefix="waddle-simulation-") as scratch:
        try:
            config = connection.recv()
            engine_type = importlib.import_module(
                f"waddle_sdk.simulators.{config['backend']}"
            ).Engine
            engine = engine_type(config, Path(scratch))
            # Readiness means both sensor streams render, not merely that an
            # engine module imports. Startup failure never degrades to a mock.
            for name in config["cameras"]:
                engine.capture(name)
            connection.send((True, None))
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
                        # Drop wall-clock lag, never enlarge a native step or
                        # replay the next command over an unfinished backlog.
                        pending_time = 0.0
                        return False
                pending_time = max(0.0, pending_time - steps * dt)
                return True

            warned_slow = False
            while True:
                operation, arguments = connection.recv()
                if real_time:
                    # Advance the OLD targets up to this request before reading
                    # state or installing a new target. Render/IPC delays must
                    # not replay new commands into the past. Bound catch-up work
                    # to 20 ms (plus one native step), so sustained overload
                    # slows simulation instead of starving controls/sensors.
                    # Explicit rollouts retain their full requested duration.
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
                if operation == "close":
                    engine.hold()
                    connection.send((True, None))
                    break
                if operation not in {
                    "read",
                    "write",
                    "hold",
                    "home",
                    "capture",
                    "step",
                    "reset",
                }:
                    raise ValueError("unsupported simulation operation")
                try:
                    if operation == "step":
                        # Explicit stepping remains available for rollouts;
                        # interactive sites use elapsed time on the same worker.
                        if not real_time:
                            advance(arguments[0])
                        result = None
                    else:
                        result = getattr(engine, operation)(*arguments)
                        if operation == "reset":
                            pending_time = 0.0
                            last_time = time.monotonic()
                    connection.send((True, result))
                except Exception as error:
                    engine.hold()
                    traceback.print_exc(file=sys.stderr)
                    connection.send((False, f"{type(error).__name__}: {error}"))
        except (EOFError, BrokenPipeError):
            pass
        except BaseException as error:
            traceback.print_exc(file=sys.stderr)
            try:
                connection.send((False, f"{type(error).__name__}: {error}"))
            except (OSError, EOFError):
                pass
        finally:
            if engine is not None:
                engine.close()
            connection.close()


if __name__ == "__main__":
    serve(Connection(int(sys.argv[1])))
