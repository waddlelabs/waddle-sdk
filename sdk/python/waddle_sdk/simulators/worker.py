"""Private process-local physics loop. No socket listener or control authority."""

from __future__ import annotations

import importlib
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

            def advance(duration):
                nonlocal pending_time
                pending_time += duration
                steps = math.floor(pending_time / dt + 1e-10)
                for _ in range(steps):
                    engine.step()
                pending_time = max(0.0, pending_time - steps * dt)

            while True:
                operation, arguments = connection.recv()
                if real_time:
                    # Advance the OLD targets up to this request before reading
                    # state or installing a new target. Render/IPC delays must
                    # neither discard elapsed physics nor replay new commands
                    # into the past. The engine always retains its fixed dt.
                    now = time.monotonic()
                    advance(now - last_time)
                    last_time = now
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
