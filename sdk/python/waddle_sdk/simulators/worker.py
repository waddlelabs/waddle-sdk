"""Private process-local physics loop. No socket listener or control authority."""

from __future__ import annotations

import importlib
import math
import sys
import tempfile
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
            while True:
                operation, arguments = connection.recv()
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
                        # The SDK's existing robot pump owns simulation time,
                        # matching the shared-world MuJoCo reference backend.
                        # Carry fractional substeps across ticks. Rounding each
                        # tick up would accelerate non-integral rate ratios.
                        pending_time += arguments[0]
                        steps = math.floor(pending_time / dt + 1e-10)
                        for _ in range(steps):
                            engine.step()
                        pending_time = max(0.0, pending_time - steps * dt)
                        result = None
                    else:
                        result = getattr(engine, operation)(*arguments)
                        if operation == "reset":
                            pending_time = 0.0
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
