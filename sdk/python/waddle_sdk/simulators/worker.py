"""Private process-local physics loop. No socket listener or control authority."""

from __future__ import annotations

import importlib
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
            deadline = time.monotonic()
            dt = config["timestep"]
            while True:
                now = time.monotonic()
                # Bounded catch-up keeps observation/hold latency bounded even
                # when rendering runs slower than the requested physics clock.
                steps = min(50, max(0, int((now - deadline) / dt) + 1))
                for _ in range(steps):
                    engine.step()
                deadline = max(deadline + steps * dt, now - 0.1)
                if not connection.poll(
                    max(0.0, min(0.01, deadline - time.monotonic()))
                ):
                    continue
                operation, arguments = connection.recv()
                if operation == "close":
                    engine.hold()
                    connection.send((True, None))
                    break
                if operation not in {"read", "write", "hold", "home", "capture"}:
                    raise ValueError("unsupported simulation operation")
                try:
                    connection.send((True, getattr(engine, operation)(*arguments)))
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
