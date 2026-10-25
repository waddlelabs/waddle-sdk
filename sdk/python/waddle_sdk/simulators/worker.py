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
            independent_render = callable(getattr(engine, "render_snapshot", None))
            if not independent_render:
                for name in config["cameras"]:
                    engine.capture(name)
            timing_kind = (
                getattr(engine, "content_timing_kind", None)
                if callable(getattr(engine, "capture_timed", None))
                else None
            )
            connection.send(
                (
                    True,
                    {
                        "camera_content_timing_kind": timing_kind,
                        "independent_render": independent_render,
                    },
                )
            )
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
            generation = 0
            while True:
                # Physics keeps its own fixed-step cadence even while no
                # control request arrives. Rendering runs on another process.
                autonomous = real_time and independent_render
                delay = (
                    max(0.0, dt - pending_time - (time.monotonic() - last_time))
                    if autonomous
                    else dt
                )
                available = not autonomous or connection.poll(delay)
                if available:
                    operation, arguments = connection.recv()
                if real_time:
                    # Advance the OLD targets up to this request before reading
                    # state or installing a new target. Render/IPC delays must
                    # not replay new commands into the past. Bound catch-up work
                    # to one timestep for autonomous physics (20 ms for legacy
                    # request-driven engines), plus one native step. Overload
                    # slows simulation instead of starving controls/sensors.
                    # Explicit rollouts retain their full requested duration.
                    now = time.monotonic()
                    caught_up = advance(
                        now - last_time,
                        deadline=now + (min(dt, 0.02) if autonomous else 0.02),
                    )
                    last_time = now if caught_up else time.monotonic()
                    if not caught_up and not warned_slow:
                        logging.getLogger(__name__).warning(
                            "Physics cannot keep up with real time; dropping "
                            "wall-clock lag while retaining fixed native steps. "
                            "Motion and camera throughput may be slower."
                        )
                        warned_slow = True
                if not available:
                    continue
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
                    "render_snapshot",
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
                    if operation in {"reset", "evaluation_reset"}:
                        generation += 1
                    if operation == "step":
                        # Explicit stepping remains available for rollouts;
                        # interactive sites use elapsed time on the same worker.
                        if not real_time:
                            advance(arguments[0])
                        result = None
                    elif operation == "render_snapshot":
                        result = engine.render_snapshot(generation, arguments[0])
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
                            preserve_free_bodies=(
                                arguments[2] if len(arguments) > 2 else ()
                            ),
                            retire_free_bodies=(
                                arguments[3] if len(arguments) > 3 else ()
                            ),
                        )
                        pending_time = 0.0
                        last_time = time.monotonic()
                    else:
                        if operation.endswith("_part"):
                            method = getattr(engine, operation.removesuffix("_part"))
                            result = method(*arguments)
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
