"""Bounded camera scheduling on a separate, read-only process connection."""

from __future__ import annotations

import queue
import subprocess
import threading
from concurrent.futures import Future, TimeoutError


class RenderClient:
    def __init__(self, process, connection, *, cameras, generation):
        self.process = process
        self.connection = connection
        self.generation = generation
        self._queue = queue.Queue(maxsize=max(1, len(cameras)))
        self._stopping = threading.Event()
        self._failure = None
        self._thread = threading.Thread(
            target=self._run, name="waddle-render", daemon=True
        )
        self._thread.start()

    def capture(self, camera, acquire, cancelled):
        future = Future()
        if self._stopping.is_set():
            raise self._failure or RuntimeError("simulation renderer is closed")
        try:
            self._queue.put_nowait((camera, acquire, future))
        except queue.Full:
            raise RuntimeError("simulation camera render queue is full") from None
        while True:
            if cancelled() or self._stopping.is_set():
                future.cancel()
                raise self._failure or RuntimeError("simulation camera is closed")
            try:
                return future.result(timeout=0.05)
            except TimeoutError:
                if future.done():
                    raise
                continue

    def _run(self):
        try:
            while not self._stopping.is_set():
                try:
                    camera, acquire, future = self._queue.get(timeout=0.05)
                except queue.Empty:
                    continue
                if not future.set_running_or_notify_cancel():
                    continue
                try:
                    # Acquire only when rendering is available. Queued cameras
                    # never accumulate stale snapshots behind a slow render.
                    snapshot, stamp = acquire(self.generation)
                    self.connection.send((camera, snapshot))
                    if not self.connection.poll(15):
                        raise TimeoutError(
                            "simulation renderer did not respond before its deadline"
                        )
                    ok, result = self.connection.recv()
                    if not ok:
                        raise RuntimeError(f"simulation renderer failed: {result}")
                    self.generation = snapshot["generation"]
                    future.set_result((*result, snapshot, stamp))
                except BaseException as error:
                    future.set_exception(error)
                    raise
        except BaseException as error:  # noqa: BLE001 -- retain the renderer's original failure
            self._failure = error
            self._stopping.set()
        finally:
            while True:
                try:
                    _, _, future = self._queue.get_nowait()
                except queue.Empty:
                    break
                if future.set_running_or_notify_cancel():
                    future.set_exception(
                        self._failure or RuntimeError("simulation renderer is closed")
                    )

    def close(self):
        self._stopping.set()
        # Rendering owns no commands. A wedged GL call can be terminated
        # independently, without delaying the physics owner's Hold/close.
        if threading.current_thread() is not self._thread:
            self._thread.join(timeout=0.2)
        if self._thread.is_alive():
            self.process.terminate()
        else:
            try:
                self.connection.send(None)
            except (OSError, EOFError):
                pass
        try:
            self.process.wait(timeout=1)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.connection.close()
        if threading.current_thread() is not self._thread:
            self._thread.join(timeout=2)
