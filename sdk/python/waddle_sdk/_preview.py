"""Bounded raw-frame handoff to an optional, hardware-free preview child.

Each camera has one anonymous shared-memory slot. Writers never wait for a
reader, encoder, network, process startup, or acknowledgement. Nothing here
opens a camera, calls a robot driver, or changes session supervision.
"""

from __future__ import annotations

import json
import mmap
import os
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

_HEADER = struct.Struct("!QQ")
_MAX_BYTES = 32 * 1024 * 1024
_MAX_STATUS = 16 * 1024


class _Slot:
    def __init__(self, name, width, height):
        self.name, self.width, self.height = name, width, height
        self.fd = os.memfd_create("waddle-rgb-preview", os.MFD_CLOEXEC)
        self.size = _HEADER.size + width * height * 3
        try:
            os.ftruncate(self.fd, self.size)
            self.memory = mmap.mmap(self.fd, self.size)
        except BaseException:
            os.close(self.fd)
            raise
        self.lock = threading.Lock()
        self.sequence = 0
        self.next_frame = 0.0
        self.dropped = 0

    def close(self):
        self.memory.close()
        os.close(self.fd)


class IsolatedPreview:
    """Private resource composition; no second SDK session or hardware owner."""

    def __init__(self, media, cameras, *, testing_loopback=False):
        self._slots = {}
        self._process = None
        self._socket = None
        self._failure = None
        self._closed = False
        self._rows = []
        self._demand = {}
        self._period = 1.0 / media.preview_fps
        child = None
        try:
            if not hasattr(os, "memfd_create"):
                raise RuntimeError("isolated RGB preview requires Linux memfd support")
            import fcntl

            self._fcntl = fcntl
            total = sum(_HEADER.size + c.width * c.height * 3 for c in cameras.values())
            if total > _MAX_BYTES:
                raise ValueError("isolated preview source slots exceed 32 MiB")
            for name, camera in cameras.items():
                self._slots[name] = _Slot(name, camera.width, camera.height)
                self._rows.append(self._row(name, "pending"))
            parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
            parent.setblocking(False)
            self._socket = parent
            config = {
                "url": media.url,
                "token": media.token,
                "preview_width": media.preview_width,
                "preview_fps": media.preview_fps,
                "preview_max_kbps": media.preview_max_kbps,
                "demand_driven": media.demand_driven,
                "worker_cpu_ids": media.worker_cpu_ids,
                "worker_memory_mb": media.worker_memory_mb,
                "parent_pid": os.getpid(),
                "package_root": str(Path(__file__).resolve().parent.parent),
                "testing_loopback": testing_loopback,
                "cameras": [
                    {"name": name, "width": s.width, "height": s.height, "fd": s.fd}
                    for name, s in self._slots.items()
                ],
            }
            payload = json.dumps(config).encode()
            if len(payload) > _MAX_STATUS:
                raise ValueError("isolated preview configuration exceeds 16 KiB")
            # Credentials travel over the inherited private socket, never argv,
            # environment, a persistent file, or diagnostic stdout/stderr.
            self._process = subprocess.Popen(
                [
                    sys.executable,
                    "-I",
                    str(Path(__file__).with_name("_preview_worker.py")),
                    str(child.fileno()),
                ],
                pass_fds=(child.fileno(), *(s.fd for s in self._slots.values())),
                env={
                    key: os.environ[key]
                    for key in ("PATH", "LANG", "LC_ALL", "LD_LIBRARY_PATH")
                    if key in os.environ
                },
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            parent.send(payload)
        except BaseException as error:
            try:
                self._release()
            except Exception as cleanup_error:  # noqa: BLE001 -- preserve the startup failure
                self._close_error = str(cleanup_error)
            if not isinstance(error, Exception):
                raise
            self._failure = f"media_worker_start: {type(error).__name__}: {error}"
        finally:
            if child is not None:
                child.close()

    @staticmethod
    def _row(name, status, dropped=0):
        return {
            "camera_id": name,
            "stream": "rgb",
            "track_name": name,
            "status": status,
            "frames_dropped": dropped,
        }

    def submit(self, camera, frame):
        """At most one memcpy per camera/presentation interval; no waiting."""
        if (
            self._closed
            or self._failure is not None
            or self._demand.get(camera) is False
        ):
            return
        slot = self._slots[camera]
        now = time.monotonic()
        if now < slot.next_frame or not slot.lock.acquire(blocking=False):
            return
        try:
            try:
                self._fcntl.lockf(slot.fd, self._fcntl.LOCK_EX | self._fcntl.LOCK_NB)
            except BlockingIOError:
                slot.dropped += 1
                return
            try:
                slot.memory[_HEADER.size :] = memoryview(frame).cast("B")
                slot.sequence += 1
                _HEADER.pack_into(slot.memory, 0, slot.sequence, time.monotonic_ns())
                slot.next_frame = now + self._period
            finally:
                self._fcntl.lockf(slot.fd, self._fcntl.LOCK_UN)
        except Exception as error:  # noqa: BLE001 -- optional IPC failure cannot poison the authoritative SDK path
            self._failure = f"media_frame_handoff: {type(error).__name__}: {error}"
        finally:
            slot.lock.release()

    def tracks(self):
        if self._socket is not None:
            # Bound status work even when an optional consumer was not polled.
            for _ in range(8):
                try:
                    status = json.loads(self._socket.recv(_MAX_STATUS))
                except BlockingIOError:
                    break
                except (OSError, ValueError) as error:
                    self._failure = f"media_worker_status: {error}"
                    break
                if "error" in status:
                    self._failure = status["error"]
                else:
                    self._rows = status["tracks"]
                    self._demand = {
                        row["camera_id"]: row.get("wants_frame", True)
                        for row in self._rows
                    }
        if (
            self._failure is None
            and self._process is not None
            and self._process.poll() is not None
        ):
            self._failure = f"media_worker_exit: status {self._process.returncode}"
        if self._failure is not None:
            raise RuntimeError(self._failure)
        return [
            dict(
                row,
                frames_dropped=row["frames_dropped"]
                + self._slots[row["camera_id"]].dropped,
            )
            for row in self._rows
        ]

    def _release(self):
        if self._process is not None and self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=1)
        if self._socket is not None:
            self._socket.close()
            self._socket = None
        for slot in self._slots.values():
            slot.close()
        self._slots.clear()

    def dropped(self, camera):
        slot = self._slots.get(camera)
        return (0 if slot is None else slot.dropped) + next(
            (row["frames_dropped"] for row in self._rows if row["camera_id"] == camera),
            0,
        )

    def close(self):
        if not self._closed:
            self._closed = True
            self._release()


class PreviewSession:
    """Delegate all control/recording to the single native owner unchanged."""

    def __init__(self, inner, preview):
        self._inner, self._preview = inner, preview

    def __getattr__(self, name):
        return getattr(self._inner, name)

    @property
    def depth_preview_enabled(self):
        return False

    def publish_frame(self, camera, frame):
        self._inner.publish_frame(camera, frame)
        self._preview.submit(camera, frame)

    def media_tracks(self):
        return self._preview.tracks()

    def camera_frames_dropped(self, camera):
        return self._inner.camera_frames_dropped(camera) + self._preview.dropped(camera)

    def shutdown(self):
        try:
            result = self._inner.shutdown()
        except BaseException as original:
            try:
                self._preview.close()
            except Exception as error:  # noqa: BLE001 -- retain optional drain failure beside the original SDK fault
                self._preview_close_error = str(error)
                note = getattr(original, "add_note", None)
                if note is not None:
                    note(f"Optional preview cleanup also failed: {error}")
            raise
        else:
            # Finish authoritative hardware cleanup before optional child drain.
            try:
                self._preview.close()
            except Exception as error:  # noqa: BLE001 -- optional cleanup cannot turn completed control cleanup into a failure
                self._preview_close_error = str(error)
            return result
