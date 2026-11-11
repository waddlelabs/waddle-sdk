"""Optional bounded exception evidence, independent of device/control authority.

Applications select a private directory before opening devices. Producers copy
only bounded scalar stack metadata; one low-priority worker owns all file I/O.
No frames, locals, source contents, network clients or hardware are retained.
"""

from __future__ import annotations

import builtins
import hashlib
import json
import os
import queue
import stat
import sys
import threading
import time
from contextlib import contextmanager
from contextvars import ContextVar
from pathlib import Path
from uuid import uuid4

MAX_RECORD_BYTES = 48 * 1024
MAX_RECORDS = 64
MAX_PENDING = 8
_scope = ContextVar("sdk_diagnostic_scope", default=None)
_store = None
_pid = None


def _safe(value, secrets=()):
    from .runtime import _diagnostic_text

    text = _diagnostic_text(str(value)[:2048])
    for secret in secrets:
        if secret:
            text = text.replace(secret, "[REDACTED]")
    return text


def exception_evidence(error, *, secrets=()):
    """Copy a bounded exception graph without introspecting user objects."""
    from .runtime import RuntimeFault

    pending, seen, nodes = [(error, None, "root")], set(), []
    while pending and len(nodes) < 16:
        current, parent, relation = pending.pop()
        if id(current) in seen:
            continue
        seen.add(id(current))
        frames, trace = [], current.__traceback__
        while trace is not None and len(frames) < 32:
            code = trace.tb_frame.f_code
            frames.append(
                {
                    "file": _safe(code.co_filename, secrets),
                    "function": _safe(code.co_name, secrets),
                    "line": trace.tb_lineno,
                }
            )
            trace = trace.tb_next
        node = {
            "type": type(current).__module__ + "." + type(current).__qualname__,
            # Do not invoke arbitrary exception __str__ a second time on the
            # producer. The original structured fault retains its full text.
            "detail": _safe(current.args[0], secrets)
            if current.args and type(current.args[0]) is str
            else "See original structured fault",
            "parent": parent,
            "relation": relation,
            "frames": frames,
            "frames_truncated": trace is not None,
        }
        if isinstance(current, RuntimeFault):
            node["code"] = current.code.value
            node["detail"] = _safe(current.detail, secrets)
        if isinstance(current, OSError):
            node.update(
                errno=current.errno,
                filename=None
                if current.filename is None
                else _safe(current.filename, secrets),
            )
        index = len(nodes)
        nodes.append(node)
        cause = current.__cause__
        if cause is None and not current.__suppress_context__:
            cause = current.__context__
        if cause is not None:
            pending.append((cause, index, "cause"))
        # ExceptionGroup exists only on Python 3.11+. No arbitrary attributes
        # or iterable exception subclasses are evaluated.
        if isinstance(current, getattr(builtins, "BaseExceptionGroup", ())):
            pending.extend(
                (child, index, "member") for child in current.exceptions[:16]
            )
    return {"exceptions": nodes, "exceptions_truncated": bool(pending)}


@contextmanager
def diagnostic_scope(**identifiers):
    token = _scope.set({**(_scope.get() or {}), **identifiers})
    try:
        yield
    finally:
        _scope.reset(token)


class DiagnosticStore:
    def __init__(self, directory, *, context=None, secrets=()):
        self.directory = Path(directory).absolute()
        self.context = dict(context or {})
        self.secrets = tuple(secrets)
        self.pending = queue.Queue(MAX_PENDING)
        self.dropped = 0
        self.storage_failures = 0
        self.versions = {}
        self.closed = False
        self.thread = threading.Thread(
            target=self._run, name="sdk-diagnostics", daemon=True
        )
        self.thread.start()

    def capture(self, error, *, context=None):
        if self.closed or self.pending.full():
            self.dropped += 1
            return None
        identity = uuid4().hex
        from .runtime import _CREDENTIAL_FIELD

        fields = {**self.context, **(_scope.get() or {}), **(context or {})}
        record = {
            "schema": "waddle.diagnostic/v1",
            "id": identity,
            "unix_ns": time.time_ns(),
            "monotonic_ns": time.monotonic_ns(),
            "pid": os.getpid(),
            "thread_id": threading.get_native_id(),
            "context": {
                str(k): "[REDACTED]"
                if _CREDENTIAL_FIELD.fullmatch(str(k))
                else _safe(v, self.secrets)
                for k, v in list(fields.items())[:32]
                if type(v) in (str, int, float, bool) or v is None
            },
            "context_truncated": len(fields) > 32,
            **exception_evidence(error, secrets=self.secrets),
        }
        try:
            self.pending.put_nowait(record)
        except queue.Full:
            self.dropped += 1
            return None
        return identity

    def _write(self, record):
        import fcntl

        self.directory.mkdir(mode=0o700, parents=True, exist_ok=True)
        info = self.directory.lstat()
        if (
            not stat.S_ISDIR(info.st_mode)
            or info.st_uid != os.getuid()
            or info.st_mode & 0o077
        ):
            raise PermissionError("Diagnostic directory must be private and user-owned")
        descriptor = os.open(
            self.directory / ".lock", os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600
        )
        try:
            # A disk stall in another producer cannot grow the in-memory queue.
            fcntl.flock(descriptor, fcntl.LOCK_EX)
            files = sorted(self.directory.glob("*.json"), key=lambda p: p.name)
            if len(files) >= MAX_RECORDS:
                # Keep retained evidence. Report saturation rather than deleting
                # an incident that has not reached its consumer.
                self.dropped += 1
                return
            record["capture_status"] = {
                "dropped": self.dropped,
                "storage_failures": self.storage_failures,
            }
            # Hash only the traceback source files, never read them on a control
            # thread. Each read has its own bound; no imported source tree walk.
            hashes, remaining = {}, 2 * 1024 * 1024
            for node in record["exceptions"]:
                for frame in node["frames"]:
                    path = frame["file"]
                    if path in hashes:
                        continue
                    if len(hashes) >= 8 or remaining <= 0 or not path.endswith(".py"):
                        hashes[path] = None
                        continue
                    try:
                        source_fd = os.open(
                            path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK
                        )
                        with os.fdopen(source_fd, "rb") as source:
                            info = os.fstat(source.fileno())
                            limit = min(remaining, 512 * 1024)
                            if not stat.S_ISREG(info.st_mode) or info.st_size > limit:
                                raise ValueError("Source is not regular")
                            raw = source.read(limit + 1)
                        remaining -= len(raw)
                        hashes[path] = (
                            hashlib.sha256(raw).hexdigest()
                            if len(raw) <= limit and len(raw) == info.st_size
                            else None
                        )
                    except (OSError, ValueError):
                        hashes[path] = None
            record["source_sha256"] = hashes
            record["python"] = {"version": sys.version, "executable": sys.executable}
            record["distributions"] = self.versions
            raw = json.dumps(record, allow_nan=False, separators=(",", ":")).encode()
            if len(raw) > MAX_RECORD_BYTES:
                record["exceptions"] = record["exceptions"][:1]
                record["exceptions"][0]["frames"] = record["exceptions"][0]["frames"][
                    -8:
                ]
                record["source_sha256"] = {}
                record["record_truncated"] = True
                raw = json.dumps(
                    record, allow_nan=False, separators=(",", ":")
                ).encode()
            if len(raw) > MAX_RECORD_BYTES:
                self.dropped += 1
                return
            path = self.directory / (f"{record['unix_ns']:020d}-{record['id']}.json")
            temporary = self.directory / (".pending-" + record["id"])
            fd = os.open(
                temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600
            )
            try:
                with os.fdopen(fd, "wb") as target:
                    target.write(raw)
                # Publish only complete records; never overwrite retained evidence.
                os.link(temporary, path, follow_symlinks=False)
            finally:
                temporary.unlink(missing_ok=True)
        finally:
            os.close(descriptor)

    def _run(self):
        try:
            # Linux niceness is per thread; never renice the device owner.
            if sys.platform == "linux":
                os.setpriority(os.PRIO_PROCESS, threading.get_native_id(), 19)
        except OSError:
            pass
        try:
            from importlib.metadata import PackageNotFoundError, version

            names = [
                "waddle-sdk",
                *json.loads(os.environ.get("WADDLE_DIAGNOSTICS_DISTRIBUTIONS", "[]")),
            ]
            for name in names[:16]:
                try:
                    self.versions[name] = version(name)
                except PackageNotFoundError:
                    self.versions[name] = "unavailable"
        except Exception:  # noqa: BLE001 -- optional metadata cannot stop capture
            self.versions = {"waddle-sdk": "unavailable"}
        while True:
            record = self.pending.get()
            try:
                if record is None:
                    return
                self._write(record)
            except Exception:  # noqa: BLE001 -- storage failure cannot replace the device fault
                self.storage_failures += 1
            finally:
                self.pending.task_done()
            if self.closed and self.pending.empty():
                return

    def close(self):
        self.closed = True
        # Never wait for a filesystem or a diagnostic worker during control drain.
        try:
            self.pending.put_nowait(None)
        except queue.Full:
            pass

    def status(self):
        return {
            "dropped": self.dropped,
            "storage_failures": self.storage_failures,
            "pending": self.pending.qsize(),
            "worker_alive": self.thread.is_alive(),
        }


def configure(directory, *, context=None, secrets=(), inherit=False):
    """Select optional capture before devices open; no device state is changed."""
    global _store, _pid
    if _store is not None and _pid == os.getpid() and not _store.closed:
        if Path(directory).absolute() != _store.directory.absolute():
            raise ValueError("A diagnostic sink is already configured for this process")
        _store.context = dict(context or {})
        _store.secrets = tuple(secrets)
    else:
        _store = DiagnosticStore(directory, context=context, secrets=secrets)
        _pid = os.getpid()
    if inherit:
        os.environ["WADDLE_DIAGNOSTICS_DIR"] = str(Path(directory).absolute())
        os.environ["WADDLE_DIAGNOSTICS_CONTEXT"] = json.dumps(context or {})
    return _store


def capture_exception(error, *, context=None):
    """Best-effort evidence; failures never replace the source exception."""
    try:
        store = _store
        if store is None or _pid != os.getpid():
            directory = os.environ.get("WADDLE_DIAGNOSTICS_DIR")
            if not directory:
                return None
            store = configure(
                directory,
                context=json.loads(os.environ.get("WADDLE_DIAGNOSTICS_CONTEXT", "{}")),
            )
        return store.capture(error, context=context)
    except Exception:  # noqa: BLE001 -- diagnostics never replace the source failure
        return None
