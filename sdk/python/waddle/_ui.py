"""Authenticated dependency-free loopback UI server.

All authority and motion semantics stay in the native Session. This module
does HTTP shape/security, local presentation configuration, and safe file
resolution only.
"""

from __future__ import annotations

import collections
import hmac
import importlib.resources
import json
import math
import mimetypes
import re
import secrets
import shutil
import threading
import urllib.parse
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from . import _core

_MAX_BODY = 16 * 1024
_MAX_MANIFEST_WINDOW = 8 * 1024 * 1024
_MAX_MANIFEST_LINE = 64 * 1024
_MAX_SIDECAR = 2 * 1024 * 1024
_MAX_RECORDINGS = 1000
_SAFE_EPISODE = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,199}\Z")
_SAFE_CHAT_ID = re.compile(r"[A-Za-z0-9_-]{1,128}\Z")
_STATIC = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/app.css": ("app.css", "text/css; charset=utf-8"),
    "/app.js": ("app.js", "text/javascript; charset=utf-8"),
}


def _positive(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a positive finite number")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be a positive finite number")
    return result


class _Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = False

    def __init__(
        self,
        session: _core.Session,
        recording_dir: Path | None,
        token: str,
        config: dict[str, float],
        config_lock: threading.Lock,
    ) -> None:
        super().__init__(("127.0.0.1", 0), _Handler)
        self.session = session
        self.recording_dir = (
            None if recording_dir is None else recording_dir.resolve(strict=False)
        )
        self.token = token
        self.config = config
        self.config_lock = config_lock
        self.control_lock = threading.Lock()
        self.closing = threading.Event()
        host, port = self.server_address
        self.expected_host = f"{host}:{port}"
        self.origin = f"http://{self.expected_host}"


class _Handler(BaseHTTPRequestHandler):
    server: _Server
    protocol_version = "HTTP/1.1"
    server_version = "WaddleLoopback"
    sys_version = ""

    def log_message(self, _format: str, *args: object) -> None:
        # Never put request material (especially future query parameters) in
        # logs. The bearer token lives in a fragment and is not sent here.
        return

    def _headers(
        self,
        status: HTTPStatus,
        content_type: str,
        length: int,
        extra: dict[str, str] | None = None,
    ) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(length))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Pragma", "no-cache")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("Cross-Origin-Opener-Policy", "same-origin")
        self.send_header("Cross-Origin-Resource-Policy", "same-origin")
        self.send_header("Permissions-Policy", "camera=(), microphone=(), geolocation=()")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; script-src 'self'; style-src 'self'; "
            "connect-src 'self'; img-src 'none'; object-src 'none'; "
            "base-uri 'none'; frame-ancestors 'none'; form-action 'none'",
        )
        if extra:
            for name, value in extra.items():
                self.send_header(name, value)
        self.end_headers()

    def _bytes(
        self,
        status: HTTPStatus,
        body: bytes,
        content_type: str,
        extra: dict[str, str] | None = None,
    ) -> None:
        self._headers(status, content_type, len(body), extra)
        self.wfile.write(body)

    def _json(
        self,
        status: HTTPStatus,
        value: object,
        extra: dict[str, str] | None = None,
    ) -> None:
        body = json.dumps(value, separators=(",", ":"), ensure_ascii=False).encode()
        self._bytes(status, body, "application/json; charset=utf-8", extra)

    def _reject(self, status: HTTPStatus, detail: str) -> None:
        self._json(status, {"error": detail})

    def _same_origin(self, *, mutation: bool) -> bool:
        if self.headers.get("Host", "") != self.server.expected_host:
            self._reject(HTTPStatus.FORBIDDEN, "invalid loopback Host")
            return False
        origin = self.headers.get("Origin")
        if mutation and origin != self.server.origin:
            self._reject(HTTPStatus.FORBIDDEN, "invalid loopback Origin")
            return False
        if origin is not None and origin != self.server.origin:
            self._reject(HTTPStatus.FORBIDDEN, "invalid loopback Origin")
            return False
        fetch_site = self.headers.get("Sec-Fetch-Site")
        if fetch_site not in (None, "none", "same-origin"):
            self._reject(HTTPStatus.FORBIDDEN, "cross-site request refused")
            return False
        return True

    def _api_auth(self, *, mutation: bool) -> bool:
        if not self._same_origin(mutation=mutation):
            return False
        if self.server.closing.is_set():
            self._reject(HTTPStatus.SERVICE_UNAVAILABLE, "the UI is closing")
            return False
        token = self.headers.get("X-Waddle-Token", "")
        if not hmac.compare_digest(token, self.server.token):
            self._reject(HTTPStatus.UNAUTHORIZED, "missing or invalid UI token")
            return False
        if self.headers.get("X-Waddle-Request", "") != "1":
            self._reject(HTTPStatus.BAD_REQUEST, "missing X-Waddle-Request header")
            return False
        return True

    def _read_json(self) -> dict[str, Any] | None:
        if self.headers.get("Transfer-Encoding") is not None:
            self._reject(HTTPStatus.BAD_REQUEST, "chunked request bodies are not accepted")
            return None
        try:
            length = int(self.headers.get("Content-Length", ""))
        except ValueError:
            self._reject(HTTPStatus.LENGTH_REQUIRED, "a valid Content-Length is required")
            return None
        if length < 0 or length > _MAX_BODY:
            self._reject(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, "request body is too large")
            return None
        if self.headers.get_content_type() != "application/json":
            self._reject(HTTPStatus.UNSUPPORTED_MEDIA_TYPE, "application/json is required")
            return None
        raw = self.rfile.read(length)
        if len(raw) != length:
            self._reject(HTTPStatus.BAD_REQUEST, "truncated request body")
            return None
        try:
            value = json.loads(raw)
        except (UnicodeDecodeError, json.JSONDecodeError):
            self._reject(HTTPStatus.BAD_REQUEST, "invalid JSON")
            return None
        if not isinstance(value, dict):
            self._reject(HTTPStatus.BAD_REQUEST, "JSON body must be an object")
            return None
        return value

    def do_OPTIONS(self) -> None:  # noqa: N802 — BaseHTTPRequestHandler API
        self._reject(HTTPStatus.METHOD_NOT_ALLOWED, "CORS is not enabled")

    def do_GET(self) -> None:  # noqa: N802 — BaseHTTPRequestHandler API
        parsed = urllib.parse.urlsplit(self.path)
        if parsed.path in _STATIC:
            if not self._same_origin(mutation=False):
                return
            name, content_type = _STATIC[parsed.path]
            body = (
                importlib.resources.files("waddle")
                .joinpath("ui_assets", name)
                .read_bytes()
            )
            self._bytes(HTTPStatus.OK, body, content_type)
            return
        if not parsed.path.startswith("/api/"):
            self._reject(HTTPStatus.NOT_FOUND, "not found")
            return
        if not self._api_auth(mutation=False):
            return
        if parsed.path == "/api/state":
            state = dict(self.server.session.status())
            with self.server.config_lock:
                state["increments"] = dict(self.server.config)
            state["local_controls_available"] = True
            self._json(HTTPStatus.OK, state)
        elif parsed.path.startswith("/api/cameras/"):
            camera = urllib.parse.unquote(parsed.path.removeprefix("/api/cameras/"))
            if not camera or "/" in camera or "\\" in camera:
                self._reject(HTTPStatus.BAD_REQUEST, "invalid camera name")
                return
            frame = self.server.session._ui_frame(camera)
            if frame is None:
                self._reject(HTTPStatus.NOT_FOUND, "no frame is available")
                return
            width, height, data = frame
            self._bytes(
                HTTPStatus.OK,
                data,
                "application/octet-stream",
                {
                    "X-Waddle-Width": str(width),
                    "X-Waddle-Height": str(height),
                    "X-Waddle-Pixel-Format": "RGB8",
                },
            )
        elif parsed.path == "/api/recordings":
            self._json(HTTPStatus.OK, {"recordings": self._recordings()})
        elif parsed.path == "/api/recordings/download":
            self._download(parsed.query)
        elif parsed.path == "/api/chat/events":
            query = urllib.parse.parse_qs(parsed.query, strict_parsing=True)
            request_id = query.get("request_id", [""])[0]
            if not _SAFE_CHAT_ID.fullmatch(request_id):
                self._reject(HTTPStatus.BAD_REQUEST, "invalid chat request_id")
                return
            try:
                after = int(query.get("after", ["0"])[0])
            except ValueError:
                self._reject(HTTPStatus.BAD_REQUEST, "invalid chat sequence")
                return
            events = self.server.session.chat_events(
                request_id, max(0, after), timeout_ms=20_000
            )
            self._json(HTTPStatus.OK, {"events": events})
        else:
            self._reject(HTTPStatus.NOT_FOUND, "not found")

    def do_POST(self) -> None:  # noqa: N802 — BaseHTTPRequestHandler API
        parsed = urllib.parse.urlsplit(self.path)
        if not parsed.path.startswith("/api/"):
            self._reject(HTTPStatus.NOT_FOUND, "not found")
            return
        if not self._api_auth(mutation=True):
            return
        body = self._read_json()
        if body is None:
            return
        # Fence shutdown against a handler that authenticated just before
        # close. Close sets `closing`, waits for this lock, then performs the
        # final core jog release; no late request can re-engage afterwards.
        self.server.control_lock.acquire()
        try:
            if self.server.closing.is_set():
                self._reject(HTTPStatus.SERVICE_UNAVAILABLE, "the UI is closing")
                return
            if parsed.path == "/api/estop":
                self._json(
                    HTTPStatus.OK,
                    {"status": self.server.session.request_estop()},
                )
            elif parsed.path == "/api/config":
                config = {
                    "joint_step_rad": _positive(
                        "joint_step_rad", body.get("joint_step_rad")
                    ),
                    "linear_step_m": _positive(
                        "linear_step_m", body.get("linear_step_m")
                    ),
                    "angular_step_rad": _positive(
                        "angular_step_rad", body.get("angular_step_rad")
                    ),
                }
                with self.server.config_lock:
                    self.server.config.clear()
                    self.server.config.update(config)
                self._json(HTTPStatus.OK, {"increments": config})
            elif parsed.path == "/api/jog":
                kind = body.get("kind")
                index = body.get("index")
                direction = body.get("direction")
                part = body.get("part")
                if kind not in ("joint", "linear", "angular"):
                    raise ValueError("kind must be joint, linear, or angular")
                if isinstance(index, bool) or not isinstance(index, int) or index < 0:
                    raise ValueError("index must be a non-negative integer")
                if isinstance(direction, bool) or direction not in (-1, 1):
                    raise ValueError("direction must be -1 or +1")
                if part is not None and (not isinstance(part, str) or not part):
                    raise ValueError("part must be a non-empty string or null")
                step_name = {
                    "joint": "joint_step_rad",
                    "linear": "linear_step_m",
                    "angular": "angular_step_rad",
                }[kind]
                with self.server.config_lock:
                    step = self.server.config[step_name]
                accepted, code, detail = self.server.session.jog(
                    kind, index, direction, step, part
                )
                self._json(
                    HTTPStatus.OK,
                    {"accepted": accepted, "code": code, "detail": detail},
                )
            elif parsed.path == "/api/jog/heartbeat":
                accepted, code, detail = self.server.session.jog_heartbeat()
                self._json(
                    HTTPStatus.OK,
                    {"accepted": accepted, "code": code, "detail": detail},
                )
            elif parsed.path == "/api/jog/release":
                self.server.session.jog_release()
                self._json(HTTPStatus.OK, {"released": True})
            elif parsed.path == "/api/chat":
                text = body.get("text")
                if not isinstance(text, str):
                    raise TypeError("text must be a string")
                request_id = secrets.token_urlsafe(18)
                self.server.session.chat_submit(request_id, text)
                self._json(HTTPStatus.ACCEPTED, {"request_id": request_id})
            else:
                self._reject(HTTPStatus.NOT_FOUND, "not found")
        except (TypeError, ValueError) as exc:
            self._reject(HTTPStatus.BAD_REQUEST, str(exc))
        except RuntimeError as exc:
            # Public-safe native errors only; no traceback or internal payload.
            self._reject(HTTPStatus.CONFLICT, str(exc))
        finally:
            self.server.control_lock.release()

    def _manifest_rows(self) -> list[dict[str, Any]]:
        root = self.server.recording_dir
        if root is None:
            return []
        manifest = root / "manifest.jsonl"
        try:
            size = manifest.stat().st_size
            with manifest.open("rb") as file:
                start = max(0, size - _MAX_MANIFEST_WINDOW)
                file.seek(start)
                if start:
                    file.readline(_MAX_MANIFEST_LINE + 1)
                rows: collections.deque[dict[str, Any]] = collections.deque(
                    maxlen=_MAX_RECORDINGS
                )
                while True:
                    line = file.readline(_MAX_MANIFEST_LINE + 1)
                    if not line:
                        break
                    if len(line) > _MAX_MANIFEST_LINE or not line.endswith(b"\n"):
                        continue
                    try:
                        value = json.loads(line)
                    except (UnicodeDecodeError, json.JSONDecodeError):
                        continue
                    if isinstance(value, dict):
                        rows.append(value)
                return list(rows)
        except (FileNotFoundError, OSError):
            return []

    def _resolved_files(
        self, row: dict[str, Any]
    ) -> tuple[str, Path | None, Path | None]:
        root = self.server.recording_dir
        episode_id = row.get("episodeId")
        if root is None or not isinstance(episode_id, str):
            return "", None, None
        if not _SAFE_EPISODE.fullmatch(episode_id):
            return "", None, None
        try:
            root = root.resolve(strict=True)
        except OSError:
            return "", None, None

        sidecar: Path | None = None
        raw_path = row.get("path")
        if isinstance(raw_path, str):
            candidate = Path(raw_path)
            if not candidate.is_absolute():
                candidate = root / candidate
            try:
                candidate = candidate.resolve(strict=True)
                candidate.relative_to(root)
                if (
                    candidate.is_file()
                    and candidate.name == f"{episode_id}.sidecar.json"
                ):
                    sidecar = candidate
            except (OSError, ValueError):
                pass

        mcap: Path | None = None
        candidate = root / f"{episode_id}.mcap"
        try:
            candidate = candidate.resolve(strict=True)
            candidate.relative_to(root)
            if candidate.is_file():
                mcap = candidate
        except (OSError, ValueError):
            pass
        return episode_id, sidecar, mcap

    def _recordings(self) -> list[dict[str, Any]]:
        result: list[dict[str, Any]] = []
        for index, row in enumerate(self._manifest_rows()):
            episode_id, sidecar, mcap = self._resolved_files(row)
            if not episode_id or sidecar is None:
                continue
            end_unix_ns = None
            try:
                if sidecar.stat().st_size <= _MAX_SIDECAR:
                    value = json.loads(sidecar.read_text(encoding="utf-8"))
                    if isinstance(value, dict):
                        end_unix_ns = value.get("tEndUnixNs")
            except (OSError, UnicodeDecodeError, json.JSONDecodeError):
                pass
            downloads = ["sidecar"]
            if mcap is not None:
                downloads.append("mcap")
            result.append(
                {
                    "entry": index,
                    "episode_id": episode_id,
                    "task": row.get("task", ""),
                    "outcome": row.get("outcome", ""),
                    "t_start_unix_ns": row.get("tStartUnixNs"),
                    "t_end_unix_ns": end_unix_ns,
                    "downloads": downloads,
                }
            )
        return result

    def _download(self, query_text: str) -> None:
        try:
            query = urllib.parse.parse_qs(query_text, strict_parsing=True)
            index = int(query.get("entry", [""])[0])
            kind = query.get("kind", [""])[0]
        except (ValueError, KeyError):
            self._reject(HTTPStatus.BAD_REQUEST, "invalid download selector")
            return
        rows = self._manifest_rows()
        if index < 0 or index >= len(rows) or kind not in ("sidecar", "mcap"):
            self._reject(HTTPStatus.NOT_FOUND, "recording not found")
            return
        _episode_id, sidecar, mcap = self._resolved_files(rows[index])
        path = sidecar if kind == "sidecar" else mcap
        if path is None:
            self._reject(HTTPStatus.NOT_FOUND, "recording not found")
            return
        try:
            length = path.stat().st_size
            content_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
            self._headers(
                HTTPStatus.OK,
                content_type,
                length,
                {"Content-Disposition": f'attachment; filename="{path.name}"'},
            )
            with path.open("rb") as file:
                shutil.copyfileobj(file, self.wfile, length=1024 * 1024)
        except OSError:
            # Headers may already be committed; simply close the connection.
            self.close_connection = True


class UIHandle:
    """One authenticated loopback server owned by an active Waddle session."""

    def __init__(
        self,
        session: _core.Session,
        recording_dir: Path | None,
        *,
        joint_step_rad: float,
        linear_step_m: float,
        angular_step_rad: float,
    ) -> None:
        self._session = session
        self._token = secrets.token_urlsafe(32)  # 256 random bits, printed once.
        self._config = {
            "joint_step_rad": _positive("joint_step_rad", joint_step_rad),
            "linear_step_m": _positive("linear_step_m", linear_step_m),
            "angular_step_rad": _positive("angular_step_rad", angular_step_rad),
        }
        self._config_lock = threading.Lock()
        self._closed = False
        self._close_lock = threading.Lock()
        self._server = _Server(
            session,
            recording_dir,
            self._token,
            self._config,
            self._config_lock,
        )
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            name="waddle-ui-loopback",
            daemon=True,
        )
        self._thread.start()
        self.url = f"{self._server.origin}/#token={urllib.parse.quote(self._token)}"

    @property
    def closed(self) -> bool:
        return self._closed

    def close(self) -> None:
        with self._close_lock:
            if self._closed:
                return
            self._closed = True
            self._server.closing.set()
            # Wait out any already-running control request, then make the
            # explicit release the final control operation before server and
            # core teardown.
            with self._server.control_lock:
                self._session.jog_release()
            self._server.shutdown()
            self._server.server_close()
            if threading.current_thread() is not self._thread:
                self._thread.join(timeout=5.0)

    def __enter__(self) -> UIHandle:
        return self

    def __exit__(self, _exc_type: object, _exc: object, _tb: object) -> bool:
        self.close()
        return False
