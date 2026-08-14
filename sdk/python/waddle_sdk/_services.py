"""Public-safe Python facades for optional session services.

The native session owns transport negotiation, correlation, and every
authority decision.  This module contributes ergonomic handles, bounded
local history, input validation, and versioned optional-backend discovery.
"""

from __future__ import annotations

import collections
import importlib.metadata
import math
import secrets
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Iterable, Sequence

if TYPE_CHECKING:
    from . import _core
    from .robots.base import RigSession

_ID_BYTES = 18
_MAX_ID_BYTES = 128
_MAX_TASK_ID_BYTES = 200
_MAX_NAME_BYTES = 200
_MAX_TEXT_BYTES = 4096
_MAX_HISTORY = 1024
_EXECUTION_GROUP = "waddle.execution.v1"


def _bounded_text(
    name: str, value: object, *, maximum: int, empty: bool = False
) -> str:
    if not isinstance(value, str):
        raise TypeError(f"{name} must be a string")
    if (not empty and not value.strip()) or len(value.encode("utf-8")) > maximum:
        qualifier = "non-empty and " if not empty else ""
        raise ValueError(f"{name} must be {qualifier}at most {maximum} UTF-8 bytes")
    return value


def _identifier(name: str, value: object) -> str:
    return _bounded_text(name, value, maximum=_MAX_ID_BYTES)


def _task_identifier(name: str, value: object) -> str:
    return _bounded_text(name, value, maximum=_MAX_TASK_ID_BYTES)


def _timeout_ms(timeout_s: object) -> int:
    if isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)):
        raise TypeError("timeout_s must be a finite non-negative number")
    value = float(timeout_s)
    if not math.isfinite(value) or value < 0.0:
        raise ValueError("timeout_s must be a finite non-negative number")
    return min(30_000, int(value * 1000.0))


def _request_id() -> str:
    return secrets.token_urlsafe(_ID_BYTES)


class TaskSession:
    """One named hosted task conversation and its bounded public event history.

    Constructed by :func:`waddle_sdk.task_session`.  A new handle submits CREATE;
    a resumed handle names an existing durable ``task_session_id``.  Message,
    interjection, and interrupt operations each receive a fresh request id and
    expose ordered public-safe events through :meth:`events`.
    """

    def __init__(
        self,
        session: _core.Session,
        name: str,
        *,
        task_session_id: str | None = None,
    ) -> None:
        self._session = session
        self.name = _bounded_text("name", name, maximum=_MAX_NAME_BYTES)
        self.task_session_id = (
            None
            if task_session_id is None
            else _task_identifier("task_session_id", task_session_id)
        )
        self._requests: dict[str, int] = {}
        self._history: collections.deque[dict[str, Any]] = collections.deque(
            maxlen=_MAX_HISTORY
        )
        self.request_id: str | None = None
        if self.task_session_id is None:
            self._submit("create", name=self.name)
        else:
            self.refresh(after_sequence=0)

    @property
    def history(self) -> tuple[dict[str, Any], ...]:
        """Events observed through this handle, oldest first."""
        return tuple(dict(event) for event in self._history)

    @property
    def request_ids(self) -> tuple[str, ...]:
        return tuple(self._requests)

    def _submit(
        self,
        operation: str,
        *,
        text: str = "",
        name: str = "",
        after_sequence: int = 0,
    ) -> str:
        request_id = _request_id()
        self._session.task_session_submit(
            request_id,
            operation,
            self.task_session_id,
            name or None,
            text or None,
            after_sequence,
        )
        self._requests[request_id] = 0
        self.request_id = request_id
        return request_id

    def _require_id(self) -> str:
        if self.task_session_id is None:
            raise RuntimeError(
                "the create operation has not returned a task_session_id; "
                "poll events() before sending another operation"
            )
        return self.task_session_id

    def message(self, text: str) -> str:
        self._require_id()
        return self._submit(
            "message", text=_bounded_text("text", text, maximum=_MAX_TEXT_BYTES)
        )

    def interject(self, text: str) -> str:
        self._require_id()
        return self._submit(
            "interject", text=_bounded_text("text", text, maximum=_MAX_TEXT_BYTES)
        )

    def interrupt(self) -> str:
        self._require_id()
        return self._submit("interrupt")

    def refresh(self, *, after_sequence: int | None = None) -> str:
        """Request a bounded page of durable history from the plane."""
        self._require_id()
        if after_sequence is None:
            cursors = [
                int(event.get("history_cursor", 0))
                for event in self._history
                if event.get("kind") == "history_complete"
            ]
            cursors.extend(
                int(event.get("sequence", 0))
                for event in self._history
                if event.get("kind") != "history_complete"
            )
            after_sequence = max(cursors, default=0)
        if isinstance(after_sequence, bool) or not isinstance(after_sequence, int):
            raise TypeError("after_sequence must be an integer")
        if after_sequence < 0:
            raise ValueError("after_sequence must be non-negative")
        return self._submit("history", after_sequence=after_sequence)

    def events(
        self,
        *,
        request_id: str | None = None,
        after_sequence: int | None = None,
        timeout_s: float = 0.0,
    ) -> list[dict[str, Any]]:
        """Return new ordered events for one operation and extend history.

        Omitting ``after_sequence`` advances this handle's cursor.  Supplying
        one is useful for stateless consumers; the returned events still join
        the bounded history, with duplicate sequence numbers suppressed.
        """
        selected = request_id or self.request_id
        if selected is None or selected not in self._requests:
            raise ValueError("request_id does not belong to this task session handle")
        after = self._requests[selected] if after_sequence is None else after_sequence
        if isinstance(after, bool) or not isinstance(after, int) or after < 0:
            raise ValueError("after_sequence must be a non-negative integer")
        events = [
            dict(event)
            for event in self._session.task_session_events(
                selected, after, _timeout_ms(timeout_s)
            )
        ]
        seen = {
            (event.get("request_id"), event.get("sequence"))
            for event in self._history
        }
        for event in events:
            if event.get("request_id") != selected:
                continue
            sequence = event.get("sequence")
            if isinstance(sequence, int):
                self._requests[selected] = max(self._requests[selected], sequence)
            task_id = event.get("task_session_id")
            if isinstance(task_id, str) and task_id:
                if self.task_session_id not in (None, task_id):
                    raise RuntimeError("the plane changed this task session's identity")
                self.task_session_id = task_id
            event_name = event.get("name")
            if isinstance(event_name, str) and event_name:
                self.name = event_name
            key = (event.get("request_id"), sequence)
            if key not in seen:
                self._history.append(event)
                seen.add(key)
        return events


@dataclass(frozen=True)
class CalibrationMeasurement:
    calibration_id: str
    sample_id: str
    camera: str
    frame_sequence: int
    session_ns: int
    frame_id: str
    point: tuple[float, float, float]
    depth_m: float


def submit_calibration_click(
    session: _core.Session,
    managed_rig: RigSession | None,
    *,
    calibration_id: str,
    sample_id: str,
    camera: str,
    frame_sequence: int,
    x: int,
    y: int,
) -> CalibrationMeasurement:
    """Resolve one retained RGB-D pixel locally and send only its 3-D point."""
    if managed_rig is None:
        raise RuntimeError("calibration clicks require waddle_sdk.init(rig=...)")
    calibration_id = _identifier("calibration_id", calibration_id)
    sample_id = _identifier("sample_id", sample_id)
    camera = _identifier("camera", camera)
    if isinstance(frame_sequence, bool) or not isinstance(frame_sequence, int):
        raise TypeError("frame_sequence must be an integer")
    if frame_sequence <= 0:
        raise ValueError("frame_sequence must be positive")
    if isinstance(x, bool) or not isinstance(x, int):
        raise TypeError("x must be an integer")
    if isinstance(y, bool) or not isinstance(y, int):
        raise TypeError("y must be an integer")

    description = managed_rig.robot.cameras.get(camera)
    if description is None:
        raise ValueError(f"camera {camera!r} is not declared by this rig")
    if not description.frame_id:
        raise ValueError(f"camera {camera!r} must declare frame_id for calibration")
    sample = managed_rig.camera_sample(camera)
    if sample is None:
        raise RuntimeError(f"camera {camera!r} has not captured a sample")
    if sample.frame_sequence != frame_sequence:
        raise RuntimeError(
            f"camera {camera!r} frame {frame_sequence} is no longer retained; "
            f"latest is {sample.frame_sequence}"
        )
    point = managed_rig.resolve_pixel(
        camera, x, y, frame_sequence=frame_sequence
    )
    depth_m = point[2]
    session.calibration_measurement_submit(
        calibration_id,
        sample_id,
        camera,
        frame_sequence,
        description.frame_id,
        sample.session_ns,
        point,
        depth_m,
    )
    return CalibrationMeasurement(
        calibration_id=calibration_id,
        sample_id=sample_id,
        camera=camera,
        frame_sequence=frame_sequence,
        session_ns=sample.session_ns,
        frame_id=description.frame_id,
        point=point,
        depth_m=depth_m,
    )


def calibration_updates(
    session: _core.Session,
    calibration_id: str,
    *,
    after_sequence: int = 0,
    timeout_s: float = 0.0,
) -> list[dict[str, Any]]:
    calibration_id = _identifier("calibration_id", calibration_id)
    if (
        isinstance(after_sequence, bool)
        or not isinstance(after_sequence, int)
        or after_sequence < 0
    ):
        raise ValueError("after_sequence must be a non-negative integer")
    return [
        dict(update)
        for update in session.calibration_updates(
            calibration_id,
            after_sequence,
            _timeout_ms(timeout_s),
        )
    ]


class WorkspaceArtifactRequest:
    """One signed-workspace request and its bounded ready/status events."""

    def __init__(
        self,
        session: _core.Session,
        graph_ids: Iterable[str],
        calibration_names: Iterable[str],
    ) -> None:
        self._session = session
        self.request_id = _request_id()
        self.graph_ids = _bounded_selection("graph_ids", graph_ids)
        self.calibration_names = _bounded_selection(
            "calibration_names", calibration_names
        )
        session.workspace_artifact_request(
            self.request_id, self.graph_ids, self.calibration_names
        )
        self._history: list[dict[str, Any]] = []

    @property
    def history(self) -> tuple[dict[str, Any], ...]:
        return tuple(dict(event) for event in self._history)

    def events(self, *, timeout_s: float = 0.0) -> list[dict[str, Any]]:
        events = [
            dict(event)
            for event in self._session.workspace_artifact_events(
                self.request_id, _timeout_ms(timeout_s)
            )
        ]
        for event in events:
            if event not in self._history:
                self._history.append(event)
        return events


def _bounded_selection(name: str, values: Iterable[str]) -> tuple[str, ...]:
    if isinstance(values, (str, bytes)):
        raise TypeError(f"{name} must be an iterable of strings")
    result = tuple(_identifier(name.removesuffix("s"), value) for value in values)
    if len(result) > 32:
        raise ValueError(f"{name} may contain at most 32 entries")
    return result


@dataclass(frozen=True)
class ExecutionBackend:
    """A hosted or optional local execution choice.

    Discovery never imports an optional integration.  :meth:`load` is the
    single versioned entry point through which a selected local integration
    enters the SDK process.
    """

    id: str
    label: str
    name: str
    _entry_point: importlib.metadata.EntryPoint | None = field(
        default=None, repr=False, compare=False
    )

    @property
    def local(self) -> bool:
        return self._entry_point is not None

    def load(self) -> object | None:
        return None if self._entry_point is None else self._entry_point.load()

    def public(self) -> dict[str, object]:
        return {
            "id": self.id,
            "label": self.label,
            "name": self.name,
            "local": self.local,
        }


def execution_backends() -> tuple[ExecutionBackend, ...]:
    """Discover Hosted plus versioned optional Local integrations lazily."""
    hosted = ExecutionBackend(id="hosted", label="Hosted", name="Hosted")
    discovered = importlib.metadata.entry_points()
    if hasattr(discovered, "select"):
        entries = discovered.select(group=_EXECUTION_GROUP)
    else:  # pragma: no cover - Python 3.10 importlib.metadata compatibility
        entries = discovered.get(_EXECUTION_GROUP, ())
    local = tuple(
        ExecutionBackend(
            id=f"local:{entry.name}",
            label="Local",
            name=entry.name,
            _entry_point=entry,
        )
        for entry in sorted(entries, key=lambda item: item.name)
    )
    return (hosted, *local)
