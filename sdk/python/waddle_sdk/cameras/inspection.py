"""Explicit camera-only inspection lifecycle.

Hardware discovery remains non-opening. This module provides the separate,
opt-in step for identifying camera views without opening robot parts, a site
transport, control, media publication, or recording.
"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import Any

import numpy as np

from ._factory import CameraFactoryError, open_camera_driver, resolve_camera_factory
from .base import CameraDriver, CameraFrame
from .site import CameraConfig


def _frozen_mapping(value: Mapping[str, Any]) -> Mapping[str, Any]:
    return MappingProxyType({str(key): item for key, item in value.items()})


def _positive_integer(value: object, field_name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"{field_name} must be a positive integer")
    return value


def _nonnegative_timeout(value: float | None, field_name: str) -> float | None:
    if value is None:
        return None
    if isinstance(value, bool):
        raise TypeError(f"{field_name} must be a finite non-negative number or None")
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        raise ValueError(f"{field_name} must be a finite non-negative number or None")
    return number


def _error_kind(error: BaseException) -> str:
    return (type(error).__name__ or "Exception")[:128]


class _CameraFrameValidationError(ValueError):
    """A safe, SDK-owned frame validation failure."""


@dataclass(frozen=True)
class CameraInspectionSpec:
    """One camera that an explicit inspection may open.

    Creating a spec performs no import and opens no hardware. ``driver`` uses
    the same lazy ``module[:attribute]`` form as a camera entry in
    ``site.yaml``. The stream must declare positive integer ``width``,
    ``height``, and ``fps`` values.
    """

    name: str
    driver: str
    connection: Mapping[str, Any]
    stream: Mapping[str, Any]
    options: Mapping[str, Any] = field(default_factory=dict)
    site_root: Path = Path(".")

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not 1 <= len(self.name) <= 128:
            raise ValueError("camera inspection name must contain 1 to 128 characters")
        if not isinstance(self.driver, str) or not 1 <= len(self.driver) <= 512:
            raise ValueError(
                "camera inspection driver must contain 1 to 512 characters"
            )
        stream = dict(self.stream)
        _positive_integer(stream.get("width"), "camera inspection width")
        _positive_integer(stream.get("height"), "camera inspection height")
        _positive_integer(stream.get("fps"), "camera inspection fps")
        object.__setattr__(self, "connection", _frozen_mapping(self.connection))
        object.__setattr__(self, "stream", _frozen_mapping(stream))
        object.__setattr__(self, "options", _frozen_mapping(self.options))
        object.__setattr__(self, "site_root", Path(self.site_root))

    @classmethod
    def from_candidate(
        cls,
        candidate: object,
        *,
        name: str | None = None,
        width: int = 640,
        height: int = 480,
        fps: int = 30,
    ) -> CameraInspectionSpec:
        """Build an unopened spec from exact camera discovery evidence.

        Discovery evidence that does not identify an executable camera driver
        is deliberately insufficient. The site operator must select and
        configure an adapter before that device can be opened.
        """

        if getattr(candidate, "kind", None) != "camera":
            raise ValueError("camera inspection requires a camera candidate")
        driver = getattr(candidate, "driver", None)
        if not isinstance(driver, str) or not driver:
            raise ValueError("camera candidate does not identify an exact SDK driver")
        connection = getattr(candidate, "connection", None)
        if not isinstance(connection, Mapping):
            raise TypeError("camera candidate connection must be a mapping")
        selected_name = name or str(getattr(candidate, "identifier", ""))
        return cls(
            name=selected_name,
            driver=driver,
            connection=connection,
            stream={"width": width, "height": height, "fps": fps},
        )

    def _config(self) -> CameraConfig:
        return CameraConfig(
            name=self.name,
            connection=self.connection,
            stream=self.stream,
            frame_id=None,
            intrinsics=None,
            options=self.options,
            site_root=self.site_root,
        )


@dataclass(frozen=True, eq=False)
class CameraInspectionFrame:
    """The latest immutable frame retained for one inspected camera."""

    camera: str
    sequence: int
    rgb: np.ndarray
    depth: np.ndarray | None = None

    def __post_init__(self) -> None:
        if not self.camera:
            raise ValueError("camera inspection frame must name its camera")
        if (
            isinstance(self.sequence, bool)
            or not isinstance(self.sequence, int)
            or self.sequence <= 0
        ):
            raise ValueError("camera inspection sequence must be a positive integer")
        frame = CameraFrame(rgb=self.rgb, depth=self.depth)
        object.__setattr__(self, "rgb", frame.rgb)
        object.__setattr__(self, "depth", frame.depth)


class CameraInspectionError(RuntimeError):
    """A camera-only inspection could not open or close safely."""


class _LatestFrames:
    def __init__(self, names: tuple[str, ...]) -> None:
        self._names = frozenset(names)
        self._frames: dict[str, CameraInspectionFrame] = {}
        self._errors: dict[str, str] = {}
        self._closed = False
        self._changed = threading.Condition()

    def _check_name(self, name: str) -> None:
        if name not in self._names:
            raise ValueError(f"camera {name!r} is not part of this inspection")

    def publish(self, frame: CameraInspectionFrame) -> None:
        with self._changed:
            self._frames[frame.camera] = frame
            self._changed.notify_all()

    def fail(self, name: str, error: BaseException) -> None:
        with self._changed:
            self._errors[name] = (
                str(error)
                if isinstance(error, _CameraFrameValidationError)
                else f"capture failed ({_error_kind(error)})"
            )
            self._changed.notify_all()

    def close(self) -> None:
        with self._changed:
            self._closed = True
            self._changed.notify_all()

    def latest(self, name: str) -> CameraInspectionFrame | None:
        self._check_name(name)
        with self._changed:
            return self._frames.get(name)

    def wait(
        self,
        name: str,
        *,
        after_sequence: int,
        timeout_s: float | None,
    ) -> CameraInspectionFrame | None:
        self._check_name(name)
        if (
            isinstance(after_sequence, bool)
            or not isinstance(after_sequence, int)
            or after_sequence < 0
        ):
            raise ValueError("after_sequence must be a non-negative integer")
        timeout_s = _nonnegative_timeout(timeout_s, "timeout_s")
        deadline = None if timeout_s is None else time.monotonic() + timeout_s
        with self._changed:
            while True:
                frame = self._frames.get(name)
                if frame is not None and frame.sequence > after_sequence:
                    return frame
                if name in self._errors:
                    return None
                if self._closed:
                    return None
                if deadline is None:
                    self._changed.wait()
                    continue
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._changed.wait(remaining)

    @property
    def errors(self) -> Mapping[str, str]:
        with self._changed:
            return MappingProxyType(dict(self._errors))


class _InspectionPump(threading.Thread):
    def __init__(
        self,
        name: str,
        spec: CameraInspectionSpec,
        driver: CameraDriver,
        latest: _LatestFrames,
    ) -> None:
        super().__init__(name=f"waddle-camera-inspection-{name}", daemon=True)
        self._camera_name = name
        self._spec = spec
        self._driver = driver
        self._latest = latest
        self._stopping = threading.Event()
        self._sequence = 0

    def run(self) -> None:
        period = 1.0 / float(self._spec.stream["fps"])
        deadline = time.monotonic()
        while not self._stopping.is_set():
            try:
                captured = self._driver.capture()
                if self._stopping.is_set():
                    return
                if not isinstance(captured, CameraFrame):
                    raise _CameraFrameValidationError(
                        "capture returned an object other than CameraFrame"
                    )
                expected = (
                    int(self._spec.stream["height"]),
                    int(self._spec.stream["width"]),
                )
                if captured.rgb.shape[:2] != expected:
                    raise _CameraFrameValidationError(
                        f"camera {self._camera_name!r} captured "
                        f"{captured.rgb.shape[1]}x{captured.rgb.shape[0]}, inspection "
                        f"requested {expected[1]}x{expected[0]}"
                    )
                self._sequence += 1
                self._latest.publish(
                    CameraInspectionFrame(
                        camera=self._camera_name,
                        sequence=self._sequence,
                        rgb=captured.rgb,
                        depth=captured.depth,
                    )
                )
            except Exception as exc:  # noqa: BLE001 -- vendor capture boundary
                if not self._stopping.is_set():
                    self._latest.fail(self._camera_name, exc)
                return
            deadline += period
            self._stopping.wait(max(0.0, deadline - time.monotonic()))

    def request_stop(self) -> None:
        self._stopping.set()


class CameraInspectionSession:
    """An entered camera-only inspection that owns drivers and capture threads."""

    def __init__(
        self,
        specs: tuple[CameraInspectionSpec, ...],
        drivers: Mapping[str, CameraDriver],
        *,
        close_timeout_s: float,
    ) -> None:
        self._specs = {spec.name: spec for spec in specs}
        self._drivers = dict(drivers)
        self._names = tuple(self._drivers)
        self._latest = _LatestFrames(self._names)
        self._pumps = {
            name: _InspectionPump(name, self._specs[name], driver, self._latest)
            for name, driver in self._drivers.items()
        }
        self._close_timeout_s = close_timeout_s
        self._close_lock = threading.Lock()
        self._closed = False

    @property
    def names(self) -> tuple[str, ...]:
        """Return camera names in the requested order."""

        return self._names

    @property
    def errors(self) -> Mapping[str, str]:
        """Return capture failures reported by camera threads."""

        return self._latest.errors

    def latest(self, name: str) -> CameraInspectionFrame | None:
        """Return the latest retained frame, or ``None`` before first capture."""

        return self._latest.latest(name)

    def wait(
        self,
        name: str,
        *,
        after_sequence: int = 0,
        timeout_s: float | None = None,
    ) -> CameraInspectionFrame | None:
        """Wait for a newer frame; ``None`` means timeout or capture failure."""

        return self._latest.wait(
            name,
            after_sequence=after_sequence,
            timeout_s=timeout_s,
        )

    def _start(self) -> None:
        started: list[_InspectionPump] = []
        try:
            for pump in self._pumps.values():
                pump.start()
                started.append(pump)
        except BaseException:
            for pump in started:
                pump.request_stop()
            self.close()
            raise

    def close(self) -> None:
        """Close every driver, unblock capture, and join every capture thread."""

        with self._close_lock:
            if self._closed:
                return
            self._closed = True
            for pump in self._pumps.values():
                pump.request_stop()
            self._latest.close()
            failures: list[str] = []
            for name, driver in reversed(tuple(self._drivers.items())):
                try:
                    driver.close()
                except Exception as exc:  # noqa: BLE001 -- vendor close boundary
                    failures.append(f"{name}: close failed ({_error_kind(exc)})")
            for name, pump in self._pumps.items():
                if pump.ident is None:
                    continue
                pump.join(timeout=self._close_timeout_s)
                if pump.is_alive():
                    failures.append(
                        f"{name}: capture did not stop within "
                        f"{self._close_timeout_s:g} seconds"
                    )
            if failures:
                raise CameraInspectionError(
                    "camera inspection did not close cleanly: " + "; ".join(failures)
                )


class CameraInspection:
    """An unopened context for an explicit camera-only inspection."""

    def __init__(
        self,
        specs: Iterable[CameraInspectionSpec],
        *,
        close_timeout_s: float = 5.0,
    ) -> None:
        self._specs = tuple(specs)
        if not self._specs:
            raise ValueError("camera inspection requires at least one camera")
        if any(not isinstance(spec, CameraInspectionSpec) for spec in self._specs):
            raise TypeError("camera inspection specs must be CameraInspectionSpec")
        names = [spec.name for spec in self._specs]
        if len(set(names)) != len(names):
            raise ValueError("camera inspection names must be unique")
        close_timeout = _nonnegative_timeout(close_timeout_s, "close_timeout_s")
        assert close_timeout is not None
        if close_timeout == 0.0:
            raise ValueError("close_timeout_s must be greater than zero")
        self._close_timeout_s = close_timeout
        self._session: CameraInspectionSession | None = None
        self._entered = False

    def __enter__(self) -> CameraInspectionSession:
        if self._entered:
            raise RuntimeError("camera inspection contexts cannot be entered twice")
        self._entered = True
        drivers: dict[str, CameraDriver] = {}
        for spec in self._specs:
            try:
                target = resolve_camera_factory(spec.driver)
                drivers[spec.name] = open_camera_driver(target, spec._config())
            except CameraFactoryError as exc:
                close_failures = self._close_partial(drivers)
                message = f"camera {spec.name!r} could not be configured: {exc}"
                if close_failures:
                    message += "; camera cleanup also failed: " + "; ".join(
                        close_failures
                    )
                raise CameraInspectionError(message) from exc
            except Exception as exc:
                close_failures = self._close_partial(drivers)
                message = f"camera {spec.name!r} failed to open ({_error_kind(exc)})"
                if close_failures:
                    message += "; camera cleanup also failed: " + "; ".join(
                        close_failures
                    )
                raise CameraInspectionError(message) from exc
        try:
            session = CameraInspectionSession(
                self._specs,
                drivers,
                close_timeout_s=self._close_timeout_s,
            )
            self._session = session
            session._start()
            return session
        except Exception as exc:
            close_failures = self._close_partial(drivers)
            message = (
                "camera inspection capture threads failed to start "
                f"({_error_kind(exc)})"
            )
            if close_failures:
                message += "; camera cleanup also failed: " + "; ".join(close_failures)
            raise CameraInspectionError(message) from exc
        except BaseException:
            self._close_partial(drivers)
            raise

    @staticmethod
    def _close_partial(drivers: Mapping[str, CameraDriver]) -> list[str]:
        failures: list[str] = []
        for name, driver in reversed(tuple(drivers.items())):
            try:
                driver.close()
            except Exception as exc:  # noqa: BLE001 -- vendor close boundary
                failures.append(f"{name}: close failed ({_error_kind(exc)})")
        return failures

    def __exit__(self, exc_type, exc, traceback) -> bool:
        session, self._session = self._session, None
        if session is not None:
            try:
                session.close()
            except CameraInspectionError:
                if exc_type is None:
                    raise
        return False


def inspect_cameras(
    specs: Iterable[CameraInspectionSpec],
    *,
    close_timeout_s: float = 5.0,
) -> CameraInspection:
    """Return an unopened camera-only inspection context.

    Entering the context opens only the requested camera drivers and starts a
    latest-only capture thread for each one. It never loads a robot part,
    creates a site session, connects a transport, publishes media, or records
    frames. Leaving the context closes every driver before joining the capture
    threads, which relies on the public ``CameraDriver.close`` contract to
    unblock pending capture.
    """

    return CameraInspection(specs, close_timeout_s=close_timeout_s)


__all__ = [
    "CameraInspection",
    "CameraInspectionError",
    "CameraInspectionFrame",
    "CameraInspectionSession",
    "CameraInspectionSpec",
    "inspect_cameras",
]
