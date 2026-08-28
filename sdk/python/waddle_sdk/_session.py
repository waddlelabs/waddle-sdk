"""Private composition seam between Site-owned drivers and the native core.

There is deliberately no module-global session here. Every call returns one
independent native session owned by a SiteSession/RigSession context. Claims,
leases, handoff, gating, clocks, and recording remain native-core behavior.
"""

from __future__ import annotations

import json
from collections.abc import Callable
from dataclasses import dataclass
from os import PathLike

from . import _native
from ._native import core
from .descriptors import Robot, _Space
from .transport import Grpc, LiveKit


@dataclass(frozen=True)
class Control:
    """Driver-facing five-verb wiring, private to SDK composition."""

    send: Callable | None = None
    hold: Callable | None = None
    resume: Callable | None = None
    home: Callable | None = None
    estop: Callable | None = None
    estop_hardware: bool = False
    estop_latency_bound_ms: float | None = None

    def __post_init__(self) -> None:
        if isinstance(self.send, dict):
            raise TypeError("Control.send takes ONE callable")
        for name in ("send", "hold", "resume", "home", "estop"):
            value = getattr(self, name)
            if value is not None and not callable(value):
                raise TypeError(f"Control.{name} must be callable or None")


def _derive_grants(control: Control, space: _Space) -> list[dict]:
    grants: list[dict] = []
    if control.send is not None:
        grants.append({"verb": "VERB_SEND", "sendInterfaces": [space._space_kind()]})
    if control.hold is not None:
        grants.append({"verb": "VERB_HOLD"})
    if control.resume is not None:
        grants.append({"verb": "VERB_RESUME"})
    if control.home is not None:
        grants.append({"verb": "VERB_HOME"})
    if control.estop is not None:
        grant: dict = {"verb": "VERB_ESTOP"}
        if control.estop_hardware:
            grant["hardware"] = True
        if control.estop_latency_bound_ms is not None:
            grant["declaredLatencyBoundNs"] = str(
                int(control.estop_latency_bound_ms * 1_000_000)
            )
        grants.append(grant)
    return grants


def _reset_kwargs(label: str, value: Callable | None) -> dict:
    if value is None:
        return {f"{label}_kind": "none"}
    if not callable(value):
        raise TypeError(f"{label} must be callable or None")

    def wrapped(task: str) -> tuple[bool, bool | None]:
        result = value(task)
        if isinstance(result, bool):
            return result, result
        if (
            isinstance(result, tuple)
            and len(result) == 2
            and isinstance(result[0], bool)
            and (result[1] is None or isinstance(result[1], bool))
        ):
            return result
        raise TypeError(
            "a reset hook must return bool or (bool, Optional[bool]); "
            f"got {result!r}"
        )

    return {f"{label}_kind": "hook", f"{label}_hook": wrapped}


def create_core_session(
    project: str,
    robot: Robot,
    control: Control,
    *,
    recording_dir: str | PathLike | None = None,
    transport: Grpc | None = None,
    media: LiveKit | None = None,
    pre_reset: Callable | None = None,
    post_reset: Callable | None = None,
    reset_verification: str = "blocking",
    _testing: bool = False,
):
    """Build one context-owned native session with fixed safety placement."""
    if not isinstance(robot, Robot):
        raise TypeError("robot must be a waddle_sdk.descriptors.Robot")
    if not isinstance(control, Control):
        raise TypeError("control must be SDK driver composition")
    if transport is not None and not isinstance(transport, Grpc):
        raise TypeError("transport must be a waddle_sdk.Grpc or None")
    if media is not None and not isinstance(media, LiveKit):
        raise TypeError("media must be a waddle_sdk.LiveKit or None")
    if transport is not None and _testing:
        raise ValueError("transport and _testing=True are mutually exclusive")
    if media is not None and _testing:
        raise ValueError("media and _testing=True are mutually exclusive")
    if media is not None and "livekit" not in _native.FEATURES:
        raise RuntimeError(
            "LiveKit media is not compiled into this core; install "
            "waddle-sdk[media]"
        )
    if transport is not None and "grpc" not in _native.FEATURES:
        raise RuntimeError(
            "the control transport is not compiled into this source build; "
            "rebuild the extension with `maturin develop --features grpc`"
        )

    robot_json = json.dumps(robot._compile(_derive_grants(control, robot.action_space)))
    return core.create_session(
        project=project,
        robot_json=robot_json,
        send=control.send,
        hold=control.hold,
        resume=control.resume,
        home=control.home,
        estop=control.estop,
        estop_hardware=control.estop_hardware,
        estop_latency_bound_ns=(
            int(control.estop_latency_bound_ms * 1_000_000)
            if control.estop_latency_bound_ms is not None
            else None
        ),
        recording_dir=None if recording_dir is None else str(recording_dir),
        handoff_kind="hold_first",
        handoff_ns=0,
        lease_enforcement="enforced",
        reset_verification=reset_verification,
        testing_loopback=_testing,
        transport_url=None if transport is None else transport.url,
        transport_token=None if transport is None else transport.token,
        connector_customer_id=None if transport is None else transport.customer_id,
        connector_project_id=None if transport is None else transport.project_id,
        connector_workspace_id=None if transport is None else transport.workspace_id,
        connector_authorization_only=(
            False if transport is None else transport.authorization_only
        ),
        media_url=None if media is None else media.url,
        media_token=None if media is None else media.token,
        **_reset_kwargs("pre_reset", pre_reset),
        **_reset_kwargs("post_reset", post_reset),
    )


__all__: list[str] = []
