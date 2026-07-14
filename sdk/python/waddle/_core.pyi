"""Typed surface of the PyO3 shim (waddle._core)."""

from collections.abc import Callable, Sequence
from typing import Any

import numpy as np
import numpy.typing as npt

class GateInfo:
    @property
    def kind(self) -> str: ...
    @property
    def provenance(self) -> str | None: ...
    @property
    def progress(self) -> float | None: ...
    @property
    def gripper(self) -> float | None: ...

class Chunk:
    @property
    def steps(self) -> list[tuple[npt.NDArray[np.float64], float | None, int]]: ...
    @property
    def provenance(self) -> str: ...
    @property
    def seq(self) -> int: ...

class Episode:
    @property
    def id(self) -> str: ...
    @property
    def done(self) -> bool: ...
    @property
    def outcome(self) -> str | None: ...
    @property
    def last_gate(self) -> GateInfo | None: ...
    def gate(
        self,
        action: npt.NDArray[np.float64] | Sequence[float],
        obs: npt.NDArray[np.float64] | Sequence[float] | None = None,
        gripper: float | None = None,
    ) -> Any: ...
    def terminate(self, outcome: str = "abort", reason: str = "") -> None: ...

class Session:
    def start_episode(self, task: str) -> Episode: ...
    def shutdown(self) -> None: ...
    def _testing_engage(self, claim_id: str, source: str) -> None: ...
    def _testing_release(self, claim_id: str) -> None: ...
    def _testing_push_teleop(
        self, values: Sequence[float], gripper: float | None = None
    ) -> None: ...

def create_session(
    project: str,
    robot_json: str,
    send: Callable | None = None,
    hold: Callable | None = None,
    resume: Callable | None = None,
    home: Callable | None = None,
    estop: Callable | None = None,
    estop_hardware: bool = False,
    estop_latency_bound_ns: int | None = None,
    recording_dir: str | None = None,
    handoff_kind: str = "hold_first",
    handoff_ns: int = 0,
    lease_enforcement: str = "advisory",
    testing_loopback: bool = False,
) -> Session: ...
def validate_robot_json(json: str) -> None: ...
