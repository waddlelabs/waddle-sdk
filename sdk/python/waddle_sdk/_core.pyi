"""Typed surface of the PyO3 shim (`waddle_sdk._core`).

Types BOTH compiled cores: the bundled `waddle_sdk._core` and the teleop
companion wheel's `waddle_teleop._core` are the same shim built with
different cargo features (`waddle_sdk._native` picks one), so one stub
describes both. Keep this file in step with `sdk/rust/src/*.rs` — it is
hand-written, and nothing regenerates it.
"""

from collections.abc import Callable, Mapping, Sequence
from typing import Any, Final

import numpy as np
import numpy.typing as npt

__version__: str
BINDING_API_VERSION: Final[int]

# Which connected transports this build carries ("grpc", "livekit"); empty
# for a from-source build with no features. The only feature detection the
# Python layer is allowed to do.
FEATURES: Final[frozenset[str]]

class GateInfo:
    @property
    def kind(self) -> str: ...
    @property
    def provenance(self) -> str | None: ...
    @property
    def progress(self) -> float | None: ...
    @property
    def gripper(self) -> float | None: ...
    @property
    def velocity_feedforward(self) -> list[float] | None: ...
    @property
    def part(self) -> str | None: ...
    def __repr__(self) -> str: ...

class Chunk:
    # A step's values are a float64 ndarray of the declared action space's
    # width, with two departures. A gripper-only step ("hold the arm, move
    # the gripper") carries no arm rows: its array is EMPTY and its gripper
    # is set. And on a Composite declaration the values are that step's rows
    # keyed by declared part instead: one key for a part-scoped action, every
    # declared part for a whole-robot one. The two compose — a gripper-only
    # step on a Composite declaration maps every part to an empty array.
    @property
    def steps(
        self,
    ) -> list[
        tuple[npt.NDArray[np.float64] | dict[str, npt.NDArray[np.float64]], float | None, int]
    ]: ...
    @property
    def velocity_feedforwards(
        self,
    ) -> list[
        npt.NDArray[np.float64] | dict[str, npt.NDArray[np.float64]] | None
    ]: ...
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
    def post_reset_failed(self) -> bool: ...
    @property
    def records_dropped(self) -> int: ...
    @property
    def last_gate(self) -> GateInfo | None: ...
    # Returns the caller's own `action` object on Pass, `None` on Noop/Hold,
    # and on Substitute/Blend a fresh float64 ndarray — or, on a Composite
    # declaration, those rows keyed by declared part (`GateInfo.part` names
    # the addressed one either way).
    def gate(
        self,
        action: npt.NDArray[np.float64] | Sequence[float],
        obs: npt.NDArray[np.float64] | Sequence[float] | None = None,
        gripper: float | None = None,
    ) -> Any: ...
    def terminate(self, outcome: str = "abort", reason: str = "") -> None: ...

class AgentResult:
    @property
    def outcome(self) -> str: ...
    @property
    def episode_id(self) -> str: ...
    @property
    def recording_ref(self) -> str | None: ...
    @property
    def detail(self) -> str: ...
    def __repr__(self) -> str: ...

class SessionStamp:
    @property
    def session_ns(self) -> int: ...
    @property
    def unix_ns(self) -> int: ...
    def __repr__(self) -> str: ...

class Session:
    def stamp(self) -> SessionStamp: ...
    def status(self) -> dict[str, Any]: ...
    def request_estop(self) -> str: ...
    def request_hold(self, reason: str) -> str: ...
    def handoff_remote_to_local(self) -> tuple[bool, str, str]: ...
    def jog(
        self,
        kind: str,
        index: int,
        direction: int,
        step: float,
        part: str | None = None,
    ) -> tuple[bool, str, str]: ...
    def jog_heartbeat(self) -> tuple[bool, str, str]: ...
    def jog_release(self) -> None: ...
    def chat_submit(self, request_id: str, text: str) -> None: ...
    def chat_events(
        self,
        request_id: str,
        after_sequence: int = 0,
        timeout_ms: int = 0,
    ) -> list[dict[str, Any]]: ...
    def task_session_submit(
        self,
        request_id: str,
        operation: str,
        task_session_id: str | None = None,
        name: str | None = None,
        text: str | None = None,
        after_sequence: int = 0,
    ) -> None: ...
    def task_session_events(
        self,
        request_id: str,
        after_sequence: int = 0,
        timeout_ms: int = 0,
    ) -> list[dict[str, Any]]: ...
    def calibration_measurement_submit(
        self,
        calibration_id: str,
        sample_id: str,
        camera: str,
        frame_sequence: int,
        frame_id: str,
        t_ns: int,
        point_xyz: Sequence[float],
        depth_m: float | None = None,
    ) -> None: ...
    def calibration_measurement_requests(
        self,
        after_cursor: int = 0,
        timeout_ms: int = 0,
    ) -> list[dict[str, Any]]: ...
    def calibration_updates(
        self,
        calibration_id: str,
        after_sequence: int = 0,
        timeout_ms: int = 0,
    ) -> list[dict[str, Any]]: ...
    def workspace_artifact_request(
        self,
        request_id: str,
        graph_ids: Sequence[str],
        calibration_names: Sequence[str],
    ) -> None: ...
    def workspace_artifact_events(
        self,
        request_id: str,
        timeout_ms: int = 0,
    ) -> list[dict[str, Any]]: ...
    def start_episode(
        self,
        task: str,
        task_metadata: Mapping[str, str] | None = None,
        pre_reset_kind: str | None = None,
        pre_reset_hook: Callable | None = None,
        pre_reset_prompt: str | None = None,
        pre_reset_timeout_ns: int = 600_000_000_000,
        post_reset_kind: str | None = None,
        post_reset_hook: Callable | None = None,
        post_reset_prompt: str | None = None,
        post_reset_timeout_ns: int = 600_000_000_000,
    ) -> Episode: ...
    def agent(
        self,
        prompt: str,
        timeout_ns: int,
        task_metadata: Mapping[str, str] | None = None,
        pre_reset_kind: str | None = None,
        pre_reset_hook: Callable | None = None,
        pre_reset_prompt: str | None = None,
        pre_reset_timeout_ns: int = 600_000_000_000,
        post_reset_kind: str | None = None,
        post_reset_hook: Callable | None = None,
        post_reset_prompt: str | None = None,
        post_reset_timeout_ns: int = 600_000_000_000,
    ) -> AgentResult: ...
    def publish_frame(self, camera: str, frame: npt.NDArray[np.uint8]) -> None: ...
    def report_proprio(
        self,
        joint_vel: npt.NDArray[np.float64] | Sequence[float] | None = None,
        ee_pose: npt.NDArray[np.float64] | Sequence[float] | None = None,
        ee_pose_frame: str = "ee",
        gripper: float | None = None,
        part: str = "",
        joint_pos: npt.NDArray[np.float64] | Sequence[float] | None = None,
    ) -> None: ...
    def shutdown(self) -> None: ...
    def _testing_frames(self, camera: str) -> list[bytes]: ...
    def _ui_frame(self, camera: str) -> tuple[int, int, bytes] | None: ...
    def _testing_engage(self, claim_id: str, source: str) -> None: ...
    def _testing_release(self, claim_id: str) -> None: ...
    def _testing_push_teleop(
        self, values: Sequence[float], gripper: float | None = None
    ) -> None: ...
    def _testing_push_chunk(
        self,
        values: Sequence[float],
        part: str | None = None,
        gripper: float | None = None,
        offset_ns: int = 0,
        velocity_feedforward: Sequence[float] | None = None,
    ) -> None: ...
    def _testing_reset_window_engage(self, claim_id: str, actor: str) -> None: ...
    def _testing_reset_window_complete(
        self, claim_id: str, ok: bool, verified: bool | None = None
    ) -> None: ...
    def _testing_mark_done(
        self, outcome: str = "success", reason: str = ""
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
    pre_reset_kind: str = "none",
    pre_reset_hook: Callable | None = None,
    pre_reset_prompt: str | None = None,
    pre_reset_timeout_ns: int = 600_000_000_000,
    post_reset_kind: str = "none",
    post_reset_hook: Callable | None = None,
    post_reset_prompt: str | None = None,
    post_reset_timeout_ns: int = 600_000_000_000,
    reset_verification: str = "blocking",
    transport_url: str | None = None,
    transport_token: str | None = None,
    connector_customer_id: str | None = None,
    connector_project_id: str | None = None,
    connector_workspace_id: str | None = None,
    connector_authorization_only: bool = False,
    media_url: str | None = None,
    media_token: str | None = None,
) -> Session: ...
def validate_robot_json(json: str) -> None: ...
def robot_json_roundtrip(json: str) -> str: ...
