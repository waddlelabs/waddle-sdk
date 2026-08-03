#!/usr/bin/env python3
"""The Waddle toy robot: one runnable file that integrates a whole robot.

This is the program a customer writes. It stands up a 6-dof arm with a
parallel gripper and one camera, registers the control verbs Waddle drives
(``send`` / ``hold`` / ``estop``), and runs the six-line rollout loop at
20 Hz — publishing camera frames and proprioception the whole time. The
"robot" is a ~100-line kinematic simulator in this file, so the program is
self-contained; swap :class:`ToyArm` for your real driver and nothing else
about the Waddle side changes.

It runs in three configurations, and the difference is entirely in what you
pass to :func:`waddle.init`:

**Offline (no configuration at all).** Everything below runs: the loop
gates every action, the arm moves, and each episode lands on disk as a
sidecar + MCAP under the recording directory. Nothing is supervised —
there is no plane at the other end — but no code path is stubbed out. This
is the default, and it is what ``python examples/toy_robot.py`` does.

**Connected** (``WADDLE_TOY_TRANSPORT=<grpc url>``, plus
``WADDLE_TOY_TOKEN`` if your plane asks for one). The session's timeline
goes up to the supervision plane, which can now intervene: a claim is
granted, the lease hands over, and the actions arriving at your ``send``
verb are the claimant's rather than your policy's. The camera declares
``still_fps=2``, so 2 JPEG stills/second ride the *control* plane — the one
bounded exception to "no pixels on the control plane" — which is what lets
a Waddle-hosted agent see the scene with no media plane wired at all.

**Agent** (``WADDLE_TOY_MODE=agent``, with a transport). After one warm-up
rollout the program calls :func:`waddle.agent` — "Waddle, drive this one" —
and blocks while a hosted agent claims the episode and drives the arm
through the same ``send`` verb. It prints the result and exits 0 on
success.

Live video for a human teleoperator is a fourth thing and a separate
install (``pip install 'waddle-sdk[teleop]'``); set ``WADDLE_TOY_MEDIA`` +
``WADDLE_TOY_MEDIA_TOKEN`` and the camera's declared ``uplink`` becomes a
real WebRTC track. See ``examples/README.md``.

Configuration (every flag also has an environment variable):

===========================  ================================================
``WADDLE_TOY_MODE``          ``loop`` (default) or ``agent``
``WADDLE_TOY_TRANSPORT``     control-plane gRPC URL; unset = offline
``WADDLE_TOY_TOKEN``         the plane's credential for this session
``WADDLE_TOY_MEDIA``         LiveKit URL (needs the ``[teleop]`` extra)
``WADDLE_TOY_MEDIA_TOKEN``   LiveKit room token (the plane mints it)
``WADDLE_TOY_PROMPT``        the agent-mode prompt
``WADDLE_TOY_EPISODES``      stop after N rollouts; ``0`` (default) = forever
``WADDLE_TOY_EPISODE_SECONDS``  wall-clock length of one rollout
``WADDLE_TOY_AGENT_TIMEOUT``    invite deadline, seconds
``WADDLE_TOY_RECORDING_DIR``    where sidecars + MCAPs land
===========================  ================================================

An EMPTY value counts as unset everywhere above (``WADDLE_TOY_TOKEN=`` is
"no credential", not "the empty credential"), so a harness can pass
``VAR=${MAYBE_UNSET}`` straight through.

Status lines are printed unbuffered, prefixed ``[toy]``, so another process
can drive this one and wait on them::

    [toy] session up ...
    [toy] rollout <n> done <outcome>
    [toy] agent result <outcome> episode=<id>
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

import waddle

# --------------------------------------------------------------------------
# The robot: one source of truth for the kinematics.
#
# `_CHAIN` below is the ONLY place link geometry and joint limits are
# written down. The URDF Waddle records, the `waddle.Joint` limits it plans
# against, and the simulator's own integrator all derive from it — so they
# cannot drift apart, which is the failure this shape exists to prevent.
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class _Link:
    name: str
    axis: tuple[float, float, float]  # unit rotation axis, in the parent frame
    origin: tuple[float, float, float]  # this joint's origin, in the parent frame
    lower: float  # rad
    upper: float  # rad
    max_velocity: float  # rad/s
    max_effort: float  # N*m


_CHAIN: tuple[_Link, ...] = (
    _Link("shoulder_pan", (0.0, 0.0, 1.0), (0.0, 0.0, 0.10), -2.90, 2.90, 2.0, 80.0),
    _Link("shoulder_lift", (0.0, 1.0, 0.0), (0.0, 0.0, 0.08), -1.90, 1.90, 2.0, 80.0),
    _Link("elbow", (0.0, 1.0, 0.0), (0.0, 0.0, 0.22), -2.40, 2.40, 2.5, 60.0),
    _Link("wrist_pitch", (0.0, 1.0, 0.0), (0.0, 0.0, 0.20), -2.00, 2.00, 3.0, 20.0),
    _Link("wrist_yaw", (0.0, 0.0, 1.0), (0.0, 0.0, 0.06), -2.90, 2.90, 3.0, 20.0),
    _Link("wrist_roll", (1.0, 0.0, 0.0), (0.0, 0.0, 0.05), -3.10, 3.10, 3.5, 20.0),
)
#: Fixed offset from the last joint to the tool frame the poses are reported in.
_TOOL_ORIGIN = np.array([0.0, 0.0, 0.06])
_TOOL_FRAME = "tool0"

#: The parallel gripper's declared units: METRES of finger separation, and
#: deliberately not 0/1. Waddle maps a claimant's normalized 0..1 gripper
#: command onto exactly these numbers before it ever reaches `send`, so the
#: verb below can write the value straight to the hardware.
GRIPPER_OPEN_M = 0.0
GRIPPER_CLOSED_M = 0.04

CAMERA_NAME = "wrist"
_CAMERA_FRAME = "wrist_cam"
CAMERA_W, CAMERA_H = 320, 240
CONTROL_HZ = 20.0
#: Frames sampled onto the CONTROL plane for agent perception. Bounded by
#: declaration — 2 fps of small JPEGs, never a video path.
STILL_FPS = 2.0

_HOME = np.zeros(len(_CHAIN))
_LOWER = np.array([link.lower for link in _CHAIN])
_UPPER = np.array([link.upper for link in _CHAIN])
_MAX_VEL = np.array([link.max_velocity for link in _CHAIN])


def urdf() -> bytes:
    """The minimal serial URDF for `_CHAIN`, generated so it can never
    disagree with the model this program actually simulates. Waddle records
    it verbatim on the session; it is what lets anything downstream (a
    viewer, a judge, a retargeting layer) reason in metres instead of
    joint indices."""
    out = ['<?xml version="1.0"?>', '<robot name="waddle_toy">', '  <link name="base_link"/>']
    parent = "base_link"
    for link in _CHAIN:
        child = f"{link.name}_link"
        xyz = " ".join(f"{v:g}" for v in link.origin)
        axis = " ".join(f"{v:g}" for v in link.axis)
        out += [
            f'  <link name="{child}"/>',
            f'  <joint name="{link.name}" type="revolute">',
            f'    <parent link="{parent}"/>',
            f'    <child link="{child}"/>',
            f'    <origin xyz="{xyz}" rpy="0 0 0"/>',
            f'    <axis xyz="{axis}"/>',
            f'    <limit lower="{link.lower:g}" upper="{link.upper:g}" '
            f'velocity="{link.max_velocity:g}" effort="{link.max_effort:g}"/>',
            "  </joint>",
        ]
        parent = child
    tool_xyz = " ".join(f"{v:g}" for v in _TOOL_ORIGIN)
    out += [
        f'  <link name="{_TOOL_FRAME}"/>',
        '  <joint name="tool_mount" type="fixed">',
        f'    <parent link="{parent}"/>',
        f'    <child link="{_TOOL_FRAME}"/>',
        f'    <origin xyz="{tool_xyz}" rpy="0 0 0"/>',
        "  </joint>",
        "</robot>",
    ]
    return "\n".join(out).encode()


def _axis_rotation(axis: tuple[float, float, float], angle: float) -> np.ndarray:
    """Rodrigues' rotation matrix for a unit `axis`."""
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    k = 1.0 - c
    return np.array(
        [
            [x * x * k + c, x * y * k - z * s, x * z * k + y * s],
            [y * x * k + z * s, y * y * k + c, y * z * k - x * s],
            [z * x * k - y * s, z * y * k + x * s, z * z * k + c],
        ]
    )


def forward_kinematics(q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Tool position (metres, base frame) and orientation for joint angles
    `q` — the real FK of `_CHAIN`, not a stand-in."""
    position = np.zeros(3)
    rotation = np.eye(3)
    for angle, link in zip(q, _CHAIN):
        position = position + rotation @ np.asarray(link.origin)
        rotation = rotation @ _axis_rotation(link.axis, float(angle))
    return position + rotation @ _TOOL_ORIGIN, rotation


def quaternion_wxyz(r: np.ndarray) -> tuple[float, float, float, float]:
    """Rotation matrix -> unit quaternion in **wxyz** order (w first).

    wxyz is this protocol's pinned convention; handing it an xyzw
    quaternion is the classic silent-corruption bug, which is why nothing
    here shortcuts through a library's default ordering."""
    trace = r[0, 0] + r[1, 1] + r[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return (0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s)
    if r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        return ((r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s)
    if r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        return ((r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s)
    s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
    return ((r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s)


# Pixel grids for the synthetic camera, built once.
_YY, _XX = np.mgrid[0:CAMERA_H, 0:CAMERA_W]


class ToyArm:
    """A rate-limited kinematic stand-in for a real 6-dof arm.

    Commands set a joint-position target; :meth:`step` walks the state
    toward it at the declared joint velocity limits and clamps to the
    declared position limits. Every method is safe to call from any thread:
    Waddle invokes the control verbs from its own dispatch thread while the
    program's loop runs on the main one, which is exactly the concurrency a
    real driver faces.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._q = _HOME.copy()
        self._qd = np.zeros(len(_CHAIN))
        self._target = _HOME.copy()
        self._gripper = GRIPPER_CLOSED_M
        self._estopped = False
        self._frames = 0

    # -- the control verbs Waddle drives ----------------------------------

    def command(self, joints, gripper: float | None = None) -> None:
        """Set the joint-position target (radians) and, optionally, the
        gripper (METRES — the units this robot declared)."""
        values = np.asarray(joints, dtype=np.float64)
        with self._lock:
            if self._estopped:
                return
            self._target = np.clip(values, _LOWER, _UPPER)
            if gripper is not None:
                self._gripper = float(gripper)

    def hold(self) -> None:
        """Freeze in place: the target becomes wherever the arm is now."""
        with self._lock:
            self._target = self._q.copy()
            self._qd[:] = 0.0

    def estop(self) -> None:
        """Latch stopped. A real e-stop is the owner's hardware envelope;
        Waddle never provides one, it only asks yours to fire."""
        with self._lock:
            self._estopped = True
            self._target = self._q.copy()
            self._qd[:] = 0.0

    # -- the simulation ---------------------------------------------------

    def step(self, dt: float) -> None:
        with self._lock:
            if self._estopped:
                self._qd[:] = 0.0
                return
            delta = np.clip(self._target - self._q, -_MAX_VEL * dt, _MAX_VEL * dt)
            self._q = np.clip(self._q + delta, _LOWER, _UPPER)
            self._qd = delta / dt

    def home(self) -> bool:
        """Snap back to the home pose — what this robot's "reset the scene"
        amounts to. Returns False, having moved nothing, while the e-stop
        is latched.

        The latch deliberately SURVIVES a reset: the envelope is the
        owner's, and clearing it is a human action at the machine
        (:meth:`clear_estop`). A reset flow that cleared it would mean every
        e-stop Waddle asked for got undone by the next episode — by
        supervision, on nobody's authority."""
        with self._lock:
            if self._estopped:
                return False
            self._q = _HOME.copy()
            self._qd[:] = 0.0
            self._target = _HOME.copy()
            self._gripper = GRIPPER_CLOSED_M
            return True

    def clear_estop(self) -> None:
        """Release the latch. Nothing Waddle sends reaches this — it stands
        in for the human who walks up, checks the cell, and twists the
        button back out."""
        with self._lock:
            self._estopped = False

    # -- state readers ----------------------------------------------------

    def joint_positions(self) -> np.ndarray:
        with self._lock:
            return self._q.copy()

    def joint_velocities(self) -> np.ndarray:
        with self._lock:
            return self._qd.copy()

    def gripper(self) -> float:
        with self._lock:
            return self._gripper

    def error_against(self, joints) -> float:
        """Largest per-joint gap between where the arm actually is and
        `joints`.

        Measured against what the SCRIPT asked for, deliberately — not
        against the arm's own last commanded target, which an episode where
        nothing ever dispatched would satisfy perfectly while the robot sat
        still."""
        with self._lock:
            return float(np.max(np.abs(np.asarray(joints, dtype=np.float64) - self._q)))

    def ee_pose(self) -> np.ndarray:
        """xyz + wxyz quaternion of the tool, the 7 values
        ``report_proprio`` takes."""
        position, rotation = forward_kinematics(self.joint_positions())
        return np.array([*position, *quaternion_wxyz(rotation)])

    def render(self) -> np.ndarray:
        """A synthetic 320x240 RGB8 frame: a scrolling gradient plus a blob
        at the tool's projected position, tinted by how open the gripper
        is. Nothing here is Waddle-specific — it stands in for whatever
        your camera hands you, as long as it is a C-contiguous uint8
        ``(height, width, 3)`` array of packed RGB."""
        with self._lock:
            frame_index = self._frames
            self._frames += 1
            gripper = self._gripper
        position, _ = forward_kinematics(self.joint_positions())
        # A fixed orthographic view of the ~1.2 m workspace: x right, z up.
        cx = float(np.clip((position[0] + 0.6) / 1.2, 0.0, 1.0)) * (CAMERA_W - 1)
        cy = float(np.clip((0.7 - position[2]) / 0.9, 0.0, 1.0)) * (CAMERA_H - 1)
        blob = np.exp(-(((_XX - cx) ** 2 + (_YY - cy) ** 2) / (2.0 * 18.0**2)))
        openness = abs(gripper - GRIPPER_CLOSED_M) / abs(GRIPPER_OPEN_M - GRIPPER_CLOSED_M)
        frame = np.empty((CAMERA_H, CAMERA_W, 3), dtype=np.uint8)
        frame[:, :, 0] = ((_XX * 255 // CAMERA_W) + frame_index * 5) % 256
        frame[:, :, 1] = (blob * 255.0).astype(np.uint8)
        frame[:, :, 2] = int(255 * min(max(openness, 0.0), 1.0))
        return frame


def robot_description() -> waddle.Robot:
    """Everything Waddle needs to know about the machine. Declaration only:
    no behavior is decided here."""
    return waddle.Robot(
        name="waddle-toy-arm",
        robot_id="toy-01",
        cell_id="toy-cell",
        action_space=waddle.JointSpace(
            joints=[
                waddle.Joint(
                    name=link.name,
                    min_position=link.lower,
                    max_position=link.upper,
                    max_velocity=link.max_velocity,
                    max_effort=link.max_effort,
                )
                for link in _CHAIN
            ],
            rate_hz=CONTROL_HZ,
            # One action per tick, replaced as soon as the next arrives.
            chunking=waddle.Chunking(horizon=1, replan="immediate", interp="hold"),
            # NOT 0/1: a claimant's normalized command is mapped onto these
            # declared metres before it reaches `send`.
            gripper=waddle.Gripper.parallel(open=GRIPPER_OPEN_M, closed=GRIPPER_CLOSED_M),
        ),
        cameras={
            CAMERA_NAME: waddle.Camera(
                width=CAMERA_W,
                height=CAMERA_H,
                fps=CONTROL_HZ,
                encoding="rgb8",
                frame_id=_CAMERA_FRAME,
                stream_policy=waddle.StreamPolicy(
                    # A low-rate video track for a human teleoperator — live
                    # only when a media plane is wired (the `[teleop]`
                    # extra); inert otherwise.
                    uplink=waddle.Uplink(fps=10, encoding="rgb8", max_kbps=1500),
                    # 2 stills/second onto the CONTROL plane, so a hosted
                    # agent can see the scene with no media plane at all.
                    # Bounded by this declaration, and by nothing else.
                    still_fps=STILL_FPS,
                ),
            )
        },
        kinematics_urdf=urdf(),
        # STATIC transforms only: where the camera is bolted relative to the
        # tool. The tool's own pose moves every tick and is reported as
        # proprioception, not declared here.
        frames=(
            waddle.FrameTransform(
                parent=_TOOL_FRAME, child=_CAMERA_FRAME, position=(0.02, 0.0, 0.01)
            ),
        ),
    )


def status(message: str) -> None:
    """Every line another process might wait on goes through here."""
    print(f"[toy] {message}", flush=True)


def scripted_policy(tick: int) -> tuple[np.ndarray, float]:
    """The stand-in for your policy: a slow joint-space sine, plus a
    gripper that opens and closes once per cycle."""
    t = tick / CONTROL_HZ
    amplitude = np.array([0.6, 0.4, 0.5, 0.3, 0.4, 0.8])
    phase = np.arange(len(_CHAIN)) * (math.pi / 6.0)
    action = np.clip(amplitude * np.sin(2.0 * math.pi * 0.25 * t + phase), _LOWER, _UPPER)
    gripper = GRIPPER_OPEN_M if math.sin(2.0 * math.pi * 0.25 * t) >= 0.0 else GRIPPER_CLOSED_M
    return action, gripper


def robot_tick(session, arm: ToyArm, dt: float) -> None:
    """One turn of the robot's own housekeeping: integrate, publish a
    frame, report proprioception.

    This is deliberately separate from the gate tick, because in agent mode
    it has to keep running on a background thread while the main thread is
    blocked inside ``waddle.agent()`` — the arm still moves, and the agent
    still needs to see it."""
    arm.step(dt)
    # A camera declared with nowhere to send (no media plane, no stills on a
    # connected plane) makes this a cheap no-op — the loop never has to know
    # which configuration it is running in.
    session.publish_frame(CAMERA_NAME, arm.render())
    # joint_pos rides the gate's own `obs`; this adds what the gate never
    # sees. `ee_pose` is 7 values (xyz + wxyz) and must name its frame.
    session.report_proprio(
        joint_vel=arm.joint_velocities(),
        ee_pose=arm.ee_pose(),
        ee_pose_frame=_TOOL_FRAME,
        gripper=arm.gripper(),
    )


class RobotPump(threading.Thread):
    """Runs :func:`robot_tick` at the control rate on its own thread, for
    the stretch where the main thread is blocked inside
    ``waddle.agent()``."""

    def __init__(self, session, arm: ToyArm) -> None:
        super().__init__(name="toy-robot-pump", daemon=True)
        self._session = session
        self._arm = arm
        # NOT `_stop`: threading.Thread already owns that name internally,
        # and shadowing it breaks `join()`.
        self._stopping = threading.Event()

    def run(self) -> None:
        period = 1.0 / CONTROL_HZ
        deadline = time.monotonic()
        while not self._stopping.is_set():
            robot_tick(self._session, self._arm, period)
            deadline += period
            self._stopping.wait(max(0.0, deadline - time.monotonic()))

    def stop(self) -> None:
        self._stopping.set()
        self.join(timeout=5.0)


def run_rollout(session, arm: ToyArm, number: int, task: str, seconds: float) -> str:
    """One supervised episode, driven by the scripted policy.

    The body is the tutorial loop: ask the policy, run the action through
    ``ep.gate``, send what comes back. ``gate`` returns your action when
    nothing is intervening, a different action when something is, and
    ``None`` when you must not send at all (a hold, or an episode Waddle is
    driving) — the one branch every integration needs."""
    period = 1.0 / CONTROL_HZ
    ticks = max(1, int(seconds * CONTROL_HZ))
    action = _HOME
    with waddle.rollout(task=task) as ep:
        status(f"rollout {number} start id={ep.id} task={task!r}")
        deadline = time.monotonic()
        for tick in range(ticks):
            if ep.done:  # the plane can end an episode too
                break
            action, gripper = scripted_policy(tick)
            out = ep.gate(action, arm.joint_positions(), gripper=gripper)
            if out is not None:
                # The gripper is the second half of the answer, and it does
                # NOT ride the return value: when something intervened,
                # `last_gate.gripper` carries the claimant's command already
                # mapped from its normalized 0..1 into the metres this robot
                # declared. It is None on a passthrough tick — nobody
                # overrode you — and then your own value stands. Sending
                # `gripper` unconditionally would move the arm where the
                # teleoperator asked while quietly ignoring the grasp.
                decided = ep.last_gate
                arm.command(out, gripper if decided.gripper is None else decided.gripper)
            robot_tick(session, arm, period)
            deadline += period
            time.sleep(max(0.0, deadline - time.monotonic()))
        if not ep.done:
            # The outcome is the customer's own judgment, and it has to be
            # earned. Measured against the scripted path, so an episode that
            # never dispatched (a hold that ran long, an intervention that
            # went somewhere else) reads as the failure it is rather than as
            # a success with a shrug — inflating the success denominator is
            # exactly what this layer exists to prevent.
            error = arm.error_against(action)
            if error <= 0.15:
                ep.terminate("success", f"tracked the scripted path (max error {error:.3f} rad)")
            else:
                ep.terminate("failure", f"lost the scripted path (max error {error:.3f} rad)")
    outcome = ep.outcome or "unknown"
    status(f"rollout {number} done {outcome}")
    return outcome


def env(name: str, default: str | None = None) -> str | None:
    """Read one of the ``WADDLE_TOY_*`` variables, treating an EMPTY value
    as unset.

    A harness parameterizes a child with ``VAR=${MAYBE_UNSET}``, and an
    empty value there means "I have nothing for this" — not "use the empty
    string", which would otherwise reach ``int("")`` or a credential
    validator as a traceback before the first status line is printed."""
    value = os.environ.get(name, "")
    return value if value.strip() else default


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the Waddle toy robot (see the module docstring).",
    )
    parser.add_argument("--mode", choices=("loop", "agent"), default=env("WADDLE_TOY_MODE", "loop"))
    parser.add_argument("--transport", default=env("WADDLE_TOY_TRANSPORT"))
    parser.add_argument("--token", default=env("WADDLE_TOY_TOKEN"))
    parser.add_argument("--media", default=env("WADDLE_TOY_MEDIA"))
    parser.add_argument("--media-token", default=env("WADDLE_TOY_MEDIA_TOKEN"))
    parser.add_argument("--prompt", default=env("WADDLE_TOY_PROMPT", "pick up the block"))
    parser.add_argument("--episodes", type=int, default=int(env("WADDLE_TOY_EPISODES", "0")))
    parser.add_argument(
        "--episode-seconds", type=float, default=float(env("WADDLE_TOY_EPISODE_SECONDS", "4.0"))
    )
    parser.add_argument(
        "--agent-timeout", type=float, default=float(env("WADDLE_TOY_AGENT_TIMEOUT", "120.0"))
    )
    parser.add_argument(
        "--recording-dir", default=env("WADDLE_TOY_RECORDING_DIR", "toy-recordings")
    )
    args = parser.parse_args(argv)
    # Same rule one layer up, for `--token ""` and friends: empty means
    # unset, so an unset credential stays None rather than becoming an
    # empty one the SDK will (rightly) refuse.
    for name in ("transport", "token", "media", "media_token"):
        setattr(args, name, getattr(args, name) or None)
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    if args.mode == "agent" and not args.transport:
        # `waddle.agent()` refuses a session that declared no transport
        # anyway (there would be nobody to ask) — this only moves that same
        # refusal to before the warm-up rollout, so the program does not do
        # a minute of work and then discover it. The rule itself lives in
        # `waddle.agent`; this example does not restate it.
        status("agent mode needs a supervision plane: set WADDLE_TOY_TRANSPORT=<grpc url>")
        return 2

    arm = ToyArm()

    def send(chunk) -> None:
        """Waddle drives the robot through this, from its own dispatch
        thread, whenever something has the lease: a teleoperator, a reset
        agent, or the hosted agent :func:`waddle.agent` invites. The chunk's
        gripper value already arrives in the metres this robot declared."""
        for values, gripper, _offset_ns in chunk.steps:
            # A real controller would schedule each step at its own
            # `offset_ns`; the toy retargets to the newest and lets its own
            # rate limiter cover the difference.
            arm.command(values, gripper)

    def pre_reset(task: str) -> bool:
        """Scripted scene reset, run before every episode. Returning True
        vouches for it; returning False keeps the episode out of RESETTING
        rather than handing a policy an invalid scene — which is exactly
        what a latched e-stop means here: the arm did not move, so there is
        nothing to vouch for until a human clears it at the machine."""
        status(f"pre_reset {task!r}")
        if not arm.home():
            status("pre_reset refused: e-stop latched (clear it at the robot)")
            return False
        return True

    control = waddle.Control(send=send, hold=arm.hold, estop=arm.estop)

    recording_dir = Path(args.recording_dir).expanduser()
    recording_dir.mkdir(parents=True, exist_ok=True)

    transport = waddle.Grpc(args.transport, args.token) if args.transport else None
    media = waddle.LiveKit(args.media, args.media_token) if args.media else None

    session = waddle.init(
        "waddle-toy-robot",
        robot_description(),
        control,
        recording_dir=recording_dir,
        transport=transport,
        media=media,
        pre_reset=pre_reset,
    )
    status(
        f"session up mode={args.mode} "
        f"transport={args.transport or 'none (offline: local recording only)'} "
        f"media={args.media or 'none'} recording_dir={recording_dir}"
    )

    try:
        if args.mode == "agent":
            return run_agent_mode(session, arm, args)
        return run_loop_mode(session, arm, args)
    except KeyboardInterrupt:
        status("interrupted")
        return 0
    finally:
        waddle.shutdown()
        status("shutdown")


def run_loop_mode(session, arm: ToyArm, args: argparse.Namespace) -> int:
    """Rollouts back to back, forever by default."""
    number = 0
    while args.episodes <= 0 or number < args.episodes:
        number += 1
        run_rollout(session, arm, number, "trace the scripted path", args.episode_seconds)
    return 0


def run_agent_mode(session, arm: ToyArm, args: argparse.Namespace) -> int:
    """One warm-up rollout to prove the robot works, then hand a whole
    episode to Waddle."""
    run_rollout(session, arm, 1, "warm-up before the agent run", args.episode_seconds)

    # `waddle.agent()` blocks this thread for the whole run, so the robot's
    # own loop moves to a background thread: the arm keeps integrating the
    # agent's commands, and the camera keeps feeding the stills the agent
    # perceives through.
    pump = RobotPump(session, arm)
    pump.start()
    status(f"agent invite prompt={args.prompt!r} timeout_s={args.agent_timeout}")
    try:
        result = waddle.agent(args.prompt, timeout_s=args.agent_timeout)
    finally:
        pump.stop()

    status(f"agent result {result.outcome} episode={result.episode_id}")
    status(f"agent detail={result.detail!r} recording_ref={result.recording_ref!r}")
    # An unanswered invite and a declined task both come back as "abort"
    # with a detail, never as an exception.
    return 0 if result.outcome == "success" else 1


if __name__ == "__main__":
    sys.exit(main())
