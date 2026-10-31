"""Local SDK owner and bridge; planning belongs to its remote peer."""

import asyncio
import concurrent.futures
import json
import os
import sys
import threading
import time
from collections import OrderedDict
from dataclasses import replace
from pathlib import Path, PurePosixPath
from uuid import uuid4

from waddle_sdk import LiveKit
from waddle_sdk.runtime import FaultCode, RuntimeFault
from waddle_sdk.workspace.files import Workspace, WorkspaceError

from .errors import RelayError, error_payload
from .protocol import MAX_CALLS, MAX_SAFETY_CALLS, SAFETY_METHODS
from .publisher import publisher_binding
from .wire import decode, encode

SKIP = {"__pycache__", "recordings"}


def is_private_component(name):
    """Exclude hidden credentials/caches and generated recordings from transfer."""
    return name.startswith(".") or name in SKIP


def checked_path(path):
    if (
        not isinstance(path, str)
        or not path
        or PurePosixPath(path).is_absolute()
        or any(not part or is_private_component(part) for part in path.split("/"))
    ):
        raise RelayError(
            "sdk_workspace_path", "Use a public workspace-relative file path"
        )
    return path


class Device:
    """Relay a configured Site through its ordinary local owner and native gates.

    Construction opens no hardware. The application authenticates and authorizes
    the peer before calling :meth:`serve`; this object does not listen or sign in.
    The workspace root must already exist. An uncertain drain retains ownership.
    """

    def __init__(self, site, root, *, torque_release_authorized=False):
        self.site, self.root = site, root
        self.torque_release_authorized = torque_release_authorized
        self.workspace = Workspace(root)
        self.sdk = self.run = None
        self._safety_sdk = None
        self.opened_once = False
        self.disconnected = threading.Event()
        self._admission = threading.RLock()
        self._observations = OrderedDict()
        self._camera_observations = OrderedDict()
        self._observation_bytes = 0
        self._observation_lock = threading.Lock()
        self._publisher = None
        self._media = None
        self._playbacks = OrderedDict()

    def inventory(self):
        result, total = [], 0
        for folder, directories, files in os.walk(self.root, followlinks=False):
            directories[:] = sorted(
                name for name in directories if not is_private_component(name)
            )
            for name in sorted(files):
                path = (Path(folder) / name).relative_to(self.root).as_posix()
                if any(is_private_component(part) for part in path.split("/")):
                    continue
                row = self.workspace.stat(path=checked_path(path))
                if row["kind"] != "file":
                    raise RelayError(
                        "sdk_workspace_type",
                        "Workspace transfer requires regular files",
                    )
                total += row["size"]
                result.append({key: row[key] for key in ("path", "size", "revision")})
                if total > 512 * 1024 * 1024 or len(result) > 10000:
                    raise RelayError(
                        "sdk_workspace_limit",
                        "Workspace transfer exceeds its size/file budget",
                    )
        return result

    def _acquired(self, identity):
        if identity is None:
            return None
        with self._observation_lock:
            observation = self._observations.get(
                identity
            ) or self._camera_observations.get(identity)
        if observation is None:
            raise RelayError(
                "sdk_observation_expired", "The acquired SDK observation has expired"
            )
        return observation[0]

    def dispatch(self, method, arguments):
        if not isinstance(arguments, dict):
            raise RelayError("sdk_bridge_protocol", "SDK arguments must be an object")
        if self.disconnected.is_set():
            raise RelayError("sdk_transport_lost", "The SDK connection has ended")
        if method == "ping":
            return None
        if method in SAFETY_METHODS:
            # These public SDK methods enqueue native control requests. Do not
            # wait for a slow observation/write holding ordinary admission.
            sdk = self._safety_sdk
            if sdk is None:
                raise RelayError("sdk_not_open", "SDK owner is not open")
            return getattr(sdk, method)(**arguments)
        if method == "inventory":
            return self.inventory()
        if method == "workspace":
            operation = arguments.pop("operation", None)
            if operation not in {
                "stat",
                "list",
                "read",
                "write",
                "mkdir",
                "move",
                "delete",
                "apply_patch",
            }:
                raise RelayError("sdk_bridge_protocol", "Unknown workspace operation")
            for name in ("path", "source", "destination"):
                if name in arguments and arguments[name] != ".":
                    checked_path(arguments[name])
            if arguments.get("path") == "." and operation not in {"stat", "list"}:
                raise RelayError(
                    "sdk_workspace_path", "Workspace root cannot be modified"
                )
            return getattr(self.workspace, operation)(**arguments)
        with self._admission:
            if self.disconnected.is_set():
                raise RelayError("sdk_transport_lost", "The SDK connection has ended")
            if method == "publisher":
                publisher = arguments.get("publisher")
                binding = publisher_binding(
                    publisher,
                    allow_loopback_ws=arguments.get("allow_loopback_ws", False),
                )
                if self.sdk is not None and binding != self._publisher:
                    raise RelayError(
                        "media_owner_binding",
                        "A different media binding requires normal SDK drain",
                    )
                if self.sdk is None:
                    self._publisher = binding
                    self._media = (
                        None
                        if publisher is None
                        else LiveKit(
                            publisher["url"],
                            publisher["token"],
                            preview_width=320,
                            preview_fps=2,
                            preview_max_kbps=max(
                                16,
                                256
                                // max(1, len(self.site.manifest.get("cameras", {}))),
                            ),
                            depth_preview=False,
                            demand_driven=True,
                            video_only=True,
                        )
                    )
                return None
            if method == "open":
                if self.opened_once:
                    raise RelayError(
                        "sdk_owner_reused",
                        "A device connection opens its SDK owner only once",
                    )
                self.opened_once = True
                opened = self.site.open(console=False, media=self._media)
                self.sdk = opened
                opened.__enter__()
                self._safety_sdk = opened
                return {"safety_slots": MAX_SAFETY_CALLS}
            if method == "close":
                if self.sdk is not None:
                    if self.torque_release_authorized:
                        arguments = {**arguments, "torque_release_authorized": True}
                    self.sdk.close(**arguments)
                    self.sdk = self.run = None
                    self._safety_sdk = None
                return None
            if self.sdk is None:
                raise RelayError("sdk_not_open", "SDK owner is not open")
            if method == "run_enter":
                if self.run is not None:
                    raise RelayError("sdk_run_busy", "An SDK run is already active")
                self.run = self.sdk.run(**arguments)
                self.run.__enter__()
                return self.run.id
            if method == "run_exit":
                if self.run is not None:
                    self.run.__exit__(None, None, None)
                    self.run = None
                return None
            if method == "playback_start":
                if self.run is None or arguments["run_id"] != self.run.id:
                    raise RelayError(
                        "sdk_run_binding", "Playback requires the active run"
                    )
                start = getattr(self.run, "start_playback", None)
                if not callable(start):
                    raise RuntimeFault(
                        FaultCode.UNSUPPORTED,
                        "SDK run does not support finite playback",
                    )
                # Bound retained handles, never evict running/unconfirmed work.
                for key, handle in list(self._playbacks.items()):
                    if len(self._playbacks) < 128:
                        break
                    if handle.snapshot()["status"] != "running":
                        del self._playbacks[key]
                if len(self._playbacks) >= 128:
                    raise RelayError(
                        "sdk_playback_busy", "Playback handles are occupied"
                    )
                handle = start(arguments["motion"])
                snapshot = handle.snapshot()
                self._playbacks[(snapshot["episode_id"], snapshot["id"])] = handle
                return snapshot
            if method in {
                "playback_status",
                "playback_cancel",
                "playback_update",
                "playback_withdraw",
            }:
                handle = self._playbacks.get((arguments["run_id"], arguments["id"]))
                if handle is None:
                    raise RelayError(
                        "sdk_playback_expired",
                        "Playback does not belong to a retained run",
                    )
                if method == "playback_cancel":
                    handle.cancel(arguments["reason"])
                if method == "playback_update":
                    update = getattr(handle, "update_curve", None)
                    if not callable(update):
                        raise RuntimeFault(
                            FaultCode.UNSUPPORTED,
                            "SDK handle does not support curve updates",
                        )
                    request_id = update(arguments["update"])
                    return {"request_id": request_id, "snapshot": handle.snapshot()}
                if method == "playback_withdraw":
                    withdraw = getattr(handle, "withdraw_curve", None)
                    if not callable(withdraw):
                        raise RuntimeFault(
                            FaultCode.UNSUPPORTED,
                            "SDK handle does not support curve withdrawal",
                        )
                    request_id = withdraw(
                        arguments["request_id"], arguments["expected_revision"]
                    )
                    return {"request_id": request_id, "snapshot": handle.snapshot()}
                # Never wait here: the admission lock must remain available to
                # Hold, disconnect and teardown while local playback advances.
                return handle.snapshot()
            if method in {"step", "step_parts"}:
                if self.run is None:
                    raise RelayError("sdk_not_open", "No active SDK run")
                arguments["observation"] = self._acquired(
                    arguments.pop("observation_id", None)
                )
                return getattr(self.run, method)(**arguments)
            if method == "submit":
                if "observation_id" in arguments:
                    arguments["observation"] = self._acquired(
                        arguments.pop("observation_id")
                    )
                return self.sdk.submit(**arguments)
            if method == "camera_point":
                observation = self._acquired(arguments["observation_id"])
                sample = observation.cameras[arguments["camera"]]
                if sample.point_resolver is None:
                    raise RelayError(
                        "sdk_camera_resolver", "This frame has no vendor point resolver"
                    )
                return sample.point_resolver(
                    arguments["x"], arguments["y"], arguments["depth_m"]
                )
            if method not in {
                "describe",
                "support",
                "observe",
                "observe_parts",
                "forward_kinematics",
                "body_geometry",
                "calibration_measurement",
                "events",
                "media_tracks",
            }:
                raise RelayError("sdk_bridge_protocol", "Unknown SDK operation")
            result = getattr(self.sdk, method)(**arguments)
            if method not in {"observe", "observe_parts"}:
                return result
            identity = uuid4().hex
            with self._observation_lock:
                size = sum(
                    sample.rgb.nbytes
                    + (sample.depth.nbytes if sample.depth is not None else 0)
                    for sample in result.cameras.values()
                )
                if size > 128 * 1024 * 1024:
                    raise RelayError(
                        "sdk_observation_limit", "Acquired cameras exceed 128 MiB"
                    )
                camera_read = bool(result.cameras) and not (
                    method == "observe_parts" and arguments.get("parts")
                )
                retained = (
                    self._camera_observations if camera_read else self._observations
                )
                retained[identity] = (result, size)
                self._observation_bytes += size
                while (
                    len(self._observations) + len(self._camera_observations) > 128
                    or len(self._camera_observations) > 8
                    or self._observation_bytes > 128 * 1024 * 1024
                ):
                    cache = (
                        self._observations
                        if self._observations
                        else self._camera_observations
                    )
                    if len(self._camera_observations) > 8:
                        cache = self._camera_observations
                    _, (_, removed) = cache.popitem(last=False)
                    self._observation_bytes -= removed
            # Control reads do not carry pixels. Explicit camera acquisition and
            # ordinary observe retain the exact paired source frame/depth.
            if method == "observe_parts" and arguments.get("parts"):
                result = replace(result, cameras={})
            return {"observation": result, "observation_id": identity}

    def disconnect(self):
        # Fence queued calls before waiting on an in-flight SDK dispatch. Keep
        # ownership after an uncertain stop/close; never open a replacement owner.
        self.disconnected.set()
        ready = self._safety_sdk
        closing_error = None
        if ready is not None:
            try:
                ready.hold("cloud SDK connection lost")
            except RuntimeFault as error:
                if error.code != FaultCode.NOT_OPEN:
                    raise
                # A concurrent close can already have shut down the native
                # session. Only confirmed closure below resolves this refusal.
                closing_error = error
        with self._admission:
            if self.sdk is not None:
                if closing_error is not None:
                    raise closing_error
                # Startup may have finished while waiting for admission.
                if self.sdk is not ready:
                    self.sdk.hold("cloud SDK connection lost")
                if self.run is not None:
                    self.run.__exit__(None, None, None)
                    self.run = None
                if self.torque_release_authorized:
                    self.sdk.close(torque_release_authorized=True)
                else:
                    self.sdk.close()
                self.sdk = None
                self._safety_sdk = None

    async def _retain_failed_drain(self, error):
        print(
            "SDK shutdown is unconfirmed; ownership is retained for local operator review: "
            + json.dumps(error_payload(error), default=str),
            file=sys.stderr,
            flush=True,
        )
        # An uncertain native close must not turn process exit into permission
        # for a replacement writer. Repeated terminal interrupts cannot release it.
        while True:
            try:
                await asyncio.Future()
            except asyncio.CancelledError:
                continue

    async def serve(self, socket, *, watchdog_s=3, stop=None):
        """Serve one authenticated binary socket, then drain the original owner.

        The socket supplies async iteration, ``send(bytes)`` and
        ``close(code=..., reason=...)``. ``stop`` may be an asyncio Event; setting
        it asks the peer to stop normally, without granting torque release.
        """
        last_seen = time.monotonic()
        pending = {}
        previous = 0
        loop = asyncio.get_running_loop()
        safety_pool = concurrent.futures.ThreadPoolExecutor(
            max_workers=MAX_SAFETY_CALLS, thread_name_prefix="sdk-safety"
        )

        async def respond(request):
            try:
                if request["method"] in SAFETY_METHODS:
                    result = await loop.run_in_executor(
                        safety_pool,
                        self.dispatch,
                        request["method"],
                        request["arguments"],
                    )
                else:
                    result = await asyncio.to_thread(
                        self.dispatch, request["method"], request["arguments"]
                    )
                response = {"id": request["id"], "result": result}
            except WorkspaceError as error:
                response = {
                    "id": request["id"],
                    "error": error.as_dict(),
                    "error_kind": "workspace",
                }
            except Exception as error:  # noqa: BLE001 -- faithfully serialize SDK faults at the process boundary
                response = {"id": request["id"], "error": error_payload(error)}
            try:
                raw = encode(response)
            except (RelayError, TypeError, ValueError) as error:
                raw = encode({"id": request["id"], "error": error_payload(error)})
            await socket.send(raw)
            if request["method"] == "close" and "error" not in response:
                await socket.close(code=1000, reason="sdk_closed")

        async def watchdog():
            while True:
                await asyncio.sleep(min(0.25, watchdog_s / 4))
                deadline = watchdog_s if previous or self.opened_once else 30
                if time.monotonic() - last_seen > deadline:
                    self.disconnected.set()
                    await socket.close(code=1011, reason="sdk_peer_expired")
                    return

        guard = asyncio.create_task(watchdog())

        async def request_stop():
            if stop is None:
                await asyncio.Future()
            else:
                await stop.wait()
                await socket.send(encode({"notice": "device_stop"}))

        stopping = asyncio.create_task(request_stop())
        try:
            async for raw in socket:
                request = decode(raw)
                if (
                    not isinstance(request, dict)
                    or set(request) != {"id", "method", "arguments"}
                    or type(request["id"]) is not int
                    or not previous < request["id"] < 2**53
                    or not isinstance(request["method"], str)
                ):
                    raise RelayError(
                        "sdk_bridge_protocol",
                        "Invalid, repeated or excessive SDK request",
                    )
                safety = request["method"] in SAFETY_METHODS
                limit = MAX_SAFETY_CALLS if safety else MAX_CALLS
                if sum(priority == safety for priority in pending.values()) >= limit:
                    raise RelayError(
                        "sdk_bridge_protocol", "Excessive SDK requests", limit=limit
                    )
                previous = request["id"]
                last_seen = time.monotonic()
                task = asyncio.create_task(respond(request))
                pending[task] = safety

                def finished(task):
                    pending.pop(task, None)
                    if not task.cancelled() and task.exception() is not None:
                        self.disconnected.set()
                        asyncio.create_task(
                            socket.close(code=1011, reason="sdk_response_failed")
                        )

                task.add_done_callback(finished)
        finally:
            self.disconnected.set()
            guard.cancel()
            stopping.cancel()
            await asyncio.gather(guard, stopping, return_exceptions=True)
            try:
                await loop.run_in_executor(safety_pool, self.disconnect)
            except Exception as error:  # noqa: BLE001 -- retain the original owner after unconfirmed native cleanup
                await self._retain_failed_drain(error)
            finally:
                await asyncio.gather(*pending, return_exceptions=True)
                safety_pool.shutdown(wait=False)
