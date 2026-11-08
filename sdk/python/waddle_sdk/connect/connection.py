"""Launch the normal Metal connector and finish its ordinary account binding."""

import json
import os
import signal
import subprocess
import sys
import time
from contextlib import contextmanager
from pathlib import Path
from tempfile import TemporaryDirectory
from uuid import uuid4

from .config import load_connection, load_workspace
from .setup import account_client, checked


@contextmanager
def _launch_lock(workspace):
    """One launcher retains the allocation until its child has normally drained."""
    import fcntl

    path = Path(workspace).parent / ".metal" / "launch.lock"
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise ValueError(
                "This rig's launcher is already running; stop it before reconnecting"
            ) from None
        yield
    finally:
        os.close(fd)


def launch(
    workspace,
    *,
    workspace_id=None,
    endpoint=None,
    key_file=None,
    release_torque_on_exit=False,
):
    with _launch_lock(workspace):
        return _launch(
            workspace,
            workspace_id=workspace_id,
            endpoint=endpoint,
            key_file=key_file,
            release_torque_on_exit=release_torque_on_exit,
        )


def _refresh_allocation(client, workspace_id, allocation):
    current = checked(client.get(f"/api/v1/workspaces/{workspace_id}/allocation"))
    if current.get("id") != allocation["id"]:
        raise ValueError("Cloud allocation changed; stop and restart this rig's setup")
    return current


def _launch(workspace, *, workspace_id, endpoint, key_file, release_torque_on_exit):
    saved = load_connection(workspace)
    if saved is None:
        raise ValueError(
            "Connection settings are missing; run waddle-connect setup again"
        )
    state_file = Path(workspace).parent / ".metal" / "hosted.json"
    state = json.loads(state_file.read_text()) if state_file.exists() else {}
    if workspace_id:
        # Registration precedes hardware opening: the ordinary gateway sends
        # rig_unbound until we bind the selected workspace through the normal API.
        with account_client(endpoint, key_file) as client:
            account = checked(client.get("/api/v1/account"))
            selected = next(
                (w for w in account["workspaces"] if w["id"] == workspace_id), None
            )
            if selected is None or (
                selected.get("rig") and selected["rig"]["name"] != saved.rig
            ):
                raise ValueError(
                    "Workspace binding changed; review Settings → Rig before starting"
                )
            existing = next(
                (r for r in account["rigs"] if r["name"] == saved.rig), None
            )
            if existing and existing.get("workspace_id") not in {None, workspace_id}:
                raise ValueError(
                    "Rig binding changed; review Settings → Rig before starting"
                )
            if (
                existing
                and existing.get("connected")
                and state.get("execution", "local") == "local"
            ):
                raise ValueError(
                    "This rig is already connected; stop its current connector first"
                )
    if release_torque_on_exit and state.get("execution") not in {"cloud", "hybrid"}:
        raise ValueError("--release-torque-on-exit is for cloud or hybrid connections")
    command = (
        [
            sys.executable,
            "-m",
            "waddle_sdk.connect.device",
            "--workspace",
            str(workspace),
        ]
        if state.get("execution") in {"cloud", "hybrid"}
        else [
            str(Path(sys.executable).with_name("waddle-metal")),
            "connect",
            "--workspace",
            str(workspace),
        ]
    )
    if state.get("execution") == "hybrid":
        command += ["--execution", "hybrid"]
    if release_torque_on_exit:
        command.append("--release-torque-on-exit")
    child = None
    allocation = None
    scratch = None
    edge = workspace_id is not None and state.get("execution", "local") == "local"
    # Keep the launcher alive until the connector confirms its ordinary drain.
    # Exiting a terminal session first can hang up its still-owning child.
    # Metal reserves SIGTERM for a separately confirmed restart, including torque
    # release. Stopping this launcher is only an ordinary local shutdown request.
    stop_signal = (
        signal.SIGTERM
        if state.get("execution") in {"cloud", "hybrid"}
        else signal.SIGINT
    )

    def stop(*_):
        if child is None:
            raise KeyboardInterrupt
        child.send_signal(stop_signal)

    previous_term = signal.signal(signal.SIGTERM, stop)
    try:
        if edge:
            config = load_workspace(workspace)
            with account_client(endpoint, key_file) as client:
                rig = checked(
                    client.post(
                        "/api/v1/account/rigs",
                        json={
                            "name": saved.rig,
                            "site_id": config.site.id,
                        },
                    )
                )
                if rig.get("workspace_id") not in {None, workspace_id}:
                    raise ValueError(
                        "Rig binding changed; review Settings → Rig before starting"
                    )
                checked(
                    client.put(
                        f"/api/v1/account/workspaces/{workspace_id}/rig",
                        json={"rig_id": rig["id"]},
                    )
                )
                print("Rig assigned to your app workspace.", flush=True)
                print(
                    "Requesting hosted Twin and harness. Hardware is still closed…",
                    flush=True,
                )
                allocation = checked(
                    client.post(
                        f"/api/v1/workspaces/{workspace_id}/allocation",
                        json={
                            "request_key": str(uuid4()),
                            "placement": "edge-metal",
                            "execution": "local",
                        },
                    )
                )
                while allocation["state"] in {"queued", "starting"}:
                    time.sleep(2)
                    allocation = _refresh_allocation(client, workspace_id, allocation)
                if allocation["state"] != "ready":
                    raise ValueError(
                        "Hosted services stopped before startup: "
                        + json.dumps(
                            allocation.get("error") or {"code": allocation["state"]}
                        )
                    )
            scene = allocation.get("scene_connection")
            if not scene:
                raise ValueError(
                    "Hosted allocation has no scene connection; check its server profile"
                )
            scratch = TemporaryDirectory(prefix="scene-", dir=state_file.parent)
            scene_file = Path(scratch.name) / "connection.json"
            fd = os.open(scene_file, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
            with os.fdopen(fd, "w") as stream:
                json.dump(
                    {**scene, "workspace_id": workspace_id, "site_id": config.site.id},
                    stream,
                )
            command += ["--scene-connection", str(scene_file)]
            print(
                "Hosted services are available. Starting local Metal; "
                "Twin reconstruction follows camera startup…",
                flush=True,
            )
        child = subprocess.Popen(command)
        if workspace_id and not edge:
            with account_client(endpoint, key_file) as client:
                for _ in range(30):
                    if child.poll() is not None:
                        raise ValueError(
                            "Connector stopped before rig registration; see its error above"
                        )
                    account = checked(client.get("/api/v1/account"))
                    selected = next(
                        (w for w in account["workspaces"] if w["id"] == workspace_id),
                        None,
                    )
                    if selected is None or (
                        selected.get("rig") and selected["rig"]["name"] != saved.rig
                    ):
                        raise ValueError(
                            "Workspace binding changed during setup; review Settings → Rig"
                        )
                    rig = next(
                        (r for r in account["rigs"] if r["name"] == saved.rig), None
                    )
                    if rig:
                        config = load_workspace(workspace)
                        if rig.get("site_id") != config.site.id or rig.get(
                            "workspace_id"
                        ) not in {
                            None,
                            workspace_id,
                        }:
                            raise ValueError(
                                "Rig registration conflicts with the selected site or workspace"
                            )
                        checked(
                            client.put(
                                f"/api/v1/account/workspaces/{workspace_id}/rig",
                                json={"rig_id": rig["id"]},
                            )
                        )
                        print("Rig assigned to your app workspace.", flush=True)
                        if state.get("execution") in {"cloud", "hybrid"}:
                            print("Requesting a cloud GPU for this rig…", flush=True)
                            allocation = checked(
                                client.post(
                                    f"/api/v1/workspaces/{workspace_id}/allocation",
                                    json={
                                        "request_key": str(uuid4()),
                                        "placement": "remote-sdk",
                                        "execution": state["execution"],
                                    },
                                )
                            )
                        break
                    time.sleep(1)
                else:
                    raise ValueError(
                        "The app has not registered this rig; check connectivity and rerun setup"
                    )
        last_state = "ready" if edge else None
        while allocation is not None:
            current = allocation["state"]
            if current != last_state:
                message = {
                    "queued": "Waiting for an available cloud GPU. Ctrl+C cancels setup.",
                    "starting": "Cloud GPU assigned. Preparing the planner and robot connection…",
                    "ready": (
                        "Cloud slot is ready. The device is preparing its planner "
                        "and hardware connection…"
                    ),
                    "draining": "Cloud services are stopping…",
                }.get(current)
                if message:
                    print(message, flush=True)
                last_state = current
            if current in {"closed", "cleanup_unconfirmed", "draining"}:
                detail = allocation.get("error") or {"code": current}
                raise ValueError("Cloud connection stopped: " + json.dumps(detail))
            try:
                return child.wait(timeout=2)
            except subprocess.TimeoutExpired:
                with account_client(endpoint, key_file) as client:
                    allocation = _refresh_allocation(client, workspace_id, allocation)
        return child.wait()
    except KeyboardInterrupt:
        # The connector receives terminal Ctrl+C itself. Wait for its ordinary
        # owner cleanup; never force-kill it or silently authorize torque release.
        if child is None:
            return 130
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
        while True:
            try:
                return child.wait()
            except KeyboardInterrupt:
                print("Waiting for normal connector cleanup…", file=sys.stderr)
    finally:
        try:
            if child is not None and child.poll() is None:
                child.send_signal(signal.SIGINT)
                print("Waiting for normal connector cleanup…", file=sys.stderr)
                while child.poll() is None:
                    try:
                        child.wait()
                    except KeyboardInterrupt:
                        continue
        finally:
            signal.signal(signal.SIGTERM, previous_term)
            try:
                if allocation is not None:
                    with account_client(endpoint, key_file) as client:
                        checked(
                            client.delete(
                                f"/api/v1/workspaces/{workspace_id}/allocation",
                                params={"allocation_id": allocation["id"]},
                            )
                        )
            finally:
                if scratch is not None:
                    scratch.cleanup()
