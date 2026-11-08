"""Pair an existing Metal workspace through the ordinary account and rig APIs."""

import json
import os
from pathlib import Path
from urllib.parse import urlsplit
from uuid import UUID

import httpx

from .config import ConnectorConfig, load_connection, load_workspace, read_key


def origin(value):
    parsed = urlsplit(value)
    if (
        parsed.scheme != "https"
        or not parsed.hostname
        or parsed.username
        or parsed.password
        or parsed.path not in {"", "/"}
        or parsed.query
        or parsed.fragment
    ):
        raise ValueError(
            "Use the app's https origin, for example https://app.waddlelabs.ai"
        )
    return value.rstrip("/")


def account_client(endpoint, key_file):
    return httpx.Client(
        base_url=origin(endpoint),
        headers={"Authorization": "Bearer " + read_key(key_file=Path(key_file))},
        timeout=20,
        follow_redirects=False,
    )


def checked(response):
    if response.is_success:
        return response.json()
    try:
        reason = response.json().get("error", "request_failed")
    except ValueError:
        reason = "request_failed"
    raise ValueError(f"App request failed ({response.status_code} {reason})")


def configure(workspace, *, options=None):
    options = dict(options or {})
    if options.keys() - {"endpoint", "key_file", "rig", "workspace_id", "execution"}:
        raise ValueError(
            "Hosted setup accepts endpoint, key_file, rig, workspace_id and execution"
        )
    execution = options.get("execution")
    if execution not in {None, "local", "cloud", "hybrid"}:
        raise ValueError("Choose local, cloud, or hybrid Metal")
    config = load_workspace(workspace)
    if config.site is None:
        raise ValueError("Hosted robot setup requires a configured SDK site")
    endpoint = origin(options.get("endpoint", "https://app.waddlelabs.ai"))
    key_file = (
        Path(options.get("key_file", "~/.config/waddle/account.key"))
        .expanduser()
        .absolute()
    )
    rig = options.get("rig", config.site.id)
    connection = ConnectorConfig(
        "wss://" + urlsplit(endpoint).netloc + "/connect",
        read_key(key_file=key_file),
        rig=rig,
    )
    destination = config.path.parent / ".metal" / "connection.json"
    previous = load_connection(config.path)
    if previous and (
        previous.endpoint != connection.endpoint
        or previous.key_file != key_file
        or previous.rig != rig
        or previous.launcher
    ):
        raise ValueError(
            "This rig has different connection settings; review them before changing accounts"
        )
    workspace_id = options.get("workspace_id")
    with account_client(endpoint, key_file) as client:
        account = checked(client.get("/api/v1/account"))
        if workspace_id:
            workspace_id = str(UUID(workspace_id))
            selected = next(
                (w for w in account["workspaces"] if w["id"] == workspace_id), None
            )
            if selected is None:
                raise ValueError(
                    "The selected workspace does not belong to this account"
                )
            if selected.get("rig") and selected["rig"]["name"] != rig:
                raise ValueError(
                    "The selected workspace already uses another rig; change its binding in the app"
                )
        existing = next((r for r in account["rigs"] if r["name"] == rig), None)
        if existing and existing.get("site_id") != config.site.id:
            raise ValueError(
                "This rig name is already registered for another site; choose a new name"
            )
        if existing and existing.get("workspace_id") not in {None, workspace_id}:
            raise ValueError(
                "This rig is bound elsewhere; move it using Settings → Rig in the app"
            )
        state = destination.with_name("hosted.json")
        saved_state = json.loads(state.read_text()) if state.exists() else {}
        previous_execution = saved_state.get("execution", "local")
        if (
            execution is not None
            and execution != previous_execution
            and existing
            and existing.get("connected")
        ):
            raise ValueError("Stop the existing rig before changing where Metal runs")
    destination.parent.mkdir(mode=0o700, exist_ok=True)
    temporary = destination.with_suffix(".tmp")
    descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        with os.fdopen(descriptor, "w") as stream:
            json.dump(
                {
                    "version": 1,
                    "endpoint": connection.endpoint,
                    "key_file": str(key_file),
                    "allow_loopback_ws": False,
                    "rig": rig,
                },
                stream,
            )
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, destination)
    finally:
        temporary.unlink(missing_ok=True)
    state = destination.with_name("hosted.json")
    descriptor = os.open(
        state, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | os.O_NOFOLLOW, 0o600
    )
    with os.fdopen(descriptor, "w") as stream:
        json.dump(
            {
                "workspace_id": workspace_id,
                **(
                    {"execution": execution}
                    if execution is not None
                    else (
                        {"execution": saved_state["execution"]}
                        if "execution" in saved_state
                        else {}
                    )
                ),
            },
            stream,
        )
    # The gateway registers a rig on its first connection. Binding is finished by
    # the setup launcher while that ordinary connector waits with its SDK closed.
    return {
        "label": "Hosted app",
        "ui_url": endpoint,
        "rig": rig,
        "workspace_id": workspace_id,
    }
