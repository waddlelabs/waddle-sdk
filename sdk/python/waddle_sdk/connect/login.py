"""Browser approval without account secrets in shell commands or URLs."""

import os
import socket
import time
import webbrowser
from pathlib import Path
from uuid import UUID

import httpx

from .setup import account_client, checked, origin


def login(endpoint, *, workspace_id=None, key_file=None, open_browser=True):
    endpoint = origin(endpoint)
    if key_file is not None and key_file.expanduser().exists():
        raise ValueError(
            "Credential file already exists; choose a new path or revoke it first"
        )
    with httpx.Client(base_url=endpoint, timeout=20, follow_redirects=False) as client:
        pairing = checked(
            client.post(
                "/api/v1/device-pairing",
                json={
                    "label": socket.gethostname()[:80],
                    "workspace_id": workspace_id,
                },
            )
        )
        # Never follow a verification URL supplied by a different origin.
        url = endpoint + "/activate"
        print(f"Open {url} and enter {pairing['user_code']}.")
        print(
            "Approve only this computer and the intended workspace. Waiting for approval…"
        )
        if open_browser:
            webbrowser.open(url)
        deadline = time.monotonic() + min(float(pairing["expires_in"]), 600)
        interval = max(float(pairing["interval"]), 3)
        token = pairing["device_token"]
        while time.monotonic() < deadline:
            time.sleep(interval)
            result = checked(
                client.post("/api/v1/device-pairing/poll", json={"device_token": token})
            )
            if result["status"] != "approved":
                continue
            workspace = str(UUID(result["workspace_id"]))
            if workspace_id and workspace != str(UUID(workspace_id)):
                raise ValueError(
                    "The approved workspace does not match the requested workspace"
                )
            path = (
                (
                    key_file
                    or Path("~/.config/waddle/devices")
                    / (UUID(token.split(".")[1]).hex + ".key")
                )
                .expanduser()
                .absolute()
            )
            path.parent.mkdir(parents=True, mode=0o700, exist_ok=True)
            # Exclusive creation prevents silently replacing a live connector's credential.
            fd = os.open(
                path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600
            )
            with os.fdopen(fd, "w") as output:
                output.write(token + "\n")
            print(
                f"Connected to workspace {workspace}. Credential expires {result['expires_at']}."
            )
            print(f"Credential file: {path}")
            return path, workspace
    raise ValueError("Pairing expired; run login again")


def logout(endpoint, key_file):
    """Revoke before removing the local key; failed requests leave it available for retry."""
    with account_client(origin(endpoint), key_file) as client:
        checked(client.post("/api/v1/account/devices/logout"))
    key_file.expanduser().unlink()
    print("Device access revoked and local credential removed.")
