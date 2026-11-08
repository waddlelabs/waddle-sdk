"""Pairing keeps secrets out of commands and saves only owner-readable device keys."""

import importlib
import stat
from uuid import uuid4

import httpx
import pytest

login_module = importlib.import_module("waddle_sdk.connect.login")


def test_login_polls_then_saves_private_key_and_logout_revokes(
    monkeypatch, tmp_path, capsys
):
    workspace = str(uuid4())
    token = "wac." + uuid4().hex + "." + "a" * 43
    requests = []

    def handle(request):
        requests.append(request)
        if request.url.path == "/api/v1/device-pairing":
            return httpx.Response(
                200,
                json={
                    "device_token": token,
                    "user_code": "ABCD-EFGH",
                    "expires_in": 600,
                    "interval": 3,
                },
            )
        if request.url.path.endswith("/poll"):
            return httpx.Response(
                200,
                json={
                    "status": "approved",
                    "workspace_id": workspace,
                    "expires_at": "2026-12-08T00:00:00Z",
                },
            )
        assert request.url.path.endswith("/logout")
        assert request.headers["Authorization"] == "Bearer " + token
        return httpx.Response(200, json={"revoked": True})

    real_client = httpx.Client
    monkeypatch.setattr(
        httpx,
        "Client",
        lambda **kwargs: real_client(**kwargs, transport=httpx.MockTransport(handle)),
    )
    monkeypatch.setattr(login_module.time, "sleep", lambda _: None)
    opened = []
    monkeypatch.setattr(login_module.webbrowser, "open", opened.append)
    key = tmp_path / "device.key"
    path, approved = login_module.login(
        "https://app.waddlelabs.ai", workspace_id=workspace, key_file=key
    )
    assert path == key and approved == workspace
    assert stat.S_IMODE(key.stat().st_mode) == 0o600
    assert key.read_text().strip() == token
    assert opened == ["https://app.waddlelabs.ai/activate"]
    assert token not in capsys.readouterr().out
    assert all(token not in str(request.url) for request in requests)
    login_module.logout("https://app.waddlelabs.ai", key)
    assert not key.exists()


def test_expired_pairing_does_not_write_credential(monkeypatch, tmp_path):
    real_client = httpx.Client

    def handle(request):
        return httpx.Response(
            200,
            json={
                "device_token": "unused",
                "user_code": "ABCD-EFGH",
                "expires_in": 0,
                "interval": 3,
            },
        )

    monkeypatch.setattr(
        httpx,
        "Client",
        lambda **kwargs: real_client(**kwargs, transport=httpx.MockTransport(handle)),
    )
    key = tmp_path / "device.key"
    with pytest.raises(ValueError, match="expired"):
        login_module.login(
            "https://app.waddlelabs.ai", key_file=key, open_browser=False
        )
    assert not key.exists()


def test_logout_failure_keeps_key_for_retry(monkeypatch, tmp_path):
    key = tmp_path / "device.key"
    key.write_text("wac." + uuid4().hex + "." + "a" * 43)
    key.chmod(0o600)
    real_client = httpx.Client
    monkeypatch.setattr(
        httpx,
        "Client",
        lambda **kwargs: real_client(
            **kwargs, transport=httpx.MockTransport(lambda _: httpx.Response(503))
        ),
    )
    with pytest.raises(ValueError):
        login_module.logout("https://app.waddlelabs.ai", key)
    assert key.exists()
