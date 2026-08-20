from __future__ import annotations

import io
import json
import urllib.error
import urllib.request
from contextlib import nullcontext
from pathlib import Path
from typing import Any, Self

import pytest
from waddle_sdk import cli
from waddle_sdk._hosted_ui import (
    PROTOCOL_VERSION,
    UiInvitationConfig,
    UiInvitationError,
    WaddleUiInvitationClient,
)


class _Response(io.BytesIO):
    def __enter__(self) -> Self:
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()


def _client() -> WaddleUiInvitationClient:
    return WaddleUiInvitationClient(
        UiInvitationConfig(
            api_url="https://api.waddlelabs.ai",
            api_key="workspace-key",
            customer_id="customer",
            project_id="project",
            workspace_id="workspace",
        )
    )


def test_sdk_invitation_client_reuses_exact_binding_api_key(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    seen: dict[str, Any] = {}

    def urlopen(request: urllib.request.Request, *, timeout: float) -> _Response:
        payload = json.loads(bytes(request.data or b""))
        seen.update(
            url=request.full_url,
            authorization=request.get_header("Authorization"),
            payload=payload,
            timeout=timeout,
        )
        return _Response(
            json.dumps(
                {
                    "protocol_version": PROTOCOL_VERSION,
                    "request_id": payload["request_id"],
                    "url": "/ui?token=wui_sdk-only",
                }
            ).encode()
        )

    monkeypatch.setattr(urllib.request, "urlopen", urlopen)

    assert _client().issue() == "https://api.waddlelabs.ai/ui?token=wui_sdk-only"
    assert seen["url"] == "https://api.waddlelabs.ai/v1/ui/invitations"
    assert seen["authorization"] == "Bearer workspace-key"
    assert seen["payload"]["binding"] == {
        "customer_id": "customer",
        "project_id": "project",
        "workspace_id": "workspace",
    }
    assert seen["timeout"] == 15.0


def test_sdk_invitation_client_is_secret_safe_and_never_retries(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    config = UiInvitationConfig(
        api_url="https://api.waddlelabs.ai",
        api_key="workspace-key",
        customer_id="customer",
        project_id="project",
        workspace_id="workspace",
    )
    assert "workspace-key" not in repr(config)
    with pytest.raises(ValueError, match="absolute https"):
        UiInvitationConfig(
            api_url="http://api.waddlelabs.ai",
            api_key="workspace-key",
            customer_id="customer",
            project_id="project",
            workspace_id="workspace",
        )

    calls = 0

    def urlopen(_request: urllib.request.Request, *, timeout: float) -> _Response:
        nonlocal calls
        del timeout
        calls += 1
        raise urllib.error.URLError("secret transport detail")

    monkeypatch.setattr(urllib.request, "urlopen", urlopen)
    with pytest.raises(UiInvitationError) as captured:
        WaddleUiInvitationClient(config).issue()
    assert calls == 1
    assert "secret transport detail" not in str(captured.value)


def test_sdk_connect_prints_derived_ui_url_after_site_open(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    class Site:
        id = "site"

        def open(self, **kwargs: Any) -> Any:
            assert kwargs["authorization_timeout_s"] == 15.0
            return nullcontext()

    class Invitation:
        def __init__(self, config: UiInvitationConfig) -> None:
            assert config.api_key == "workspace-key"
            assert config.workspace_id == "workspace"

        def issue(self) -> str:
            return "https://api.waddlelabs.ai/ui?token=wui_sdk-only"

    class Stop:
        def set(self) -> None:
            pass

        def wait(self, _timeout: float) -> bool:
            return True

    monkeypatch.setattr(cli, "load_site", lambda _path: Site())
    monkeypatch.setattr(cli, "Grpc", lambda *args, **kwargs: (args, kwargs))
    monkeypatch.setattr(cli, "WaddleUiInvitationClient", Invitation)
    monkeypatch.setattr(cli.threading, "Event", Stop)
    monkeypatch.setattr(cli.signal, "signal", lambda *_args: None)
    monkeypatch.setenv("WADDLE_API_KEY", "workspace-key")
    args = cli._parser().parse_args(
        [
            "connect",
            "--site",
            str(tmp_path / "site.yaml"),
            "--customer",
            "customer",
            "--project",
            "project",
            "--workspace",
            "workspace",
        ]
    )

    assert cli._connect(args) == 0

    output = capsys.readouterr()
    assert output.out.splitlines() == [
        "connected site 'site' to customer/project/workspace",
        "UI: https://api.waddlelabs.ai/ui?token=wui_sdk-only",
    ]
    assert output.err == ""
