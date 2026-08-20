"""Private one-key bootstrap client for Waddle's hosted workspace UI."""

from __future__ import annotations

import json
import math
import urllib.error
import urllib.request
import uuid
from collections.abc import Mapping
from dataclasses import dataclass, field
from urllib.parse import parse_qs, urljoin, urlsplit

PROTOCOL_VERSION = "waddle.hosted.ui/v1"
BINDING_PROTOCOL_VERSION = "waddle.hosted.binding/v1"
MAX_RESPONSE_BYTES = 64 * 1024


class UiInvitationError(RuntimeError):
    """A secret-safe hosted UI bootstrap failure."""


@dataclass(frozen=True)
class HostedBinding:
    customer_id: str
    project_id: str
    workspace_id: str

    def __post_init__(self) -> None:
        if not self.customer_id or not self.project_id or not self.workspace_id:
            raise ValueError("hosted binding identifiers must be non-empty")


@dataclass(frozen=True)
class UiInvitationConfig:
    api_url: str
    api_key: str = field(repr=False)
    workspace_id: str
    timeout_s: float = 15.0
    allow_insecure: bool = False

    def __post_init__(self) -> None:
        parsed = urlsplit(self.api_url)
        allowed = {"https", "http"} if self.allow_insecure else {"https"}
        if parsed.scheme not in allowed or not parsed.netloc:
            expected = "http(s)" if self.allow_insecure else "https"
            raise ValueError(f"hosted Waddle API must be an absolute {expected} URL")
        if parsed.username or parsed.password or parsed.query or parsed.fragment:
            raise ValueError(
                "hosted Waddle API URL cannot contain credentials/query/fragment"
            )
        if not self.api_key:
            raise ValueError("hosted UI invitation requires a Waddle API key")
        if not self.workspace_id:
            raise ValueError("hosted connection requires a workspace ID")
        if not math.isfinite(self.timeout_s) or self.timeout_s <= 0:
            raise ValueError("hosted UI invitation timeout must be positive and finite")

    @property
    def endpoint(self) -> str:
        return self.api_url.rstrip("/") + "/v1/ui/invitations"

    @property
    def binding_endpoint(self) -> str:
        return self.api_url.rstrip("/") + "/v1/connector/binding"


class WaddleUiInvitationClient:
    def __init__(self, config: UiInvitationConfig) -> None:
        self._config = config

    @staticmethod
    def _json(content: bytes) -> Mapping[str, object]:
        try:
            value = json.loads(content)
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise UiInvitationError(
                "hosted Waddle returned an invalid UI invitation response"
            ) from error
        if not isinstance(value, Mapping):
            raise UiInvitationError(
                "hosted Waddle returned an invalid UI invitation response"
            )
        return value

    def issue(self) -> str:
        request_id = uuid.uuid4().hex
        payload = {
            "protocol_version": PROTOCOL_VERSION,
            "request_id": request_id,
            "workspace_id": self._config.workspace_id,
        }
        request = urllib.request.Request(
            self._config.endpoint,
            data=json.dumps(payload, separators=(",", ":")).encode("utf-8"),
            headers={
                "Authorization": f"Bearer {self._config.api_key}",
                "Content-Type": "application/json",
                "Accept": "application/json",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(
                request, timeout=self._config.timeout_s
            ) as response:
                content = response.read(MAX_RESPONSE_BYTES + 1)
        except urllib.error.HTTPError as error:
            content = error.read(MAX_RESPONSE_BYTES + 1)
            body = self._json(content) if content else {}
            fault = body.get("fault")
            detail = (
                str(fault.get("detail"))[:512]
                if isinstance(fault, Mapping) and fault.get("detail")
                else "hosted Waddle refused the UI invitation"
            )
            raise UiInvitationError(detail) from error
        except (OSError, TimeoutError, urllib.error.URLError) as error:
            raise UiInvitationError(
                "hosted Waddle UI invitation service is unreachable"
            ) from error
        if len(content) > MAX_RESPONSE_BYTES:
            raise UiInvitationError(
                "hosted Waddle UI invitation response exceeded its size bound"
            )
        body = self._json(content)
        if (
            body.get("protocol_version") != PROTOCOL_VERSION
            or body.get("request_id") != request_id
        ):
            raise UiInvitationError(
                "hosted Waddle UI invitation response identity does not match the request"
            )
        relative = body.get("url")
        if not isinstance(relative, str):
            raise UiInvitationError("hosted Waddle UI invitation response has no URL")
        parsed = urlsplit(relative)
        tokens = parse_qs(parsed.query).get("token", [])
        if (
            parsed.scheme
            or parsed.netloc
            or parsed.path != "/ui"
            or parsed.fragment
            or len(tokens) != 1
            or not tokens[0].startswith("wui_")
        ):
            raise UiInvitationError(
                "hosted Waddle UI invitation response has an invalid URL"
            )
        return urljoin(self._config.api_url, relative)

    def resolve_binding(self) -> HostedBinding:
        request_id = uuid.uuid4().hex
        payload = {
            "protocol_version": BINDING_PROTOCOL_VERSION,
            "request_id": request_id,
            "workspace_id": self._config.workspace_id,
        }
        request = urllib.request.Request(
            self._config.binding_endpoint,
            data=json.dumps(payload, separators=(",", ":")).encode("utf-8"),
            headers={
                "Authorization": f"Bearer {self._config.api_key}",
                "Content-Type": "application/json",
                "Accept": "application/json",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(
                request, timeout=self._config.timeout_s
            ) as response:
                content = response.read(MAX_RESPONSE_BYTES + 1)
        except urllib.error.HTTPError as error:
            content = error.read(MAX_RESPONSE_BYTES + 1)
            body = self._json(content) if content else {}
            fault = body.get("fault")
            detail = (
                str(fault.get("detail"))[:512]
                if isinstance(fault, Mapping) and fault.get("detail")
                else "hosted Waddle refused the connector binding"
            )
            raise UiInvitationError(detail) from error
        except (OSError, TimeoutError, urllib.error.URLError) as error:
            raise UiInvitationError(
                "hosted Waddle binding service is unreachable"
            ) from error
        if len(content) > MAX_RESPONSE_BYTES:
            raise UiInvitationError(
                "hosted Waddle binding response exceeded its size bound"
            )
        body = self._json(content)
        if (
            body.get("protocol_version") != BINDING_PROTOCOL_VERSION
            or body.get("request_id") != request_id
        ):
            raise UiInvitationError(
                "hosted Waddle binding response identity does not match the request"
            )
        row = body.get("binding")
        if not isinstance(row, Mapping) or set(row) != {
            "customer_id",
            "project_id",
            "workspace_id",
        }:
            raise UiInvitationError("hosted Waddle binding response is invalid")
        customer_id = row["customer_id"]
        project_id = row["project_id"]
        workspace_id = row["workspace_id"]
        if (
            not isinstance(customer_id, str)
            or not customer_id
            or not isinstance(project_id, str)
            or not project_id
            or not isinstance(workspace_id, str)
            or not workspace_id
        ):
            raise UiInvitationError("hosted Waddle binding response is invalid")
        try:
            binding = HostedBinding(
                customer_id,
                project_id,
                workspace_id,
            )
        except (KeyError, ValueError) as error:
            raise UiInvitationError(
                "hosted Waddle binding response is invalid"
            ) from error
        if binding.workspace_id != self._config.workspace_id:
            raise UiInvitationError("hosted Waddle resolved a different workspace")
        return binding


__all__ = [
    "BINDING_PROTOCOL_VERSION",
    "MAX_RESPONSE_BYTES",
    "PROTOCOL_VERSION",
    "HostedBinding",
    "UiInvitationConfig",
    "UiInvitationError",
    "WaddleUiInvitationClient",
]
