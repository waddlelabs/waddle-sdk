"""Declarative SDK transport selection.

These values contain no connection state. The native core owns dialing,
feature negotiation, reconnect behavior, and connection-scoped safety.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class Grpc:
    """Select the waddle.v0 control transport.

    Connector bindings are all-or-none. authorization_only is reserved for
    the Site lifecycle pre-open authorization probe.
    """

    url: str
    token: str | None = field(default=None, repr=False)
    customer_id: str | None = None
    project_id: str | None = None
    workspace_id: str | None = None
    authorization_only: bool = False

    def __post_init__(self) -> None:
        if not isinstance(self.url, str) or not self.url:
            raise ValueError("Grpc.url must be a non-empty str")
        if self.token is not None and (
            not isinstance(self.token, str) or not self.token
        ):
            raise ValueError("Grpc.token must be a non-empty str or None")
        binding = (self.customer_id, self.project_id, self.workspace_id)
        if any(value is not None for value in binding):
            if not all(isinstance(value, str) and bool(value) for value in binding):
                raise ValueError(
                    "Grpc customer_id, project_id, and workspace_id must all be "
                    "non-empty strings or all be omitted"
                )
        elif self.authorization_only:
            raise ValueError("Grpc.authorization_only requires a connector binding")
        if not isinstance(self.authorization_only, bool):
            raise TypeError("Grpc.authorization_only must be bool")


@dataclass(frozen=True)
class LiveKit:
    """Select the optional LiveKit media transport.

    The companion waddle-sdk-teleop wheel supplies the native feature.
    """

    url: str
    token: str = field(repr=False)

    def __post_init__(self) -> None:
        if not isinstance(self.url, str) or not self.url:
            raise ValueError("LiveKit.url must be a non-empty str")
        if not isinstance(self.token, str) or not self.token:
            raise ValueError("LiveKit.token must be a non-empty str")


__all__ = ["Grpc", "LiveKit"]
