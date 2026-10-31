"""Validation of an application-authorized camera publisher grant."""

import ipaddress
from urllib.parse import urlsplit

from jsonschema import Draft202012Validator

from .errors import RelayError

_ID = {"type": "string", "minLength": 1, "maxLength": 128}
PUBLISHER_SCHEMA = {
    "type": "object",
    "additionalProperties": False,
    "required": ["url", "token", "room", "publisher_identity"],
    "properties": {
        "url": {"type": "string", "minLength": 1, "maxLength": 2048},
        "token": {"type": "string", "minLength": 1, "maxLength": 16384},
        "room": _ID,
        "publisher_identity": _ID,
    },
}


def publisher_binding(publisher, *, allow_loopback_ws):
    """Validate transport fields; return URL/room/identity without the token.

    This validates shape and TLS selection, not the authority of a grant.
    Applications must authorize the publisher before supplying it.
    """
    if publisher is None:
        return None
    if not Draft202012Validator(PUBLISHER_SCHEMA).is_valid(publisher):
        raise RelayError("media_configuration", "Publisher configuration is invalid")
    try:
        parsed = urlsplit(publisher["url"])
        port = parsed.port
        try:
            loopback = ipaddress.ip_address(parsed.hostname or "").is_loopback
        except ValueError:
            loopback = False
        valid = (
            bool(parsed.hostname)
            and all(ord(char) > 0x20 for char in publisher["url"])
            and parsed.username is None
            and parsed.password is None
            and not parsed.fragment
            and not parsed.query
            and port != 0
            and (
                parsed.scheme in ("https", "wss")
                or (parsed.scheme in ("http", "ws") and allow_loopback_ws and loopback)
            )
        )
    except ValueError:
        valid = False
    if not valid:
        raise RelayError(
            "media_configuration",
            "Publisher requires a credential-free TLS URL or explicit loopback development URL",
        )
    return publisher["url"], publisher["room"], publisher["publisher_identity"]
