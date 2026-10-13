"""Verify the reviewed derived I2RT distribution without importing vendor code.

The installed package's own version, provenance and RECORD are insufficient:
every reviewed source/asset byte must also match the manifest shipped by the SDK.
Verification establishes source identity, not physical transport qualification.
"""

from __future__ import annotations

import base64
import hashlib
import json
from importlib import metadata
from importlib.resources import files
from pathlib import Path

TRANSPORT_VERSION = "1.1.2+waddle.transport.570ef666"
_MAX_FILE_BYTES = 32 * 1024 * 1024


class I2rtRuntimeError(RuntimeError):
    """A selected derived vendor distribution differs from its reviewed source."""

    code = "hardware.vendor_runtime"

    def __init__(self, detail, **context):
        super().__init__(detail)
        self.detail, self.context = detail, context

    def as_dict(self):
        return {"code": self.code, "detail": self.detail, "context": self.context}


def reviewed_manifest():
    """Return the SDK's source manifest; no vendor imports, downloads or devices."""
    return json.loads(
        (files(__package__) / "yam_data" / "i2rt-transport-runtime.json").read_text()
    )


def verify_transport_runtime(dist=None):
    """Verify exact derived package bytes and return its reviewed source identity.

    This does not accept arbitrary local versions, self-declared provenance or
    changes accompanied by a rewritten installed RECORD. It does not alter any
    installation metadata. Call before importing the selected vendor runtime.
    """
    if dist is None:
        try:
            dist = metadata.distribution("i2rt")
        except metadata.PackageNotFoundError as error:
            raise I2rtRuntimeError(
                "Install the reviewed I2RT transport runtime before selecting USB-CAN"
            ) from error
    expected = reviewed_manifest()
    if dist.version != TRANSPORT_VERSION:
        raise I2rtRuntimeError(
            "The selected I2RT transport runtime has an unreviewed version",
            expected_version=TRANSPORT_VERSION,
            received_version=dist.version,
        )
    entries = {str(item): item for item in dist.files or ()}
    package_files = expected["package_files"]
    package_root = Path(dist.locate_file("i2rt"))
    installed_files = set()
    try:
        if package_root.is_symlink() or not package_root.is_dir():
            raise I2rtRuntimeError(
                "The selected I2RT runtime package directory is invalid"
            )
        for path in package_root.rglob("*"):
            if path.is_symlink():
                raise I2rtRuntimeError(
                    "The selected I2RT runtime contains a symbolic link",
                    file=str(path.relative_to(package_root)),
                )
            if path.is_file() and not path.name.endswith(".pyc"):
                installed_files.add("i2rt/" + path.relative_to(package_root).as_posix())
            if len(installed_files) > 4096:
                raise I2rtRuntimeError(
                    "The selected I2RT runtime file inventory is too large"
                )
    except OSError as error:
        raise I2rtRuntimeError(
            "The selected I2RT runtime cannot be inspected"
        ) from error
    if installed_files != set(package_files):
        raise I2rtRuntimeError(
            "The selected I2RT transport runtime file inventory changed",
            missing_files=sorted(set(package_files) - installed_files),
            extra_files=sorted(installed_files - set(package_files)),
        )
    for name, expected_hash in {**package_files, **expected["metadata"]}.items():
        entry = entries.get(name)
        if entry is None or entry.hash is None or entry.hash.mode != "sha256":
            raise I2rtRuntimeError(
                "A reviewed I2RT runtime file lacks an installed SHA256 package record",
                file=name,
            )
        try:
            with Path(dist.locate_file(entry)).open("rb") as handle:
                data = handle.read(_MAX_FILE_BYTES + 1)
        except OSError as error:
            raise I2rtRuntimeError(
                "A reviewed I2RT runtime file cannot be read", file=name
            ) from error
        digest = hashlib.sha256(data).digest()
        record_hash = base64.urlsafe_b64encode(digest).decode().rstrip("=")
        if len(data) > _MAX_FILE_BYTES or record_hash != entry.hash.value:
            raise I2rtRuntimeError(
                "An installed I2RT runtime file differs from its package record",
                file=name,
            )
        if digest.hex() != expected_hash:
            raise I2rtRuntimeError(
                "An installed I2RT runtime file differs from the SDK's reviewed source",
                file=name,
                expected_sha256=expected_hash,
                received_sha256=digest.hex(),
            )
    return {
        key: expected[key]
        for key in (
            "version",
            "upstream_repository",
            "upstream_commit",
            "transport_commit",
            "transport_patch_sha256",
            "qualification",
        )
    }
