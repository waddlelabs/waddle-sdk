"""Process-local site exclusivity shared by applications using Site.open().

Locks coordinate the same site ID for one OS user; they are not device discovery,
command authority, or a barrier against direct vendor access or different IDs.
"""

from __future__ import annotations

import errno
import hashlib
import os
import stat
import tempfile
from pathlib import Path


class SiteOwnershipError(RuntimeError):
    """A site cannot be acquired or its teardown has not been confirmed."""

    def __init__(self, code: str, detail: str):
        super().__init__(detail)
        self.code = code


class _SiteLock:
    def __init__(self, site_id: str):
        if os.name == "posix":
            directory = Path("/tmp") / f"waddle-sdk-{os.getuid()}"
        elif os.name == "nt":
            directory = Path(tempfile.gettempdir()) / "waddle-sdk-ownership"
        else:
            raise SiteOwnershipError(
                "ownership_unsupported", "Site locking is unsupported on this platform"
            )
        directory.mkdir(mode=0o700, exist_ok=True)
        info = directory.lstat()
        if not stat.S_ISDIR(info.st_mode) or (
            os.name == "posix" and (info.st_uid != os.getuid() or info.st_mode & 0o077)
        ):
            raise SiteOwnershipError(
                "ownership_directory",
                "Site ownership directory is not private to this user",
            )
        key = hashlib.sha256(site_id.encode()).hexdigest()
        path = directory / (key + ".lock")
        if path.is_symlink():
            raise SiteOwnershipError(
                "ownership_file", "Site ownership file must be a regular file"
            )
        fd = os.open(path, os.O_CREAT | os.O_RDWR | getattr(os, "O_NOFOLLOW", 0), 0o600)
        try:
            info = os.fstat(fd)
            if not stat.S_ISREG(info.st_mode) or (
                os.name == "posix"
                and (info.st_uid != os.getuid() or info.st_mode & 0o077)
            ):
                raise SiteOwnershipError(
                    "ownership_file", "Site ownership file is not private to this user"
                )
            if os.name == "posix":
                import fcntl

                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            else:
                import msvcrt

                msvcrt.locking(fd, msvcrt.LK_NBLCK, 1)
        except OSError as error:
            os.close(fd)
            if error.errno in (errno.EACCES, errno.EAGAIN, errno.EDEADLK):
                raise SiteOwnershipError(
                    "site_owned", "Another local process owns this SDK site"
                ) from error
            raise
        except BaseException:
            os.close(fd)
            raise
        self.fd = fd

    def release(self):
        # Never unlink: another opener could otherwise lock a different inode.
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
