"""Bounded customer-local files, with revision checks and atomic replacements."""

from __future__ import annotations

import base64
import binascii
import ctypes
import errno
import fcntl
import hashlib
import json
import mimetypes
import os
import re
import stat as stat_mode
import subprocess
import sys
import uuid
from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path

MAX_FILE_BYTES = 8 * 1024 * 1024
MAX_READ_BYTES = 256 * 1024
MAX_ENTRIES = 1000
MAX_DEPTH = 64
MAX_SEARCH_BYTES = 8 * 1024 * 1024
_DIRECTORY_FLAGS = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW


class WorkspaceError(ValueError):
    def __init__(self, code: str, detail: str):
        self.code, self.detail = code, detail
        super().__init__(detail)

    def as_dict(self) -> dict:
        return {"code": self.code, "detail": self.detail}


def _fail(code: str, detail: str) -> None:
    raise WorkspaceError("workspace." + code, detail)


def _bound(value: int, maximum: int, name: str, minimum: int = 1) -> int:
    if type(value) is not int or not minimum <= value <= maximum:
        _fail(
            "invalid_argument", f"{name} must be an integer from {minimum} to {maximum}"
        )
    return value


def _parts(path: str) -> tuple[str, ...]:
    if not isinstance(path, str) or not path or "\x00" in path or "\\" in path:
        _fail("invalid_path", "Use a nonempty workspace-relative POSIX path")
    if path == ".":
        return ()
    parts = tuple(path.split("/"))
    if any(part in ("", ".", "..") for part in parts):
        _fail(
            "invalid_path", "Absolute paths, empty components, '.' and '..' are refused"
        )
    if len(path.encode()) > 4096:
        _fail("invalid_path", "Path exceeds 4096 bytes")
    return parts


def _revision(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _expected(value: str | None) -> None:
    if value is not None and (
        not isinstance(value, str) or not re.fullmatch(r"[0-9a-f]{64}", value)
    ):
        _fail(
            "invalid_revision",
            "Expected revision must be a SHA256 hex string or explicit null for absence",
        )


def _kind(info: os.stat_result) -> str:
    if stat_mode.S_ISLNK(info.st_mode):
        return "symlink"
    if stat_mode.S_ISDIR(info.st_mode):
        return "directory"
    if stat_mode.S_ISREG(info.st_mode):
        return "file"
    return "special"


def _rename_absent(
    source_parent: int, source: str, destination_parent: int, destination: str
) -> None:
    """Linux atomic no-replace rename, including against uncooperative creators."""
    libc = ctypes.CDLL(None, use_errno=True)
    rename = libc.renameat2
    rename.argtypes = (
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_uint,
    )
    rename.restype = ctypes.c_int
    if rename(
        source_parent,
        os.fsencode(source),
        destination_parent,
        os.fsencode(destination),
        1,
    ):
        error = ctypes.get_errno()
        if error == errno.EEXIST:
            raise FileExistsError(error, os.strerror(error))
        raise OSError(error, os.strerror(error))


def _read_file(parent: int, name: str) -> tuple[bytes, os.stat_result]:
    fd = os.open(name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=parent)
    with os.fdopen(fd, "rb") as handle:
        before = os.fstat(handle.fileno())
        if not stat_mode.S_ISREG(before.st_mode):
            _fail("unsupported_type", "Only regular files have readable content")
        if before.st_size > MAX_FILE_BYTES:
            _fail("size_limit", f"File exceeds the {MAX_FILE_BYTES}-byte content limit")
        data = handle.read(MAX_FILE_BYTES + 1)
        after = os.fstat(handle.fileno())
        if len(data) > MAX_FILE_BYTES:
            _fail("size_limit", "File grew past the content limit")
        if (before.st_size, before.st_mtime_ns, before.st_ctime_ns) != (
            after.st_size,
            after.st_mtime_ns,
            after.st_ctime_ns,
        ):
            _fail("conflict", "File changed while being read; read it again")
        return data, after


# Run regexes in a bounded child: a legal but pathological regex cannot stall
# the connector's control lane. Literal search stays in-process.
_REGEX_SEARCH = r"""
import json, re, sys
request = json.load(sys.stdin)
pattern = re.compile(request["query"])
found = []
for path, text in request["files"]:
    for number, line in enumerate(text.splitlines(), 1):
        for match in pattern.finditer(line):
            found.append({"path": path, "line": number, "column": match.start() + 1,
                          "text": line[:1024]})
            if len(found) >= request["limit"]:
                print(json.dumps(found))
                sys.exit(0)
print(json.dumps(found))
"""


class Workspace:
    """One explicitly selected local root; shell tasks are not sandboxed by it."""

    def __init__(self, root: Path):
        self.root = Path(root).resolve(strict=True)
        if not self.root.is_dir():
            _fail("not_directory", "Workspace root must be an existing directory")

    @contextmanager
    def _locked(self, *, lock: bool = True) -> Iterator[int]:
        try:
            fd = os.open(self.root, _DIRECTORY_FLAGS)
            try:
                # A root-directory flock serializes cooperating service instances
                # without introducing a lock file into customer data.
                if lock:
                    fcntl.flock(fd, fcntl.LOCK_EX)
                yield fd
            finally:
                os.close(fd)
        except WorkspaceError:
            raise
        except FileNotFoundError:
            _fail("not_found", "Workspace path does not exist")
        except FileExistsError:
            _fail("conflict", "Destination already exists")
        except NotADirectoryError:
            _fail("invalid_path", "A path component is a symlink or is not a directory")
        except OSError as exc:
            _fail("io_error", f"Workspace operation failed: {exc.strerror}")

    @contextmanager
    def _parent(self, root: int, path: str) -> Iterator[tuple[int, str]]:
        parts = _parts(path)
        fd = os.dup(root)
        try:
            for component in parts[:-1]:
                next_fd = os.open(component, _DIRECTORY_FLAGS, dir_fd=fd)
                os.close(fd)
                fd = next_fd
            yield fd, parts[-1] if parts else "."
        finally:
            os.close(fd)

    def directory_fd(self, *, path: str) -> int:
        """Return an owned directory descriptor for a task's race-safe cwd."""
        with (
            self._locked(lock=False) as root,
            self._parent(root, path) as (parent, name),
        ):
            return os.open(name, _DIRECTORY_FLAGS, dir_fd=parent)

    def _metadata(self, parent: int, name: str, path: str) -> dict:
        info = os.stat(name, dir_fd=parent, follow_symlinks=False)
        kind = _kind(info)
        result = {
            "path": path,
            "kind": kind,
            "size": info.st_size,
            "modified_ns": info.st_mtime_ns,
        }
        if kind == "file":
            data, info = _read_file(parent, name)
            result.update(
                revision=_revision(data), size=len(data), modified_ns=info.st_mtime_ns
            )
        elif kind == "directory":
            fd = os.open(name, _DIRECTORY_FLAGS, dir_fd=parent)
            try:
                rows = []
                with os.scandir(fd) as entries:
                    for entry in entries:
                        if len(rows) >= MAX_ENTRIES:
                            _fail(
                                "entry_limit",
                                "Directory exceeds the revision entry limit",
                            )
                        item = entry.stat(follow_symlinks=False)
                        rows.append(
                            (
                                entry.name,
                                _kind(item),
                                item.st_ino,
                                item.st_size,
                                item.st_mtime_ns,
                                item.st_ctime_ns,
                            )
                        )
                encoded = json.dumps(
                    (info.st_ino, info.st_mtime_ns, sorted(rows)), separators=(",", ":")
                ).encode()
                result["revision"] = _revision(encoded)
            finally:
                os.close(fd)
        else:
            _fail("unsupported_type", "Symlinks and special files are refused")
        return result

    def _check(
        self, parent: int, name: str, path: str, expected: str | None
    ) -> dict | None:
        _expected(expected)
        try:
            current = self._metadata(parent, name, path)
        except FileNotFoundError:
            current = None
        actual = current["revision"] if current else None
        if actual != expected:
            _fail(
                "conflict",
                f"Revision conflict for {path}; read current state before retrying",
            )
        return current

    def stat(self, *, path: str = ".") -> dict:
        with self._locked() as root, self._parent(root, path) as (parent, name):
            return self._metadata(parent, name, path)

    def _walk(
        self, fd: int, prefix: str, recursive: bool, budget: int
    ) -> tuple[list[dict], bool]:
        rows: list[dict] = []
        truncated = False

        def visit(directory: int, base: str, depth: int = 0) -> None:
            nonlocal truncated
            with os.scandir(directory) as entries:
                for entry in entries:
                    if len(rows) == budget:
                        truncated = True
                        return
                    info = entry.stat(follow_symlinks=False)
                    path = entry.name if base == "." else base + "/" + entry.name
                    kind = _kind(info)
                    rows.append(
                        {
                            "path": path,
                            "kind": kind,
                            "size": info.st_size,
                            "modified_ns": info.st_mtime_ns,
                        }
                    )
                    if recursive and kind == "directory":
                        if depth == MAX_DEPTH:
                            truncated = True
                            return
                        child = os.open(entry.name, _DIRECTORY_FLAGS, dir_fd=directory)
                        try:
                            visit(child, path, depth + 1)
                        finally:
                            os.close(child)
                        if truncated:
                            return

        visit(fd, prefix)
        return rows, truncated

    def list(
        self, *, path: str = ".", recursive: bool = False, limit: int = 200
    ) -> dict:
        _bound(limit, MAX_ENTRIES, "limit")
        if type(recursive) is not bool:
            _fail("invalid_argument", "recursive must be a boolean")
        with self._locked() as root, self._parent(root, path) as (parent, name):
            fd = os.open(name, _DIRECTORY_FLAGS, dir_fd=parent)
            try:
                entries, truncated = self._walk(fd, path, recursive, limit)
                return {"path": path, "entries": entries, "truncated": truncated}
            finally:
                os.close(fd)

    def read(
        self,
        *,
        path: str,
        offset: int = 0,
        limit: int = MAX_READ_BYTES,
        encoding: str = "utf-8",
    ) -> dict:
        _bound(offset, MAX_FILE_BYTES, "offset", 0)
        _bound(limit, MAX_READ_BYTES, "limit")
        if encoding not in ("utf-8", "base64"):
            _fail("invalid_argument", "encoding must be utf-8 or base64")
        with self._locked() as root, self._parent(root, path) as (parent, name):
            data, _ = _read_file(parent, name)
        if offset > len(data):
            _fail("invalid_argument", "offset is beyond the end of the file")
        selected = data[offset : offset + limit]
        if encoding == "utf-8":
            try:
                content = selected.decode("utf-8")
                if "\x00" in content:
                    raise UnicodeError
            except UnicodeError:
                _fail(
                    "binary_content",
                    "Requested range is not UTF-8 text; use base64 or align the byte range",
                )
        else:
            content = base64.b64encode(selected).decode("ascii")
        return {
            "path": path,
            "revision": _revision(data),
            "size": len(data),
            "offset": offset,
            "next_offset": offset + len(selected),
            "truncated": offset + len(selected) < len(data),
            "encoding": encoding,
            "content": content,
            "mime_type": mimetypes.guess_type(path)[0] or "application/octet-stream",
        }

    def search(
        self, *, query: str, path: str = ".", regex: bool = False, limit: int = 100
    ) -> dict:
        if not isinstance(query, str) or not query or len(query.encode()) > 1024:
            _fail("invalid_argument", "query must contain 1 to 1024 bytes")
        if type(regex) is not bool:
            _fail("invalid_argument", "regex must be a boolean")
        _bound(limit, MAX_ENTRIES, "limit")
        if regex:
            try:
                re.compile(query)
            except re.error as exc:
                _fail("invalid_regex", str(exc))
        files, revisions, scanned = [], {}, 0
        with self._locked() as root, self._parent(root, path) as (parent, name):
            info = os.stat(name, dir_fd=parent, follow_symlinks=False)
            if stat_mode.S_ISDIR(info.st_mode):
                fd = os.open(name, _DIRECTORY_FLAGS, dir_fd=parent)
                try:
                    rows, truncated = self._walk(fd, path, True, MAX_ENTRIES)
                finally:
                    os.close(fd)
            elif stat_mode.S_ISREG(info.st_mode):
                rows, truncated = (
                    [{"path": path, "kind": "file", "size": info.st_size}],
                    False,
                )
            else:
                _fail("unsupported_type", "Search requires a regular file or directory")
            skipped = []
            for row in rows:
                if row["kind"] != "file":
                    continue
                if scanned + row["size"] > MAX_SEARCH_BYTES:
                    truncated = True
                    break
                with self._parent(root, row["path"]) as (file_parent, file_name):
                    data, _ = _read_file(file_parent, file_name)
                scanned += len(data)
                try:
                    value = data.decode("utf-8")
                    if "\x00" in value:
                        raise UnicodeError
                except UnicodeError:
                    skipped.append(row["path"])
                    continue
                files.append((row["path"], value))
                revisions[row["path"]] = _revision(data)
        if regex:
            try:
                output = subprocess.run(
                    [sys.executable, "-I", "-c", _REGEX_SEARCH],
                    input=json.dumps(
                        {"query": query, "files": files, "limit": limit + 1}
                    ),
                    capture_output=True,
                    text=True,
                    env={},
                    timeout=1,
                    check=True,
                )
                matches = json.loads(output.stdout)
            except subprocess.TimeoutExpired:
                _fail(
                    "search_timeout",
                    "Regular expression exceeded the one-second search deadline",
                )
            except subprocess.CalledProcessError:
                _fail("search_failed", "Regular expression search failed")
        else:
            matches = []
            for file_path, value in files:
                for number, line in enumerate(value.splitlines(), 1):
                    start = 0
                    while (column := line.find(query, start)) >= 0:
                        matches.append(
                            {
                                "path": file_path,
                                "line": number,
                                "column": column + 1,
                                "text": line[:1024],
                            }
                        )
                        start = column + len(query)
                        if len(matches) > limit:
                            break
                    if len(matches) > limit:
                        break
                if len(matches) > limit:
                    break
        for match in matches:
            match["revision"] = revisions[match["path"]]
        return {
            "matches": matches[:limit],
            "truncated": truncated or len(matches) > limit,
            "scanned_bytes": scanned,
            "skipped_binary": skipped,
        }

    def _replace(
        self,
        parent: int,
        name: str,
        path: str,
        data: bytes,
        expected: str | None,
        commit_check=None,
    ) -> dict:
        current = self._check(parent, name, path, expected)
        if current and current["kind"] != "file":
            _fail("unsupported_type", "Only regular files may be replaced")
        if len(data) > MAX_FILE_BYTES:
            _fail("size_limit", f"Content exceeds {MAX_FILE_BYTES} bytes")
        temporary = ".sdk-write-" + uuid.uuid4().hex
        fd = os.open(
            temporary,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW,
            0o600,
            dir_fd=parent,
        )
        try:
            with os.fdopen(fd, "wb") as handle:
                if current:
                    mode = (
                        os.stat(name, dir_fd=parent, follow_symlinks=False).st_mode
                        & 0o777
                    )
                    os.fchmod(handle.fileno(), mode)
                handle.write(data)
                handle.flush()
                os.fsync(handle.fileno())
            self._check(parent, name, path, expected)
            if commit_check is not None:
                commit_check()
            if expected is None:
                # link is an atomic no-replace publication, including against a
                # noncooperating creator between the check and publication.
                os.link(
                    temporary,
                    name,
                    src_dir_fd=parent,
                    dst_dir_fd=parent,
                    follow_symlinks=False,
                )
            else:
                os.replace(temporary, name, src_dir_fd=parent, dst_dir_fd=parent)
            os.fsync(parent)
        finally:
            try:
                os.unlink(temporary, dir_fd=parent)
            except FileNotFoundError:
                pass
        return self._metadata(parent, name, path)

    def write(
        self,
        *,
        path: str,
        content: str,
        expected_revision: str | None,
        encoding: str = "utf-8",
        _commit_check=None,
    ) -> dict:
        if not _parts(path):
            _fail("invalid_path", "The workspace root cannot be replaced")
        if not isinstance(content, str):
            _fail("invalid_argument", "content must be a string")
        if len(content) > MAX_FILE_BYTES * 4 // 3 + 4:
            _fail("size_limit", "Encoded content exceeds the file limit")
        if encoding == "utf-8":
            data = content.encode("utf-8")
        elif encoding == "base64":
            try:
                data = base64.b64decode(content, validate=True)
            except (binascii.Error, ValueError):
                _fail("invalid_argument", "content is not canonical base64")
        else:
            _fail("invalid_argument", "encoding must be utf-8 or base64")
        with self._locked() as root, self._parent(root, path) as (parent, name):
            return self._replace(
                parent, name, path, data, expected_revision, _commit_check
            )

    def apply_patch(self, *, path: str, patch: str, expected_revision: str) -> dict:
        _expected(expected_revision)
        if expected_revision is None:
            _fail("invalid_revision", "A patch requires the existing file revision")
        if not isinstance(patch, str) or len(patch.encode()) > MAX_FILE_BYTES:
            _fail("size_limit", "Patch must be bounded text")
        with self._locked() as root, self._parent(root, path) as (parent, name):
            self._check(parent, name, path, expected_revision)
            data, _ = _read_file(parent, name)
            try:
                original = data.decode("utf-8")
            except UnicodeError:
                _fail("binary_content", "Unified patches require UTF-8 text")
            result = _apply_unified_patch(original, patch, path)
            return self._replace(parent, name, path, result.encode(), expected_revision)

    def mkdir(self, *, path: str) -> dict:
        if not _parts(path):
            _fail("invalid_path", "The workspace root already exists")
        with self._locked() as root, self._parent(root, path) as (parent, name):
            os.mkdir(name, mode=0o700, dir_fd=parent)
            os.fsync(parent)
            return self._metadata(parent, name, path)

    def move(
        self,
        *,
        source: str,
        destination: str,
        expected_revision: str,
        expected_destination_revision: str | None,
    ) -> dict:
        if not _parts(source) or not _parts(destination) or source == destination:
            _fail("invalid_path", "Move requires distinct non-root paths")
        if expected_revision is None:
            _fail("invalid_revision", "Source requires its existing revision")
        with (
            self._locked() as root,
            self._parent(root, source) as (src, src_name),
            self._parent(root, destination) as (dst, dst_name),
        ):
            current = self._check(src, src_name, source, expected_revision)
            target = self._check(
                dst, dst_name, destination, expected_destination_revision
            )
            if target and target["kind"] != current["kind"]:
                _fail("conflict", "Move cannot replace an entry of another type")
            if expected_destination_revision is None:
                _rename_absent(src, src_name, dst, dst_name)
            else:
                os.rename(src_name, dst_name, src_dir_fd=src, dst_dir_fd=dst)
            os.fsync(src)
            os.fsync(dst)
            return self._metadata(dst, dst_name, destination)

    def delete(self, *, path: str, expected_revision: str, _commit_check=None) -> dict:
        if not _parts(path):
            _fail("invalid_path", "The workspace root cannot be deleted")
        if expected_revision is None:
            _fail("invalid_revision", "Delete requires the existing revision")
        with self._locked() as root, self._parent(root, path) as (parent, name):
            current = self._check(parent, name, path, expected_revision)
            if current["kind"] == "directory":
                fd = os.open(name, _DIRECTORY_FLAGS, dir_fd=parent)
                try:
                    with os.scandir(fd) as entries:
                        if next(entries, None) is not None:
                            _fail("not_empty", "Only empty directories may be deleted")
                finally:
                    os.close(fd)
                if _commit_check is not None:
                    _commit_check()
                os.rmdir(name, dir_fd=parent)
            else:
                if _commit_check is not None:
                    _commit_check()
                os.unlink(name, dir_fd=parent)
            os.fsync(parent)
            return {
                "path": path,
                "deleted": True,
                "previous_revision": expected_revision,
            }


def _apply_unified_patch(original: str, patch: str, path: str) -> str:
    lines = patch.splitlines(keepends=True)
    if (
        len(lines) < 3
        or not lines[0].startswith("--- ")
        or not lines[1].startswith("+++ ")
    ):
        _fail(
            "invalid_patch", "Patch must begin with '--- path' and '+++ path' headers"
        )
    for header, prefix in zip(lines[:2], ("a/", "b/")):
        label = header[4:].rstrip("\r\n").split("\t", 1)[0]
        if label not in (path, prefix + path):
            _fail(
                "invalid_patch", "Patch headers must name the requested existing file"
            )
    source, output, position, index = original.splitlines(keepends=True), [], 0, 2
    hunks = 0
    while index < len(lines):
        header = re.fullmatch(
            r"@@ -(\d+)(?:,(\d+))? \+(\d+)(?:,(\d+))? @@[^\r\n]*\r?\n?", lines[index]
        )
        if not header:
            _fail(
                "invalid_patch",
                f"Patch line {index + 1}: expected '@@ -old,count +new,count @@'",
            )
        old_start, old_count, new_start, new_count = (
            int(header[1]),
            int(header[2] or 1),
            int(header[3]),
            int(header[4] or 1),
        )
        old_index = old_start - 1 if old_count else old_start
        new_index = new_start - 1 if new_count else new_start
        if (
            old_index < position
            or old_index > len(source)
            or new_index != len(output) + old_index - position
        ):
            _fail(
                "patch_conflict",
                f"Patch line {index + 1}: hunk ranges do not match source/output positions",
            )
        output.extend(source[position:old_index])
        position, index, removed, added = old_index, index + 1, 0, 0
        while index < len(lines) and not lines[index].startswith("@@ "):
            line = lines[index]
            if not line or line[0] not in " +-":
                _fail(
                    "invalid_patch",
                    f"Patch line {index + 1}: expected context, addition, or removal",
                )
            marker, value = line[0], line[1:]
            index += 1
            if (
                index < len(lines)
                and lines[index].rstrip("\r\n") == "\\ No newline at end of file"
            ):
                value = value.removesuffix("\n")
                index += 1
            if marker in " -":
                if position >= len(source) or source[position] != value:
                    _fail(
                        "patch_conflict",
                        f"Source line {position + 1} does not match patch context; read again",
                    )
                position += 1
                removed += 1
            if marker in " +":
                output.append(value)
                added += 1
        if (removed, added) != (old_count, new_count):
            _fail(
                "invalid_patch",
                f"Hunk counts require {old_count} old/{new_count} new lines; got {removed}/{added}",
            )
        hunks += 1
    if not hunks:
        _fail("invalid_patch", "Patch needs at least one hunk")
    output.extend(source[position:])
    return "".join(output)
