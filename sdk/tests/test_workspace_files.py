import base64
import difflib
import hashlib
import os
import sys
import threading
from concurrent.futures import ThreadPoolExecutor

import pytest

pytest.importorskip("fcntl")

from waddle_sdk.workspace.files import MAX_FILE_BYTES, Workspace, WorkspaceError


@pytest.fixture
def workspace(tmp_path):
    return Workspace(tmp_path)


def write(workspace, path="file.txt", content="old\n"):
    return workspace.write(path=path, content=content, expected_revision=None)


def test_revision_ranges_binary_and_image(workspace):
    initial = write(workspace, content="abcdef\n")
    assert initial["revision"] == hashlib.sha256(b"abcdef\n").hexdigest()
    view = workspace.read(path="file.txt", offset=2, limit=3)
    assert (view["content"], view["next_offset"], view["truncated"]) == ("cde", 5, True)
    assert view["revision"] == initial["revision"]
    raw = b"\x89PNG\r\n\x1a\n\x00"
    workspace.write(
        path="image.png",
        content=base64.b64encode(raw).decode(),
        expected_revision=None,
        encoding="base64",
    )
    with pytest.raises(WorkspaceError, match="base64"):
        workspace.read(path="image.png")
    binary = workspace.read(path="image.png", encoding="base64")
    assert base64.b64decode(binary["content"]) == raw
    assert binary["mime_type"] == "image/png"


def test_creates_and_replacements_require_correct_revision(workspace):
    initial = write(workspace)
    with pytest.raises(WorkspaceError) as conflict:
        write(workspace)
    assert conflict.value.code == "workspace.conflict"
    changed = workspace.write(
        path="file.txt", content="new\n", expected_revision=initial["revision"]
    )
    assert changed["revision"] != initial["revision"]
    with pytest.raises(WorkspaceError):
        workspace.write(
            path="file.txt", content="stale", expected_revision=initial["revision"]
        )
    assert workspace.read(path="file.txt")["content"] == "new\n"
    assert all(
        not path.name.startswith(".sdk-write-") for path in workspace.root.iterdir()
    )


def test_two_service_instances_cannot_both_win_compare_and_swap(workspace):
    revision = write(workspace)["revision"]
    barrier = threading.Barrier(2)

    def replace(content):
        peer = Workspace(workspace.root)
        barrier.wait()
        try:
            peer.write(path="file.txt", content=content, expected_revision=revision)
            return "written"
        except WorkspaceError as exc:
            return exc.code

    with ThreadPoolExecutor(2) as pool:
        outcomes = list(pool.map(replace, ["first", "second"]))
    assert sorted(outcomes) == ["workspace.conflict", "written"]
    assert workspace.read(path="file.txt")["content"] in ("first", "second")


def test_competing_create_is_not_overwritten_even_at_publication(
    workspace, monkeypatch
):
    original_link = os.link

    def insert_competitor(source, destination, **kwargs):
        (workspace.root / "file.txt").write_text("external")
        return original_link(source, destination, **kwargs)

    monkeypatch.setattr(os, "link", insert_competitor)
    with pytest.raises(WorkspaceError):
        write(workspace)
    assert (workspace.root / "file.txt").read_text() == "external"


@pytest.mark.parametrize(
    "path", ["/etc/passwd", "../outside", "a/../b", "a//b", "./a", "", "a\\b", "a\x00b"]
)
def test_paths_are_strictly_relative(workspace, path):
    with pytest.raises(WorkspaceError) as invalid:
        workspace.stat(path=path)
    assert invalid.value.code == "workspace.invalid_path"


def test_symlinks_special_files_and_swapped_parent_never_escape(workspace, tmp_path):
    outside = tmp_path.parent / (tmp_path.name + "-outside")
    outside.mkdir()
    (outside / "secret").write_text("outside")
    (workspace.root / "escape").symlink_to(outside, target_is_directory=True)
    (workspace.root / "alias").symlink_to(outside / "secret")
    for operation in (
        lambda: workspace.read(path="escape/secret"),
        lambda: workspace.read(path="alias"),
        lambda: workspace.write(path="escape/new", content="x", expected_revision=None),
        lambda: workspace.delete(path="alias", expected_revision="0" * 64),
    ):
        with pytest.raises(WorkspaceError):
            operation()
    os.mkfifo(workspace.root / "pipe")
    with pytest.raises(WorkspaceError):
        workspace.read(path="pipe")
    assert not (outside / "new").exists()
    assert (outside / "secret").read_text() == "outside"
    listing = workspace.list(recursive=True)
    assert {row["kind"] for row in listing["entries"]} == {"symlink", "special"}


def test_patch_matches_standard_unified_diff_and_is_atomic(workspace):
    before = "one\ntwo\nthree\nfour\nfive\nsix\nseven\neight\n"
    after = "zero\none\nTWO\nthree\nfour\nfive\nsix\nseven\nEIGHT\n"
    initial = write(workspace, content=before)
    patch = "".join(
        difflib.unified_diff(
            before.splitlines(True),
            after.splitlines(True),
            fromfile="a/file.txt",
            tofile="b/file.txt",
            n=1,
        )
    )
    updated = workspace.apply_patch(
        path="file.txt", patch=patch, expected_revision=initial["revision"]
    )
    assert workspace.read(path="file.txt")["content"] == after
    assert updated["revision"] == hashlib.sha256(after.encode()).hexdigest()
    with pytest.raises(WorkspaceError):
        workspace.apply_patch(
            path="file.txt", patch=patch, expected_revision=initial["revision"]
        )


def test_patch_insertion_deletion_and_missing_final_newline(workspace):
    current = write(workspace, content="one\ntwo")
    patch = "--- file.txt\n+++ file.txt\n@@ -1,2 +1,2 @@\n one\n-two\n\\ No newline at end of file\n+THREE\n\\ No newline at end of file\n"
    workspace.apply_patch(
        path="file.txt", patch=patch, expected_revision=current["revision"]
    )
    assert workspace.read(path="file.txt")["content"] == "one\nTHREE"
    empty = write(workspace, path="empty", content="")
    inserted = workspace.apply_patch(
        path="empty",
        patch="--- empty\n+++ empty\n@@ -0,0 +1 @@\n+hello\n",
        expected_revision=empty["revision"],
    )
    workspace.apply_patch(
        path="empty",
        patch="--- empty\n+++ empty\n@@ -1 +0,0 @@\n-hello\n",
        expected_revision=inserted["revision"],
    )
    assert workspace.read(path="empty")["content"] == ""


@pytest.mark.parametrize(
    "patch",
    [
        "--- wrong\n+++ file.txt\n@@ -1 +1 @@\n-old\n+new\n",
        "--- file.txt\n+++ file.txt\n@@ -1,2 +1 @@\n-old\n+new\n",
        "--- file.txt\n+++ file.txt\n@@ -1 +1 @@\n-other\n+new\n",
        "--- file.txt\n+++ file.txt\n@@ -1 +9 @@\n-old\n+new\n",
        "*** Begin Patch\n",
    ],
)
def test_malformed_or_conflicting_patch_leaves_file_unchanged(workspace, patch):
    revision = write(workspace)["revision"]
    with pytest.raises(WorkspaceError):
        workspace.apply_patch(path="file.txt", patch=patch, expected_revision=revision)
    assert workspace.read(path="file.txt")["content"] == "old\n"


def test_move_checks_both_paths_and_delete_requires_empty_directory(workspace):
    source = write(workspace)
    target = write(workspace, path="target", content="target")
    with pytest.raises(WorkspaceError):
        workspace.move(
            source="file.txt",
            destination="target",
            expected_revision=source["revision"],
            expected_destination_revision=None,
        )
    moved = workspace.move(
        source="file.txt",
        destination="target",
        expected_revision=source["revision"],
        expected_destination_revision=target["revision"],
    )
    assert moved["revision"] == source["revision"]
    with pytest.raises(WorkspaceError):
        workspace.stat(path="file.txt")
    directory = workspace.mkdir(path="dir")
    write(workspace, path="dir/child")
    with pytest.raises(WorkspaceError) as stale:
        workspace.delete(path="dir", expected_revision=directory["revision"])
    assert stale.value.code == "workspace.conflict"
    with pytest.raises(WorkspaceError) as nonempty:
        workspace.delete(
            path="dir", expected_revision=workspace.stat(path="dir")["revision"]
        )
    assert nonempty.value.code == "workspace.not_empty"
    workspace.delete(
        path="dir/child", expected_revision=workspace.stat(path="dir/child")["revision"]
    )
    workspace.delete(
        path="dir", expected_revision=workspace.stat(path="dir")["revision"]
    )
    assert not (workspace.root / "dir").exists()


def test_listing_and_search_have_limits_and_locations(workspace):
    workspace.mkdir(path="nested")
    write(workspace, path="nested/words", content="alpha beta alpha\nBETA\n")
    workspace.write(
        path="bytes", content="AA==", encoding="base64", expected_revision=None
    )
    assert workspace.list(limit=1)["truncated"]
    found = workspace.search(query="alpha", path="nested", limit=1)
    assert found["truncated"]
    assert found["matches"][0]["line"] == 1
    assert found["matches"][0]["column"] == 1
    assert (
        found["matches"][0]["revision"]
        == workspace.stat(path="nested/words")["revision"]
    )
    regexp = workspace.search(query="(?i)beta", regex=True)
    assert len(regexp["matches"]) == 2
    assert regexp["skipped_binary"] == ["bytes"]
    with pytest.raises(WorkspaceError) as syntax:
        workspace.search(query="[", regex=True)
    assert syntax.value.code == "workspace.invalid_regex"


def test_regex_deadline_rejects_catastrophic_backtracking(workspace):
    write(workspace, content="a" * 10000 + "!")
    with pytest.raises(WorkspaceError) as timeout:
        workspace.search(query="(a+)+$", regex=True)
    assert timeout.value.code == "workspace.search_timeout"


def test_file_and_range_limits_are_enforced(workspace):
    with pytest.raises(WorkspaceError):
        write(workspace, content="x" * (MAX_FILE_BYTES + 1))
    write(workspace)
    with pytest.raises(WorkspaceError):
        workspace.read(path="file.txt", limit=1024 * 1024)
    with pytest.raises(WorkspaceError):
        workspace.read(path="file.txt", offset=100)
    with pytest.raises(WorkspaceError):
        workspace.read(path="file.txt", limit=True)


@pytest.mark.skipif(
    sys.platform != "linux", reason="Atomic absent rename uses Linux renameat2"
)
def test_move_absent_precondition_survives_competing_destination(
    workspace, monkeypatch
):
    import waddle_sdk.workspace.files as module

    source = write(workspace)
    original = module._rename_absent

    def competitor(src, src_name, dst, dst_name):
        (workspace.root / "target").write_text("external")
        return original(src, src_name, dst, dst_name)

    monkeypatch.setattr(module, "_rename_absent", competitor)
    with pytest.raises(WorkspaceError) as conflict:
        workspace.move(
            source="file.txt",
            destination="target",
            expected_revision=source["revision"],
            expected_destination_revision=None,
        )
    assert conflict.value.code == "workspace.conflict"
    assert (workspace.root / "target").read_text() == "external"
    assert workspace.read(path="file.txt")["content"] == "old\n"


def test_recursive_listing_stops_at_explicit_depth_bound(workspace, monkeypatch):
    monkeypatch.setattr("waddle_sdk.workspace.files.MAX_DEPTH", 1)
    (workspace.root / "a" / "b" / "c").mkdir(parents=True)
    result = workspace.list(recursive=True)
    assert result["truncated"]
    assert [entry["path"] for entry in result["entries"]] == ["a", "a/b"]


def test_task_directory_descriptor_is_independent_of_content_lock(workspace):
    # Reading a task's cwd must not wait behind another process's content edit.
    with workspace._locked():
        descriptor = workspace.directory_fd(path=".")
        try:
            assert os.fstat(descriptor).st_ino == workspace.root.stat().st_ino
        finally:
            os.close(descriptor)


def test_operator_commit_fence_after_blocked_fsync_preserves_old_revision(
    workspace, monkeypatch
):
    initial = write(workspace)
    entered, resume, cancelled = threading.Event(), threading.Event(), threading.Event()
    original = os.fsync

    def blocked(fd):
        entered.set()
        assert resume.wait(2)
        return original(fd)

    def check():
        if cancelled.is_set():
            raise ValueError("operator fenced")

    monkeypatch.setattr(os, "fsync", blocked)
    with ThreadPoolExecutor(max_workers=1) as pool:
        future = pool.submit(
            workspace.write,
            path="file.txt",
            content="late replacement",
            expected_revision=initial["revision"],
            _commit_check=check,
        )
        assert entered.wait(1)
        cancelled.set()
        resume.set()
        with pytest.raises(ValueError, match="fenced"):
            future.result()
    assert workspace.read(path="file.txt")["revision"] == initial["revision"]
    assert [row["path"] for row in workspace.list()["entries"]] == ["file.txt"]
