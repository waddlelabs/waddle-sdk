"""Failure evidence cannot steal control authority or block on slow storage."""

import builtins
import json
import threading
import time

import pytest
from waddle_sdk import diagnostics
from waddle_sdk.runtime import RuntimeFault
from waddle_sdk_relay.errors import RelayError, error_payload


@pytest.fixture(autouse=True)
def isolated(monkeypatch):
    monkeypatch.setattr(diagnostics, "_store", None)
    monkeypatch.setattr(diagnostics, "_pid", None)
    for key in (
        "WADDLE_DIAGNOSTICS_DIR",
        "WADDLE_DIAGNOSTICS_CONTEXT",
        "WADDLE_DIAGNOSTICS_DISTRIBUTIONS",
    ):
        monkeypatch.delenv(key, raising=False)
    yield
    if diagnostics._store is not None:
        diagnostics._store.close()
        diagnostics._store.thread.join(2)


def test_unconfigured_faults_preserve_original_contract_without_workers():
    before = {thread.ident for thread in threading.enumerate()}
    error = RelayError("motor.vendor_fault", "original detail", motor=2)
    assert error_payload(error) == {
        "code": "motor.vendor_fault",
        "detail": "original detail",
        "context": {"motor": 2},
    }
    assert diagnostics.capture_exception(error) is None
    assert before == {thread.ident for thread in threading.enumerate()}


def test_private_cause_stack_is_retrievable_without_locals_or_credentials(tmp_path):
    store = diagnostics.configure(
        tmp_path, context={"workspace_id": "test"}, secrets=("unlabelled-secret",)
    )
    try:
        local_password = "must-never-be-inspected"
        try:
            raise OSError(122, "disk quota exceeded", "/tmp/vendor.py")
        except OSError as cause:
            raise ValueError("Bearer secret-token unlabelled-secret") from cause
    except ValueError as error:
        with diagnostics.diagnostic_scope(command_id="move-1", token="private-value"):
            fault = RuntimeFault.from_exception(error)
    store.pending.join()
    files = list(tmp_path.glob("*.json"))
    assert len(files) == 1
    raw = files[0].read_text()
    record = json.loads(raw)
    assert record["id"] == fault.context["diagnostic_id"]
    assert record["context"]["command_id"] == "move-1"
    assert record["context"]["token"] == "[REDACTED]"
    assert all(
        secret not in raw
        for secret in [
            local_password,
            "secret-token",
            "unlabelled-secret",
            "private-value",
        ]
    )
    assert (
        record["exceptions"][0]["frames"][-1]["function"]
        == "test_private_cause_stack_is_retrievable_without_locals_or_credentials"
    )
    assert record["exceptions"][1]["errno"] == 122
    assert record["exceptions"][1]["relation"] == "cause"
    assert files[0].stat().st_mode & 0o077 == 0
    assert len(raw.encode()) <= diagnostics.MAX_RECORD_BYTES


def test_slow_storage_has_a_bounded_nonblocking_producer(tmp_path, monkeypatch):
    entered, release = threading.Event(), threading.Event()

    def blocked(_self, record):
        entered.set()
        assert release.wait(5)

    monkeypatch.setattr(diagnostics.DiagnosticStore, "_write", blocked)
    store = diagnostics.configure(tmp_path)
    try:
        assert store.capture(ValueError("first"))
        assert entered.wait(2)
        started = time.monotonic()
        for _ in range(100):
            store.capture(ValueError("queued"))
        elapsed = time.monotonic() - started
        assert elapsed < 0.25
        assert store.pending.qsize() == diagnostics.MAX_PENDING
        assert store.dropped == 100 - diagnostics.MAX_PENDING
    finally:
        release.set()
        store.pending.join()


def test_storage_failure_and_saturation_do_not_replace_original_fault(
    tmp_path, monkeypatch
):
    def unavailable(*_):
        raise OSError(122, "storage quota")

    monkeypatch.setattr(diagnostics.DiagnosticStore, "_write", unavailable)
    store = diagnostics.configure(tmp_path)
    result = error_payload(RelayError("sdk.original", "keep this", motor="joint4"))
    store.pending.join()
    assert result["code"] == "sdk.original"
    assert result["detail"] == "keep this"
    assert result["context"]["motor"] == "joint4"
    assert store.status()["storage_failures"] == 1


def test_exception_groups_are_bounded_and_never_evaluate_custom_stringification():
    class Dangerous(Exception):
        def __str__(self) -> str:
            raise AssertionError("Diagnostic producer invoked arbitrary code")

    leaf = Dangerous(object())
    group_type = getattr(builtins, "ExceptionGroup", None)
    if group_type is None:
        pytest.skip("Exception groups require Python 3.11")
    grouped = group_type("group", [leaf] * 100)
    result = diagnostics.exception_evidence(grouped)
    assert len(result["exceptions"]) <= 16
    assert result["exceptions"][1]["detail"] == "See original structured fault"


def test_spool_preserves_unacknowledged_evidence_at_its_bound(tmp_path, monkeypatch):
    monkeypatch.setattr(diagnostics, "MAX_RECORDS", 2)
    store = diagnostics.configure(tmp_path)
    for number in range(3):
        store.capture(ValueError(str(number)))
        store.pending.join()
    files = list(tmp_path.glob("*.json"))
    assert len(files) == 2
    assert store.dropped == 1


def test_closed_full_worker_drains_without_leaving_a_thread_or_partial_record(
    tmp_path, monkeypatch
):
    entered, release = threading.Event(), threading.Event()
    original = diagnostics.DiagnosticStore._write

    def slow(self, record):
        entered.set()
        assert release.wait(3)
        original(self, record)

    monkeypatch.setattr(diagnostics.DiagnosticStore, "_write", slow)
    store = diagnostics.configure(tmp_path)
    store.capture(ValueError("first"))
    assert entered.wait(2)
    for _ in range(diagnostics.MAX_PENDING):
        store.capture(ValueError("retained"))
    store.close()
    release.set()
    store.thread.join(3)
    assert not store.thread.is_alive()
    assert len(list(tmp_path.glob("*.json"))) == diagnostics.MAX_PENDING + 1
    assert not list(tmp_path.glob(".pending-*"))


def test_reconfiguration_redacts_the_current_credential_and_updates_child_attribution(
    tmp_path, monkeypatch
):
    store = diagnostics.configure(tmp_path, secrets=("old-private-key",), inherit=True)
    assert (
        diagnostics.configure(
            tmp_path,
            context={"workspace_id": "new"},
            secrets=("new-private-key",),
            inherit=True,
        )
        is store
    )
    store.capture(ValueError("new-private-key"))
    store.pending.join()
    assert "new-private-key" not in next(tmp_path.glob("*.json")).read_text()
    import os

    assert json.loads(os.environ["WADDLE_DIAGNOSTICS_CONTEXT"]) == {
        "workspace_id": "new"
    }
