"""A speculative native model is private, bounded and never live state."""

import dataclasses
import hashlib
import os
import threading

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(
    not hasattr(os, "memfd_create"), reason="sealed preparation requires Linux"
)


@pytest.fixture
def payload():
    mj = pytest.importorskip("mujoco")
    from waddle_sdk.simulators.prepared_model import seal

    xml = b'<mujoco><worldbody><body><freejoint/><geom size=".1"/></body></worldbody></mujoco>'
    model = mj.MjModel.from_xml_string(xml)
    identity = ("world.xml", (("world.xml", hashlib.sha256(xml).hexdigest()),))
    value = seal(model, identity=identity, xml=xml)
    assert value is not None
    try:
        yield value
    finally:
        value.close()


def test_loading_is_nonblocking_and_retains_no_mutable_data(payload, monkeypatch):
    import mujoco as mj
    from waddle_sdk.simulators import prepared_model
    from waddle_sdk.simulators.resident_preparation import ResidentPreparation

    entered, release = threading.Event(), threading.Event()
    original = prepared_model.load_sealed

    def load(value):
        entered.set()
        assert release.wait(5)
        return original(value)

    def no_data(*args, **kwargs):
        pytest.fail("idle preparation allocated mutable simulation state")

    monkeypatch.setattr(prepared_model, "load_sealed", load)
    monkeypatch.setattr(mj, "MjData", no_data)
    cache = ResidentPreparation()
    try:
        token = cache.offer(payload)
        assert entered.wait(2)
        assert cache.status(token) is None
        assert cache.take(payload) is None
        assert cache.offer(payload) is None  # Only one native load can run.
        release.set()
        cache.join()
        assert cache.status(token) is True
        model = cache.take(payload)
        np.testing.assert_array_equal(model.geom_size[0], [0.1, 0, 0])
        assert cache.take(payload) is None
    finally:
        release.set()
        cache.close()


@pytest.mark.parametrize(
    "mismatch", ["descriptor", "runtime", "identity", "xml", "contract", "bodies"]
)
def test_only_exact_sealed_source_can_consume_once(payload, mismatch):
    from waddle_sdk.simulators.prepared_model import load_sealed, seal
    from waddle_sdk.simulators.resident_preparation import ResidentPreparation

    cache = ResidentPreparation()
    duplicate = payload.duplicate()
    if mismatch == "descriptor":
        changed = seal(load_sealed(payload), identity=payload.identity, xml=payload.xml)
    else:
        value = {
            "runtime": ("other", "Linux", "x86_64"),
            "identity": ("other.xml", ()),
            "xml": b"changed XML",
            "contract": b"changed contract",
            "bodies": b"changed comparison",
        }[mismatch]
        changed = dataclasses.replace(
            payload, fd=os.dup(payload.descriptor()), **{mismatch: value}
        )
    try:
        token = cache.offer(payload)
        cache.join()
        assert cache.status(token) is True
        assert cache.take(changed) is None
        model = cache.take(duplicate)
        assert model is not None
        assert cache.take(duplicate) is None
    finally:
        changed.close()
        duplicate.close()
        cache.close()


def test_budget_failure_and_close_leave_ordinary_loading_available(
    payload, monkeypatch
):
    from waddle_sdk.simulators import prepared_model
    from waddle_sdk.simulators.resident_preparation import ResidentPreparation

    too_small = ResidentPreparation(max_bytes=2 * payload.size - 1)
    try:
        assert too_small.offer(payload) is None
        assert prepared_model.load_sealed(payload) is not None
    finally:
        too_small.close()
    cache = ResidentPreparation()
    original = prepared_model.load_sealed

    def failed(_):
        raise ValueError("optional native preparation failed")

    try:
        monkeypatch.setattr(prepared_model, "load_sealed", failed)
        token = cache.offer(payload)
        cache.join()
        assert cache.status(token) is False
        assert cache.take(payload) is None
        monkeypatch.setattr(prepared_model, "load_sealed", original)
        token = cache.offer(payload)
        cache.join()
        assert cache.status(token) is True
        cache.close()
        assert cache.take(payload) is None
        assert cache.offer(payload) is None
        assert original(payload) is not None
    finally:
        cache.close()


@pytest.mark.parametrize("fails", [False, True])
def test_worker_serves_control_while_optional_native_load_is_blocked(
    payload, monkeypatch, fails
):
    import multiprocessing
    from multiprocessing.reduction import send_handle
    from types import SimpleNamespace

    from waddle_sdk.simulators import prepared_model, worker
    from waddle_sdk.simulators.resident_preparation import ResidentPreparation

    entered, release, finished = threading.Event(), threading.Event(), threading.Event()
    holds = []
    original_load = prepared_model.load_sealed
    original_import = worker.importlib.import_module
    original_run = ResidentPreparation._load

    def load(value):
        entered.set()
        assert release.wait(5)
        if fails:
            raise ValueError("optional failure")
        return original_load(value)

    def run(cache):
        try:
            original_run(cache)
        finally:
            finished.set()

    class Engine:
        supports_resident_preparation = True

        def __init__(self, config, scratch):
            self.value = 0

        def render_snapshot(self, *args):
            pass

        def write(self, value):
            self.value = value

        def read(self):
            return self.value

        def hold(self):
            holds.append(True)

        def close(self):
            pass

    def imported(name):
        return (
            SimpleNamespace(Engine=Engine)
            if name.endswith(".resident_test")
            else original_import(name)
        )

    monkeypatch.setattr(worker.importlib, "import_module", imported)
    monkeypatch.setattr(prepared_model, "load_sealed", load)
    monkeypatch.setattr(ResidentPreparation, "_load", run)
    parent, child = multiprocessing.Pipe()
    thread = threading.Thread(target=worker.serve, args=(child,))
    thread.start()

    def response():
        assert parent.poll(3)
        ok, value = parent.recv()
        assert ok, value
        return value

    def call(operation, *args):
        parent.send((operation, args))
        return response()

    try:
        parent.send({"backend": "resident_test", "timestep": 0.01, "cameras": {}})
        assert response()["scene_preparation"] is True
        parent.send(("evaluation_prepare_scene", ({}, payload.metadata())))
        send_handle(parent, payload.descriptor(), os.getpid())
        token = response()
        assert entered.wait(2)
        assert call("evaluation_preparation_status", token) is None
        call("write", 17)
        assert call("read") == 17
        assert not holds
        release.set()
        assert finished.wait(2)
        assert call("evaluation_preparation_status", token) is (not fails)
        assert call("read") == 17
        assert not holds
    finally:
        release.set()
        call("close")
        thread.join(3)
        assert not thread.is_alive()
        parent.close()


def test_administration_preparation_does_not_block_snapshots_or_change_episode():
    from concurrent.futures import ThreadPoolExecutor

    from waddle_sdk.simulation import SimulationAdministration, WorldConfig

    entered, release = threading.Event(), threading.Event()

    class Backend:
        def evaluation_snapshot(self):
            return {"value": 17}

        def evaluation_reset(self, **kwargs):
            pytest.fail("preparation reset the world")

        def evaluation_prepare_scene(self, config):
            entered.set()
            assert release.wait(5)
            return True

    admin = SimulationAdministration()
    owner = object()
    admin._bind(
        {"world": Backend()},
        lifecycle_lock=threading.RLock(),
        dispatch_lock=threading.RLock(),
        owner=owner,
        fully_simulated=True,
    )
    before = admin.snapshot()
    with ThreadPoolExecutor(max_workers=2) as pool:
        task = pool.submit(admin.prepare_scene, {"world": WorldConfig("world", {})})
        try:
            assert entered.wait(2)
            during = pool.submit(admin.snapshot).result(timeout=2)
            assert during == before
        finally:
            release.set()
        assert task.result(timeout=2) is True
    assert admin.snapshot() == before
    admin._unbind(owner)
