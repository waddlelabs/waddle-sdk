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
