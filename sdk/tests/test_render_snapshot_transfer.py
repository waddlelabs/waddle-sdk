"""Current native models cross private render pipes without large copied buffers."""

import multiprocessing
import os

import numpy as np
import pytest
from test_render_revisions import model_for, renderer_config, snapshot


@pytest.mark.parametrize("sealed", [False, True])
def test_current_model_handoff_matches_buffer_render_and_releases_descriptors(
    monkeypatch, sealed
):
    mj = pytest.importorskip("mujoco")
    from waddle_sdk.simulators import render_snapshot
    from waddle_sdk.simulators.prepared_model import PreparedModel
    from waddle_sdk.simulators.render_worker import SnapshotRenderer

    if sealed and not hasattr(os, "memfd_create"):
        pytest.skip("sealed descriptor transport is Linux-only")
    if not sealed:
        monkeypatch.setattr(render_snapshot, "seal", lambda *args, **kwargs: None)
    model = model_for(mj)
    # The current physics model can differ from its authored preparation.
    model.geom_rgba[-1] = [0, 1, 0, 1]
    frame = snapshot(mj, model, 7)
    expected = SnapshotRenderer(renderer_config(True))
    actual = SnapshotRenderer(renderer_config(True))
    sender, receiver = multiprocessing.Pipe()
    received = None
    try:
        expected.restore(frame)
        frame["model"] = payload = render_snapshot.capture_model(model)
        assert isinstance(payload, PreparedModel) == sealed
        render_snapshot.send(sender, "view", frame, os.getpid())
        if sealed:
            with pytest.raises(ValueError, match="released"):
                payload.descriptor()
        camera, wire = receiver.recv()
        assert camera == "view"
        received = render_snapshot.receive(receiver, wire)
        actual.restore(received)
        assert actual.model is not model
        assert actual.data.time == 7.25
        for a, b in zip(actual.capture(camera), expected.capture(camera), strict=True):
            np.testing.assert_array_equal(a, b)
    finally:
        render_snapshot.release(received)
        actual.close()
        expected.close()
        sender.close()
        receiver.close()
    if sealed:
        with pytest.raises(ValueError, match="released"):
            received["model"].descriptor()


@pytest.mark.skipif(not hasattr(os, "memfd_create"), reason="Linux sealed descriptors")
@pytest.mark.parametrize("close_fails", [False, True])
def test_failed_render_send_releases_sealed_model(close_fails):
    mj = pytest.importorskip("mujoco")
    from waddle_sdk.simulators import render_snapshot

    class BrokenConnection:
        closed = False

        def send(self, value):
            raise BrokenPipeError("original send failure")

        def close(self):
            self.closed = True
            if close_fails:
                raise OSError("secondary close failure")

    model = model_for(mj)
    frame = snapshot(mj, model, 1)
    frame["model"] = render_snapshot.capture_model(model)
    assert not isinstance(frame["model"], bytes)
    connection = BrokenConnection()
    with pytest.raises(BrokenPipeError, match="original send failure"):
        render_snapshot.send(connection, True, frame, os.getpid())
    assert connection.closed
    with pytest.raises(ValueError, match="released"):
        frame["model"].descriptor()


def test_unchanged_generation_has_no_model_transfer():
    from waddle_sdk.simulators import render_snapshot

    sender, receiver = multiprocessing.Pipe()
    try:
        frame = {"generation": 4, "model": None}
        render_snapshot.send(sender, True, frame, os.getpid())
        ok, wire = receiver.recv()
        assert ok
        assert render_snapshot.receive(receiver, wire) == frame
        render_snapshot.release(frame)
    finally:
        sender.close()
        receiver.close()


@pytest.mark.skipif(not hasattr(os, "memfd_create"), reason="Linux sealed descriptors")
def test_failed_fresh_acquisition_releases_retained_model():
    mj = pytest.importorskip("mujoco")
    from types import SimpleNamespace

    from waddle_sdk.simulators import render_snapshot
    from waddle_sdk.simulators.adapters import World

    model = model_for(mj)
    frame = snapshot(mj, model, 2)
    frame["model"] = render_snapshot.capture_model(model)
    reads = 0

    def call(*_):
        nonlocal reads
        reads += 1
        if reads == 1:
            return frame
        raise RuntimeError("original state acquisition failure")

    renderer = SimpleNamespace(capture=lambda camera, acquire, cancelled: acquire(None))
    world = SimpleNamespace(_renderer=renderer, call=call)
    with pytest.raises(RuntimeError, match="original state acquisition failure"):
        World.capture(world, "view")
    with pytest.raises(ValueError, match="released"):
        frame["model"].descriptor()


@pytest.mark.skipif(not hasattr(os, "memfd_create"), reason="Linux sealed descriptors")
def test_failed_descriptor_transfer_closes_partial_response(monkeypatch):
    mj = pytest.importorskip("mujoco")
    from multiprocessing import reduction

    from waddle_sdk.simulators import render_snapshot

    def failed(*_):
        raise OSError("descriptor transfer failed")

    monkeypatch.setattr(reduction, "send_handle", failed)
    frame = {"model": render_snapshot.capture_model(model_for(mj)), "generation": 1}
    sender, receiver = multiprocessing.Pipe()
    try:
        with pytest.raises(OSError, match="descriptor transfer failed"):
            render_snapshot.send(sender, True, frame, os.getpid())
        assert sender.closed
        ok, wire = receiver.recv()
        assert ok
        assert receiver.poll(0.1), (
            "receiver must see EOF, not wait for a missing handle"
        )
        with pytest.raises(EOFError):
            render_snapshot.receive(receiver, wire)
        with pytest.raises(ValueError, match="released"):
            frame["model"].descriptor()
    finally:
        sender.close()
        receiver.close()
