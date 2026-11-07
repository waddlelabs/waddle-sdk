"""Move current model snapshots across existing private owner/render pipes."""

import pickle

from .prepared_model import PreparedModel, load_sealed, seal
from .prepared_model import receive as receive_model

_IDENTITY = ("waddle.runtime-render-model/v1",)


def capture_model(model):
    """Seal the current physics model; unsupported storage keeps copied buffers."""
    import mujoco as mj

    if mj.mj_sizeModel(model) <= 256 * 1024 * 1024:
        prepared = seal(model, identity=_IDENTITY)
        if prepared is not None:
            return prepared
    return pickle.dumps(model)


def load_model(payload):
    if not isinstance(payload, PreparedModel):
        return pickle.loads(payload)
    if payload.identity != _IDENTITY:
        raise ValueError("simulation.render_identity: not a current-model snapshot")
    model = load_sealed(payload)
    if model is None:
        raise ValueError("simulation.render_runtime: incompatible model snapshot")
    return model


def release(snapshot):
    if snapshot is not None and isinstance(snapshot.get("model"), PreparedModel):
        snapshot["model"].close()


def send(connection, prefix, snapshot, destination_pid):
    """Transfer a model handle, consuming this sender's short-lived ownership."""
    payload = snapshot.get("model")
    if not isinstance(payload, PreparedModel):
        connection.send((prefix, snapshot))
        return
    from multiprocessing.reduction import send_handle

    try:
        wire = dict(snapshot, model={"prepared_model": payload.metadata()})
        connection.send((prefix, wire))
        send_handle(connection, payload.descriptor(), destination_pid)
    except BaseException:
        # Once metadata is sent, another response cannot repair its missing FD.
        # Closing the private pipe releases the receiver and preserves the fault.
        try:
            connection.close()
        except OSError:
            pass
        raise
    finally:
        payload.close()


def receive(connection, snapshot):
    """Hydrate only a private render response; public messages never reach here."""
    if not isinstance(snapshot, dict):
        return snapshot
    payload = snapshot.get("model")
    if isinstance(payload, dict) and set(payload) == {"prepared_model"}:
        snapshot["model"] = receive_model(connection, payload["prepared_model"])
    return snapshot
