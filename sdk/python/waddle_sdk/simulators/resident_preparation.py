"""One unpublished native model loaded without blocking the physics loop."""

import os
import threading


def _key(payload):
    stat = os.fstat(payload.descriptor())
    return stat.st_dev, stat.st_ino, payload.metadata()


class ResidentPreparation:
    def __init__(self, *, max_bytes=256 * 1024 * 1024):
        self.max_bytes = max_bytes
        self.lock = threading.Lock()
        self.thread = self.payload = self.model = self.key = None
        self.sequence = 0
        self.loading = self.closed = False
        self.failure = None

    def _clear(self):
        self.model = self.key = None
        if self.payload is not None:
            self.payload.close()
            self.payload = None

    def offer(self, payload):
        """Start at most one load; retain a sealed descriptor until consumption."""
        with self.lock:
            if self.closed or self.loading or 2 * payload.size > self.max_bytes:
                return None
            if os.fstat(payload.descriptor()).st_size > payload.size:
                return None
            self._clear()
            self.payload = payload.duplicate()
            self.key = _key(self.payload)
            self.sequence += 1
            self.failure = None
            self.loading = True
            self.thread = threading.Thread(
                target=self._load, name="waddle-scene-preparation", daemon=True
            )
            try:
                self.thread.start()
            except BaseException:
                self.loading = False
                self._clear()
                raise
            return self.sequence

    def _load(self):
        from .prepared_model import load_sealed

        model, failure = None, None
        try:
            model = load_sealed(self.payload)
            if model is not None:
                import mujoco

                if self.payload.size + mujoco.mj_sizeModel(model) > self.max_bytes:
                    model = None
        except Exception as error:  # noqa: BLE001 -- optional preparation preserves ordinary loading
            failure = f"{type(error).__name__}: {error}"
        with self.lock:
            self.model = None if self.closed else model
            self.failure = failure
            self.loading = False

    def status(self, token):
        with self.lock:
            if self.closed or token is None or token != self.sequence:
                return False
            return None if self.loading else self.model is not None

    def take(self, payload):
        """Transfer only the exact sealed model, with no state or validity reuse."""
        with self.lock:
            if self.closed or self.loading or self.model is None:
                return None
            if _key(payload) != self.key:
                return None
            result = self.model
            self._clear()
            return result

    def join(self):
        thread = self.thread
        if thread is not None and thread.ident is not None:
            thread.join()

    def close(self):
        with self.lock:
            self.closed = True
        self.join()
        with self.lock:
            self._clear()
