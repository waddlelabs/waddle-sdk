"""Public adaptation of native input handles; no polling or renewal thread."""

from ._native import core
from .runtime import FaultCode, RuntimeFault, SupportFact


class PlaybackInput:
    def __init__(self, run, native):
        self._run, self._native = run, native

    def snapshot(self):
        return self._native.snapshot()

    def refresh(self, sequence: int):
        try:
            return self._native.refresh(sequence)
        except (TypeError, ValueError, OverflowError) as error:
            raise RuntimeFault.from_exception(
                error, code=FaultCode.INVALID_REQUEST
            ) from error

    def release(self, reason: str) -> None:
        self._native.release(reason)


def start(run, timeout_ns: int) -> PlaybackInput:
    if run._episode is None:
        raise RuntimeFault(FaultCode.NOT_OPEN, "run has not started")
    if run._episode.done:
        raise RuntimeFault(FaultCode.CONFLICT, "run is already terminal")
    if not any(
        SupportFact.INPUT_WATCHDOG_PLAYBACK in row.facts
        for row in run._session.support().rows
    ):
        raise RuntimeFault(FaultCode.UNSUPPORTED, "run has no input watchdog support")
    try:
        return PlaybackInput(run, run._episode.start_playback_input(timeout_ns))
    except core.PlaybackBusyError as error:
        raise RuntimeFault(FaultCode.BUSY, str(error)) from error
    except (TypeError, ValueError, OverflowError) as error:
        raise RuntimeFault.from_exception(
            error, code=FaultCode.INVALID_REQUEST
        ) from error
