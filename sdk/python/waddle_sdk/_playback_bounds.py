"""Optional local joint envelopes; callers own the reason those ranges are safe."""

from collections.abc import Mapping

import numpy as np

from .runtime import FaultCode, JointPositionBounds, RuntimeFault, SupportFact


def install(run, bounds):
    with run._session._dispatch_lock:
        if run._episode is None or run._episode.done:
            raise RuntimeFault(FaultCode.CONFLICT, "Bounds require an active run")
        if not isinstance(bounds, Mapping):
            raise RuntimeFault(FaultCode.INVALID_REQUEST, "Provide named joint bounds")
        arms = run._session._require().arms
        facts = {row.scope: row.facts for row in run._session.support().rows}
        prepared = {}
        for part, limits in bounds.items():
            if part not in arms or not isinstance(limits, JointPositionBounds):
                raise RuntimeFault(
                    FaultCode.INVALID_REQUEST, "Bounds require a declared joint part"
                )
            if SupportFact.JOINT_PLAYBACK_BOUNDS not in facts[f"robot:{part}"]:
                raise RuntimeFault(FaultCode.UNSUPPORTED, "Part has no playback bounds")
            try:
                values = np.asarray((limits.lower, limits.upper), dtype=float)
            except (TypeError, ValueError) as error:
                raise RuntimeFault(
                    FaultCode.INVALID_REQUEST, "Bounds require finite position vectors"
                ) from error
            if (
                values.shape != (2, len(arms[part].joint_names))
                or not np.isfinite(values).all()
                or np.any(values[0] > values[1])
            ):
                raise RuntimeFault(
                    FaultCode.INVALID_REQUEST,
                    "Bounds require ordered finite limits for every declared joint",
                    context={"part": part},
                )
            prepared[part] = JointPositionBounds(*values.tolist())
        # Publish only a complete validated copy, at the ordinary dispatch boundary.
        run._playback_bounds = prepared


def observe(run, parts, targets=None):
    """Called with the dispatch lock, before native playback acknowledges a write."""
    try:
        observation = run._session.observe_parts(
            tuple(dict.fromkeys((*parts, *run._playback_bounds)))
        )
        for part, limits in run._playback_bounds.items():
            if part in observation.faults:
                raise observation.faults[part]
            measured = observation.parts[part].joint_position
            rows = [("feedback", measured)]
            if targets is not None and part in targets:
                rows.append(("reference", targets[part]))
            for source, values in rows:
                values = np.asarray(values, dtype=float)
                if (
                    values.shape != (len(limits.lower),)
                    or not np.isfinite(values).all()
                    or np.any(values < limits.lower)
                    or np.any(values > limits.upper)
                ):
                    raise RuntimeFault(
                        FaultCode.SAFETY_REFUSAL,
                        "playback_joint_bounds",
                        context={
                            "part": part,
                            "source": source,
                            "lower": list(limits.lower),
                            "upper": list(limits.upper),
                            "positions": [
                                float(v) if np.isfinite(v) else None
                                for v in values.reshape(-1)
                            ],
                        },
                    )
        return observation
    except Exception as error:
        if not run._playback_bounds:
            raise
        fault = RuntimeFault.from_exception(error)
        try:
            run.hold("Playback joint bounds are no longer established")
        except Exception as stop_error:  # noqa: BLE001 -- retain original evidence and separate stop uncertainty
            fault = RuntimeFault(
                fault.code,
                fault.detail,
                fault.retryable,
                {
                    **fault.context,
                    "hold_request_error": RuntimeFault.from_exception(
                        stop_error
                    ).as_dict(),
                },
                fault.causes,
            )
        raise fault
