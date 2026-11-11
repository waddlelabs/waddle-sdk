"""Bounded errors carried unchanged across the device boundary."""


def error_payload(error):
    """Keep structured provider faults intact, including their original causes."""
    if hasattr(error, "as_dict"):
        result = error.as_dict()
        from waddle_sdk.diagnostics import capture_exception

        if not isinstance(result, dict) or (
            result.get("context") is not None and not isinstance(result["context"], dict)
        ):
            capture_exception(error)
            return result
        diagnostic_id = (
            None
            if (result.get("context") or {}).get("diagnostic_id")
            else capture_exception(error)
        )
        if diagnostic_id is not None:
            result = {
                **result,
                "context": {
                    **(result.get("context") or {}),
                    "diagnostic_id": diagnostic_id,
                },
            }
        return result
    from waddle_sdk.runtime import RuntimeFault

    return RuntimeFault.from_exception(error).as_dict()


class RelayError(ValueError):
    """A bounded relay refusal retaining its code, detail and structured context."""

    def __init__(self, code: str, detail: str, **context):
        super().__init__(detail)
        self.code, self.detail, self.context = code, detail, context

    def as_dict(self):
        return {"code": self.code, "detail": self.detail, "context": self.context}
