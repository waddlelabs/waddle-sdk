"""Bounded errors carried unchanged across the device boundary."""


def error_payload(error):
    """Keep structured provider faults intact, including their original causes."""
    if hasattr(error, "as_dict"):
        return error.as_dict()
    from waddle_sdk.runtime import RuntimeFault

    return RuntimeFault.from_exception(error).as_dict()


class RelayError(ValueError):
    """A bounded relay refusal retaining its code, detail and structured context."""

    def __init__(self, code: str, detail: str, **context):
        super().__init__(detail)
        self.code, self.detail, self.context = code, detail, context

    def as_dict(self):
        return {"code": self.code, "detail": self.detail, "context": self.context}
