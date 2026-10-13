"""Port call/return logging without coupling the domain to ROS logging APIs."""

from functools import wraps

MAX_REPR_LENGTH = 500


def _bounded_repr(value) -> str:
    text = repr(value)
    if len(text) <= MAX_REPR_LENGTH:
        return text
    return f"{text[: MAX_REPR_LENGTH - 3]}..."


class LoggedPort:
    """Transparent decorator that logs every public adapter method boundary."""

    def __init__(self, name: str, delegate, logger):
        self._name = name
        self._delegate = delegate
        self._logger = logger

    def __getattr__(self, method_name: str):
        attribute = getattr(self._delegate, method_name)
        if method_name.startswith("_") or not callable(attribute):
            return attribute

        @wraps(attribute)
        def logged_call(*args, **kwargs):
            call = f"{self._name}.{method_name}"
            self._logger.info(
                f"[PORT] CALL {call} " f"args={_bounded_repr(args)} kwargs={_bounded_repr(kwargs)}"
            )
            try:
                result = attribute(*args, **kwargs)
            except Exception as exc:
                self._logger.error(f"[PORT] ERROR {call} exception={_bounded_repr(exc)}")
                raise
            self._logger.info(f"[PORT] RETURN {call} result={_bounded_repr(result)}")
            return result

        return logged_call
