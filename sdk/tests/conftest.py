"""Shared fixtures.

Only one so far, and it exists because of a rule every test that touches
`waddle.robots.base`'s console recovery has to obey: **no test may take the
developer's own terminal.** The reader starts only when stdin is a foreground
TTY — which it IS under `pytest -s` — and a thread reading the real one would
sit in `for line in sys.stdin` eating keystrokes for the rest of the run. So a
test that wants a console says so and gets one of its own.
"""

from __future__ import annotations

import queue
import sys

import pytest

from waddle.robots import base


class Terminal:
    """A terminal the test types at, one line at a time.

    Blocking between lines the way a real one is: a reader of this is a thread
    parked mid-read, which is the state everything about retiring a reader has
    to survive. A `StringIO` cannot show that — its reader reaches
    end-of-input immediately and dies, which is the one case the lifecycle
    handles for free.
    """

    def __init__(self) -> None:
        self._typed: queue.Queue[str | None] = queue.Queue()

    def type(self, line: str) -> None:
        """Type one line, as a site operator would."""
        self._typed.put(line)

    def end(self) -> None:
        """End of input — the reader thread ends here, observably."""
        self._typed.put(None)

    def isatty(self) -> bool:
        return True

    def __iter__(self) -> Terminal:
        return self

    def __next__(self) -> str:
        line = self._typed.get()
        if line is None:
            raise StopIteration
        return line


@pytest.fixture
def terminal(monkeypatch):
    """A foreground terminal of this test's own, as `sys.stdin`.

    The predicate is decided here too (`console_is_at_the_machine`), so no
    test inherits an answer from however the suite happened to be invoked, and
    the input is ended at teardown so the reader thread this test started ends
    with it rather than outliving it.
    """
    stream = Terminal()
    monkeypatch.setattr(base, "console_is_at_the_machine", lambda: True)
    monkeypatch.setattr(sys, "stdin", stream)
    yield stream
    stream.end()
