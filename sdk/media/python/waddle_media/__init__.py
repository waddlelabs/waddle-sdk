"""Internal media companion wheel for ``waddle-sdk`` — import ``waddle_sdk``, not this.

This distribution exists only to carry a second build of the same compiled
core (``waddle_media._core``), the one with the LiveKit media plane
compiled in. It has no Python surface of its own and never will: everything
you call lives in ``waddle_sdk``, which picks this core up automatically when it
is installed and its version matches (``waddle_sdk._native``).

Install it through the extra, never by name::

    pip install 'waddle-sdk[media]'
"""
