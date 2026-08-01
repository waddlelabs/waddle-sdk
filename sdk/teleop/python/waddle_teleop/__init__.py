"""Internal companion wheel for ``waddle-sdk`` — import ``waddle``, not this.

This distribution exists only to carry a second build of the same compiled
core (``waddle_teleop._core``), the one with the LiveKit media plane
compiled in. It has no Python surface of its own and never will: everything
you call lives in ``waddle``, which picks this core up automatically when it
is installed and its version matches (``waddle._native``).

Install it through the extra, never by name::

    pip install 'waddle-sdk[teleop]'
"""
