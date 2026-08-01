"""Which compiled core this process runs on.

``waddle-sdk`` ships two distributions built from ONE source tree (the
psycopg / psycopg-binary shape). The default ``waddle-sdk`` wheel bundles
``waddle._core``, built with the gRPC control transport. The companion
``waddle-sdk-teleop`` wheel — installed via ``pip install
'waddle-sdk[teleop]'`` — ships the SAME shim with the LiveKit media plane
also compiled in, as ``waddle_teleop._core``. This module is the one place
that decides which of the two the package actually runs on, so that no
other module ever has to.

Selection, in order:

1. No ``waddle_teleop`` installed → the bundled core.
2. ``WADDLE_NO_TELEOP=1`` → the bundled core (the escape hatch: reproduce
   a default install's behavior, or step around a bad companion wheel,
   without uninstalling anything).
3. Versions disagree → the bundled core, plus a warning naming both. The
   two wheels are one build of one ``Cargo.toml``, so a mismatch means a
   half-upgraded environment; loading a core built from different sources
   than this Python surface was written against is not a risk worth taking
   for a media transport.
4. Otherwise the teleop core: same shim, a strict superset of the bundled
   one's transports.

``import waddle._core`` keeps meaning the BUNDLED module everywhere — this
module never reassigns that submodule attribute, it only chooses what the
package's own call sites use. ``_core.pyi`` types both cores, because both
are the same shim.
"""

from __future__ import annotations

import os
import warnings
from typing import TYPE_CHECKING


def _select_core():
    """Return the core module this process should use (see the rules in the
    module docstring). Called exactly once, at import.

    The bundled core is imported first and unconditionally: it is the one
    that is always present, and it is what every rule falls back to. When
    the companion wins, both extension modules stay loaded — two
    independent modules with their own statics, of which only the selected
    one ever builds a session, so the cost is a few pages of memory and
    nothing is shared between them."""
    from . import _core as bundled

    try:
        from waddle_teleop import _core as teleop
    except ImportError:
        return bundled
    if os.environ.get("WADDLE_NO_TELEOP") == "1":
        return bundled
    if teleop.__version__ != bundled.__version__:
        warnings.warn(
            f"waddle-sdk-teleop {teleop.__version__} does not match waddle-sdk "
            f"{bundled.__version__}: ignoring it and running on the bundled core, "
            f"so LiveKit media stays unavailable. Install the matched pair with "
            f"pip install 'waddle-sdk[teleop]=={bundled.__version__}'",
            RuntimeWarning,
            stacklevel=2,
        )
        return bundled
    return teleop


if TYPE_CHECKING:  # both cores are the same shim; `_core.pyi` types both
    from . import _core as core
else:
    core = _select_core()

#: Which connected transports the selected core carries — ``"grpc"`` (the
#: control plane), ``"livekit"`` (the teleop media plane). This is the ONLY
#: feature detection the Python layer does, and it answers "can this build
#: do it at all", never "what should happen now".
FEATURES = core.FEATURES
