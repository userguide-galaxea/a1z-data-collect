"""Shared monotonic wall-clock for cross-modal timestamp alignment.

time.monotonic() is immune to system-clock adjustments; the fixed epoch offset
maps it onto a wall-clock (epoch) timescale. The arm and camera collectors both
source timestamps from here, so their streams share one comparable clock and can
be aligned directly despite running at different rates.
"""

import time

_EPOCH_OFFSET: float = time.time() - time.monotonic()


def now() -> float:
    """Monotonic timestamp expressed on the wall-clock (epoch) timescale."""
    return time.monotonic() + _EPOCH_OFFSET
