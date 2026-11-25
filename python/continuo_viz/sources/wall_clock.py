"""The wall clock the sources pace by."""

from __future__ import annotations

import time


def wall_seconds() -> float:
    """Seconds on a clock that never goes backward, read finely enough to tell
    one frame from the next.

    `perf_counter` rather than `monotonic`, which on Windows before Python
    3.13 ticks only every 15.6 ms, so one frame would see no time pass and the
    next see two ticks of it.
    """
    return time.perf_counter()
