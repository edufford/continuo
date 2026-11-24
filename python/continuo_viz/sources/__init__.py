"""Where events come from.

Every source exposes the same four things, so the render loop is written once:

- ``drain()`` returns the events that have become current since the last call,
  and never blocks
- ``started_again`` says whether that call saw the world begin a new run
- ``done`` says whether anything more can arrive
- ``close()`` releases whatever the source holds open

A live source is limited by a display clock trailing what has been received,
and a log source by where its replay clock has reached. Both decisions are the
source's own, which is what keeps pacing out of the renderer.
"""

from .log_source import LogSource, read_log
from .zenoh_source import ZenohSource

__all__ = ["LogSource", "ZenohSource", "read_log"]
