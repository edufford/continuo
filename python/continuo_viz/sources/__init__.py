"""Where the viewer's events come from: a running world or a recorded log.

Both kinds of source have the same members, so one draw loop serves both:

- ``drain()`` returns the events that have come due since the last call,
  without blocking
- ``started_again`` says whether that call saw the world begin a new run
- ``done`` says whether anything more can arrive
- ``close()`` releases whatever the source holds open

Each source decides when an event comes due, a live one by its playout buffer
and a log by its replay clock, which keeps pacing out of the renderer.
"""

from .log_source import LogSource, read_log
from .zenoh_source import ZenohSource

__all__ = ["LogSource", "ZenohSource", "read_log"]
