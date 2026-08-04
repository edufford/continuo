"""Where events come from.

Every source exposes the same two things, so the render loop is written once:

- ``drain()`` returns the events that have become current since the last call,
  and never blocks
- ``done`` says whether anything more can arrive

A live source is limited by what has been received; a log source is limited by
where its replay clock has reached. Both decisions are the source's own, which
is what keeps pacing out of the renderer.
"""

from .log_source import LogSource, read_log
from .zenoh_source import ZenohSource

__all__ = ["LogSource", "ZenohSource", "read_log"]
