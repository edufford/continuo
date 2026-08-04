"""Replaying a recorded run from its event log.

The log holds the run's sim times, so replay can be paced against them instead
of dumped as fast as the file reads. A thirty-second demo run is about five
megabytes and twenty-five thousand events, which streams line by line without
ever being held in memory.
"""

from __future__ import annotations

import time
from collections.abc import Iterator
from pathlib import Path

from ..events import Event, event_from_log_line
from ..protocol import check_log_header


def read_log(path: Path | str) -> Iterator[Event]:
    """Yields every event in a log, as fast as the file reads.

    Unpaced on purpose. This is what a test or a headless check wants, and it
    is what :class:`LogSource` wraps to add a clock.

    The first line must be the header ``Recorder`` writes, declaring a version
    this viewer reads. A headerless stream, a ``WriterSink`` capture for
    instance, is refused rather than tolerated: the version gate only means
    something if there is no unversioned way past it.
    """
    with Path(path).open(encoding="utf-8") as lines:
        check_log_header(next(lines, ""))

        for line in lines:
            event = event_from_log_line(line)
            if event is not None:
                yield event


class LogSource:
    """A log replayed against a wall clock.

    ``speed`` multiplies sim time against real time, so 1.0 replays a run at
    the rate it was simulated and 2.0 at twice that. The first event's sim time
    becomes the origin, so a log that does not start at zero still begins
    immediately rather than after a wait.
    """

    def __init__(self, path: Path | str, speed: float = 1.0) -> None:
        if speed <= 0.0:
            raise ValueError(f"replay speed must be positive, got {speed}")
        self._events = read_log(path)
        self._speed = speed
        self._pending: Event | None = next(self._events, None)
        self._sim_origin: float | None = None
        self._wall_origin: float | None = None
        self.done = self._pending is None

    def drain(self) -> list[Event]:
        """Every event whose sim time the replay clock has now reached."""
        if self._pending is None:
            self.done = True
            return []

        now = time.monotonic()
        if self._wall_origin is None:
            self._wall_origin = now
            self._sim_origin = self._pending.event_time
        assert self._sim_origin is not None

        # Where the replay has got to, in the log's own time base.
        reached = self._sim_origin + (now - self._wall_origin) * self._speed

        ready: list[Event] = []
        while self._pending is not None and self._pending.event_time <= reached:
            ready.append(self._pending)
            self._pending = next(self._events, None)

        if self._pending is None:
            self.done = True
        return ready
