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

    Unpaced on purpose. This is what a test or a ``--check`` run wants, and it
    is what :class:`LogSource` wraps to add a clock.

    The first line must be the header ``Recorder`` writes, declaring a version
    this viewer reads. See :func:`~continuo_viz.protocol.check_log_header`.
    """
    with Path(path).open(encoding="utf-8") as lines:
        check_log_header(next(lines, ""))

        for line in lines:
            event = event_from_log_line(line)
            if event is not None:
                yield event


class LogSource:
    """A log replayed against a wall clock, one sim second per real second.

    Pacing is the only thing this adds to :func:`read_log`, which is what to
    use when a log is being processed rather than watched. The first event's
    sim time becomes the origin, so a log that does not start at zero still
    begins immediately rather than after a wait.
    """

    def __init__(self, path: Path | str) -> None:
        self._events = read_log(path)
        self._pending_event: Event | None = next(self._events, None)
        self._sim_origin: float | None = None
        self._wall_origin: float | None = None

    @property
    def done(self) -> bool:
        """Whether the log has been read to the end.

        Derived rather than stored, so it cannot fall out of step with what is
        left to read. One event is always held back unreturned, so this is
        false until that one has been handed out and nothing replaced it.
        """
        return self._pending_event is None

    def drain(self) -> list[Event]:
        """Every event whose sim time the replay clock has now reached."""
        if self._pending_event is None:
            return []

        now = time.monotonic()
        # Anchored on the first drain rather than in the constructor, so a
        # replay does not lose time to whatever happened between being built
        # and being asked for anything.
        if self._wall_origin is None:
            self._wall_origin = now
            self._sim_origin = self._pending_event.event_time
        assert self._sim_origin is not None

        # How far into the log's own time base the replay has got.
        sim_time_reached = self._sim_origin + (now - self._wall_origin)

        due_events: list[Event] = []
        while (
            self._pending_event is not None
            and self._pending_event.event_time <= sim_time_reached
        ):
            due_events.append(self._pending_event)
            self._pending_event = next(self._events, None)

        return due_events

    def close(self) -> None:
        """Closes the log, whether or not it was read to the end.

        Closing the generator closes the file it holds open. Without this the
        file would stay open until the collector got to it, which for a viewer
        abandoned partway through a long log is an arbitrary amount of time.
        """
        self._events.close()
