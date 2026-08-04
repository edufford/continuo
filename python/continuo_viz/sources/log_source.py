"""Replaying a recorded run from its event log.

The log holds the run's sim times, so replay can be paced against them instead
of dumped as fast as the file reads. A thirty-second demo run is about five
megabytes and twenty-five thousand events, which streams line by line without
ever being held in memory.
"""

from __future__ import annotations

import json
import time
from collections.abc import Iterator
from pathlib import Path

from ..record import Event, check_log_version, event_from_log_line


def read_log(path: Path | str) -> Iterator[Event]:
    """Yields every event in a log, as fast as the file reads.

    Unpaced on purpose. This is what a test or a headless check wants, and it
    is what :class:`LogSource` wraps to add a clock.

    The header is validated when present. A log written by ``Recorder`` opens
    with ``{"version": 1, ...}``; a file written by the bridge's ``WriterSink``
    is a stream of the same lines with no header, so a missing one is accepted
    rather than treated as corruption.
    """
    with Path(path).open(encoding="utf-8") as lines:
        for lineno, line in enumerate(lines, start=1):
            if lineno == 1:
                stripped = line.strip()
                if stripped:
                    first = json.loads(stripped)
                    if isinstance(first, dict) and "version" in first:
                        check_log_version(first)
                        continue
            event = event_from_log_line(line)
            if event is not None:
                yield event


def event_time(event: Event) -> float:
    """The sim instant an event belongs at, for pacing."""
    for attribute in ("sim_time", "first_due", "leaves_at"):
        instant = getattr(event, attribute, None)
        if instant is not None:
            return float(instant)
    return 0.0


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
            self._sim_origin = event_time(self._pending)
        assert self._sim_origin is not None

        # Where the replay has got to, in the log's own time base.
        reached = self._sim_origin + (now - self._wall_origin) * self._speed

        ready: list[Event] = []
        while self._pending is not None and event_time(self._pending) <= reached:
            ready.append(self._pending)
            self._pending = next(self._events, None)

        if self._pending is None:
            self.done = True
        return ready
