"""Releasing live events at the pace of their own sim times.

Every car publishes its own pose, so the poses of one instant reach a live
viewer as separate samples spread over a few milliseconds. A frame drawn
partway through that spread would show some cars at the new instant and the
rest at the one before, and a car drawn one instant behind the car the camera
follows looks as if it twitched back by the distance it covers in that time.

A playout holds what arrives and releases it against a display clock that runs
a fixed delay behind the newest sim time received. By the time the clock
reaches an instant, everything published at it has arrived, so a frame shows
each car's own pose at the instant it occurred.
"""

from __future__ import annotations

import heapq
import itertools

from ..events import Event

# Sim seconds the display clock trails the newest event received.
#
# It has to cover how far apart the samples of one instant arrive, which is a
# few milliseconds on one machine. Longer only delays the picture, while
# shorter lets a late sample miss the frame its instant is drawn in.
DELAY_SECONDS = 0.05


class Playout:
    """Events held until a display clock trailing the newest one reaches them.

    The clock advances with the wall clock, so a burst of arrivals is drawn
    over the time it covers rather than all at once. It never passes the
    newest time received less the delay, so a world that pauses pauses the
    picture rather than running it dry. And it never falls more than a second
    delay behind that, so a world running faster than real time, or a viewer
    attaching late, catches up at once rather than drifting further behind.
    """

    def __init__(self, delay: float = DELAY_SECONDS) -> None:
        self._delay = delay
        # Ordered by sim time, then by arrival, so events of one instant keep
        # the order they came in.
        self._held: list[tuple[float, int, Event]] = []
        self._arrivals = itertools.count()
        self._newest: float | None = None
        self._shown: float | None = None
        self._shown_at_wall: float | None = None

    def add(self, event: Event) -> None:
        """Holds an event until the display clock reaches its instant."""
        heapq.heappush(self._held, (event.event_time, next(self._arrivals), event))
        if self._newest is None or event.event_time > self._newest:
            self._newest = event.event_time

    def release(self, wall_now: float) -> list[Event]:
        """Every held event the display clock has reached at ``wall_now``, in
        sim-time order.

        ``wall_now`` is a wall clock in seconds that never goes backward, read
        finely enough to tell one frame from the next.
        """
        if self._newest is None:
            return []

        latest = self._newest - self._delay
        if self._shown is None or self._shown_at_wall is None:
            shown = latest
        else:
            shown = min(self._shown + (wall_now - self._shown_at_wall), latest)
            shown = max(shown, latest - self._delay)
        self._shown, self._shown_at_wall = shown, wall_now

        released = []
        while self._held and self._held[0][0] <= shown:
            released.append(heapq.heappop(self._held)[2])
        return released
