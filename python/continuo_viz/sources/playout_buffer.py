"""Releasing live events at the pace of their own sim times.

Each car publishes its own pose, so the poses of one instant reach a live
viewer as separate samples a few milliseconds apart. A frame drawn between
them would show some cars an instant behind the car the camera follows, which
looks like a twitch back and forth.

A playout buffer holds what arrives and releases it against a display clock
running a fixed delay behind the newest sim time received. By the time the
clock reaches an instant, all of that instant's samples have arrived.
"""

from __future__ import annotations

import heapq
import itertools

from ..events import Event

# Sim seconds the display clock trails the newest sim time received. It covers
# how far apart one instant's samples arrive, which is a few milliseconds on
# one machine, and longer only delays the picture.
DELAY_SECONDS = 0.05

# Sim seconds an event may be older than the newest one before it is taken to
# begin a new run of the world, such as a second run while the viewer stays
# open. A late sample from the same run is behind by milliseconds.
RESTART_SECONDS = 1.0


class PlayoutBuffer:
    """Events held until a display clock reaches their sim times.

    The display clock is a sim time. It advances at the wall clock's rate, so
    a burst of samples is drawn over the time it covers, but never past one
    delay behind the newest sim time received, so a world that pauses pauses
    the picture. When it falls more than an additional delay behind that, as
    for a viewer joining a run under way, it jumps forward.
    """

    def __init__(
        self, delay: float = DELAY_SECONDS, restart: float = RESTART_SECONDS
    ) -> None:
        self._delay = delay
        self._restart = restart
        # Ordered by sim time, then by arrival, so events of one instant keep
        # the order they came in.
        self._held: list[tuple[float, int, Event]] = []
        self._arrivals = itertools.count()
        self._newest_sim_time: float | None = None
        # The display clock: the sim time the last release reached, and the
        # wall time it was called at. Unset until the first release.
        self._released_to_sim_time: float | None = None
        self._released_at_wall_time: float | None = None

    def begins_new_run(self, event: Event) -> bool:
        """Whether ``event`` is older than the newest one received by more than
        the restart threshold, which is taken to mean the world began a new
        run.
        """
        # TODO(PLAN "Deferred"): a signal from the world when a run starts
        # replaces this guess.
        return (
            self._newest_sim_time is not None
            and event.event_time < self._newest_sim_time - self._restart
        )

    def reset(self) -> None:
        """Drops everything held and resets the display clock, for a new run
        of the world, whose events the clock would otherwise already be past.
        """
        self._held.clear()
        self._newest_sim_time = None
        self._released_to_sim_time = None
        self._released_at_wall_time = None

    def add(self, event: Event) -> None:
        """Holds an event until the display clock reaches its sim time."""
        heapq.heappush(self._held, (event.event_time, next(self._arrivals), event))
        if self._newest_sim_time is None or event.event_time > self._newest_sim_time:
            self._newest_sim_time = event.event_time

    def release(self, wall_now: float) -> list[Event]:
        """Every held event the display clock has reached at ``wall_now``, in
        sim-time order. ``wall_now`` is in seconds, from any clock that never
        goes backward.

        Each call releases up to the last call's sim time plus the wall time
        since, held to no later than the buffer limit, one delay behind the
        newest sim time received, and no earlier than an additional delay
        behind the buffer limit.
        """
        if self._newest_sim_time is None:
            return []

        # The buffer limit, the latest sim time the buffer releases: one delay
        # behind the newest sample received.
        buffer_limit_sim_time = self._newest_sim_time - self._delay
        if self._released_to_sim_time is None or self._released_at_wall_time is None:
            # The first release, or the first after a reset, goes up to the
            # buffer limit.
            release_to_sim_time = buffer_limit_sim_time
        else:
            # Move forward by the wall time since the last frame, so the
            # picture runs at real-time speed.
            elapsed_wall_time = wall_now - self._released_at_wall_time
            release_to_sim_time = self._released_to_sim_time + elapsed_wall_time
            # With no newer samples, as when the world pauses, wait at the
            # buffer limit, so the picture always trails the newest sample by
            # the delay.
            release_to_sim_time = min(release_to_sim_time, buffer_limit_sim_time)
            # If more than an additional delay behind the buffer limit, as when
            # the viewer joins a run already under way, catch up to an
            # additional delay behind it.
            release_to_sim_time = max(
                release_to_sim_time, buffer_limit_sim_time - self._delay
            )

        released = []
        while self._held and self._held[0][0] <= release_to_sim_time:
            released.append(heapq.heappop(self._held)[2])
        self._released_to_sim_time = release_to_sim_time
        self._released_at_wall_time = wall_now
        return released
