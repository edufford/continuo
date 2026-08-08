"""Reading a recorded run from its event log.

The log holds the run's sim times, so a reader can release events against them
instead of dumping everything as fast as the file reads. It is read a line at a
time behind a small lookahead rather than held whole, since a recording grows
with the length of the run.
"""

from __future__ import annotations

import logging
import time
from collections.abc import Generator
from pathlib import Path

from ..events import Event, event_from_log_line
from ..protocol import check_log_header

logger = logging.getLogger(__name__)

# How many not-yet-due events the reader keeps in hand while looking ahead for
# events that are due.
#
# A log is in the order things were recorded, which is not the order things
# take effect in, so a line naming a future instant can sit in front of lines
# that are due now. Reading has to see past it, and has to stop somewhere:
# reading on until a due event turns up would buffer the whole log.
#
# A size rather than a limit. Reading stops on nothing else, so it tops the
# lookahead back up as arriving instants free part of it, and this many are
# held throughout a read.
#
# Size it above the events that can be dated ahead at one instant, which is
# components joining together, each naming when it will first step. Falling
# short of that costs a pause rather than correctness, and says so.
_NUM_LOOKAHEAD_EVENTS = 1024

# How long a pause has to be before it is worth reporting.
#
# A full lookahead is the ordinary state, so what deserves a word is one that
# stays full long enough to read as the picture stopping, rather than for the
# moment between one frame and the next.
_STALL_WARNING_SECONDS = 0.1


def read_log(path: Path | str) -> Generator[Event, None, None]:
    """Yields every event in a log, as fast as the file reads.

    In the order the log recorded them, which is not the order they take effect
    in. This is what a test or a ``--check`` run wants, and it is what
    :class:`LogTimeline` wraps to put them back in order.

    The first line must be the header ``Recorder`` writes, declaring a version
    this viewer reads. See :func:`~continuo_viz.protocol.check_log_header`.
    """
    with Path(path).open(encoding="utf-8") as lines:
        check_log_header(next(lines, ""))

        for line in lines:
            event = event_from_log_line(line)
            if event is not None:
                yield event


class LogTimeline:
    """A log, handing out events as the instants they take effect arrive.

    A log is in the order things were recorded, which is not the order things
    take effect in: a join is written where it happened but names the later
    instant its component will first step. Reading that stopped at the first
    event it had not reached would let one such line hold back every message
    behind it, so an event dated ahead is set aside and released when its own
    instant comes.

    Where those instants come from is the caller's business: :class:`LogSource`
    takes them from a wall clock, and recording steps them.
    """

    def __init__(self, path: Path | str) -> None:
        self._events = read_log(path)
        self._pending_event: Event | None = next(self._events, None)
        self._reported_stall = False
        """Whether the running stall has been reported, so it is said once."""

        self._lookahead_events: list[Event] = []
        """Events read from the log whose instant has not arrived yet.

        The whole of a reader's memory beyond the line being read, and it moves
        with neither the length of the log nor how often it is asked.
        """

    @property
    def done(self) -> bool:
        """Whether everything in the log has been handed out.

        Derived rather than stored, so it cannot fall out of step with what is
        left. Reaching the end of the file is not enough on its own: an event
        held back for an instant still to come is still owed.
        """
        return self._pending_event is None and not self._lookahead_events

    def earliest_instant(self) -> float:
        """The first instant the log names, which is where a reader starts.

        Not simply the first line's instant. A log opens with the joins of
        everything already in the world, and those name when each component
        will first step, which can be later than the messages behind them.
        Starting there would make everything earlier instantly due and hand out
        a run's opening moments at once.

        So it fills the lookahead and takes the earliest instant in it. Reading
        that far costs nothing extra, since the lookahead is where those events
        were going anyway.
        """
        self._fill_lookahead()

        # Return the earliest instant anything now in hand takes effect at.
        return min(event.event_time for event in self._lookahead_events)

    def until(self, sim_time: float) -> list[Event]:
        """Every event whose instant has arrived by ``sim_time``."""
        if self.done:
            return []

        # First the lookahead, which holds only events read earlier and set
        # aside, splitting it into whatever has since come due and what is
        # still waiting.
        due_events: list[Event] = []
        still_waiting: list[Event] = []
        for event in self._lookahead_events:
            if event.event_time <= sim_time:
                due_events.append(event)
            else:
                still_waiting.append(event)
        self._lookahead_events = still_waiting

        # Then the log itself, which is where all but a handful of events come
        # from. Each one read is either due and handed out, or set aside into
        # the space the pass above just freed.
        self._fill_lookahead(due_events, sim_time)
        self._report_any_stall(due_events, sim_time)

        # Return what has come due, in the order the log recorded it.
        return due_events

    def _fill_lookahead(
        self, due_events: list[Event] | None = None, sim_time: float = 0.0
    ) -> None:
        """Reads until the lookahead is full, handing out anything already due.

        :meth:`earliest_instant` calls this with nowhere to put due events,
        since it runs before a reader has any notion of now: nothing can be due
        yet, so everything read is held.
        """
        while (
            self._pending_event is not None
            and len(self._lookahead_events) < _NUM_LOOKAHEAD_EVENTS
        ):
            event = self._pending_event
            self._pending_event = next(self._events, None)
            if due_events is not None and event.event_time <= sim_time:
                due_events.append(event)
            else:
                self._lookahead_events.append(event)

    def _report_any_stall(self, due_events: list[Event], sim_time: float) -> None:
        """Says so when the lookahead is full of instants that are far off.

        The one way reading can still pause: every slot holds an event whose
        instant has not come, so no slot frees and nothing behind them can be
        read. Worth saying, since the symptom is a picture that stops for no
        visible reason. See :data:`_NUM_LOOKAHEAD_EVENTS` for what to change.
        """
        # A full lookahead alone is ordinary; handing nothing out from one is
        # what makes a read fruitless.
        if due_events or len(self._lookahead_events) < _NUM_LOOKAHEAD_EVENTS:
            self._reported_stall = False
            return
        if self._reported_stall:
            return

        earliest = min(event.event_time for event in self._lookahead_events)
        pause = earliest - sim_time
        if pause < _STALL_WARNING_SECONDS:
            return

        self._reported_stall = True
        logger.warning(
            "reading paused %.3f s: all %d lookahead slots hold events not yet due",
            pause,
            _NUM_LOOKAHEAD_EVENTS,
        )

    def close(self) -> None:
        """Closes the log, whether or not it was read to the end.

        Closing the generator closes the file it holds open. Without this the
        file would stay open until the collector got to it, which for a reader
        abandoned partway through a long log is an arbitrary amount of time.
        """
        self._events.close()


class LogSource:
    """A log replayed against a wall clock, one sim second per real second.

    Pacing is the only thing this adds to :class:`LogTimeline`. The earliest
    instant the log names becomes the origin, so a log that does not start at
    zero still begins immediately rather than after a wait.
    """

    def __init__(self, path: Path | str) -> None:
        self._timeline = LogTimeline(path)
        self._sim_origin: float | None = None
        self._wall_origin: float | None = None

    @property
    def done(self) -> bool:
        """Whether everything in the log has been handed out."""
        return self._timeline.done

    def drain(self) -> list[Event]:
        """Every event whose sim time the replay clock has now reached."""
        if self.done:
            return []

        now = time.monotonic()
        # Anchored on the first drain rather than in the constructor, so a
        # replay does not lose time to whatever happened between being built
        # and being asked for anything.
        if self._wall_origin is None:
            self._wall_origin = now
            self._sim_origin = self._timeline.earliest_instant()
        assert self._sim_origin is not None

        # Return everything the replay has reached, in the log's own time base.
        return self._timeline.until(self._sim_origin + (now - self._wall_origin))

    def close(self) -> None:
        """Closes the log, whether or not it was read to the end."""
        self._timeline.close()
