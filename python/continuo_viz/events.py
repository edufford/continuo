"""The events a viewer reads, and the two arrangements they arrive in.

A recorded log line and a live Zenoh sample carry the same information in two
arrangements, so both are parsed into the types here and nothing downstream
knows which was attached:

- a log line is ``{"msg": {time, key, publisher, seq, payload}}``, complete in
  itself because lines of every kind share one file
- a Zenoh sample is the payload bytes with
  ``{message_type, sim_time, key, publisher, seq}`` attached, because the
  payload already travelled as the payload and sending it twice would double
  every byte on the wire

Every live sample carries metadata, whatever kind it is, and says what kind it
is in ``message_type``. Nothing is identified by which fields happen to be
present or by matching its key against a pattern.

The ``key`` in both cases is the key the component *published* on, not the
viewer side channel it was relayed onto, which is what lets one parser name
actors the same way for a live run and a replay.
"""

from __future__ import annotations

import json
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any

from .protocol import MessageType


class Event(ABC):
    """Something that happened in the world, at a knowable instant.

    Each kind names its instant with its own field, because they mean
    different things: a message was published *at* one, a join first steps
    *at* one, a leave first *stops* at one. :attr:`event_time` is what they
    have in common, and it is abstract so that a new kind of event cannot
    exist without answering when it belongs.

    ``__slots__`` is empty so subclasses declared with ``slots=True`` keep
    theirs. Inheriting from a slotless base would hand every event a
    ``__dict__`` back and quietly undo them.
    """

    __slots__ = ()

    @property
    @abstractmethod
    def event_time(self) -> float:
        """The sim instant this belongs at, for ordering and pacing."""


@dataclass(frozen=True, slots=True)
class Message(Event):
    """One published message, with the metadata a raw payload lacks."""

    sim_time: float
    key: str
    publisher: str
    seq: int
    payload: dict[str, Any]

    @property
    def event_time(self) -> float:
        return self.sim_time


@dataclass(frozen=True, slots=True)
class Join(Event):
    """A component admitted to the world."""

    path: str
    first_due: float
    """The instant the newcomer first steps."""

    @property
    def event_time(self) -> float:
        return self.first_due


@dataclass(frozen=True, slots=True)
class Leave(Event):
    """A component removed from the world.

    The exact event a viewer needs in order to stop drawing something. Without
    it the only options are a staleness timer, which cannot tell a departed
    actor from a stalled simulation, or drawing ghosts forever.
    """

    path: str
    leaves_at: float
    """The first instant the component does not step."""

    @property
    def event_time(self) -> float:
        return self.leaves_at


def event_from_log_line(line: str) -> Event | None:
    """Parses one line of a recorded log.

    Returns ``None`` for a line the viewer has no use for, which is most of
    them: tick fingerprints outnumber everything, and observations describe
    what the machine did rather than what the world did. Skipping by returning
    ``None`` rather than by filtering on a list of known kinds means a log kind
    added later is ignored instead of crashing a viewer that predates it.
    """
    stripped = line.strip()
    if not stripped:
        return None
    event_dict = json.loads(stripped)
    if not isinstance(event_dict, dict):
        return None

    if (msg := event_dict.get("msg")) is not None:
        # TODO: the log spells this `time` and live metadata spells it
        # `sim_time`, for the same instant. `msg` is the only timestamped log
        # line that does not already say `sim_time`: `tick` and the `observed`
        # lines do. Renaming `RecordedMessage.time` to match changes the log
        # format and invalidates existing recordings, so it waits for a version
        # bump. When that lands, this becomes a straight read.
        return Message(
            sim_time=float(msg["time"]),
            key=str(msg["key"]),
            publisher=str(msg["publisher"]),
            seq=int(msg["seq"]),
            payload=msg["payload"],
        )
    if (join := event_dict.get("join")) is not None:
        return Join(path=str(join["path"]), first_due=float(join["first_due"]))
    if (leave := event_dict.get("leave")) is not None:
        return Leave(path=str(leave["path"]), leaves_at=float(leave["leaves_at"]))
    return None


def event_from_sample(payload: bytes, attachment: bytes) -> Event | None:
    """Parses one live sample into the same event a log line would give.

    The two halves are recombined here: a sample arrives as payload bytes with
    metadata attached, so this is where they become the single record a log
    line already is.

    Which kind it is comes from the metadata's ``message_type`` rather than
    from the key or from which fields turned up, so a key moving or a field
    being added cannot change how a sample is read. A membership payload is
    already a complete log line, so it is parsed as one.

    Every known type is matched explicitly and anything else returns ``None``.
    Milestone 7 adds the tick protocol and the join and leave requests as
    further types, and a viewer that predates them should ignore them rather
    than read them as something they are not.
    """
    meta = json.loads(attachment.decode("utf-8"))
    message_type = MessageType.parse(meta.get("message_type"))

    if message_type is MessageType.MEMBERSHIP_STATUS:
        return event_from_log_line(payload.decode("utf-8"))

    if message_type is MessageType.SIM_DATA:
        return Message(
            sim_time=float(meta["sim_time"]),
            key=str(meta["key"]),
            publisher=str(meta["publisher"]),
            seq=int(meta["seq"]),
            payload=json.loads(payload.decode("utf-8")),
        )

    return None
