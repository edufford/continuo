"""What a viewer reads, independent of where it came from.

A recorded log line and a live Zenoh sample carry the same information in two
arrangements, so both are parsed into the types here and nothing downstream
knows which was attached:

- a log line is ``{"msg": {time, key, publisher, seq, payload}}``, complete in
  itself because lines of every kind share one file
- a Zenoh sample is the payload bytes with ``{time, key, publisher, seq}``
  attached, because the payload already travelled as the payload and sending
  it twice would double every byte on the wire

The ``key`` in both cases is the key the component *published* on, not the
viewer side channel it was relayed onto, which is what lets one parser name
actors the same way for a live run and a replay.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass
from typing import Any

# The log header's `version` field. A log written by an older or newer
# continuo is not silently half-understood: parsing refuses it instead.
LOG_VERSION = 1


class UnsupportedLogVersion(Exception):
    """A log declares a format this viewer does not know how to read."""


@dataclass(frozen=True, slots=True)
class Message:
    """One published message, with the provenance a raw payload lacks."""

    time: float
    key: str
    publisher: str
    seq: int
    payload: dict[str, Any]


@dataclass(frozen=True, slots=True)
class Join:
    """A component admitted to the world."""

    path: str
    first_due: float


@dataclass(frozen=True, slots=True)
class Leave:
    """A component removed from the world.

    The exact event a viewer needs in order to stop drawing something. Without
    it the only options are a staleness timer, which cannot tell a departed
    actor from a stalled simulation, or drawing ghosts forever.
    """

    path: str
    leaves_at: float


Event = Message | Join | Leave

# `continuo/{world}/actor/{name}/{signal}`. Anchored at both ends so a longer
# key cannot match by accident.
_ACTOR_KEY = re.compile(r"^continuo/[^/]+/actor/(?P<actor>[^/]+)/(?P<signal>[^/]+)$")


def actor_signal(key: str) -> tuple[str, str] | None:
    """Splits an actor key into its actor name and signal.

    Returns ``None`` for anything else, which is how world-level traffic and
    conductor notifications are ignored without listing them.
    """
    match = _ACTOR_KEY.match(key)
    if match is None:
        return None
    return match["actor"], match["signal"]


@dataclass(frozen=True, slots=True)
class Pose:
    """A pose flattened to what a top-down view can draw.

    The simulation works in three dimensions and a full quaternion. A plan view
    needs two of the axes and one angle, so the conversion happens once here
    rather than in the renderer every frame.
    """

    x: float
    y: float
    z: float
    yaw: float
    """Heading in radians, counter-clockwise from the +x axis."""


def pose_from_payload(payload: dict[str, Any]) -> Pose | None:
    """Reads a pose payload, or ``None`` if it is not one.

    Returning ``None`` rather than raising keeps an unexpected payload on a
    pose key from ending a live session. A viewer that stops at the first
    surprise is worse than one that draws what it understood.
    """
    position = payload.get("position")
    orientation = payload.get("orientation")
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        return None
    try:
        x = float(position["x"])
        y = float(position["y"])
        z = float(position["z"])
        w = float(orientation["w"])
        qx = float(orientation["x"])
        qy = float(orientation["y"])
        qz = float(orientation["z"])
    except (KeyError, TypeError, ValueError):
        return None

    # Yaw about +z, the only rotation a plan view can show. The standard
    # extraction, and stable for the near-level orientations a road produces.
    yaw = math.atan2(2.0 * (w * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return Pose(x=x, y=y, z=z, yaw=yaw)


def check_log_version(header: dict[str, Any]) -> None:
    """Raises unless a log header declares a format this viewer reads."""
    version = header.get("version")
    if version != LOG_VERSION:
        raise UnsupportedLogVersion(
            f"log declares version {version!r}, this viewer reads {LOG_VERSION}"
        )


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
    event = json.loads(stripped)
    if not isinstance(event, dict):
        return None

    if (msg := event.get("msg")) is not None:
        return Message(
            time=float(msg["time"]),
            key=str(msg["key"]),
            publisher=str(msg["publisher"]),
            seq=int(msg["seq"]),
            payload=msg["payload"],
        )
    if (join := event.get("join")) is not None:
        return Join(path=str(join["path"]), first_due=float(join["first_due"]))
    if (leave := event.get("leave")) is not None:
        return Leave(path=str(leave["path"]), leaves_at=float(leave["leaves_at"]))
    return None


def event_from_sample(payload: bytes, attachment: bytes | None) -> Event | None:
    """Parses one live sample into the same event a log line would give.

    The two halves are recombined here: a message arrives as payload bytes with
    provenance attached, so this is where they become the single record a log
    line already is. A sample with no attachment is a conductor notification,
    whose payload is a complete log line and needs nothing alongside it.
    """
    if attachment is None:
        return event_from_log_line(payload.decode("utf-8"))

    meta = json.loads(attachment.decode("utf-8"))
    return Message(
        time=float(meta["time"]),
        key=str(meta["key"]),
        publisher=str(meta["publisher"]),
        seq=int(meta["seq"]),
        payload=json.loads(payload.decode("utf-8")),
    )
