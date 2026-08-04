"""Turning a published pose into what a plan view can draw.

Kept apart from the events that carry it: a pose is what a payload happens
to contain, and the event machinery neither knows nor cares. This is also
the only place that reduces three dimensions to two, so the renderer never
has to.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True, slots=True)
class PoseTopDown:
    """A pose projected onto the ground plane, which is all a plan view draws.

    The simulation works in three dimensions with a full quaternion. Reducing
    that to two axes and one angle happens once here rather than in the
    renderer every frame, which is why this is not simply continuo's `Pose`
    and is not named as though it were.

    There is deliberately no ``z``. Nothing that draws a top-down view has a
    use for one, and carrying it would suggest the renderer accounts for
    elevation somewhere. :func:`pose_from_payload` still requires it in the
    payload, because a position without one is not a pose continuo published.
    """

    x: float
    y: float
    yaw: float
    """Heading in radians, counter-clockwise from the +x axis."""


def pose_from_payload(payload: dict[str, Any]) -> PoseTopDown | None:
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
        # Required, then dropped: a position without a `z` is not one continuo
        # published, but the projection has no use for the value.
        _z = float(position["z"])
        w = float(orientation["w"])
        qx = float(orientation["x"])
        qy = float(orientation["y"])
        qz = float(orientation["z"])
    except (KeyError, TypeError, ValueError):
        return None

    # Yaw about +z, the only rotation a plan view can show. The standard
    # extraction, and stable for the near-level orientations a road produces.
    yaw = math.atan2(2.0 * (w * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return PoseTopDown(x=x, y=y, yaw=yaw)
