"""Turning a published pose into what a plan view can draw.

Kept apart from the events that carry it: a pose is what a payload happens
to contain, and the event machinery neither knows nor cares. This is also
the only place that reduces three dimensions to two, so the renderer never
has to.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Any

logger = logging.getLogger(__name__)


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
        logger.debug("not a pose payload: %r", payload)
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
    except (KeyError, TypeError, ValueError) as unreadable:
        logger.debug("pose payload %r cannot be read: %s", payload, unreadable)
        return None

    # Scaled to unit length first, because the yaw below is not scale
    # invariant: its `1 - 2(y*y + z*z)` term assumes a norm of 1, so a
    # quaternion 5% off unit turns a 30 degree heading into 32.9 degrees.
    #
    # Rust: `continuo_core::Quat::normalized`, including the all-zero
    # quaternion, which has no direction to preserve and becomes the identity
    # on both sides. Without that case Python raises ZeroDivisionError here,
    # where Rust would carry a NaN heading into every corner of a drawn body.
    norm = math.sqrt(w * w + qx * qx + qy * qy + qz * qz)
    if norm == 0.0:
        w, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
    else:
        w, qx, qy, qz = w / norm, qx / norm, qy / norm, qz / norm

    # Yaw about +z, the only rotation a plan view can show, taken from the
    # intrinsic Z-Y-X (aerospace 3-2-1) decomposition.
    #
    # Rust: the same expression as `continuo_core::Quat::to_euler`, which cites
    # Diebel, "Representing Attitude: Euler Angles, Unit Quaternions, and
    # Rotation Vectors" (2006), eq. 290, and Wikipedia "Conversion between
    # quaternions and Euler angles" (Quaternion to ZYX Euler).
    yaw = math.atan2(2.0 * (w * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return PoseTopDown(x=x, y=y, yaw=yaw)
