"""Watching a continuo run, live or replayed.

The viewer is deliberately outside the simulation. Nothing here can perturb a
run, which is the same reason the Rust side observes the transport instead of
joining the world as a component: a component's presence changes the world
hash, and watching must not.

Reading a recorded log needs nothing beyond the standard library, so the parser
and the scene can be tested anywhere. Drawing brings in ``pygame`` and watching
a live run brings in ``eclipse-zenoh``; both install with the package, and both
are imported only by the code that needs them.
"""

from .events import Event, Join, Leave, Message
from .pose import PoseTopDown
from .protocol import (
    KEY_ROOT,
    LOG_VERSION,
    VIZ_KEY_ROOT,
    MessageType,
    UnsupportedLogVersion,
)
from .scene import Actor, Scene

__all__ = [
    "KEY_ROOT",
    "LOG_VERSION",
    "VIZ_KEY_ROOT",
    "Actor",
    "Event",
    "Join",
    "Leave",
    "Message",
    "MessageType",
    "PoseTopDown",
    "Scene",
    "UnsupportedLogVersion",
]
