"""Watching a continuo run, live or replayed.

The viewer is deliberately outside the simulation. Nothing here can perturb a
run, which is the same reason the Rust side observes the transport instead of
joining the world as a component: a component's presence changes the world
hash, and watching must not.

Reading a recorded log needs nothing beyond the standard library. Drawing needs
``pygame`` and watching a live run needs ``eclipse-zenoh``, both optional, so
the parser and the scene can be tested anywhere.
"""

from .protocol import KEY_ROOT, LOG_VERSION, VIZ_KEY_ROOT, MessageType
from .record import Event, Join, Leave, Message, Pose
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
    "Pose",
    "Scene",
]
