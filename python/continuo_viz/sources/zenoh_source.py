"""Watching a run as it happens, over Zenoh.

Subscribes to the viewer side channel the bridge publishes on. Only poses and
membership are asked for, which filters at Zenoh rather than in Python: a
command is published ten times a second per actor and never drawn, so there is
no reason for it to cross the process boundary at all.

The bridge does no filtering of its own, deliberately. At milestone 7
components publish these keys themselves and there is nothing in the middle to
filter, so a subscriber choosing its own key expression is the arrangement that
survives.
"""

from __future__ import annotations

from collections import deque
from typing import Any

from ..record import Event, event_from_sample

# How many samples may wait between frames before the oldest are discarded.
#
# The renderer drains at its frame rate and the world publishes far faster, so
# a backlog means the viewer is behind. Dropping the oldest is right for a live
# view: a pose from two seconds ago is not worth drawing when a newer one for
# the same actor is already queued behind it.
_QUEUE_LIMIT = 8192


class ZenohSource:
    """Live events from a running world.

    ``done`` is always ``False``. A live run has no end a subscriber can
    observe: silence means the world is paused, finished, or on the other side
    of a network partition, and none of those are distinguishable from here.
    """

    def __init__(self, world: str, config: Any = None) -> None:
        # Imported here rather than at module scope so the package stays
        # importable, lintable, and testable without Zenoh installed. Only a
        # live session needs it.
        try:
            import zenoh
        except ImportError as missing:  # pragma: no cover - depends on install
            raise RuntimeError(
                "watching a live run needs the `live` extra: "
                "pip install 'continuo-viz[live]'"
            ) from missing

        self.world = world
        self.done = False
        self._queue: deque[Event] = deque(maxlen=_QUEUE_LIMIT)
        self._session = zenoh.open(config if config is not None else zenoh.Config())
        self._subscribers = [
            self._session.declare_subscriber(expression, self._on_sample)
            for expression in self.key_expressions(world)
        ]

    @staticmethod
    def key_expressions(world: str) -> list[str]:
        """What a viewer subscribes to, and nothing more."""
        return [
            f"continuo_viz/{world}/actor/*/pose",
            f"continuo_viz/{world}/conductor/membership/status",
        ]

    def _on_sample(self, sample: Any) -> None:
        """Called on a Zenoh thread, so it parses and queues and nothing else."""
        attachment = sample.attachment
        try:
            event = event_from_sample(
                bytes(sample.payload.to_bytes()),
                bytes(attachment.to_bytes()) if attachment is not None else None,
            )
        except (ValueError, KeyError, UnicodeDecodeError):
            # A sample this viewer cannot read is not worth ending a live
            # session over, and raising here would only kill a Zenoh thread.
            return
        if event is not None:
            self._queue.append(event)

    def drain(self) -> list[Event]:
        """Everything received since the last call."""
        received = list(self._queue)
        self._queue.clear()
        return received

    def close(self) -> None:
        for subscriber in self._subscribers:
            subscriber.undeclare()
        self._subscribers.clear()
        self._session.close()

    def __enter__(self) -> ZenohSource:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
