"""Watching a run as it happens, over Zenoh.

Subscribes to the viewer side channel, asking only for what gets drawn. That
filters at Zenoh rather than in Python, so traffic the viewer would discard
never crosses the process boundary.

The subscriber picks its own keys rather than relying on anything upstream to
narrow the stream for it, which is what lets the same code work whether those
keys are relayed or published at the source.
"""

from __future__ import annotations

from collections import deque
from typing import Any

from ..events import Event, event_from_sample
from ..protocol import VIZ_KEY_ROOT

# How many samples may wait between frames before the oldest are discarded.
#
# The renderer drains at its frame rate and the world publishes far faster, so
# a backlog means the viewer is behind. Dropping the oldest is right for a live
# view: a pose from two seconds ago is not worth drawing when a newer one for
# the same actor is already queued behind it.
_QUEUE_LIMIT = 8192


class ZenohSource:
    """Live events from a running world."""

    def __init__(self, world_name: str, zenoh_config: Any = None) -> None:
        """Opens a session and subscribes to one world's viewer keys.

        ``zenoh_config`` is handed straight to ``zenoh.open``. ``None`` takes
        Zenoh's own default, which finds peers on the local network by
        multicast and is what a viewer and a world on one machine need. Pass
        one to reach a world somewhere else, or to pin a transport.

        Its type is in the name rather than the annotation. Writing
        ``zenoh.Config`` there would mean importing Zenoh at module scope,
        which would load it for every replay as well, so the annotation can
        only say ``Any``.
        """
        # Imported here rather than at module scope so replaying a log does not
        # pay to load Zenoh. It is a plain dependency, so failing to import it
        # means a broken install.
        import zenoh

        self.world_name = world_name
        self._queue: deque[Event] = deque(maxlen=_QUEUE_LIMIT)
        self._session = zenoh.open(
            zenoh_config if zenoh_config is not None else zenoh.Config()
        )
        self._subscribers = [
            self._session.declare_subscriber(expression, self._on_sample)
            for expression in self.subscription_keys(world_name)
        ]

    @property
    def done(self) -> bool:
        """Always ``False``: a live run has no end a subscriber can observe.

        Silence means the world is paused, finished, or on the other side of a
        network partition, and none of those are distinguishable from here.
        """
        return False

    @staticmethod
    def subscription_keys(world_name: str) -> list[str]:
        """What a viewer subscribes to, and nothing more."""
        return [
            f"{VIZ_KEY_ROOT}/{world_name}/actor/*/pose",
            f"{VIZ_KEY_ROOT}/{world_name}/conductor/membership/status",
        ]

    def _on_sample(self, sample: Any) -> None:
        """Called on a Zenoh thread, so it parses and queues and nothing else."""
        attachment = sample.attachment
        if attachment is None:
            # Zenoh types this as optional, and every frame the bridge sends
            # carries metadata, so a sample without it was published by
            # something else that found its way onto the side channel.
            return
        try:
            event = event_from_sample(
                bytes(sample.payload.to_bytes()), bytes(attachment.to_bytes())
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
