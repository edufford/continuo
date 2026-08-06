"""Who is in the world right now, and where.

The simulation has no notion of an "actor". It has components, each with a
path like ``traffic1/physics``, and keys like ``continuo/demo/actor/traffic1/
pose``. A viewer draws *cars*, so it needs an entity that groups those, knows
which message moves it, and knows when it is gone.

Every part of that is worked out here rather than read off the wire, because
nothing publishes it. See ``TODO(scene-graph)`` in :meth:`Scene._apply_message`
for what would replace the inference, and the matching note in
:mod:`~continuo_viz.render` for the extents the renderer guesses for the same
reason.

**Presence is added on a pose and removed on a leave**, and the asymmetry is
deliberate. A live viewer can attach at any moment and Zenoh replays no
history, so waiting for a join event would mean never learning about cars that
were already driving. A pose arrives every 10 ms, so a late viewer is complete
within one frame. Departure cannot work that way, because nothing is published
when a car stops existing, which is exactly why the conductor publishes an
explicit leave.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .events import Event, Join, Leave, Message
from .pose import PoseTopDown, pose_from_payload
from .protocol import parse_actor_key


@dataclass
class Actor:
    """One drawable thing, and the component whose poses move it.

    ``pose_source`` is the load-bearing field. Membership is reported per
    *component*, so a car retiring arrives as two leaves, one for
    ``traffic1/controller`` and one for ``traffic1/physics``, and there is no
    event that says "the car is gone". Binding the actor to the component that
    actually publishes its pose answers that exactly: when that component
    leaves, nothing will move this actor again, so it goes.
    """

    name: str
    pose_source: str
    pose: PoseTopDown
    updated_at: float
    """Sim time of the most recent pose, for the HUD and for drawing staleness."""


@dataclass
class Scene:
    """Latest known state of every actor present.

    Fed one event at a time from whichever source is attached, so a replay and
    a live session build the same thing.
    """

    actors: dict[str, Actor] = field(default_factory=dict)
    sim_time: float = 0.0
    """Latest sim instant seen, which is what the viewer calls "now"."""

    departed: set[str] = field(default_factory=set)
    """Component paths known to have left.

    Kept so a pose that arrives after its publisher's leave cannot resurrect an
    actor. Bounded by the number of components a run ever had, and a path that
    joins again is removed from it.
    """

    messages_seen: int = 0
    poses_applied: int = 0
    actors_seen: int = 0
    """How many distinct actors have ever appeared, including retired ones.

    Against ``len(actors)`` this is what shows a run's turnover: the demo
    cycles fifteen cars through a road that holds seven.
    """

    def apply(self, event: Event) -> None:
        """Brings the scene up to date with one event, from whichever source."""
        if isinstance(event, Message):
            self._apply_message(event)
        elif isinstance(event, Join):
            # A name can in principle be reused after its owner left, so a
            # join clears any tombstone before a pose is believed again.
            self.departed.discard(event.path)
        elif isinstance(event, Leave):
            self._apply_leave(event)

    def _apply_message(self, message: Message) -> None:
        self.messages_seen += 1
        self.sim_time = max(self.sim_time, message.sim_time)

        parsed = parse_actor_key(message.key)
        if parsed is None:
            return
        name, signal = parsed
        # A pose is the only signal there is anything to draw for. Every other
        # one an actor publishes, commands today and whatever a later world
        # adds, stops here. Over Zenoh the viewer asks for poses alone, but a
        # recorded log holds every signal, so the filter has to exist here too.
        if signal != "pose":
            return
        if message.publisher in self.departed:
            return
        pose = pose_from_payload(message.payload)
        if pose is None:
            return

        self.poses_applied += 1
        existing = self.actors.get(name)
        if existing is None:
            # TODO(scene-graph): this binding is *inferred*, from whoever
            # published this actor's first pose. It holds for every world built
            # so far, where one component owns an actor's motion, and it needs
            # no naming convention, so it beats assuming a `{actor}/physics`
            # layout. It is still an assumption. `continuo/{world}/map` and the
            # deferred scene-graph publisher are where an actor's identity,
            # its pose source, and its extents should come from stated rather
            # than deduced; when they exist, this whole branch is replaced by
            # reading them.
            self.actors[name] = Actor(
                name=name,
                pose_source=message.publisher,
                pose=pose,
                updated_at=message.sim_time,
            )
            self.actors_seen += 1
            return

        # A second publisher on the same actor's key does not steal ownership,
        # so lifetime stays bound to whoever established the actor.
        existing.pose = pose
        existing.updated_at = message.sim_time

    def _apply_leave(self, leave: Leave) -> None:
        self.departed.add(leave.path)
        self.sim_time = max(self.sim_time, leave.leaves_at)
        gone = [
            name
            for name, actor in self.actors.items()
            if actor.pose_source == leave.path
        ]
        for name in gone:
            del self.actors[name]
