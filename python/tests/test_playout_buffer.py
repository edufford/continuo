"""How a live source paces what it receives by the sim times it carries.

Times here are whole seconds with a two-second delay and a ten-second restart,
so every comparison is exact. The buffer does not care what the unit is.
"""

from __future__ import annotations

from continuo_viz.events import Message
from continuo_viz.sources.playout_buffer import PlayoutBuffer


def pose_at(time: float, actor: str = "ego") -> Message:
    return Message(
        sim_time=time,
        key=f"continuo/demo/actor/{actor}/pose",
        publisher=f"{actor}/physics",
        seq=0,
        payload={},
    )


def times(released: list) -> list[float]:
    return [event.event_time for event in released]


def test_events_are_released_as_the_display_clock_reaches_their_instants():
    buffer = PlayoutBuffer(delay=2.0, restart=10.0)
    for time in range(3):
        buffer.add(pose_at(float(time)))
    assert times(buffer.release(wall_now=0.0)) == [0.0]

    buffer.add(pose_at(3.0))
    assert times(buffer.release(wall_now=0.5)) == []
    assert times(buffer.release(wall_now=1.0)) == [1.0]


def test_an_instant_is_released_only_once_every_car_has_had_the_delay_to_arrive():
    buffer = PlayoutBuffer(delay=2.0, restart=10.0)
    for time in (0.0, 1.0, 2.0):
        buffer.add(pose_at(time, "ego"))
    buffer.add(pose_at(0.0, "traffic1"))
    buffer.release(wall_now=0.0)

    buffer.add(pose_at(1.0, "traffic1"))
    buffer.add(pose_at(3.0, "ego"))

    released = buffer.release(wall_now=1.0)
    assert [(event.event_time, event.publisher) for event in released] == [
        (1.0, "ego/physics"),
        (1.0, "traffic1/physics"),
    ]


def test_the_display_clock_stops_at_the_delay_behind_the_newest_event():
    buffer = PlayoutBuffer(delay=2.0, restart=10.0)
    buffer.add(pose_at(5.0))
    buffer.release(wall_now=0.0)

    assert buffer.release(wall_now=100.0) == []


def test_a_display_clock_over_an_additional_delay_behind_the_buffer_limit_catches_up():
    # The display clock starts at sim time 0, and then a hundred seconds of a
    # run arrive at once, as when the world runs far faster than real time.
    # Rather than play them out over a hundred seconds, it jumps to an
    # additional delay behind the buffer limit, the newest less two delays.
    buffer = PlayoutBuffer(delay=2.0, restart=10.0)
    buffer.add(pose_at(0.0))
    buffer.release(wall_now=0.0)
    for time in range(101):
        buffer.add(pose_at(float(time)))

    assert times(buffer.release(wall_now=1.0))[-1] == 96.0


def test_sim_time_going_back_past_the_restart_threshold_begins_a_new_run():
    # Inferred from sim time until the world says when a run starts.
    buffer = PlayoutBuffer(delay=2.0, restart=10.0)
    buffer.add(pose_at(30.0))

    assert not buffer.begins_new_run(pose_at(25.0))
    assert buffer.begins_new_run(pose_at(0.0))


def test_a_reset_buffer_holds_back_a_new_run_like_the_first():
    buffer = PlayoutBuffer(delay=2.0, restart=10.0)
    for time in range(31):
        buffer.add(pose_at(float(time)))
    buffer.release(wall_now=0.0)

    buffer.reset()
    for time in range(4):
        buffer.add(pose_at(float(time)))

    assert times(buffer.release(wall_now=1.0)) == [0.0, 1.0]
