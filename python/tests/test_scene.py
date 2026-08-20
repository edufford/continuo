"""What the scene concludes from a stream of events.

The load-bearing case is departure. A car retiring is reported as two leaves,
one per component, with no event saying "the car is gone", so the scene has to
work it out. These pin that it works it out correctly and for the stated
reason, rather than by coincidence of the demo's naming.
"""

from __future__ import annotations

import json
import math

import pytest

from continuo_viz.events import (
    Event,
    Join,
    Leave,
    Message,
    event_from_log_line,
    event_from_sample,
)
from continuo_viz.pose import PoseTopDown, pose_from_payload
from continuo_viz.protocol import UnsupportedLogVersion, parse_actor_key
from continuo_viz.scene import Scene
from continuo_viz.sources.log_source import read_log

IDENTITY = {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}


# What every pose here carries, because the plant publishes its speed on
# the pose key and a payload without one is not a message the simulation
# sends.
#
# A speed a highway car would plausibly hold, rather than any number that
# parses. Nothing reads the field yet, but it is what a speed arrow on a
# car would be drawn from, and a fixture full of zeros or of nonsense
# would make the first thing to draw one look broken.
CRUISE_SPEED = 20.0


def pose_payload(
    x: float,
    y: float = 0.0,
    orientation: dict | None = None,
    speed: float = CRUISE_SPEED,
) -> dict:
    return {
        "position": {"x": x, "y": y, "z": 0.0},
        "orientation": orientation or IDENTITY,
        "speed": speed,
    }


def pose_message(
    actor: str, x: float, time: float = 0.0, part: str = "physics"
) -> Message:
    return Message(
        sim_time=time,
        key=f"continuo/demo/actor/{actor}/pose",
        publisher=f"{actor}/{part}",
        seq=0,
        payload=pose_payload(x),
    )


def test_an_actor_appears_on_its_first_pose():
    # Not on its join. A live viewer attaches whenever it likes and Zenoh
    # replays no history, so waiting for a join would mean never learning
    # about cars that were already driving.
    scene = Scene()
    scene.apply(pose_message("ego", 12.0, time=0.5))

    assert set(scene.actors) == {"ego"}
    assert scene.actors["ego"].pose.x == 12.0
    assert scene.sim_time == 0.5


def test_an_actor_is_bound_to_whoever_published_its_first_pose():
    scene = Scene()
    scene.apply(pose_message("traffic1", 40.0))

    assert scene.actors["traffic1"].pose_source == "traffic1/physics"


def test_a_car_retires_when_its_pose_source_leaves():
    scene = Scene()
    scene.apply(pose_message("traffic1", 40.0))
    scene.apply(pose_message("ego", 0.0))

    # The controller going first must not remove the car: it is not what
    # moves it, and there is still a physics component publishing poses.
    scene.apply(Leave(path="traffic1/controller", leaves_at=11.5))
    assert set(scene.actors) == {"traffic1", "ego"}

    scene.apply(Leave(path="traffic1/physics", leaves_at=11.5))
    assert set(scene.actors) == {"ego"}


def test_a_pose_after_a_leave_does_not_resurrect_an_actor():
    # Ordering is not guaranteed across a live network, so a straggling pose
    # behind a leave must not bring a car back from the dead.
    scene = Scene()
    scene.apply(pose_message("traffic1", 40.0))
    scene.apply(Leave(path="traffic1/physics", leaves_at=11.5))
    scene.apply(pose_message("traffic1", 41.0, time=11.6))

    assert scene.actors == {}


def test_a_rejoining_path_can_be_drawn_again():
    scene = Scene()
    scene.apply(pose_message("traffic1", 40.0))
    scene.apply(Leave(path="traffic1/physics", leaves_at=11.5))
    scene.apply(Join(path="traffic1/physics", first_due=20.0))
    scene.apply(pose_message("traffic1", 5.0, time=20.0))

    assert scene.actors["traffic1"].pose.x == 5.0


def test_world_level_components_are_never_drawn():
    # `logger` and `traffic_spawner` join and leave like anything else but
    # publish no pose, so they must not become actors.
    scene = Scene()
    scene.apply(Join(path="logger", first_due=0.0))
    scene.apply(Join(path="traffic_spawner", first_due=0.0))
    scene.apply(
        Message(
            sim_time=1.0,
            key="continuo/demo/conductor/membership/status",
            publisher="conductor",
            seq=0,
            payload={},
        )
    )

    assert scene.actors == {}


def test_commands_are_not_poses():
    scene = Scene()
    scene.apply(
        Message(
            sim_time=1.0,
            key="continuo/demo/actor/ego/steer_cmd",
            publisher="ego/controller",
            seq=0,
            payload={"yaw_rate_cmd": 0.0},
        )
    )

    assert scene.actors == {}
    assert scene.messages_seen == 1
    assert scene.poses_applied == 0


def test_a_second_publisher_moves_an_actor_without_taking_ownership():
    scene = Scene()
    scene.apply(pose_message("ego", 1.0))
    scene.apply(pose_message("ego", 2.0, time=0.1, part="shadow"))

    assert scene.actors["ego"].pose.x == 2.0
    assert scene.actors["ego"].pose_source == "ego/physics"


def test_parse_actor_key_only_matches_actor_keys():
    assert parse_actor_key("continuo/demo/actor/ego/pose") == ("ego", "pose")
    assert parse_actor_key("continuo/demo/actor/ego/steer_cmd") == (
        "ego",
        "steer_cmd",
    )
    assert parse_actor_key("continuo/demo/conductor/membership/status") is None
    assert parse_actor_key("continuo/demo/actor/ego/pose/extra") is None


def test_yaw_comes_out_of_the_quaternion():
    quarter_turn = math.pi / 2
    half = quarter_turn / 2
    pose = pose_from_payload(
        pose_payload(
            0.0,
            orientation={"w": math.cos(half), "x": 0.0, "y": 0.0, "z": math.sin(half)},
        )
    )

    assert pose is not None
    assert pose.yaw == pytest.approx(quarter_turn)


def test_a_quaternion_off_unit_length_still_gives_its_true_heading():
    # The yaw expression is not scale invariant, so a quaternion that has
    # drifted would otherwise read as a heading the car is not on. Matches
    # `Quat::normalized` on the Rust side, which scales before converting.
    quarter_turn = math.pi / 2
    half = quarter_turn / 2
    drifted = {
        "w": math.cos(half) * 1.05,
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half) * 1.05,
    }

    pose = pose_from_payload(pose_payload(0.0, orientation=drifted))

    assert pose is not None
    assert pose.yaw == pytest.approx(quarter_turn)


def test_a_quaternion_with_no_direction_reads_as_no_rotation():
    # All zeroes is not a rotation, and scaling it to unit length divides by
    # zero. This guards the guard rather than the formula: the yaw expression
    # answers 0 for it either way, and what would break is normalising without
    # the case for it. `Quat::normalized` answers with the identity, so this
    # does too.
    no_direction = {"w": 0.0, "x": 0.0, "y": 0.0, "z": 0.0}

    pose = pose_from_payload(pose_payload(0.0, orientation=no_direction))

    assert pose is not None
    assert pose.yaw == 0.0


def test_a_malformed_payload_is_ignored_rather_than_fatal():
    assert pose_from_payload({}) is None
    assert pose_from_payload({"position": {"x": None, "y": 0, "z": 0}}) is None
    assert pose_from_payload({"position": {"x": 1, "y": 0, "z": 0}}) is None


def test_a_log_line_and_a_live_sample_parse_to_the_same_event():
    # The whole point of splitting payload from metadata: two arrangements of
    # the same information, one record type, so the scene cannot tell which
    # source is attached.
    payload = pose_payload(1.5)
    line = json.dumps(
        {
            "msg": {
                "sim_time": 0.5,
                "key": "continuo/demo/actor/car1/pose",
                "publisher": "car1/physics",
                "seq": 7,
                "payload": payload,
            }
        }
    )
    attachment = json.dumps(
        {
            "message_type": "sim_data",
            "sim_time": 0.5,
            "key": "continuo/demo/actor/car1/pose",
            "publisher": "car1/physics",
            "seq": 7,
        }
    )

    from_log = event_from_log_line(line)
    from_live = event_from_sample(json.dumps(payload).encode(), attachment.encode())

    assert from_log == from_live


def test_a_membership_sample_is_identified_by_its_stated_type():
    # Not by its key, and not by having no metadata. The type is on the wire
    # so that a key moving or a field being added cannot change how a sample
    # is read.
    payload = json.dumps({"leave": {"path": "traffic1/physics", "leaves_at": 11.5}})
    attachment = json.dumps(
        {
            "message_type": "membership_status",
            "sim_time": 11.5,
            "key": "continuo/demo/conductor/membership/status",
            "publisher": "conductor",
            "seq": 3,
        }
    )

    assert event_from_sample(payload.encode(), attachment.encode()) == Leave(
        path="traffic1/physics", leaves_at=11.5
    )


def test_a_message_type_this_viewer_does_not_know_is_ignored():
    # M7 adds the tick protocol and the join and leave requests as further
    # types. A viewer that predates them must skip them, not read them as a
    # pose: falling through to `Message` would be the same mistake as
    # inferring the kind from which fields turned up.
    payload = json.dumps({"tick": 1})
    attachment = json.dumps(
        {
            "message_type": "tick_start",
            "sim_time": 0.5,
            "key": "continuo/demo/tick",
            "publisher": "conductor",
            "seq": 0,
        }
    )

    assert event_from_sample(payload.encode(), attachment.encode()) is None


def test_lines_the_viewer_has_no_use_for_are_skipped():
    assert (
        event_from_log_line(
            '{"tick":{"tick":1,"sim_time":0.0,"tick_hash":"ab","world_hash":"cd"}}'
        )
        is None
    )
    assert event_from_log_line('{"observed":{"budget":{}}}') is None
    assert event_from_log_line("  ") is None


def test_a_log_from_an_unknown_version_is_refused(tmp_path):
    log = tmp_path / "future.jsonl"
    log.write_text(json.dumps({"version": 99, "world_name": "demo"}) + "\n")

    with pytest.raises(UnsupportedLogVersion):
        list(read_log(log))


def test_a_log_without_a_header_is_refused(tmp_path):
    # A `WriterSink` capture is exactly this shape: valid log lines with no
    # header. Accepting it would let any headerless file bypass the version
    # gate, so refusal is the point rather than a limitation.
    log = tmp_path / "headerless.jsonl"
    log.write_text(
        json.dumps(
            {
                "msg": {
                    "sim_time": 0.0,
                    "key": "continuo/demo/actor/ego/pose",
                    "publisher": "ego/physics",
                    "seq": 0,
                    "payload": pose_payload(3.0),
                }
            }
        )
        + "\n",
        encoding="utf-8",
    )

    with pytest.raises(UnsupportedLogVersion):
        list(read_log(log))


def test_an_empty_file_is_not_a_log(tmp_path):
    log = tmp_path / "empty.jsonl"
    log.write_text("", encoding="utf-8")

    with pytest.raises(UnsupportedLogVersion):
        list(read_log(log))


def test_pose_is_hashable_and_comparable():
    # Frozen with slots, so a scene can be diffed cheaply and a pose cannot be
    # mutated out from under a frame that is mid-draw.
    assert PoseTopDown(1.0, 2.0, 0.0) == PoseTopDown(1.0, 2.0, 0.0)
    assert len({PoseTopDown(1.0, 2.0, 0.0), PoseTopDown(1.0, 2.0, 0.0)}) == 1


def test_every_event_reports_the_instant_it_belongs_at():
    # Each kind names its instant differently, and `LogSource` paces on the
    # one thing they share.
    assert pose_message("ego", 1.0, time=2.5).event_time == 2.5
    assert Join(path="ego/physics", first_due=4.0).event_time == 4.0
    assert Leave(path="ego/physics", leaves_at=9.0).event_time == 9.0


def test_an_event_kind_cannot_exist_without_an_instant():
    # The reason `Event` is abstract rather than a union alias. Pacing read
    # the instant off whichever known field it found and fell back to zero,
    # so a kind that forgot one would silently pace at the start of the run.
    # Now it cannot be constructed at all.
    class Undated(Event):
        __slots__ = ()

    with pytest.raises(TypeError, match="abstract"):
        # Instantiating an abstract class is exactly what is under test, so
        # the checker is right and is told to allow it here rather than be
        # worked around by routing the call through something it cannot see.
        Undated()  # pyright: ignore[reportAbstractUsage]
