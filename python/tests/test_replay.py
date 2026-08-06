"""Folding a whole run into a scene, and reading it at a known instant.

Built as a log file rather than as events in memory, so this exercises the same
path a replay takes: the header, the lines the viewer skips, and the pacing
wrapper that decides when each event becomes current.

The stream is shaped like the traffic demo, where cars spawn ahead and retire
behind while the ego drives through, because turnover is what a viewer has to
get right and a static scene would prove nothing.
"""

from __future__ import annotations

import json
import logging
from types import SimpleNamespace

from continuo_viz.scene import Scene
from continuo_viz.sources import log_source
from continuo_viz.sources.log_source import LogSource, read_log

IDENTITY = {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0}


def msg_line(actor: str, x: float, time: float, part: str = "physics") -> str:
    return json.dumps(
        {
            "msg": {
                "time": time,
                "key": f"continuo/demo/actor/{actor}/pose",
                "publisher": f"{actor}/{part}",
                "seq": 0,
                "payload": {
                    "position": {"x": x, "y": 0.0, "z": 0.0},
                    "orientation": IDENTITY,
                },
            }
        }
    )


def build_log(path) -> None:
    """A miniature demo: ego throughout, two cars that retire, one that stays."""
    lines = [json.dumps({"version": 1, "world_name": "demo", "world_seed": 42})]

    for actor in ("ego", "traffic1", "traffic2"):
        for part in ("controller", "physics"):
            lines.append(
                json.dumps({"join": {"path": f"{actor}/{part}", "first_due": 0.0}})
            )

    for step in range(3):
        time = step * 1.0
        lines.append(msg_line("ego", 30.0 * step, time))
        lines.append(msg_line("traffic1", 100.0 + 25.0 * step, time))
        lines.append(msg_line("traffic2", 200.0 + 25.0 * step, time))
        # Ticks outnumber everything in a real log and carry nothing a viewer
        # wants, so one is here to be skipped.
        lines.append(
            json.dumps(
                {
                    "tick": {
                        "tick": step,
                        "sim_time": time,
                        "tick_hash": "0" * 16,
                        "world_hash": "1" * 16,
                    }
                }
            )
        )

    # traffic1 retires at 3 s, both of its components.
    for part in ("controller", "physics"):
        lines.append(
            json.dumps({"leave": {"path": f"traffic1/{part}", "leaves_at": 3.0}})
        )

    # A late arrival, so the scene has to add as well as remove.
    for part in ("controller", "physics"):
        lines.append(
            json.dumps({"join": {"path": f"traffic3/{part}", "first_due": 4.0}})
        )
    lines.append(msg_line("ego", 120.0, 4.0))
    lines.append(msg_line("traffic2", 275.0, 4.0))
    lines.append(msg_line("traffic3", 400.0, 4.0))

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def test_the_final_scene_holds_exactly_who_is_still_driving(tmp_path):
    log = tmp_path / "run.jsonl"
    build_log(log)

    scene = Scene()
    for event in read_log(log):
        scene.apply(event)

    assert set(scene.actors) == {"ego", "traffic2", "traffic3"}
    assert "traffic1" not in scene.actors, "a retired car must not be drawn"
    assert scene.actors_seen == 4, "four cars appeared over the run"
    assert scene.sim_time == 4.0
    assert scene.actors["ego"].pose.x == 120.0
    assert scene.actors["traffic3"].pose.x == 400.0


def test_a_scene_read_partway_through_still_holds_the_retired_car(tmp_path):
    # The known instant that matters: before the leave, traffic1 is present
    # and drawn. Without this the test above would also pass on a viewer that
    # never added it in the first place.
    log = tmp_path / "run.jsonl"
    build_log(log)

    scene = Scene()
    for event in read_log(log):
        if event.event_time > 2.0:
            break
        scene.apply(event)

    assert set(scene.actors) == {"ego", "traffic1", "traffic2"}
    assert scene.actors["traffic1"].pose.x == 150.0


def test_real_time_pacing_withholds_what_its_clock_has_not_reached(tmp_path):
    # Withholding is the whole job, and it is observable on the first drain,
    # so this asserts it directly rather than draining to completion. Draining
    # to completion would have to wait out the log's four seconds, and would
    # still only prove that the loop terminates.
    log = tmp_path / "run.jsonl"
    build_log(log)

    source = LogSource(log)
    first = source.drain()

    assert first, "whatever sits at the origin is current straight away"
    assert all(event.event_time == 0.0 for event in first)
    assert not source.done, "four seconds of log cannot have arrived yet"


def log_with_a_join_from_the_future(path) -> None:
    """Two poses due now, with a join for a much later instant between them."""
    path.write_text(
        "\n".join(
            [
                json.dumps({"version": 1, "world_name": "demo", "world_seed": 42}),
                msg_line("ego", 0.0, 0.0),
                json.dumps({"join": {"path": "traffic1/physics", "first_due": 30.0}}),
                msg_line("ego", 1.0, 0.0),
            ]
        )
        + "\n",
        encoding="utf-8",
    )


def test_a_join_from_the_future_does_not_hold_back_the_poses_behind_it(tmp_path):
    # A join is written where it happened but names the later instant its
    # component first steps, so a log is not sorted by `event_time`. Reading
    # used to stop at the first event it had not reached, so one such line held
    # back every pose behind it and the view froze until its instant arrived.
    log = tmp_path / "run.jsonl"
    log_with_a_join_from_the_future(log)

    source = LogSource(log)
    released = source.drain()

    assert [type(event).__name__ for event in released] == ["Message", "Message"]
    assert not source.done, "the join is still owed until its own instant"


def test_a_join_from_the_future_is_delivered_when_its_instant_arrives(
    tmp_path, monkeypatch
):
    # Held back, not dropped. Reading past it must not turn into skipping it.
    log = tmp_path / "run.jsonl"
    log_with_a_join_from_the_future(log)

    # A clock the test moves rather than one it waits on, so thirty seconds
    # pass between the two drains without the suite taking thirty seconds.
    now = 1000.0
    monkeypatch.setattr(log_source, "time", SimpleNamespace(monotonic=lambda: now))

    source = LogSource(log)
    source.drain()
    now += 31.0
    released = source.drain()

    assert [type(event).__name__ for event in released] == ["Join"]
    assert source.done


def test_a_lookahead_too_small_for_the_log_says_so_rather_than_just_pausing(
    tmp_path, monkeypatch, caplog
):
    # The one way a replay can still pause: every slot holds an event whose
    # instant has not come, so nothing can be read past them. Silent would mean
    # a picture that stops for no visible reason.
    monkeypatch.setattr(log_source, "_NUM_LOOKAHEAD_EVENTS", 2)
    log = tmp_path / "run.jsonl"
    log.write_text(
        "\n".join(
            [
                json.dumps({"version": 1, "world_name": "demo", "world_seed": 42}),
                msg_line("ego", 0.0, 0.0),
                json.dumps({"join": {"path": "a/physics", "first_due": 30.0}}),
                json.dumps({"join": {"path": "b/physics", "first_due": 30.0}}),
                json.dumps({"join": {"path": "c/physics", "first_due": 30.0}}),
                msg_line("ego", 1.0, 0.0),
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    source = LogSource(log)
    with caplog.at_level(logging.WARNING):
        released = source.drain()
        source.drain()

    assert [type(event).__name__ for event in released] == ["Message"]
    assert not source.done, "the second pose is stuck behind joins that do not fit"
    warnings = [r for r in caplog.records if r.levelno == logging.WARNING]
    assert len(warnings) == 1, "said once per stall, not once per drain"
    assert "lookahead" in warnings[0].getMessage()


def test_a_log_source_closes_the_file_it_holds_open(tmp_path):
    # The draw loop closes in a `finally` and calls `source.close()` directly
    # rather than probing for the method, so this has to exist, has to release
    # the file even when the log was abandoned partway through, and has to
    # tolerate being called on a source that is already closed.
    log = tmp_path / "run.jsonl"
    build_log(log)

    source = LogSource(log)
    source.drain()
    source.close()
    source.close()

    # `read_log` holds the file open inside a generator, so a closed generator
    # is a closed file. Asserted on the generator rather than by removing the
    # log, because whether an open file can be removed is a property of the
    # platform: a leak that one refuses would go unnoticed on another.
    assert source._events.gi_frame is None


def test_the_first_drain_is_not_delayed_by_a_late_starting_log(tmp_path):
    # Sim time is the log's own, so a run whose first event is at t=1000 must
    # begin immediately rather than after a thousand seconds of nothing.
    log = tmp_path / "late.jsonl"
    header = json.dumps({"version": 1, "world_name": "demo", "world_seed": 42})
    log.write_text(
        header + "\n" + msg_line("ego", 5.0, 1000.0) + "\n", encoding="utf-8"
    )

    source = LogSource(log)
    assert len(source.drain()) == 1
