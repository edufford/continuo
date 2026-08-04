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

from continuo_viz.scene import Scene
from continuo_viz.sources.log_source import LogSource, event_time, read_log

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
        if event_time(event) > 2.0:
            break
        scene.apply(event)

    assert set(scene.actors) == {"ego", "traffic1", "traffic2"}
    assert scene.actors["traffic1"].pose.x == 150.0


def test_replay_pacing_releases_events_as_its_clock_reaches_them(tmp_path):
    log = tmp_path / "run.jsonl"
    build_log(log)

    # Fast enough that the whole four-second run is current immediately, which
    # keeps the test from waiting on a wall clock.
    source = LogSource(log, speed=100_000.0)
    collected = []
    while not source.done:
        collected.extend(source.drain())
    collected.extend(source.drain())

    scene = Scene()
    for event in collected:
        scene.apply(event)

    assert set(scene.actors) == {"ego", "traffic2", "traffic3"}
    assert source.done


def test_the_first_drain_is_not_delayed_by_a_late_starting_log(tmp_path):
    # Sim time is the log's own, so a run whose first event is at t=1000 must
    # begin immediately rather than after a thousand seconds of nothing.
    log = tmp_path / "late.jsonl"
    log.write_text(msg_line("ego", 5.0, 1000.0) + "\n", encoding="utf-8")

    source = LogSource(log, speed=1.0)
    assert len(source.drain()) == 1
