"""Recording a log to an animated GIF.

The property worth pinning is that a recording does not depend on the machine
it was made on. Watching is paced against a wall clock and so drops frames when
the renderer falls behind; recording walks the log in fixed sim-time steps
instead, which is what lets a clip be regenerated and compared.
"""

from __future__ import annotations

import json

import pytest
from PIL import Image, ImageSequence
from test_replay import build_log, msg_line

from continuo_viz.recording import record_gif


def _frame_delays(image: Image.Image) -> list[int]:
    """How long each frame the file kept is held for, in milliseconds.

    Walking it is the only way to learn either the count or the delays, since
    the encoder drops a frame identical to the one before it and hands the
    survivor the time of the ones it replaced.
    """
    # Return one delay per surviving frame, in the order the file holds them.
    return [frame.info["duration"] for frame in ImageSequence.Iterator(image)]


def test_a_recording_has_a_frame_for_every_step(tmp_path):
    log = tmp_path / "run.jsonl"
    build_log(log)
    clip = tmp_path / "clip.gif"

    frames = record_gif(log, clip, seconds=2.0, fps=10)

    # Inclusive of both ends: a two-second clip at ten a second is the frame at
    # zero plus twenty more.
    assert frames == 21

    # The file holds fewer, because the encoder drops a frame identical to the
    # one before it and this log only moves on whole seconds. What it must keep
    # is how long the clip runs for, which it does by giving a surviving frame
    # the time of the ones it replaced.
    with Image.open(clip) as image:
        delays = _frame_delays(image)
        assert len(delays) < frames
        assert sum(delays) == frames * 100
        assert image.info["loop"] == 0


def test_the_same_log_records_the_same_clip(tmp_path):
    # The reason recording steps sim time rather than watching a clock. A
    # wall-paced capture would draw whatever it managed to keep up with, so two
    # runs on one machine could differ, let alone two machines.
    log = tmp_path / "run.jsonl"
    build_log(log)
    first, second = tmp_path / "a.gif", tmp_path / "b.gif"

    record_gif(log, first, seconds=2.0, fps=10)
    record_gif(log, second, seconds=2.0, fps=10)

    assert first.read_bytes() == second.read_bytes()


def test_a_clip_can_start_partway_through(tmp_path):
    # Events before the start are applied to the scene without being drawn,
    # so the first frame shows the world as it already was, not an empty one.
    log = tmp_path / "run.jsonl"
    build_log(log)
    clip = tmp_path / "clip.gif"

    frames = record_gif(log, clip, start=2.0, seconds=1.0, fps=10)

    assert frames == 11


def test_a_recording_stops_at_the_end_of_the_log(tmp_path):
    # Asking for more than there is gives what there is, rather than padding
    # the tail with copies of the last frame.
    log = tmp_path / "run.jsonl"
    build_log(log)
    clip = tmp_path / "clip.gif"

    frames = record_gif(log, clip, seconds=600.0, fps=10)

    assert 0 < frames < 6001


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"fps": 0}, "frames per second"),
        ({"seconds": 0.0}, "positive length"),
    ],
)
def test_a_recording_that_could_not_be_watched_is_refused(tmp_path, kwargs, message):
    log = tmp_path / "run.jsonl"
    build_log(log)

    with pytest.raises(ValueError, match=message):
        record_gif(log, tmp_path / "clip.gif", **kwargs)


def _moving_log(path, *, blocked: bool) -> None:
    """Two cars moving every 20 ms, optionally behind a join dated far ahead.

    Two, because the camera follows one: a lone car would sit still on screen
    and every frame would match whether or not reading had stalled.
    """
    lines = [json.dumps({"version": 1, "world_name": "demo", "world_seed": 42})]
    for actor in ("ego", "other"):
        lines.append(
            json.dumps({"join": {"path": f"{actor}/physics", "first_due": 0.0}})
        )
    for step in range(50):
        instant = step * 0.02
        if blocked and step == 25:
            # Dated well beyond the clip, so it is never drawn either way.
            lines.append(
                json.dumps({"join": {"path": "late/physics", "first_due": 99.0}})
            )
        lines.append(msg_line("ego", step * 1.0, instant))
        lines.append(msg_line("other", 20.0 + step * 0.5, instant))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def test_a_join_dated_ahead_does_not_freeze_the_picture(tmp_path):
    # A log is in recording order, so a join naming a later instant sits in
    # front of the poses that follow it. A reader that stopped there would draw
    # the same frame until that instant arrived, and the encoder would store
    # the repeats as one long pause. Both clips are compared whole, because a
    # join changes nothing that is drawn: it only clears a tombstone.
    plain, blocked = tmp_path / "plain.jsonl", tmp_path / "blocked.jsonl"
    _moving_log(plain, blocked=False)
    _moving_log(blocked, blocked=True)
    first, second = tmp_path / "a.gif", tmp_path / "b.gif"

    record_gif(plain, first, seconds=0.8, fps=25, follow="ego")
    record_gif(blocked, second, seconds=0.8, fps=25, follow="ego")

    assert first.read_bytes() == second.read_bytes()

    # Named directly as well, since that is the symptom someone would report.
    with Image.open(second) as clip:
        delays = set(_frame_delays(clip))
    assert delays == {40}, f"a frame was held longer than the rest: {delays}"
