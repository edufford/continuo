"""Command line entry point.

    python -m continuo_viz
    python -m continuo_viz --log run.jsonl
    python -m continuo_viz --log run.jsonl --check
    python -m continuo_viz --log run.jsonl --record clip.gif --record-from 20

Watching a live world is what no arguments gets you, since a world is the thing
you are most often already running. ``--log`` replays a recording instead, and
``--live`` says the default out loud for a script that would rather be explicit.

``--check`` folds a whole log into a scene and prints what it found, without
opening a window. It exists so a replay can be checked in CI, where there is no
display and installing one would be the only reason to.

``--record`` writes an animated GIF of a log instead of watching it, for
showing someone what a run looked like. It opens no window and does not use the
wall clock, so the same log gives the same clip anywhere.

Anything the viewer skips over is skipped silently by design, since one
unreadable sample is not worth ending a session for. ``--verbose`` is how you
find out what was skipped and why.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

from .events import Join, Leave
from .recording import DEFAULT_FPS, DEFAULT_SECONDS, record_gif
from .scene import Scene
from .sources import LogSource, ZenohSource, read_log

DEFAULT_WORLD = "demo"
DEFAULT_FOLLOW = "ego"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="continuo-viz",
        description="Watch a continuo run, live or replayed from a log.",
    )
    # Not required, so giving neither means live. The group stays so that
    # `--live --log run.jsonl` is an error rather than a silent choice between
    # them.
    source_group = parser.add_mutually_exclusive_group()
    source_group.add_argument(
        "--live",
        action="store_true",
        help="watch a running world over Zenoh (the default)",
    )
    source_group.add_argument(
        "--log", type=Path, metavar="PATH", help="replay a recorded event log"
    )

    parser.add_argument(
        "--world",
        default=DEFAULT_WORLD,
        help=f"world name to subscribe to when live (default: {DEFAULT_WORLD})",
    )
    parser.add_argument(
        "--follow",
        default=DEFAULT_FOLLOW,
        help=(
            f"actor to keep centered (default: {DEFAULT_FOLLOW}); "
            "pass an empty string to fit all actors instead"
        ),
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="fold the log into a scene and print a summary, drawing nothing",
    )
    parser.add_argument(
        "--record",
        type=Path,
        metavar="PATH",
        help="write an animated GIF of the log rather than watching it",
    )
    parser.add_argument(
        "--record-from",
        type=float,
        default=0.0,
        metavar="SECONDS",
        help="sim time to start the recording at (default: 0)",
    )
    parser.add_argument(
        "--record-seconds",
        type=float,
        default=DEFAULT_SECONDS,
        metavar="SECONDS",
        help=f"sim-seconds to record (default: {DEFAULT_SECONDS:g})",
    )
    parser.add_argument(
        "--record-fps",
        type=int,
        default=DEFAULT_FPS,
        metavar="N",
        help=f"frames per second of the finished clip (default: {DEFAULT_FPS})",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="also report every sample and payload the viewer could not read",
    )
    return parser


def run_check(path: Path) -> int:
    """Replays a whole log as fast as it reads and reports the final scene."""
    scene = Scene()
    joins = leaves = 0
    for event in read_log(path):
        scene.apply(event)
        if isinstance(event, Join):
            joins += 1
        elif isinstance(event, Leave):
            leaves += 1

    print(f"read {path}")
    print(f"  sim time reached : {scene.sim_time:.3f} s")
    print(f"  messages seen    : {scene.messages_seen}")
    print(f"  poses applied    : {scene.poses_applied}")
    print(f"  joins / leaves   : {joins} / {leaves}")
    print(f"  actors seen      : {scene.actors_seen}")
    print(f"  actors remaining : {len(scene.actors)}")
    for name in sorted(scene.actors):
        actor = scene.actors[name]
        print(
            f"    {name:<12} x {actor.pose.x:8.2f}  y {actor.pose.y:6.2f}"
            f"  from {actor.pose_source}"
        )
    if scene.poses_applied == 0:
        print("no poses found; nothing would have been drawn", file=sys.stderr)
        return 1
    return 0


def run_viewer(event_source, follow: str | None, status: str) -> None:
    """Drives the draw loop until the user closes the window.

    The event source running out does not end it. The loop keeps redrawing the
    scene it already has, so a replay leaves its last frame up to be read rather
    than vanishing the instant the log does. Only the HUD changes, to say the
    replay is over rather than merely paused.
    """
    # Deferred so a `--check` run does not import the renderer at all. Unlike
    # the sources, this module is not pulled in by the package.
    from .render import Renderer

    scene = Scene()
    renderer = Renderer()
    try:
        while renderer.process_events():
            for event in event_source.drain():
                scene.apply(event)
            if event_source.done:
                status = "replay finished"
            renderer.draw(scene, follow, status)
    finally:
        renderer.close()
        event_source.close()


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    # Configured here and nowhere else. The package logs through
    # `logging.getLogger(__name__)` and chooses no destination, which is what
    # lets anything embedding it decide where the lines go. This program is the
    # one place entitled to answer that.
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s: %(message)s",
    )

    # `--follow` defaults to the ego. Passing an empty string is the documented
    # way to ask for every actor instead of one, and the renderer spells that
    # `None`, so this converts it and passes any real name straight through.
    follow = args.follow or None

    # One branch per mode, each of them either refusing or acting, and one
    # exit code out of the whole chain.
    exit_code = 0

    # Refusing a mode that cannot run. argparse has no way to say that an
    # option requires another, so the two that read a log say it themselves.
    if args.log is None and (args.check or args.record is not None):
        wanted = "--check" if args.check else "--record"
        print(
            f"{wanted} reads a log; it has nothing to do with a live run",
            file=sys.stderr,
        )
        exit_code = 2

    # Watching a running world, which is the only mode that reads no log.
    # Every branch below this one therefore has one, which is also how the
    # type checker knows there is a log to read.
    elif args.log is None:
        run_viewer(ZenohSource(args.world), follow, f"live {args.world}")

    # Refusing a log that is not there, once, rather than in each mode under
    # it and in whatever order they happen to open it.
    elif not args.log.exists():
        print(f"no such log: {args.log}", file=sys.stderr)
        exit_code = 1

    # Reporting a whole log without drawing it, which is what has no display
    # to draw on: it returns its own code, since finding no poses is a result.
    elif args.check:
        exit_code = run_check(args.log)

    # Drawing a log to a file rather than a window, off its own sim-time
    # clock, so what comes out does not depend on the machine that ran it.
    elif args.record is not None:
        frames = record_gif(
            args.log,
            args.record,
            start=args.record_from,
            seconds=args.record_seconds,
            fps=args.record_fps,
            follow=follow,
        )
        print(f"wrote {args.record}: {frames} frames at {args.record_fps} fps")

    # Watching a log, which is the same loop the live case runs, differing
    # only in where the events come from.
    else:
        run_viewer(LogSource(args.log), follow, "replay")

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
