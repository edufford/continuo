"""Command line entry point.

    python -m continuo_viz
    python -m continuo_viz --log run.jsonl
    python -m continuo_viz --log run.jsonl --check

Watching a live world is what no arguments gets you, since a world is the thing
you are most often already running. ``--log`` replays a recording instead, and
``--live`` says the default out loud for a script that would rather be explicit.

``--check`` folds a whole log into a scene and prints what it found, without
opening a window. It exists so a replay can be checked in CI, where there is no
display and installing one would be the only reason to.

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

    # Everything that can refuse the arguments, before anything acts on them.
    # `--check` needs a log and argparse has no way to say so.
    if args.check and args.log is None:
        print(
            "--check reads a log; it has nothing to do with a live run",
            file=sys.stderr,
        )
        return 2
    if args.log is not None and not args.log.exists():
        print(f"no such log: {args.log}", file=sys.stderr)
        return 1

    if args.check:
        return run_check(args.log)

    # `--follow` defaults to the ego. Passing an empty string is the documented
    # way to ask for every actor instead of one, and the renderer spells that
    # `None`, so this converts it and passes any real name straight through.
    follow = args.follow or None
    if args.log is not None:
        event_source, status = LogSource(args.log), "replay"
    else:
        event_source, status = ZenohSource(args.world), f"live {args.world}"

    run_viewer(event_source, follow, status)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
