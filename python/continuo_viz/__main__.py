"""Command line entry point.

    python -m continuo_viz --live
    python -m continuo_viz --log run.jsonl --speed 2
    python -m continuo_viz --log run.jsonl --headless

``--headless`` folds a whole log into a scene and prints what it found, without
opening a window. It exists so a replay can be checked in CI, where there is no
display and installing one would be the only reason to.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .record import Join, Leave
from .scene import Scene
from .sources import LogSource, read_log

DEFAULT_WORLD = "demo"
DEFAULT_FOLLOW = "ego"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="continuo-viz",
        description="Watch a continuo run, live or replayed from a log.",
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument(
        "--live", action="store_true", help="watch a running world over Zenoh"
    )
    source.add_argument(
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
            f"actor to keep centred (default: {DEFAULT_FOLLOW}); "
            "pass an empty string to fit all actors instead"
        ),
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="replay rate against real time (default: 1.0)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="fold the log into a scene and print a summary, drawing nothing",
    )
    return parser


def run_headless(path: Path) -> int:
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


def run_viewer(source, follow: str | None, status: str) -> int:
    """Drives the draw loop until the window closes or the source runs out."""
    from .render import Renderer

    scene = Scene()
    renderer = Renderer()
    try:
        while renderer.pump():
            for event in source.drain():
                scene.apply(event)
            renderer.draw(scene, follow, status)
            if source.done:
                # Keep the last frame on screen rather than vanishing the
                # instant a replay ends, so what happened stays readable.
                status = "replay finished"
    finally:
        renderer.close()
        close = getattr(source, "close", None)
        if close is not None:
            close()
    return 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    follow = args.follow or None

    if args.log is not None:
        if not args.log.exists():
            print(f"no such log: {args.log}", file=sys.stderr)
            return 1
        if args.headless:
            return run_headless(args.log)
        return run_viewer(
            LogSource(args.log, speed=args.speed), follow, f"replay x{args.speed:g}"
        )

    if args.headless:
        print(
            "--headless replays a log; it has nothing to do with --live",
            file=sys.stderr,
        )
        return 2

    from .sources import ZenohSource

    return run_viewer(ZenohSource(args.world), follow, f"live {args.world}")


if __name__ == "__main__":
    raise SystemExit(main())
