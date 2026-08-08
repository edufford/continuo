"""Turning a recorded log into an animated GIF.

Deliberately not paced against a wall clock, unlike watching. A recording
walks the log in fixed sim-time steps and draws one frame per step, so the
same log gives the same GIF on any machine: a slow encoder makes the recording
take longer rather than making the animation stutter, and there is no window,
so it works over ssh and in CI.

GIF rather than a video because the reason to record is usually to show
someone, and a GIF plays inline wherever it is pasted without a player or a
codec. A longer clip would want a real video format and the tooling that
comes with.
"""

from __future__ import annotations

from pathlib import Path

from .render import Renderer
from .scene import Scene
from .sources.log_source import LogTimeline

DEFAULT_FPS = 25
"""Frames per second of the finished animation, not the renderer's own rate.

A GIF stores each frame's delay in hundredths of a second, so only rates that
divide 100 are played at the rate they were asked for. Thirty is stored as
three hundredths and runs a three-second clip in 2.7; twenty-five is exact at
four, and far enough from the very short delays that some viewers quietly
replace with a longer one.
"""

DEFAULT_SECONDS = 3.0
"""How much of the run to record, in sim-seconds."""


def record_gif(
    log: Path | str,
    output: Path | str,
    *,
    start: float = 0.0,
    seconds: float = DEFAULT_SECONDS,
    fps: int = DEFAULT_FPS,
    follow: str | None = None,
) -> int:
    """Draws `seconds` of a log into an animated GIF. Returns the frame count.

    Events before `start` are applied to the scene without being drawn, so a
    clip can begin at the interesting part of a run and still show the world
    as it already was rather than an empty one.
    """
    # Imported here rather than at module scope so the package stays importable
    # for reading a log without an image library present.
    from PIL import Image

    if fps <= 0:
        raise ValueError(f"frames per second must be positive, got {fps}")
    if seconds <= 0.0:
        raise ValueError(f"a recording needs a positive length, got {seconds}")

    renderer = Renderer(onscreen=False)
    scene = Scene()
    timeline = LogTimeline(log)
    step = 1.0 / fps

    frames: list[Image.Image] = []
    try:
        # Counted rather than accumulated. Adding a step repeatedly drifts, and
        # a drift of one float bit at the end drops the last frame.
        for index in range(round(seconds * fps) + 1):
            instant = start + index * step

            # Everything the log has reached, drawing nothing on the way. Going
            # through the timeline rather than the file is what keeps a line
            # dated ahead of where it sits, such as the join of a car that will
            # first step later, from holding back the poses behind it and
            # freezing the picture until its own instant.
            for event in timeline.until(instant):
                scene.apply(event)

            # The HUD already carries the sim time, so the status says only
            # what kind of run this is, as it does when watching.
            renderer.draw(scene, follow, "recording")
            # Copied out now, since the next draw paints over the same surface.
            raw, size = renderer.frame_rgb()
            frames.append(Image.frombytes("RGB", size, raw))
            if timeline.done:
                break
    finally:
        timeline.close()
        renderer.close()

    if not frames:
        raise ValueError(f"{log} held nothing to record from {start} s")

    # `duration` is per frame in milliseconds, and `loop=0` means forever.
    # The encoder drops a frame identical to the one before it, so a clip of a
    # world where nothing moves comes out shorter than it was drawn.
    frames[0].save(
        output,
        save_all=True,
        append_images=frames[1:],
        duration=round(1000 / fps),
        loop=0,
        optimize=True,
    )

    # Return how many frames were drawn, which is what a caller reports.
    return len(frames)
