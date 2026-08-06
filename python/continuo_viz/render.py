"""Drawing a scene, top down.

The scale is uniform on both axes and the camera follows one car, rather than
stretching a long thin road to fill the window. Stretching would fit the whole
route on screen but shear every car the moment it turned, and turning is what
there is to watch. The cost is that the window has to be the shape of what it
shows, so the default is wide and short.

Frame rate is fixed and independent of message rate, so a world can publish as
fast as it likes without the renderer trying to draw every message.
"""

from __future__ import annotations

import colorsys
import math
import zlib
from dataclasses import dataclass

from .scene import Scene

# Nominal car footprint in metres, length by width.
#
# TODO(scene-graph): the simulation does not publish extents. A pose is a
# position and an orientation, so every drawn body is this one guess. The
# deferred `continuo/{world}/map` and scene-graph work is where a real size
# comes from; until then a lorry and a hatchback are the same rectangle.
#
# Extents are also what occlusion would need, whenever a world has anything to
# occlude. Poses carry a `z`, but `PoseTopDown` drops it because physics
# hardcodes it to zero, so today nothing is above anything else and there is
# nothing to order. A bridge over a road would need the bridge's footprint and
# height, not just a centre height, so depth ordering arrives with extents
# rather than before them, and it will want a stated rule rather than the
# incidental one below.
CAR_LENGTH = 4.5
CAR_WIDTH = 1.8

# Lateral offsets to draw lane markings at.
#
# TODO(map): these are *invented*. `traffic_world.rs` is explicit that "a lane
# is the lateral offset the controller holds, not geometry of the world", so
# the simulation has no lanes to publish and these are the viewer guessing what
# the demo meant. They are drawn faintly for that reason. A published world
# spec on `continuo/{world}/map` replaces them.
LANE_OFFSETS = (-3.5, 0.0, 3.5)
LANE_WIDTH = 3.5

TARGET_FPS = 60

# How wide a slice of road the camera shows, in metres.
#
# Traffic spawns and retires about 100 m either side of the ego, so this is
# what it takes to see a car coming rather than have it appear alongside.
VIEW_METRES = 200.0

# Pixels between the HUD and the road, leaving the top of the window to text.
_HUD_HEIGHT = 46

_BACKGROUND = (18, 20, 24)
_ROAD = (38, 41, 48)
_LANE_LINE = (70, 74, 84)
_TEXT = (208, 212, 220)
_TEXT_DIM = (128, 133, 143)
_FOCUS = (250, 214, 92)


@dataclass
class Camera:
    """Where the view is centred, and how many pixels a metre is worth."""

    centre_x: float
    centre_y: float
    scale: float
    width: int
    height: int
    top: int = 0
    """First row the world may be drawn on, below whatever the HUD occupies.

    Without this the road centres on the whole window and the HUD eats the
    space above it, so the picture ends up sitting high with a dead band
    underneath.
    """

    def to_screen(self, x: float, y: float) -> tuple[float, float]:
        """World metres to pixels, with +y drawn upward as a plan view expects."""
        return (
            self.width * 0.5 + (x - self.centre_x) * self.scale,
            (self.top + self.height) * 0.5 - (y - self.centre_y) * self.scale,
        )


def actor_colour(name: str, focused: bool) -> tuple[int, int, int]:
    """A stable colour per actor, so a car keeps its identity across frames.

    Derived from the name rather than from arrival order, which means a replay
    and a live view of the same run colour the same cars the same way, and a
    late-attaching viewer agrees with one that watched from the start.

    ``crc32`` rather than the built-in ``hash``, which is seeded per process
    for strings: two viewers of one run are two processes, so ``hash`` would
    have given each of them its own palette and quietly broken the property
    this function exists for.
    """
    if focused:
        return _FOCUS
    hue = (zlib.crc32(name.encode("utf-8")) % 360) / 360.0
    red, green, blue = colorsys.hsv_to_rgb(hue, 0.55, 0.95)
    return (int(red * 255), int(green * 255), int(blue * 255))


def body_corners(
    x: float, y: float, yaw: float, length: float, width: float
) -> list[tuple[float, float]]:
    """The four corners of a car body, in world metres.

    A rectangle rather than an arrow because the demo is traffic, and a row of
    arrows does not read as cars on a road. Heading is still legible: the body
    is longer than it is wide and it is drawn with a marked front.
    """
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    half_length, half_width = length * 0.5, width * 0.5
    return [
        (
            x + dx * cos_yaw - dy * sin_yaw,
            y + dx * sin_yaw + dy * cos_yaw,
        )
        for dx, dy in (
            (half_length, -half_width),
            (half_length, half_width),
            (-half_length, half_width),
            (-half_length, -half_width),
        )
    ]


class Renderer:
    """A pygame window showing a scene.

    Constructing this opens a window, so it is created only when something is
    actually being drawn. Everything above it in this package runs headless,
    which is what lets CI test the parser and the scene without a display.
    """

    def __init__(
        self, width: int = 1400, height: int = 240, title: str = "continuo"
    ) -> None:
        # Imported here rather than at module scope so a run that draws nothing
        # pays neither the SDL setup nor pygame's import banner. It is a plain
        # dependency, so failing to import it means a broken install.
        import pygame

        self._pygame = pygame
        pygame.init()
        self.surface = pygame.display.set_mode((width, height))
        pygame.display.set_caption(title)
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("consolas,dejavusansmono,monospace", 15)
        self.width = width
        self.height = height
        self.scale = width / VIEW_METRES

    def process_events(self) -> bool:
        """Handles pending window events.

        Returns ``False`` once the user has asked to close the window, which is
        the only thing that ends a viewing session.
        """
        for event in self._pygame.event.get():
            if event.type == self._pygame.QUIT:
                return False
            if event.type == self._pygame.KEYDOWN and event.key in (
                self._pygame.K_ESCAPE,
                self._pygame.K_q,
            ):
                return False
        return True

    def camera_for(self, scene: Scene, follow: str | None) -> Camera:
        """Centres on the followed actor, falling back to fitting what there is.

        A fallback matters more than it sounds: the followed car may not have
        joined yet, and it will eventually retire, so a viewer that only knows
        how to follow would show an empty road at both ends of a run.
        """
        followed = scene.actors.get(follow) if follow else None
        if followed is not None:
            centre_x = followed.pose.x
        elif scene.actors:
            xs = [actor.pose.x for actor in scene.actors.values()]
            centre_x = (min(xs) + max(xs)) * 0.5
        else:
            centre_x = 0.0
        return Camera(
            centre_x=centre_x,
            centre_y=0.0,
            scale=self.scale,
            width=self.width,
            height=self.height,
            top=_HUD_HEIGHT,
        )

    def draw(self, scene: Scene, follow: str | None, status: str) -> None:
        camera = self.camera_for(scene, follow)
        self.surface.fill(_BACKGROUND)
        self._draw_road(camera)

        # Ordered by position so labels are placed left to right, which makes
        # which one gets dropped in a crowd predictable instead of dependent
        # on dictionary order.
        #
        # Bodies inherit that order, so a car further along the road paints
        # over one behind it. Cars do overlap on screen, two in a lane a few
        # metres apart, and this is what decides them. It is stable rather
        # than chosen: nothing here reasons about which car should be on top,
        # because at one elevation there is no answer. See the extents note
        # above for when there is.
        in_view = sorted(scene.actors.values(), key=lambda actor: actor.pose.x)
        for actor in in_view:
            self._draw_body(camera, actor, focused=actor.name == follow)
        self._draw_labels(camera, in_view, follow)

        self._draw_hud(scene, follow, status)
        self._pygame.display.flip()
        self.clock.tick(TARGET_FPS)

    def _draw_road(self, camera: Camera) -> None:
        edge = max(LANE_OFFSETS) + LANE_WIDTH * 0.5
        top = camera.to_screen(0.0, edge)[1]
        bottom = camera.to_screen(0.0, -edge)[1]
        self._pygame.draw.rect(
            self.surface, _ROAD, (0, top, self.width, max(1.0, bottom - top))
        )
        # Lane boundaries rather than centres: a car drives *on* a centre line,
        # so drawing those would put a stripe down every roof.
        for offset in LANE_OFFSETS:
            for boundary in (offset - LANE_WIDTH * 0.5, offset + LANE_WIDTH * 0.5):
                y = camera.to_screen(0.0, boundary)[1]
                self._pygame.draw.line(
                    self.surface, _LANE_LINE, (0, y), (self.width, y), 1
                )

    def _draw_body(self, camera: Camera, actor, focused: bool) -> None:
        colour = actor_colour(actor.name, focused)
        corners = [
            camera.to_screen(x, y)
            for x, y in body_corners(
                actor.pose.x, actor.pose.y, actor.pose.yaw, CAR_LENGTH, CAR_WIDTH
            )
        ]
        self._pygame.draw.polygon(self.surface, colour, corners)
        # The leading edge, so heading is readable at a glance and a car
        # travelling backwards would be obvious rather than merely wrong.
        self._pygame.draw.line(self.surface, _BACKGROUND, corners[0], corners[1], 2)

    def _draw_labels(self, camera: Camera, actors: list, follow: str | None) -> None:
        """Names the cars, in a band above the road.

        Not beside each car, which is the obvious arrangement and does not fit:
        lanes are 3.5 m apart, which at this scale is 24 px, against a line of
        text 15 px tall. A label placed next to a car in one lane lands on the
        body of a car in the next.

        So the names go above the road and are **coloured to match their car**,
        which is what keeps them attributable once they are no longer touching
        what they name. Horizontal position still lines each one up with its
        car. Two rows are available, and a label with nowhere to go is dropped
        rather than overlaid, because two overlapping names read as one
        unfamiliar word and identify neither car. The followed car is placed
        first, so it never loses its label to a crowd.
        """
        road_top = camera.to_screen(0.0, max(LANE_OFFSETS) + LANE_WIDTH * 0.5)[1]
        rows = (road_top - 21, road_top - 38)
        taken: list[tuple[float, float, int]] = []

        ordered = sorted(actors, key=lambda actor: (actor.name != follow, actor.pose.x))
        for actor in ordered:
            name = self.font.render(
                actor.name, True, actor_colour(actor.name, actor.name == follow)
            )
            # Held inside the window, so a car at the edge of the view is still
            # named rather than showing half a word running off the side.
            centre_x = camera.to_screen(actor.pose.x, 0.0)[0]
            left = min(
                max(0.0, centre_x - name.get_width() * 0.5),
                self.width - name.get_width(),
            )
            right = left + name.get_width()

            for row, top in enumerate(rows):
                if any(
                    row == other_row and left < other_right and right > other_left
                    for other_left, other_right, other_row in taken
                ):
                    continue
                taken.append((left, right, row))
                self.surface.blit(name, (left, top))
                break

    def _draw_hud(self, scene: Scene, follow: str | None, status: str) -> None:
        followed = scene.actors.get(follow) if follow else None
        position = f"x {followed.pose.x:8.1f} m" if followed else "x        -"
        lines = [
            f"sim {scene.sim_time:7.2f} s   actors {len(scene.actors):3d}   {position}",
            f"{status}   poses {scene.poses_applied}   fps {self.clock.get_fps():4.0f}",
        ]
        for row, text in enumerate(lines):
            self.surface.blit(self.font.render(text, True, _TEXT), (12, 10 + row * 19))

    def close(self) -> None:
        self._pygame.quit()
