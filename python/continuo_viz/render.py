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
from enum import Enum

from .scene import Scene

# Nominal car footprint in meters, length by width.
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
# height, not just a center height, so depth ordering arrives with extents
# rather than before them, and it will want a stated rule rather than the
# incidental one below.
CAR_LENGTH = 4.5
CAR_WIDTH = 1.8

# The windshield, which is what makes a car's heading readable at a glance.
#
# Proportions of the body rather than absolute meters, so they stay right if
# the footprint above ever comes from a published extent instead of a guess.
# Drawn as an outline, set forward of center, and entirely within the body: a
# mark laid on the outline itself would fall half on the road and only shorten
# the car, which at this scale is one pixel and reads as nothing.
_WINDSHIELD_LENGTH = CAR_LENGTH * 0.20
_WINDSHIELD_WIDTH = CAR_WIDTH * 0.65
_WINDSHIELD_FORWARD = CAR_LENGTH * 0.20
_WINDSHIELD_PIXELS = 1

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

# How wide a slice of road the camera shows, in meters.
#
# Traffic spawns and retires about 100 m either side of the ego, so this is
# what it takes to see a car coming rather than have it appear alongside.
VIEW_METERS = 200.0

# Pixels between the HUD and the road, leaving the top of the window to text.
_HUD_HEIGHT = 46


class _Color(tuple, Enum):
    """The fixed palette, as RGB.

    A type rather than loose constants, so being a color is said once by what
    they are instead of repeated in each of their names. Actor colors are not
    in here: those are derived per name by :func:`actor_color` rather than
    chosen.

    Inherits ``tuple`` so a member is an RGB triple wherever pygame wants one,
    without every call site reaching for ``.value``. Same reasoning as
    ``MessageType`` inheriting ``str``.
    """

    BACKGROUND = (18, 20, 24)
    ROAD = (38, 41, 48)
    LANE_LINE = (70, 74, 84)
    TEXT = (208, 212, 220)
    FOCUS = (250, 214, 92)
    WINDSHIELD = (0, 0, 0)


@dataclass
class Camera:
    """Where the view is centered, and how many pixels a meter is worth.

    Both units meet here, which is the whole job, so each field says which one
    it is in.
    """

    center_x: float
    """Where the view is centered, in world meters."""

    center_y: float
    """Where the view is centered, in world meters."""

    scale: float
    """Pixels per world meter, the same on both axes."""

    width: int
    """Window width in pixels."""

    height: int
    """Window height in pixels."""

    top_y: int = 0
    """First pixel row the world may be drawn on, below whatever the HUD takes.

    Without this the road centers on the whole window and the HUD eats the
    space above it, so the picture ends up sitting high with a dead band
    underneath.
    """

    def to_pixels(self, world_x: float, world_y: float) -> tuple[float, float]:
        """World meters to pixels, with +y drawn upward as a plan view expects."""
        return (
            self.width * 0.5 + (world_x - self.center_x) * self.scale,
            (self.top_y + self.height) * 0.5 - (world_y - self.center_y) * self.scale,
        )


# Degrees around the color wheel, which is the space a hue is picked from.
# Named because the two uses below have to be the same number: one picks a
# degree, the other scales it to the 0 to 1 that `colorsys` wants.
_HUE_DEGREES = 360

# The saturation and brightness every derived actor color is drawn at.
#
# Held constant so that hue is the only thing that varies, which is the whole
# point of going through HSV: one dial to turn, and two cars that differ do so
# in a way that reads as identity rather than as lighting.
#
# Neither is at full. These two are chosen by eye, for colors that stay legible
# both as a body against `_Color.ROAD` and as 15 px label text, and nothing
# measured backs the exact values.
_ACTOR_SATURATION = 0.55
_ACTOR_BRIGHTNESS = 0.95


def actor_color(name: str, focused: bool) -> tuple[int, int, int]:
    """A stable color per actor, so a car keeps its identity across frames.

    Derived from the name rather than from arrival order, which means a replay
    and a live view of the same run color the same cars the same way, and a
    late-attaching viewer agrees with one that watched from the start.

    ``focused`` marks the actor the viewer is following, which takes the one
    reserved color instead of a derived one so that the car you asked to watch
    can be found without reading a label. It is the only actor whose color does
    not follow from its name.

    ``crc32`` rather than the built-in ``hash``, which is seeded per process
    for strings: two viewers of one run are two processes, so ``hash`` would
    have given each its own palette and quietly broken the property above. A
    fixed seed is not something this module can ask for either, since
    ``PYTHONHASHSEED`` is read once at interpreter start and nothing here can
    reach back before that. Any hash stable across processes would do, and
    ``crc32`` is the cheapest in the standard library that hands back an int.

    Two actors can still land on near enough the same hue, and that is not the
    hash's doing: a handful of names over a wheel of 360 collide by birthday,
    and blake2b and md5 cluster the demo's cast just as tightly. Spreading them
    properly would mean assigning by arrival order, which is the one thing this
    function must not do.
    """
    if focused:
        return _Color.FOCUS
    hue = (zlib.crc32(name.encode("utf-8")) % _HUE_DEGREES) / _HUE_DEGREES
    red, green, blue = colorsys.hsv_to_rgb(hue, _ACTOR_SATURATION, _ACTOR_BRIGHTNESS)
    return (int(red * 255), int(green * 255), int(blue * 255))


def body_corners(
    x: float, y: float, yaw: float, length: float, width: float
) -> list[tuple[float, float]]:
    """The four corners of a car body, in world meters.

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
        self.scale = width / VIEW_METERS

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
        """Centers on the followed actor, falling back to fitting what there is.

        A fallback matters more than it sounds: the followed car may not have
        joined yet, and it will eventually retire, so a viewer that only knows
        how to follow would show an empty road at both ends of a run.
        """
        followed = scene.actors.get(follow) if follow else None
        if followed is not None:
            center_x = followed.pose.x
        elif scene.actors:
            xs = [actor.pose.x for actor in scene.actors.values()]
            center_x = (min(xs) + max(xs)) * 0.5
        else:
            center_x = 0.0
        return Camera(
            center_x=center_x,
            center_y=0.0,
            scale=self.scale,
            width=self.width,
            height=self.height,
            top_y=_HUD_HEIGHT,
        )

    def draw(self, scene: Scene, follow: str | None, status: str) -> None:
        camera = self.camera_for(scene, follow)
        self.surface.fill(_Color.BACKGROUND)
        self._draw_road(camera)

        # Ordered by position so labels are placed left to right, which makes
        # which one gets dropped in a crowd predictable instead of dependent
        # on dictionary order.
        #
        # Bodies inherit that order, so a car further along the road paints
        # over one behind it. Cars do overlap on screen, two in a lane a few
        # meters apart, and this is what decides them. It is stable rather
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
        top = camera.to_pixels(0.0, edge)[1]
        bottom = camera.to_pixels(0.0, -edge)[1]
        self._pygame.draw.rect(
            self.surface, _Color.ROAD, (0, top, self.width, max(1.0, bottom - top))
        )
        # Lane boundaries rather than centers: a car drives *on* a center line,
        # so drawing those would put a stripe down every roof.
        for offset in LANE_OFFSETS:
            for boundary in (offset - LANE_WIDTH * 0.5, offset + LANE_WIDTH * 0.5):
                y = camera.to_pixels(0.0, boundary)[1]
                self._pygame.draw.line(
                    self.surface, _Color.LANE_LINE, (0, y), (self.width, y), 1
                )

    def _draw_body(self, camera: Camera, actor, focused: bool) -> None:
        color = actor_color(actor.name, focused)
        corners = [
            camera.to_pixels(x, y)
            for x, y in body_corners(
                actor.pose.x, actor.pose.y, actor.pose.yaw, CAR_LENGTH, CAR_WIDTH
            )
        ]
        self._pygame.draw.polygon(self.surface, color, corners)

        # The windshield, so heading is readable at a glance and a car
        # travelling backwards is obvious rather than merely wrong. Sitting
        # forward of center is what says which way the car faces; being a
        # windshield rather than a stripe is what makes that read as a car.
        #
        # `body_corners` is a rotated rectangle either way, so it places this
        # one too, about its own center rather than the body's.
        cos_yaw, sin_yaw = math.cos(actor.pose.yaw), math.sin(actor.pose.yaw)
        windshield = [
            camera.to_pixels(x, y)
            for x, y in body_corners(
                actor.pose.x + _WINDSHIELD_FORWARD * cos_yaw,
                actor.pose.y + _WINDSHIELD_FORWARD * sin_yaw,
                actor.pose.yaw,
                _WINDSHIELD_LENGTH,
                _WINDSHIELD_WIDTH,
            )
        ]
        self._pygame.draw.polygon(
            self.surface, _Color.WINDSHIELD, windshield, _WINDSHIELD_PIXELS
        )

    def _draw_labels(self, camera: Camera, actors: list, follow: str | None) -> None:
        """Names the cars, in a band above the road.

        Not beside each car, which is the obvious arrangement and does not fit:
        lanes are 3.5 m apart, which at this scale is 24 px, against a line of
        text 15 px tall. A label placed next to a car in one lane lands on the
        body of a car in the next.

        So the names go above the road and are **colored to match their car**,
        which is what keeps them attributable once they are no longer touching
        what they name. Horizontal position still lines each one up with its
        car. Two rows are available, and a label with nowhere to go is dropped
        rather than overlaid, because two overlapping names read as one
        unfamiliar word and identify neither car. The followed car is placed
        first, so it never loses its label to a crowd.
        """
        road_top = camera.to_pixels(0.0, max(LANE_OFFSETS) + LANE_WIDTH * 0.5)[1]
        rows = (road_top - 21, road_top - 38)
        taken: list[tuple[float, float, int]] = []

        ordered = sorted(actors, key=lambda actor: (actor.name != follow, actor.pose.x))
        for actor in ordered:
            name = self.font.render(
                actor.name, True, actor_color(actor.name, actor.name == follow)
            )
            # Held inside the window, so a car at the edge of the view is still
            # named rather than showing half a word running off the side.
            center_x = camera.to_pixels(actor.pose.x, 0.0)[0]
            left = min(
                max(0.0, center_x - name.get_width() * 0.5),
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
            self.surface.blit(
                self.font.render(text, True, _Color.TEXT), (12, 10 + row * 19)
            )

    def close(self) -> None:
        self._pygame.quit()
