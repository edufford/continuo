"""The renderer's arithmetic, which needs no window.

`render` imports pygame lazily, inside the one class that opens a display, so
the projection and the body geometry can be checked anywhere. That split is
deliberate: it is what lets CI test this without installing a display driver.
"""

from __future__ import annotations

import math

import pytest

from continuo_viz.render import Camera, actor_color, body_corners


def camera(center_x: float = 0.0, scale: float = 7.0) -> Camera:
    return Camera(center_x=center_x, center_y=0.0, scale=scale, width=1400, height=560)


def test_the_camera_center_lands_in_the_middle_of_the_window():
    assert camera(center_x=500.0).to_pixels(500.0, 0.0) == (700.0, 280.0)


def test_the_world_centers_below_the_hud_rather_than_on_the_whole_window():
    # Otherwise the HUD takes the space above the road and the picture sits
    # high with a dead band underneath it.
    reserved = Camera(
        center_x=0.0, center_y=0.0, scale=7.0, width=1400, height=560, top_y=60
    )

    assert reserved.to_pixels(0.0, 0.0)[1] == 310.0


def test_y_is_drawn_upward():
    # A plan view has +y up the screen, but pixels count downward, so getting
    # this backwards silently mirrors every lane change.
    _, above = camera().to_pixels(0.0, 3.5)
    _, below = camera().to_pixels(0.0, -3.5)

    assert above < below


def test_moving_the_camera_moves_the_world_the_other_way():
    ahead = camera(center_x=0.0).to_pixels(100.0, 0.0)[0]
    alongside = camera(center_x=100.0).to_pixels(100.0, 0.0)[0]

    assert ahead > alongside


def test_a_body_pointing_along_x_is_longer_than_it_is_wide():
    corners = body_corners(0.0, 0.0, yaw=0.0, length=4.5, width=1.8)
    xs = [x for x, _ in corners]
    ys = [y for _, y in corners]

    assert max(xs) - min(xs) == pytest.approx(4.5)
    assert max(ys) - min(ys) == pytest.approx(1.8)


def test_a_body_turned_a_quarter_turn_swaps_its_extents():
    # The check that the rotation is applied at all, and in the right sense.
    corners = body_corners(0.0, 0.0, yaw=math.pi / 2, length=4.5, width=1.8)
    xs = [x for x, _ in corners]
    ys = [y for _, y in corners]

    assert max(xs) - min(xs) == pytest.approx(1.8)
    assert max(ys) - min(ys) == pytest.approx(4.5)


def test_a_body_is_centered_on_its_pose():
    corners = body_corners(120.0, -3.5, yaw=0.4, length=4.5, width=1.8)
    xs = [x for x, _ in corners]
    ys = [y for _, y in corners]

    assert sum(xs) / 4 == pytest.approx(120.0)
    assert sum(ys) / 4 == pytest.approx(-3.5)


def test_the_first_two_corners_are_the_leading_edge():
    # The renderer marks corners[0:2] as the front, so heading is readable.
    # If the corner order ever changes, cars gain a stripe across the boot.
    front_left, front_right, *_ = body_corners(0.0, 0.0, yaw=0.0, length=4.5, width=1.8)

    assert front_left[0] == pytest.approx(2.25)
    assert front_right[0] == pytest.approx(2.25)


def test_an_actor_keeps_its_color_across_frames_and_sources():
    # Derived from the name, not from arrival order, so a replay and a live
    # view of the same run agree, and so does a viewer that attached late.
    assert actor_color("traffic7", focused=False) == actor_color(
        "traffic7", focused=False
    )
    assert actor_color("traffic7", focused=False) != actor_color(
        "traffic8", focused=False
    )


def test_colors_are_the_same_in_every_process():
    # Pinned values, which is the only way a single-process test can catch a
    # per-process hash. Python seeds `hash()` for strings on each start, so a
    # replay and a live view are two processes and would have disagreed. These
    # constants failing is the signal that a seeded hash crept back in.
    assert actor_color("ego", focused=False) == (242, 109, 233)
    assert actor_color("traffic7", focused=False) == (109, 242, 233)


def test_the_followed_actor_is_highlighted():
    assert actor_color("ego", focused=True) != actor_color("ego", focused=False)
