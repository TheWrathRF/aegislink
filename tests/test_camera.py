"""Nadir camera geometry and the local lat/lon frame."""

import math

import numpy as np
import pytest

from aegislink.common import geo
from aegislink.uav.camera import NadirCamera, Pose


def test_latlon_round_trip():
    origin = (47.3769, 8.5417)
    east = np.array([0.0, 120.5, -800.0, 999.9])
    north = np.array([0.0, -40.25, 650.0, -999.9])
    lat, lon = geo.enu_to_latlon(east, north, origin)
    back_east, back_north = geo.latlon_to_enu(lat, lon, origin)
    np.testing.assert_allclose(back_east, east, atol=1e-6)
    np.testing.assert_allclose(back_north, north, atol=1e-6)


def test_one_degree_of_latitude_is_about_111_km():
    lat, _ = geo.enu_to_latlon(0.0, 111_319.5, (0.0, 0.0))
    assert lat == pytest.approx(1.0, abs=1e-3)


def test_focal_length_and_footprint():
    camera = NadirCamera(width=1280, height=720, hfov_deg=60.0)
    assert camera.focal_px == pytest.approx(640 / math.tan(math.radians(30)))
    # The image covers 2 * alt * tan(hfov / 2) metres across.
    assert camera.metres_per_pixel(150.0) * 1280 == pytest.approx(
        2 * 150 * math.tan(math.radians(30)), rel=1e-9)


@pytest.mark.parametrize("heading", [0.0, 37.0, 90.0, 180.0, 271.5])
def test_pixel_to_ground_round_trip(heading):
    camera = NadirCamera()
    pose = Pose(east=310.0, north=-55.0, alt=140.0, heading_deg=heading)
    rng = np.random.default_rng(1)
    pixels = rng.uniform([0, 0], [camera.width, camera.height], size=(100, 2))
    ground = camera.pixel_to_ground(pose, pixels)
    np.testing.assert_allclose(camera.ground_to_pixel(pose, ground), pixels, atol=1e-9)


def test_image_up_points_along_the_heading():
    camera = NadirCamera()
    pose = Pose(east=0.0, north=0.0, alt=100.0, heading_deg=90.0)  # flying east
    center = camera.pixel_to_ground(pose, [camera.cx, camera.cy])
    above = camera.pixel_to_ground(pose, [camera.cx, camera.cy - 100])
    right = camera.pixel_to_ground(pose, [camera.cx + 100, camera.cy])
    s = 100 * camera.metres_per_pixel(100.0)
    np.testing.assert_allclose(center, [0.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(above, [s, 0.0], atol=1e-9)    # ahead = east
    np.testing.assert_allclose(right, [0.0, -s], atol=1e-9)   # right of east = south


def test_affine_matrix_matches_pixel_to_ground():
    camera = NadirCamera()
    pose = Pose(east=12.0, north=34.0, alt=120.0, heading_deg=200.0)
    matrix = camera.pixel_to_ground_matrix(pose)
    pixels = np.array([[0.0, 0.0], [640.0, 100.0], [1279.0, 719.0]])
    by_matrix = pixels @ matrix[:, :2].T + matrix[:, 2]
    np.testing.assert_allclose(by_matrix, camera.pixel_to_ground(pose, pixels), atol=1e-9)


def test_motion_matrix_maps_old_pixels_to_new_pixels():
    camera = NadirCamera()
    old = Pose(east=0.0, north=0.0, alt=150.0, heading_deg=10.0)
    new = Pose(east=3.0, north=-1.5, alt=148.0, heading_deg=11.0)
    ground = np.array([[5.0, 7.0], [-30.0, 12.0], [44.0, -20.0]])
    m = camera.motion_matrix(old, new)
    moved = camera.ground_to_pixel(old, ground) @ m[:, :2].T + m[:, 2]
    np.testing.assert_allclose(moved, camera.ground_to_pixel(new, ground), atol=1e-9)
