"""Ghost tracks: boxes to ground measurements, LIVE / GHOST / LOST, the ellipse and the path, and
the filter's consistency on the simulated traffic."""

import math

import numpy as np
import pytest

from aegislink.common import config
from aegislink.common.messages import TrackBox
from aegislink.gcs import ghost_tracks as gt
from aegislink.gcs.ghost_tracks import GHOST, LIVE, GhostTracks
from aegislink.uav.camera import NadirCamera, Pose
from aegislink.uav.sim_world import World, WorldConfig

CAMERA = NadirCamera()


def box_at(pixel, w=30, h=16):
    u, v = pixel
    return (u - w / 2, v - h / 2, u + w / 2, v + h / 2)


@pytest.mark.parametrize("heading", [0.0, 37.0, 90.0, 210.0])
def test_box_centre_lands_on_its_ground_point(heading):
    pose = Pose(120.0, -40.0, 150.0, heading)
    ground = np.array([135.5, -22.0])
    pixel = CAMERA.ground_to_pixel(pose, ground)
    position, _ = gt.box_measurement(CAMERA, pose, box_at(pixel))
    assert np.allclose(position, ground, atol=1e-9)


def test_box_noise_is_turned_and_scaled_into_ground_metres():
    mpp = CAMERA.metres_per_pixel(150.0)
    wide = box_at((640, 360), w=60, h=10)
    _, r_north_up = gt.box_measurement(CAMERA, Pose(0, 0, 150.0, 0.0), wide)
    # Heading 0: image right is east, so the wide box's larger noise is along east.
    var_u = ((1 + 0.04 * 60) ** 2 + 1 / 12) / 2
    var_v = ((1 + 0.04 * 10) ** 2 + 1 / 12) / 2
    assert r_north_up == pytest.approx(np.diag([var_u, var_v]) * mpp ** 2)
    _, r_east_up = gt.box_measurement(CAMERA, Pose(0, 0, 150.0, 90.0), wide)
    assert r_east_up == pytest.approx(np.diag([var_v, var_u]) * mpp ** 2, abs=1e-12)
    _, r_high = gt.box_measurement(CAMERA, Pose(0, 0, 300.0, 0.0), wide)
    assert r_high == pytest.approx(r_north_up * 4)


def test_a_box_cut_at_the_image_edge_is_less_certain_along_that_axis():
    pose = Pose(0, 0, 150.0, 0.0)
    _, inside = gt.box_measurement(CAMERA, pose, (100, 100, 130, 116))
    _, cut = gt.box_measurement(CAMERA, pose, (0, 100, 20, 116))
    mpp = CAMERA.metres_per_pixel(150.0)
    assert cut[0, 0] > inside[0, 0] + (20 / 2) ** 2 * mpp ** 2 * 0.99
    assert cut[1, 1] == pytest.approx(inside[1, 1])
    # Edge noise (sigma 1 + 0.04 x 30 = 2.2 px) can leave a cut box a few pixels inside.
    _, near_edge = gt.box_measurement(CAMERA, pose, (4, 100, 34, 116))
    _, clear = gt.box_measurement(CAMERA, pose, (5, 100, 35, 116))
    assert near_edge[0, 0] > 10 * clear[0, 0]
    _, bottom = gt.box_measurement(CAMERA, pose, (100, 700, 130, 717))
    assert bottom[1, 1] > 10 * inside[1, 1] and bottom[0, 0] == pytest.approx(inside[0, 0])


def r_of(sigma):
    return np.eye(2) * sigma ** 2


def test_live_then_ghost_then_lost():
    tracks = GhostTracks()
    for i in range(30):
        tracks.measure(7, i / 30, (10.0 + i / 3, 5.0), r_of(0.2))
    last = 29 / 30
    (e,) = tracks.estimates(last + 0.1)
    assert e.id == 7 and e.state == LIVE and e.tslf_s == pytest.approx(0.1)
    (e,) = tracks.estimates(last + 0.2)
    assert e.state == GHOST
    major_1s = gt.semi_axes(tracks.estimates(last + 1)[0].cov)[0]
    major_5s = gt.semi_axes(tracks.estimates(last + 5)[0].cov)[0]
    assert major_5s > 5 * major_1s
    assert tracks.estimates(last + 9.9)
    assert tracks.estimates(last + 10.1) == []
    tracks.prune(last + 9.9)
    assert 7 in tracks.tracks
    tracks.prune(last + 10.1)
    assert tracks.tracks == {}


def test_lost_when_the_ellipse_is_too_large():
    # The constant-acceleration ellipse grows past the limit before the time limit runs out.
    tracks = GhostTracks("ca")
    for i in range(30):
        tracks.measure(1, i / 30, (0.0, 0.0), r_of(0.2))
    t_lost = next(t for t in np.arange(1.0, 10.0, 0.1) if not tracks.estimates(29 / 30 + t))
    assert 5.0 < t_lost < 10.0
    tracks.prune(29 / 30 + t_lost)
    assert tracks.tracks == {}


def test_old_and_repeated_measurements_are_ignored():
    tracks = GhostTracks()
    assert tracks.measure(1, 1.0, (0.0, 0.0), r_of(0.2))
    assert tracks.measure(1, 2.0, (1.0, 0.0), r_of(0.2))
    assert not tracks.measure(1, 2.0, (50.0, 0.0), r_of(0.2))
    assert not tracks.measure(1, 1.5, (50.0, 0.0), r_of(0.2))
    assert tracks.tracks[1].fix[1] < 2.0


def test_only_matched_boxes_are_measurements():
    tracks = GhostTracks()
    pose = Pose(0.0, 0.0, 150.0, 0.0)
    tracks.add_boxes(30, [TrackBox(1, box_at((CAMERA.cx, CAMERA.cy)), True),
                          TrackBox(2, box_at((300, 200)), False)], pose)
    assert set(tracks.tracks) == {1}
    assert tracks.tracks[1].t == pytest.approx(1.0)
    (e,) = tracks.estimates(1.0)
    assert np.allclose(e.mean, (0.0, 0.0), atol=1e-6)


def test_path_runs_ahead_along_the_velocity():
    tracks = GhostTracks()
    for i in range(90):
        tracks.measure(3, i / 30, (12.0 * i / 30, -4.0 * i / 30), r_of(0.05))
    (e,) = tracks.estimates(89 / 30)
    assert e.velocity == pytest.approx((12.0, -4.0), abs=0.3)
    assert len(e.path) == round(gt.PATH_S / gt.PATH_STEP_S)
    assert e.path[-1] == pytest.approx(e.mean + 2.0 * e.velocity, abs=0.05)
    assert tracks.estimates(89 / 30, path=False)[0].path.shape == (0, 2)


def test_ellipse_points_sit_on_the_95_percent_contour():
    cov = np.array([[9.0, 2.5], [2.5, 4.0]])
    mean = np.array([3.0, -1.0])
    points = gt.ellipse_points(mean, cov)
    assert [gt.mahalanobis2(p, mean, cov) for p in points] == pytest.approx([gt.CHI2_95] * 48)
    major, minor = gt.semi_axes(cov)
    assert np.max(np.linalg.norm(points - mean, axis=1)) == pytest.approx(major, rel=1e-2)
    assert major > minor


@pytest.fixture(scope="module")
def traffic():
    """90 s of every vehicle's true position (frames x vehicles x 2) on one simulated map."""
    world = World(WorldConfig(seed=4))
    positions = []
    for _ in range(90 * config.FPS):
        positions.append([world.vehicle_pose(v)[:2] for v in world.vehicles])
        world.step(1 / config.FPS)
    return np.array(positions)


@pytest.mark.parametrize("model", ["cv", "ca"])
def test_truth_stays_inside_the_ellipse_in_90_to_98_percent_of_blackout_frames(traffic, model):
    """Every vehicle is measured at 30 Hz (0.25 m noise, 5 % misses) except during 5 s
    blackouts, one every 10 s; in each blackout frame the true position is checked against the
    95 % ellipse of the prediction. A consistent filter holds it in about 95 % of them."""
    rng = np.random.default_rng(11)
    sigma = 0.25
    measured = traffic + rng.normal(0, sigma, traffic.shape)
    tracks = GhostTracks(model)
    inside = []
    for i in range(len(traffic)):
        t = i / config.FPS
        if t >= 10 and t % 10 >= 5:
            for e in tracks.estimates(t, path=False):
                inside.append(gt.mahalanobis2(traffic[i, e.id], e.mean, e.cov) <= gt.CHI2_95)
            continue
        for v in range(traffic.shape[1]):
            if rng.random() < 0.95:
                tracks.measure(v, t, measured[i, v], r_of(sigma))
    share = float(np.mean(inside))
    assert len(inside) > 7 * 150 * 60
    assert 0.90 <= share <= 0.98, share


def test_at_most_max_tracks_and_the_oldest_fix_makes_room():
    tracks = GhostTracks()
    for i in range(gt.MAX_TRACKS):
        assert tracks.measure(i, 1.0 + i / 1000, (float(i), 0.0), r_of(0.2))
    # Full: a new id replaces the track with the oldest fix (id 0)...
    assert tracks.measure(500, 2.0, (0.0, 0.0), r_of(0.2))
    assert 0 not in tracks.tracks and 500 in tracks.tracks
    assert len(tracks.tracks) == gt.MAX_TRACKS
    # ...but never one fixed at the same moment: then the new id is refused.
    for i in range(1, gt.MAX_TRACKS):
        tracks.measure(i, 2.0, (float(i), 0.0), r_of(0.2))
    assert not tracks.measure(501, 2.0, (0.0, 0.0), r_of(0.2))
    assert 501 not in tracks.tracks and len(tracks.tracks) == gt.MAX_TRACKS


@pytest.mark.parametrize("model", ["cv", "ca"])
def test_path_is_the_filters_own_prediction(model):
    tracks = GhostTracks(model)
    for i in range(60):
        tracks.measure(1, i / 30, (8.0 * i / 30 + 0.5 * (i / 30) ** 2, 3.0), r_of(0.05))
    (e,) = tracks.estimates(2.2)
    kf = tracks.tracks[1].kf
    expected = [kf.predicted(2.2 - tracks.tracks[1].t + dt).state[:2]
                for dt in np.arange(1, 21) * gt.PATH_STEP_S]
    assert e.path == pytest.approx(np.array(expected), abs=1e-9)
