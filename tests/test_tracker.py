"""The simulated detector and the IoU tracker."""

import numpy as np
import pytest

from aegislink.uav.camera import NadirCamera, Pose
from aegislink.uav.detector import Detection, SimulatedDetector
from aegislink.uav.sim_world import Truth
from aegislink.uav.tracker import IouTracker, iou

W, H = 1280, 720


def test_iou():
    assert iou((0, 0, 10, 10), (0, 0, 10, 10)) == pytest.approx(1.0)
    assert iou((0, 0, 10, 10), (5, 0, 15, 10)) == pytest.approx(50 / 150)
    assert iou((0, 0, 10, 10), (20, 20, 30, 30)) == 0.0


def test_detector_finds_visible_vehicles_with_noisy_boxes():
    detector = SimulatedDetector(W, H, seed=1, false_per_frame=0.0)
    truth = Truth(id=0, box=(600.0, 300.0, 640.0, 320.0), in_frame=1.0, visible=1.0)
    found = [detector.detect([truth]) for _ in range(10_000)]
    rate = np.mean([len(d) for d in found])
    assert rate == pytest.approx(1 - detector.miss_rate, abs=0.01)
    errors = np.array([np.subtract(d[0].box, truth.box) for d in found if d])
    assert np.abs(errors.mean(axis=0)).max() < 0.1
    # Each edge: sigma = box_sigma_px + box_sigma_frac x box size (40 x 20 px).
    expected = detector.box_sigma_px + detector.box_sigma_frac * np.array([40, 20, 40, 20])
    np.testing.assert_allclose(errors.std(axis=0), expected, rtol=0.05)


def test_detector_misses_hidden_and_cut_off_vehicles():
    detector = SimulatedDetector(W, H, seed=2, false_per_frame=0.0)
    hidden = Truth(1, (100.0, 100.0, 140.0, 120.0), in_frame=1.0, visible=0.2)
    cut = Truth(2, (0.0, 100.0, 12.0, 120.0), in_frame=0.3, visible=1.0)
    half = Truth(3, (300.0, 100.0, 340.0, 120.0), in_frame=1.0, visible=0.65)
    counts = {1: 0, 2: 0, 3: 0}
    for _ in range(5000):
        for d in detector.detect([hidden, cut, half]):
            x = d.box[0]
            counts[2 if x < 50 else 1 if x < 200 else 3] += 1
    assert counts[1] == 0 and counts[2] == 0
    # Half way between min_visible (0.4) and 0.9: about half the clear-view rate.
    assert counts[3] / 5000 == pytest.approx(0.5 * (1 - detector.miss_rate), abs=0.03)


def test_detector_false_positives():
    detector = SimulatedDetector(W, H, seed=3, false_per_frame=0.05)
    frames = [detector.detect([]) for _ in range(20_000)]
    assert np.mean([len(f) for f in frames]) == pytest.approx(0.05, abs=0.005)
    for f in frames:
        for d in f:
            x1, y1, x2, y2 = d.box
            assert 0 <= x1 < x2 <= W - 1 and 0 <= y1 < y2 <= H - 1
            assert d.score < 0.6


def moving_box(frame, x0, y0, vx, vy=0.0, w=30.0, h=14.0):
    x, y = x0 + vx * frame, y0 + vy * frame
    return (x, y, x + w, y + h)


def run(tracker, frames):
    """Feeds lists of boxes, returns the confirmed, currently seen track ids per frame."""
    ids = []
    for boxes in frames:
        tracks = tracker.update([Detection(b, 0.9) for b in boxes])
        ids.append({t.id: t.box for t in tracks if t.misses == 0})
    return ids


def test_track_is_confirmed_after_three_hits_and_keeps_its_id_through_gaps():
    gaps = set(range(20, 30)) | set(range(50, 62))  # 10 and 12 missed frames
    frames = [[] if f in gaps else [moving_box(f, 100, 300, 4.0)] for f in range(100)]
    ids = run(IouTracker(), frames)
    assert ids[0] == {} and ids[1] == {}
    assert len(ids[2]) == 1
    seen = {i for frame in ids for i in frame}
    assert seen == {1}
    assert all(ids[f] == {} for f in gaps)


def test_crossing_vehicles_keep_their_ids():
    frames = [[moving_box(f, 100, 300, 5.0), moving_box(f, 700, 306, -5.0)] for f in range(120)]
    ids = run(IouTracker(), frames)
    first = None
    for frame in ids[3:]:
        assert len(frame) == 2
        left_to_right = min(frame, key=lambda i: frame[i][1])  # the one at y = 300
        first = first or left_to_right
        assert left_to_right == first


def test_single_frame_false_positives_never_become_tracks():
    rng = np.random.default_rng(4)
    frames = []
    for _ in range(300):
        x, y = rng.uniform(0, 1200), rng.uniform(0, 700)
        frames.append([(x, y, x + 25, y + 12)])
    ids = run(IouTracker(), frames)
    assert all(frame == {} for frame in ids)


def test_track_is_dropped_after_max_misses():
    tracker = IouTracker(max_misses=30)
    frames = [[moving_box(f, 100, 300, 0.0)] for f in range(10)] + [[]] * 31 \
        + [[moving_box(0, 100, 300, 0.0)] for _ in range(5)]
    ids = run(tracker, frames)
    assert {i for frame in ids for i in frame} == {1, 2}
    assert tracker.update([]) and len(tracker.tracks) == 1


def test_camera_motion_keeps_ids_when_the_camera_stops_turning():
    # A parked vehicle seen by a camera that yaws at 25 deg/s and then stops; the detector
    # misses it for 12 frames around the stop, so velocity alone overshoots by about 26 px.
    camera = NadirCamera()
    vehicle = np.array([40.0, 0.0])
    poses = [Pose(0.0, 0.0, 150.0, 25.0 / 30 * min(f, 60)) for f in range(90)]
    gap = set(range(54, 66))

    def run_with(motion):
        tracker = IouTracker()
        ids = set()
        for f, pose in enumerate(poses):
            u, v = camera.ground_to_pixel(pose, vehicle)
            boxes = [] if f in gap else [Detection((u - 15, v - 7, u + 15, v + 7), 0.9)]
            m = camera.motion_matrix(poses[f - 1], pose) if motion and f else None
            ids |= {t.id for t in tracker.update(boxes, m) if t.misses == 0}
        return ids

    assert len(run_with(motion=False)) == 2
    assert run_with(motion=True) == {1}
