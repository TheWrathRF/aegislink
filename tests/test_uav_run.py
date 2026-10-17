"""The UAV frame loop end to end: world, flight, detector, tracker, ground-truth log."""

import json

import numpy as np
import pytest

from aegislink.common import geo
from aegislink.uav.camera import NadirCamera, Pose
from aegislink.uav.main import TrackScore, UavSim, run
from aegislink.uav.sim_world import Truth
from aegislink.uav.tracker import Track


@pytest.fixture(scope="module")
def result(tmp_path_factory):
    log_path = tmp_path_factory.mktemp("logs") / "gt.jsonl"
    score = run(seconds=40, seed=2, preview=False, log_path=log_path)
    return score, log_path


def test_ground_truth_log(result):
    _, log_path = result
    lines = [json.loads(line) for line in log_path.read_text().splitlines()]
    header, frames = lines[0], lines[1:]
    assert header["type"] == "header"
    assert header["seed"] == 2 and header["fps"] == 30
    assert header["camera"]["width"] == 1280 and header["camera"]["height"] == 720
    assert len(frames) == 40 * 30
    assert [f["frame"] for f in frames] == list(range(len(frames)))
    first = frames[0]
    assert set(first["uav"]) == {"east", "north", "alt", "heading", "pitch", "roll", "speed",
                                 "lat", "lon"}
    vehicle = first["vehicles"][0]
    assert set(vehicle) == {"id", "east", "north", "heading", "speed", "visible", "in_frame", "box"}
    followed = sum(any(v["id"] == 0 and v["box"] for v in f["vehicles"]) for f in frames)
    assert followed / len(frames) > 0.95


def test_log_entries_of_a_frame_agree_with_each_other(result):
    # The pose, the vehicle positions and the boxes of one line must all be from the same
    # instant: a vehicle's position seen from the logged pose is the centre of its logged box.
    _, log_path = result
    lines = [json.loads(line) for line in log_path.read_text().splitlines()]
    header, frames = lines[0], lines[1:]
    camera = NadirCamera(header["camera"]["width"], header["camera"]["height"],
                         header["camera"]["hfov_deg"])
    checked = 0
    for f in frames[::10]:
        uav = f["uav"]
        pose = Pose(uav["east"], uav["north"], uav["alt"], uav["heading"])
        east, north = geo.latlon_to_enu(uav["lat"], uav["lon"], header["origin_latlon"])
        assert abs(east - uav["east"]) < 0.01 and abs(north - uav["north"]) < 0.01
        for v in f["vehicles"]:
            if v["in_frame"] == 1.0:
                x1, y1, x2, y2 = v["box"]
                u, w = camera.ground_to_pixel(pose, [v["east"], v["north"]])
                assert abs(u - (x1 + x2) / 2) < 1.0 and abs(w - (y1 + y2) / 2) < 1.0
                checked += 1
    assert checked > 100


def test_tracks_follow_the_true_vehicles_with_stable_ids(result):
    score, _ = result
    assert score.truth_frames > 2000
    assert score.coverage > 0.9
    assert score.id_switches <= 1
    assert score.false_track_frames / score.truth_frames < 0.01


def test_score_counts_a_new_id_after_a_long_gap_as_reacquired():
    score = TrackScore(max_gap=30)
    truth = Truth(id=7, box=(100.0, 100.0, 130.0, 115.0), in_frame=1.0, visible=1.0)
    score.add([truth], [Track(np.array(truth.box), 0.9, id=1)])
    for _ in range(31):
        score.add([], [])
    score.add([truth], [Track(np.array(truth.box), 0.9, id=2)])
    assert score.id_switches == 0 and score.reacquired == 1


def test_score_counts_an_id_switch_and_a_false_track():
    score = TrackScore()
    truth = Truth(id=7, box=(100.0, 100.0, 130.0, 115.0), in_frame=1.0, visible=1.0)
    track_a = Track(np.array(truth.box), 0.9, id=1)
    track_b = Track(np.array(truth.box), 0.9, id=2)
    ghost = Track(np.array([500.0, 500.0, 530.0, 515.0]), 0.9, id=3)
    score.add([truth], [track_a])
    score.add([truth], [track_a, ghost])
    score.add([truth], [track_b])
    score.add([truth], [])
    assert score.truth_frames == 4
    assert score.coverage == pytest.approx(0.75)
    assert score.id_switches == 1
    assert score.false_track_frames == 1


def test_same_seed_same_run():
    a, b = UavSim(seed=9), UavSim(seed=9)
    for _ in range(60):
        ra, rb = a.step(), b.step()
    assert np.array_equal(ra.frame, rb.frame)
    assert [t.box.tolist() for t in ra.tracks] == [t.box.tolist() for t in rb.tracks]
