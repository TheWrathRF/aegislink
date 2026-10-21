"""Scenario files, the track score against the ground truth, and one short scenario run through
the real proxy."""

import json

import numpy as np
import pytest

from aegislink.common import geo
from aegislink.gcs.ghost_tracks import CHI2_95
from aegislink.tools import evaluate, run_scenario
from test_proxy import free_port_block

ORIGIN = (46.0, 7.0)


def truth_lines(frames=90, fps=30):
    """Vehicle 0 drives east at 10 m/s from (0, 0); vehicle 1 stands at (50, 50)."""
    lines = [{"type": "header", "fps": fps, "origin_latlon": list(ORIGIN)}]
    for i in range(frames):
        lines.append({"frame": i, "traffic": [[10.0 * i / fps, 0.0], [50.0, 50.0]]})
    return lines


def test_truth_interpolates_between_frames_and_knows_its_bounds():
    truth = evaluate.Truth(truth_lines())
    assert truth.at(0.5)[0] == pytest.approx((5.0, 0.0))
    assert truth.at(0.51)[0] == pytest.approx((5.1, 0.0))
    assert truth.at(89 / 30)[0] == pytest.approx((29.667, 0.0), abs=1e-3)
    assert truth.at(-0.01) is None and truth.at(3.0) is None
    lat, lon = geo.enu_to_latlon(100.0, -20.0, ORIGIN)
    assert truth.offset((float(lat), float(lon))) == pytest.approx((100.0, -20.0), abs=1e-3)


def sample(t, silent, tracks, origin=ORIGIN):
    return {"t": t, "silent": silent, "origin": list(origin), "tracks": tracks}


def track(track_id, state, mean, cov, fix, tslf=0.5):
    return {"id": track_id, "state": state, "tslf": tslf, "mean": list(mean),
            "cov": list(cov), "fix": list(fix)}


def test_blackout_samples_are_scored_apart_from_the_link_up_ones():
    truth = evaluate.Truth(truth_lines())
    sigma2 = 4.0
    near = np.sqrt(CHI2_95 * sigma2) - 0.1      # just inside the 95 % ellipse
    far = np.sqrt(CHI2_95 * sigma2) + 0.1       # just outside
    cov, fix = (sigma2, 0, sigma2), (0.5, 5.0, 0.0)
    samples = [
        # Link up: a LIVE track on vehicle 0, exactly right.
        sample(0.5, 0.01, [track(1, "LIVE", (5.0, 0.0), cov, fix, 0.0)]),
        # Blackout: the same track as a GHOST, once inside and once outside, and a ghost on
        # vehicle 1 inside.
        sample(1.0, 0.5, [track(1, "GHOST", (10.0 + near, 0.0), cov, fix),
                          track(2, "GHOST", (50.0, 50.0), (1.0, 0, 1.0), (0.5, 50.5, 50.0))]),
        sample(2.0, 1.5, [track(1, "GHOST", (20.0, far), cov, fix, 1.5)]),
        # Link up again; a track whose last fix matches no vehicle.
        sample(2.5, 0.0, [track(3, "LIVE", (0.0, 0.0), (1.0, 0, 1.0), (2.5, 0.0, 30.0), 0.0)]),
    ]
    result = evaluate.ghost_consistency(truth, samples)
    assert result["blackouts"] == 1 and result["blackout_s"] == [1.5]
    assert result["blackout_samples"] == 3
    assert result["blackout_inside"] == pytest.approx(2 / 3, abs=1e-4)
    assert result["blackout_inside_by_tslf_s"] == {"0": 1.0, "1": 0.0}
    assert result["link_up_samples"] == 1 and result["link_up_inside"] == 1.0
    assert result["tracks_matched"] == 2 and result["unmatched_samples"] == 1


def test_positions_measured_from_another_origin_are_moved_into_the_map_frame():
    truth = evaluate.Truth(truth_lines())
    lat, lon = geo.enu_to_latlon(5.0, 0.0, ORIGIN)       # the station's first telemetry
    origin = (float(lat), float(lon))
    samples = [sample(1.0, 0.5, [track(1, "GHOST", (5.0, 0.0), (0.01, 0, 0.01), (0.5, 0.0, 0.0))],
                      origin)]
    result = evaluate.ghost_consistency(truth, samples)
    assert result["tracks_matched"] == 1 and result["blackout_inside"] == 1.0


def test_scenario_files_are_checked_and_completed(tmp_path):
    path = tmp_path / "x.json"
    path.write_text(json.dumps({"seconds": 10, "events": [
        {"at_s": 5, "set": {"kbps": 50}}, {"at_s": 2, "set": {"blackout_ms": 1000}}]}))
    scenario = run_scenario.load(path)
    assert scenario["name"] == "x" and scenario["seed"] == 1 and scenario["warmup_s"] == 5.0
    assert [e["at_s"] for e in scenario["events"]] == [2, 5]
    for bad in ({}, {"seconds": 0}, {"seconds": 5, "events": [{"set": {}}]},
                {"seconds": 5, "events": [{"at_s": 1, "set": 3}]}):
        path.write_text(json.dumps(bad))
        with pytest.raises(ValueError):
            run_scenario.load(path)


def test_the_shipped_scenarios_load():
    from pathlib import Path
    files = sorted(Path(__file__).resolve().parent.parent.glob("scenarios/*.json"))
    assert len(files) >= 4
    for path in files:
        run_scenario.load(path)


def test_pooled_share_weights_runs_by_their_samples():
    results = [{"tracks": {"blackout_samples": 100, "blackout_inside": 0.9}},
               {"tracks": {"blackout_samples": 300, "blackout_inside": 1.0}},
               {"tracks": {"blackout_samples": 0, "blackout_inside": None}}]
    assert run_scenario.pooled_blackout_inside(results) == {"blackout_samples": 400,
                                                            "blackout_inside": 0.975}


def test_a_short_blackout_scenario_end_to_end(tmp_path):
    """UAV, proxy and a headless ground station: a 3 s blackout on all links turns the tracks
    into ghosts, and the logs are scored against the ground truth."""
    scenario = {"name": "short", "seconds": 9, "warmup_s": 2, "seed": 2,
                "events": [{"at_s": 6, "set": {"blackout_ms": 3000}}]}
    result = run_scenario.run(scenario, tmp_path, free_port_block())
    summary, tracks = result["summary"], result["tracks"]
    assert summary["telemetry_hz"] > 15 and summary["bad_messages"] == 0
    assert tracks["blackouts"] >= 1 and 2.5 < max(tracks["blackout_s"]) < 3.5
    assert tracks["blackout_samples"] > 100 and tracks["tracks_matched"] >= 1
    # A few tracks over one blackout: a loose bound (the 90-98 % target is for long runs).
    assert tracks["blackout_inside"] > 0.6
    assert tracks["link_up_inside"] > 0.8
    saved = json.loads((tmp_path / "result.json").read_text())
    assert saved["tracks"] == tracks
    assert (tmp_path / "ground_truth.jsonl").stat().st_size > 0
