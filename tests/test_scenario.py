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


def nav_sample(t, frame, gps, used, alarm):
    return {"t": t, "silent": 0.0, "origin": list(ORIGIN), "tracks": [],
            "nav": {"frame": frame, "gps": list(gps), "used": list(used), "alarm": alarm}}


def test_spoof_detection_is_timed_from_the_moment_the_gps_went_wrong():
    lines = truth_lines(frames=300)
    for i, line in enumerate(lines[1:]):
        line["uav"] = {"east": 10.0 * i / 30, "north": 0.0}       # flies east at 10 m/s
    truth = evaluate.Truth(lines)
    samples = [sample(0.0, 0.0, [])]        # a line without "nav" (an older log) is skipped
    for i in range(300):
        t = i / 30
        jumped = 3.0 <= t < 6.0                     # the GPS 50 m north from 3 s to 6 s
        alarm = 0.5 <= t < 0.8 or 4.0 <= t < 7.0    # one false alarm, one detection
        gps = (10.0 * t, 50.0 if jumped else 0.0)
        used = (10.0 * t + (0.3 if alarm else 0.0), 0.0) if alarm else gps
        samples.append(nav_sample(t, i, gps, used, alarm))
    result = evaluate.spoof_detection(truth, samples)
    assert result["spoof_onsets"] == 1 and result["missed"] == 0
    assert result["detection_s"] == pytest.approx([1.0], abs=0.02)
    assert result["false_alarms"] == 1
    assert result["clear_s"] == pytest.approx([1.0], abs=0.02)
    assert result["flow_error_m"]["max"] == pytest.approx(0.3, abs=1e-3)
    assert result["gps_error_max_m"] == pytest.approx(50.0, abs=1e-3)


def test_a_spoof_that_never_raises_the_alarm_is_missed():
    lines = truth_lines(frames=60)
    for i, line in enumerate(lines[1:]):
        line["uav"] = {"east": 0.0, "north": 0.0}
    samples = [nav_sample(i / 30, i, (0.0, 20.0 if i >= 30 else 0.0), (0.0, 0.0), False)
               for i in range(60)]
    result = evaluate.spoof_detection(evaluate.Truth(lines), samples)
    assert result["spoof_onsets"] == 1 and result["missed"] == 1 and result["detection_s"] == []


def test_pooled_spoof_results_add_up_over_runs():
    results = [{"nav": {"spoof_onsets": 1, "detection_s": [1.2], "missed": 0, "false_alarms": 0}},
               {"nav": {"spoof_onsets": 1, "detection_s": [], "missed": 1, "false_alarms": 2}}]
    assert run_scenario.pooled_spoof(results) == {"spoof_onsets": 2, "detection_s": [1.2],
                                                  "max_detection_s": 1.2, "missed": 1,
                                                  "false_alarms": 2}


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


def test_a_video_scenario_names_its_clip(tmp_path):
    path = tmp_path / "v.json"
    path.write_text(json.dumps({"seconds": 10, "video": "data/clip.mp4"}))
    assert run_scenario.load(path)["video"] == "data/clip.mp4"
    path.write_text(json.dumps({"seconds": 10, "video": 3}))
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


@pytest.mark.slow
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
    assert result["nav"]["spoof_onsets"] == 0 and result["nav"]["false_alarms"] == 0


@pytest.mark.slow
def test_a_short_video_scenario_end_to_end(tmp_path):
    """The UAV over the real clip with YOLO on the GPU (skipped without the GPU, the model or the
    clip): the tracks arrive, then a 50 kbps limit stops the baseline; the UAV's report is kept."""
    from pathlib import Path
    root = Path(__file__).resolve().parent.parent
    clip = root / "data" / "roundabout_tuzla.mp4"
    ort = pytest.importorskip("onnxruntime")
    if ("CUDAExecutionProvider" not in ort.get_available_providers()
            or not (root / "models" / "yolo26n-obb.onnx").is_file() or not clip.is_file()):
        pytest.skip("no GPU, model or clip")
    scenario = {"name": "video", "seconds": 8, "warmup_s": 2, "seed": 1, "video": str(clip),
                "events": [{"at_s": 6, "set": {"kbps": 50}}]}
    result = run_scenario.run(scenario, tmp_path, free_port_block())
    summary, uav = result["summary"], result["uav"]
    assert summary["bad_messages"] == 0 and summary["tracks_hz"] > 5
    # About half of the window at 50 kbps: the baseline averages well below its 30 per second.
    assert summary["baseline_fps"] < 25
    assert result["tracks"] is None and result["nav"] is None
    assert uav["yolo_provider"] == "CUDAExecutionProvider" and uav["yolo_median_ms"] < 15
    assert uav["frames"] > 200
    saved = json.loads((tmp_path / "result.json").read_text())
    assert saved["uav"] == uav


@pytest.mark.slow
def test_a_short_gps_jump_scenario_end_to_end(tmp_path):
    """The proxy moves the telemetry position 50 m east; the station raises the alarm within
    3 s from the keyframes' flow and navigates by the flow until the GPS is right again."""
    scenario = {"name": "jump", "seconds": 12, "warmup_s": 2, "seed": 3, "events": [
        {"at_s": 6, "links": [1], "set": {"spoof_bearing_deg": 90, "spoof_offset_m": 50}},
        {"at_s": 11, "links": [1], "set": {"spoof_offset_m": 0}}]}
    result = run_scenario.run(scenario, tmp_path, free_port_block())
    nav = result["nav"]
    assert nav["spoof_onsets"] == 1 and nav["missed"] == 0 and nav["false_alarms"] == 0
    assert nav["detection_s"][0] <= 3.0
    assert nav["flow_error_m"]["max"] < 2.0
    assert len(nav["clear_s"]) == 1 and result["summary"]["gps_alarms"] == 1


def test_a_slow_uav_gets_the_time_to_finish_a_long_run(tmp_path, monkeypatch):
    """The UAV flies a fixed number of frames. At 27 frames per second (a busy machine) the
    10-minute scenario's flight ends about 70 s after the station stopped; the run must wait for
    it, or the UAV is killed and its log is cut."""
    import subprocess
    from pathlib import Path

    scenario = run_scenario.load(
        Path(__file__).resolve().parent.parent / "scenarios" / "clean_10min.json")
    flight_s = (scenario["warmup_s"] + scenario["seconds"] + 3) * 30 / 27

    class FakeProcess:
        def __init__(self, args, **kwargs):
            self.uav = "aegislink.uav.main" in args
            self.gcs = "aegislink.gcs.main" in args

        def communicate(self, timeout=None):
            if self.uav and timeout < flight_s - scenario["warmup_s"] - scenario["seconds"]:
                raise subprocess.TimeoutExpired("uav", timeout)
            return ('{"telemetry_hz": 27}\n' if self.gcs else ""), None

        def poll(self):
            return 0

    class FakeControl:
        def __init__(self, port):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            pass

        def wait_ready(self):
            pass

        def configure(self, link, **settings):
            pass

    monkeypatch.setattr(run_scenario, "find_proxy", lambda: "ew_chaos_proxy")
    monkeypatch.setattr(run_scenario.subprocess, "Popen", FakeProcess)
    monkeypatch.setattr(run_scenario, "ProxyControl", FakeControl)
    monkeypatch.setattr(run_scenario.time, "sleep", lambda s: None)
    monkeypatch.setattr(evaluate.Truth, "read", lambda path: None)
    monkeypatch.setattr(evaluate, "read_jsonl", lambda path: [])
    monkeypatch.setattr(evaluate, "ghost_consistency", lambda truth, samples: {})
    monkeypatch.setattr(evaluate, "spoof_detection", lambda truth, samples: {})
    result = run_scenario.run(scenario, tmp_path)
    assert result["summary"] == {"telemetry_hz": 27}
