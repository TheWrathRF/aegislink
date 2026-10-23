"""Runs a scripted EW scenario through the real proxy and scores what the ground station got.

    python -m aegislink.tools.run_scenario scenarios/blackout.json
    python -m aegislink.tools.run_scenario scenarios/blackout.json --seeds 1 2 3

A scenario is a JSON file:

    {"name": "blackout", "seconds": 60, "warmup_s": 5, "seed": 1,
     "events": [{"at_s": 15, "set": {"blackout_ms": 5000}}]}

It starts the proxy, the ground station (headless, with its summary and track log) and the UAV
(with its ground-truth log). Each event sets the given keys (the proxy's control keys) on all
three links, like the EW deck, at `at_s` seconds after the UAV started; "links": [1, 2] limits
it. The ground station measures for `seconds` after `warmup_s`. The logs and a result.json with
the station's summary, the track score and the GPS check's score (`evaluate`) go to a new folder
under logs/scenarios. With --seeds the scenario runs once per seed (world, traffic and proxy),
each in its own sub-folder, and the blackout samples and GPS checks of all runs are pooled.

With "video": "data/clip.mp4" the UAV flies over that clip with YOLO instead (see uav/video.py).
The event clock then starts once the UAV has measured the clip, nothing is scored (a clip has
no ground truth), and result.json keeps the UAV's own report instead: YOLO's time per frame.
"""

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

from aegislink.common import config
from aegislink.common.proxy_control import ProxyControl
from aegislink.tools import evaluate
from aegislink.tools.run_demo import find_proxy, ports

ALL_LINKS = (config.FOVEA_LINK, config.BASELINE_LINK, config.REPORTS_LINK)


def load(path):
    scenario = json.loads(Path(path).read_text(encoding="utf-8"))
    scenario.setdefault("name", Path(path).stem)
    scenario.setdefault("warmup_s", 5.0)
    scenario.setdefault("seed", 1)
    scenario.setdefault("events", [])
    if not scenario.get("seconds", 0) > 0:
        raise ValueError("a scenario needs 'seconds' > 0")
    for event in scenario["events"]:
        if "at_s" not in event or not isinstance(event.get("set"), dict):
            raise ValueError(f"an event needs 'at_s' and 'set': {event}")
    if "video" in scenario and not isinstance(scenario["video"], str):
        raise ValueError("'video' is the path of a clip")
    scenario["events"].sort(key=lambda e: e["at_s"])
    return scenario


def run(scenario, out_dir, base_port=config.FOVEA_TO_PROXY):
    """Runs one scenario; returns the result (also written to out_dir/result.json)."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    truth_log, track_log = out_dir / "ground_truth.jsonl", out_dir / "tracks.jsonl"
    p = ports(base_port)
    proxy_path = find_proxy()
    if proxy_path is None:
        sys.exit("ew_chaos_proxy not found: run 'python -m pip install -e .' first")
    # The UAV flies on a little after the station has stopped measuring, then ends by itself so
    # that its log is complete.
    uav_seconds = scenario["warmup_s"] + scenario["seconds"] + 3
    processes = []
    try:
        processes.append(subprocess.Popen(
            [proxy_path, "--link", f"{p['fovea_to_proxy']}:{p['fovea_to_gcs']}",
             "--link", f"{p['baseline_to_proxy']}:{p['baseline_to_gcs']}",
             "--link", f"{p['reports_to_proxy']}:{p['reports_to_uav']}",
             "--control", str(p["proxy_control"]), "--seed", str(scenario["seed"])],
            stdout=subprocess.DEVNULL))
        with ProxyControl(p["proxy_control"]) as control:
            control.wait_ready()

            def start_gcs():
                gcs = subprocess.Popen(
                    [sys.executable, "-m", "aegislink.gcs.main", "--no-view", "--summary",
                     "--seconds", str(scenario["seconds"]), "--warmup", str(scenario["warmup_s"]),
                     "--motion-model", scenario.get("motion_model", "cv"),
                     "--track-log", str(track_log),
                     "--fovea-port", str(p["fovea_to_gcs"]),
                     "--baseline-port", str(p["baseline_to_gcs"]),
                     "--report-port", str(p["reports_to_proxy"])],
                    stdout=subprocess.PIPE, text=True)
                processes.append(gcs)
                return gcs

            video = ["--video", scenario["video"]] if "video" in scenario else []
            if not video:
                gcs = start_gcs()
            uav = subprocess.Popen(
                [sys.executable, "-m", "aegislink.uav.main", "--no-preview", "--send",
                 "--seed", str(scenario["seed"]), "--seconds", str(uav_seconds),
                 "--log", str(truth_log), "--fovea-port", str(p["fovea_to_proxy"]),
                 "--baseline-port", str(p["baseline_to_proxy"]),
                 "--report-port", str(p["reports_to_uav"]), *video],
                stdout=subprocess.PIPE if video else subprocess.DEVNULL, text=True)
            processes.append(uav)
            if video:
                # The UAV first measures the clip's camera path, which takes seconds (about 7 s
                # for 26 s of clip), and the station gives up when nothing arrives for 15 s: it
                # starts, and the clock with it, once the UAV says it is ready.
                for line in uav.stdout:
                    if line.startswith("clip:"):
                        break
                gcs = start_gcs()
            started = time.monotonic()
            for event in scenario["events"]:
                time.sleep(max(0.0, started + event["at_s"] - time.monotonic()))
                for link in event.get("links", ALL_LINKS):
                    control.configure(link, **event["set"])
        summary_text, _ = gcs.communicate(timeout=uav_seconds + 30)
        uav_text, _ = uav.communicate(timeout=60)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
    lines = summary_text.strip().splitlines()
    summary = json.loads(lines[-1]) if lines else {"error": "no summary"}
    if video:
        # A clip has no ground truth to score against; the UAV's own report (YOLO's time per
        # frame) is kept instead.
        reports = [line for line in (uav_text or "").splitlines() if line.startswith("{")]
        result = {"scenario": scenario, "summary": summary, "tracks": None, "nav": None,
                  "uav": json.loads(reports[-1]) if reports else {"error": "no UAV report"}}
    else:
        truth, samples = evaluate.Truth.read(truth_log), evaluate.read_jsonl(track_log)
        result = {"scenario": scenario, "summary": summary,
                  "tracks": evaluate.ghost_consistency(truth, samples),
                  "nav": evaluate.spoof_detection(truth, samples)}
    (out_dir / "result.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return result


def pooled_blackout_inside(results):
    """The share of blackout samples inside the ellipse over several runs."""
    samples = sum(r["tracks"]["blackout_samples"] for r in results)
    inside = sum(r["tracks"]["blackout_samples"] * (r["tracks"]["blackout_inside"] or 0)
                 for r in results)
    return {"blackout_samples": samples,
            "blackout_inside": round(inside / samples, 4) if samples else None}


def pooled_spoof(results):
    """The GPS check over several runs: every detection time, the slowest, misses, false alarms."""
    detections = [d for r in results for d in r["nav"]["detection_s"]]
    return {"spoof_onsets": sum(r["nav"]["spoof_onsets"] for r in results),
            "detection_s": detections,
            "max_detection_s": max(detections, default=None),
            "missed": sum(r["nav"]["missed"] for r in results),
            "false_alarms": sum(r["nav"]["false_alarms"] for r in results)}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("scenario", type=Path, help="the scenario's JSON file")
    parser.add_argument("--out", type=Path, help="folder for the logs and result.json "
                                                 "(default: logs/scenarios/<name>_<time>)")
    parser.add_argument("--base-port", type=int, default=config.FOVEA_TO_PROXY,
                        help="first of the six link ports; control is this + 100")
    parser.add_argument("--seeds", type=int, nargs="+",
                        help="run once per seed instead of with the scenario's own seed")
    args = parser.parse_args()
    scenario = load(args.scenario)
    if "video" in scenario and args.seeds:
        parser.error("--seeds varies the synthetic world; a clip is always the same")
    out = args.out or (Path("logs") / "scenarios"
                       / f"{scenario['name']}_{time.strftime('%Y%m%d_%H%M%S')}")
    if not args.seeds:
        result = run(scenario, out, args.base_port)
        printed = {key: result[key] for key in ("summary", "tracks", "nav", "uav") if key in result}
        print(json.dumps({"out": str(out), **printed}))
        return
    results = []
    for seed in args.seeds:
        result = run(dict(scenario, seed=seed), out / f"seed_{seed}", args.base_port)
        results.append(result)
        print(json.dumps({"seed": seed, "summary": result["summary"], "tracks": result["tracks"],
                          "nav": result["nav"]}), flush=True)
    print(json.dumps({"out": str(out), "seeds": args.seeds,
                      "pooled": pooled_blackout_inside(results), "pooled_nav": pooled_spoof(results)}))


if __name__ == "__main__":
    main()
