"""The UAV side: fly over the synthetic world, detect and track vehicles, log the ground truth.

    python -m aegislink.uav.main                 # OpenCV preview window (q or Esc quits)
    python -m aegislink.uav.main --no-preview --seconds 60
    python -m aegislink.uav.main --send          # also stream to the GCS through the proxy
    python -m aegislink.uav.main --video data/clip.mp4   # a real clip with YOLO on the GPU

Every run over the synthetic world writes the ground truth (UAV pose, the vehicles near the camera and where every vehicle
is, one JSON object per frame) to logs/, and prints how well the tracks matched the true
vehicles. With --send it runs in real time and streams the foveated and the baseline video (see
aegislink.common.config).
"""

import argparse
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from aegislink._core import MsgType
from aegislink.common import config
from aegislink.common.config import FPS
from aegislink.uav.camera import NadirCamera
from aegislink.uav.detector import SimulatedDetector
from aegislink.uav.sim_world import World, WorldConfig
from aegislink.uav.streamer import Streamer
from aegislink.uav.telemetry import Flight
from aegislink.uav.tracker import IouTracker, iou
from aegislink.uav.video import VideoClip
from aegislink.uav.yolo_detector import DEFAULT_MODEL


@dataclass
class FrameResult:
    index: int
    t: float
    frame: object        # BGR image
    pose: object         # camera Pose the frame was taken from
    telemetry: object
    truths: list
    vehicles: dict       # id -> (east, north, heading, speed) of the vehicles in `truths`
    detections: list
    tracks: list         # confirmed tracks, coasting ones included
    traffic: list        # (east, north) of every vehicle, by vehicle id


class UavSim:
    """Steps the world, the flight, the camera, the detector and the tracker together."""

    def __init__(self, seed=1, world_config=None, follow_id=0):
        self.seed = seed
        self.world = World(world_config or WorldConfig(seed=seed))
        self.camera = NadirCamera()
        self.flight = Flight(self.world, follow_id=follow_id)
        self.detector = SimulatedDetector(self.camera.width, self.camera.height, seed=seed + 1)
        self.tracker = IouTracker()
        self.index = 0
        self.last_pose = None

    def step(self):
        """Renders and processes the current frame, then advances the world by one frame."""
        t = self.index / FPS
        pose = self.flight.pose
        poses = self.world.vehicle_poses()
        frame, truths = self.world.render(self.camera, pose, poses)
        detections = self.detector.detect(truths)
        motion = self.camera.motion_matrix(self.last_pose, pose) if self.last_pose else None
        tracks = self.tracker.update(detections, motion)
        vehicles = {t.id: (*poses[t.id], self.world.vehicles[t.id].speed) for t in truths}
        traffic = [p[:2] for p in poses]
        result = FrameResult(self.index, t, frame, pose, self.flight.telemetry(t), truths,
                             vehicles, detections, tracks, traffic)
        self.world.step(1 / FPS)
        self.flight.step(1 / FPS)
        self.last_pose = pose
        self.index += 1
        return result


class VideoUav:
    """Plays a real clip as the camera (see uav/video.py) and finds the vehicles with YOLO. A clip
    has no ground truth, so its frames carry no truths and nothing is scored."""

    def __init__(self, clip, detector):
        self.clip = clip
        self.camera = NadirCamera()
        self.detector = detector
        self.tracker = IouTracker()
        self.index = 0
        self.last_pose = None
        self.detect_seconds = []

    def step(self):
        t = self.index / FPS
        frame, pose = self.clip.frame(self.index)
        started = time.perf_counter()
        detections = self.detector.detect(frame)
        self.detect_seconds.append(time.perf_counter() - started)
        motion = self.camera.motion_matrix(self.last_pose, pose) if self.last_pose else None
        tracks = self.tracker.update(detections, motion)
        result = FrameResult(self.index, t, frame, pose, self.clip.telemetry(self.index, t), [],
                             {}, detections, tracks, [])
        self.last_pose = pose
        self.index += 1
        return result


class TrackScore:
    """How well the tracks follow the true vehicles.

    A truth counts when the vehicle is fully in the image and at least 90 % visible. It is
    covered when a track seen this frame overlaps it (IoU >= 0.3). A vehicle covered by a
    different track id than last time is an id switch if it was covered within the last
    `max_gap` frames (the tracker could have kept the id), and a reacquisition otherwise (it
    was hidden or out of the image for longer than a track survives). A false track frame is a
    track seen this frame that overlaps no vehicle at all.
    """

    MIN_IOU = 0.3

    def __init__(self, max_gap=IouTracker().max_misses):
        self.max_gap = max_gap
        self.frame = 0
        self.truth_frames = 0
        self.covered = 0
        self.id_switches = 0
        self.reacquired = 0
        self.false_track_frames = 0
        self.last_track_id = {}
        self.last_covered = {}

    @property
    def coverage(self):
        return self.covered / self.truth_frames if self.truth_frames else 0.0

    def add(self, truths, tracks):
        seen = [t for t in tracks if t.misses == 0]
        pairs = sorted(((iou(tr.box, t.box), ti, ri) for ti, t in enumerate(seen)
                        for ri, tr in enumerate(truths) if tr.box), reverse=True)
        match = {}  # truth index -> track
        used = set()
        for overlap, ti, ri in pairs:
            if overlap < self.MIN_IOU:
                break
            if ti not in used and ri not in match:
                used.add(ti)
                match[ri] = seen[ti]
        self.false_track_frames += len(seen) - len(used)

        for ri, truth in enumerate(truths):
            if not truth.box or truth.in_frame < 1.0 or truth.visible < 0.9:
                continue
            self.truth_frames += 1
            if ri not in match:
                continue
            self.covered += 1
            track_id = match[ri].id
            last = self.last_track_id.get(truth.id)
            if last is not None and last != track_id:
                if self.frame - self.last_covered[truth.id] <= self.max_gap:
                    self.id_switches += 1
                else:
                    self.reacquired += 1
            self.last_track_id[truth.id] = track_id
            self.last_covered[truth.id] = self.frame
        self.frame += 1


class GroundTruthLog:
    """One JSON object per line: a header, then one object per frame."""

    def __init__(self, path, sim):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = path.open("w")
        cfg, cam = sim.world.config, sim.camera
        self._write({"type": "header", "seed": sim.seed, "fps": FPS, "size_m": cfg.size_m,
                     "origin_latlon": list(cfg.origin_latlon), "follow_id": sim.flight.follow.id,
                     "camera": {"width": cam.width, "height": cam.height,
                                "hfov_deg": cam.hfov_deg, "focal_px": cam.focal_px}})

    def _write(self, obj):
        self.file.write(json.dumps(obj, separators=(",", ":")) + "\n")

    def write(self, r):
        pose, tel = r.pose, r.telemetry
        vehicles = []
        for truth in r.truths:
            east, north, heading, speed = r.vehicles[truth.id]
            vehicles.append({
                "id": truth.id, "east": round(east, 3), "north": round(north, 3),
                "heading": round(heading, 2), "speed": round(speed, 3),
                "visible": round(truth.visible, 3), "in_frame": round(truth.in_frame, 3),
                "box": [round(x, 1) for x in truth.box] if truth.box else None})
        self._write({
            "frame": r.index, "t": round(r.t, 4),
            "uav": {"east": round(pose.east, 3), "north": round(pose.north, 3),
                    "alt": round(pose.alt, 3), "heading": round(pose.heading_deg, 3),
                    "pitch": round(tel.pitch_deg, 3), "roll": round(tel.roll_deg, 3),
                    "speed": round(tel.speed_mps, 3), "lat": round(tel.lat, 8),
                    "lon": round(tel.lon, 8)},
            "vehicles": vehicles,
            "traffic": [[round(east, 2), round(north, 2)] for east, north in r.traffic]})

    def close(self):
        self.file.close()


def draw_overlay(r):
    """The camera image with track boxes and ids, and a line of telemetry."""
    image = r.frame.copy()
    for track in r.tracks:
        x1, y1, x2, y2 = (int(round(v)) for v in track.box)
        seen = track.misses == 0
        color = (0, 255, 0) if seen else (0, 200, 255)  # green when seen, amber when coasting
        cv2.rectangle(image, (x1 - 2, y1 - 2), (x2 + 2, y2 + 2), color, 2 if seen else 1)
        cv2.putText(image, str(track.id), (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1,
                    cv2.LINE_AA)
    tel = r.telemetry
    text = (f"t {r.t:6.1f} s  alt {tel.alt_m:5.1f} m  speed {tel.speed_mps:4.1f} m/s  "
            f"hdg {tel.heading_deg:5.1f}  pitch {tel.pitch_deg:5.1f}  roll {tel.roll_deg:5.1f}  "
            f"lat {tel.lat:.6f}  lon {tel.lon:.6f}  tracks {len(r.tracks)}")
    image[:32] //= 3  # a dark strip, so the text reads on any background
    cv2.putText(image, text, (10, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                cv2.LINE_AA)
    return image


def run(seconds=60.0, seed=1, preview=True, log_path=None, streamer=None, sim=None):
    """Runs the loop for `seconds` of simulated time and returns the TrackScore. The preview and
    the streamer pace it to real time. `sim` is the UavSim to run (built from `seed` if None) or
    a VideoUav, which has no ground truth: then nothing is logged or scored and None is
    returned."""
    log = score = None
    if sim is None:
        sim = UavSim(seed=seed)
    if isinstance(sim, UavSim):
        if log_path is None:
            log_path = Path("logs") / f"ground_truth_{time.strftime('%Y%m%d_%H%M%S')}.jsonl"
        log = GroundTruthLog(log_path, sim)
        score = TrackScore()
    next_time = time.perf_counter()
    try:
        for _ in range(int(seconds * FPS)):
            captured_us = time.monotonic_ns() // 1000
            r = sim.step()
            if log:
                log.write(r)
                score.add(r.truths, r.tracks)
            if streamer:
                streamer.send(r, time.monotonic(), captured_us)
            next_time += 1 / FPS
            if preview:
                cv2.imshow("AegisLink UAV", draw_overlay(r))
                key = cv2.waitKey(max(1, int((next_time - time.perf_counter()) * 1000)))
                if key in (ord("q"), 27):
                    break
            elif streamer:
                time.sleep(max(0.0, next_time - time.perf_counter()))
    finally:
        if log:
            log.close()
        if preview:
            cv2.destroyAllWindows()
    return score


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seconds", type=float, default=60.0, help="simulated time to run")
    parser.add_argument("--seed", type=int, default=1, help="world and detector seed")
    parser.add_argument("--no-preview", action="store_true", help="run as fast as possible")
    parser.add_argument("--log", type=Path, help="ground-truth log path (default: logs/...)")
    parser.add_argument("--send", action="store_true", help="stream to the GCS in real time")
    parser.add_argument("--direct", action="store_true", help="send to the GCS, not the proxy")
    parser.add_argument("--no-baseline", action="store_true", help="send only the foveated stream")
    parser.add_argument("--fovea-port", type=int, help="where the foveated stream goes")
    parser.add_argument("--baseline-port", type=int, help="where the baseline stream goes")
    parser.add_argument("--report-port", type=int, default=config.REPORTS_TO_UAV,
                        help="where receiver reports arrive")
    parser.add_argument("--video", type=Path,
                        help="fly over this clip (looking straight down) with YOLO, not the world")
    parser.add_argument("--alt0", type=float, default=55.0,
                        help="altitude in metres at the clip's first frame")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL, help="YOLO ONNX model")
    args = parser.parse_args()
    if args.video and not (math.isfinite(args.alt0) and args.alt0 > 0):
        parser.error("--alt0 must be above 0 m")
    # Built before the clock starts (the world takes over a second), so the time and the sent
    # kbps printed at the end are the run's own.
    sim = None if args.video else UavSim(seed=args.seed)
    if args.video:
        # Imported here: onnxruntime is only needed with --video.
        from aegislink.uav.yolo_detector import YoloDetector

        started = time.perf_counter()
        clip = VideoClip(args.video, NadirCamera(), args.alt0)
        detector = YoloDetector(args.model)
        sim = VideoUav(clip, detector)
        # Flushed: a scenario run waits for this line before it starts its clock.
        print(f"clip: {len(clip)} frames, camera path measured in "
              f"{time.perf_counter() - started:.1f} s; YOLO on {detector.provider}", flush=True)
    streamer = None
    if args.send:
        fovea = args.fovea_port or (config.FOVEA_TO_GCS if args.direct else config.FOVEA_TO_PROXY)
        baseline = args.baseline_port or (config.BASELINE_TO_GCS if args.direct
                                          else config.BASELINE_TO_PROXY)
        streamer = Streamer(fovea, baseline, args.report_port, baseline=not args.no_baseline)
    started = time.perf_counter()
    try:
        score = run(args.seconds, args.seed, not args.no_preview, args.log, streamer, sim)
    finally:
        if streamer:
            streamer.close()
    elapsed = time.perf_counter() - started
    if score:
        print(f"ran {elapsed:.1f} s; vehicles in clear view: "
              f"{score.truth_frames} frames, covered by a track: {score.coverage:.1%}, "
              f"id switches: {score.id_switches}, reacquired after a long gap: {score.reacquired}, "
              f"false track frames: {score.false_track_frames}")
    else:
        ms = 1000 * np.array(sim.detect_seconds)
        print(json.dumps({"seconds": round(elapsed, 1), "frames": sim.index,
                          "fps": round(sim.index / elapsed, 1),
                          "yolo_provider": sim.detector.provider,
                          "yolo_median_ms": round(float(np.median(ms)), 2) if len(ms) else None,
                          "yolo_p95_ms": round(float(np.percentile(ms, 95)), 2) if len(ms)
                          else None}))
    if streamer:
        sent = streamer.scheduler.sent
        print(f"sent: foveated {streamer.sent_bytes['fovea'] * 8 / elapsed / 1000:.1f} kbps "
              f"({sent[MsgType.TRACKS]} track lists, {sent[MsgType.ROI_PATCH]} patches, "
              f"{sent[MsgType.BG_KEYFRAME]} keyframes), baseline "
              f"{streamer.sent_bytes['baseline'] * 8 / elapsed / 1000:.1f} kbps; "
              f"final budget {streamer.budget.bps / 1000:.1f} kbps")


if __name__ == "__main__":
    main()
