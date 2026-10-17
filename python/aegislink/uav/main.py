"""The UAV side: fly over the synthetic world, detect and track vehicles, log the ground truth.

    python -m aegislink.uav.main                 # OpenCV preview window (q or Esc quits)
    python -m aegislink.uav.main --no-preview --seconds 60

Every run writes the ground truth (UAV pose and the vehicles near the camera, one JSON object per
frame) to logs/, and prints how well the tracks matched the true vehicles.
"""

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path

import cv2

from aegislink.uav.camera import NadirCamera
from aegislink.uav.detector import SimulatedDetector
from aegislink.uav.sim_world import World, WorldConfig
from aegislink.uav.telemetry import Flight
from aegislink.uav.tracker import IouTracker, iou

FPS = 30


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
        frame, truths = self.world.render(self.camera, pose)
        detections = self.detector.detect(truths)
        motion = self.camera.motion_matrix(self.last_pose, pose) if self.last_pose else None
        tracks = self.tracker.update(detections, motion)
        vehicles = {}
        for truth in truths:
            v = self.world.vehicles[truth.id]
            vehicles[v.id] = (*self.world.vehicle_pose(v), v.speed)
        result = FrameResult(self.index, t, frame, pose, self.flight.telemetry(t), truths,
                             vehicles, detections, tracks)
        self.world.step(1 / FPS)
        self.flight.step(1 / FPS)
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
            "vehicles": vehicles})

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


def run(seconds=60.0, seed=1, preview=True, log_path=None):
    """Runs the loop for `seconds` of simulated time (the preview paces it to real time) and
    returns the TrackScore."""
    if log_path is None:
        log_path = Path("logs") / f"ground_truth_{time.strftime('%Y%m%d_%H%M%S')}.jsonl"
    sim = UavSim(seed=seed)
    log = GroundTruthLog(log_path, sim)
    score = TrackScore()
    next_time = time.perf_counter()
    try:
        for _ in range(int(seconds * FPS)):
            r = sim.step()
            log.write(r)
            score.add(r.truths, r.tracks)
            if preview:
                cv2.imshow("AegisLink UAV", draw_overlay(r))
                next_time += 1 / FPS
                key = cv2.waitKey(max(1, int((next_time - time.perf_counter()) * 1000)))
                if key in (ord("q"), 27):
                    break
    finally:
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
    args = parser.parse_args()
    started = time.perf_counter()
    score = run(args.seconds, args.seed, not args.no_preview, args.log)
    print(f"ran {time.perf_counter() - started:.1f} s; vehicles in clear view: "
          f"{score.truth_frames} frames, covered by a track: {score.coverage:.1%}, "
          f"id switches: {score.id_switches}, reacquired after a long gap: {score.reacquired}, "
          f"false track frames: {score.false_track_frames}")


if __name__ == "__main__":
    main()
