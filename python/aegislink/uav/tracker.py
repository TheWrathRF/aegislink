"""An IoU tracker: gives each vehicle a stable id across frames.

Every frame, each track first moves its box with the camera's motion (the UAV knows its own pose)
and then by the vehicle's own velocity in pixels per frame, then tracks and detections are paired
greedily, the pair with the largest overlap (IoU) first. A new track needs a few hits in a row before it gets an id, so a
one-frame false detection never becomes a track; a track survives some missed frames by coasting
on its velocity, so a vehicle that drives under a tree keeps its id.
"""

from dataclasses import dataclass, field

import numpy as np


def iou(a, b):
    """Intersection over union of two (x1, y1, x2, y2) boxes."""
    w = min(a[2], b[2]) - max(a[0], b[0])
    h = min(a[3], b[3]) - max(a[1], b[1])
    if w <= 0 or h <= 0:
        return 0.0
    inter = w * h
    return inter / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter)


@dataclass
class Track:
    box: np.ndarray                 # (x1, y1, x2, y2), predicted while the track is missed
    score: float
    id: int | None = None           # given once the track is confirmed
    hits: int = 1
    misses: int = 0                 # frames since the last matched detection
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(2))  # box centre, px/frame

    @property
    def confirmed(self):
        return self.id is not None


def _center(box):
    return np.array([(box[0] + box[2]) / 2, (box[1] + box[3]) / 2])


class IouTracker:
    def __init__(self, iou_min=0.2, min_hits=3, max_misses=30, velocity_smoothing=0.8):
        self.iou_min = iou_min
        self.min_hits = min_hits
        self.max_misses = max_misses
        self.velocity_smoothing = velocity_smoothing  # weight of the old velocity
        self.tracks = []
        self.next_id = 1

    def update(self, detections, camera_motion=None):
        """Takes this frame's detections and, if known, the camera's motion since the last
        frame (a 2x3 affine matrix from old to new pixels, see NadirCamera.motion_matrix).
        Returns the confirmed tracks (coasting ones too, with misses > 0)."""
        for track in self.tracks:
            if camera_motion is not None:
                self._move_with_camera(track, camera_motion)
            track.box = track.box + np.tile(track.velocity, 2)

        pairs = sorted(((iou(t.box, d.box), ti, di) for ti, t in enumerate(self.tracks)
                        for di, d in enumerate(detections)), reverse=True)
        matched_tracks, matched_detections = set(), set()
        for overlap, ti, di in pairs:
            if overlap < self.iou_min:
                break
            if ti in matched_tracks or di in matched_detections:
                continue
            matched_tracks.add(ti)
            matched_detections.add(di)
            self._hit(self.tracks[ti], detections[di])

        for ti, track in enumerate(self.tracks):
            if ti not in matched_tracks:
                track.misses += 1
        # A track that is not yet confirmed must be hit in every frame.
        self.tracks = [t for t in self.tracks
                       if t.misses <= (self.max_misses if t.confirmed else 0)]

        for di, d in enumerate(detections):
            if di not in matched_detections:
                self.tracks.append(Track(np.array(d.box, dtype=float), d.score))
        return [t for t in self.tracks if t.confirmed]

    @staticmethod
    def _move_with_camera(track, m):
        # Without this, a track's velocity mixes the vehicle's motion with the camera's, and a
        # coasting track overshoots as soon as the camera changes how it moves (stops turning).
        # The box keeps its axis-aligned shape and only follows the change of scale.
        linear = m[:, :2]
        scale = np.sqrt(abs(np.linalg.det(linear)))
        center = linear @ _center(track.box) + m[:, 2]
        half = np.array([track.box[2] - track.box[0], track.box[3] - track.box[1]]) * scale / 2
        track.box = np.concatenate([center - half, center + half])
        track.velocity = linear @ track.velocity

    def _hit(self, track, detection):
        new_box = np.array(detection.box, dtype=float)
        # The predicted box moved by velocity x (misses + 1) frames from the last real one.
        frames = track.misses + 1
        last_center = _center(track.box) - track.velocity * frames
        measured = (_center(new_box) - last_center) / frames
        if track.hits == 1:
            track.velocity = measured
        else:
            a = self.velocity_smoothing
            track.velocity = a * track.velocity + (1 - a) * measured
        track.box = new_box
        track.score = detection.score
        track.hits += 1
        track.misses = 0
        if not track.confirmed and track.hits >= self.min_hits:
            track.id = self.next_id
            self.next_id += 1
