"""Ghost tracks: one Kalman filter per track id, in ground metres, that keeps predicting where a
vehicle is while its boxes do not arrive (a blackout, or the vehicle under a tree).

Tracks are kept in ground metres (east/north, the frame of `PoseHistory`), not pixels: the camera
moves, and pixel motion mixes the vehicle's motion with the camera's. A box is turned into a
ground position with the camera pose of its frame, and only boxes that matched a detection in
their frame are measurements; a coasting box is the UAV tracker's own guess.

A track is LIVE while measurements arrive, GHOST when the last one (the last fix) is more than
GHOST_AFTER_S old: the filter only predicts and its ellipse grows. It is LOST, and forgotten, when
the last fix is LOST_AFTER_S old or the ellipse has grown past MAX_SEMI_AXIS_M, and it ends at
once when a track list arrives without it (the UAV dropped the track itself). At most MAX_TRACKS
are kept: a track list can hold 255 ids, and a forged one must not fill the station with filters
that every drawn frame has to work through.
"""

import math
from dataclasses import dataclass

import numpy as np

from aegislink._core import KalmanFilter, MotionModel
from aegislink.common import config
from aegislink.uav.camera import NadirCamera

LIVE, GHOST = "LIVE", "GHOST"
GHOST_AFTER_S = 0.15
LOST_AFTER_S = 10.0
# Far more than the UAV lists at once (a few vehicles in view, plus the ones it dropped within
# the last LOST_AFTER_S).
MAX_TRACKS = 64
MAX_SEMI_AXIS_M = 100.0
# The 95 % point of the chi-square distribution with 2 degrees of freedom: a position whose
# squared Mahalanobis distance is below it lies inside the 95 % ellipse.
CHI2_95 = 5.991
PATH_S, PATH_STEP_S = 2.0, 0.1

# Process noise, tuned on the simulated traffic (cars that brake, stop and follow curved roads) so
# that the true position stays inside the 95 % ellipse in about 95 % of the frames of a 5 s
# blackout (tests/test_ghost_tracks.py). The constant-acceleration model needs less noise but its
# ellipse grows with t^5 instead of t^3.
MODELS = {"cv": (MotionModel.CONSTANT_VELOCITY, 3.0),       # m^2/s^3 (white acceleration)
          "ca": (MotionModel.CONSTANT_ACCELERATION, 2.0)}   # m^2/s^5 (white jerk)
START_SPEED_STD = 10.0      # m/s: vehicles drive at up to 13 m/s
START_ACCEL_STD = 3.0       # m/s^2

# Box edge noise of the detector: sigma = 1 px + 4 % of the box size, on each edge.
BOX_SIGMA_PX, BOX_SIGMA_FRAC = 1.0, 0.04
# Box coordinates arrive rounded to whole pixels.
ROUNDING_VAR_PX2 = 1 / 12


@dataclass
class Estimate:
    """A track as predicted for one moment."""
    id: int
    state: str              # LIVE or GHOST
    tslf_s: float           # time since the last fix
    mean: np.ndarray        # (east, north) metres
    cov: np.ndarray         # 2 x 2 position covariance, m^2
    velocity: np.ndarray    # (east, north) m/s
    path: np.ndarray        # (n, 2): the predicted positions over the next PATH_S seconds
    fix: tuple              # (t, east, north) of the filtered position at the last fix


class GhostTrack:
    def __init__(self, kf, t):
        self.kf = kf            # the filter, as of time t
        self.t = t              # seconds of UAV time (frame / FPS)
        self.fix = (t, *kf.state[:2])


def semi_axes(cov, chi2=CHI2_95):
    """The (major, minor) semi-axes in metres of the ellipse holding `chi2` of the probability."""
    eigenvalues = np.linalg.eigvalsh(np.asarray(cov, float))
    return tuple(float(math.sqrt(chi2 * max(v, 0.0))) for v in eigenvalues[::-1])


def ellipse_points(mean, cov, n=48, chi2=CHI2_95):
    """(n, 2) points on the ellipse around `mean` holding `chi2` of the probability."""
    eigenvalues, vectors = np.linalg.eigh(np.asarray(cov, float))
    radii = np.sqrt(chi2 * np.maximum(eigenvalues, 0.0))
    angles = np.linspace(0, 2 * math.pi, n, endpoint=False)
    unit = np.stack([np.cos(angles), np.sin(angles)], axis=1)
    return np.asarray(mean, float) + (unit * radii) @ vectors.T


def mahalanobis2(point, mean, cov):
    """The squared Mahalanobis distance of `point` from `mean`."""
    error = np.asarray(point, float) - np.asarray(mean, float)
    return float(error @ np.linalg.solve(np.asarray(cov, float), error))


def box_measurement(camera, pose, box):
    """((east, north), 2 x 2 covariance) of a box's centre on the ground."""
    x1, y1, x2, y2 = box
    w, h = x2 - x1, y2 - y1
    # The centre is the mean of two edges, each with its own noise and rounding.
    sigma = BOX_SIGMA_PX + BOX_SIGMA_FRAC * np.array([w, h])
    var = (sigma ** 2 + ROUNDING_VAR_PX2) / 2
    # A box cut at the image edge has its centre moved inwards, by up to half the visible size
    # (the detector needs half the vehicle in the image). The edge noise can leave a cut box a
    # few pixels inside, so "at the edge" means within two sigmas of it.
    mx, my = 2 * sigma
    if x1 <= mx or x2 >= camera.width - 1 - mx:
        var[0] += (w / 2) ** 2
    if y1 <= my or y2 >= camera.height - 1 - my:
        var[1] += (h / 2) ** 2
    m = camera.pixel_to_ground_matrix(pose)
    a = m[:, :2]
    centre = a @ np.array([(x1 + x2) / 2, (y1 + y2) / 2]) + m[:, 2]
    return centre, a @ np.diag(var) @ a.T


class GhostTracks:
    def __init__(self, model="cv", camera=None):
        self.model, self.q = MODELS[model]
        self.camera = camera or NadirCamera()
        self.tracks = {}            # track id -> GhostTrack

    def clear(self):
        self.tracks = {}

    def measure(self, track_id, t, position, r):
        """Takes a measured position (east, north) with covariance `r` at UAV time `t`. A
        measurement not newer than the track's last one is ignored; returns whether it was
        used."""
        r = (float(r[0][0]), float(r[0][1]), float(r[1][1]))
        track = self.tracks.get(track_id)
        if track is None:
            if len(self.tracks) >= MAX_TRACKS:
                # A real track is measured every frame, so the oldest fix is the least useful;
                # one fixed at this very moment is never replaced (the new id is refused).
                oldest = min(self.tracks, key=lambda i: self.tracks[i].fix[0])
                if not self.tracks[oldest].fix[0] < t:
                    return False
                del self.tracks[oldest]
            kf = KalmanFilter(self.model, self.q)
            kf.init(float(position[0]), float(position[1]), r, START_SPEED_STD, START_ACCEL_STD)
            self.tracks[track_id] = GhostTrack(kf, t)
            return True
        if not t > track.t:
            return False
        kf = track.kf.predicted(t - track.t)
        if kf.update(float(position[0]), float(position[1]), r) < 0:
            return False
        track.kf, track.t = kf, t
        track.fix = (t, *kf.state[:2])
        return True

    def add_boxes(self, frame, boxes, pose):
        """Takes the matched boxes of one frame's track list, seen from camera `pose`."""
        t = frame / config.FPS
        for box in boxes:
            if box.seen:
                position, r = box_measurement(self.camera, pose, box.box)
                self.measure(box.id, t, position, r)

    def keep_only(self, track_ids):
        """Ends the tracks the UAV no longer lists: it has dropped them itself (the vehicle left
        the view or went undetected too long), so they are not waiting for the link."""
        self.tracks = {i: track for i, track in self.tracks.items() if i in track_ids}

    def prune(self, t):
        """Forgets the tracks that are LOST at time `t`."""
        for track_id in [i for i, track in self.tracks.items() if self._lost(track, t)]:
            del self.tracks[track_id]

    def _lost(self, track, t):
        if t - track.fix[0] > LOST_AFTER_S:
            return True
        kf = track.kf.predicted(max(0.0, t - track.t))
        return semi_axes(_cov2(kf))[0] > MAX_SEMI_AXIS_M

    def estimates(self, t, path=True):
        """Every track that is not lost, predicted to UAV time `t` (never before its last fix)."""
        result = []
        for track_id, track in self.tracks.items():
            kf = track.kf.predicted(max(0.0, t - track.t))
            cov = _cov2(kf)
            tslf = max(0.0, t - track.fix[0])
            if tslf > LOST_AFTER_S or semi_axes(cov)[0] > MAX_SEMI_AXIS_M:
                continue
            state = np.array(kf.state)
            points = np.empty((0, 2))
            if path:
                # The predicted mean moves as x + v dt (+ a dt^2 / 2): no filter copies needed.
                dt = np.arange(1, round(PATH_S / PATH_STEP_S) + 1)[:, None] * PATH_STEP_S
                points = state[:2] + state[2:4] * dt
                if len(state) == 6:
                    points = points + state[4:6] * dt ** 2 / 2
            result.append(Estimate(track_id, LIVE if tslf <= GHOST_AFTER_S else GHOST, tslf,
                                   state[:2], cov, state[2:4], points, track.fix))
        return result


def _cov2(kf):
    xx, xy, yy = kf.position_covariance
    return np.array([[xx, xy], [xy, yy]])
