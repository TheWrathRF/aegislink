"""GPS spoofing detection at the ground station, and navigation without the GPS.

Anyone on the link can change the position in the telemetry and recompute the CRC. The background
keyframes show how the camera really moved: optical flow between two keyframes, with the
altitude and heading they carry, gives the ground distance the UAV flew between them, without
the GPS. Chained from keyframe to keyframe (visual odometry), it is compared with the GPS:

- After a keyframe whose GPS and flow agree, the chain is tied to the GPS there again.
- A keyframe where they disagree (squared Mahalanobis distance d2 above 9.21, the 99 % point of
  the chi-square distribution with 2 degrees of freedom) is suspect; the chain keeps running from
  the last keyframe that agreed, so a jump stays visible in every later keyframe and a drift
  grows.
- 3 suspect keyframes in a row raise the alarm. While it is on, the GPS is not used: the
  positions come from the flow (`FlowPath`). 5 keyframes in a row that agree with the GPS
  again clear it.

A pair of keyframes too far apart in time, or one the flow cannot follow, decides nothing: the
chain is carried on by the last velocity with a growing uncertainty, so a jump made while
keyframes are sparse (on a slow link) is still seen at the next pair the flow can follow.
"""

import bisect
import math
from dataclasses import dataclass

import cv2
import numpy as np

from aegislink.common import config
from aegislink.uav.camera import NadirCamera, Pose
from aegislink.uav.encoder import KEYFRAME_SIZE

# The camera model at the keyframe's size: a keyframe pixel lies on the same ground point as the
# full-size pixel it was averaged from.
KEYFRAME_CAMERA = NadirCamera(*KEYFRAME_SIZE)
CHI2_99 = 9.21
ALARM_AFTER = 3
CLEAR_AFTER = 5
MAX_PAIR_S = 3.0
MAX_CORNERS = 200
MIN_INLIERS = 30
# Points that moved this far from the median are on moving vehicles (or tracked wrongly).
INLIER_M = 1.0
# Per axis and keyframe pair: the flow's spread underestimates errors shared by all points.
FLOW_FLOOR_M = 0.1
GPS_SIGMA_M = 0.05
# When a pair cannot be followed, the chain is carried on by the last velocity with this much
# more uncertainty per second.
COAST_SIGMA_MPS = 3.0
LK_PARAMS = dict(winSize=(21, 21), maxLevel=3, flags=cv2.OPTFLOW_USE_INITIAL_FLOW,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))


@dataclass
class Flow:
    displacement: np.ndarray    # (east, north) metres the camera moved between the keyframes
    cov: np.ndarray             # 2 x 2
    inliers: int
    tracked: int


def flow_displacement(old_image, old_attitude, new_image, new_attitude, guess=(0.0, 0.0)):
    """How far the camera moved from the old keyframe to the new one, from optical flow: Flow,
    or None when too few points could be followed. Attitudes are (alt, heading_deg); `guess` is
    the expected displacement, which places the search windows."""
    corners = cv2.goodFeaturesToTrack(old_image, MAX_CORNERS, 0.01, 8, blockSize=7)
    if corners is None or len(corners) < MIN_INLIERS:
        return None
    old_px = corners.reshape(-1, 2).astype(np.float64)
    # Ground offsets from each camera's own position: the same ground point is
    # old position + old offset = new position + new offset.
    old_offset = KEYFRAME_CAMERA.pixel_to_ground(Pose(0.0, 0.0, *old_attitude), old_px)
    predicted = KEYFRAME_CAMERA.ground_to_pixel(Pose(guess[0], guess[1], *new_attitude),
                                                old_offset)
    new_px, status, _ = cv2.calcOpticalFlowPyrLK(old_image, new_image,
                                                 old_px.astype(np.float32),
                                                 predicted.astype(np.float32), **LK_PARAMS)
    w, h = KEYFRAME_SIZE
    ok = ((status.ravel() == 1) & (new_px[:, 0] >= 0) & (new_px[:, 0] <= w - 1)
          & (new_px[:, 1] >= 0) & (new_px[:, 1] <= h - 1))
    if ok.sum() < MIN_INLIERS:
        return None
    new_offset = KEYFRAME_CAMERA.pixel_to_ground(Pose(0.0, 0.0, *new_attitude),
                                                 new_px[ok].astype(np.float64))
    moved = old_offset[ok] - new_offset
    inliers = moved[np.linalg.norm(moved - np.median(moved, axis=0), axis=1) < INLIER_M]
    if len(inliers) < MIN_INLIERS:
        return None
    cov = np.cov(inliers.T) / len(inliers) + FLOW_FLOOR_M ** 2 * np.eye(2)
    return Flow(inliers.mean(axis=0), cov, len(inliers), int(ok.sum()))


class FlowPath:
    """The UAV's positions from the flow alone, from the last keyframe whose GPS was trusted:
    known at each keyframe, in between on a straight line, after the last one carried on with
    its velocity. It never changes once made, so a view can keep it."""

    def __init__(self, frames, positions, velocity):
        self.frames = frames            # increasing keyframe numbers, the first one trusted
        self.positions = positions      # (east, north) at each
        self.velocity = velocity        # (east, north) m/s

    def position(self, frame):
        """(east, north) at `frame`, or None before the first keyframe."""
        if frame < self.frames[0]:
            return None
        i = bisect.bisect_right(self.frames, frame) - 1
        (e0, n0), f0 = self.positions[i], self.frames[i]
        if i + 1 < len(self.frames):
            (e1, n1), f1 = self.positions[i + 1], self.frames[i + 1]
            w = (frame - f0) / (f1 - f0)
            return e0 + w * (e1 - e0), n0 + w * (n1 - n0)
        dt = (frame - f0) / config.FPS
        return e0 + self.velocity[0] * dt, n0 + self.velocity[1] * dt


@dataclass
class _Keyframe:
    frame: int
    image: np.ndarray
    attitude: tuple     # (alt, heading_deg)
    gps: np.ndarray     # the telemetry's (east, north) at its frame


class SpoofDetector:
    """Feed it every new keyframe with the GPS position of its frame (see the module notes)."""

    KEEP_KEYFRAMES = 64     # of the flow path; more than the GCS's 10 s of poses

    def __init__(self):
        self.clear()

    def clear(self):
        self.last = None            # _Keyframe
        self.frames = []            # keyframes since the chain was last tied to the GPS
        self.positions = []         # the chain's position at each
        self.cov = np.zeros((2, 2))     # its uncertainty at the last one
        self.velocity = None        # from the last pair the flow followed, m/s
        self.followed = False       # whether the flow followed the last pair
        self.alarm = False
        self.suspect = 0            # keyframes in a row that disagree with the GPS
        self.agree = 0              # and that agree
        self.d2 = None              # of the last compared keyframe
        self.gps_velocity = None    # over the last pair, m/s
        self.alarms = 0             # alarms raised
        self.undecided = 0          # pairs that decided nothing

    @property
    def fallback(self):
        """The FlowPath to navigate by while the alarm is on, else None."""
        if not self.alarm:
            return None
        return FlowPath(list(self.frames), list(self.positions), tuple(self.velocity))

    def _tie(self, frame, gps):
        self.frames, self.positions = [frame], [tuple(gps)]
        self.cov = np.zeros((2, 2))

    def _extend(self, frame, position, cov):
        self.frames.append(frame)
        self.positions.append(tuple(position))
        self.cov = cov
        excess = len(self.frames) - self.KEEP_KEYFRAMES
        if excess > 0:
            del self.frames[:excess]
            del self.positions[:excess]

    def add_keyframe(self, frame, image, alt, heading_deg, gps):
        """`gps`: the telemetry's (east, north) at `frame`, or None if not known."""
        if gps is None or (self.last is not None and frame == self.last.frame):
            return
        if self.last is not None and frame < self.last.frame:     # the UAV started again
            self.clear()
        gps = np.array(gps, float)
        new = _Keyframe(frame, image, (alt, heading_deg), gps)
        old, self.last = self.last, new
        if old is None:
            self._tie(frame, gps)
            return
        dt = (frame - old.frame) / config.FPS
        last_position = np.array(self.positions[-1])
        # The guess only places the search windows: without one, a pair a few seconds apart is
        # too far for the flow to find. A wrong one makes the pair undecided, never an alarm.
        # After a pair the flow could not follow, its last velocity may be long out of date, and
        # the GPS's over that pair is the better guess.
        recent = self.velocity if self.followed else None
        expected = recent or self.gps_velocity or self.velocity or (0.0, 0.0)
        self.gps_velocity = tuple((gps - old.gps) / dt)
        flow = None
        if dt <= MAX_PAIR_S:
            flow = flow_displacement(old.image, old.attitude, image, new.attitude,
                                     tuple(np.array(expected) * dt))
        self.followed = flow is not None
        if flow is None:
            self.undecided += 1
            if self.velocity is None:
                # Nothing to carry the chain on with yet.
                self.suspect = self.agree = 0
                self._tie(frame, gps)
            else:
                # Not tied to the GPS here: a jump made during the gap would become the new
                # reference, and every later pair would agree with it.
                coast = (COAST_SIGMA_MPS * dt) ** 2 * np.eye(2)
                self._extend(frame, last_position + np.array(self.velocity) * dt, self.cov + coast)
            return
        self.velocity = tuple(flow.displacement / dt)
        position = last_position + flow.displacement
        cov = self.cov + flow.cov
        residual = gps - position
        s = cov + GPS_SIGMA_M ** 2 * np.eye(2)
        self.d2 = float(residual @ np.linalg.solve(s, residual))
        if self.d2 > CHI2_99:
            self.suspect, self.agree = self.suspect + 1, 0
        else:
            self.suspect, self.agree = 0, self.agree + 1
        if self.alarm and self.agree >= CLEAR_AFTER:
            self.alarm = False
        elif not self.alarm and self.suspect >= ALARM_AFTER:
            self.alarm = True
            self.alarms += 1
        if not self.alarm and self.suspect == 0:
            self._tie(frame, gps)
        else:
            self._extend(frame, position, cov)

    def flow_speed(self):
        return None if self.velocity is None else math.hypot(*self.velocity)

    def gps_speed(self):
        return None if self.gps_velocity is None else math.hypot(*self.gps_velocity)
