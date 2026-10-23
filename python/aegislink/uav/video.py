"""A real video clip as the UAV's camera, with telemetry made to fit it.

A downloaded clip has no flight log, so the camera's path is measured from the clip itself: each
frame is registered to the one before (corner features, Lucas-Kanade flow, a RANSAC similarity
transform, so moving vehicles count as outliers), and the chained transforms give the position,
heading and altitude relative to the first frame. Only the first altitude is a guess (`alt0_m`,
from the size of a car in the image); the rest follows from the images. This holds for a camera
that looks straight down, as `NadirCamera` does.
"""

import math
from pathlib import Path

import cv2
import numpy as np

from aegislink.common import geo
from aegislink.common.config import FPS
from aegislink.uav.camera import Pose
from aegislink.uav.telemetry import Telemetry

REGISTER_SCALE = 0.5   # registration runs on a half-size gray image
MIN_INLIERS = 20


def _gray(frame):
    small = cv2.resize(frame, None, fx=REGISTER_SCALE, fy=REGISTER_SCALE,
                       interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)


def register(previous_gray, gray):
    """3x3 similarity taking pixels of `gray` to pixels of `previous_gray` (both half size), or
    None when too few features agree."""
    points = cv2.goodFeaturesToTrack(gray, maxCorners=500, qualityLevel=0.01, minDistance=8)
    if points is None or len(points) < MIN_INLIERS:
        return None
    moved, status, _ = cv2.calcOpticalFlowPyrLK(gray, previous_gray, points, None,
                                                winSize=(21, 21), maxLevel=3)
    found = status.ravel() == 1
    if found.sum() < MIN_INLIERS:
        return None
    matrix, inliers = cv2.estimateAffinePartial2D(points[found], moved[found], method=cv2.RANSAC,
                                                  ransacReprojThreshold=1.0)
    if matrix is None or inliers.sum() < MIN_INLIERS:
        return None
    return np.vstack([matrix, [0, 0, 1]])


def _pose_from_matrix(camera, pixel_to_ground):
    # A nadir camera's pixel -> ground map is m x [[cos h, -sin h], [-sin h, -cos h]] plus a
    # shift (see NadirCamera._rotation), so the heading and the scale can be read back from it.
    a = pixel_to_ground[:2, :2]
    heading = math.degrees(math.atan2(-a[0, 1], a[0, 0])) % 360
    metres_per_pixel = math.hypot(a[0, 0], a[0, 1])
    east, north = pixel_to_ground[:2] @ np.array([camera.cx, camera.cy, 1.0])
    return Pose(float(east), float(north), metres_per_pixel * camera.focal_px, heading)


def estimate_path(frames, camera, start):
    """The camera pose of every frame (BGR images of the camera's size), given the first one.

    A frame that cannot be registered (blank, blurred) is assumed to move like the frame before
    it, so the path goes on without a jump."""
    scale = np.diag([REGISTER_SCALE, REGISTER_SCALE, 1.0])
    first_to_ground = np.vstack([camera.pixel_to_ground_matrix(start), [0, 0, 1]])
    to_first = np.eye(3)   # pixels of the current frame -> pixels of the first frame
    step = np.eye(3)
    path = []
    previous = None
    for frame in frames:
        gray = _gray(frame)
        if previous is not None:
            small = register(previous, gray)
            if small is not None:
                step = np.linalg.inv(scale) @ small @ scale
            to_first = to_first @ step
        path.append(_pose_from_matrix(camera, first_to_ground @ to_first))
        previous = gray
    return path


class VideoClip:
    """The frames of a clip at the camera's size with the camera path measured from them.

    The first frame is at the local origin, heading north, at `alt0_m`. Past its end the clip
    plays backwards and then forwards again, so the images and the telemetry never jump. The
    frames are kept as JPEG in memory (about 0.27 MB each instead of 2.8 MB)."""

    def __init__(self, path, camera, alt0_m, origin_latlon=(46.0, 7.0)):
        path = Path(path)
        if not path.is_file():
            raise FileNotFoundError(path)
        if not (math.isfinite(alt0_m) and alt0_m > 0):
            raise ValueError(f"the first altitude must be above 0 m, not {alt0_m}")
        capture = cv2.VideoCapture(str(path))
        self.fps = capture.get(cv2.CAP_PROP_FPS) or FPS
        self.origin_latlon = origin_latlon
        self.jpegs = []

        def frames():
            while True:
                ok, frame = capture.read()
                if not ok:
                    return
                frame = cv2.resize(frame, (camera.width, camera.height),
                                   interpolation=cv2.INTER_AREA)
                self.jpegs.append(cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 90])[1])
                yield frame

        try:
            self.poses = estimate_path(frames(), camera, Pose(0.0, 0.0, alt0_m, 0.0))
        finally:
            capture.release()
        if len(self.poses) < 2:
            raise ValueError(f"{path}: fewer than two frames could be read")

    def __len__(self):
        return len(self.poses)

    def index(self, i):
        """Clip frame shown at loop frame `i` (the loop runs at FPS, the clip at its own rate)."""
        k = int(i * self.fps / FPS)
        period = 2 * (len(self) - 1)
        k %= period
        return k if k < len(self) else period - k

    def frame(self, i):
        k = self.index(i)
        return cv2.imdecode(self.jpegs[k], cv2.IMREAD_COLOR), self.poses[k]

    def telemetry(self, i, t):
        k = self.index(i)
        pose = self.poses[k]
        before, after = self.poses[max(k - 1, 0)], self.poses[min(k + 1, len(self) - 1)]
        frames = min(k + 1, len(self) - 1) - max(k - 1, 0)
        speed = math.hypot(after.east - before.east, after.north - before.north) * self.fps / frames
        lat, lon = geo.enu_to_latlon(pose.east, pose.north, self.origin_latlon)
        # The gimbal hides the airframe's tilt and nothing in the clip shows it: level flight.
        return Telemetry(t, float(lat), float(lon), pose.alt, speed, pose.heading_deg, 0.0, 0.0)
