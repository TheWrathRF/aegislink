"""A gimbal-stabilised camera that looks straight down (nadir) over flat ground.

With a nadir camera and flat ground, the map from image pixels to ground metres is one scale
(altitude / focal length), one rotation (the heading) and one shift (the UAV position). Image up
points along the heading.
"""

import math
from dataclasses import dataclass

import numpy as np

from aegislink.common import config


@dataclass
class Pose:
    east: float         # metres, local frame
    north: float
    alt: float          # metres above the ground
    heading_deg: float  # 0 = north, 90 = east (clockwise, as on a compass)


class NadirCamera:
    def __init__(self, width=config.FRAME_WIDTH, height=config.FRAME_HEIGHT, hfov_deg=60.0):
        self.width = width
        self.height = height
        self.hfov_deg = hfov_deg
        self.focal_px = (width / 2) / math.tan(math.radians(hfov_deg / 2))
        # Pixel centres sit on integer coordinates (as in OpenCV), so the middle of the image is
        # between the two centre pixels.
        self.cx = (width - 1) / 2
        self.cy = (height - 1) / 2

    def metres_per_pixel(self, alt):
        return alt / self.focal_px

    def _rotation(self, pose):
        # Columns: the ground direction of image right and of image down, in (east, north).
        h = math.radians(pose.heading_deg)
        c, s = math.cos(h), math.sin(h)
        return np.array([[c, -s], [-s, -c]])

    def pixel_to_ground_matrix(self, pose):
        """2x3 affine matrix taking (u, v) pixels to (east, north) metres."""
        a = self._rotation(pose) * self.metres_per_pixel(pose.alt)
        shift = np.array([pose.east, pose.north]) - a @ np.array([self.cx, self.cy])
        return np.hstack([a, shift[:, None]])

    def pixel_to_ground(self, pose, pixels):
        """(..., 2) pixel coordinates -> (..., 2) east/north metres."""
        m = self.pixel_to_ground_matrix(pose)
        return np.asarray(pixels, dtype=float) @ m[:, :2].T + m[:, 2]

    def motion_matrix(self, old_pose, new_pose):
        """2x3 affine matrix taking pixels of a ground point seen from `old_pose` to its pixels
        seen from `new_pose`: how the whole image moved because the camera moved."""
        old = np.vstack([self.pixel_to_ground_matrix(old_pose), [0, 0, 1]])
        new = np.vstack([self.pixel_to_ground_matrix(new_pose), [0, 0, 1]])
        return (np.linalg.inv(new) @ old)[:2]

    def ground_to_pixel(self, pose, points):
        """(..., 2) east/north metres -> (..., 2) pixel coordinates."""
        # The rotation is its own inverse (orthogonal and symmetric).
        r = self._rotation(pose) / self.metres_per_pixel(pose.alt)
        offset = np.asarray(points, dtype=float) - np.array([pose.east, pose.north])
        return offset @ r.T + np.array([self.cx, self.cy])
