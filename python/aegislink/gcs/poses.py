"""The UAV's position at each recent frame, from its telemetry.

Image messages carry the camera's altitude and heading but not its position, so the GCS looks the
position up here by frame number. Positions are metres east/north of the first telemetry it
received, which is all the compositor needs: it only uses differences between two poses.
"""

import bisect

from aegislink.common import config, geo
from aegislink.uav.camera import Pose

HISTORY_FRAMES = 10 * config.FPS


class PoseHistory:
    def __init__(self):
        self.origin = None      # (lat, lon) of the first telemetry
        self.frames = []        # increasing frame numbers
        self.positions = []     # (east, north) for each frame

    def add(self, telemetry):
        """Adds a telemetry message. The receiver passes only ones at least as new as the held
        ones, so an older frame means the UAV started again; the same frame replaces it."""
        if self.frames and telemetry.frame < self.frames[-1]:
            self.clear()
        if self.frames and telemetry.frame == self.frames[-1]:
            self.frames.pop()
            self.positions.pop()
        if self.origin is None:
            self.origin = (telemetry.lat, telemetry.lon)
        east, north = geo.latlon_to_enu(telemetry.lat, telemetry.lon, self.origin)
        self.frames.append(telemetry.frame)
        self.positions.append((float(east), float(north)))
        excess = len(self.frames) - HISTORY_FRAMES
        if excess > 0:
            del self.frames[:excess]
            del self.positions[:excess]

    def clear(self):
        self.origin = None
        self.frames = []
        self.positions = []

    def copy(self):
        other = PoseHistory()
        other.origin = self.origin
        other.frames = list(self.frames)
        other.positions = list(self.positions)
        return other

    def position(self, frame):
        """(east, north) at `frame`, interpolated between the held frames around it (telemetry
        can be lost or skipped), or None outside the held frames."""
        i = bisect.bisect_left(self.frames, frame)
        if i == len(self.frames):
            return None
        if self.frames[i] == frame:
            return self.positions[i]
        if i == 0:
            return None
        f0, f1 = self.frames[i - 1], self.frames[i]
        (e0, n0), (e1, n1) = self.positions[i - 1], self.positions[i]
        w = (frame - f0) / (f1 - f0)
        return e0 + w * (e1 - e0), n0 + w * (n1 - n0)

    def pose(self, frame, alt, heading_deg):
        position = self.position(frame)
        if position is None:
            return None
        return Pose(position[0], position[1], alt, heading_deg)
