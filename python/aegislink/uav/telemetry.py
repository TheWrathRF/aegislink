"""The UAV's flight over the world and the telemetry it reports.

The UAV is a multirotor that follows one vehicle: it steers its velocity towards the vehicle's
position plus the vehicle's own velocity, with limited acceleration, and turns its nose (and so
the camera's image up) towards the direction it flies at a limited yaw rate. Pitch and roll
follow from the accelerations, as for a real multirotor.
"""

import math
from dataclasses import dataclass

import numpy as np

from aegislink.common import geo
from aegislink.uav.camera import Pose

GRAVITY = 9.81


@dataclass
class Telemetry:
    t: float              # seconds since the start
    lat: float            # degrees
    lon: float
    alt_m: float          # above the ground
    speed_mps: float      # ground speed
    heading_deg: float    # where the nose (and image up) points, clockwise from north
    pitch_deg: float      # nose down is negative
    roll_deg: float       # right wing down is positive


class Flight:
    GAIN = 0.4            # 1/s: how fast a position error turns into a velocity
    MAX_SPEED = 18.0      # m/s
    MAX_ACCEL = 3.0       # m/s^2
    VELOCITY_TAU_S = 1.0  # time constant of reaching the wanted velocity
    MAX_YAW_RATE = 25.0   # deg/s
    MIN_TURN_SPEED = 1.5  # m/s; slower than this the nose stays where it is

    def __init__(self, world, follow_id=0, alt_m=150.0, alt_swing_m=10.0, alt_period_s=90.0):
        self.world = world
        self.follow = world.vehicles[follow_id]
        east, north, heading = world.vehicle_pose(self.follow)
        self.position = np.array([east, north])
        self.velocity = self._target_velocity()
        self.heading = heading
        self.alt_m = alt_m
        self.alt_swing_m = alt_swing_m
        self.alt_period_s = alt_period_s
        self.t = 0.0
        self.yaw_rate = 0.0      # deg/s
        self.accel = np.zeros(2)

    def _target_velocity(self):
        east, north, heading = self.world.vehicle_pose(self.follow)
        h = math.radians(heading)
        lead_velocity = self.follow.speed * np.array([math.sin(h), math.cos(h)])
        wanted = lead_velocity + self.GAIN * (np.array([east, north]) - self.position)
        speed = np.linalg.norm(wanted)
        return wanted if speed <= self.MAX_SPEED else wanted * self.MAX_SPEED / speed

    def step(self, dt):
        accel = (self._target_velocity() - self.velocity) / self.VELOCITY_TAU_S
        norm = np.linalg.norm(accel)
        if norm > self.MAX_ACCEL:
            accel *= self.MAX_ACCEL / norm
        self.accel = accel
        self.velocity = self.velocity + accel * dt
        self.position = self.position + self.velocity * dt

        old_heading = self.heading
        if np.linalg.norm(self.velocity) > self.MIN_TURN_SPEED:
            course = math.degrees(math.atan2(self.velocity[0], self.velocity[1]))
            error = (course - self.heading + 180) % 360 - 180
            limit = self.MAX_YAW_RATE * dt
            self.heading = (self.heading + max(-limit, min(limit, error))) % 360
        self.yaw_rate = ((self.heading - old_heading + 180) % 360 - 180) / dt
        self.t += dt

    @property
    def pose(self):
        alt = self.alt_m + self.alt_swing_m * math.sin(2 * math.pi * self.t / self.alt_period_s)
        return Pose(float(self.position[0]), float(self.position[1]), alt, self.heading)

    def telemetry(self, t):
        pose = self.pose
        lat, lon = geo.enu_to_latlon(pose.east, pose.north, self.world.config.origin_latlon)
        h = math.radians(self.heading)
        forward = np.array([math.sin(h), math.cos(h)])
        speed = float(np.linalg.norm(self.velocity))
        # A multirotor tilts into its acceleration: forward acceleration pitches the nose down,
        # and a turn needs a sideways (centripetal) acceleration of speed x yaw rate.
        pitch = -math.degrees(math.atan2(float(self.accel @ forward), GRAVITY))
        roll = math.degrees(math.atan2(speed * math.radians(self.yaw_rate), GRAVITY))
        return Telemetry(t, float(lat), float(lon), pose.alt, speed, pose.heading_deg, pitch, roll)
