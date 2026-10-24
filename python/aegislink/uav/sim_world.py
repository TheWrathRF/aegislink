"""A procedural world for the UAV to fly over, with exact ground truth.

The ground (grass, soil, roads) is one raster image in local east/north metres, drawn once.
Vehicles drive on closed road loops, keep their distance to the vehicle ahead and stop now and
then. Trees and a bridge sit above the roads; they are drawn after the vehicles, so a vehicle
under them is hidden in the image, and `visible_fraction` says how much of it can be seen.
"""

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

ROAD_COLOR = (78, 80, 82)
ROAD_EDGE_COLOR = (170, 175, 175)
BRIDGE_COLOR = (150, 158, 160)
BRIDGE_RAIL_COLOR = (95, 100, 102)
# BGR, as OpenCV draws.
GROUND_COLORS = [(62, 120, 86), (78, 140, 150), (64, 95, 120)]  # grass, dry grass, soil
VEHICLE_COLORS = [(235, 235, 235), (40, 38, 36), (185, 185, 190), (40, 40, 190),
                  (170, 95, 40), (40, 190, 225), (90, 60, 40), (120, 120, 125)]
TREE_COLOR = (40, 85, 45)
TREE_LIGHT_COLOR = (60, 115, 70)


@dataclass
class WorldConfig:
    seed: int = 1
    size_m: float = 1000.0       # the map is size_m x size_m, centred on the origin
    res_m: float = 0.25          # metres per pixel of the ground raster
    loops: int = 4               # closed roads
    vehicles_per_loop: int = 16
    trees: int = 1500
    road_width_m: float = 7.0    # two lanes, right-hand traffic
    origin_latlon: tuple = (46.0, 7.0)


class Road:
    """A closed loop, sampled as a polyline of (east, north) points."""

    def __init__(self, points):
        self.points = points
        closed = np.vstack([points, points[:1]])
        steps = np.hypot(*np.diff(closed, axis=0).T)
        self.cumulative = np.concatenate([[0.0], np.cumsum(steps)])
        self.length = float(self.cumulative[-1])
        self._closed = closed

    def point_at(self, s):
        s = s % self.length
        east = np.interp(s, self.cumulative, self._closed[:, 0])
        north = np.interp(s, self.cumulative, self._closed[:, 1])
        return np.array([east, north])

    def direction_at(self, s):
        """Unit vector along the loop (towards growing s)."""
        d = self.point_at(s + 0.5) - self.point_at(s - 0.5)
        return d / np.linalg.norm(d)


@dataclass
class Vehicle:
    id: int
    road: int
    direction: int               # +1 drives towards growing s, -1 the other way
    s: float                     # position along the road centre line, metres
    speed: float
    desired_speed: float
    length: float
    width: float
    color: tuple
    roof_color: tuple
    next_stop_s: float           # seconds of driving left before the next stop
    stop_left_s: float = 0.0     # seconds left standing still (once stopped)
    stopping: bool = False


@dataclass
class Tree:
    east: float
    north: float
    radius: float


@dataclass
class Truth:
    """One vehicle as the camera sees it."""
    id: int
    box: tuple | None            # (x1, y1, x2, y2) pixels, clipped to the image; None if outside
    in_frame: float              # fraction of the unclipped box inside the image
    visible: float               # fraction of the vehicle not hidden by trees or bridges


@dataclass
class _Bridge:
    corners: np.ndarray          # 4 x (east, north)
    rails: list = field(default_factory=list)  # two (start, end) segments


class World:
    LANE_GAP_M = 1.0             # vehicles never get closer than this, bumper to bumper
    HEADWAY_S = 1.2              # time gap a following vehicle keeps
    MIN_GAP_M = 3.0              # gap a queue of stopped vehicles keeps
    ACCEL = 2.0                  # m/s^2
    BRAKE = 6.0

    def __init__(self, config=WorldConfig()):
        self.config = config
        self.rng = np.random.default_rng(config.seed)
        self.size_px = int(round(config.size_m / config.res_m))
        self.roads = [self._make_loop() for _ in range(config.loops)]
        self.ground = self._draw_ground()
        self.trees = []
        self.bridges = []
        self.occluders = []      # (road index, s) of each spot where something covers the road
        self.cover = np.zeros((self.size_px, self.size_px), np.uint8)  # 255 under trees/bridges
        self._place_occluders()
        self._place_trees()
        self._tree_array = np.array([[t.east, t.north, t.radius] for t in self.trees])
        self.vehicles = self._place_vehicles()

    def to_raster(self, points):
        """Map (east, north) to raster (column, row): ((east + size/2) / res, (size/2 - north) / res)."""
        half = self.config.size_m / 2
        points = np.asarray(points, dtype=float)
        return np.stack([(points[..., 0] + half), (half - points[..., 1])], -1) / self.config.res_m

    def raster_matrix(self):
        """2x3 affine matrix taking (east, north) to raster (column, row)."""
        half, res = self.config.size_m / 2, self.config.res_m
        return np.array([[1 / res, 0, half / res], [0, -1 / res, half / res]])

    def _make_loop(self):
        half = self.config.size_m / 2
        theta = np.linspace(0, 2 * np.pi, 1000, endpoint=False)
        while True:
            radius = self.rng.uniform(0.25, 0.5) * half
            center = self.rng.uniform(-0.45, 0.45, 2) * half
            wobble = np.ones_like(theta)
            for k in (2, 3, 4):
                wobble += self.rng.uniform(0, 0.12) * np.cos(k * theta + self.rng.uniform(0, 2 * np.pi))
            points = center + (radius * wobble)[:, None] * np.stack([np.cos(theta), np.sin(theta)], 1)
            if np.all(np.abs(points) < half - 30):
                return Road(points)

    def _noise(self, cells):
        """Smooth noise in [0, 1] that varies over about size / cells pixels."""
        small = self.rng.random((cells + 1, cells + 1), dtype=np.float32)
        return cv2.resize(small, (self.size_px, self.size_px), interpolation=cv2.INTER_CUBIC)

    def _draw_ground(self):
        cells = max(4, int(self.config.size_m / 150))
        mix = np.clip(self._noise(cells) * 1.4 - 0.2, 0, 1)
        soil = np.clip(self._noise(cells * 3) * 2.0 - 1.1, 0, 1)
        colors = np.array(GROUND_COLORS, np.float32)
        image = (1 - mix)[..., None] * colors[0] + mix[..., None] * colors[1]
        image = (1 - soil)[..., None] * image + soil[..., None] * colors[2]
        # Texture at several scales, fixed to the ground, so image matching has something to grip.
        detail = sum(w * (self._noise(c) - 0.5) for w, c in
                     ((40, int(self.config.size_m / 8)), (30, int(self.config.size_m / 2)),
                      (20, int(self.config.size_m * 1.5))))
        image = np.clip(image + detail[..., None], 0, 255).astype(np.uint8)

        shift = 4  # sub-pixel bits for cv2 drawing
        width_px = self.config.road_width_m / self.config.res_m
        for road in self.roads:
            pts = np.round(self.to_raster(road.points) * (1 << shift)).astype(np.int32)
            cv2.polylines(image, [pts], True, ROAD_EDGE_COLOR, int(width_px + 4), cv2.LINE_AA, shift)
            cv2.polylines(image, [pts], True, ROAD_COLOR, int(width_px), cv2.LINE_AA, shift)
        grain = (self._noise(int(self.config.size_m * 2)) - 0.5) * 16
        return np.clip(image + grain[..., None], 0, 255).astype(np.uint8)

    def _place_occluders(self):
        # Two tree clumps over every road and one bridge over the first, at random spots.
        for i, road in enumerate(self.roads):
            for s in self.rng.uniform(0, road.length, 2):
                center = road.point_at(s)
                self._add_tree(center[0], center[1], self.rng.uniform(5.5, 7.0))
                self.occluders.append((i, float(s)))
        road = self.roads[0]
        s = float(self.rng.uniform(0, road.length))
        along = road.direction_at(s)
        across = np.array([along[1], -along[0]])
        center = road.point_at(s)
        half_along, half_across = 6.0, 15.0
        corners = np.array([center + a * half_along * along + b * half_across * across
                            for a, b in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
        rails = [(center + a * (half_along - 0.5) * along - half_across * across,
                  center + a * (half_along - 0.5) * along + half_across * across) for a in (-1, 1)]
        self.bridges.append(_Bridge(corners, rails))
        cv2.fillConvexPoly(self.cover, np.round(self.to_raster(corners)).astype(np.int32), 255)
        self.occluders.append((0, s))

    def _place_trees(self):
        on_road = np.zeros_like(self.cover)
        width_px = int(self.config.road_width_m / self.config.res_m)
        for road in self.roads:
            pts = np.round(self.to_raster(road.points)).astype(np.int32)
            cv2.polylines(on_road, [pts], True, 255, width_px + int(8 / self.config.res_m))
        half = self.config.size_m / 2
        placed = 0
        while placed < self.config.trees:
            east, north = self.rng.uniform(-half, half, 2)
            col, row = self.to_raster([east, north]).astype(int)
            if 0 <= col < self.size_px and 0 <= row < self.size_px and not on_road[row, col]:
                self._add_tree(east, north, self.rng.uniform(2.0, 4.5))
                placed += 1

    def _add_tree(self, east, north, radius):
        self.trees.append(Tree(float(east), float(north), float(radius)))
        col, row = np.round(self.to_raster([east, north])).astype(int)
        cv2.circle(self.cover, (int(col), int(row)), int(radius / self.config.res_m), 255, -1)

    def _place_vehicles(self):
        vehicles = []
        for road_index, road in enumerate(self.roads):
            n = self.config.vehicles_per_loop
            spacing = road.length / n
            for k in range(n):
                truck = self.rng.random() < 0.15
                length, width = (8.0, 2.5) if truck else (4.5, 1.9)
                color = VEHICLE_COLORS[self.rng.integers(len(VEHICLE_COLORS))]
                roof = tuple(int(c * 0.8 + 20) for c in color)
                desired = float(self.rng.uniform(7, 13))
                vehicles.append(Vehicle(
                    id=len(vehicles), road=road_index, direction=1 if k % 2 == 0 else -1,
                    s=float(k * spacing + self.rng.uniform(0, spacing / 3)), speed=desired,
                    desired_speed=desired, length=length, width=width, color=color,
                    roof_color=roof, next_stop_s=float(self.rng.uniform(5, 40))))
        return vehicles

    def vehicle_pose(self, v):
        """(east, north, heading in degrees) of a vehicle's centre, in its lane."""
        road = self.roads[v.road]
        forward = road.direction_at(v.s) * v.direction
        right = np.array([forward[1], -forward[0]])
        east, north = road.point_at(v.s) + right * self.config.road_width_m / 4
        heading = math.degrees(math.atan2(forward[0], forward[1])) % 360
        return float(east), float(north), heading

    def vehicle_poses(self):
        """Every vehicle's pose, by vehicle id. Computed once per frame and passed to `render`."""
        return [self.vehicle_pose(v) for v in self.vehicles]

    def _gaps(self):
        """{vehicle id: bumper-to-bumper distance to the next vehicle ahead in its lane}."""
        lanes = {}
        for v in self.vehicles:
            lanes.setdefault((v.road, v.direction), []).append(v)
        gaps = {}
        for (road, direction), lane in lanes.items():
            length = self.roads[road].length
            lane.sort(key=lambda v: (v.s * direction) % length)
            for i, v in enumerate(lane):
                if len(lane) == 1:
                    gaps[v.id] = math.inf
                    continue
                leader = lane[(i + 1) % len(lane)]
                d = ((leader.s - v.s) * direction) % length
                gaps[v.id] = d - (leader.length + v.length) / 2
        return gaps

    def gap_to_leader(self, v):
        return self._gaps()[v.id]

    def step(self, dt):
        # Gaps from the start of the step stay safe: a leader only ever moves away.
        gaps = self._gaps()
        for v in self.vehicles:
            if v.stop_left_s > 0:
                v.stop_left_s -= dt
                if v.stop_left_s <= 0:
                    v.stopping = False
                    v.next_stop_s = float(self.rng.uniform(15, 45))
                continue
            v.next_stop_s -= dt
            if v.next_stop_s <= 0:
                v.stopping = True
            target = 0.0 if v.stopping else v.desired_speed
            gap = gaps[v.id]
            target = min(target, max(0.0, (gap - self.MIN_GAP_M) / self.HEADWAY_S))
            if target > v.speed:
                v.speed = min(target, v.speed + self.ACCEL * dt)
            else:
                v.speed = max(target, v.speed - self.BRAKE * dt)
            move = min(v.speed * dt, max(0.0, gap - self.LANE_GAP_M))
            v.speed = move / dt
            v.s = (v.s + move * v.direction) % self.roads[v.road].length
            if v.stopping and v.speed == 0:
                v.stop_left_s = float(self.rng.uniform(2, 6))

    @staticmethod
    def _corners(pose, length, width):
        east, north, heading = pose
        h = math.radians(heading)
        forward = np.array([math.sin(h), math.cos(h)])
        right = np.array([math.cos(h), -math.sin(h)])
        half_l, half_w = length / 2, width / 2
        center = np.array([east, north])
        return np.array([center + a * half_l * forward + b * half_w * right
                         for a, b in ((1, 1), (1, -1), (-1, -1), (-1, 1))])

    def visible_fraction(self, v, pose=None):
        """Share of a vehicle's footprint not under a tree or bridge (from a 5 x 3 sample grid)."""
        east, north, heading = pose or self.vehicle_pose(v)
        h = math.radians(heading)
        forward = np.array([math.sin(h), math.cos(h)])
        right = np.array([math.cos(h), -math.sin(h)])
        a, b = np.meshgrid(np.linspace(-0.4, 0.4, 5) * v.length, np.linspace(-0.4, 0.4, 3) * v.width)
        points = np.array([east, north]) + a.reshape(-1, 1) * forward + b.reshape(-1, 1) * right
        cols, rows = np.round(self.to_raster(points)).astype(int).T
        return float(np.mean(self.cover[rows, cols] == 0))

    def render(self, camera, pose, poses=None):
        """Renders the camera image at `pose`; returns (BGR image, list of Truth). `poses` are the
        vehicles' poses from `vehicle_poses`, if the caller has them already."""
        if poses is None:
            poses = self.vehicle_poses()
        to_raster = np.vstack([self.raster_matrix(), [0, 0, 1]])
        to_ground = np.vstack([camera.pixel_to_ground_matrix(pose), [0, 0, 1]])
        matrix = (to_raster @ to_ground)[:2]
        frame = cv2.warpAffine(self.ground, matrix, (camera.width, camera.height),
                               flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
                               borderMode=cv2.BORDER_CONSTANT, borderValue=GROUND_COLORS[0])
        mpp = camera.metres_per_pixel(pose.alt)
        view_radius = math.hypot(camera.width, camera.height) / 2 * mpp + 10

        def to_px(points):
            return np.round(camera.ground_to_pixel(pose, points) * 16).astype(np.int32)

        truths = []
        for v in self.vehicles:
            v_pose = poses[v.id]
            if math.hypot(v_pose[0] - pose.east, v_pose[1] - pose.north) > view_radius:
                continue
            corners = self._corners(v_pose, v.length, v.width)
            cv2.fillConvexPoly(frame, to_px(corners), v.color, cv2.LINE_AA, 4)
            roof = self._corners(v_pose, v.length * 0.55, v.width * 0.8)
            cv2.fillConvexPoly(frame, to_px(roof), v.roof_color, cv2.LINE_AA, 4)
            truths.append(self._truth(v, v_pose, camera, pose, corners))

        if len(self._tree_array):
            near = np.hypot(self._tree_array[:, 0] - pose.east,
                            self._tree_array[:, 1] - pose.north) < view_radius
            for east, north, radius in self._tree_array[near]:
                cx, cy = to_px([east, north])
                r = int(radius / mpp * 16)
                cv2.circle(frame, (int(cx), int(cy)), r, TREE_COLOR, -1, cv2.LINE_AA, 4)
                cv2.circle(frame, (int(cx - r * 0.2), int(cy - r * 0.2)), int(r * 0.55),
                           TREE_LIGHT_COLOR, -1, cv2.LINE_AA, 4)
        for bridge in self.bridges:
            cv2.fillConvexPoly(frame, to_px(bridge.corners), BRIDGE_COLOR, cv2.LINE_AA, 4)
            for start, end in bridge.rails:
                p, q = to_px([start, end])
                cv2.line(frame, tuple(map(int, p)), tuple(map(int, q)), BRIDGE_RAIL_COLOR,
                         max(1, int(0.6 / mpp)), cv2.LINE_AA, 4)
        return frame, truths

    def _truth(self, v, v_pose, camera, pose, corners):
        pixels = camera.ground_to_pixel(pose, corners)
        x1, y1 = pixels.min(axis=0)
        x2, y2 = pixels.max(axis=0)
        cx1, cx2 = np.clip([x1, x2], 0, camera.width - 1)
        cy1, cy2 = np.clip([y1, y2], 0, camera.height - 1)
        area = (x2 - x1) * (y2 - y1)
        in_frame = float(max(0, cx2 - cx1) * max(0, cy2 - cy1) / area)
        box = (float(cx1), float(cy1), float(cx2), float(cy2)) if in_frame > 0 else None
        return Truth(v.id, box, in_frame, self.visible_fraction(v, v_pose))
