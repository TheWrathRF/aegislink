"""The synthetic world: terrain, roads, trees, vehicles, and the UAV flight over it."""

import math

import numpy as np
import pytest

from aegislink.common import geo
from aegislink.uav.camera import NadirCamera, Pose
from aegislink.uav.sim_world import World, WorldConfig
from aegislink.uav.telemetry import Flight

SMALL = dict(size_m=500.0, res_m=0.5, loops=3, vehicles_per_loop=5, trees=300)
DT = 1 / 30


@pytest.fixture(scope="module")
def world():
    return World(WorldConfig(seed=3, **SMALL))


def test_same_seed_gives_the_same_world():
    a = World(WorldConfig(seed=5, **SMALL))
    b = World(WorldConfig(seed=5, **SMALL))
    for _ in range(100):
        a.step(DT)
        b.step(DT)
    assert np.array_equal(a.ground, b.ground)
    assert [a.vehicle_pose(v) for v in a.vehicles] == [b.vehicle_pose(v) for v in b.vehicles]
    c = World(WorldConfig(seed=6, **SMALL))
    assert not np.array_equal(a.ground, c.ground)


def test_roads_stay_inside_the_map(world):
    half = world.config.size_m / 2
    for road in world.roads:
        assert np.all(np.abs(road.points) < half - 20)


def test_vehicles_drive_in_their_lane_without_overlapping():
    w = World(WorldConfig(seed=4, **SMALL))
    lane_offset = w.config.road_width_m / 4
    min_gap = math.inf
    stopped_s = {v.id: 0.0 for v in w.vehicles}
    for _ in range(3000):  # 100 s
        w.step(DT)
        for v in w.vehicles:
            east, north, _ = w.vehicle_pose(v)
            road = w.roads[v.road]
            distance = np.min(np.hypot(road.points[:, 0] - east, road.points[:, 1] - north))
            assert distance == pytest.approx(lane_offset, abs=0.3)
            if v.speed == 0:
                stopped_s[v.id] += DT
        min_gap = min(min_gap, min(w.gap_to_leader(v) for v in w.vehicles))
    assert min_gap >= 1.0
    # Every vehicle stops now and then, but none stands still for most of the time.
    assert sum(t > 1.0 for t in stopped_s.values()) >= len(w.vehicles) // 2
    assert max(stopped_s.values()) < 60.0


def test_vehicle_appears_where_the_camera_says(world):
    camera = NadirCamera()
    for v in world.vehicles:
        if world.visible_fraction(v) == 1.0:
            break
    east, north, heading = world.vehicle_pose(v)
    pose = Pose(east=east + 10, north=north - 5, alt=120.0, heading_deg=33.0)
    frame, truths = world.render(camera, pose)
    assert frame.shape == (camera.height, camera.width, 3)
    u, v_px = np.round(camera.ground_to_pixel(pose, [east, north])).astype(int)
    assert np.abs(frame[v_px, u].astype(int) - v.roof_color).max() <= 2
    truth = next(t for t in truths if t.id == v.id)
    x1, y1, x2, y2 = truth.box
    assert x1 < u < x2 and y1 < v_px < y2
    # The box spans the rotated body: at least its width, at most its diagonal.
    mpp = camera.metres_per_pixel(pose.alt)
    size = max(x2 - x1, y2 - y1) * mpp
    assert v.width <= size <= math.hypot(v.length, v.width) + 2 * mpp
    assert truth.in_frame == 1.0


def test_trees_are_drawn_over_vehicles_and_hide_them(world):
    camera = NadirCamera()
    road, s = world.occluders[0]
    v = world.vehicles[0]
    v.road, v.s = road, s
    assert world.visible_fraction(v) < 0.5
    east, north, _ = world.vehicle_pose(v)
    pose = Pose(east=east, north=north, alt=120.0, heading_deg=0.0)
    frame, truths = world.render(camera, pose)
    u, v_px = np.round(camera.ground_to_pixel(pose, [east, north])).astype(int)
    assert np.abs(frame[v_px, u].astype(int) - v.roof_color).max() > 20
    assert next(t for t in truths if t.id == v.id).visible < 0.5


def test_truth_box_is_clipped_at_the_frame_edge(world):
    camera = NadirCamera()
    v = world.vehicles[1]
    east, north, _ = world.vehicle_pose(v)
    # Put the vehicle on the left edge of the image (image up = north).
    half_width_m = camera.cx * camera.metres_per_pixel(120.0)
    pose = Pose(east=east + half_width_m, north=north, alt=120.0, heading_deg=0.0)
    _, truths = world.render(camera, pose)
    truth = next(t for t in truths if t.id == v.id)
    assert truth.box[0] == 0.0
    assert 0.2 < truth.in_frame < 0.8


def test_flight_keeps_the_lead_vehicle_in_view():
    w = World(WorldConfig(seed=4, **SMALL))
    camera = NadirCamera()
    flight = Flight(w, follow_id=0)
    in_view = 0
    frames = 1800  # 60 s
    max_roll = 0.0
    for i in range(frames):
        w.step(DT)
        flight.step(DT)
        pose = flight.pose
        east, north, _ = w.vehicle_pose(w.vehicles[0])
        u, v = camera.ground_to_pixel(pose, [east, north])
        in_view += 0 <= u < camera.width and 0 <= v < camera.height
        tel = flight.telemetry(i * DT)
        max_roll = max(max_roll, abs(tel.roll_deg))
    assert in_view / frames > 0.95
    assert max_roll < 30
    back_east, back_north = geo.latlon_to_enu(tel.lat, tel.lon, w.config.origin_latlon)
    assert back_east == pytest.approx(pose.east, abs=1e-6)
    assert back_north == pytest.approx(pose.north, abs=1e-6)
    assert tel.alt_m == pose.alt and tel.heading_deg == pose.heading_deg
