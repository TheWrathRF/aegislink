"""A video clip as the camera: the camera path measured from the images, the clip played back."""

import math

import cv2
import numpy as np
import pytest

from aegislink.common.config import FPS
from aegislink.uav.camera import NadirCamera, Pose
from aegislink.uav.sim_world import World, WorldConfig
from aegislink.uav.telemetry import Flight
from aegislink.uav.video import VideoClip, estimate_path


@pytest.fixture(scope="module")
def flight_frames():
    """Four seconds of the simulated flight (turning and climbing) and its true poses."""
    world = World(WorldConfig(seed=3))
    camera = NadirCamera()
    flight = Flight(world, follow_id=2)
    frames, poses = [], []
    for _ in range(4 * FPS):
        pose = flight.pose
        frames.append(world.render(camera, pose)[0])
        poses.append(pose)
        world.step(1 / FPS)
        flight.step(1 / FPS)
    return camera, frames, poses


def heading_error(a, b):
    return abs((a - b + 180) % 360 - 180)


def test_path_from_the_images_follows_the_true_flight(flight_frames):
    camera, frames, truth = flight_frames
    path = estimate_path(frames, camera, truth[0])
    assert len(path) == len(truth)
    position_error = [math.hypot(p.east - t.east, p.north - t.north) for p, t in zip(path, truth)]
    travelled = math.hypot(truth[-1].east - truth[0].east, truth[-1].north - truth[0].north)
    assert travelled > 20
    assert max(position_error) < 0.5
    assert max(abs(p.alt - t.alt) for p, t in zip(path, truth)) < 0.5
    assert max(heading_error(p.heading_deg, t.heading_deg) for p, t in zip(path, truth)) < 0.5
    turned = heading_error(truth[-1].heading_deg, truth[0].heading_deg)
    assert turned > 5  # the test covers a turn, not only a straight line


def test_a_frame_that_cannot_be_registered_keeps_the_last_motion(flight_frames):
    camera, frames, truth = flight_frames
    frames = list(frames[:30])
    frames[15] = np.zeros_like(frames[15])
    path = estimate_path(frames, camera, truth[0])
    # The blank frame and the one after it are guessed from the motion before; the rest is
    # measured again, so the error stays at about two frames of motion.
    assert all(math.isfinite(p.east) and math.isfinite(p.alt) for p in path)
    assert math.hypot(path[29].east - truth[29].east, path[29].north - truth[29].north) < 1.0


@pytest.fixture(scope="module")
def clip_file(tmp_path_factory, flight_frames):
    _, frames, _ = flight_frames
    path = tmp_path_factory.mktemp("video") / "clip.mp4"
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (640, 360))
    for frame in frames[:20]:
        writer.write(cv2.resize(frame, (640, 360), interpolation=cv2.INTER_AREA))
    writer.release()
    return path


def test_clip_is_resized_and_played_back_and_forth(clip_file, flight_frames):
    camera, _, truth = flight_frames
    clip = VideoClip(clip_file, camera, alt0_m=truth[0].alt)
    assert len(clip) == 20
    image, pose = clip.frame(0)
    assert image.shape == (720, 1280, 3)
    # Back and forth without a jump: 0..19, then 18..1, then 0 again.
    order = [clip.index(i) for i in range(40)]
    assert order[:20] == list(range(20))
    assert order[20:38] == list(range(18, 0, -1))
    assert order[38] == 0 and order[39] == 1
    assert clip.frame(21)[1] == clip.frame(17)[1]


def test_clip_telemetry_matches_its_path(clip_file, flight_frames):
    camera, _, truth = flight_frames
    clip = VideoClip(clip_file, camera, alt0_m=truth[0].alt)
    _, pose = clip.frame(10)
    tel = clip.telemetry(10, t=10 / FPS)
    assert tel.alt_m == pose.alt and tel.heading_deg == pose.heading_deg
    step = math.hypot(clip.frame(11)[1].east - clip.frame(9)[1].east,
                      clip.frame(11)[1].north - clip.frame(9)[1].north)
    assert tel.speed_mps == pytest.approx(step * FPS / 2, rel=1e-6)
    # The first frame starts at the origin, heading north, at the given altitude.
    first = clip.frame(0)[1]
    assert (first.east, first.north, first.heading_deg, first.alt) == (0.0, 0.0, 0.0, truth[0].alt)
    # The flight's true speed comes out of the (rescaled, compressed) images within 5 %.
    true_speed = math.hypot(truth[11].east - truth[9].east, truth[11].north - truth[9].north)
    assert tel.speed_mps == pytest.approx(true_speed * FPS / 2, rel=0.05)


def test_missing_clip_is_an_error(tmp_path):
    with pytest.raises(FileNotFoundError):
        VideoClip(tmp_path / "missing.mp4", NadirCamera(), alt0_m=50)


@pytest.mark.parametrize("alt0", [0.0, -55.0, float("nan"), float("inf")])
def test_an_impossible_first_altitude_is_an_error(clip_file, alt0):
    # A negative altitude would turn the whole path by 180 degrees without a word, and 0 makes
    # the ground station drop every message.
    with pytest.raises(ValueError):
        VideoClip(clip_file, NadirCamera(), alt0_m=alt0)
