"""GPS spoofing detection from optical flow between background keyframes, and the flow path the
GCS navigates by while the GPS is rejected. The keyframes are recorded from a simulated flight."""

import json
import math

import cv2
import numpy as np
import pytest

from aegislink._core import MsgType, Packetizer
from aegislink.common import config, geo, messages
from aegislink.gcs import spoof_detector
from aegislink.gcs.receiver import GroundStation
from aegislink.gcs.spoof_detector import FlowPath, SpoofDetector, flow_displacement
from aegislink.uav.camera import NadirCamera
from aegislink.uav.encoder import KEYFRAME_SIZE, decode_image, encode_keyframe
from aegislink.uav.sim_world import World, WorldConfig
from aegislink.uav.telemetry import Flight

STEP = 15           # frames between keyframes (2 Hz)
SECONDS = 40
ORIGIN = (46.0, 7.0)


@pytest.fixture(scope="module")
def flight():
    """[(frame, keyframe image, true pose)] every STEP frames, as the GCS decodes them."""
    world = World(WorldConfig(seed=2))
    camera = NadirCamera()
    uav = Flight(world)
    keyframes = []
    for frame in range(SECONDS * config.FPS):
        if frame % STEP == 0:
            image, _ = world.render(camera, uav.pose)
            gray = decode_image(encode_keyframe(image), gray=True, size=KEYFRAME_SIZE)
            keyframes.append((frame, gray, uav.pose))
        world.step(1 / config.FPS)
        uav.step(1 / config.FPS)
    return keyframes


def run(flight, spoof=lambda t: (0.0, 0.0), until_s=SECONDS):
    """Feeds the keyframes with the true position plus spoof(t); returns the detector and, per
    keyframe, (t, alarm, d2, flow position or None)."""
    detector = SpoofDetector()
    out = []
    for frame, image, pose in flight:
        t = frame / config.FPS
        if t > until_s:
            break
        offset = spoof(t)
        detector.add_keyframe(frame, image, pose.alt, pose.heading_deg,
                              (pose.east + offset[0], pose.north + offset[1]))
        path = detector.fallback
        out.append((t, detector.alarm, detector.d2, None if path is None else path.position(frame)))
    return detector, out


def test_flow_finds_the_true_displacement_between_keyframes(flight):
    errors = []
    velocity = (0.0, 0.0)
    for (_, a, pa), (_, b, pb) in zip(flight, flight[1:]):
        flow = flow_displacement(a, (pa.alt, pa.heading_deg), b, (pb.alt, pb.heading_deg),
                                 velocity)
        assert flow is not None and flow.inliers >= 100
        velocity = tuple(flow.displacement)
        errors.append(np.linalg.norm(flow.displacement - (pb.east - pa.east, pb.north - pa.north)))
    # The UAV flies 5-9 m between keyframes, and turns and climbs on the way.
    assert np.max(errors) < 0.1
    assert np.mean(errors) < 0.03


def test_flow_needs_texture():
    flat = np.full(KEYFRAME_SIZE[::-1], 90, np.uint8)
    assert flow_displacement(flat, (150.0, 0.0), flat, (150.0, 0.0)) is None


def test_no_alarm_on_a_clean_flight(flight):
    detector, out = run(flight)
    assert detector.alarms == 0 and detector.undecided == 0
    assert max(d2 for _, _, d2, _ in out if d2 is not None) < 1.0


def test_a_3_mps_drift_raises_the_alarm_within_3_keyframes(flight):
    start = 10.1

    def drift(t):
        return (3.0 * (t - start), 0.0) if t >= start else (0.0, 0.0)

    detector, out = run(flight, drift)
    first = next(t for t, alarm, _, _ in out if alarm)
    assert first - start <= 3 * STEP / config.FPS
    assert detector.alarms == 1
    assert all(alarm for t, alarm, _, _ in out if t >= first)


def test_a_50_m_jump_raises_the_alarm_and_ends_after_5_agreeing_keyframes(flight):
    def jump(t):
        return (0.0, 50.0) if 10.0 <= t < 20.0 else (0.0, 0.0)

    detector, out = run(flight, jump)
    alarms = [t for t, alarm, _, _ in out if alarm]
    assert alarms[0] == pytest.approx(11.0)            # keyframes at 10.0, 10.5, 11.0
    assert alarms[-1] == pytest.approx(21.5)           # agreeing at 20.0 ... 22.0, clear at 22.0
    assert detector.alarms == 1 and not detector.alarm


def test_while_the_alarm_is_on_the_flow_path_follows_the_true_position(flight):
    def drift(t):
        return (0.0, -3.0 * (t - 5.0)) if t >= 5.0 else (0.0, 0.0)

    detector, out = run(flight, drift)
    errors = []
    for (t, alarm, _, position), (frame, _, pose) in zip(out, flight):
        if alarm:
            errors.append(math.hypot(position[0] - pose.east, position[1] - pose.north))
    assert len(errors) > 50
    # 35 s on the flow alone, with the GPS 100 m off at the end.
    assert max(errors) < 1.0
    # Half a second after the last keyframe, carried on with the flow's velocity.
    path = detector.fallback
    ahead = path.position(path.frames[-1] + STEP)
    flown = np.array(path.positions[-1]) + np.array(detector.velocity) * 0.5
    assert np.linalg.norm(np.array(ahead) - flown) < 1e-6


def test_flow_path_interpolates_and_carries_on_with_its_velocity():
    path = FlowPath([30, 45], [(0.0, 0.0), (6.0, -3.0)], (12.0, -6.0))
    assert path.position(29) is None
    assert path.position(30) == (0.0, 0.0)
    assert path.position(40) == pytest.approx((4.0, -2.0))
    assert path.position(60) == pytest.approx((12.0, -6.0))


def test_pairs_that_the_flow_cannot_follow_decide_nothing(flight):
    detector = SpoofDetector()
    flat = np.full(KEYFRAME_SIZE[::-1], 90, np.uint8)
    for frame, _, pose in flight[:20]:
        # Untextured keyframes and a GPS that jumps 50 m every time: never an alarm.
        detector.add_keyframe(frame, flat, pose.alt, pose.heading_deg, (frame * 50.0, 0.0))
    assert detector.alarms == 0 and detector.undecided == 19


def test_a_repeated_keyframe_changes_nothing(flight):
    def jump(t):
        return (0.0, 50.0) if t >= 5.0 else (0.0, 0.0)

    detector, _ = run(flight, jump, until_s=7.0)
    assert detector.alarm
    chain = list(detector.frames)
    frame, image, pose = flight[14]                    # 7.0 s, the last one fed
    detector.add_keyframe(frame, image, pose.alt, pose.heading_deg, (pose.east, pose.north))
    assert detector.alarm and detector.frames == chain


@pytest.fixture
def station(tmp_path):
    s = GroundStation(0, 0, 9, track_log=tmp_path / "tracks.jsonl")
    yield s
    s.close()


def fly_station(station, flight, spoof, until_s):
    """Telemetry (true position + spoof(t), ORIGIN at the start) and keyframes, every STEP frames."""
    tx = Packetizer()
    start = flight[0][2]
    for frame, image, pose in flight:
        t = frame / config.FPS
        if t > until_s:
            break
        offset = spoof(t)
        lat, lon = geo.enu_to_latlon(pose.east - start.east + offset[0],
                                     pose.north - start.north + offset[1], ORIGIN)
        tel = messages.TelemetryMsg(frame, float(lat), float(lon), pose.alt, 10.0, pose.heading_deg,
                                    0.0, 0.0)
        webp = cv2.imencode(".webp", image, [cv2.IMWRITE_WEBP_QUALITY, 25])[1].tobytes()
        keyframe = messages.KeyframeMsg(frame, pose.alt, pose.heading_deg, webp)
        for msg_type, body in ((MsgType.TELEMETRY, messages.encode_telemetry(tel)),
                               (MsgType.BG_KEYFRAME, messages.encode_keyframe(keyframe))):
            for datagram in tx.packetize(msg_type, frame * 33_333, body):
                station._receive("fovea", datagram, t)
        station.log_tracks_if_due(t)
    return start


def test_the_station_navigates_by_the_flow_while_the_gps_is_spoofed(station, flight):
    start = fly_station(station, flight, lambda t: (50.0, 0.0) if t >= 10.0 else (0.0, 0.0), 20.0)
    frame, _, pose = flight[40]                         # 20.0 s
    stats = station.stats(20.0)
    assert stats["gps_alarm"] and stats["gps_alarms"] == 1 and stats["spoof_d2"] > 1000
    assert stats["flow_speed"] == pytest.approx(stats["gps_speed"], abs=1.0)
    # Every position the view uses since the alarm comes from the flow, not the GPS 50 m off.
    view = station.view()
    for f in range(330, frame + 1):
        east, north = view.poses.position(f)
        assert view.poses.gps_position(f)[0] == pytest.approx(east + 50, abs=1.5)
    east, north = view.poses.position(frame)
    assert math.hypot(east - (pose.east - start.east), north - (pose.north - start.north)) < 0.5
    line = json.loads(station.track_log.path.read_text().splitlines()[-1])
    assert line["nav"]["alarm"] and line["nav"]["frame"] == frame
    assert line["nav"]["gps"][0] == pytest.approx(line["nav"]["used"][0] + 50, abs=0.5)


def test_the_station_uses_the_gps_again_once_it_agrees(station, flight):
    fly_station(station, flight, lambda t: (0.0, -50.0) if 5.0 <= t < 10.0 else (0.0, 0.0), 15.0)
    stats = station.stats(15.0)
    assert not stats["gps_alarm"] and stats["gps_alarms"] == 1
    assert station.poses.fallback is None
    assert station.poses.position(450) == station.poses.gps_position(450)


def test_keyframes_too_far_apart_are_not_compared(flight):
    detector = SpoofDetector()
    (f0, a, pa), (f1, b, pb) = flight[0], flight[int(spoof_detector.MAX_PAIR_S * 2) + 1]
    detector.add_keyframe(f0, a, pa.alt, pa.heading_deg, (pa.east, pa.north))
    detector.add_keyframe(f1, b, pb.alt, pb.heading_deg, (pb.east + 100, pb.north))
    assert detector.undecided == 1 and detector.d2 is None
