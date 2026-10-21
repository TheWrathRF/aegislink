"""The ground station's handling of what arrives, without the UAV."""

import socket
import time

import cv2
import numpy as np
import pytest

from aegislink._core import MsgType, Packetizer, parse_packet
from aegislink.common import messages
from aegislink.gcs import compositor
from aegislink.gcs.receiver import GroundStation, ReceiverThread


@pytest.fixture
def station():
    reports = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    reports.bind(("127.0.0.1", 0))
    reports.settimeout(1.0)
    s = GroundStation(0, 0, reports.getsockname()[1])
    s.reports = reports
    yield s
    s.close()
    reports.close()


TX = Packetizer()   # shared, so every message gets its own msg_id


def deliver(station, msg_type, body, now_s=1.0, link="fovea"):
    for datagram in TX.packetize(msg_type, int(now_s * 1e6), body):
        station._receive(link, datagram, now_s)


def webp(width, height, color=(0, 0, 255)):
    image = np.full((height, width, 3), color, np.uint8)
    return cv2.imencode(".webp", image)[1].tobytes()


def test_keeps_the_newest_telemetry_and_tracks(station):
    tel = messages.TelemetryMsg(10, 47.0, 8.0, 150, 10, 90, 0, 0)
    deliver(station, MsgType.TELEMETRY, messages.encode_telemetry(tel))
    old = messages.TelemetryMsg(9, 1.0, 1.0, 1, 1, 1, 0, 0)
    deliver(station, MsgType.TELEMETRY, messages.encode_telemetry(old))
    assert station.telemetry.frame == 10
    track = messages.TrackBox(1, (0, 0, 9, 9), True)
    deliver(station, MsgType.TRACKS, messages.encode_tracks(10, [track]))
    deliver(station, MsgType.TRACKS, messages.encode_tracks(8, []))
    assert station.tracks_frame == 10 and len(station.tracks) == 1


def test_view_holds_the_positions_of_recent_frames_until_a_restart(station):
    for frame, lat in ((400, 46.0), (401, 46.0001)):
        deliver(station, MsgType.TELEMETRY, messages.encode_telemetry(
            messages.TelemetryMsg(frame, lat, 7.0, 150, 10, 0, 0, 0)))
    view = station.view()
    assert view.telemetry.frame == 401 and view.tracks == [] and view.patches == {}
    assert view.poses.position(401) == pytest.approx((0.0, 11.13), abs=0.01)
    deliver(station, MsgType.TELEMETRY, messages.encode_telemetry(
        messages.TelemetryMsg(0, 46.0, 7.0, 150, 10, 0, 0, 0)))     # the UAV started again
    assert station.view().poses.position(401) is None
    assert view.poses.position(401) is not None     # the earlier view is a copy


def test_patch_is_kept_and_dropped_with_its_track(station):
    track = messages.TrackBox(7, (10, 10, 30, 20), True)
    deliver(station, MsgType.TRACKS, messages.encode_tracks(1, [track]))
    patch = messages.PatchMsg(1, 150, 0, 7, (2, 2, 38, 28), webp(36, 26))
    deliver(station, MsgType.ROI_PATCH, messages.encode_patch(patch))
    assert station.patches[7].image.shape == (26, 36, 3)
    deliver(station, MsgType.TRACKS, messages.encode_tracks(2, []))
    assert station.patches == {}


@pytest.mark.parametrize("box, size", [
    ((-4, 2, 32, 28), (36, 26)),            # outside the frame
    ((1270, 2, 1306, 28), (36, 26)),
    ((2, 2, 38, 28), (30, 26)),             # image not the size of the box
])
def test_bad_patches_are_counted_and_dropped(station, box, size):
    patch = messages.PatchMsg(1, 150, 0, 7, box, webp(*size))
    deliver(station, MsgType.ROI_PATCH, messages.encode_patch(patch))
    assert station.patches == {} and station.bad_messages == 1


def test_garbage_bodies_and_images_are_counted(station):
    deliver(station, MsgType.TELEMETRY, b"short")
    deliver(station, MsgType.BG_KEYFRAME, messages.encode_keyframe(
        messages.KeyframeMsg(1, 150, 0, b"not a webp")))
    assert station.bad_messages == 2
    assert station.telemetry is None and station.keyframe is None


def test_reports_go_out_four_times_a_second(station):
    deliver(station, MsgType.TELEMETRY, messages.encode_telemetry(
        messages.TelemetryMsg(1, 0, 0, 0, 0, 0, 0, 0)), now_s=0.99)
    for i in range(9):
        station.send_report_if_due(1.0 + i * 0.125)
    reports = []
    station.reports.settimeout(0.2)
    try:
        while True:
            reports.append(parse_packet(station.reports.recv(2048)))
    except TimeoutError:
        pass
    assert len(reports) == 5   # at 1.0, 1.25, 1.5, 1.75, 2.0
    first = messages.decode_report(reports[0].payload)
    assert first.received == 1 and first.min_delay_us == 0


def test_a_huge_timestamp_does_not_stop_the_reports(station):
    from aegislink._core import Header, encode_packet
    datagram = encode_packet(Header(MsgType.TELEMETRY, 0, 2**64 - 1, 0, 0, 1), b"x")
    station._receive("fovea", datagram, 1.0)
    station.send_report_if_due(1.0)
    assert station.reports.recv(2048)


def huge_jpeg(width=65000, height=65000):
    data = bytearray(cv2.imencode(".jpg", np.zeros((8, 8, 3), np.uint8))[1].tobytes())
    sof = data.index(b"\xff\xc0")
    data[sof + 5:sof + 9] = height.to_bytes(2, "big") + width.to_bytes(2, "big")
    return bytes(data)


def test_images_with_a_wrong_size_are_dropped_before_decoding(station):
    track = messages.TrackBox(7, (10, 10, 30, 20), True)
    deliver(station, MsgType.TRACKS, messages.encode_tracks(1, [track]))
    bomb = huge_jpeg()
    deliver(station, MsgType.ROI_PATCH, messages.encode_patch(
        messages.PatchMsg(1, 150, 0, 7, (2, 2, 38, 28), bomb)))
    deliver(station, MsgType.BG_KEYFRAME, messages.encode_keyframe(
        messages.KeyframeMsg(1, 150, 0, bomb)))
    big = cv2.imencode(".webp", np.zeros((540, 960), np.uint8))[1].tobytes()
    deliver(station, MsgType.BG_KEYFRAME, messages.encode_keyframe(
        messages.KeyframeMsg(1, 150, 0, big)))   # decodes, but is not a 480 x 270 keyframe
    deliver(station, MsgType.FULL_FRAME, messages.encode_full_frame(
        messages.FullFrameMsg(1, bomb)), link="baseline")
    assert station.bad_messages == 4
    assert station.patches == {} and station.keyframe is None and station.full_frame is None
    compositor.render_fovea(station.view(), (640, 360))


def test_a_restarted_uav_is_followed(station):
    for frame in (500, 0, 1):
        deliver(station, MsgType.TELEMETRY, messages.encode_telemetry(
            messages.TelemetryMsg(frame, 0, 0, 150, 0, 0, 0, 0)))
        deliver(station, MsgType.TRACKS, messages.encode_tracks(frame, []))
    assert station.telemetry.frame == 1 and station.tracks_frame == 1


def test_one_forged_frame_number_does_not_freeze_the_tracks(station):
    deliver(station, MsgType.TRACKS, messages.encode_tracks(0xFFFFFFFF, []))
    track = messages.TrackBox(1, (0, 0, 9, 9), True)
    deliver(station, MsgType.TRACKS, messages.encode_tracks(10, [track]))
    assert station.tracks_frame == 10


def test_older_messages_are_not_counted_and_patches_need_a_track(station):
    deliver(station, MsgType.TRACKS, messages.encode_tracks(10, []))
    deliver(station, MsgType.TRACKS, messages.encode_tracks(9, []))
    assert station.message_rate[MsgType.TRACKS].count(1.0) == 1
    patch = messages.PatchMsg(10, 150, 0, 7, (2, 2, 38, 28), webp(36, 26))
    deliver(station, MsgType.ROI_PATCH, messages.encode_patch(patch))
    assert station.patches == {}


def test_stats_sum_up_what_arrived(station):
    assert station.stats(1.0)["fovea_silent_s"] is None
    deliver(station, MsgType.TELEMETRY, messages.encode_telemetry(
        messages.TelemetryMsg(60, 46.0, 7.0, 150, 10, 0, 0, 0)), now_s=1.0)
    deliver(station, MsgType.BG_KEYFRAME, messages.encode_keyframe(messages.KeyframeMsg(
        30, 150, 0, cv2.imencode(".webp", np.zeros((270, 480), np.uint8))[1].tobytes())),
        now_s=1.2)
    deliver(station, MsgType.TELEMETRY, b"short", now_s=1.3)
    s = station.stats(1.5)
    assert s["keyframe_age_s"] == pytest.approx(1.0)
    assert s["fovea_silent_s"] == pytest.approx(0.2)
    assert s["telemetry_hz"] == 1 and s["bad_messages"] == 1
    assert s["fovea_kbps"] > 0 and s["baseline_kbps"] == 0


def test_receiver_thread_receives_reports_and_stops(station):
    thread = ReceiverThread(station)
    thread.start()
    try:
        port = station.sockets["fovea"].getsockname()[1]
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        tx = Packetizer()
        body = messages.encode_tracks(5, [messages.TrackBox(2, (10, 10, 40, 30), True)])
        for datagram in tx.packetize(MsgType.TRACKS, 0, body):
            sender.sendto(datagram, ("127.0.0.1", port))
        sender.close()
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            view, full_frame, stats = thread.snapshot()
            if view.tracks:
                break
            time.sleep(0.01)
        assert view.tracks_frame == 5 and view.tracks[0].id == 2
        assert full_frame is None and stats["tracks_hz"] >= 1
        assert parse_packet(station.reports.recv(2048)).header.type == MsgType.RECEIVER_REPORT
    finally:
        thread.stop()
    assert not thread.is_alive() and thread.error is None


@pytest.mark.parametrize("alt, heading", [
    (0.0, 0.0), (-5.0, 0.0), (float("nan"), 0.0), (float("inf"), 0.0), (1e30, 0.0),
    (150.0, float("nan")), (150.0, float("inf")),
])
def test_impossible_altitude_or_heading_is_dropped(station, alt, heading):
    deliver(station, MsgType.TELEMETRY, messages.encode_telemetry(
        messages.TelemetryMsg(10, 46.0, 7.0, alt, 10, heading, 0, 0)))
    deliver(station, MsgType.TRACKS, messages.encode_tracks(
        10, [messages.TrackBox(7, (10, 10, 30, 20), True)]))
    deliver(station, MsgType.ROI_PATCH, messages.encode_patch(
        messages.PatchMsg(10, alt, heading, 7, (2, 2, 38, 28), webp(36, 26))))
    deliver(station, MsgType.BG_KEYFRAME, messages.encode_keyframe(messages.KeyframeMsg(
        10, alt, heading, cv2.imencode(".webp", np.zeros((270, 480), np.uint8))[1].tobytes())))
    assert station.bad_messages == 3
    assert station.telemetry is None and station.patches == {} and station.keyframe is None


def test_a_patch_much_larger_than_its_box_is_dropped(station):
    deliver(station, MsgType.TRACKS, messages.encode_tracks(
        1, [messages.TrackBox(7, (10, 10, 30, 20), True)]))
    deliver(station, MsgType.ROI_PATCH, messages.encode_patch(
        messages.PatchMsg(1, 150, 0, 7, (0, 0, 640, 360), webp(640, 360))))
    assert station.patches == {} and station.bad_messages == 1


def test_snapshot_needs_the_window(tmp_path):
    import subprocess
    import sys
    result = subprocess.run([sys.executable, "-m", "aegislink.gcs.main", "--no-view",
                             "--snapshot", str(tmp_path / "x.png")], capture_output=True, timeout=30)
    assert result.returncode == 2


def telemetry_at(frame, lat=46.0, lon=7.0, alt=150.0, heading=0.0):
    return messages.encode_telemetry(messages.TelemetryMsg(frame, lat, lon, alt, 10, heading, 0, 0))


def centre_box(dx=0.0, dy=0.0):
    cx, cy = compositor.CAMERA.cx + dx, compositor.CAMERA.cy + dy
    return (round(cx - 15), round(cy - 8), round(cx + 15), round(cy + 8))


def test_matched_boxes_feed_the_ghost_tracks_in_ground_metres(station):
    deliver(station, MsgType.TELEMETRY, telemetry_at(30, heading=90.0), now_s=1.0)
    # 100 px right of the centre with the nose east: 100 x 0.135 m south of the UAV.
    deliver(station, MsgType.TRACKS, messages.encode_tracks(30, [
        messages.TrackBox(4, centre_box(dx=100), True),
        messages.TrackBox(5, centre_box(dx=-50), False)]), now_s=1.0)
    (e,) = station.ghost_estimates(1.0)
    mpp = compositor.CAMERA.metres_per_pixel(150.0)
    assert e.id == 4 and e.state == "LIVE"
    assert e.mean == pytest.approx((0.0, -100 * mpp), abs=0.1)


def test_a_track_list_before_its_telemetry_is_measured_when_the_telemetry_comes(station):
    deliver(station, MsgType.TELEMETRY, telemetry_at(30), now_s=1.0)
    deliver(station, MsgType.TRACKS, messages.encode_tracks(31, [
        messages.TrackBox(4, centre_box(), True)]), now_s=1.03)
    assert station.ghosts.tracks == {}
    deliver(station, MsgType.TELEMETRY, telemetry_at(31), now_s=1.04)
    assert station.ghosts.tracks[4].t == pytest.approx(31 / 30)


def test_tracks_turn_ghost_when_the_link_goes_silent(station):
    for frame in range(30, 60):
        now = frame / 30
        deliver(station, MsgType.TELEMETRY, telemetry_at(frame, lat=46.0 + frame * 1e-6), now)
        deliver(station, MsgType.TRACKS, messages.encode_tracks(
            frame, [messages.TrackBox(9, centre_box(), True)]), now)
    last = 59 / 30
    assert station.uav_time(last + 2.0) == pytest.approx(last + 2.0)
    (e,) = station.ghost_estimates(last + 0.1)
    assert e.state == "LIVE"
    (e,) = station.ghost_estimates(last + 2.0)
    assert e.state == "GHOST" and e.tslf_s == pytest.approx(2.0)
    # The UAV (and the vehicle under it) moves north at about 3.3 m/s: so does the prediction.
    assert e.velocity[1] == pytest.approx(3.34, abs=0.3)
    stats = station.stats(last + 2.0)
    assert stats["ghost_tracks"] == 1 and stats["filtered_tracks"] == 1
    assert stats["max_tslf_s"] == pytest.approx(2.0)
    assert stats["max_ellipse_m2"] > 10
    assert station.ghost_estimates(last + 10.5) == [] and station.ghosts.tracks == {}


def test_a_restarted_uav_clears_the_ghost_tracks(station):
    deliver(station, MsgType.TELEMETRY, telemetry_at(900), now_s=1.0)
    deliver(station, MsgType.TRACKS, messages.encode_tracks(
        900, [messages.TrackBox(1, centre_box(), True)]), now_s=1.0)
    assert station.ghosts.tracks
    deliver(station, MsgType.TELEMETRY, telemetry_at(0), now_s=1.1)
    assert station.ghosts.tracks == {}
    assert station.stats(1.2)["ghost_tracks"] == 0


def test_view_with_a_time_holds_the_predicted_tracks(station):
    deliver(station, MsgType.TELEMETRY, telemetry_at(30), now_s=1.0)
    deliver(station, MsgType.TRACKS, messages.encode_tracks(
        30, [messages.TrackBox(2, centre_box(), True)]), now_s=1.0)
    assert station.view().ghosts == []
    (e,) = station.view(1.5).ghosts
    assert e.id == 2 and e.state == "GHOST" and len(e.path) == 20


def test_track_log_writes_the_predictions_30_times_a_second(tmp_path):
    import json
    path = tmp_path / "tracks.jsonl"
    s = GroundStation(0, 0, 9, track_log=path)
    try:
        s.log_tracks_if_due(0.5)            # no telemetry yet: nothing to write
        deliver(s, MsgType.TELEMETRY, telemetry_at(30), now_s=1.0)
        deliver(s, MsgType.TRACKS, messages.encode_tracks(
            30, [messages.TrackBox(2, centre_box(), True)]), now_s=1.0)
        for i in range(31):
            s.log_tracks_if_due(1.0 + i / 60 + (0.002 if i else 0))  # 60 Hz: every other writes
        lines = [json.loads(line) for line in path.read_text().splitlines()]
    finally:
        s.close()
    assert len(lines) == 16
    first, last = lines[0], lines[-1]
    assert first["t"] == pytest.approx(1.0) and first["origin"] == pytest.approx([46.0, 7.0])
    assert last["t"] == pytest.approx(1.502, abs=1e-3)
    (track,) = last["tracks"]
    assert track["id"] == 2 and track["state"] == "GHOST" and track["tslf"] == pytest.approx(0.502)
    assert track["fix"] == pytest.approx([1.0, 0.0, 0.0], abs=0.1) and len(track["cov"]) == 3


def test_telemetry_pose_interpolates_the_heading_the_short_way(station):
    deliver(station, MsgType.TELEMETRY, telemetry_at(10, alt=100.0, heading=350.0))
    deliver(station, MsgType.TELEMETRY, telemetry_at(12, alt=120.0, heading=10.0))
    pose = station.poses.telemetry_pose(11)
    assert pose.alt == pytest.approx(110.0) and pose.heading_deg == pytest.approx(0.0, abs=1e-9)
    assert station.poses.telemetry_pose(12).heading_deg == pytest.approx(10.0)
    assert station.poses.telemetry_pose(13) is None


def test_forged_track_lists_cannot_grow_the_filters_without_bound(station):
    """A track list may hold 255 ids; forged lists with fresh ids at the held frame would each
    add 255 filters, and every drawn frame works through all of them."""
    from aegislink.gcs.ghost_tracks import MAX_TRACKS
    # On the snapshot's own clock, so that the forged tracks are still alive when drawn.
    now = time.monotonic()
    deliver(station, MsgType.TELEMETRY, telemetry_at(30), now_s=now)
    for i in range(20):
        forged = [messages.TrackBox(1000 + 255 * i + k, centre_box(dx=k % 50, dy=k // 50), True)
                  for k in range(255)]
        deliver(station, MsgType.TRACKS, messages.encode_tracks(30, forged), now_s=now)
    assert len(station.ghosts.tracks) <= MAX_TRACKS
    receiver = ReceiverThread(station)
    start = time.perf_counter()
    for _ in range(10):
        view, _, stats = receiver.snapshot()
        compositor.render_fovea(view, (1280, 720))
    # Unbounded, these 5,100 filters take over half a second a frame; capped, about 13 ms here.
    # The bound is loose for slower CI machines.
    assert (time.perf_counter() - start) / 10 < 0.050
    assert stats["filtered_tracks"] == len(view.ghosts) == MAX_TRACKS


def test_a_real_track_survives_a_flood_of_forged_ids(station):
    deliver(station, MsgType.TELEMETRY, telemetry_at(30), now_s=1.0)
    deliver(station, MsgType.TELEMETRY, telemetry_at(31), now_s=1.03)
    deliver(station, MsgType.TRACKS, messages.encode_tracks(
        31, [messages.TrackBox(7, centre_box(), True)]), now_s=1.03)
    forged = [messages.TrackBox(100 + k, centre_box(dx=k), True) for k in range(255)]
    deliver(station, MsgType.TRACKS, messages.encode_tracks(30, forged), now_s=1.04)  # OLD: ignored
    deliver(station, MsgType.TELEMETRY, telemetry_at(32), now_s=1.06)
    deliver(station, MsgType.TRACKS, messages.encode_tracks(
        32, [messages.TrackBox(7, centre_box(), True)] + forged[:200]), now_s=1.06)
    # The real track has the newest fix, so it is not the one replaced.
    assert 7 in station.ghosts.tracks
