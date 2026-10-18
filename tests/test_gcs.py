"""The ground station's handling of what arrives, and the viewer, without the UAV."""

import socket

import cv2
import numpy as np
import pytest

from aegislink._core import MsgType, Packetizer, parse_packet
from aegislink.common import messages
from aegislink.gcs import viewer
from aegislink.gcs.receiver import GroundStation


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


def test_viewer_draws_without_data_and_with_a_patch_at_the_edge(station):
    image = viewer.compose(station, 1.0)
    assert image.shape == (viewer.PANE_HEIGHT + viewer.HUD_HEIGHT, 2 * viewer.PANE_WIDTH, 3)
    track = messages.TrackBox(3, (1270, 700, 1300, 730), True)
    deliver(station, MsgType.TRACKS, messages.encode_tracks(1, [track]))
    patch = messages.PatchMsg(1, 150, 0, 3, (1240, 690, 1280, 720), webp(40, 30))
    deliver(station, MsgType.ROI_PATCH, messages.encode_patch(patch))
    fovea = viewer.render_fovea(station)
    # The patch is pasted centred on the track's box, cut at the frame edge.
    assert tuple(fovea[715, 1279]) == pytest.approx((0, 0, 255), abs=8)
    assert viewer.compose(station, 1.0).shape == image.shape


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
    viewer.compose(station, 1.0)


def test_a_restarted_uav_is_followed(station):
    for frame in (500, 0, 1):
        deliver(station, MsgType.TELEMETRY, messages.encode_telemetry(
            messages.TelemetryMsg(frame, 0, 0, 0, 0, 0, 0, 0)))
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
