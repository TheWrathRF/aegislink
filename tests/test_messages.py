"""Message bodies: exact sizes, round trips, and rejection of bad lengths."""

import pytest

from aegislink.common import messages as m


def test_telemetry_round_trip_keeps_centimetre_position():
    tel = m.TelemetryMsg(frame=1234, lat=47.3977419, lon=8.5455938, alt_m=151.25,
                         speed_mps=11.5, heading_deg=271.5, pitch_deg=-3.25, roll_deg=8.5)
    data = m.encode_telemetry(tel)
    assert len(data) == 32
    back = m.decode_telemetry(data)
    assert back.frame == 1234
    # 1e-7 degree is about 1.1 cm of latitude.
    assert abs(back.lat - tel.lat) < 1e-7 and abs(back.lon - tel.lon) < 1e-7
    for name in ("alt_m", "speed_mps", "heading_deg", "pitch_deg", "roll_deg"):
        assert getattr(back, name) == pytest.approx(getattr(tel, name), abs=1e-4)


def test_negative_latitude_and_longitude():
    tel = m.TelemetryMsg(7, -33.8688197, -151.2092955, 100, 0, 0, 0, 0)
    back = m.decode_telemetry(m.encode_telemetry(tel))
    assert abs(back.lat - tel.lat) < 1e-7 and abs(back.lon - tel.lon) < 1e-7


def test_tracks_round_trip_and_size():
    tracks = [m.TrackBox(1, (10.4, 20.6, 50.0, 40.0), True),
              m.TrackBox(70000, (-30.0, 700.2, 12.0, 719.0), False)]
    data = m.encode_tracks(99, tracks)
    assert len(data) == 5 + 11 * 2
    frame, back = m.decode_tracks(data)
    assert frame == 99
    assert back[0] == m.TrackBox(1, (10, 21, 50, 40), True)
    # Ids wrap at 16 bits; boxes are whole pixels and may leave the image.
    assert back[1] == m.TrackBox(70000 % 65536, (-30, 700, 12, 719), False)


def test_no_tracks():
    assert m.decode_tracks(m.encode_tracks(5, [])) == (5, [])


def test_patch_and_keyframe_round_trip():
    patch = m.PatchMsg(frame=3, alt_m=150.5, heading_deg=90.0, track_id=12,
                       box=(100, 200, 150, 240), image=b"RIFF....WEBP")
    assert m.decode_patch(m.encode_patch(patch)) == patch
    key = m.KeyframeMsg(frame=4, alt_m=149.0, heading_deg=359.5, image=b"x" * 5000)
    assert m.decode_keyframe(m.encode_keyframe(key)) == key
    full = m.FullFrameMsg(frame=5, image=b"\xff\xd8jpeg")
    assert m.decode_full_frame(m.encode_full_frame(full)) == full


def test_receiver_report_round_trip():
    report = m.ReceiverReport(time_us=2**40 + 5, highest_seq=4_000_000_000, received=123456,
                              received_bytes=2**33, min_delay_us=-1500)
    data = m.encode_report(report)
    assert len(data) == 32
    assert m.decode_report(data) == report


@pytest.mark.parametrize("decode, good", [
    (m.decode_telemetry, m.encode_telemetry(m.TelemetryMsg(1, 0, 0, 0, 0, 0, 0, 0))),
    (m.decode_tracks, m.encode_tracks(1, [m.TrackBox(1, (0, 0, 1, 1), True)])),
    (m.decode_patch, m.encode_patch(m.PatchMsg(1, 1, 1, 1, (0, 0, 1, 1), b"img"))),
    (m.decode_keyframe, m.encode_keyframe(m.KeyframeMsg(1, 1, 1, b"img"))),
    (m.decode_full_frame, m.encode_full_frame(m.FullFrameMsg(1, b"img"))),
    (m.decode_report, m.encode_report(m.ReceiverReport(1, 1, 1, 1, 1))),
])
def test_bad_lengths_raise_value_error(decode, good):
    decode(good)
    with pytest.raises(ValueError):
        decode(good[:3])
    if decode in (m.decode_telemetry, m.decode_tracks, m.decode_report):
        with pytest.raises(ValueError):
            decode(good + b"\x00")


def test_track_count_must_match_the_length():
    data = bytearray(m.encode_tracks(1, [m.TrackBox(1, (0, 0, 1, 1), True)]))
    data[4] = 2  # claims two tracks, carries one
    with pytest.raises(ValueError):
        m.decode_tracks(bytes(data))


def test_images_need_a_body():
    with pytest.raises(ValueError):
        m.decode_patch(m.encode_patch(m.PatchMsg(1, 1, 1, 1, (0, 0, 1, 1), b"")))


def test_decode_dispatches_on_type():
    from aegislink._core import MsgType
    tel = m.TelemetryMsg(1, 0, 0, 0, 0, 0, 0, 0)
    assert m.decode(MsgType.TELEMETRY, m.encode_telemetry(tel)).frame == 1
    assert m.decode(MsgType.TRACKS, m.encode_tracks(2, [])) == (2, [])
