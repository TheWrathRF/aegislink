"""What the UAV puts into its messages."""

from aegislink._core import MsgType, parse_packet
from aegislink.common import messages
from aegislink.uav.encoder import decode_image
from aegislink.uav.main import UavSim
from aegislink.uav.streamer import BaselineSender, frame_data


def test_frame_data_holds_this_frame():
    sim = UavSim(seed=1)
    for _ in range(10):  # tracks are confirmed after 3 hits
        r = sim.step()
    data = frame_data(r, 123)
    tel = messages.decode_telemetry(data.telemetry)
    assert tel.frame == r.index
    assert abs(tel.lat - r.telemetry.lat) < 1e-7
    frame, tracks = messages.decode_tracks(data.tracks)
    assert frame == r.index and [t.id for t in tracks] == [t.id for t in r.tracks]
    seen = [t.id for t in r.tracks if t.misses == 0]
    assert seen and sorted(data.patches) == sorted(seen)
    patch = messages.decode_patch(data.patches[seen[0]]())
    x1, y1, x2, y2 = patch.box
    assert decode_image(patch.image).shape == (y2 - y1, x2 - x1, 3)
    assert abs(patch.alt_m - r.pose.alt) < 1e-3
    keyframe = messages.decode_keyframe(data.keyframe())
    assert keyframe.frame == r.index
    assert decode_image(keyframe.image, gray=True).shape == (270, 480)


def test_baseline_is_one_jpeg_per_frame():
    r = UavSim(seed=1).step()
    datagrams = BaselineSender().datagrams(r, 5)
    packets = [parse_packet(d) for d in datagrams]
    assert all(p.header.type == MsgType.FULL_FRAME and p.header.timestamp_us == 5 for p in packets)
    body = messages.decode_full_frame(b"".join(p.payload for p in packets))
    assert body.frame == 0
    assert decode_image(body.image).shape == r.frame.shape
