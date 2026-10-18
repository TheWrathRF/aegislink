"""The bodies of the messages the UAV and the GCS exchange (the part after the wire header).

All are fixed little-endian structs; image messages end with the encoded image bytes. Every
decoder checks the length first and raises ValueError for a body that does not fit, so a
receiver can count and drop it.

Image messages carry the camera's altitude and heading (from the barometer and the gimbal) but
not its position: the GCS takes the position from the telemetry of the same frame, so a spoofed
telemetry position reaches everything the GCS does with the images.
"""

import struct
from dataclasses import dataclass

from aegislink._core import MsgType

# frame, latitude and longitude in 1e-7 degrees (about 1 cm, as MAVLink does), altitude, speed,
# heading, pitch, roll
_TELEMETRY = struct.Struct("<Iiifffff")
_TRACKS_HEAD = struct.Struct("<IB")     # frame, number of tracks
_TRACK = struct.Struct("<HhhhhB")       # id, x1, y1, x2, y2 (pixels), flags
_PATCH_HEAD = struct.Struct("<IffHhhhh")  # frame, alt, heading, track id, crop box
_KEYFRAME_HEAD = struct.Struct("<Iff")  # frame, alt, heading
_FULL_FRAME_HEAD = struct.Struct("<I")  # frame
# GCS time, highest seq seen, packets and datagram bytes received (both since the start), and the
# smallest one-way delay since the last report (NO_DELAY if there was no sample). The counters
# are cumulative so that a lost report loses nothing but its delay sample.
_REPORT = struct.Struct("<QIIQq")
NO_DELAY = -2**63

_SEEN = 1
_MAX_TRACKS = 255


@dataclass
class TelemetryMsg:
    frame: int
    lat: float
    lon: float
    alt_m: float
    speed_mps: float
    heading_deg: float
    pitch_deg: float
    roll_deg: float


@dataclass
class TrackBox:
    id: int
    box: tuple          # (x1, y1, x2, y2) in pixels of the frame
    seen: bool          # matched a detection in this frame (otherwise coasting)


@dataclass
class PatchMsg:
    frame: int
    alt_m: float
    heading_deg: float
    track_id: int
    box: tuple          # where the crop was taken, in pixels of the frame
    image: bytes        # WebP


@dataclass
class KeyframeMsg:
    frame: int
    alt_m: float
    heading_deg: float
    image: bytes        # gray, downscaled WebP


@dataclass
class FullFrameMsg:
    frame: int
    image: bytes        # JPEG


@dataclass
class ReceiverReport:
    time_us: int
    highest_seq: int
    received: int
    received_bytes: int
    min_delay_us: int


def _i16(x):
    return max(-32768, min(32767, round(x)))


def _check(data, size, exact=True):
    if len(data) < size or (exact and len(data) != size):
        raise ValueError(f"body of {len(data)} bytes, expected {size}")


def encode_telemetry(t):
    return _TELEMETRY.pack(t.frame, round(t.lat * 1e7), round(t.lon * 1e7), t.alt_m, t.speed_mps,
                           t.heading_deg, t.pitch_deg, t.roll_deg)


def decode_telemetry(data):
    _check(data, _TELEMETRY.size)
    frame, lat, lon, *rest = _TELEMETRY.unpack(data)
    return TelemetryMsg(frame, lat / 1e7, lon / 1e7, *rest)


def encode_tracks(frame, tracks):
    tracks = tracks[:_MAX_TRACKS]
    parts = [_TRACKS_HEAD.pack(frame, len(tracks))]
    for t in tracks:
        parts.append(_TRACK.pack(t.id % 65536, *(_i16(v) for v in t.box), _SEEN if t.seen else 0))
    return b"".join(parts)


def decode_tracks(data):
    """Returns (frame, [TrackBox, ...])."""
    _check(data, _TRACKS_HEAD.size, exact=False)
    frame, count = _TRACKS_HEAD.unpack_from(data)
    _check(data, _TRACKS_HEAD.size + count * _TRACK.size)
    tracks = []
    for i in range(count):
        id_, x1, y1, x2, y2, flags = _TRACK.unpack_from(data, _TRACKS_HEAD.size + i * _TRACK.size)
        tracks.append(TrackBox(id_, (x1, y1, x2, y2), bool(flags & _SEEN)))
    return frame, tracks


def encode_patch(p):
    return _PATCH_HEAD.pack(p.frame, p.alt_m, p.heading_deg, p.track_id % 65536,
                            *(_i16(v) for v in p.box)) + p.image


def decode_patch(data):
    _check(data, _PATCH_HEAD.size + 1, exact=False)
    frame, alt, heading, track_id, *box = _PATCH_HEAD.unpack_from(data)
    return PatchMsg(frame, alt, heading, track_id, tuple(box), bytes(data[_PATCH_HEAD.size:]))


def encode_keyframe(k):
    return _KEYFRAME_HEAD.pack(k.frame, k.alt_m, k.heading_deg) + k.image


def decode_keyframe(data):
    _check(data, _KEYFRAME_HEAD.size + 1, exact=False)
    frame, alt, heading = _KEYFRAME_HEAD.unpack_from(data)
    return KeyframeMsg(frame, alt, heading, bytes(data[_KEYFRAME_HEAD.size:]))


def encode_full_frame(f):
    return _FULL_FRAME_HEAD.pack(f.frame) + f.image


def decode_full_frame(data):
    _check(data, _FULL_FRAME_HEAD.size + 1, exact=False)
    (frame,) = _FULL_FRAME_HEAD.unpack_from(data)
    return FullFrameMsg(frame, bytes(data[_FULL_FRAME_HEAD.size:]))


def encode_report(r):
    return _REPORT.pack(r.time_us, r.highest_seq, r.received, r.received_bytes, r.min_delay_us)


def decode_report(data):
    _check(data, _REPORT.size)
    return ReceiverReport(*_REPORT.unpack(data))


_DECODERS = {
    MsgType.TELEMETRY: decode_telemetry,
    MsgType.TRACKS: decode_tracks,
    MsgType.ROI_PATCH: decode_patch,
    MsgType.BG_KEYFRAME: decode_keyframe,
    MsgType.FULL_FRAME: decode_full_frame,
    MsgType.RECEIVER_REPORT: decode_report,
}


def decode(msg_type, data):
    """Decodes a body of the given MsgType; raises ValueError if it does not fit."""
    return _DECODERS[msg_type](data)
