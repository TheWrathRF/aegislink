"""The C++ protocol, seen from Python, must match a plain struct + zlib reference of the format."""

import random
import struct
import zlib

import pytest

from aegislink import _core
from aegislink._core import Header, MsgType, ParseError

# magic, version, type, seq, timestamp_us, msg_id, frag_index, frag_count, payload_len, crc32
REFERENCE_HEADER = struct.Struct("<HBBIQIHHHI")


def reference_encode(header, payload):
    fields = (0xAE61, 1, int(header.type), header.seq, header.timestamp_us, header.msg_id,
              header.frag_index, header.frag_count, len(payload))
    without_crc = REFERENCE_HEADER.pack(*fields, 0)[:-4]
    crc = zlib.crc32(payload, zlib.crc32(without_crc))
    return without_crc + struct.pack("<I", crc) + payload


def random_header(rng):
    frag_count = rng.randint(1, 0xFFFF)
    return Header(type=MsgType(rng.randint(1, 6)), seq=rng.getrandbits(32),
                  timestamp_us=rng.getrandbits(64), msg_id=rng.getrandbits(32),
                  frag_index=rng.randrange(frag_count), frag_count=frag_count)


def test_constants():
    assert _core.HEADER_SIZE == REFERENCE_HEADER.size == 30
    assert _core.MAX_PAYLOAD == 1200
    assert _core.MAX_DATAGRAM == 1230


def test_crc32_matches_zlib():
    rng = random.Random(1)
    for size in [0, 1, 7, 1200, 5000]:
        data = rng.randbytes(size)
        assert _core.crc32(data) == zlib.crc32(data)
    assert _core.crc32(b"world", _core.crc32(b"hello ")) == zlib.crc32(b"hello world")


def test_encode_matches_reference():
    rng = random.Random(2)
    for _ in range(500):
        header = random_header(rng)
        payload = rng.randbytes(rng.randint(0, _core.MAX_PAYLOAD))
        assert _core.encode_packet(header, payload) == reference_encode(header, payload)


def test_parse_reference_packet():
    rng = random.Random(3)
    for _ in range(500):
        header = random_header(rng)
        payload = rng.randbytes(rng.randint(0, _core.MAX_PAYLOAD))
        packet = _core.parse_packet(reference_encode(header, payload))
        assert packet.error == ParseError.OK
        got = packet.header
        assert (got.type, got.seq, got.timestamp_us, got.msg_id, got.frag_index, got.frag_count) == (
            header.type, header.seq, header.timestamp_us, header.msg_id, header.frag_index,
            header.frag_count)
        assert got.payload_len == len(payload)
        assert packet.payload == payload


def test_parse_reports_errors():
    good = _core.encode_packet(Header(type=MsgType.TRACKS), b"abc")
    assert _core.parse_packet(good[:10]).error == ParseError.TOO_SHORT
    assert _core.parse_packet(good[:-1]).error == ParseError.BAD_LENGTH
    corrupted = bytearray(good)
    corrupted[-1] ^= 1
    assert _core.parse_packet(bytes(corrupted)).error == ParseError.BAD_CRC
    assert _core.parse_packet(bytes(corrupted)).payload == b""


def test_accepts_bytearray_and_memoryview():
    good = _core.encode_packet(Header(type=MsgType.TRACKS), b"abc")
    assert _core.parse_packet(bytearray(good)).error == ParseError.OK
    assert _core.parse_packet(memoryview(good)).payload == b"abc"


def test_encode_rejects_oversized_payload():
    with pytest.raises(ValueError):
        _core.encode_packet(Header(), bytes(_core.MAX_PAYLOAD + 1))


def test_packetize_and_reassemble_shuffled_50kb():
    rng = random.Random(4)
    message = rng.randbytes(50_000)
    packets = _core.Packetizer(first_seq=10).packetize(MsgType.FULL_FRAME, 99, message)
    assert len(packets) == 42
    assert all(len(p) <= _core.MAX_DATAGRAM for p in packets)
    rng.shuffle(packets)

    reassembler = _core.Reassembler()
    done = []
    for raw in packets:
        packet = _core.parse_packet(raw)
        assert packet.error == ParseError.OK
        result = reassembler.push(packet.header, packet.payload, 0)
        if result is not None:
            done.append(result)
    assert len(done) == 1
    assert done[0].data == message
    assert done[0].type == MsgType.FULL_FRAME
    assert done[0].timestamp_us == 99
    assert reassembler.stats.completed == 1
    assert reassembler.pending == 0


def test_reassembler_timeout_from_python():
    packets = _core.Packetizer().packetize(MsgType.ROI_PATCH, 0, bytes(3000))
    reassembler = _core.Reassembler(timeout_us=200_000)
    first = _core.parse_packet(packets[0])
    assert reassembler.push(first.header, first.payload, 1_000) is None
    reassembler.expire(201_001)
    assert reassembler.pending == 0
    assert reassembler.stats.expired == 1
