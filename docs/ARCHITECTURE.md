# Architecture

AegisLink's processes talk over UDP using one wire protocol, implemented in C++ (`cpp/core`) and
used from Python through pybind11 (`aegislink._core`).

## Layout
- `cpp/core/`: the `aegis_core` static library (protocol, CRC, reassembler); headers in
  `include/aegis/`.
- `cpp/bindings/aegis_core.cpp`: the pybind11 module `aegislink._core`.
- `cpp/tests/`: C++ unit tests, one executable per module, with a tiny runner (`test_runner.hpp`).
- `python/aegislink/`: the Python package; `tools/loopback_check.py` sends messages over
  loopback UDP and checks that they arrive intact.
- `tests/`: pytest.
- One `CMakeLists.txt` builds both the C++ tests (plain CMake) and the Python module (when
  scikit-build-core runs it, `SKBUILD` is set and the tests are skipped).

## Wire protocol (`cpp/core/include/aegis/protocol.hpp`)
Every datagram is a 30-byte little-endian header followed by at most 1200 bytes of payload, so a
datagram stays under the 1280-byte IPv6 minimum MTU and is never IP-fragmented.

| Offset | Field | Type | Meaning |
|---|---|---|---|
| 0 | magic | u16 | `0xAE61` |
| 2 | version | u8 | 1 |
| 3 | type | u8 | 1 TELEMETRY, 2 TRACKS, 3 ROI_PATCH, 4 BG_KEYFRAME, 5 FULL_FRAME, 6 RECEIVER_REPORT |
| 4 | seq | u32 | per-link packet counter (for loss measurement) |
| 8 | timestamp_us | u64 | UAV capture time |
| 16 | msg_id | u32 | shared by all fragments of one message |
| 20 | frag_index | u16 | 0 to frag_count - 1 |
| 22 | frag_count | u16 | at least 1 |
| 24 | payload_len | u16 | must equal the datagram size - 30 |
| 26 | crc32 | u32 | CRC-32 (zlib) over bytes 0-25, then the payload |

- `encode_packet` writes the fields with explicit shifts (byte order is fixed on any host) and
  throws `std::invalid_argument` for an oversized payload or invalid fragment fields.
- `parse_packet` checks, in order: size >= 30, magic, version, type, payload length, fragment
  fields, CRC. It returns a `ParseError` instead of throwing, so a receiver can count bad packets;
  the payload span points into the caller's datagram.
- `Packetizer` splits a message (up to 1200 x 65535 bytes) into datagrams for one link, numbering
  packets with `seq` and messages with `msg_id`. An empty message is one packet with no payload.
- The CRC detects corruption, not tampering: anyone on the path can recompute it.

## Reassembler (`cpp/core/include/aegis/reassembler.hpp`)
- Keeps one partial message per `msg_id`; fragments may arrive in any order. The message is
  returned by the `push` that delivers its last missing fragment.
- A partial whose first fragment is older than the timeout (200 ms) is dropped; `push` runs
  `expire` first, so a late fragment never completes an expired message.
- Completed `msg_id`s are remembered for one timeout, so late duplicates are counted, not
  mistaken for a new message.
- At most `max_pending` (64) partials exist; a new one evicts the oldest. This bounds memory when
  a peer sends many first fragments.
- Fragments whose type or frag_count disagree with the partial are dropped as inconsistent.
- Stats: completed, duplicates, inconsistent, expired, evicted.
- Time is the receiver's clock in microseconds, passed in by the caller (tests control it;
  Python passes `time.monotonic_ns() // 1000`).

## Python module `aegislink._core`
Mirrors the C++ API: `crc32`, `MsgType` (IntEnum), `ParseError` (Enum), `Header`,
`encode_packet`, `parse_packet` (returns a `Packet` with its own copy of the payload),
`Packetizer`, `Reassembler` (`push`, `expire`, `pending`, `stats`), and the constants
`HEADER_SIZE`, `MAX_PAYLOAD`, `MAX_DATAGRAM`, `MAX_MESSAGE`, `MAGIC`, `VERSION`. Byte arguments
accept any contiguous buffer (bytes, bytearray, memoryview) without a copy; `std::invalid_argument`
becomes `ValueError`.
