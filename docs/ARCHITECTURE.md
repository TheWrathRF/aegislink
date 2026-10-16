# Architecture

AegisLink's processes talk over UDP using one wire protocol, implemented in C++ (`cpp/core`) and
used from Python through pybind11 (`aegislink._core`).

## Layout
- `cpp/core/`: the `aegis_core` static library (protocol, CRC, reassembler); headers in
  `include/aegis/`.
- `cpp/proxy/`: `ew_chaos_proxy`, the link simulator. Everything but `main.cpp` is the
  `aegis_proxy` static library, so the tests can link it.
- `cpp/bindings/aegis_core.cpp`: the pybind11 module `aegislink._core`.
- `cpp/tests/`: C++ unit tests, one executable per module, with a tiny runner (`test_runner.hpp`).
- `python/aegislink/`: the Python package. `common/proxy_control.py` talks to the proxy's
  control port; `tools/loopback_check.py` sends messages over loopback UDP and checks that they
  arrive intact; `tools/link_meter.py` measures a proxy link.
- `tests/`: pytest.
- One `CMakeLists.txt` builds the C++ tests and the proxy (plain CMake), and the Python module
  plus the proxy when scikit-build-core runs it (`SKBUILD` is set, the tests are skipped, and the
  proxy is installed next to the Python scripts).

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

## Chaos proxy (`cpp/proxy`)
`ew_chaos_proxy --link LISTEN:FORWARD [--link ...] [--control PORT] [--seed N]`

- Each `--link` is one direction of the radio link: it receives on `127.0.0.1:LISTEN` and sends
  on to `127.0.0.1:FORWARD`, from the same socket. Links are numbered from 1 in the order given.
  The proxy does not parse the datagrams.
- One thread runs one `select()` loop over all sockets. A received datagram goes through its
  link (`Link::admit`); a forwarded one waits in a priority queue ordered by release time (ties
  keep arrival order), and the loop sleeps until the next release, stats report or datagram.
- `Link` applies, in order:
  1. **Blackout**: drops everything until it ends.
  2. **Burst loss** (`GilbertElliott`): a good state that passes every packet and a bad state
     that drops every packet. From loss rate `l` and mean burst length `L`: `r = 1/L` (bad to
     good), `p = r * l / (1 - l)` (good to bad). `p <= 1` limits the loss to `L / (L + 1)`.
     Changing either value restarts the chain in the good state.
  3. **Rate limit** (`TokenBucket`, `kbps` > 0): one full datagram (1230 B) deep. A packet that
     finds too few tokens borrows them and leaves once the debt is paid, so the bucket is also
     the FIFO queue; over any interval T at most 1230 + rate x T bytes leave. A packet that would
     wait longer than `queue_ms` is dropped (drop-tail). The bucket is replaced (full) only when
     the rate changes.
  4. **Delay and jitter**: `delay_ms` plus a uniform extra in `[0, jitter_ms)`; jitter can reorder.
- Each link has its own random generator, seeded with `seed + index`, and draws doubles from the
  top 53 bits, so a seed gives the same fates with any compiler.
- **Control** (`control.hpp`): one text command per datagram, for example
  `link=1 loss_pct=30 burst=4 kbps=50 queue_ms=250 delay_ms=20 jitter_ms=10 blackout_ms=5000`.
  Every key but `link` is optional. The reply is `ok` and all of the link's settings in the same
  form (`blackout_ms` is the time left), or `error: <reason>` with nothing changed: the command
  is applied to a copy of the link first.
- **Stats**: every 250 ms, to the port that sent the last command:
  `stats uptime_ms=N`, then per link `link=1 in= out= drop_blackout= drop_loss= drop_queue=
  send_errors= in_kbps= out_kbps= blackout_ms=` (rates over the last interval, in datagram bytes).
- `UdpSocket` wraps Winsock2 and POSIX sockets: bound to 127.0.0.1, non-blocking, 4 MB buffers.
  `receive` skips failed reads, because on Windows a send to a closed port makes the ICMP reply
  show up as an error on the next read.
- On Windows the proxy calls `timeBeginPeriod(1)`; otherwise `select()` wakes up to 8.5 ms late.

## Python module `aegislink._core`
Mirrors the C++ API: `crc32`, `MsgType` (IntEnum), `ParseError` (Enum), `Header`,
`encode_packet`, `parse_packet` (returns a `Packet` with its own copy of the payload),
`Packetizer`, `Reassembler` (`push`, `expire`, `pending`, `stats`), and the constants
`HEADER_SIZE`, `MAX_PAYLOAD`, `MAX_DATAGRAM`, `MAX_MESSAGE`, `MAGIC`, `VERSION`. Byte arguments
accept any contiguous buffer (bytes, bytearray, memoryview) without a copy; `std::invalid_argument`
becomes `ValueError`.

## Proxy tools (Python)
- `common/proxy_control.py`: `ProxyControl(port)` sends commands (`configure(1, loss_pct=30,
  burst=4)` returns the reply's settings as a dict, an `error:` reply raises `ProxyError`) and
  reads stats (`read_stats()`); the same socket receives both, so replies skip stats reports.
  `wait_ready()` retries until a just-started proxy answers.
- `tools/link_meter.py`: `measure()` sends numbered datagrams (built with `Packetizer`, one
  message each) at a fixed rate through a link while a thread receives them. Loss and burst
  lengths come from the missing `seq` numbers; delay from the header timestamp (same clock on
  both ends); throughput from the received bytes between the first and the last arrival, the
  first datagram excluded because it only starts the clock. The command line sets the link
  through the control port first, then prints the measurement next to the settings.
