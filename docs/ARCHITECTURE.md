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
  control port; `common/geo.py` converts local metres to latitude/longitude;
  `tools/loopback_check.py` sends messages over loopback UDP and checks that they arrive intact;
  `tools/link_meter.py` measures a proxy link; `uav/` is the UAV side (synthetic world, camera,
  flight, detector, tracker).
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

## Synthetic world (`uav/sim_world.py`)
Everything uses a local frame in metres: x = east, y = north, origin at the map centre
(`common/geo.py` turns it into latitude/longitude with a flat-earth approximation around
`origin_latlon`, accurate to millimetres over a few kilometres).

- **Ground**: one BGR raster (default 1000 m x 1000 m at 0.25 m per pixel, 4000 x 4000 px),
  drawn once from a seed: smooth noise blends grass, dry grass and soil, and noise at 8 m, 2 m and
  0.7 m gives texture that is fixed to the ground. Raster column = (east + size/2) / res, row =
  (size/2 - north) / res.
- **Roads**: closed loops (a circle with random 2nd-4th harmonic wobble), 7 m wide, two lanes,
  right-hand traffic. Loops may cross; there are no junctions, so vehicles on crossing roads drive
  through each other.
- **Occluders**: two tree clumps (radius 5.5-7 m) over every road and one bridge (12 m x 30 m) over
  the first, plus scattered trees off the roads. They are drawn after the vehicles, so they hide
  them. A `cover` raster marks the covered ground; `visible_fraction` samples 5 x 3 points over a
  vehicle's footprint against it.
- **Vehicles** (default 16 per loop, 15 % trucks of 8 x 2.5 m, cars 4.5 x 1.9 m): each has a
  position `s` along its loop and a direction. Per step (1/30 s): a vehicle aims for its desired
  speed (7-13 m/s), or for 0 when it is due to stop (every 15-45 s, for 2-6 s), but never faster
  than `(gap - 3 m) / 1.2 s` to the vehicle ahead in its lane; it accelerates at up to 2 m/s^2
  and brakes at up to 6 m/s^2, and its move is cut so the bumper gap never drops below 1 m.
- **Rendering** (`render(camera, pose)`): one `warpAffine` of the ground raster through the
  camera's pixel -> ground -> raster matrix, then the vehicles (body and roof as anti-aliased
  polygons), then trees and bridges near the view. It returns the image and a `Truth` per vehicle
  near the view: its axis-aligned pixel box clipped to the image, the share of the box inside the
  image (`in_frame`) and `visible`. About 4 ms per 1280 x 720 frame.

## Camera and flight (`uav/camera.py`, `uav/telemetry.py`)
- `NadirCamera` (1280 x 720, 60 degree horizontal field of view) looks straight down from a
  gimbal, so over flat ground pixel -> ground is one affine map: scale `alt / focal_px` metres per
  pixel, rotation by the heading (image up = heading), shift by the UAV position. Pixel centres sit
  on integer coordinates, as in OpenCV. `motion_matrix(old, new)` is the affine map from a ground
  point's pixel in one frame to its pixel in the next: the image motion caused by the camera.
- `Flight` is a multirotor that follows one vehicle: wanted velocity = the vehicle's velocity +
  0.4/s x the position error (at most 18 m/s), reached with a 1 s time constant and at most
  3 m/s^2. The nose turns towards the direction of flight at up to 25 deg/s (only above 1.5 m/s).
  Altitude is 150 m +- 10 m over 90 s. Pitch and roll follow from the accelerations (forward
  acceleration and speed x yaw rate, against gravity).
- `Telemetry`: time, latitude, longitude, altitude, ground speed, heading, pitch, roll.

## Detector and tracker (`uav/detector.py`, `uav/tracker.py`)
- `SimulatedDetector` turns truths into detections. A vehicle with less than half its box in the
  image, or less than 40 % visible, is never found; one at least 90 % visible is found with
  probability 0.95, falling linearly in between. Each box edge gets Gaussian noise with sigma
  1 px + 4 % of the box size. False boxes appear at 0.05 per frame with scores below 0.6.
- `IouTracker`, per frame:
  1. each track's box moves with the camera (`motion_matrix`, scale included), then by the
     vehicle's own image velocity, so a coasting track stays on a parked vehicle while the camera
     turns, and does not overshoot when the camera stops turning;
  2. track/detection pairs are taken greedily by IoU, largest first, down to 0.2;
  3. a matched track takes the detection's box and updates its velocity (0.8 old + 0.2 measured,
     the measurement spread over the frames since the last match); an unmatched one coasts;
  4. an unmatched detection starts a track, which gets an id after 3 hits in a row (one miss
     before that deletes it); a confirmed track is deleted after 30 missed frames (1 s).

## UAV loop and ground-truth log (`uav/main.py`)
`python -m aegislink.uav.main [--seconds 60] [--seed 1] [--no-preview] [--log PATH]`

- `UavSim.step()` renders the current state, detects, tracks (with the camera motion since the
  last frame), then advances the world and the flight by 1/30 s. A seed fixes the whole run.
- The preview (OpenCV window, paced to real time) draws the confirmed tracks: green when matched
  this frame, amber while coasting, with ids and a line of telemetry. `q` or Esc quits.
- The ground-truth log (default `logs/ground_truth_<date>_<time>.jsonl`) holds a header line
  (seed, fps, map size, origin, followed vehicle, camera) and then one line per frame: the UAV pose
  with latitude/longitude, pitch, roll and speed, and every vehicle near the view with its
  position, heading, speed, `visible`, `in_frame` and pixel box. All of one line is from the same
  instant.
- `TrackScore` compares tracks with the truth over a run: a vehicle fully in the image and at
  least 90 % visible counts, and is covered when a track matched this frame overlaps it (IoU >=
  0.3). A new id for a vehicle covered within the last 30 frames is an id switch; after a longer
  gap (hidden or out of the image longer than a track lives) it is a reacquisition. Seeds 1-5,
  60 s each: covered 94.7-95.0 % of 2740-5913 vehicle frames (the misses are the detector's 5 %),
  0 id switches, 20 reacquisitions, 0 false track frames.
