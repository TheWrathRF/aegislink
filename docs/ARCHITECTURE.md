# Architecture

AegisLink's processes talk over UDP using one wire protocol, implemented in C++ (`cpp/core`) and
used from Python through pybind11 (`aegislink._core`), which also holds the Kalman filter that the
ground station follows its tracks with.

## Layout
- `cpp/core/`: the `aegis_core` static library (protocol, CRC, reassembler, Kalman filter);
  headers in `include/aegis/`.
- `cpp/proxy/`: `ew_chaos_proxy`, the link simulator. Everything but `main.cpp` is the
  `aegis_proxy` static library, so the tests can link it.
- `cpp/bindings/aegis_core.cpp`: the pybind11 module `aegislink._core`.
- `cpp/tests/`: C++ unit tests, one executable per module, with a tiny runner (`test_runner.hpp`).
- `python/aegislink/`: the Python package. `common/config.py` holds the frame rate, image size
  and default ports; `common/messages.py` the message bodies; `common/net.py` the link receiver;
  `common/proxy_control.py` talks to the proxy's control port; `common/geo.py` converts local
  metres to latitude/longitude; `tools/loopback_check.py` sends messages over loopback UDP and
  checks that they arrive intact; `tools/link_meter.py` measures a proxy link; `uav/` is the UAV
  side (synthetic world, camera, flight, detector, tracker, encoder, scheduler, streamer);
  `gcs/` the ground station (receiver, poses, ghost tracks, compositor, dashboard, EW deck);
  `tools/run_demo.py` starts the proxy, the ground station and the UAV together;
  `tools/run_scenario.py` runs a scripted timeline from `scenarios/`, and `tools/evaluate.py`
  scores the station's tracks against the ground truth.
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

## Kalman filter (`cpp/core/include/aegis/kalman.hpp`)
A linear Kalman filter for one target on flat ground, in metres, measured by its position only.

- Two motion models: constant velocity, state `x, y, vx, vy`, driven by white-noise acceleration;
  and constant acceleration, state `x, y, vx, vy, ax, ay`, driven by white-noise jerk. Both are
  linear, so the plain filter is exact (no EKF needed). The axes are independent: per axis, `F`
  moves each derivative into the lower ones (`1, dt, dt^2/2`), and over `dt` the noise of the
  highest derivative adds `Q = q [dt^3/3 dt^2/2; dt^2/2 dt]` (constant velocity) or the 3 x 3
  version from `dt^5/20` to `dt` (constant acceleration).
- `update` takes a position with its 2 x 2 covariance, uses the Joseph form
  `P = (I - KH) P (I - KH)^T + K R K^T` (it stays symmetric and positive definite despite
  rounding) and returns the normalised innovation squared. With an innovation covariance that is
  not positive definite it changes nothing and returns -1.
- `predicted(dt)` returns a moved copy, so the ground station can draw any moment ahead without
  touching the filter. A negative or non-finite `dt`, or a negative `q`, throws
  `std::invalid_argument`.

## Python module `aegislink._core`
Mirrors the C++ API: `crc32`, `MsgType` (IntEnum), `ParseError` (Enum), `Header`,
`encode_packet`, `parse_packet` (returns a `Packet` with its own copy of the payload),
`Packetizer`, `Reassembler` (`push`, `expire`, `pending`, `stats`), `MotionModel` (Enum),
`KalmanFilter` (`init`, `predict`, `update`, `predicted`, `copy`; `state`, `covariance` and
`position_covariance` as lists), and the constants
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
`python -m aegislink.uav.main [--seconds 60] [--seed 1] [--no-preview] [--log PATH] [--send]`

- `UavSim.step()` renders the current state, detects, tracks (with the camera motion since the
  last frame), then advances the world and the flight by 1/30 s. A seed fixes the whole run.
- The preview (OpenCV window, paced to real time) draws the confirmed tracks: green when matched
  this frame, amber while coasting, with ids and a line of telemetry. `q` or Esc quits.
- The ground-truth log (default `logs/ground_truth_<date>_<time>.jsonl`) holds a header line
  (seed, fps, map size, origin, followed vehicle, camera) and then one line per frame: the UAV pose
  with latitude/longitude, pitch, roll and speed, every vehicle near the view with its
  position, heading, speed, `visible`, `in_frame` and pixel box, and `traffic`: the position of
  every vehicle on the map by id, so a vehicle can still be scored after it leaves the view. All
  of one line is from the same instant. About 3.9 MB per minute.
- `TrackScore` compares tracks with the truth over a run: a vehicle fully in the image and at
  least 90 % visible counts, and is covered when a track matched this frame overlaps it (IoU >=
  0.3). A new id for a vehicle covered within the last 30 frames is an id switch; after a longer
  gap (hidden or out of the image longer than a track lives) it is a reacquisition. Seeds 1-5,
  60 s each: covered 94.7-95.0 % of 2740-5913 vehicle frames (the misses are the detector's 5 %),
  0 id switches, 20 reacquisitions, 0 false track frames.
- With `--send` the loop runs in real time and hands every frame to the `Streamer` (below); the
  header timestamp is the capture time from the monotonic clock.

## Streams and ports (`common/config.py`)
Three proxy links, numbered in this order on the proxy command line:

| Link | UAV sends to | GCS receives on | Carries |
|---|---|---|---|
| 1 | 5600 | 5601 | foveated stream: telemetry, tracks, ROI patches, keyframes |
| 2 | 5602 | 5603 | baseline: every frame as a JPEG |
| 3 | 5604 (from the GCS) | 5605 (UAV) | receiver reports for link 1 |

`--direct` on both nodes skips the proxy (the UAV sends to 5601/5603, the GCS reports to 5605).

## Message bodies (`common/messages.py`)
Fixed little-endian structs after the wire header; image messages end with the encoded image.
Every decoder checks the length first and raises `ValueError`.

| Type | Body | Bytes |
|---|---|---|
| TELEMETRY | frame u32, lat i32, lon i32 (1e-7 degree), alt, speed, heading, pitch, roll (f32) | 32 |
| TRACKS | frame u32, count u8, then per track: id u16, x1 y1 x2 y2 i16 (pixels), flags u8 (1 = matched this frame) | 5 + 11 per track |
| ROI_PATCH | frame u32, alt f32, heading f32, track id u16, crop box 4 x i16, WebP | 20 + image |
| BG_KEYFRAME | frame u32, alt f32, heading f32, WebP | 12 + image |
| FULL_FRAME | frame u32, JPEG | 4 + image |
| RECEIVER_REPORT | GCS time u64 (us), highest seq u32, packets u32, bytes u64, min delay i64 (us) | 32 |

- Image messages carry the camera's altitude and heading but not its position: the GCS takes
  the position from the telemetry of the same frame.
- Report counters are cumulative since the start, so a lost report loses only its delay sample.
  The delay is the smallest arrival time minus header timestamp since the previous report, or
  `NO_DELAY` (-2^63) if there was no sample; only its changes matter, so the two clocks need not
  agree. It is clamped to the i64 range (the timestamp is any u64).

## Encoding (`uav/encoder.py`)
- Keyframe: the frame in gray, resized to 480 x 270 (`INTER_AREA`), WebP quality 25.
- Patch: the track box rounded, padded by 8 px and clipped to the image, in colour, WebP quality
  70; a box entirely outside the image gives no patch.
- Baseline: the whole 1280 x 720 frame as JPEG quality 50.
- `decode_image` returns `None` for bytes that do not decode. Given the expected size, it first
  reads the size from the WebP or JPEG header (`image_size`) and refuses any other format or
  size: a few hundred bytes can claim a 16000 x 16000 image, which would take seconds and
  gigabytes to decode.

## Foveated stream (`uav/scheduler.py`, `uav/streamer.py`)
`Scheduler.tick` runs once per frame with the current budget and spends it in priority order:

1. **Tokens**: the budget refills a token bucket for the time since the last frame. The bucket
   holds at most 0.1 s of budget plus one image datagram (or the telemetry and tracks, if
   larger), so a quiet spell does not end in a burst. A datagram is sent only when the tokens
   cover it.
2. **Telemetry and tracks** of this frame, both or neither, so the GCS always has both of a
   frame. If they do not fit they are skipped (the next frame's are newer), and so is every
   image in that frame, so that images never take the tokens they are waiting for.
3. **One image message at a time**, chosen by how overdue it is: `age / interval`, where a patch
   of a track matched this frame has an interval of 0.1 s (10 Hz) and the keyframe an interval of
   1 s, stretched so that keyframes use at most 15 % of the budget. The most overdue image is
   encoded and its datagrams go out as the tokens allow, over several frames if needed; nothing
   else is started until it is done.
4. **Datagram size**: an image is cut into datagrams of at most 30 ms of budget (at least 200
   bytes of payload, at most 1200). At 50 kbps a full 1230-byte datagram would hold the link for
   about 200 ms and delay the next telemetry behind it. When the budget drops while an image is
   being sent, its remaining larger datagrams leave once the tokens cover one datagram of the
   new size (the bucket then runs a short debt).

The scheduler numbers datagrams itself when they leave (`seq`, `msg_id`), so a skipped message
leaves no gap that the GCS would count as a loss. It sets `limited` whenever the budget held
something back.

`AimdBudget` (start 64 kbps, between 8 kbps and 2 Mbps) updates the budget on every receiver
report:

- **Congestion** is a filling queue (reports with packets but no delay sample are skipped): the
  report's minimum delay minus the lowest minimum of the
  last 10 s is above 40 ms. Random loss from jamming does not raise the delay, so it never cuts
  the budget. On congestion the budget becomes at most 0.85 x the rate the GCS received over the
  report interval (while the link is congested, that is its capacity), at most once per 0.5 s
  while the queue drains.
- **Growth**, only when `limited` was set: x 1.25 per report until the first congestion (TCP's
  slow start), then + 2 kbps per report.
- **No reports** (blackout): the budget is held.
- A report older than the last one is ignored. A fixed delay that grows (a slower route) looks
  like congestion until it has been the lowest for 10 s.

`Streamer` builds each frame's bodies (`frame_data`), sends the scheduler's datagrams to link 1
and the baseline (`BaselineSender`: one JPEG per frame, all fragments at once) to link 2, and
reads receiver reports from its own socket. Sending and receiving use separate sockets, because
on Windows a send to a closed port makes the next receive on the same socket fail.

## Ground station (`gcs/`)
`python -m aegislink.gcs.main [--no-view] [--seconds S] [--warmup S] [--summary] [--snapshot PNG] [--direct] [--proxy-control PORT] [--motion-model cv|ca] [--track-log PATH]`

- `GroundStation` (`gcs/receiver.py`) reads both links with `select()` and one `LinkReceiver`
  each. The foveated link's reassembler waits 4 s (a paced keyframe takes about 3 s at 50
  kbps), the baseline's 200 ms. Only telemetry and tracks count for the delay: they are sent the
  moment they are captured, while a paced image keeps its capture time.
- It keeps the newest of everything by frame number (older arrivals are ignored and not
  counted): telemetry, the track list, the keyframe and one patch per track of the newest list.
  A frame number more than 10 s (300 frames) below the held one is taken as a new run of the UAV
  (or the held one as forged): telemetry or tracks from it reset the foveated state.
- Images must have their expected size in their header before they are decoded: a patch the
  size of its box (inside the image), a keyframe 480 x 270, a baseline frame 1280 x 720 (checked
  on arrival, decoded once when first shown). Anything else, and any body that does not decode,
  is counted in `bad_messages` and dropped. So are telemetry and images with an altitude
  outside 1-10,000 m or a heading that is not finite (the camera model divides by the altitude),
  and a patch more than twice the size of its track's box plus the pad (each patch is warped
  on every drawn frame).
- Every 250 ms it sends a receiver report for link 1 to the UAV.
- `PoseHistory` (`gcs/poses.py`) keeps the UAV's position for the last 300 telemetry messages
  (10 s at 30 Hz) by frame number, in metres east/north of the first telemetry received, interpolated between
  frames whose telemetry was lost. Images carry altitude and heading but no position, so the
  compositor looks the position up here. Track lists carry neither, so their whole pose
  (`telemetry_pose`: altitude too, and the heading interpolated the short way round) comes from
  here as well.
- Every track list is handed to the ghost tracks (below) with the pose of its frame. A list that
  arrives before the telemetry of its frame waits for it (only the newest one waits).
- The station keeps a UAV clock: the newest telemetry's frame / 30 plus the time since it
  arrived. It keeps running through a blackout, and the tracks are predicted to it. With
  `--track-log` (in both modes) the predictions are written 30 times a second, one JSON line
  each: the UAV time, how long the foveated link has been silent, the origin of the ground frame
  (latitude, longitude of the first telemetry), and per track its state, time since the last fix,
  mean, covariance (xx, xy, yy) and the filtered position at the last fix. Every line is flushed,
  so a stopped process leaves only whole lines.
- With `--no-view` the station runs in one loop and `--summary` prints one JSON line about the
  `--seconds` after `--warmup` seconds from the first foveated datagram: kbps per link and their
  ratio, telemetry and track messages per second and the share of frames that arrived, patches
  per second, keyframes, baseline frames per second, bad packets and messages, expired partial
  messages.

### Ghost tracks (`gcs/ghost_tracks.py`)
One Kalman filter (constant velocity by default, `--motion-model ca` for constant acceleration)
per track id, in metres east/north of the first telemetry. Pixel motion mixes the vehicle's motion
with the camera's, ground metres do not.

- **Measurements**: only boxes that matched a detection in their frame (a coasting box is the UAV
  tracker's own guess). The box centre goes to the ground through the camera pose of its frame
  (`NadirCamera.pixel_to_ground_matrix`). Its covariance starts in pixels, from the detector's
  edge noise (sigma 1 px + 4 % of the box size per edge, plus rounding; the centre averages two
  edges), and is turned and scaled into metres with the same matrix. A box within two sigmas of
  the image edge may have been cut there, which moves its centre inwards by up to half its size:
  that axis gets `(size / 2)^2` more variance. A measurement not newer than the track's last one
  is ignored.
- **Time** is UAV time, frame / 30. A new track starts at its first position, still, with a
  speed standard deviation of 10 m/s (and 3 m/s^2 for the acceleration).
- **States**: LIVE while fixes arrive; GHOST when the last fix is more than 150 ms old (the
  filter only predicts and its ellipse grows); LOST, and forgotten, when the last fix is 10 s old
  or the 95 % ellipse's longer semi-axis exceeds 100 m. A track the UAV stopped listing stays
  until then. At most 64 tracks are kept: when full, a new id replaces the track with the oldest
  fix, or is refused if every track was fixed at that same moment (a forged track list of 255
  new ids cannot fill the station with filters that every drawn frame works through).
- **Estimates** for a moment `t`: state, time since the last fix (TSLF), mean, 2 x 2 position
  covariance, velocity, and the path predicted 0.1 s apart over the next 2 s.
- **The 95 % ellipse**: a position lies inside when its squared Mahalanobis distance from the
  mean is at most 5.991 (the 95 % point of the chi-square distribution with 2 degrees of
  freedom); the semi-axes are `sqrt(5.991 x eigenvalue)` of the covariance.
- **Process noise**, tuned on the simulated traffic: `q` = 3 m^2/s^3 (constant velocity) and 2
  m^2/s^5 (constant acceleration). With every vehicle of a map measured at 30 Hz (0.25 m noise,
  5 % misses) and blacked out for 5 s every 10 s, the true position lay inside the ellipse in
  95.7 % and 96.4 % of the blackout frames. What it cannot foresee is a full stop: a car braking
  from 12 m/s to 0 during a blackout leaves the ellipse after 2-3 s, so over a run with only a few
  tracks the share depends on how many of them stopped.

### Compositor (`gcs/compositor.py`)
Every piece arrives in the pixels of the frame it was captured in, and the view is drawn from
the camera pose of the newest telemetry. Each piece is mapped into it with
`NadirCamera.motion_matrix` between the two poses (a nadir camera over flat ground: one affine
map). Without a known position for either frame, a piece is not moved; a piece that the motion
would scale by more than 4 (or less than 1/4) is not drawn.

- **Background**: the keyframe (480 x 270 gray) is scaled to the camera's size, moved by the
  camera's motion since it was taken and scaled to the view, in one `warpAffine`. Where the
  keyframe does not reach, the view is black.
- **Track boxes** are moved from their frame to the view's (usually the same frame; the track
  list carries no heading, so only the position change is used).
- **Patches**: each track's newest patch is turned and scaled by the camera's motion since it was
  taken and placed so that its track's box lands on the newest box: the target moves at the track
  rate (30 Hz) even when its patch is older. A crop cut at the image edge is placed by its uncut
  side, which lies 8 px (the pad) from the box. The patch's border fades into the background over
  6 px, inside the pad, so the vehicle stays opaque.
- Boxes are drawn green when matched in their frame, amber while coasting, with their ids.
- **Ghost tracks** are drawn into the view through the view's camera pose (not drawn without
  one). A GHOST track's box and patch move, at their size, to where its filter predicts the
  vehicle now, so during a blackout the vehicles keep driving over the frozen background. It is
  drawn in magenta with its 95 % ellipse and a label with its id and TSLF (above the ellipse when
  the track has no box). Every track's predicted path for the next 2 s is drawn from 4.5 m out
  (half a truck plus a little), so it never covers the vehicle.

### Dashboard (`gcs/dashboard.py`, `gcs/ew_deck.py`)
- PyQt6 with a dark Fusion palette and pyqtgraph plots. Top: the baseline and the foveated view
  side by side, each with its numbers below (kbps and frames per second; kbps, share of the
  baseline, telemetry, track and patch rates, keyframe age, the filtered and ghost tracks, the
  longest TSLF and the largest ghost ellipse in m^2). Bottom: the EW deck, received kbps
  (log scale) and messages per second over the last minute, and a status line.
- `ReceiverThread` runs the station in its own thread: it waits for datagrams without the lock,
  then handles them and sends reports under it. The window reads the station only through
  `snapshot()` (a copy of the view's inputs, the newest baseline message and the numbers), so
  drawing never holds up receiving.
- A 16 ms precise timer draws a frame: snapshot, compose the foveated view at the widget's size
  (at most 1280 x 720), decode the baseline JPEG once per frame and scale it, and paint both
  views at once (`repaint`). `FrameClock` records each frame's work time and the interval
  between frame starts; the status line shows their 95th percentiles, and when the window closes
  they are printed as one JSON line with the last second's numbers.
- Every 125 ms half of the numbers and one of the two plots are refreshed, in turn: a plot takes
  3-5 ms to paint, and both in one frame made the next frame late.
- No foveated datagram for 0.5 s: the view shows LINK LOST and how long.
- In PyQt6 an exception that leaves a Qt slot ends the process, so the two timer slots catch
  everything and show it in the status line. Ctrl+C quits the event loop (a SIGINT handler), so
  the receiver stops and the frame times are still printed. The frame times of the last 10
  minutes are kept for that summary.
- **EW deck**: sliders for packet loss (0-60 %), mean burst length (1-20), rate limit (off, 15 to
  20,000 kbps in steps), delay (0-300 ms) and jitter (0-200 ms), a timed blackout and Clear.
  Every change goes to all three links: the jammer hits the radio channel both ways, so a
  blackout also silences the receiver reports and the UAV holds its budget.
- `ProxyLink` never waits for the proxy: commands go out on a non-blocking socket, and replies
  and stats are read whenever they have arrived. Datagrams from any other address, and replies
  or stats that do not parse or hold a value that is not finite, are ignored; synced slider
  values are clamped to the slider's range. On the first reply (and the first after a
  silence) the sliders move to the proxy's settings. Without a word for 2 s it asks again, which also takes the stats back from a
  script that sent a command in between. The deck shows each link's output rate and drops by
  cause from the proxy's stats.

## Demo (`tools/run_demo.py`)
`python -m aegislink.tools.run_demo [--seed N] [--seconds S] [--snapshot PNG] [--log PATH] [--base-port P]`

Starts `ew_chaos_proxy` with the three links, waits until it answers, then the ground station
window and the UAV (`--send`, real time). When the window closes it stops the UAV and the proxy.
The ports are the defaults (5600-5605, control 5700) or the same layout from `--base-port`.

## Scenarios (`tools/run_scenario.py`, `tools/evaluate.py`, `scenarios/`)
`python -m aegislink.tools.run_scenario SCENARIO.json [--seeds N ...] [--out DIR] [--base-port P]`

- A scenario is a JSON file: `name`, `seconds` (how long the station measures), `warmup_s`
  (default 5), `seed` (default 1), optional `motion_model`, and `events`, each
  `{"at_s": T, "set": {...}}` with the proxy's control keys, applied at T seconds after the UAV
  started to all three links (like the EW deck) or to the ones in `"links"`.
- `run_scenario` starts the proxy, the ground station (headless, `--summary`, `--track-log`) and
  the UAV (`--send`, with its ground-truth log), plays the events through the control port, and
  waits for both nodes to end by themselves (the UAV flies 3 s longer than the station measures,
  so both logs are complete). The logs and `result.json` (scenario, the station's summary, the
  track score) go to `logs/scenarios/<name>_<date>_<time>/`. With `--seeds` it runs once per seed,
  each in `seed_<n>/`, and pools the blackout samples of all runs.
- `evaluate` matches each track to the true vehicle nearest (within 3 m) to its filtered
  position at its last fix, moving the station's ground frame into the map frame by the two
  origins, and checks every sample against that vehicle's true position at the same UAV time,
  interpolated between frames. Samples taken while the foveated link was silent for 150 ms or
  more and the track was a GHOST are the blackout samples; it reports their share inside the
  95 % ellipse, overall and per whole second of TSLF, the length of each silence, and the share
  for samples with the link up.
- `scenarios/`: `clean`, `loss30` (30 % loss in bursts of 4), `kbps50` (50 kbps on all links)
  and `blackout` (three 5 s blackouts, 15 s apart, in 60 s).
