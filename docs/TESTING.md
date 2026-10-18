# Testing and build

## Requirements
- Python 3.12+ (developed on 3.14), ideally in a virtual environment.
- CMake 3.25+ and a C++20 compiler: MSVC 2022 on Windows (found by the Visual Studio generator,
  no developer prompt needed), GCC on Linux. `pip install cmake ninja` provides CMake if needed.
- All build output goes to `build/` (git-ignored).

## Commands (from the repository root)
C++ unit tests (CTest; presets in `CMakePresets.json`):
```
cmake --preset msvc
cmake --build --preset msvc
ctest --preset msvc
```
On Linux the `gcc-asan` preset builds the same tests with AddressSanitizer and UBSan.

Python module, proxy and tests (builds `aegislink._core` and `ew_chaos_proxy` into
`build/<wheel tag>` and installs the proxy next to the Python scripts; rerun the install after any
C++ change, since the proxy tests run the installed binary):
```
python -m pip install -e ".[test]"
python -m pytest
```

Loopback check (a 50 KB message over loopback UDP in 42 fragments):
```
python -m aegislink.tools.loopback_check --size 50000
```

Chaos proxy by hand: start it in one terminal, measure a link from another (the meter sets the
link through the control port first):
```
ew_chaos_proxy --link 5600:5601 --control 5700
python -m aegislink.tools.link_meter --loss-pct 30 --burst 4
python -m aegislink.tools.link_meter --kbps 50 --size 1230 --rate 10 --packets 60
python -m aegislink.tools.link_meter --delay-ms 30 --jitter-ms 20 --packets 2000 --rate 500
```

UAV simulation with an OpenCV preview (q or Esc quits), or headless; both write the ground
truth to `logs/` and print how well the tracks matched the true vehicles:
```
python -m aegislink.uav.main
python -m aegislink.uav.main --no-preview --seconds 60 --seed 2
```

Streaming through the proxy: three terminals. The GCS window shows the baseline and the
foveated stream side by side (q or Esc quits).
```
ew_chaos_proxy --link 5600:5601 --link 5602:5603 --link 5604:5605 --control 5700
python -m aegislink.gcs.main
python -m aegislink.uav.main --no-preview --send
```
The links can be changed while it runs, for example to 50 kbps on both video links:
```
python -c "from aegislink.common.proxy_control import ProxyControl as P; c = P(5700); c.configure(1, kbps=50); c.configure(2, kbps=50)"
```
Headless, the GCS measures what arrives (here 10 s after a 5 s warm-up) and prints it as JSON:
```
python -m aegislink.gcs.main --no-view --seconds 10 --warmup 5 --summary
```
Without the proxy: `--direct` on both nodes.

## What the tests cover
- `cpp/tests/test_crc32`: catalogued check values, chaining.
- `cpp/tests/test_protocol`: byte layout, round trip, every rejection path, a flipped bit
  anywhere, packetizing (50,000 B -> 42 fragments).
- `cpp/tests/test_reassembler`: in order, reversed, shuffled, duplicates (also after completion),
  timeout at exactly 200 ms, interleaved messages, inconsistent fragments, eviction.
- `cpp/tests/test_fuzz`: 200,000 random datagrams (none accepted); 3000 messages with random bit
  flips, truncation and drops (no corrupt message is ever delivered). About 4 s in a Debug build.
- `cpp/tests/test_udp_socket`: send and receive on loopback, select timeout, a send to a closed
  port does not break the next read.
- `cpp/tests/test_gilbert_elliott`: loss rate and mean burst length within 2 % over 1,000,000
  draws for five settings, the L = 1 limit, rejected settings, reset to the good state.
- `cpp/tests/test_token_bucket`: exact release times, a backlog never above depth + rate x T,
  refill capped at the depth.
- `cpp/tests/test_link`: delay and jitter bounds, loss counting, queueing and drop-tail, blackout
  timing and its precedence, rejected settings, a settings change keeps the rate backlog, seeds.
- `cpp/tests/test_control`: replies, every key, errors leave the link unchanged, the stats text.
- `tests/test_bindings.py`: the module against a `struct` + `zlib` reference of the format.
- `tests/test_loopback.py`: real UDP sockets on 127.0.0.1.
- `tests/test_proxy.py`: starts the installed `ew_chaos_proxy` and measures it with `link_meter`:
  datagrams pass unchanged and in order; control replies, errors and stats (4 per second);
  measured loss and mean burst length within 10 % of the settings (20,000 packets); throughput
  never above the limit at 50 and 500 kbps (and above 90 % of it); delay and jitter; blackout.
  About 25 s.
- `tests/test_camera.py`: latitude/longitude round trip, focal length and footprint, pixel ->
  ground -> pixel at five headings, image up = heading, the affine matrices against the point
  functions, the camera-motion matrix.
- `tests/test_sim_world.py`: a seed gives the same world and motion; roads inside the map;
  100 s of traffic stays in lane (0.3 m) with bumper gaps >= 1 m, and vehicles stop and go; a
  rendered vehicle sits at the pixel the camera predicts, inside its truth box; trees hide a
  vehicle and lower `visible`; boxes are clipped at the image edge; the flight keeps the followed
  vehicle in view over 95 % of 60 s, rolls under 30 degrees, and its telemetry matches its pose.
- `tests/test_tracker.py`: IoU; detection rate, edge noise (mean and sigma within 5 %) and false
  boxes over 10,000-20,000 frames; hidden and cut-off vehicles never detected; ids kept through
  gaps of 10-12 frames, through a crossing, and while the camera stops turning during a 12-frame
  gap (the last fails without camera-motion compensation); one-frame false detections never
  become tracks; a track ends after 30 missed frames.
- `tests/test_uav_run.py`: a 40 s run writes a log with the expected header and fields, whose
  pose, positions and boxes agree within 1 px; tracks cover over 90 % of the vehicles in clear
  view with at most 1 id switch and under 1 % false track frames; the score counts switches,
  reacquisitions and false tracks; a seed repeats a run exactly. About 10 s.
- `tests/test_messages.py`: body sizes, round trips (centimetre latitude/longitude, wrapped ids,
  boxes outside the image), every decoder rejects a wrong length.
- `tests/test_encoder.py`: gray 480 x 270 keyframe, padded and clipped patch boxes, patch close
  to the original crop, JPEG baseline, garbage decodes to `None`; image sizes read from lossy
  and lossless WebP and JPEG headers, and a JPEG claiming 65000 x 65000 refused before decoding
  (OpenCV raises on it otherwise).
- `tests/test_net.py`: counters, highest seq, bad datagrams, minimum delay per report interval
  (also negative), loss as seq gaps, a 4 s reassembly timeout, delay only from chosen types and
  `NO_DELAY` without a sample, a u64 timestamp that still encodes, the rate meter (bounded
  without reads).
- `tests/test_scheduler.py`: never more than the budget over 0.1-2 s windows (and over 95 % of
  it used); telemetry and tracks every frame at 50 kbps; a keyframe paced and whole at 50 kbps;
  no seq gaps; whole frames skipped below the control rate; 10 Hz patches and 1 Hz keyframes at a
  high budget; a new track's patch first; keyframes within their budget share; no image in a
  frame whose telemetry and tracks were skipped (16 tracks at 30 kbps); telemetry and tracks
  larger than 0.1 s of budget still sent; an image cut at 2 Mbps still leaves after a cut to 50
  kbps. AIMD: slow start
  then fixed steps, growth only when limited, cut to 0.85 x the received rate on queueing delay
  with a hold-off, no cut for 30 % random loss, held without reports, old low delays forgotten,
  any clock offset, limits, a report without a delay sample is not a base delay. A closed loop with a simulated 50 kbps drop-tail link: no drops
  after 10 s, 99 % of frames' tracks delivered, budget within 60-130 % of the link (this test
  failed with full-size image datagrams).
- `tests/test_streamer.py`: a simulated frame's telemetry, tracks, patches and keyframe decode
  to what the frame holds; the baseline is one JPEG per frame.
- `tests/test_gcs.py`: newest telemetry and tracks kept, patches dropped with their track, bad
  patch boxes and sizes and undecodable bodies counted, reports 4 times per second, the viewer
  with no data and with a patch cut at the image edge; a u64 timestamp does not stop the
  reports; oversize or wrong-size patches, keyframes and baseline frames dropped before decoding;
  a restarted UAV (frame 500, then 0) and a forged frame 0xFFFFFFFF are followed; older messages
  not counted; patches only for listed tracks.
- `tests/test_stream.py`: the UAV, the proxy and the GCS as three processes, measured by the
  GCS over 8 s after a 5 s warm-up. Clean link: the foveated stream is at most 10 % of the
  baseline and 99 % of frames' telemetry and tracks arrive. Both links at 50 kbps: the foveated
  stream stays under the limit, 99 % of frames' tracks arrive (at least 29.5 per second), and no
  baseline frame gets through. About 30 s.

## CI
`.github/workflows/ci.yml` runs on every push: Windows (`msvc` preset) and Ubuntu (`gcc-asan`
preset), CTest, then `pip install ".[test]"` and pytest.
