# Testing and build

## Requirements
- Python 3.12+ (developed on 3.14), ideally in a virtual environment.
- CMake 3.25+ and a C++20 compiler: MSVC 2022 on Windows (found by the Visual Studio generator,
  no developer prompt needed), GCC on Linux. `pip install cmake ninja` provides CMake if needed.
- All build output goes to `build/` (git-ignored).
- For real video only: an NVIDIA GPU, `python -m pip install -e ".[gpu]"` (onnxruntime-gpu with
  the CUDA 13 and cuDNN libraries as pip packages, about 1.6 GB), the YOLO model in `models/` and
  a clip filmed straight down in `data/` (both git-ignored; see below).

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

The whole demo in one command: the proxy, the ground station window and the UAV; closing the
window stops all three, and the window's frame times and what arrived over the whole run are
printed as JSON. `--seconds 30
--snapshot logs/demo.png` closes it after 30 s and saves a picture of it; `--record logs/rec`
saves the window 10 times a second as numbered JPEGs.
```
python -m aegislink.tools.run_demo
```

Real video with YOLO on the GPU. The model is exported once with Ultralytics, which is only
needed for that (a separate environment keeps PyTorch out of this one); the clip used here is
https://www.pexels.com/video/33269758/ (1920 x 1080, saved as `data/roundabout_tuzla.mp4`):
```
python -m pip install ultralytics onnx onnxslim
python -c "from ultralytics import YOLO; YOLO('yolo26n-obb.pt').export(format='onnx', imgsz=(384, 640), nms=True, agnostic_nms=True)"
```
Move `yolo26n-obb.onnx` to `models/`, then:
```
python -m aegislink.uav.main --video data/roundabout_tuzla.mp4
python -m aegislink.uav.main --video data/roundabout_tuzla.mp4 --no-preview --seconds 26
python -m aegislink.tools.run_demo --video data/roundabout_tuzla.mp4
```
`--alt0` sets the altitude at the clip's first frame (default 55 m, from the size of its cars).
The same by hand, in three terminals:
```
ew_chaos_proxy --link 5600:5601 --link 5602:5603 --link 5604:5605 --control 5700
python -m aegislink.gcs.main
python -m aegislink.uav.main --no-preview --send
```
The EW deck in the window changes all three links. A script can change them too, for example
to 50 kbps on both video links (the deck takes the proxy's stats back after 2 s):
```
python -c "from aegislink.common.proxy_control import ProxyControl as P; c = P(5700); c.configure(1, kbps=50); c.configure(2, kbps=50)"
```
or to move the telemetry position 50 m east (GPS spoofing; `spoof_offset_m=0` ends it):
```
python -c "from aegislink.common.proxy_control import ProxyControl as P; P(5700).configure(1, spoof_bearing_deg=90, spoof_offset_m=50)"
```
Headless, the GCS measures what arrives (here 10 s after a 5 s warm-up) and prints it as JSON:
```
python -m aegislink.gcs.main --no-view --seconds 10 --warmup 5 --summary
```
Without the proxy: `--direct` on both nodes.

Scripted scenarios (`scenarios/*.json`): the proxy, a headless ground station and the UAV run
the timeline, and the station's tracks are scored against the ground truth. The logs and
`result.json` go to `logs/scenarios/`; `--seeds` runs it once per seed and pools the blackout
samples. A 60 s scenario takes about 70 s.
```
python -m aegislink.tools.run_scenario scenarios/blackout.json
python -m aegislink.tools.run_scenario scenarios/blackout.json --seeds 1 2 3
python -m aegislink.tools.run_scenario scenarios/spoof_jump.json --seeds 1 2 3
python -m aegislink.tools.run_scenario scenarios/clean_10min.json
python -m aegislink.tools.run_scenario scenarios/video_clean.json
python -m aegislink.tools.run_scenario scenarios/video_kbps50.json
python -m aegislink.tools.evaluate --truth logs/scenarios/<run>/ground_truth.jsonl --tracks logs/scenarios/<run>/tracks.jsonl
```

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
  refill capped at the depth, a new rate keeps the debt as waiting time.
- `cpp/tests/test_link`: delay and jitter bounds, loss counting, queueing and drop-tail, blackout
  timing and its precedence, rejected settings, a settings change keeps the rate backlog, a rate
  change (also to unlimited) lets the queued packets leave first and keeps the new rate, seeds.
- `cpp/tests/test_spoofer`: an idle spoofer changes nothing; an offset moves the position along
  its bearing and the packet still parses (CRC), with every other byte unchanged; a drift grows
  with time; stopping it keeps the offset reached; other types, fragments, bad CRCs and short
  bodies are left alone; bad settings change nothing; a huge offset stays within +-90 degrees.
- `cpp/tests/test_control`: replies, every key (the spoof keys too), a command without spoof
  keys keeps a drift going, errors leave the link unchanged, the stats text.
- `cpp/tests/test_kalman`: predict and update of both models against hand-computed values (also
  with correlated measurement noise), the acceleration moving into position, a constant velocity
  learned, symmetric and positive covariance over 20,000 steps, bad input; and with a target
  that really moves by the model, a mean innovation of 2 (1.9-2.1) and 94-96 % of positions
  inside the 95 % ellipse over 40,000 steps.
- `tests/test_bindings.py`: the module against a `struct` + `zlib` reference of the format.
- `tests/test_kalman.py`: the bound filter against a NumPy reference of the same equations over
  200 random steps (both models), `predicted` and `copy` leave it alone, `ValueError` on bad
  arguments.
- `tests/test_ghost_tracks.py`: a box centre lands on its ground point at four headings; the box
  noise turned and scaled into metres; more variance for a box at (or within two sigmas of) the
  image edge; LIVE, GHOST and LOST (by time, and by ellipse size for constant acceleration); old
  measurements and coasting boxes ignored; the path runs ahead along the velocity; ellipse
  points on the 95 % contour. Consistency: 90 s of a whole simulated map, every vehicle measured
  except during 5 s blackouts every 10 s; the truth must lie inside the ellipse in 90-98 % of the
  blackout frames for both models (it fails with a process noise of 0.5: 75 %).
- `tests/test_loopback.py`: real UDP sockets on 127.0.0.1.
- `tests/test_proxy.py`: starts the installed `ew_chaos_proxy` and measures it with `link_meter`:
  datagrams pass unchanged and in order; control replies, errors and stats (4 per second);
  measured loss and mean burst length within 10 % of the settings (20,000 packets); throughput
  never above the limit at 50 and 500 kbps (and above 90 % of it); delay and jitter; blackout;
  spoofing moves the telemetry position 50 m east (the packet still parses, other fields and
  other message types unchanged, counted in the stats), a drift grows and clearing it ends it.
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
  pose, positions and boxes agree within 1 px, and whose `traffic` holds every vehicle; tracks cover over 90 % of the vehicles in clear
  view with at most 1 id switch and under 1 % false track frames; the score counts switches,
  reacquisitions and false tracks; a seed repeats a run exactly; a paced 3 s run reports 3.0 s
  (within 0.2), without the time it took to build the world. About 15 s.
- `tests/test_video.py`: 4 s of the simulated flight (a turn and a climb), rendered and
  registered frame to frame: the measured path stays within 0.5 m, 0.5 m of altitude and 0.5
  degrees of the truth (a wrong sign in the heading or the wrong order of the chained transforms
  fails it); a blank frame is bridged with the motion before it; a clip written to disk is
  resized, played back and forth without a jump, starts at the origin heading north, and its
  telemetry speed is the true speed within 5 %; a missing file and a first altitude at or below
  0 m (or not finite) are errors.
- `tests/test_yolo_detector.py`: letterboxing (scale, padding, RGB order, grey 114); output rows
  turned into upright boxes in frame pixels (a quarter turn swaps width and height, 45 degrees
  gives the square around it), unsure rows, other classes and empty slots dropped, boxes clipped
  at the image edge. With onnxruntime's CUDA provider, the model and the clip present (never on
  CI, so skipped there): the model runs on the GPU, finds at least 15 car-sized boxes on the
  clip's first frame (19 here) and takes under 15 ms per frame (median).
- `tests/test_messages.py`: body sizes, round trips (centimetre latitude/longitude, wrapped ids,
  boxes outside the image), every decoder rejects a wrong length.
- `tests/test_encoder.py`: gray 480 x 270 keyframe, padded and clipped patch boxes, patch close
  to the original crop, JPEG baseline, garbage decodes to `None`; image sizes read from lossy
  and lossless WebP and JPEG headers, and a JPEG claiming 65000 x 65000 refused before decoding
  (OpenCV raises on it otherwise).
- `tests/test_net.py`: counters, highest seq, bad datagrams, minimum delay per report interval
  (also negative), loss as seq gaps, a 4 s reassembly timeout, delay only from chosen types and
  `NO_DELAY` without a sample, a u64 timestamp that still encodes, the rate meter (bounded
  without reads); packets claiming 65,535 fragments counted as bad and no slower than normal
  ones, a message of 512 fragments still delivered and one of 513 refused.
- `tests/test_scheduler.py`: never more than the budget over 0.1-2 s windows (and over 95 % of
  it used); telemetry and tracks every frame at 50 kbps; a keyframe paced and whole at 50 kbps;
  no seq gaps; whole frames skipped below the control rate; 10 Hz patches and 2 Hz keyframes at a
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
- `tests/test_gcs.py`: newest telemetry and tracks kept, positions of recent frames in the view
  (cleared by a restart, the view a copy), patches dropped with their track, bad patch boxes and
  sizes and undecodable bodies counted, reports 4 times per second; a u64 timestamp does not stop
  the reports; oversize or wrong-size patches, keyframes and baseline frames dropped before
  decoding; a restarted UAV (frame 500, then 0) is followed on the 30th telemetry message of its
  new run and not before; one forged frame jump (far below or far ahead) changes nothing, real
  telemetry in between cancels a pending restart, after a 20 s silence the stream goes on
  without a reset; a forged track frame 0xFFFFFFFF before any telemetry does not freeze; older
  messages not counted; patches only for listed tracks; the dashboard's numbers; the summary
  starts after the warm-up, counts frames by number and a baseline frame from before the
  measurement not at all; the receiver
  thread receives, reports and stops; impossible altitudes (0, negative, NaN, infinite, 1e30)
  and headings (NaN, infinite) in telemetry, patches and keyframes dropped; a patch much larger
  than its box dropped; `--snapshot` and `--record` refused without the window. Ghost tracks:
  matched boxes become ground positions (coasting ones are not measured), a track list before
  its telemetry is measured when the telemetry comes, the UAV clock runs on and tracks turn
  GHOST in a silence (and are counted in the numbers), a track missing from a new list ends while
  a listed coasting one stays (as a GHOST), a restart clears them, the view holds them predicted to
  now, the track log is written 30 times a second, the heading is interpolated the short way;
  telemetry repeating the held frame does not turn the UAV clock back.
- `tests/test_spoof_detector.py`: keyframes recorded from a 40 s simulated flight (2 Hz). The
  flow finds the true displacement between every pair (under 10 cm, mean under 3 cm) while the
  UAV turns and climbs, and nothing on an untextured image; no alarm on the clean flight; a
  3 m/s drift raises the alarm within 3 keyframes; a 50 m jump at the first suspect keyframe +
  1 s, cleared after 5 agreeing ones; during 35 s of alarm the flow path stays within 1 m of the
  true position while the GPS ends 100 m off; the flow path interpolates and carries on; pairs
  the flow cannot follow, or too far apart, decide nothing; a repeated keyframe changes nothing;
  a 50 m jump inside a 4 s gap between keyframes raises the alarm at the third keyframe after
  it (also at the station), a 100 m jump inside a 6.5 s gap (too far apart to compare) once the
  coasted chain sees it, and a 50 m jump while keyframes come 3-6 s apart (as after a rate limit
  is lifted) through the check of the pair alone, and one inside a pair 5.5 s apart (compared
  up to 6 s); a pair after one that held a 100 m jump, whose GPS-velocity guess is then far off,
  is still followed by retrying with the flow velocity;
  the same keyframe spacings and three gaps of 4-7 s on the clean flight raise none; after 20 s without keyframes, pairs 2 s apart are
  followed again (with the GPS velocity as the search guess) and raise no alarm.
  The station: with the GPS 50 m off, every position the view uses comes from the flow, the
  stats and the track log show the alarm, it uses the GPS again once it agrees, and one forged
  telemetry with an old frame number (a fake restart) does not end the alarm. Removing
  the outlier rule or tying the chain to a suspect keyframe makes these tests fail.
- `tests/test_compositor.py`: positions relative to the first telemetry, interpolated, 10 s
  kept; a dot on the background lands where the camera model puts its ground point after a
  move, a turn and a climb (within 1 px), and is only scaled without poses; a patch lands on its
  crop, opaque inside and feathered at its edge; it follows the newest box, turns with the camera,
  and a crop cut at the image edge keeps its uncut side on the box; boxes from an older frame move
  with the camera. These fail without the warp, the edge rule or the feathering. Telemetry
  repeating a frame keeps the history; extreme altitudes and position jumps neither raise nor
  take over 10 ms (after one normal render, which pays for OpenCV's set-up). A GHOST moves its patch to the predicted position (a LIVE one does not); the
  ghost ellipse has the filter's size, its label sits above it and the path starts 4.5 m out;
  no ghost is drawn without a view pose. Fade masks are shared for car-sized patches, and 20
  large patch sizes add nothing to that cache.
- `tests/test_dashboard.py` (Qt offscreen; skipped where Qt cannot load): frame-time statistics;
  the window draws both views (patch, background and baseline pixels where expected) in under
  16.7 ms at the 95th percentile; `record` saves 9-11 numbered pictures of the window in 1 s,
  the patch visible in them, and a folder that cannot be written stops the recording with a
  message instead of ending the window; a ghost drawn and counted while the link is silent; LINK
  LOST after a silence; GPS SPOOFED and the speeds while the alarm is on; the EW deck starts at
  the proxy's settings and sets loss, burst, rate, jitter, a blackout, a GPS drift, bearing and
  jump, and Clear on all three links of the real proxy; without a proxy it says so; bad replies (NaN, infinite, huge, missing stats keys)
  and datagrams from another address change nothing; an exception while drawing is shown, not
  fatal; the kept frame times are bounded; Ctrl+C closes the window cleanly (a subprocess that
  raises SIGINT exits 0 and prints its frame times). A live run (UAV process, proxy, this window, about
  20 s): on a clean link the tracks and baseline frames of at least 97 % of the UAV's frames
  arrive, then the deck's rate slider at 50 kbps stops the baseline while the tracks keep
  arriving just as completely. Frames are counted by number: under the load of the whole test
  run the UAV itself makes fewer than 30 frames a second. Neither the streams nor the draw loop
  may stall (15 track lists and 30 drawn frames per second or more: with the UAV and the proxy
  on the same machine the rates depend on the machine).
- `tests/test_demo.py`: `run_demo` for 8 s (window offscreen) exits cleanly, draws over 300
  frames, receives the tracks and baseline frames of at least 97 % of the UAV's frames (counted
  by number over the whole run, with 15 per second or more), reports no error, and saves its
  snapshot; with the UAV's report port taken, the UAV stops and `run_demo` exits with an error.
  A test that runs out of time kills the whole process tree, the proxy included. About 15 s.
- `tests/test_stream.py`: the UAV, the proxy and the GCS as three processes, measured by the
  GCS over 8 s after a 5 s warm-up. Clean link: the foveated stream is at most 10 % of the
  baseline and 99 % of frames' telemetry, tracks and baseline frames arrive. Both links at 50
  kbps: the foveated stream stays under the limit, 99 % of frames' tracks arrive, and no
  baseline frame gets through. The rates must only stay above 15 per second (the UAV's own
  frame rate depends on the machine). About 30 s.
- `tests/test_scenario.py`: the ground truth interpolated between frames and moved between
  origins; blackout and link-up samples scored apart, by TSLF, with unmatched tracks counted;
  scenario files checked and completed, the shipped ones load; pooled shares weighted by
  samples; with fake processes, a UAV at 27 frames per second still finishes the 10-minute
  scenario instead of being killed. The GPS check's score: detection timed from the moment the GPS went wrong, a false
  alarm, the clear time and the error of the position used; a spoof without an alarm is
  missed; pooled over runs. Two short scenarios end to end (UAV, proxy, headless station): a
  3 s blackout (the silence is found, the tracks are scored with loose bounds, `result.json` is
  written, no GPS alarm), and a 50 m GPS jump (alarm within 3 s, no false alarm, the position
  used within 2 m of the truth, cleared after the GPS is right again). About 35 s. A scenario
  file's `video` must be a path. Where the GPU, the model and the clip are present, a short run
  over the real clip (8 s, the links cut to 50 kbps halfway): tracks arrive, nothing is scored,
  the baseline averages under 25 frames per second, and the UAV's report says YOLO ran on CUDA
  under 15 ms per frame (about 20 s more).

## CI
`.github/workflows/ci.yml` runs on every push: Windows (`msvc` preset) and Ubuntu (`gcc-asan`
preset), CTest, then `pip install ".[test]"` and pytest. On Ubuntu it installs the system
libraries that Qt's offscreen platform needs for the dashboard tests. It installs neither the
`gpu` extra nor a model, so the YOLO test on the GPU is skipped there.
