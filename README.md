# AegisLink

A simulated UAV-to-ground video and telemetry link that keeps working under electronic warfare:
bursty packet loss, bandwidth down to tens of kbps, total blackouts and GPS spoofing.

Work in progress. So far it has:

- **The wire protocol** that all parts of the link use: a C++20 library with Python bindings for
  framing, CRC-checked parsing, fragmentation, and a reassembler that copes with lost, duplicated
  and reordered fragments.
- **`ew_chaos_proxy`**, a C++20 UDP proxy that plays the radio link. Each link it forwards has
  bursty packet loss (a Gilbert-Elliott model set by loss rate and mean burst length), a rate
  limit with a bounded queue, delay and jitter, and timed blackouts. All of it can be changed
  while traffic flows, through plain-text commands on a control port, and the proxy reports
  per-link counters 4 times per second.
- **`link_meter`**, a tool that sends a numbered stream through a proxy link and measures the
  loss, burst length, throughput and delay that come out.
- **A synthetic UAV world** with exact ground truth: a procedural 1 km x 1 km map with roads,
  trees and a bridge, traffic that follows, stops and disappears under trees, and a multirotor
  that follows one vehicle with a downward-looking camera. A simulated detector (misses, box
  noise, false alarms) feeds an IoU tracker that compensates for the camera's own motion. Over
  five 60 s runs the tracks covered 94.7-95.0 % of the vehicles in clear view with no id
  switches.
- **Foveated streaming** from the UAV to a ground station, next to a plain full-frame JPEG
  stream for comparison. The UAV sends telemetry and track boxes every frame, small colour
  patches of the tracked vehicles, and now and then a small gray background, in that order of
  priority, within a byte budget that follows the ground station's receiver reports (grown
  while the link keeps up, cut when its queue starts to fill). Measured through the proxy in
  scenario runs: over 10 minutes of a clean link the foveated stream used 208 kbps against
  14,642 kbps for the baseline (1.4 %); over 30 s with all links limited to 50 kbps, the
  telemetry and boxes of every frame arrived (30 per second) with about 4.6 patches per second,
  while not one baseline frame got through.
- **A ground station dashboard** (PyQt6 and pyqtgraph) with the baseline and the foveated stream
  side by side. The foveated view is rebuilt from what arrived: the last gray background, moved
  and turned with the camera since it was taken, with each vehicle's newest colour patch blended
  onto its newest track box. Below them: bitrates, message rates and the background's age,
  plots of the last minute, and an EW deck whose sliders set loss, burst length, rate limit,
  delay, jitter, timed blackouts and GPS spoofing on the proxy while the link runs. In a 30 s demo run on the
  development PC it drew 62 frames per second with a frame time of 4.5 ms at the 95th
  percentile; after the links were limited to 50 kbps, the boxes kept arriving 30 times a
  second while the baseline froze.
- **Ghost tracks**: the ground station follows every track with a Kalman filter (C++, constant
  velocity or constant acceleration) in ground metres, so camera motion does not leak into the
  vehicles' motion. When boxes stop arriving, in a blackout or under a tree, a track turns into a
  ghost: its patch keeps moving along the prediction, drawn with its 95 % uncertainty ellipse,
  its predicted path and the time since its last fix. The ellipse is only useful if it can be
  trusted, so it is measured: in three 60 s runs (seeds 1-3) with three 5 s blackouts each, the
  true vehicle position lay inside the drawn ellipse in 94.2 % of 6,153 blackout samples (90.9,
  100 and 94.4 % per run; the misses are cars that braked to a stop during a blackout).
- **GPS spoofing and its detection**: the proxy can act as a man-in-the-middle on the
  telemetry, moving the reported position by a drift or a jump. It recomputes the CRC, so the
  packets are accepted: a CRC catches noise, not an attacker (a real link would add a message
  authentication code, which is left out here on purpose). The ground station checks the GPS
  against what the camera saw: optical flow between consecutive background keyframes, with
  their altitude and heading, gives the distance flown without the GPS. Three keyframes in a row
  that disagree (chi-square test at 99 %) raise an alarm, and until five agree again the station
  navigates by the flow alone, so the background, the boxes and the ghost tracks stay in place.
  In scenario runs (seeds 1-3, two attacks each): a 3 m/s drift was detected after 1.3-1.7 s and
  a 50 m jump after 1.4-1.8 s, with the position used during the alarm at most 1.2 m from the
  truth; 10 minutes of clean flight raised no false alarm.
- **Scripted scenarios**: JSON timelines of link settings (`scenarios/`) that run the proxy,
  the ground station and the UAV together and score what arrived against the ground truth.

## Build and test

Needs Python 3.12+, CMake 3.25+ (for the presets) and a C++20 compiler (tested with MSVC 14.44
and GCC 16).

```
python -m pip install -e ".[test]"
python -m pytest
python -m aegislink.tools.loopback_check --size 50000
```

The install also builds `ew_chaos_proxy` and puts it next to the Python scripts. To try it, start
it in one terminal and measure a link with 30 % loss in bursts of 4 packets from another:

```
ew_chaos_proxy --link 5600:5601 --control 5700
python -m aegislink.tools.link_meter --loss-pct 30 --burst 4
```

To watch the UAV simulation with its tracked boxes (it also writes the ground truth to `logs/`):

```
python -m aegislink.uav.main
```

To run the whole demo, the proxy, the ground station window and the UAV, in one command (closing
the window stops all three):

```
python -m aegislink.tools.run_demo
```

Or each in its own terminal:

```
ew_chaos_proxy --link 5600:5601 --link 5602:5603 --link 5604:5605 --control 5700
python -m aegislink.gcs.main
python -m aegislink.uav.main --no-preview --send
```

To run a scenario (here three 5 s blackouts in 60 s) and score the tracks; the logs and a
`result.json` go to `logs/scenarios/`:

```
python -m aegislink.tools.run_scenario scenarios/blackout.json
```

The spoofing scenarios score the GPS check the same way (detection time, false alarms):

```
python -m aegislink.tools.run_scenario scenarios/spoof_jump.json --seeds 1 2 3
```

C++ unit tests: `cmake --preset msvc` (Windows) or `cmake --preset gcc-asan` (Linux), then
`cmake --build --preset <name>` and `ctest --preset <name>`. More in `docs/TESTING.md`; how the
code works is in `docs/ARCHITECTURE.md`.

## License

AGPL-3.0 (see `LICENSE`).
