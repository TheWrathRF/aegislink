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

C++ unit tests: `cmake --preset msvc` (Windows) or `cmake --preset gcc-asan` (Linux), then
`cmake --build --preset <name>` and `ctest --preset <name>`. More in `docs/TESTING.md`; how the
code works is in `docs/ARCHITECTURE.md`.

## License

AGPL-3.0 (see `LICENSE`).
