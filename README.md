# AegisLink

A simulated UAV-to-ground video and telemetry link that keeps working under electronic warfare:
bursty packet loss, bandwidth down to tens of kbps, total blackouts and GPS spoofing.

Work in progress. So far it has the wire protocol that all parts of the link use: a C++20 library
with Python bindings for framing, CRC-checked parsing, fragmentation, and a reassembler that copes
with lost, duplicated and reordered fragments.

## Build and test

Needs Python 3.12+, CMake 3.25+ (for the presets) and a C++20 compiler (tested with MSVC 14.44
and GCC 16).

```
python -m pip install -e ".[test]"
python -m pytest
python -m aegislink.tools.loopback_check --size 50000
```

C++ unit tests: `cmake --preset msvc` (Windows) or `cmake --preset gcc-asan` (Linux), then
`cmake --build --preset <name>` and `ctest --preset <name>`. More in `docs/TESTING.md`; how the
code works is in `docs/ARCHITECTURE.md`.

## License

AGPL-3.0 (see `LICENSE`).
