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

Python module and tests (builds `aegislink._core` into `build/<wheel tag>`, about 15 s the first
time; rerun the install after any C++ change):
```
python -m pip install -e ".[test]"
python -m pytest
```

Loopback check (a 50 KB message over loopback UDP in 42 fragments):
```
python -m aegislink.tools.loopback_check --size 50000
```

## What the tests cover
- `cpp/tests/test_crc32`: catalogued check values, chaining.
- `cpp/tests/test_protocol`: byte layout, round trip, every rejection path, a flipped bit
  anywhere, packetizing (50,000 B -> 42 fragments).
- `cpp/tests/test_reassembler`: in order, reversed, shuffled, duplicates (also after completion),
  timeout at exactly 200 ms, interleaved messages, inconsistent fragments, eviction.
- `cpp/tests/test_fuzz`: 200,000 random datagrams (none accepted); 3000 messages with random bit
  flips, truncation and drops (no corrupt message is ever delivered). About 4 s in a Debug build.
- `tests/test_bindings.py`: the module against a `struct` + `zlib` reference of the format.
- `tests/test_loopback.py`: real UDP sockets on 127.0.0.1.

## CI
`.github/workflows/ci.yml` runs on every push: Windows (`msvc` preset) and Ubuntu (`gcc-asan`
preset), CTest, then `pip install ".[test]"` and pytest.
