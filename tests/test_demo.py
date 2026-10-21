"""The whole demo started by run_demo: proxy, ground station window (offscreen) and UAV."""

import json
import os
import subprocess
import sys

import pytest

# The demo opens the dashboard; Qt may not load on a bare Linux machine.
pytest.importorskip("PyQt6.QtWidgets")

from test_proxy import free_port_block  # noqa: E402


def test_run_demo_starts_everything_and_stops_with_the_window(tmp_path):
    snapshot = tmp_path / "demo.png"
    env = dict(os.environ, QT_QPA_PLATFORM="offscreen")
    result = subprocess.run(
        [sys.executable, "-m", "aegislink.tools.run_demo", "--seconds", "8", "--snapshot",
         str(snapshot), "--log", str(tmp_path / "gt.jsonl"), "--base-port",
         str(free_port_block())],
        capture_output=True, text=True, timeout=60, env=env)
    assert result.returncode == 0, result.stderr
    out = json.loads(result.stdout.strip().splitlines()[-1])
    assert out["frame_times"]["frames"] > 300
    assert out["received"]["tracks_hz"] > 25 and out["received"]["baseline_fps"] > 25
    assert snapshot.stat().st_size > 10_000
