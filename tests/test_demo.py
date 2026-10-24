"""The whole demo started by run_demo: proxy, ground station window (offscreen) and UAV."""

import json
import os
import signal
import socket
import subprocess
import sys

import pytest

# The demo opens the dashboard; Qt may not load on a bare Linux machine.
pytest.importorskip("PyQt6.QtWidgets")

from test_proxy import free_port_block  # noqa: E402

pytestmark = pytest.mark.slow


def run_demo(*args, timeout=60):
    """Runs run_demo with the window offscreen. On a timeout the whole process tree is killed:
    killing run_demo alone would leave its proxy holding the ports."""
    env = dict(os.environ, QT_QPA_PLATFORM="offscreen")
    windows = sys.platform == "win32"
    process = subprocess.Popen(
        [sys.executable, "-m", "aegislink.tools.run_demo", *args],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=env,
        start_new_session=not windows)
    try:
        out, err = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        if windows:
            subprocess.run(["taskkill", "/T", "/F", "/PID", str(process.pid)],
                           capture_output=True)
        else:
            os.killpg(process.pid, signal.SIGKILL)
        process.communicate()
        raise
    return process.returncode, out, err


def test_run_demo_starts_everything_and_stops_with_the_window(tmp_path):
    snapshot = tmp_path / "demo.png"
    code, out, err = run_demo("--seconds", "8", "--snapshot", str(snapshot),
                              "--log", str(tmp_path / "gt.jsonl"),
                              "--base-port", str(free_port_block()))
    assert code == 0, err
    result = json.loads(out.strip().splitlines()[-1])
    assert result["frame_times"]["frames"] > 300
    # Counted by frame number over the whole run: how fast the UAV renders depends on the
    # machine (a busy CI runner gives it under 30 frames per second), but whatever it makes must
    # arrive. The rates only catch a stall.
    received = result["received"]
    assert received["tracks_delivered"] >= 0.97 and received["baseline_delivered"] >= 0.97
    assert received["tracks_hz"] >= 15 and received["baseline_fps"] >= 15
    assert received["uav_fps"] >= 15 and result["errors"] == []
    assert snapshot.stat().st_size > 10_000


def test_run_demo_fails_when_the_uav_cannot_start(tmp_path):
    base = free_port_block()
    # The UAV receives the reports on base + 5; with that port taken it stops at once.
    taken = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    taken.bind(("127.0.0.1", base + 5))
    try:
        code, out, err = run_demo("--seconds", "3", "--log", str(tmp_path / "gt.jsonl"),
                                  "--base-port", str(base))
    finally:
        taken.close()
    assert code != 0
    assert "UAV" in err
