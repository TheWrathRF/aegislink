"""The whole demo started by run_demo: proxy, ground station window (offscreen) and UAV."""

import json
import os
import socket
import subprocess
import sys

import pytest

# The demo opens the dashboard; Qt may not load on a bare Linux machine.
pytest.importorskip("PyQt6.QtWidgets")

from test_proxy import free_ports  # noqa: E402


def free_port_block():
    """A base port with the six link ports after it and base + 100 all free."""
    for _ in range(20):
        base = free_ports(1)[0]
        if base > 65000:
            continue
        sockets = []
        try:
            for port in [base + i for i in range(6)] + [base + 100]:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sockets.append(s)
                s.bind(("127.0.0.1", port))
            return base
        except OSError:
            continue
        finally:
            for s in sockets:
                s.close()
    pytest.fail("no free block of ports")


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
