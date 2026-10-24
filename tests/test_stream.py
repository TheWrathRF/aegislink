"""UAV -> ew_chaos_proxy -> GCS as three real processes, headless, measured by the GCS."""

import json
import subprocess
import sys

import pytest

from aegislink.common.proxy_control import ProxyControl
from test_proxy import find_proxy, free_ports

pytestmark = pytest.mark.slow

WARMUP_S = 5
MEASURE_S = 8


@pytest.fixture
def links(tmp_path):
    """Starts the proxy with the three links; yields (control, ports, run)."""
    fovea_in, fovea_out, base_in, base_out, rep_in, rep_out, control_port = free_ports(7)
    proxy = subprocess.Popen(
        [find_proxy(), "--link", f"{fovea_in}:{fovea_out}", "--link", f"{base_in}:{base_out}",
         "--link", f"{rep_in}:{rep_out}", "--control", str(control_port), "--seed", "1"],
        stdout=subprocess.DEVNULL)
    control = ProxyControl(control_port)

    def run():
        gcs = subprocess.Popen(
            [sys.executable, "-m", "aegislink.gcs.main", "--no-view", "--summary",
             "--seconds", str(MEASURE_S), "--warmup", str(WARMUP_S),
             "--fovea-port", str(fovea_out), "--baseline-port", str(base_out),
             "--report-port", str(rep_in)],
            stdout=subprocess.PIPE, text=True)
        uav = subprocess.Popen(
            [sys.executable, "-m", "aegislink.uav.main", "--no-preview", "--send",
             "--seconds", str(WARMUP_S + MEASURE_S + 10), "--log", str(tmp_path / "gt.jsonl"),
             "--fovea-port", str(fovea_in), "--baseline-port", str(base_in),
             "--report-port", str(rep_out)],
            stdout=subprocess.DEVNULL)
        try:
            out, _ = gcs.communicate(timeout=WARMUP_S + MEASURE_S + 30)
        finally:
            for process in (uav, gcs):
                if process.poll() is None:
                    process.terminate()
                    process.wait(timeout=5)
        assert gcs.returncode == 0, out
        return json.loads(out.strip().splitlines()[-1])

    try:
        control.wait_ready()
        yield control, run
    finally:
        proxy.terminate()
        proxy.wait(timeout=5)
        control.close()


def test_clean_link_foveated_is_a_small_share_of_the_baseline(links):
    control, run = links
    s = run()
    assert s["bad_packets"] == 0 and s["bad_messages"] == 0
    assert s["fovea_share"] <= 0.10
    # Counted by frame number: the UAV makes fewer than 30 frames a second on a busy machine,
    # and the rates only catch a stall.
    assert s["tracks_delivered"] >= 0.99 and s["telemetry_delivered"] >= 0.99
    assert s["baseline_delivered"] >= 0.99 and s["baseline_fps"] >= 15
    assert s["patches_per_s"] >= 5
    assert s["keyframes"] >= MEASURE_S - 2


def test_50_kbps_keeps_tracks_at_30_hz(links):
    control, run = links
    for link in (1, 2):
        control.configure(link, kbps=50)
    s = run()
    assert s["fovea_kbps"] <= 50 * 1.01
    assert s["tracks_delivered"] >= 0.99 and s["tracks_hz"] >= 15
    assert s["telemetry_delivered"] >= 0.99
    assert s["patches_per_s"] > 0
    # The baseline cannot get a single frame through.
    assert s["baseline_fps"] < 1
