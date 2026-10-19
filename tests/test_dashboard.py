"""The ground station's window (Qt, drawn offscreen) and the EW deck against the real proxy."""

import os
import socket
import subprocess
import sys
import time

import cv2
import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
# Qt needs system libraries that a bare Linux machine may lack; there these tests are skipped.
pytest.importorskip("PyQt6.QtWidgets")
pytest.importorskip("pyqtgraph")

from PyQt6.QtWidgets import QApplication  # noqa: E402

from aegislink._core import MsgType, Packetizer  # noqa: E402
from aegislink.common import config, geo, messages  # noqa: E402
from aegislink.common.proxy_control import ProxyControl  # noqa: E402
from aegislink.gcs.dashboard import Dashboard, FrameClock, dark_palette  # noqa: E402
from aegislink.gcs.ew_deck import RATE_STEPS_KBPS, EwDeck, ProxyLink  # noqa: E402
from aegislink.gcs.receiver import GroundStation, ReceiverThread  # noqa: E402
from test_proxy import find_proxy, free_ports  # noqa: E402

W, H = config.FRAME_WIDTH, config.FRAME_HEIGHT


@pytest.fixture(scope="module")
def app():
    app = QApplication.instance() or QApplication([])
    dark_palette(app)
    return app


@pytest.fixture
def station():
    reports = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    reports.bind(("127.0.0.1", 0))
    s = GroundStation(0, 0, reports.getsockname()[1])
    yield s
    s.close()
    reports.close()


TX = Packetizer()


def deliver(station, msg_type, body, now_s=None, link="fovea"):
    now_s = time.monotonic() if now_s is None else now_s
    for datagram in TX.packetize(msg_type, int(now_s * 1e6), body):
        station._receive(link, datagram, now_s)


def fill(station, frame=40):
    """One frame of everything: telemetry, a track, its red patch, a keyframe, a baseline."""
    lat, lon = geo.enu_to_latlon(0.0, 0.0, (46.0, 7.0))
    deliver(station, MsgType.TELEMETRY, messages.encode_telemetry(
        messages.TelemetryMsg(frame, float(lat), float(lon), 150, 10, 0, 0, 0)))
    box = (600, 300, 680, 340)
    deliver(station, MsgType.TRACKS, messages.encode_tracks(
        frame, [messages.TrackBox(1, box, True)]))
    patch = np.full((56, 96, 3), (0, 0, 255), np.uint8)
    deliver(station, MsgType.ROI_PATCH, messages.encode_patch(messages.PatchMsg(
        frame, 150, 0, 1, (592, 292, 688, 348), cv2.imencode(".webp", patch)[1].tobytes())))
    keyframe = np.full((270, 480), 90, np.uint8)
    deliver(station, MsgType.BG_KEYFRAME, messages.encode_keyframe(messages.KeyframeMsg(
        frame, 150, 0, cv2.imencode(".webp", keyframe)[1].tobytes())))
    full = np.full((H, W, 3), (200, 50, 50), np.uint8)
    deliver(station, MsgType.FULL_FRAME, messages.encode_full_frame(messages.FullFrameMsg(
        frame, cv2.imencode(".jpg", full)[1].tobytes())), link="baseline")


def test_frame_clock_reports_rates_and_percentiles():
    clock = FrameClock(window=3)
    for i in range(5):
        clock.record(i * 0.02, i * 0.02 + 0.004)
    recent = clock.recent()
    assert recent["fps"] == pytest.approx(50)
    assert recent["work_p95_ms"] == pytest.approx(4)
    s = clock.summary()
    assert s["frames"] == 5 and s["fps"] == pytest.approx(50) and s["interval_p95_ms"] == 20


def test_window_draws_both_views_within_a_frame_time(app, station):
    fill(station)
    window = Dashboard(ReceiverThread(station))
    window.show()
    for _ in range(120):
        window.draw_frame()
        app.processEvents()
    fovea = window.fovea_view
    w, h = fovea.render_size()
    assert (fovea.image.width(), fovea.image.height()) == (w, h) and w >= 640
    pixels = fovea._pixels
    # The red patch at the track's box, scaled into the view, on the gray background.
    centre = (round(320 * h / H), round(640 * w / W))
    assert tuple(pixels[centre]) == pytest.approx((0, 0, 255), abs=2)
    assert tuple(pixels[5, 5]) == pytest.approx((90, 90, 90), abs=3)
    baseline = window.baseline_view._pixels
    assert tuple(baseline[h // 2, w // 2]) == pytest.approx((200, 50, 50), abs=4)
    window.update_status()      # numbers and the first plot
    window.update_status()      # the second plot and the status line
    assert "of baseline" in window.fovea_hud.text() and "frames/s" in window.baseline_hud.text()
    assert "frame time p95" in window.status.text()
    s = window.clock.summary()
    assert s["frames"] == 120 and s["work_p95_ms"] < 1000 / 60
    window.close()


def test_a_silent_link_is_shown_as_lost(app, station):
    window = Dashboard(ReceiverThread(station))
    window.draw_frame()
    assert window.fovea_view.banner == ""
    fill(station)
    window.draw_frame()
    assert window.fovea_view.banner == ""
    deliver(station, MsgType.TRACKS, messages.encode_tracks(41, []), now_s=time.monotonic() - 2)
    window.draw_frame()
    assert window.fovea_view.banner.startswith("LINK LOST")
    window.close()


@pytest.fixture
def proxy():
    ports = free_ports(7)
    links = [f"{ports[i]}:{ports[i + 1]}" for i in (0, 2, 4)]
    process = subprocess.Popen(
        [find_proxy(), *sum((["--link", link] for link in links), []), "--control",
         str(ports[6])], stdout=subprocess.DEVNULL)
    control = ProxyControl(ports[6])
    try:
        control.wait_ready()
        yield ports, control
    finally:
        process.terminate()
        process.wait(timeout=5)
        control.close()


def wait_for(condition, deck, app, timeout_s=3.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        deck.update_status()
        app.processEvents()
        if condition():
            return True
        time.sleep(0.02)
    return False


def test_deck_sets_all_three_links(app, proxy):
    ports, control = proxy
    control.configure(1, loss_pct=12)
    deck = EwDeck(ProxyLink(ports[6]))
    assert wait_for(lambda: deck.synced, deck, app)
    assert deck.setting("loss_pct") == 12          # the sliders start at the proxy's settings
    deck.sliders["loss_pct"].setValue(30)
    deck.sliders["burst"].setValue(4)
    deck.sliders["kbps"].setValue(RATE_STEPS_KBPS.index(50))
    deck.sliders["jitter_ms"].setValue(20)
    assert wait_for(lambda: deck.proxy.settings.get(3, {}).get("jitter_ms") == 20, deck, app)
    for link in (1, 2, 3):
        s = control.configure(link)
        assert (s["loss_pct"], s["burst"], s["kbps"], s["jitter_ms"]) == (30, 4, 50, 20)
    deck.blackout_s.setValue(3.0)
    deck.blackout_button.click()
    time.sleep(0.1)
    assert all(0 < control.configure(link)["blackout_ms"] <= 3000 for link in (1, 2, 3))
    assert wait_for(lambda: "BLACKOUT" in deck.status.text(), deck, app)
    deck.clear()
    time.sleep(0.1)
    for link in (1, 2, 3):
        s = control.configure(link)
        assert (s["loss_pct"], s["kbps"], s["jitter_ms"], s["blackout_ms"]) == (0, 0, 0, 0)
    deck.proxy.close()


def test_deck_without_a_proxy_says_so(app):
    port = free_ports(1)[0]
    deck = EwDeck(ProxyLink(port))
    deck.sliders["loss_pct"].setValue(10)      # sent into the void, without an error
    deck.update_status()
    assert "not answering" in deck.status.text()
    deck.proxy.close()


def test_rate_slider_starves_the_baseline_while_tracks_keep_coming(app, proxy, tmp_path):
    """The whole chain: UAV process -> proxy -> this window, with the deck cutting the rate."""
    ports, control = proxy
    station = GroundStation(ports[1], ports[3], ports[4])
    receiver = ReceiverThread(station)
    window = Dashboard(receiver, ProxyLink(ports[6]))
    uav = subprocess.Popen(
        [sys.executable, "-m", "aegislink.uav.main", "--no-preview", "--send", "--seconds", "40",
         "--log", str(tmp_path / "gt.jsonl"), "--fovea-port", str(ports[0]),
         "--baseline-port", str(ports[2]), "--report-port", str(ports[5])],
        stdout=subprocess.DEVNULL)
    receiver.start()
    window.show()
    window.start()

    def run_for(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            app.processEvents()
            time.sleep(0.001)
        return window.last_stats

    try:
        clean = run_for(8)
        assert clean["baseline_fps"] > 25 and clean["tracks_hz"] > 25
        window.deck.sliders["kbps"].setValue(RATE_STEPS_KBPS.index(50))
        limited = run_for(8)
    finally:
        window.close()
        receiver.stop()
        window.proxy.close()
        uav.terminate()
        uav.wait(timeout=5)
        station.close()
    assert control.configure(2)["kbps"] == 50
    assert limited["baseline_fps"] < 1
    # Over one second the proxy lets through at most its rate plus one 1230-byte datagram.
    assert limited["tracks_hz"] >= 28 and limited["fovea_kbps"] <= 50 + 1230 * 8 / 1000
    # The UAV, the proxy and the window share the machine here, so the frame rate depends on the
    # machine (52 fps on 4-core CI runners); this only catches a stalled draw loop. The frame time
    # is checked without that load in test_window_draws_both_views_within_a_frame_time.
    assert window.clock.summary()["fps"] >= 30



class FakeProxy:
    """Something on the proxy's port that answers with whatever the test wants."""

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(1.0)
        self.port = self.sock.getsockname()[1]

    def answer(self, text):
        _, addr = self.sock.recvfrom(65535)
        self.sock.sendto(text.encode(), addr)
        return addr


@pytest.mark.parametrize("reply", [
    "ok link=1 loss_pct=nan", "ok link=1 loss_pct=inf", "ok link=1 loss_pct=1e12 kbps=-5",
    "stats uptime_ms=1\nlink=1 foo=2", "stats uptime_ms=1\nlink=1 out_kbps=nan", "garbage=",
])
def test_deck_survives_bad_replies(app, reply):
    fake = FakeProxy()
    deck = EwDeck(ProxyLink(fake.port))
    deck.update_status()            # asks the proxy for its settings
    fake.answer(reply)
    time.sleep(0.05)
    deck.update_status()
    for key, slider in deck.sliders.items():
        assert slider.minimum() <= slider.value() <= slider.maximum()
    deck.proxy.close()
    fake.sock.close()


def test_deck_ignores_datagrams_from_others(app):
    fake = FakeProxy()
    deck = EwDeck(ProxyLink(fake.port))
    deck.update_status()
    fake.sock.recvfrom(65535)
    other = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    other.sendto(b"ok link=1 loss_pct=40", deck.proxy.sock.getsockname())
    time.sleep(0.05)
    deck.update_status()
    assert deck.proxy.settings == {} and deck.setting("loss_pct") == 0
    other.close()
    deck.proxy.close()
    fake.sock.close()


def test_an_error_while_drawing_is_shown_not_fatal(app, station, monkeypatch):
    window = Dashboard(ReceiverThread(station))
    def broken(*args, **kwargs):
        raise ValueError("broken view")
    monkeypatch.setattr("aegislink.gcs.compositor.render_fovea", broken)
    window.draw_frame()             # raised out of a Qt slot, this would end the process
    monkeypatch.undo()
    window.draw_frame()
    window.update_status()
    window.update_status()
    assert "broken view" in window.status.text()
    window.close()


def test_frame_clock_memory_is_bounded():
    clock = FrameClock(window=10, keep=100)
    for i in range(1000):
        clock.record(i * 0.016, i * 0.016 + 0.002)
    assert len(clock.all_work_s) == 100 and clock.summary()["frames"] == 1000


def test_ctrl_c_closes_the_window_cleanly():
    script = (
        "import signal, socket\n"
        "from PyQt6.QtCore import QTimer\n"
        "from PyQt6.QtWidgets import QApplication\n"
        "app = QApplication([])\n"
        "from aegislink.gcs import dashboard\n"
        "from aegislink.gcs.receiver import GroundStation\n"
        "r = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); r.bind(('127.0.0.1', 0))\n"
        "station = GroundStation(0, 0, r.getsockname()[1])\n"
        "QTimer.singleShot(500, lambda: signal.raise_signal(signal.SIGINT))\n"
        "dashboard.run(station)\n")
    env = dict(os.environ, QT_QPA_PLATFORM="offscreen")
    result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True,
                            timeout=30, env=env)
    assert result.returncode == 0, result.stderr
    assert "frame_times" in result.stdout
