"""The ground station's window: the baseline and the foveated stream side by side, the numbers
under each, plots of the last minute, and the EW deck that drives the proxy.

Receiving runs in its own thread (`ReceiverThread`); this thread only draws. A precise 16 ms
timer draws both views (about 60 times a second, twice per camera frame) and records how long
each frame took; the numbers and plots are refreshed 4 times a second.
"""

import json
import signal
import time
from collections import deque

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import QRectF, Qt, QTimer
from PyQt6.QtGui import QColor, QFont, QImage, QPainter, QPalette
from PyQt6.QtWidgets import (QApplication, QGridLayout, QLabel, QMainWindow, QSizePolicy,
                             QVBoxLayout, QWidget)

from aegislink.common import config
from aegislink.gcs import compositor
from aegislink.gcs.ew_deck import EwDeck, ProxyLink
from aegislink.gcs.receiver import ReceiverThread
from aegislink.uav.encoder import decode_image

FRAME_INTERVAL_MS = 16
STATUS_INTERVAL_MS = 125
PLOT_SECONDS = 60
# No foveated datagram for this long and the view says so.
LINK_LOST_S = 0.5
BACKGROUND = "#15181c"


class FrameClock:
    """Frame times of the draw loop: how long each frame's work took, and the time between frame
    starts. Keeps the last `window` frames for the display and the last `keep` (10 minutes at
    60 Hz) for the summary, and counts all of them."""

    def __init__(self, window=600, keep=36000):
        self.work_s = deque(maxlen=window)
        self.interval_s = deque(maxlen=window)
        self.all_work_s = deque(maxlen=keep)
        self.all_interval_s = deque(maxlen=keep)
        self.frames = 0
        self.last_start_s = None

    def record(self, start_s, end_s):
        self.frames += 1
        self.work_s.append(end_s - start_s)
        self.all_work_s.append(end_s - start_s)
        if self.last_start_s is not None:
            self.interval_s.append(start_s - self.last_start_s)
            self.all_interval_s.append(start_s - self.last_start_s)
        self.last_start_s = start_s

    @staticmethod
    def _p95_ms(values):
        return float(np.percentile(values, 95) * 1000) if values else None

    def recent(self):
        fps = len(self.interval_s) / sum(self.interval_s) if self.interval_s else None
        return {"fps": fps, "work_p95_ms": self._p95_ms(self.work_s),
                "interval_p95_ms": self._p95_ms(self.interval_s)}

    def summary(self):
        total = sum(self.all_interval_s)
        return {"frames": self.frames,
                "fps": round(len(self.all_interval_s) / total, 1) if total else None,
                "work_p95_ms": _round(self._p95_ms(self.all_work_s)),
                "work_max_ms": _round(max(self.all_work_s, default=0) * 1000),
                "interval_p95_ms": _round(self._p95_ms(self.all_interval_s))}


def _round(x):
    return None if x is None else round(x, 2)


class VideoView(QWidget):
    """Draws one BGR image, as large as fits at 16:9, with a title and an optional banner."""

    def __init__(self, title):
        super().__init__()
        self.title = title
        self.banner = ""
        self.image = None       # the QImage, and below the array it points into
        self._pixels = None
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setMinimumSize(320, 180)

    def render_size(self):
        """The size in device pixels to render at: the 16:9 area that fits, at most 1280 x 720."""
        ratio = self.devicePixelRatioF()
        w = min(self.width(), self.height() * 16 / 9) * ratio
        w = int(min(w, config.FRAME_WIDTH)) // 2 * 2
        return max(w, 2), max(w * 9 // 16, 2)

    def set_frame(self, bgr):
        self._pixels = np.ascontiguousarray(bgr)
        h, w = self._pixels.shape[:2]
        self.image = QImage(self._pixels.data, w, h, 3 * w, QImage.Format.Format_BGR888)
        self.image.setDevicePixelRatio(self.devicePixelRatioF())

    def paintEvent(self, event):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor(BACKGROUND))
        target = QRectF(0, 0, 0, 0)
        if self.image is not None:
            size = self.image.deviceIndependentSize()
            target = QRectF((self.width() - size.width()) / 2, (self.height() - size.height()) / 2,
                            size.width(), size.height())
            p.drawImage(target.topLeft(), self.image)
        p.setPen(QColor("#e8e8e8"))
        p.setFont(QFont("Segoe UI", 10, QFont.Weight.Bold))
        p.drawText(int(target.left()) + 10, int(target.top()) + 22, self.title)
        if self.banner:
            p.setPen(QColor("#ff5050"))
            p.setFont(QFont("Segoe UI", 18, QFont.Weight.Bold))
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, self.banner)
        p.end()


def _hud(text):
    label = QLabel(text)
    label.setFont(QFont("Consolas", 10))
    label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
    return label


def baseline_text(s):
    return f"{s['baseline_kbps']:,.1f} kbps   {s['baseline_fps']:.1f} frames/s"


def fovea_text(s):
    share = (f"{s['fovea_kbps'] / s['baseline_kbps']:.1%} of baseline"
             if s["baseline_kbps"] > 0 else "baseline silent")
    age = "none" if s["keyframe_age_s"] is None else f"{s['keyframe_age_s']:.1f} s old"
    ghosts = f"filtered tracks {s['filtered_tracks']}, ghosts {s['ghost_tracks']}"
    if s["ghost_tracks"]:
        ghosts += (f"   TSLF up to {s['max_tslf_s']:.1f} s   largest 95 % ellipse "
                   f"{s['max_ellipse_m2']:,.0f} m2")
    return (f"{s['fovea_kbps']:,.1f} kbps ({share})\n"
            f"telemetry {s['telemetry_hz']:.1f} Hz   tracks {s['tracks_hz']:.1f} Hz   "
            f"patches {s['patches_per_s']:.1f}/s   keyframe {age}\n{ghosts}\n{nav_text(s)}")


def nav_text(s):
    """The GPS check: the alarm, both speeds over the last keyframe pair and its d2."""
    state = "GPS SPOOFED: navigating by optical flow" if s["gps_alarm"] else "GPS checked"
    if s["flow_speed"] is None or s["gps_speed"] is None:
        return f"{state}   (no keyframe pair yet)"
    return (f"{state}   GPS {s['gps_speed']:.1f} m/s   flow {s['flow_speed']:.1f} m/s   "
            f"d2 {s['spoof_d2']:,.1f}   alarms {s['gps_alarms']}")


class Dashboard(QMainWindow):
    def __init__(self, receiver, proxy=None, snapshot=None):
        super().__init__()
        self.receiver = receiver
        self.proxy = proxy
        self.snapshot = snapshot    # where to save the window as an image when it closes
        self.clock = FrameClock()
        self.error = None                       # the last exception while drawing
        self._baseline = (None, None)           # (frame, decoded image)
        self._baseline_scaled = (None, None)    # ((frame, size), scaled image)
        self.setWindowTitle("AegisLink ground station")
        self.resize(1600, 960)

        self.baseline_view = VideoView("BASELINE  full-frame JPEG")
        self.fovea_view = VideoView("AEGISLINK  foveated")
        self.baseline_hud = _hud("")
        self.fovea_hud = _hud("")
        self.status = _hud("")

        self.history = {key: deque(maxlen=PLOT_SECONDS * 4) for key in
                        ("t", "fovea_kbps", "baseline_kbps", "tracks_hz", "patches_per_s")}
        self.started_s = time.monotonic()
        self._first_half = False
        kbps_plot = pg.PlotWidget(title="Received kbps")
        kbps_plot.setLogMode(y=True)
        kbps_plot.addLegend(offset=(-10, 10))
        self.curves = {
            "fovea_kbps": kbps_plot.plot(pen=pg.mkPen("#4cc2ff", width=2), name="foveated"),
            "baseline_kbps": kbps_plot.plot(pen=pg.mkPen("#ff9f43", width=2), name="baseline"),
        }
        rate_plot = pg.PlotWidget(title="Messages per second")
        rate_plot.addLegend(offset=(-10, 10))
        self.curves["tracks_hz"] = rate_plot.plot(pen=pg.mkPen("#7bed9f", width=2), name="tracks")
        self.curves["patches_per_s"] = rate_plot.plot(pen=pg.mkPen("#eccc68", width=2),
                                                      name="patches")
        for plot in (kbps_plot, rate_plot):
            plot.showGrid(x=True, y=True, alpha=0.2)
            plot.setLabel("bottom", "seconds")
            plot.setMinimumHeight(200)

        grid = QGridLayout()
        grid.addWidget(self.baseline_view, 0, 0)
        grid.addWidget(self.fovea_view, 0, 1)
        grid.addWidget(self.baseline_hud, 1, 0)
        grid.addWidget(self.fovea_hud, 1, 1)
        grid.setRowStretch(0, 3)
        bottom = QGridLayout()
        if proxy is not None:
            self.deck = EwDeck(proxy)
            bottom.addWidget(self.deck, 0, 0)
        else:
            self.deck = None
            bottom.addWidget(QLabel("No proxy (--direct): the link cannot be changed."), 0, 0)
        bottom.addWidget(kbps_plot, 0, 1)
        bottom.addWidget(rate_plot, 0, 2)
        bottom.setColumnStretch(1, 1)
        bottom.setColumnStretch(2, 1)
        layout = QVBoxLayout()
        layout.addLayout(grid, 3)
        layout.addLayout(bottom, 1)
        layout.addWidget(self.status)
        central = QWidget()
        central.setLayout(layout)
        self.setCentralWidget(central)

        self.frame_timer = QTimer(self)
        self.frame_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self.frame_timer.timeout.connect(self.draw_frame)
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.update_status)

    def closeEvent(self, event):
        if self.snapshot is not None:
            self.snapshot.parent.mkdir(parents=True, exist_ok=True)
            self.grab().save(str(self.snapshot))
        super().closeEvent(event)

    def start(self):
        self.frame_timer.start(FRAME_INTERVAL_MS)
        self.status_timer.start(STATUS_INTERVAL_MS)

    def _baseline_image(self, full_frame, size):
        """The newest baseline frame at `size`, decoded and scaled once per frame."""
        if full_frame is None:
            return compositor.render_baseline(None, size)
        if self._baseline[0] != full_frame.frame:
            image = decode_image(full_frame.image, size=(config.FRAME_WIDTH, config.FRAME_HEIGHT))
            self._baseline = (full_frame.frame, image)
        key = (full_frame.frame, size)
        if self._baseline_scaled[0] != key:
            self._baseline_scaled = (key, compositor.render_baseline(self._baseline[1], size))
        return self._baseline_scaled[1]

    # An exception that leaves a Qt slot ends the whole process (PyQt6), so the two timer slots
    # catch everything and the status line shows it instead.
    def draw_frame(self):
        try:
            self._draw_frame()
        except Exception as e:
            self.error = e

    def update_status(self):
        try:
            self._update_status()
        except Exception as e:
            self.error = e
            self.status.setText(f"error: {e!r}")

    def _draw_frame(self):
        start = time.perf_counter()
        view, full_frame, stats = self.receiver.snapshot()
        self.fovea_view.set_frame(compositor.render_fovea(view, self.fovea_view.render_size()))
        self.baseline_view.set_frame(
            self._baseline_image(full_frame, self.baseline_view.render_size()))
        silent = stats["fovea_silent_s"]
        banners = []
        if silent is not None and silent >= LINK_LOST_S:
            banners.append(f"LINK LOST  {silent:.1f} s")
        if stats["gps_alarm"]:
            banners.append("GPS SPOOFED  optical-flow navigation")
        self.fovea_view.banner = "\n".join(banners)
        # repaint() draws now, so the frame time includes the drawing.
        self.fovea_view.repaint()
        self.baseline_view.repaint()
        self.clock.record(start, time.perf_counter())
        self.last_stats = stats

    def _plot(self, *keys):
        t = list(self.history["t"])
        for key in keys:
            self.curves[key].setData(t, list(self.history[key]))

    def _update_status(self):
        """Runs every 125 ms and does half of the work in turn: a plot takes a few milliseconds
        to paint, and both in one frame would make the next frame late."""
        stats = getattr(self, "last_stats", None)
        if stats is None:
            return
        self._first_half = not self._first_half
        if self._first_half:
            self.baseline_hud.setText(baseline_text(stats))
            self.fovea_hud.setText(fovea_text(stats))
            self.history["t"].append(time.monotonic() - self.started_s)
            for key in self.curves:
                # Log axes cannot show 0; 0.1 kbps sits at the bottom of the plot instead.
                floor = 0.1 if key.endswith("kbps") else 0.0
                self.history[key].append(max(stats[key], floor))
            self._plot("fovea_kbps", "baseline_kbps")
            return
        self._plot("tracks_hz", "patches_per_s")
        frames = self.clock.recent()
        parts = []
        if frames["fps"] is not None:
            parts.append(f"draw {frames['fps']:.1f} fps, frame time p95 "
                         f"{frames['work_p95_ms']:.1f} ms, interval p95 "
                         f"{frames['interval_p95_ms']:.1f} ms")
        parts.append(f"bad packets {stats['bad_packets']}, bad messages {stats['bad_messages']}, "
                     f"expired {stats['expired']}")
        if self.receiver.error is not None:
            parts.append(f"receiver stopped: {self.receiver.error!r}")
        if self.error is not None:
            parts.append(f"error: {self.error!r}")
        self.status.setText("   |   ".join(parts))
        if self.deck is not None:
            self.deck.update_status()


def dark_palette(app):
    app.setStyle("Fusion")
    palette = QPalette()
    colors = {QPalette.ColorRole.Window: "#1e2227", QPalette.ColorRole.WindowText: "#e8e8e8",
              QPalette.ColorRole.Base: "#15181c", QPalette.ColorRole.AlternateBase: "#22262c",
              QPalette.ColorRole.Text: "#e8e8e8", QPalette.ColorRole.Button: "#2a2f36",
              QPalette.ColorRole.ButtonText: "#e8e8e8", QPalette.ColorRole.Highlight: "#4cc2ff",
              QPalette.ColorRole.HighlightedText: "#000000",
              QPalette.ColorRole.ToolTipBase: "#2a2f36", QPalette.ColorRole.ToolTipText: "#e8e8e8"}
    for role, color in colors.items():
        palette.setColor(role, QColor(color))
    app.setPalette(palette)
    pg.setConfigOptions(background=BACKGROUND, foreground="#c8c8c8", antialias=True)


def run(station, proxy_port=None, seconds=None, snapshot=None):
    """Shows the window until it is closed (or for `seconds`), then prints its frame times and
    the last second's numbers as one JSON line; returns the frame times."""
    app = QApplication.instance() or QApplication([])
    dark_palette(app)
    receiver = ReceiverThread(station)
    proxy = ProxyLink(proxy_port) if proxy_port is not None else None
    window = Dashboard(receiver, proxy, snapshot)
    # Ctrl+C would otherwise raise inside a timer slot and end the process at once, without
    # stopping the receiver or printing the frame times.
    previous = signal.signal(signal.SIGINT, lambda *args: app.quit())
    receiver.start()
    try:
        window.show()
        window.start()
        if seconds is not None:
            QTimer.singleShot(round(seconds * 1000), window.close)
        app.exec()
    finally:
        signal.signal(signal.SIGINT, previous)
        window.frame_timer.stop()
        window.status_timer.stop()
        receiver.stop()
        if proxy is not None:
            proxy.close()
    summary = window.clock.summary()
    received = {key: _round(value) if isinstance(value, float) else value
                for key, value in getattr(window, "last_stats", {}).items()}
    print(json.dumps({"frame_times": summary, "received": received}), flush=True)
    return summary
