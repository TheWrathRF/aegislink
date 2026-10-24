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

import cv2
import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import QRectF, QSize, Qt, QTimer
from PyQt6.QtGui import QColor, QFont, QFontMetrics, QImage, QPainter, QPalette
from PyQt6.QtWidgets import (QApplication, QGridLayout, QLabel, QMainWindow, QSizePolicy,
                             QVBoxLayout, QWidget)

from aegislink.common import config
from aegislink.gcs import compositor
from aegislink.gcs.ew_deck import EwDeck, ProxyLink
from aegislink.gcs.receiver import ReceiverThread
from aegislink.gcs.spoof_detector import CHI2_99
from aegislink.uav.encoder import decode_image

FRAME_INTERVAL_MS = 16
STATUS_INTERVAL_MS = 125
RECORD_INTERVAL_MS = 100
PLOT_SECONDS = 60
# No foveated datagram (or no new baseline frame) for this long and the view says so.
LINK_LOST_S = 0.5
FROZEN_S = 0.5
# A background older than this is labelled with its age (keyframes come up to twice a second).
OLD_BACKGROUND_S = 1.0
WAITING = "WAITING FOR THE UAV"
# Fixed plot ranges, widened only when a value goes beyond them: before any data the plots
# have something to show, and a line never runs along the top edge (log10 of kbps).
KBPS_RANGE = (-1, 5)
# Log axes cannot show 0; this sits at the bottom of the kbps plot instead.
KBPS_FLOOR = 0.1
RATE_RANGE = (0, 40)
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
    """Draws one BGR image, as large as fits at 16:9, with a title, an optional note under it and
    an optional banner."""

    def __init__(self, title):
        super().__init__()
        self.title = title
        self.note = ""
        self.banner = ""
        self.image = None       # the QImage, and below the array it points into
        self._pixels = None
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setMinimumSize(320, 180)

    def sizeHint(self):
        # 16:9 of the width it has: the window's layout then gives the views their full height
        # before the plots below get any.
        w = self.width() if self.width() > self.minimumWidth() else 640
        return QSize(640, round(w * 9 / 16))

    def resizeEvent(self, event):
        # Taller than 16:9 would only add empty bands; the plots below get that height instead.
        self.setMaximumHeight(max(self.minimumHeight(), round(self.width() * 9 / 16)))
        super().resizeEvent(event)

    def fit_width(self):
        """The width of the 16:9 area that fits, in logical pixels."""
        return min(self.width(), self.height() * 16 / 9)

    def render_size(self):
        """The size in device pixels to render at: the 16:9 area that fits, at most 1280 x 720
        (larger is shown scaled up)."""
        w = self.fit_width() * self.devicePixelRatioF()
        w = int(min(w, config.FRAME_WIDTH)) // 2 * 2
        return max(w, 2), max(w * 9 // 16, 2)

    def text_scale(self):
        """Image pixels per logical pixel: labels drawn this much larger keep their size on the
        screen whatever the window size and the screen's scaling."""
        return self.render_size()[0] / max(self.fit_width(), 1.0)

    def set_frame(self, bgr):
        self._pixels = np.ascontiguousarray(bgr)
        h, w = self._pixels.shape[:2]
        self.image = QImage(self._pixels.data, w, h, 3 * w, QImage.Format.Format_BGR888)
        self.image.setDevicePixelRatio(self.devicePixelRatioF())

    def target_rect(self):
        """Where the image is drawn: at its own size, or scaled up to fill the 16:9 area when its
        render size was capped."""
        if self.image is None:
            return QRectF(0, 0, 0, 0)
        size = self.image.deviceIndependentSize()
        w, h = size.width(), size.height()
        if w < self.fit_width() - 2:
            w = self.fit_width()
            h = w * 9 / 16
        return QRectF((self.width() - w) / 2, (self.height() - h) / 2, w, h)

    def paintEvent(self, event):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor(BACKGROUND))
        target = self.target_rect()
        if self.image is not None:
            p.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
            p.drawImage(target, self.image)
        p.setPen(QColor("#e8e8e8"))
        p.setFont(QFont("Segoe UI", 10, QFont.Weight.Bold))
        p.drawText(int(target.left()) + 10, int(target.top()) + 22, self.title)
        if self.note:
            p.setFont(QFont("Segoe UI", 9))
            p.drawText(int(target.left()) + 10, int(target.top()) + 40, self.note)
        if self.banner:
            p.setPen(QColor("#ff5050"))
            p.setFont(QFont("Segoe UI", 18, QFont.Weight.Bold))
            p.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, self.banner)
        p.end()


def window_size(available):
    """1600 x 960, or the screen's free area (a QRect) where that is smaller."""
    if available is None:
        return 1600, 960
    return min(1600, available.width()), min(960, available.height())


def _hud(text, tip="", lines=1):
    label = QLabel(text)
    label.setFont(QFont("Consolas", 10))
    label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
    label.setToolTip(tip)
    # Room for `lines` lines from the start, so the layout does not jump when the numbers come.
    # They are kept short instead of wrapped (a wrapped label asks for the height of a narrow
    # one, which left a gap under the views); in a very small window they are cut at the edge.
    label.setFixedHeight(QFontMetrics(label.font()).lineSpacing() * lines + 4)
    label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Fixed)
    label.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)
    return label


def _title(name, *curves):
    """A plot title naming its curves in their colours: a legend inside the plot would sit on
    the lines in a short plot."""
    keys = "".join(f'&nbsp;&nbsp;<span style="color:{color}">{label}</span>'
                   for label, color in curves)
    return f"{name}{keys}"


BASELINE_TIP = ("What the full-frame JPEG stream delivers: received kbps and whole frames per "
                "second. Under a rate limit its fragments still trickle in, but no frame "
                "completes.")
FOVEA_TIP = (
    "Received kbps, compared with the highest baseline rate of the last minute (what full-frame "
    "video needs on this link).\n"
    "Filtered tracks have a Kalman filter at the ground station; ghosts are tracks without a "
    "fix for a while, drawn where the filter predicts them, with their 95 % ellipse.\n"
    "The GPS check compares the telemetry's motion with the optical flow between background "
    f"keyframes. d2 is the squared Mahalanobis distance of the last pair: above {CHI2_99} (the "
    "99 % point) a keyframe is suspect, and suspect keyframes in a row raise the alarm.")


def baseline_text(s):
    text = f"{s['baseline_kbps']:,.1f} kbps   {s['baseline_fps']:.1f} frames/s"
    if s["baseline_fps"] == 0 and s["baseline_kbps"] > 0:
        text += "   fragments only, no whole frame"
    return text


def fovea_text(s, baseline_peak_kbps):
    """The foveated stream's numbers; its rate is compared with `baseline_peak_kbps`, the highest
    baseline rate lately: the current one says nothing while a rate limit starves it."""
    kbps = f"{s['fovea_kbps']:,.1f} kbps"
    if baseline_peak_kbps > KBPS_FLOOR:
        kbps += (f" = {s['fovea_kbps'] / baseline_peak_kbps:.1%} of the baseline's "
                 f"{baseline_peak_kbps:,.0f} kbps peak")
    else:
        kbps += " (baseline silent)"
    age = "none" if s["keyframe_age_s"] is None else f"{s['keyframe_age_s']:.1f} s old"
    ghosts = f"filtered tracks {s['filtered_tracks']}, ghosts {s['ghost_tracks']}"
    if s["ghost_tracks"]:
        ghosts += (f"   up to {s['max_tslf_s']:.1f} s without a fix, 95 % area up to "
                   f"{s['max_ellipse_m2']:,.0f} m2")
    return (f"{kbps}\n"
            f"telemetry {s['telemetry_hz']:.1f} Hz   tracks {s['tracks_hz']:.1f} Hz   "
            f"patches {s['patches_per_s']:.1f}/s   keyframe {age}\n{ghosts}\n{nav_text(s)}")


def nav_text(s):
    """The GPS check: the alarm, both speeds over the last keyframe pair and its d2."""
    paired = s["flow_speed"] is not None and s["gps_speed"] is not None
    if s["gps_alarm"]:
        state = "GPS SPOOFED"       # the view's banner says what the station does about it
    elif s["fovea_silent_s"] is None:
        return "GPS: no data"
    elif not paired:
        return "GPS not checked yet (no keyframe pair the flow could follow)"
    else:
        state = "GPS checked"
    if not paired:
        return state
    return (f"{state}   GPS {s['gps_speed']:.1f} m/s   flow {s['flow_speed']:.1f} m/s   "
            f"d2 {s['spoof_d2']:,.1f} (alarm > {CHI2_99})   alarms {s['gps_alarms']}")


class Dashboard(QMainWindow):
    def __init__(self, receiver, proxy=None, snapshot=None, record=None):
        super().__init__()
        self.receiver = receiver
        self.proxy = proxy
        self.snapshot = snapshot    # where to save the window as an image when it closes
        self.record = record        # a folder for a picture of the window every RECORD_INTERVAL_MS
        self.recorded = 0
        self.clock = FrameClock()
        self.error = None                       # the last exception while drawing
        self._baseline = (None, None)           # (frame, decoded image)
        self._baseline_scaled = (None, None)    # ((frame, size, dimmed), scaled image)
        self.setWindowTitle("AegisLink ground station")
        screen = QApplication.primaryScreen()
        self.resize(*window_size(None if screen is None else screen.availableGeometry()))

        self.baseline_view = VideoView("BASELINE  full-frame JPEG")
        self.fovea_view = VideoView("AEGISLINK  foveated")
        self.baseline_hud = _hud("", BASELINE_TIP, lines=4)
        self.fovea_hud = _hud("", FOVEA_TIP, lines=4)
        self.status = _hud("")

        self.history = {key: deque(maxlen=PLOT_SECONDS * 4) for key in
                        ("t", "fovea_kbps", "baseline_kbps", "tracks_hz", "patches_per_s")}
        self.started_s = time.monotonic()
        self._first_half = False
        colors = {"fovea_kbps": "#4cc2ff", "baseline_kbps": "#ff9f43", "tracks_hz": "#7bed9f",
                  "patches_per_s": "#eccc68"}
        self.kbps_plot = pg.PlotWidget(title=_title(
            "Received kbps", ("foveated", colors["fovea_kbps"]),
            ("baseline", colors["baseline_kbps"])))
        self.kbps_plot.setLogMode(y=True)
        self.rate_plot = pg.PlotWidget(title=_title(
            "Messages per second", ("tracks", colors["tracks_hz"]),
            ("patches", colors["patches_per_s"])))
        self.curves = {key: (self.kbps_plot if key.endswith("kbps") else self.rate_plot).plot(
            pen=pg.mkPen(color, width=2)) for key, color in colors.items()}
        self.ranges = {self.kbps_plot: KBPS_RANGE, self.rate_plot: RATE_RANGE}
        for plot, (low, high) in self.ranges.items():
            plot.showGrid(x=True, y=True, alpha=0.2)
            plot.setLabel("bottom", "seconds")
            plot.setYRange(low, high, padding=0)
            plot.setMinimumHeight(120)
            # Its size hint (640 x 480) would otherwise take height from the video first.
            plot.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Ignored)

        grid = QGridLayout()
        grid.addWidget(self.baseline_view, 0, 0)
        grid.addWidget(self.fovea_view, 0, 1)
        grid.addWidget(self.baseline_hud, 1, 0)
        grid.addWidget(self.fovea_hud, 1, 1)
        grid.setRowStretch(0, 1)
        bottom = QGridLayout()
        if proxy is not None:
            self.deck = EwDeck(proxy)
            bottom.addWidget(self.deck, 0, 0)
        else:
            self.deck = None
            bottom.addWidget(QLabel("No proxy (--direct): the link cannot be changed."), 0, 0)
        bottom.addWidget(self.kbps_plot, 0, 1)
        bottom.addWidget(self.rate_plot, 0, 2)
        bottom.setColumnStretch(1, 1)
        bottom.setColumnStretch(2, 1)
        layout = QVBoxLayout()
        # The views grow first (up to 16:9 of their width), then the plots take what is left.
        layout.addLayout(grid, 1)
        layout.addLayout(bottom, 0)
        layout.addWidget(self.status)
        central = QWidget()
        central.setLayout(layout)
        self.setCentralWidget(central)

        self.frame_timer = QTimer(self)
        self.frame_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self.frame_timer.timeout.connect(self.draw_frame)
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.update_status)
        self.record_timer = QTimer(self)
        self.record_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self.record_timer.timeout.connect(self.save_picture)

    def save_picture(self):
        # A timer slot like draw_frame: an exception leaving it would end the process.
        try:
            if self.recorded == 0:
                self.record.mkdir(parents=True, exist_ok=True)
            path = self.record / f"{self.recorded:05d}.jpg"
            # JPEG: saving a PNG of the window takes a few frames' time.
            if not self.grab().save(str(path), quality=90):
                raise OSError(f"cannot write {path}")
            self.recorded += 1
        except Exception as e:
            self.record_timer.stop()
            self.error = OSError(f"recording stopped: {e}")

    def closeEvent(self, event):
        self.record_timer.stop()
        if self.snapshot is not None:
            self.snapshot.parent.mkdir(parents=True, exist_ok=True)
            self.grab().save(str(self.snapshot))
        super().closeEvent(event)

    def start(self):
        self.frame_timer.start(FRAME_INTERVAL_MS)
        self.status_timer.start(STATUS_INTERVAL_MS)
        if self.record is not None:
            self.record_timer.start(RECORD_INTERVAL_MS)

    def _baseline_image(self, full_frame, size, dim):
        """The newest baseline frame at `size` (dimmed when `dim`), decoded and scaled once per
        frame."""
        if full_frame is None:
            return compositor.render_baseline(None, size)
        if self._baseline[0] != full_frame.frame:
            image = decode_image(full_frame.image, size=(config.FRAME_WIDTH, config.FRAME_HEIGHT))
            self._baseline = (full_frame.frame, image)
        key = (full_frame.frame, size, dim)
        if self._baseline_scaled[0] != key:
            image = compositor.render_baseline(self._baseline[1], size)
            if dim and image is not None:
                image = cv2.convertScaleAbs(image, alpha=0.45)
            self._baseline_scaled = (key, image)
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
        fovea = self.fovea_view
        fovea.set_frame(compositor.render_fovea(view, fovea.render_size(),
                                                text_scale=fovea.text_scale()))
        silent, frozen = stats["fovea_silent_s"], stats["baseline_silent_s"]
        self.baseline_view.set_frame(self._baseline_image(
            full_frame, self.baseline_view.render_size(), frozen is not None and frozen >= FROZEN_S))
        banners = []
        if silent is None:
            banners.append(WAITING)
        elif silent >= LINK_LOST_S:
            banners.append(f"LINK LOST  {silent:.1f} s")
        if stats["gps_alarm"]:
            banners.append("GPS SPOOFED  optical-flow navigation")
        fovea.banner = "\n".join(banners)
        age = stats["keyframe_age_s"]
        fovea.note = (f"background {age:.1f} s old"
                      if age is not None and age >= OLD_BACKGROUND_S else "")
        if frozen is None:
            self.baseline_view.banner = WAITING if silent is None else "NO FRAME YET"
        else:
            self.baseline_view.banner = f"FROZEN  {frozen:.1f} s" if frozen >= FROZEN_S else ""
        # repaint() draws now, so the frame time includes the drawing.
        self.fovea_view.repaint()
        self.baseline_view.repaint()
        self.clock.record(start, time.perf_counter())
        self.last_stats = stats

    def _plot(self, plot, *keys):
        t = list(self.history["t"])
        for key in keys:
            self.curves[key].setData(t, list(self.history[key]))
        low, high = self.ranges[plot]
        peak = max((max(self.history[key], default=0) for key in keys), default=0)
        if plot is self.kbps_plot:
            high = max(high, np.log10(max(peak, 1)) + 0.5)
        else:
            high = max(high, 1.1 * peak)
        plot.setYRange(low, high, padding=0)

    def _update_status(self):
        """Runs every 125 ms and does half of the work in turn: a plot takes a few milliseconds
        to paint, and both in one frame would make the next frame late."""
        stats = getattr(self, "last_stats", None)
        if stats is None:
            return
        self._first_half = not self._first_half
        if self._first_half:
            peak = max(stats["baseline_kbps"], *self.history["baseline_kbps"], 0)
            self.baseline_hud.setText(baseline_text(stats))
            self.fovea_hud.setText(fovea_text(stats, peak))
            self.history["t"].append(time.monotonic() - self.started_s)
            for key in self.curves:
                floor = KBPS_FLOOR if key.endswith("kbps") else 0.0
                self.history[key].append(max(stats[key], floor))
            self._plot(self.kbps_plot, "fovea_kbps", "baseline_kbps")
            return
        self._plot(self.rate_plot, "tracks_hz", "patches_per_s")
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


def run(station, proxy_port=None, seconds=None, snapshot=None, record=None, warmup_s=0.0):
    """Shows the window until it is closed (or for `seconds`), then prints its frame times, what
    arrived from `warmup_s` after the first foveated datagram until the window closed (see
    GroundStation.summary) and any error as one JSON line; returns the frame times."""
    app = QApplication.instance() or QApplication([])
    dark_palette(app)
    receiver = ReceiverThread(station, warmup_s=warmup_s)
    proxy = ProxyLink(proxy_port) if proxy_port is not None else None
    window = Dashboard(receiver, proxy, snapshot, record)
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
        window.record_timer.stop()
        with receiver.lock:
            received = station.summary(time.monotonic())
        receiver.stop()
        if proxy is not None:
            proxy.close()
    summary = window.clock.summary()
    errors = [repr(e) for e in (receiver.error, window.error) if e is not None]
    print(json.dumps({"frame_times": summary,
                      "received": received or {"error": "no foveated stream arrived"},
                      "errors": errors}), flush=True)
    return summary
