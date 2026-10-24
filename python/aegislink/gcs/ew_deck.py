"""The EW deck: sliders that set the proxy's impairments while the link runs.

The jammer hits the radio channel, so every change goes to all three links (both video links and
the uplink of receiver reports). The deck never waits for the proxy: commands go out and replies
and stats are read whenever they have arrived, so a missing proxy cannot freeze the window.
"""

import math
import socket
import time

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (QDoubleSpinBox, QGridLayout, QGroupBox, QLabel, QPushButton,
                             QSizePolicy, QSlider)

from aegislink.common import config
from aegislink.common.proxy_control import parse_pairs, parse_stats

LINKS = (config.FOVEA_LINK, config.BASELINE_LINK, config.REPORTS_LINK)
# The proxy sends its stats to whoever sent the last command; asking again after this long
# without a word brings them back (after a restart of the proxy, or a script that took them).
SILENT_S = 2.0
RATE_STEPS_KBPS = [0, 15, 25, 50, 100, 250, 500, 1000, 2000, 5000, 20000]   # 0 = no limit
STATS_KEYS = ("out_kbps", "drop_loss", "drop_queue", "drop_blackout", "blackout_ms")
MAX_LOSS_PCT = 60
ERROR_COLOR = "#ff5050"


def _finite(pairs):
    return all(math.isfinite(v) for v in pairs.values())


def _valid_stats(stats):
    return all(all(k in s for k in STATS_KEYS) and _finite(s) for s in stats["links"].values())


def max_loss_pct(burst):
    """The most loss the proxy accepts for a mean burst of `burst` packets: L / (L + 1)."""
    return min(MAX_LOSS_PCT, 100 * burst // (burst + 1))


class ProxyLink:
    """A non-blocking client for the proxy's control port."""

    def __init__(self, port, links=LINKS):
        self.addr = ("127.0.0.1", port)
        self.links = links
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.setblocking(False)
        self.settings = {}          # link -> its settings from the proxy's last reply
        self.stats = None           # the last stats report
        self.error = None           # the proxy's last "error: ..." reply
        self.refusals = 0           # how many commands the proxy refused
        self.last_heard_s = None
        self.last_asked_s = None

    def close(self):
        self.sock.close()

    def send(self, **settings):
        """Sends the same settings to every link; send() alone asks for the current ones."""
        self.last_asked_s = time.monotonic()
        for link in self.links:
            text = " ".join([f"link={link}"] + [f"{k}={v}" for k, v in settings.items()])
            try:
                self.sock.sendto(text.encode(), self.addr)
            except OSError:
                pass        # the proxy is not running; poll() reports it as silent

    def poll(self, now_s=None):
        """Reads whatever has arrived; asks again if the proxy has been silent too long."""
        now_s = time.monotonic() if now_s is None else now_s
        while True:
            try:
                data, addr = self.sock.recvfrom(65535)
            except BlockingIOError:
                break
            except OSError:     # Windows: an ICMP "port unreachable" from an earlier send
                continue
            if addr != self.addr:
                continue
            text = data.decode(errors="replace")
            self.last_heard_s = now_s
            # Anything that does not parse, or holds a value that is not finite, is ignored.
            try:
                if text.startswith("stats"):
                    stats = parse_stats(text)
                    if _valid_stats(stats):
                        self.stats = stats
                elif text.startswith("ok"):
                    pairs = parse_pairs(text.removeprefix("ok"))
                    link = pairs.pop("link")
                    if _finite(pairs):
                        self.settings[link] = pairs
                        self.error = None
                elif text.startswith("error: "):
                    self.error = text.removeprefix("error: ")
                    self.refusals += 1
            except (ValueError, KeyError, IndexError):
                continue
        quiet_since = self.last_heard_s if self.last_heard_s is not None else -SILENT_S
        if now_s - quiet_since > SILENT_S and (self.last_asked_s is None
                                               or now_s - self.last_asked_s > SILENT_S):
            self.send()

    def connected(self, now_s=None):
        now_s = time.monotonic() if now_s is None else now_s
        return self.last_heard_s is not None and now_s - self.last_heard_s <= SILENT_S


def _plural(fmt):
    return lambda v: fmt.format(v, "" if v == 1 else "s")


class EwDeck(QGroupBox):
    """Loss, burst length, rate limit, delay, jitter, a timed blackout and GPS spoofing (a drift
    or a jump of the telemetry position, along a bearing), for all links."""

    def __init__(self, proxy):
        super().__init__("EW deck")
        self.proxy = proxy
        self.synced = False
        self.refusals_seen = 0
        grid = QGridLayout(self)
        self.sliders = {}
        self.values = {}
        rows = [("loss_pct", "Packet loss", 0, max_loss_pct(1), "{} %",
                 "Share of packets the jammer drops. The proxy allows at most L / (L + 1) for a "
                 "mean burst of L packets, so the slider stops there."),
                ("burst", "Mean burst", 1, 20, _plural("{} packet{}"),
                 "How many packets are lost in a row on average (Gilbert-Elliott model)."),
                ("kbps", "Rate limit", 0, len(RATE_STEPS_KBPS) - 1, None,
                 "The most each link carries; packets beyond it wait in the proxy's queue or "
                 "are dropped."),
                ("delay_ms", "Delay", 0, 300, "{} ms", "Delay added to every packet."),
                ("jitter_ms", "Jitter", 0, 200, "{} ms",
                 "A random extra delay of up to this much per packet; it can reorder them."),
                ("spoof_drift_mps", "GPS drift", 0, 10, "{} m/s",
                 "Moves the GPS position in the telemetry away at this speed, along the "
                 "bearing."),
                ("spoof_bearing_deg", "Spoof bearing", 0, 359, "{} deg",
                 "The direction of the GPS drift and jump (0 north, 90 east).")]
        # Two columns, to keep the deck short: the link on the left, the GPS on the right.
        places = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (0, 4), (1, 4)]
        for (key, label, lo, hi, fmt, tip), (row, col) in zip(rows, places):
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(lo, hi)
            slider.setToolTip(tip)
            value = QLabel()
            value.setMinimumWidth(56)
            name = QLabel(label)
            name.setToolTip(tip)
            grid.addWidget(name, row, col)
            grid.addWidget(slider, row, col + 1)
            grid.addWidget(value, row, col + 2)
            self.sliders[key] = slider
            self.values[key] = (value, fmt)
            slider.valueChanged.connect(lambda v, key=key: self._changed(key))
        self.sliders["burst"].setValue(1)
        for key in self.sliders:
            self._show(key)

        grid.setColumnMinimumWidth(3, 16)
        self.blackout_s = QDoubleSpinBox()
        self.blackout_s.setRange(0.5, 30.0)
        self.blackout_s.setValue(5.0)
        self.blackout_s.setSuffix(" s")
        self.blackout_button = QPushButton("Blackout")
        self.blackout_button.setToolTip("Drops every packet on all links for this long.")
        self.blackout_button.clicked.connect(self.blackout)
        grid.addWidget(self.blackout_button, 5, 0)
        grid.addWidget(self.blackout_s, 5, 1)
        self.jump_m = QDoubleSpinBox()
        self.jump_m.setRange(5.0, 500.0)
        self.jump_m.setValue(50.0)
        self.jump_m.setSuffix(" m")
        self.jump_button = QPushButton("GPS jump")
        self.jump_button.setToolTip("Moves the GPS position in the telemetry this far along the "
                                    "bearing at once.")
        self.jump_button.clicked.connect(self.jump)
        grid.addWidget(self.jump_button, 2, 4)
        grid.addWidget(self.jump_m, 2, 5)
        self.clear_button = QPushButton("Clear all")
        self.clear_button.setToolTip("Ends every impairment and the GPS spoofing.")
        self.clear_button.clicked.connect(self.clear)
        grid.addWidget(self.clear_button, 4, 4, 1, 3)
        self.status = QLabel()
        self.status.setFont(QFont("Consolas", 9))
        self.status.setToolTip("What each link let through in the last second, and how many "
                               "packets it dropped since the proxy started: by loss, by a full "
                               "queue, by a blackout.")
        for button in (self.blackout_button, self.jump_button):
            # Otherwise the style's wide minimum sets the deck's width.
            button.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Fixed)
            button.setMinimumWidth(80)
        grid.addWidget(self.status, 6, 0, 1, 7)
        self.error = QLabel()
        self.error.setWordWrap(True)
        self.error.setStyleSheet(f"color: {ERROR_COLOR}")
        grid.addWidget(self.error, 7, 0, 1, 7)

    def setting(self, key):
        """The slider's value in the proxy's units."""
        value = self.sliders[key].value()
        return RATE_STEPS_KBPS[value] if key == "kbps" else value

    def _show(self, key):
        label, fmt = self.values[key]
        value = self.setting(key)
        if key == "kbps":
            label.setText("off" if value == 0 else f"{value} kbps")
        else:
            label.setText(fmt(value) if callable(fmt) else fmt.format(value))

    def _set(self, key, value):
        """Moves a slider without sending its value."""
        slider = self.sliders[key]
        slider.blockSignals(True)
        slider.setValue(round(min(max(value, slider.minimum()), slider.maximum())))
        slider.blockSignals(False)
        self._show(key)

    def _limit_loss(self):
        """Lowers the loss slider's end to what the burst allows (moving the loss down with it)."""
        loss = self.sliders["loss_pct"]
        loss.blockSignals(True)
        loss.setMaximum(max_loss_pct(self.setting("burst")))
        loss.blockSignals(False)
        self._show("loss_pct")

    def _changed(self, key):
        self._show(key)
        if key == "burst":
            # Both in one command: the proxy refuses a burst the current loss does not fit.
            self._limit_loss()
            self.proxy.send(loss_pct=self.setting("loss_pct"), burst=self.setting("burst"))
            return
        self.proxy.send(**{key: self.setting(key)})

    def blackout(self):
        self.proxy.send(blackout_ms=round(self.blackout_s.value() * 1000))

    def jump(self):
        """Moves the spoofed position this far along the bearing at once (from no offset)."""
        self.proxy.send(spoof_offset_m=self.jump_m.value())

    def clear(self):
        for key in self.sliders:
            self._set(key, 1 if key == "burst" else 0)
        self._limit_loss()
        self.proxy.send(loss_pct=0, burst=1, kbps=0, delay_ms=0, jitter_ms=0, blackout_ms=0,
                        spoof_drift_mps=0, spoof_bearing_deg=0, spoof_offset_m=0)

    def _sync(self, settings):
        """Moves the sliders to the proxy's settings, without sending them back."""
        if "burst" in settings:
            self._set("burst", settings["burst"])
            self._limit_loss()
        for key in self.sliders:
            if key not in settings or key == "burst":
                continue
            value = settings[key]
            if key == "kbps":
                value = min(range(len(RATE_STEPS_KBPS)),
                            key=lambda i: abs(RATE_STEPS_KBPS[i] - value))
            self._set(key, value)

    def update_status(self, now_s=None):
        """Reads the proxy's replies and stats and shows them; called a few times a second."""
        self.proxy.poll(now_s)
        if not self.proxy.connected(now_s):
            self.synced = False
            self.status.setText("Proxy not answering on port %d" % self.proxy.addr[1])
            self.error.setText("")
            return
        # A refused command leaves the proxy as it was, so the sliders go back to its settings.
        if self.proxy.refusals != self.refusals_seen:
            self.refusals_seen = self.proxy.refusals
            self.synced = False
        if not self.synced and config.FOVEA_LINK in self.proxy.settings:
            self._sync(self.proxy.settings[config.FOVEA_LINK])
            self.synced = True
        self.status.setText(self._stats_text())
        self.error.setText(f"Refused: {self.proxy.error}" if self.proxy.error else "")

    def _stats_text(self):
        stats = self.proxy.stats["links"] if self.proxy.stats else {}
        lines = [f"{'link':9}{'kbps out':>10}{'loss':>8}{'queue':>8}{'blackout':>9}"]
        events = []
        for link, name in zip(LINKS, ("fovea", "baseline", "reports")):
            s = stats.get(link)
            if s is None:
                continue
            lines.append(f"{name:9}{s['out_kbps']:>10,.1f}{s['drop_loss']:>8,.0f}"
                         f"{s['drop_queue']:>8,.0f}{s['drop_blackout']:>9,.0f}")
            if s["blackout_ms"] and not events:
                events.append(f"BLACKOUT {s['blackout_ms'] / 1000:.1f} s")
            # Only the foveated link carries telemetry, so only its offset matters.
            offset = s.get("spoof_offset_m", 0) if link == config.FOVEA_LINK else 0
            if offset:
                events.append(f"SPOOFING GPS {offset:.0f} m")
        if len(lines) == 1:
            return "Proxy connected"
        return "\n".join(lines + ["   ".join(events)] if events else lines)
