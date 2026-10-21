"""The EW deck: sliders that set the proxy's impairments while the link runs.

The jammer hits the radio channel, so every change goes to all three links (both video links and
the uplink of receiver reports). The deck never waits for the proxy: commands go out and replies
and stats are read whenever they have arrived, so a missing proxy cannot freeze the window.
"""

import math
import socket
import time

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (QDoubleSpinBox, QGridLayout, QGroupBox, QLabel, QPushButton,
                             QSlider)

from aegislink.common import config
from aegislink.common.proxy_control import parse_pairs, parse_stats

LINKS = (config.FOVEA_LINK, config.BASELINE_LINK, config.REPORTS_LINK)
# The proxy sends its stats to whoever sent the last command; asking again after this long
# without a word brings them back (after a restart of the proxy, or a script that took them).
SILENT_S = 2.0
RATE_STEPS_KBPS = [0, 15, 25, 50, 100, 250, 500, 1000, 2000, 5000, 20000]   # 0 = no limit
STATS_KEYS = ("out_kbps", "drop_loss", "drop_queue", "drop_blackout", "blackout_ms")


def _finite(pairs):
    return all(math.isfinite(v) for v in pairs.values())


def _valid_stats(stats):
    return all(all(k in s for k in STATS_KEYS) and _finite(s) for s in stats["links"].values())


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
            except (ValueError, KeyError, IndexError):
                continue
        quiet_since = self.last_heard_s if self.last_heard_s is not None else -SILENT_S
        if now_s - quiet_since > SILENT_S and (self.last_asked_s is None
                                               or now_s - self.last_asked_s > SILENT_S):
            self.send()

    def connected(self, now_s=None):
        now_s = time.monotonic() if now_s is None else now_s
        return self.last_heard_s is not None and now_s - self.last_heard_s <= SILENT_S


class EwDeck(QGroupBox):
    """Loss, burst length, rate limit, delay, jitter, a timed blackout and GPS spoofing (a drift
    or a jump of the telemetry position, along a bearing), for all links."""

    def __init__(self, proxy):
        super().__init__("EW deck")
        self.proxy = proxy
        self.synced = False
        grid = QGridLayout(self)
        self.sliders = {}
        self.values = {}
        rows = [("loss_pct", "Packet loss", 0, 60, "{} %"),
                ("burst", "Mean burst", 1, 20, "{} packets"),
                ("kbps", "Rate limit", 0, len(RATE_STEPS_KBPS) - 1, None),
                ("delay_ms", "Delay", 0, 300, "{} ms"),
                ("jitter_ms", "Jitter", 0, 200, "{} ms"),
                ("spoof_drift_mps", "GPS drift", 0, 10, "{} m/s"),
                ("spoof_bearing_deg", "Spoof bearing", 0, 359, "{} deg")]
        for row, (key, label, lo, hi, fmt) in enumerate(rows):
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(lo, hi)
            value = QLabel()
            value.setMinimumWidth(80)
            grid.addWidget(QLabel(label), row, 0)
            grid.addWidget(slider, row, 1)
            grid.addWidget(value, row, 2)
            self.sliders[key] = slider
            self.values[key] = (value, fmt)
            slider.valueChanged.connect(lambda v, key=key: self._changed(key))
        self.sliders["burst"].setValue(1)
        for key in self.sliders:
            self._show(key)

        row = len(rows)
        self.blackout_s = QDoubleSpinBox()
        self.blackout_s.setRange(0.5, 30.0)
        self.blackout_s.setValue(5.0)
        self.blackout_s.setSuffix(" s")
        self.blackout_button = QPushButton("Blackout")
        self.blackout_button.clicked.connect(self.blackout)
        grid.addWidget(self.blackout_button, row, 0)
        grid.addWidget(self.blackout_s, row, 1)
        clear = QPushButton("Clear")
        clear.clicked.connect(self.clear)
        grid.addWidget(clear, row, 2)
        self.jump_m = QDoubleSpinBox()
        self.jump_m.setRange(5.0, 500.0)
        self.jump_m.setValue(50.0)
        self.jump_m.setSuffix(" m")
        self.jump_button = QPushButton("GPS jump")
        self.jump_button.clicked.connect(self.jump)
        grid.addWidget(self.jump_button, row + 1, 0)
        grid.addWidget(self.jump_m, row + 1, 1)
        self.status = QLabel()
        self.status.setWordWrap(True)
        grid.addWidget(self.status, row + 2, 0, 1, 3)

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
            label.setText(fmt.format(value))

    def _changed(self, key):
        self._show(key)
        self.proxy.send(**{key: self.setting(key)})

    def blackout(self):
        self.proxy.send(blackout_ms=round(self.blackout_s.value() * 1000))

    def jump(self):
        """Moves the spoofed position this far along the bearing at once (from no offset)."""
        self.proxy.send(spoof_offset_m=self.jump_m.value())

    def clear(self):
        for key, slider in self.sliders.items():
            slider.blockSignals(True)
            slider.setValue(1 if key == "burst" else 0)
            slider.blockSignals(False)
            self._show(key)
        self.proxy.send(loss_pct=0, burst=1, kbps=0, delay_ms=0, jitter_ms=0, blackout_ms=0,
                        spoof_drift_mps=0, spoof_bearing_deg=0, spoof_offset_m=0)

    def _sync(self, settings):
        """Moves the sliders to the proxy's settings, without sending them back."""
        for key, slider in self.sliders.items():
            if key not in settings:
                continue
            value = settings[key]
            if key == "kbps":
                value = min(range(len(RATE_STEPS_KBPS)),
                            key=lambda i: abs(RATE_STEPS_KBPS[i] - value))
            slider.blockSignals(True)
            slider.setValue(round(min(max(value, slider.minimum()), slider.maximum())))
            slider.blockSignals(False)
            self._show(key)

    def update_status(self, now_s=None):
        """Reads the proxy's replies and stats and shows them; called a few times a second."""
        self.proxy.poll(now_s)
        if not self.proxy.connected(now_s):
            self.synced = False
            self.status.setText("Proxy not answering on port %d" % self.proxy.addr[1])
            return
        if not self.synced and config.FOVEA_LINK in self.proxy.settings:
            self._sync(self.proxy.settings[config.FOVEA_LINK])
            self.synced = True
        lines = []
        stats = self.proxy.stats["links"] if self.proxy.stats else {}
        for link, name in zip(LINKS, ("fovea", "baseline", "reports")):
            s = stats.get(link)
            if s is None:
                continue
            blackout = f"  BLACKOUT {s['blackout_ms'] / 1000:.1f} s" if s["blackout_ms"] else ""
            # Only the foveated link carries telemetry, so only its offset matters.
            offset = s.get("spoof_offset_m", 0) if link == config.FOVEA_LINK else 0
            spoofing = f"  SPOOFING GPS {offset:.0f} m" if offset else ""
            lines.append(f"{link} {name}: {s['out_kbps']:.1f} kbps out, dropped "
                         f"{s['drop_loss']:.0f} loss / {s['drop_queue']:.0f} queue / "
                         f"{s['drop_blackout']:.0f} blackout{blackout}{spoofing}")
        if self.proxy.error:
            lines.append(f"proxy: {self.proxy.error}")
        self.status.setText("\n".join(lines) or "Proxy connected")
