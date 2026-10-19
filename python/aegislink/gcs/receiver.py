"""The GCS end of the links: receive both streams, keep the newest of everything, and send the
receiver reports that set the UAV's budget."""

import select
import socket
import threading
import time
from collections import Counter
from dataclasses import dataclass, field

import numpy as np

from aegislink._core import MsgType, Packetizer
from aegislink.common import config, messages
from aegislink.common.net import LinkReceiver, RateMeter
from aegislink.gcs.poses import PoseHistory
from aegislink.uav.encoder import KEYFRAME_SIZE, decode_image, image_size

REPORT_INTERVAL_S = 0.25
# A keyframe is paced out at the budget rate: about 3 s for 5 KB at 50 kbps.
FOVEA_TIMEOUT_US = 4_000_000
# Sent the moment they are captured, so their delay is the link's delay.
CONTROL_TYPES = {MsgType.TELEMETRY, MsgType.TRACKS}
# A frame number this far below the held one means the UAV started again (or the held one was
# forged): it replaces the held one instead of being ignored as old.
RESTART_FRAMES = 10 * config.FPS

USED, OLD, BAD = "used", "old", "bad"


def _newer(frame, held):
    return held is None or frame >= held or held - frame > RESTART_FRAMES


def _restarted(frame, held):
    return held is not None and held - frame > RESTART_FRAMES


@dataclass
class Patch:
    msg: messages.PatchMsg
    image: np.ndarray       # BGR, the size of msg.box


@dataclass
class Keyframe:
    msg: messages.KeyframeMsg
    image: np.ndarray       # gray, downscaled


@dataclass
class View:
    """What the station held at one moment: everything the foveated view is drawn from."""
    telemetry: messages.TelemetryMsg | None
    tracks_frame: int | None
    tracks: list
    keyframe: Keyframe | None
    patches: dict               # track id -> Patch
    poses: PoseHistory = field(default_factory=PoseHistory)


class Measurement:
    """What arrived between start() and stop(), for the summary of a headless run."""

    def __init__(self):
        self.start_s = None
        self.stop_s = None
        self.bytes = Counter()          # link name -> datagram bytes
        self.messages = Counter()       # MsgType -> messages
        self.frames = {MsgType.TELEMETRY: set(), MsgType.TRACKS: set()}

    @property
    def active(self):
        return self.start_s is not None and self.stop_s is None

    def summary(self):
        seconds = self.stop_s - self.start_s
        fovea = self.bytes["fovea"] * 8 / seconds / 1000
        baseline = self.bytes["baseline"] * 8 / seconds / 1000
        result = {"seconds": round(seconds, 2), "fovea_kbps": round(fovea, 1),
                  "baseline_kbps": round(baseline, 1),
                  "fovea_share": round(fovea / baseline, 4) if baseline else None}
        for msg_type, name in ((MsgType.TELEMETRY, "telemetry"), (MsgType.TRACKS, "tracks")):
            frames = self.frames[msg_type]
            span = max(frames) - min(frames) + 1 if frames else 0
            result[f"{name}_hz"] = round(len(frames) / seconds, 2)
            result[f"{name}_delivered"] = round(len(frames) / span, 4) if span else 0.0
        result["patches_per_s"] = round(self.messages[MsgType.ROI_PATCH] / seconds, 2)
        result["keyframes"] = self.messages[MsgType.BG_KEYFRAME]
        result["baseline_fps"] = round(self.messages[MsgType.FULL_FRAME] / seconds, 2)
        return result


class GroundStation:
    def __init__(self, fovea_port, baseline_port, report_port):
        self.sockets = {}
        for name, port in (("fovea", fovea_port), ("baseline", baseline_port)):
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
            s.bind(("127.0.0.1", port))
            s.setblocking(False)
            self.sockets[name] = s
        self.report_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.report_addr = ("127.0.0.1", report_port)
        self.report_packetizer = Packetizer()
        self.next_report_s = None
        self.links = {"fovea": LinkReceiver(FOVEA_TIMEOUT_US, delay_types=CONTROL_TYPES),
                      "baseline": LinkReceiver()}

        self.telemetry = None           # newest TelemetryMsg
        self.poses = PoseHistory()      # positions of the recent frames
        self.tracks_frame = None
        self.tracks = []                # newest list of TrackBox
        self.keyframe = None            # newest Keyframe
        self.patches = {}               # track id -> newest Patch
        self.full_frame = None          # newest FullFrameMsg (decoded only when shown)
        self.bad_messages = 0           # complete messages that did not decode
        self.first_fovea_s = None
        self.last_fovea_s = None
        self.link_rate = {name: RateMeter() for name in self.sockets}       # bytes per second
        self.message_rate = {t: RateMeter() for t in MsgType}               # messages per second
        self.measurement = Measurement()

    def close(self):
        for s in self.sockets.values():
            s.close()
        self.report_tx.close()

    def poll(self, timeout_s):
        """Waits up to `timeout_s` for datagrams and handles all that are waiting."""
        self.read(self.wait(timeout_s))

    def wait(self, timeout_s):
        """Waits up to `timeout_s` for datagrams; returns the sockets that have some."""
        readable, _, _ = select.select(list(self.sockets.values()), [], [], timeout_s)
        return readable

    def read(self, readable):
        for name, s in self.sockets.items():
            if s not in readable:
                continue
            while True:
                try:
                    datagram = s.recv(65535)
                except BlockingIOError:
                    break
                self._receive(name, datagram, time.monotonic())

    def _receive(self, name, datagram, now_s):
        if name == "fovea":
            if self.first_fovea_s is None:
                self.first_fovea_s = now_s
            self.last_fovea_s = now_s
        self.link_rate[name].add(now_s, len(datagram))
        if self.measurement.active:
            self.measurement.bytes[name] += len(datagram)
        message = self.links[name].push(datagram, int(now_s * 1e6))
        if message is None:
            return
        try:
            body = messages.decode(message.type, message.data)
        except ValueError:
            self.bad_messages += 1
            return
        result = self._apply(message.type, body)
        if result == BAD:
            self.bad_messages += 1
        if result != USED:
            return
        self.message_rate[message.type].add(now_s)
        if self.measurement.active:
            self.measurement.messages[message.type] += 1
            if message.type in self.measurement.frames:
                frame = body.frame if message.type == MsgType.TELEMETRY else body[0]
                self.measurement.frames[message.type].add(frame)

    def _apply(self, msg_type, body):
        """Keeps the body if it is newer than what is held: USED, OLD (ignored) or BAD."""
        if msg_type == MsgType.TELEMETRY:
            held = self.telemetry.frame if self.telemetry else None
            if not _newer(body.frame, held):
                return OLD
            if _restarted(body.frame, held):
                self._forget()
            self.telemetry = body
            self.poses.add(body)
        elif msg_type == MsgType.TRACKS:
            frame, tracks = body
            if not _newer(frame, self.tracks_frame):
                return OLD
            if _restarted(frame, self.tracks_frame):
                self._forget()
            self.tracks_frame, self.tracks = frame, tracks
            ids = {t.id for t in tracks}
            self.patches = {tid: p for tid, p in self.patches.items() if tid in ids}
        elif msg_type == MsgType.ROI_PATCH:
            x1, y1, x2, y2 = body.box
            if not (0 <= x1 < x2 <= config.FRAME_WIDTH and 0 <= y1 < y2 <= config.FRAME_HEIGHT):
                return BAD
            image = decode_image(body.image, size=(x2 - x1, y2 - y1))
            if image is None:
                return BAD
            old = self.patches.get(body.track_id)
            # Only for a track in the newest list (sent just before it), so patches stay bounded.
            if (body.track_id not in {t.id for t in self.tracks}
                    or not _newer(body.frame, old.msg.frame if old else None)):
                return OLD
            self.patches[body.track_id] = Patch(body, image)
        elif msg_type == MsgType.BG_KEYFRAME:
            image = decode_image(body.image, gray=True, size=KEYFRAME_SIZE)
            if image is None:
                return BAD
            if not _newer(body.frame, self.keyframe.msg.frame if self.keyframe else None):
                return OLD
            self.keyframe = Keyframe(body, image)
        elif msg_type == MsgType.FULL_FRAME:
            # Decoded only when shown, but its size is checked now.
            if image_size(body.image) != (config.FRAME_WIDTH, config.FRAME_HEIGHT):
                return BAD
            if not _newer(body.frame, self.full_frame.frame if self.full_frame else None):
                return OLD
            self.full_frame = body
        return USED

    def _forget(self):
        """Drops the foveated state of an earlier run of the UAV."""
        self.telemetry = None
        self.poses.clear()
        self.tracks_frame, self.tracks = None, []
        self.keyframe = None
        self.patches = {}

    def view(self):
        """A copy of what the foveated view needs, safe to draw from while receiving goes on."""
        return View(self.telemetry, self.tracks_frame, list(self.tracks), self.keyframe,
                    dict(self.patches), self.poses.copy())

    def stats(self, now_s):
        """The numbers the dashboard shows: rates over the last second, and ages."""
        rate = {t: self.message_rate[t].rate(now_s) for t in MsgType}
        keyframe_age = None
        if self.keyframe is not None and self.telemetry is not None:
            keyframe_age = max(0, self.telemetry.frame - self.keyframe.msg.frame) / config.FPS
        return {
            "fovea_kbps": self.link_rate["fovea"].rate(now_s) * 8 / 1000,
            "baseline_kbps": self.link_rate["baseline"].rate(now_s) * 8 / 1000,
            "telemetry_hz": rate[MsgType.TELEMETRY],
            "tracks_hz": rate[MsgType.TRACKS],
            "patches_per_s": rate[MsgType.ROI_PATCH],
            "baseline_fps": rate[MsgType.FULL_FRAME],
            "keyframe_age_s": keyframe_age,
            "fovea_silent_s": None if self.last_fovea_s is None else now_s - self.last_fovea_s,
            "bad_packets": self.links["fovea"].bad + self.links["baseline"].bad,
            "bad_messages": self.bad_messages,
            "expired": self.links["fovea"].reassembler.stats.expired,
        }

    def send_report_if_due(self, now_s):
        if self.next_report_s is None:
            self.next_report_s = now_s
        if now_s < self.next_report_s:
            return
        self.next_report_s += REPORT_INTERVAL_S
        if self.next_report_s < now_s:  # the loop was held up; do not send a burst
            self.next_report_s = now_s + REPORT_INTERVAL_S
        now_us = int(now_s * 1e6)
        body = messages.encode_report(self.links["fovea"].report(now_us))
        for datagram in self.report_packetizer.packetize(MsgType.RECEIVER_REPORT, now_us, body):
            self.report_tx.sendto(datagram, self.report_addr)


class ReceiverThread:
    """Runs a GroundStation in its own thread, so that drawing never holds up receiving or the
    receiver reports. The UI reads the station only through `snapshot`, under the same lock."""

    def __init__(self, station, poll_s=0.005):
        self.station = station
        self.poll_s = poll_s
        self.lock = threading.Lock()
        self.error = None           # the exception that stopped the thread, if any
        self._stopping = threading.Event()
        self._thread = threading.Thread(target=self._run, name="gcs-receiver", daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stopping.set()
        self._thread.join(timeout=2.0)

    def is_alive(self):
        return self._thread.is_alive()

    def _run(self):
        try:
            while not self._stopping.is_set():
                readable = self.station.wait(self.poll_s)     # without the lock
                with self.lock:
                    self.station.read(readable)
                    self.station.send_report_if_due(time.monotonic())
        except Exception as e:      # shown by the UI instead of dying silently
            self.error = e

    def snapshot(self):
        """(View, newest FullFrameMsg or None, stats) as of now."""
        with self.lock:
            s = self.station
            return s.view(), s.full_frame, s.stats(time.monotonic())
