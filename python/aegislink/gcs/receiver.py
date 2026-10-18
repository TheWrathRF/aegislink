"""The GCS end of the links: receive both streams, keep the newest of everything, and send the
receiver reports that set the UAV's budget."""

import select
import socket
import time
from collections import Counter
from dataclasses import dataclass

import numpy as np

from aegislink._core import MsgType, Packetizer
from aegislink.common import config, messages
from aegislink.common.net import LinkReceiver, RateMeter
from aegislink.uav.encoder import decode_image

REPORT_INTERVAL_S = 0.25
# A keyframe is paced out at the budget rate: about 3 s for 5 KB at 50 kbps.
FOVEA_TIMEOUT_US = 4_000_000
# Sent the moment they are captured, so their delay is the link's delay.
CONTROL_TYPES = {MsgType.TELEMETRY, MsgType.TRACKS}


@dataclass
class Patch:
    msg: messages.PatchMsg
    image: np.ndarray       # BGR, the size of msg.box


@dataclass
class Keyframe:
    msg: messages.KeyframeMsg
    image: np.ndarray       # gray, downscaled


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
        self.tracks_frame = -1
        self.tracks = []                # newest list of TrackBox
        self.keyframe = None            # newest Keyframe
        self.patches = {}               # track id -> newest Patch
        self.full_frame = None          # newest FullFrameMsg (decoded only when shown)
        self.bad_messages = 0           # complete messages that did not decode
        self.first_fovea_s = None
        self.link_rate = {name: RateMeter() for name in self.sockets}       # bytes per second
        self.message_rate = {t: RateMeter() for t in MsgType}               # messages per second
        self.measurement = Measurement()

    def close(self):
        for s in self.sockets.values():
            s.close()
        self.report_tx.close()

    def poll(self, timeout_s):
        """Waits up to `timeout_s` for datagrams and handles all that are waiting."""
        readable, _, _ = select.select(list(self.sockets.values()), [], [], timeout_s)
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
        if name == "fovea" and self.first_fovea_s is None:
            self.first_fovea_s = now_s
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
        if not self._apply(message.type, body):
            self.bad_messages += 1
            return
        self.message_rate[message.type].add(now_s)
        if self.measurement.active:
            self.measurement.messages[message.type] += 1
            if message.type in self.measurement.frames:
                frame = body.frame if message.type == MsgType.TELEMETRY else body[0]
                self.measurement.frames[message.type].add(frame)

    def _apply(self, msg_type, body):
        """Keeps the body if it is newer than what is held; False if it is unusable."""
        if msg_type == MsgType.TELEMETRY:
            if self.telemetry is None or body.frame >= self.telemetry.frame:
                self.telemetry = body
        elif msg_type == MsgType.TRACKS:
            frame, tracks = body
            if frame >= self.tracks_frame:
                self.tracks_frame, self.tracks = frame, tracks
                ids = {t.id for t in tracks}
                self.patches = {tid: p for tid, p in self.patches.items() if tid in ids}
        elif msg_type == MsgType.ROI_PATCH:
            x1, y1, x2, y2 = body.box
            if not (0 <= x1 < x2 <= config.FRAME_WIDTH and 0 <= y1 < y2 <= config.FRAME_HEIGHT):
                return False
            image = decode_image(body.image)
            if image is None or image.shape[:2] != (y2 - y1, x2 - x1):
                return False
            old = self.patches.get(body.track_id)
            if old is None or body.frame >= old.msg.frame:
                self.patches[body.track_id] = Patch(body, image)
        elif msg_type == MsgType.BG_KEYFRAME:
            image = decode_image(body.image, gray=True)
            if image is None:
                return False
            if self.keyframe is None or body.frame >= self.keyframe.msg.frame:
                self.keyframe = Keyframe(body, image)
        elif msg_type == MsgType.FULL_FRAME:
            if self.full_frame is None or body.frame >= self.full_frame.frame:
                self.full_frame = body
        return True

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
