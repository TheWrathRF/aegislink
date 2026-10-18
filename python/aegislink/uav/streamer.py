"""Sends the UAV's frames to the GCS: the foveated stream, the full-frame baseline, and the
budget that follows the GCS's receiver reports."""

import socket

from aegislink._core import MsgType, Packetizer
from aegislink.common import messages
from aegislink.common.net import LinkReceiver
from aegislink.uav import encoder
from aegislink.uav.scheduler import AimdBudget, FrameData, Scheduler


def frame_data(r, timestamp_us):
    """What a FrameResult offers the scheduler."""
    tel = r.telemetry
    telemetry = messages.encode_telemetry(messages.TelemetryMsg(
        r.index, tel.lat, tel.lon, tel.alt_m, tel.speed_mps, tel.heading_deg, tel.pitch_deg,
        tel.roll_deg))
    tracks = messages.encode_tracks(
        r.index, [messages.TrackBox(t.id, tuple(t.box), t.misses == 0) for t in r.tracks])
    alt, heading = r.pose.alt, r.pose.heading_deg

    def patch(track):
        encoded = encoder.encode_patch(r.frame, track.box)
        if encoded is None:
            return None
        box, image = encoded
        return messages.encode_patch(messages.PatchMsg(r.index, alt, heading, track.id, box, image))

    def keyframe():
        image = encoder.encode_keyframe(r.frame)
        return messages.encode_keyframe(messages.KeyframeMsg(r.index, alt, heading, image))

    # Only tracks matched in this frame: a coasting box may no longer hold the vehicle.
    patches = {t.id: (lambda t=t: patch(t)) for t in r.tracks if t.misses == 0}
    return FrameData(r.index, timestamp_us, telemetry, tracks, patches, keyframe)


class BaselineSender:
    """The plain stream to compare with: every frame as one JPEG, sent whole on its own link."""

    def __init__(self):
        self.packetizer = Packetizer()

    def datagrams(self, r, timestamp_us):
        body = messages.encode_full_frame(
            messages.FullFrameMsg(r.index, encoder.encode_full_frame(r.frame)))
        return self.packetizer.packetize(MsgType.FULL_FRAME, timestamp_us, body)


class Streamer:
    def __init__(self, fovea_port, baseline_port, report_port, baseline=True):
        self.fovea_addr = ("127.0.0.1", fovea_port)
        self.baseline_addr = ("127.0.0.1", baseline_port)
        # Sending and receiving use separate sockets: on Windows a send to a port nobody listens
        # on makes the next receive on that socket fail.
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tx.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 * 1024 * 1024)
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(("127.0.0.1", report_port))
        self.rx.setblocking(False)
        self.scheduler = Scheduler()
        self.budget = AimdBudget()
        self.baseline = BaselineSender() if baseline else None
        self.reports = LinkReceiver()
        self.sent_bytes = {"fovea": 0, "baseline": 0}

    def close(self):
        self.tx.close()
        self.rx.close()

    def send(self, r, now_s, timestamp_us):
        """Sends one frame. `now_s` paces the budget; `timestamp_us` is the capture time that
        goes into the headers (the GCS measures the link's delay with it)."""
        self._read_reports(now_s)
        for datagram in self.scheduler.tick(now_s, self.budget.bps, frame_data(r, timestamp_us)):
            self.tx.sendto(datagram, self.fovea_addr)
            self.sent_bytes["fovea"] += len(datagram)
        if self.baseline:
            for datagram in self.baseline.datagrams(r, timestamp_us):
                self.tx.sendto(datagram, self.baseline_addr)
                self.sent_bytes["baseline"] += len(datagram)

    def _read_reports(self, now_s):
        while True:
            try:
                datagram = self.rx.recv(65535)
            except BlockingIOError:
                return
            message = self.reports.push(datagram, int(now_s * 1e6))
            if message is None or message.type != MsgType.RECEIVER_REPORT:
                continue
            try:
                report = messages.decode_report(message.data)
            except ValueError:
                continue
            self.budget.on_report(report, now_s, self.scheduler.limited)
            self.scheduler.limited = False
