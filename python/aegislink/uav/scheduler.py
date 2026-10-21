"""What the UAV sends on the foveated link, and how fast.

The `Scheduler` runs once per frame and spends a byte budget in priority order: telemetry, then
the track boxes, then one image message at a time (an ROI patch or the background keyframe).
It paces datagrams with a token bucket, so even a keyframe of several datagrams leaves at the
budget rate instead of in one burst that a slow link's queue would drop.

`AimdBudget` sets that budget from the GCS's receiver reports, the way TCP sets its window:
while the budget is what holds data back it grows, by a factor per report until the first sign
of congestion (TCP's slow start) and by a fixed step after that, and it is cut when the link's
queue is filling. A filling queue shows as a rising one-way delay, which separates congestion
from the random loss of jamming (cutting on every loss would starve the link under jamming).
"""

import math
from collections import Counter, deque
from dataclasses import dataclass
from typing import Callable

from aegislink._core import HEADER_SIZE, MAX_PAYLOAD, Header, MsgType, encode_packet
from aegislink.common.messages import NO_DELAY, ReceiverReport


@dataclass
class FrameData:
    """What one frame offers to send. Images are encoded only if they are chosen."""
    frame: int
    timestamp_us: int
    telemetry: bytes                            # message bodies
    tracks: bytes
    patches: dict[int, Callable[[], bytes | None]]  # track id -> patch body (None: not in view)
    keyframe: Callable[[], bytes]


class _Outgoing:
    """A message being sent, one datagram at a time."""

    def __init__(self, msg_type, timestamp_us, body, chunk=MAX_PAYLOAD):
        self.msg_type = msg_type
        self.timestamp_us = timestamp_us
        self.msg_id = None
        self.chunks = [body[i:i + chunk] for i in range(0, len(body), chunk)] or [b""]
        self.next = 0

    @property
    def done(self):
        return self.next == len(self.chunks)

    @property
    def size(self):
        """Wire size of all remaining datagrams."""
        return sum(HEADER_SIZE + len(c) for c in self.chunks[self.next:])

    @property
    def next_size(self):
        return HEADER_SIZE + len(self.chunks[self.next])


class Scheduler:
    PATCH_INTERVAL_S = 0.1      # each track's patch at up to 10 Hz
    # The background at up to 2 Hz: the GCS checks the GPS against the optical flow between
    # keyframes, and needs 3 of them in a row to raise an alarm.
    KEYFRAME_INTERVAL_S = 0.5
    KEYFRAME_SHARE = 0.15       # keyframes take at most this share of the budget (patches first)
    BURST_S = 0.1               # tokens saved up beyond one image datagram
    # An image datagram takes at most this much of the budget (but carries at least MIN_CHUNK
    # bytes). On a slow link a full 1230-byte datagram holds the link for 200 ms (at 50 kbps),
    # and the telemetry queued behind it would look like congestion.
    DATAGRAM_S = 0.03
    MIN_CHUNK = 200

    def __init__(self):
        self.tokens = 0.0
        self.last_s = None
        self.seq = 0
        self.msg_id = 0
        self.image = None           # the _Outgoing image message, until its last datagram is sent
        self.patch_sent_s = {}      # track id -> when its last patch was chosen
        self.keyframe_sent_s = -math.inf
        self.keyframe_bytes = 0     # size of the last keyframe, to space out the next one
        self.keyframe_interval_s = self.KEYFRAME_INTERVAL_S
        self.limited = False        # the budget held data back; the caller clears it
        self.sent = Counter()       # MsgType -> messages started

    def tick(self, now_s, budget_bps, frame):
        """Returns the datagrams to send now."""
        rate = budget_bps / 8
        chunk = max(self.MIN_CHUNK, min(MAX_PAYLOAD, round(rate * self.DATAGRAM_S) - HEADER_SIZE))
        if rate > 0:
            self.keyframe_interval_s = max(self.KEYFRAME_INTERVAL_S,
                                           self.keyframe_bytes / (self.KEYFRAME_SHARE * rate))
        # Telemetry and tracks go together or not at all, so the GCS always gets both of a frame.
        control = [_Outgoing(MsgType.TELEMETRY, frame.timestamp_us, frame.telemetry),
                   _Outgoing(MsgType.TRACKS, frame.timestamp_us, frame.tracks)]
        control_size = sum(m.size for m in control)
        # Saved-up tokens are capped, so that a quiet spell is not followed by a burst that a slow
        # link would have to queue; the cap still lets the largest single item through.
        cap = rate * self.BURST_S + max(HEADER_SIZE + chunk, control_size)
        elapsed = 1 / 30 if self.last_s is None else now_s - self.last_s
        self.tokens = min(cap, self.tokens + rate * elapsed)
        self.last_s = now_s

        out = []
        if control_size > self.tokens:
            # Skipped (the next frame's are newer), and no image may take the tokens they wait for.
            self.limited = True
            return out
        for message in control:
            self._start(message)
            while not message.done:
                out.append(self._next_datagram(message))

        if self.image is None:
            self.image = self._choose_image(now_s, frame, chunk)
        # An image cut for a higher budget may have datagrams larger than the tokens can grow to
        # now: such a datagram leaves when the tokens cover one of today's size (and runs a debt).
        datagram_size = HEADER_SIZE + chunk
        while self.image is not None and min(self.image.next_size, datagram_size) <= self.tokens:
            out.append(self._next_datagram(self.image))
            if self.image.done:
                self.image = self._choose_image(now_s, frame, chunk)
        if self.image is not None:
            self.limited = True
        return out

    def _choose_image(self, now_s, frame, chunk):
        """The image that is most overdue relative to its interval, or None if none is due."""
        self.patch_sent_s = {tid: t for tid, t in self.patch_sent_s.items() if tid in frame.patches}
        candidates = [((now_s - self.keyframe_sent_s) / self.keyframe_interval_s, -1)]
        for tid in frame.patches:
            age = now_s - self.patch_sent_s.get(tid, -math.inf)
            candidates.append((age / self.PATCH_INTERVAL_S, tid))
        for overdue, tid in sorted(candidates, reverse=True):
            # Times are sums of frame steps, so an interval of 3 frames may come out a hair short.
            if overdue < 1 - 1e-6:
                break
            if tid == -1:
                self.keyframe_sent_s = now_s
                body = frame.keyframe()
                keyframe = _Outgoing(MsgType.BG_KEYFRAME, frame.timestamp_us, body, chunk)
                self.keyframe_bytes = keyframe.size
                return self._start(keyframe)
            body = frame.patches[tid]()
            if body is not None:
                self.patch_sent_s[tid] = now_s
                return self._start(_Outgoing(MsgType.ROI_PATCH, frame.timestamp_us, body, chunk))
        return None

    def _start(self, message):
        # msg_id is taken only by messages that are sent, like seq.
        message.msg_id = self.msg_id
        self.msg_id += 1
        self.sent[message.msg_type] += 1
        return message

    def _next_datagram(self, message):
        # seq is given when a datagram leaves, so the receiver sees no gap for skipped messages.
        header = Header(message.msg_type, self.seq, message.timestamp_us, message.msg_id,
                        message.next, len(message.chunks))
        datagram = encode_packet(header, message.chunks[message.next])
        message.next += 1
        self.seq += 1
        self.tokens -= len(datagram)
        return datagram


class AimdBudget:
    START_BPS = 64_000
    MIN_BPS = 8_000
    MAX_BPS = 2_000_000
    STEP_BPS = 2_000            # added per report (4 per second) while the budget limits,
    START_GROWTH = 1.25         # or this factor per report until the first congestion
    QUEUE_DELAY_US = 40_000     # congested above this much queueing delay
    BACKOFF = 0.85              # of the received rate, on congestion
    HOLD_OFF_S = 0.5            # one cut, then time for the queue to drain
    BASE_WINDOW_S = 10.0        # how long the lowest delay is remembered

    def __init__(self, start_bps=START_BPS):
        self.bps = float(start_bps)
        self.last = None
        self.delays = deque()           # (time, min delay) of recent reports with packets
        self.last_cut_s = -math.inf
        self.slow_start = True
        self.loss = 0.0                 # over the last report interval
        self.received_bps = 0.0
        self.queue_delay_us = 0

    def on_report(self, report, now_s, limited):
        """`limited`: the scheduler held data back since the last report."""
        last = self.last
        if last is not None and report.time_us <= last.time_us:
            return  # reordered or duplicated on the uplink
        self.last = report
        if last is None:
            # The first delay sample matters most: it was taken before any queue could build.
            last = ReceiverReport(report.time_us, 0, 0, 0, 0)
            dt = 0
        else:
            dt = (report.time_us - last.time_us) / 1e6
        received = report.received - last.received
        expected = report.highest_seq - last.highest_seq
        if dt > 0:
            self.received_bps = (report.received_bytes - last.received_bytes) * 8 / dt
        self.loss = min(1.0, max(0.0, 1 - received / expected)) if expected > 0 else 0.0

        if received > 0 and report.min_delay_us != NO_DELAY:
            # The lowest delay is the empty-queue delay (plus any clock offset), so the rest of
            # a report's lowest delay is time spent queueing. Jitter rarely lifts the lowest
            # delay of a whole interval.
            self.delays.append((now_s, report.min_delay_us))
            while self.delays[0][0] < now_s - self.BASE_WINDOW_S:
                self.delays.popleft()
            self.queue_delay_us = report.min_delay_us - min(d for _, d in self.delays)
            if self.queue_delay_us > self.QUEUE_DELAY_US:
                if now_s - self.last_cut_s >= self.HOLD_OFF_S:
                    # While congested, the received rate is what the link carries.
                    self.bps = max(self.MIN_BPS, min(self.bps, self.BACKOFF * self.received_bps))
                    self.last_cut_s = now_s
                    self.slow_start = False
                return
        if limited:
            grown = self.bps * self.START_GROWTH if self.slow_start else self.bps + self.STEP_BPS
            self.bps = min(self.MAX_BPS, grown)
