"""Receiving one link: parse, reassemble, and count what arrives for the receiver reports."""

from collections import deque

from aegislink._core import ParseError, Reassembler, parse_packet
from aegislink.common.messages import ReceiverReport


class LinkReceiver:
    """Turns one link's datagrams into messages and keeps the counters a receiver report needs.

    Times are in microseconds. The one-way delay is the arrival time minus the header timestamp;
    on two machines the clocks differ by an unknown offset, so only changes of the delay mean
    something (the UAV compares each report's delay with the lowest it has seen). Only packets
    of `delay_types` count for the delay: an image the sender paces out over seconds keeps its
    capture time, so its late fragments would look like queueing.
    """

    def __init__(self, timeout_us=200_000, delay_types=None):
        self.reassembler = Reassembler(timeout_us=timeout_us)
        self.delay_types = delay_types
        self.received = 0
        self.received_bytes = 0
        self.bad = 0
        self.highest_seq = None
        self._min_delay_us = None

    def push(self, datagram, now_us):
        """Returns the completed Message, or None."""
        packet = parse_packet(datagram)
        if packet.error != ParseError.OK:
            self.bad += 1
            return None
        header = packet.header
        self.received += 1
        self.received_bytes += len(datagram)
        if self.highest_seq is None or header.seq > self.highest_seq:
            self.highest_seq = header.seq
        if self.delay_types is None or header.type in self.delay_types:
            delay = now_us - header.timestamp_us
            if self._min_delay_us is None or delay < self._min_delay_us:
                self._min_delay_us = delay
        return self.reassembler.push(header, packet.payload, now_us)

    def report(self, now_us):
        """The receiver report for now; starts a new delay interval."""
        report = ReceiverReport(now_us, self.highest_seq or 0, self.received, self.received_bytes,
                                self._min_delay_us or 0)
        self._min_delay_us = None
        return report


class RateMeter:
    """Sums amounts (bytes, messages) over the last `window_s` seconds."""

    def __init__(self, window_s=1.0):
        self.window_s = window_s
        self.events = deque()   # (time, amount)

    def add(self, now, amount=1):
        self.events.append((now, amount))

    def _trim(self, now):
        while self.events and self.events[0][0] <= now - self.window_s:
            self.events.popleft()

    def rate(self, now):
        """Amount per second over the window."""
        self._trim(now)
        return sum(a for _, a in self.events) / self.window_s

    def count(self, now):
        self._trim(now)
        return len(self.events)
