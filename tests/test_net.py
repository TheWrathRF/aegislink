"""LinkReceiver: parsing, reassembly and the counters behind the receiver reports."""

from aegislink._core import MsgType, Packetizer
from aegislink.common.net import LinkReceiver, RateMeter


def test_counts_packets_bytes_and_highest_seq():
    rx = LinkReceiver()
    packets = Packetizer().packetize(MsgType.TELEMETRY, 1000, b"a" * 3000)  # 3 fragments
    message = None
    for datagram in reversed(packets):
        message = rx.push(datagram, now_us=1500) or message
    assert message.data == b"a" * 3000 and message.type == MsgType.TELEMETRY
    report = rx.report(now_us=2000)
    assert (report.received, report.received_bytes) == (3, sum(map(len, packets)))
    assert report.highest_seq == 2
    assert report.time_us == 2000


def test_bad_datagrams_are_counted_and_not_reported_as_received():
    rx = LinkReceiver()
    good = Packetizer().packetize(MsgType.TRACKS, 0, b"xyz")[0]
    corrupt = bytearray(good)
    corrupt[-1] ^= 1
    assert rx.push(bytes(corrupt), 0) is None
    assert rx.push(b"short", 0) is None
    assert rx.bad == 2
    assert rx.report(0).received == 0


def test_min_delay_is_per_report_interval():
    rx = LinkReceiver()
    tx = Packetizer()
    for sent_us, arrived_us in [(0, 30_000), (10_000, 25_000), (20_000, 60_000)]:
        rx.push(tx.packetize(MsgType.TELEMETRY, sent_us, b"t")[0], arrived_us)
    assert rx.report(70_000).min_delay_us == 15_000
    # Nothing arrived since: no delay sample, the counters stay.
    quiet = rx.report(80_000)
    assert quiet.min_delay_us == 0 and quiet.received == 3
    rx.push(tx.packetize(MsgType.TELEMETRY, 90_000, b"t")[0], 140_000)
    assert rx.report(150_000).min_delay_us == 50_000


def test_delay_may_be_negative_with_another_clock():
    rx = LinkReceiver()
    rx.push(Packetizer().packetize(MsgType.TELEMETRY, 10_000_000, b"t")[0], 5_000)
    assert rx.report(6_000).min_delay_us == 5_000 - 10_000_000


def test_lost_packets_show_as_gaps_in_seq():
    rx = LinkReceiver()
    tx = Packetizer()
    packets = [tx.packetize(MsgType.TELEMETRY, 0, b"t")[0] for _ in range(10)]
    for i, datagram in enumerate(packets):
        if i not in (3, 4):
            rx.push(datagram, 0)
    report = rx.report(0)
    assert report.highest_seq == 9 and report.received == 8


def test_long_timeout_keeps_a_slow_message():
    rx = LinkReceiver(timeout_us=4_000_000)
    first, second = Packetizer().packetize(MsgType.BG_KEYFRAME, 0, b"k" * 2000)
    assert rx.push(first, 0) is None
    assert rx.push(second, 3_000_000).data == b"k" * 2000


def test_rate_meter_over_a_sliding_window():
    meter = RateMeter(window_s=1.0)
    for i in range(30):
        meter.add(i / 30, 100)
    assert meter.rate(29 / 30) == 3000
    assert meter.count(29 / 30) == 30
    # Two seconds later the window is empty.
    assert meter.rate(3.0) == 0
