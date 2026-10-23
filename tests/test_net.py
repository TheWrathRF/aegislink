"""LinkReceiver: parsing, reassembly and the counters behind the receiver reports."""

from aegislink._core import MAX_PAYLOAD, MsgType, Packetizer
from aegislink.common.messages import NO_DELAY
from aegislink.common.net import MAX_FRAGMENTS, LinkReceiver, RateMeter


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
    assert quiet.min_delay_us == NO_DELAY and quiet.received == 3
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


def test_delay_only_from_the_chosen_types():
    rx = LinkReceiver(delay_types={MsgType.TELEMETRY})
    tx = Packetizer()
    rx.push(tx.packetize(MsgType.BG_KEYFRAME, 0, b"k")[0], 3_000_000)   # paced: 3 s late
    rx.push(tx.packetize(MsgType.TELEMETRY, 2_990_000, b"t")[0], 3_000_000)
    assert rx.report(3_000_000).min_delay_us == 10_000


def test_rate_meter_forgets_old_events_without_being_read():
    meter = RateMeter(window_s=1.0)
    for i in range(100_000):
        meter.add(i / 1000, 1)
    assert len(meter.events) <= 1001


def test_huge_timestamps_still_give_a_report():
    from aegislink.common import messages
    rx = LinkReceiver()
    rx.push(Packetizer().packetize(MsgType.TELEMETRY, 2**64 - 1, b"t")[0], 1000)
    messages.encode_report(rx.report(2000))   # packs the delay as i64


def test_an_interval_without_a_delay_sample_says_so():
    rx = LinkReceiver(delay_types={MsgType.TELEMETRY})
    rx.push(Packetizer().packetize(MsgType.ROI_PATCH, 0, b"p")[0], 100_000)
    report = rx.report(200_000)
    assert report.received == 1 and report.min_delay_us == NO_DELAY
    rx.push(Packetizer(5).packetize(MsgType.TELEMETRY, 300_000, b"t")[0], 300_000)
    assert rx.report(400_000).min_delay_us == 0     # a real delay of 0 is a sample


def test_a_forged_fragment_count_is_bad_and_costs_no_more_than_a_normal_packet():
    import time

    from aegislink._core import Header, encode_packet

    def push_all(frag_count, n=2000):
        rx = LinkReceiver(timeout_us=4_000_000)
        packets = [encode_packet(Header(MsgType.ROI_PATCH, i, 0, i, 0, frag_count), b"x" * 10)
                   for i in range(n)]
        start = time.perf_counter()
        for i, p in enumerate(packets):
            rx.push(p, 1000 + i)
        return rx, (time.perf_counter() - start) / n

    # Each would make the reassembler set aside 65,535 fragment slots (about 100 times the cost
    # of a normal packet, which then evicts a real partial message).
    forged, forged_s = push_all(65535)
    assert forged.bad == 2000 and forged.received == 0 and forged.reassembler.pending == 0
    normal, normal_s = push_all(2)
    assert normal.bad == 0 and normal.reassembler.pending == 64
    assert forged_s < 3 * normal_s


def test_the_largest_allowed_message_still_arrives():
    big = bytes(range(256)) * (MAX_FRAGMENTS * MAX_PAYLOAD // 256)
    rx = LinkReceiver()
    message = None
    for datagram in Packetizer().packetize(MsgType.FULL_FRAME, 0, big):
        message = rx.push(datagram, 0) or message
    assert message is not None and bytes(message.data) == big and rx.bad == 0
    rx = LinkReceiver()
    for datagram in Packetizer().packetize(MsgType.FULL_FRAME, 0, big + b"x"):
        assert rx.push(datagram, 0) is None
    assert rx.bad == MAX_FRAGMENTS + 1
