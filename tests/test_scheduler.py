"""The UAV's priority scheduler and its AIMD budget, without sockets."""

import math

import pytest

from aegislink._core import MAX_DATAGRAM, MsgType, ParseError, parse_packet
from aegislink.common.messages import ReceiverReport
from aegislink.common.net import LinkReceiver
from aegislink.uav.scheduler import AimdBudget, FrameData, Scheduler

FPS = 30


def frame_data(i, tracks=3, patch_size=300, keyframe_size=5000):
    t_us = round(i / FPS * 1e6)
    return FrameData(
        frame=i, timestamp_us=t_us,
        telemetry=b"T" * 32,
        tracks=b"K" * (5 + 11 * tracks),
        patches={tid: (lambda tid=tid: bytes([tid]) * patch_size) for tid in range(1, tracks + 1)},
        keyframe=lambda: b"B" * keyframe_size)


def parsed(datagrams):
    packets = [parse_packet(d) for d in datagrams]
    assert all(p.error == ParseError.OK for p in packets)
    return packets


def run(scheduler, seconds, budget_bps, **kwargs):
    """Ticks at 30 Hz; returns [(time, [datagrams])]."""
    out = []
    for i in range(round(seconds * FPS)):
        now = i / FPS
        out.append((now, scheduler.tick(now, budget_bps, frame_data(i, **kwargs))))
    return out


def test_never_sends_more_than_the_budget_allows():
    budget = 50_000
    ticks = run(Scheduler(), 10, budget)
    cap = MAX_DATAGRAM + budget / 8 * Scheduler.BURST_S
    sizes = [(t, sum(map(len, d))) for t, d in ticks]
    for window in (0.1, 0.5, 2.0):
        for start in range(len(sizes)):
            t0 = sizes[start][0]
            sent = sum(s for t, s in sizes[start:] if t < t0 + window)
            assert sent <= budget / 8 * window + cap
    total = sum(s for _, s in sizes)
    assert total >= 0.95 * budget / 8 * 10  # and it uses what it may


def test_telemetry_and_tracks_every_frame_at_50_kbps():
    ticks = run(Scheduler(), 10, 50_000, tracks=6)
    for _, datagrams in ticks:
        types = [p.header.type for p in parsed(datagrams)]
        assert types[:2] == [MsgType.TELEMETRY, MsgType.TRACKS]


def test_keyframe_is_paced_and_arrives_whole_at_50_kbps():
    rx = LinkReceiver(timeout_us=4_000_000)
    keyframes = 0
    patches = 0
    for now, datagrams in run(Scheduler(), 20, 50_000):
        for d in datagrams:
            message = rx.push(d, round(now * 1e6))
            if message and message.type == MsgType.BG_KEYFRAME:
                assert message.data == b"B" * 5000
                keyframes += 1
            if message and message.type == MsgType.ROI_PATCH:
                patches += 1
    assert keyframes >= 1
    assert patches >= 20  # about 1-2 per second besides the keyframes


def test_seq_numbers_have_no_gaps():
    # The receiver counts gaps in seq as losses, so a skipped item must not use a number.
    ticks = run(Scheduler(), 5, 20_000, tracks=6)
    seqs = [p.header.seq for _, d in ticks for p in parsed(d)]
    assert seqs == list(range(len(seqs)))


def test_low_budget_skips_whole_frames_and_says_so():
    s = Scheduler()
    ticks = run(s, 5, 16_000, tracks=6)  # control alone needs about 38 kbps
    assert s.limited
    tracks = sum(1 for _, d in ticks for p in parsed(d) if p.header.type == MsgType.TRACKS)
    assert 0.3 * 150 < tracks < 0.7 * 150


def test_high_budget_meets_the_target_rates_and_is_not_limited():
    s = Scheduler()
    counts = {}
    for i in range(10 * FPS):
        if i == 2 * FPS:
            s.limited = False
        for p in parsed(s.tick(i / FPS, 2_000_000, frame_data(i))):
            if i >= 2 * FPS and p.header.frag_index == 0:
                counts[p.header.type] = counts.get(p.header.type, 0) + 1
    assert not s.limited
    assert counts[MsgType.TRACKS] == 8 * FPS
    # 3 tracks at 10 Hz, keyframes at 1 Hz, over 8 s.
    assert 3 * 10 * 8 * 0.9 <= counts[MsgType.ROI_PATCH] <= 3 * 10 * 8 * 1.1
    assert 7 <= counts[MsgType.BG_KEYFRAME] <= 9


def test_new_track_gets_its_patch_first():
    s = Scheduler()
    for i in range(30):
        s.tick(i / FPS, 200_000, frame_data(i, tracks=2))
    data = frame_data(30, tracks=3)
    patch_ids = [p.payload[0] for p in parsed(s.tick(1.0, 200_000, data))
                 if p.header.type == MsgType.ROI_PATCH]
    assert patch_ids[0] == 3


def test_missing_patch_is_skipped():
    s = Scheduler()
    data = frame_data(0, tracks=2)
    data.patches[1] = lambda: None   # the box left the image
    types = [p.header.type for p in parsed(s.tick(0.0, 1_000_000, data))]
    assert types.count(MsgType.ROI_PATCH) == 1


def report(t_s, received, received_bytes, min_delay_us, highest_seq=None):
    return ReceiverReport(round(t_s * 1e6), received if highest_seq is None else highest_seq,
                          received, received_bytes, min_delay_us)


def feed(aimd, reports, limited=True):
    for r in reports:
        aimd.on_report(r, r.time_us / 1e6, limited)


def test_grows_only_while_the_budget_holds_data_back():
    aimd = AimdBudget()
    start = aimd.bps
    feed(aimd, [report(0.25 * i, 10 * i, 1000 * i, 5000) for i in range(5)])
    assert aimd.bps == start + 5 * AimdBudget.STEP_BPS
    feed(aimd, [report(0.25 * i, 10 * i, 1000 * i, 5000) for i in range(5, 10)], limited=False)
    assert aimd.bps == start + 5 * AimdBudget.STEP_BPS


def test_queueing_delay_cuts_to_085_of_the_received_rate():
    aimd = AimdBudget(start_bps=100_000)
    feed(aimd, [report(0.25 * i, 10 * i, 1562 * i, 5000) for i in range(4)])  # 50 kbps received
    before = aimd.bps
    aimd.on_report(report(1.0, 40, 1562 * 4, 5000 + 50_000), 1.0, True)
    assert aimd.bps == pytest.approx(0.85 * 1562 * 8 / 0.25)
    assert aimd.bps < before
    # Within the hold-off the queue is still draining: no second cut.
    cut = aimd.bps
    aimd.on_report(report(1.25, 50, 1562 * 5, 5000 + 45_000), 1.25, True)
    assert aimd.bps == cut


def test_random_loss_alone_never_cuts():
    aimd = AimdBudget()
    reports = []
    for i in range(40):
        # 30 % of the packets lost, the delay flat (with jitter).
        reports.append(report(0.25 * i, 7 * i, 700 * i, 5000 + (i * 7919) % 3000, highest_seq=10 * i))
    feed(aimd, reports)
    assert aimd.loss == pytest.approx(0.3)
    assert aimd.bps == AimdBudget.START_BPS + 40 * AimdBudget.STEP_BPS


def test_no_reports_hold_the_budget():
    aimd = AimdBudget()
    feed(aimd, [report(0.25 * i, 10 * i, 1000 * i, 5000) for i in range(3)])
    held = aimd.bps
    # A 5 s blackout: no reports, then one that shows nothing arrived in between.
    aimd.on_report(report(5.5, 20, 2000, 0), 5.5, False)
    assert aimd.bps == held


def test_old_low_delays_are_forgotten():
    # The link's fixed delay grows by 100 ms: congestion at first, normal after 10 s.
    aimd = AimdBudget()
    feed(aimd, [report(0.25 * i, 10 * i, 1000 * i, 5000) for i in range(4)])
    feed(aimd, [report(1 + 0.25 * i, 40 + 10 * i, 4000 + 1000 * i, 105_000) for i in range(48)])
    assert aimd.queue_delay_us == 0


def test_works_with_any_clock_offset():
    aimd = AimdBudget(start_bps=100_000)
    offset = -7_000_000_000
    feed(aimd, [report(0.25 * i, 10 * i, 1562 * i, offset + 5000) for i in range(4)])
    aimd.on_report(report(1.0, 40, 1562 * 4, offset + 60_000), 1.0, True)
    assert aimd.bps < 100_000


def test_stays_within_limits_and_ignores_stale_reports():
    aimd = AimdBudget(start_bps=20_000)
    feed(aimd, [report(0.25 * i, 10 * i, 10 * i, 5000) for i in range(3)])
    aimd.on_report(report(1.0, 40, 40, 900_000), 1.0, True)
    assert aimd.bps == AimdBudget.MIN_BPS
    bps = aimd.bps
    aimd.on_report(report(0.5, 20, 20, 5000), 1.1, True)  # older than the last one
    assert aimd.bps == bps
    many = AimdBudget(start_bps=AimdBudget.MAX_BPS - 1000)
    feed(many, [report(0.25 * i, 10 * i, 10 * i, 5000) for i in range(5)])
    assert many.bps == AimdBudget.MAX_BPS


class Bottleneck:
    """A rate-limited link with a drop-tail queue of `queue_s`, like the proxy's."""

    def __init__(self, bps, queue_s=0.25):
        self.rate = bps / 8
        self.queue_s = queue_s
        self.free_at = 0.0
        self.dropped = 0

    def send(self, now, size):
        """Returns the arrival time, or None if the queue drops it."""
        start = max(now, self.free_at)
        if start - now > self.queue_s:
            self.dropped += 1
            return None
        self.free_at = start + size / self.rate
        return self.free_at


def test_closed_loop_at_50_kbps_converges_without_drops():
    link = Bottleneck(50_000)
    scheduler = Scheduler()
    aimd = AimdBudget()
    rx = LinkReceiver(timeout_us=4_000_000, delay_types={MsgType.TELEMETRY, MsgType.TRACKS})
    in_flight = []          # (arrival, datagram)
    next_report = 0.25
    tracks_frames = set()
    dropped_after = None
    budgets = []
    for i in range(40 * FPS):
        now = i / FPS
        for d in scheduler.tick(now, aimd.bps, frame_data(i, tracks=4)):
            arrival = link.send(now, len(d))
            if arrival is not None:
                in_flight.append((arrival, d))
        if i == 10 * FPS:
            dropped_after = link.dropped
        arrived = [x for x in in_flight if x[0] <= now]
        in_flight = [x for x in in_flight if x[0] > now]
        for arrival, d in arrived:
            message = rx.push(d, round(arrival * 1e6))
            if message and message.type == MsgType.TRACKS:
                tracks_frames.add(round(message.timestamp_us * FPS / 1e6))
        if now >= next_report:
            # The report reaches the UAV right away; reports are sent 4 times a second.
            aimd.on_report(rx.report(round(now * 1e6)), now, scheduler.limited)
            scheduler.limited = False
            next_report += 0.25
        if i >= 10 * FPS:
            budgets.append(aimd.bps)
    assert link.dropped == dropped_after          # no drops after the first 10 s
    first, last = 10 * FPS, 40 * FPS - 10         # the last frames may still be in flight
    delivered = sum(1 for f in range(first, last) if f in tracks_frames) / (last - first)
    assert delivered >= 0.99
    assert 0.6 * 50_000 <= min(budgets) and max(budgets) <= 1.3 * 50_000
    assert math.isclose(sum(budgets) / len(budgets), 50_000, rel_tol=0.25)
