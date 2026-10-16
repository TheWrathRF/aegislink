"""Send a numbered packet stream through ew_chaos_proxy and measure what comes out.

With a proxy running, for example
    ew_chaos_proxy --link 5600:5601 --control 5700
this sets link 1 and measures loss, burst length, throughput and delay:
    python -m aegislink.tools.link_meter --loss-pct 30 --burst 4
    python -m aegislink.tools.link_meter --kbps 50 --size 1230 --rate 10 --packets 60
"""

import argparse
import socket
import sys
import threading
import time
from dataclasses import dataclass, field

from aegislink import _core
from aegislink.common.proxy_control import ProxyControl


@dataclass
class Measurement:
    sent: int
    received: int
    loss: float         # fraction of the sent packets that never arrived
    mean_burst: float   # mean run of consecutive lost packets (0 when none was lost)
    kbps: float         # received rate in datagram bytes, from the first to the last arrival
    min_delay_ms: float
    mean_delay_ms: float
    max_delay_ms: float
    corrupt: int        # datagrams that failed the header or CRC check
    bursts: list = field(default_factory=list)  # (first lost seq, length) of each loss burst


def loss_bursts(received_seqs, sent):
    """Runs of missing sequence numbers in 0..sent-1, as (first, length) pairs."""
    bursts = []
    run_start = None
    for seq in range(sent + 1):
        missing = seq < sent and seq not in received_seqs
        if missing and run_start is None:
            run_start = seq
        elif not missing and run_start is not None:
            bursts.append((run_start, seq - run_start))
            run_start = None
    return bursts


def measure(send_port, receiver, packets=20_000, size=200, rate_pps=10_000.0, settle_s=0.5):
    """Sends `packets` datagrams of `size` bytes (header included) at `rate_pps` to 127.0.0.1:send_port
    and receives them on `receiver`, a bound UDP socket. Receiving stops `settle_s` after the
    last datagram was sent or arrived, so settle_s must exceed the link's delay plus queueing."""
    if not _core.HEADER_SIZE <= size <= _core.MAX_DATAGRAM:
        raise ValueError(f"size must be {_core.HEADER_SIZE} to {_core.MAX_DATAGRAM} bytes")

    arrivals = []  # (arrival time in us, datagram)
    sending_done = threading.Event()

    def receive():
        receiver.settimeout(0.05)
        last_activity = time.monotonic()
        while True:
            try:
                datagram = receiver.recv(65535)
            except TimeoutError:
                if sending_done.is_set() and time.monotonic() - last_activity >= settle_s:
                    return
                continue
            arrivals.append((time.monotonic_ns() // 1000, datagram))
            last_activity = time.monotonic()

    thread = threading.Thread(target=receive)
    thread.start()
    packetizer = _core.Packetizer()
    payload = bytes(range(256)) * (size // 256 + 1)
    payload = payload[:size - _core.HEADER_SIZE]
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as tx:
        start = time.perf_counter()
        for i in range(packets):
            # Sleep only when well ahead: a sleep is too coarse to pace single packets.
            ahead = start + i / rate_pps - time.perf_counter()
            if ahead > 0.002:
                time.sleep(ahead)
            [datagram] = packetizer.packetize(_core.MsgType.FULL_FRAME, time.monotonic_ns() // 1000,
                                              payload)
            tx.sendto(datagram, ("127.0.0.1", send_port))
    sending_done.set()
    thread.join()

    seqs = set()
    delays_ms = []
    corrupt = 0
    for arrival_us, datagram in arrivals:
        packet = _core.parse_packet(datagram)
        if packet.error != _core.ParseError.OK:
            corrupt += 1
            continue
        seqs.add(packet.header.seq)
        delays_ms.append((arrival_us - packet.header.timestamp_us) / 1000)

    bursts = loss_bursts(seqs, packets)
    lost = packets - len(seqs)
    kbps = 0.0
    if len(arrivals) >= 2:
        # The first datagram only starts the clock, so it is not counted.
        seconds = (arrivals[-1][0] - arrivals[0][0]) / 1e6
        bits = 8 * sum(len(d) for _, d in arrivals[1:])
        kbps = bits / seconds / 1000 if seconds > 0 else 0.0
    return Measurement(
        sent=packets,
        received=len(seqs),
        loss=lost / packets if packets else 0.0,
        mean_burst=lost / len(bursts) if bursts else 0.0,
        kbps=kbps,
        min_delay_ms=min(delays_ms, default=0.0),
        mean_delay_ms=sum(delays_ms) / len(delays_ms) if delays_ms else 0.0,
        max_delay_ms=max(delays_ms, default=0.0),
        corrupt=corrupt,
        bursts=bursts,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--listen", type=int, default=5600, help="the proxy's listen port")
    parser.add_argument("--forward", type=int, default=5601, help="the port the proxy sends to")
    parser.add_argument("--control", type=int, default=5700, help="the proxy's control port")
    parser.add_argument("--link", type=int, default=1, help="which proxy link to set")
    parser.add_argument("--loss-pct", type=float, default=0.0)
    parser.add_argument("--burst", type=float, default=1.0, help="mean loss burst length (packets)")
    parser.add_argument("--kbps", type=float, default=0.0, help="rate limit, 0 for none")
    parser.add_argument("--queue-ms", type=float, default=250.0)
    parser.add_argument("--delay-ms", type=float, default=0.0)
    parser.add_argument("--jitter-ms", type=float, default=0.0)
    parser.add_argument("--packets", type=int, default=20_000)
    parser.add_argument("--size", type=int, default=200, help="datagram size in bytes")
    parser.add_argument("--rate", type=float, default=10_000.0, help="packets per second")
    args = parser.parse_args()

    with ProxyControl(args.control) as control:
        settings = control.configure(
            args.link, loss_pct=args.loss_pct, burst=args.burst, kbps=args.kbps,
            queue_ms=args.queue_ms, delay_ms=args.delay_ms, jitter_ms=args.jitter_ms)
    print("proxy " + " ".join(f"{key}={value:g}" for key, value in settings.items()))

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        receiver.bind(("127.0.0.1", args.forward))
        settle_s = (args.queue_ms + args.delay_ms + args.jitter_ms) / 1000 + 0.3
        m = measure(args.listen, receiver, args.packets, args.size, args.rate, settle_s)

    print(f"sent {m.sent}, received {m.received}, corrupt {m.corrupt}")
    print(f"loss        {m.loss * 100:6.2f} %    (set {args.loss_pct:g} %)")
    print(f"mean burst  {m.mean_burst:6.2f}      (set {args.burst:g})")
    limit = f"limit {args.kbps:g}" if args.kbps > 0 else "no limit"
    print(f"throughput  {m.kbps:8.1f} kbps ({limit})")
    print(f"delay       {m.min_delay_ms:.1f} / {m.mean_delay_ms:.1f} / {m.max_delay_ms:.1f} ms "
          f"(min / mean / max)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
