"""Send random messages over loopback UDP in fragments and check that they arrive intact.

Run: python -m aegislink.tools.loopback_check --size 50000
"""

import argparse
import random
import socket
import sys
import time
from dataclasses import dataclass

from aegislink import _core


@dataclass
class Result:
    messages: int      # messages reassembled
    fragments: int     # datagrams sent
    intact: bool       # every message arrived, byte for byte, in order
    parse_errors: int
    seconds: float


def run(size=50_000, count=1, seed=0, timeout_s=2.0):
    rng = random.Random(seed)
    sent = [rng.randbytes(size) for _ in range(count)]

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as rx, \
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as tx:
        # Every fragment is sent before any is read, so the receive buffer must hold them all.
        rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        rx.bind(("127.0.0.1", 0))
        rx.settimeout(timeout_s)

        start = time.perf_counter()
        packetizer = _core.Packetizer()
        fragments = 0
        for i, message in enumerate(sent):
            for packet in packetizer.packetize(_core.MsgType.FULL_FRAME, i, message):
                tx.sendto(packet, rx.getsockname())
                fragments += 1

        reassembler = _core.Reassembler()
        received = []
        parse_errors = 0
        while len(received) < count:
            try:
                # Larger than MAX_DATAGRAM: on Windows a too-small buffer raises instead of truncating.
                datagram = rx.recv(65535)
            except TimeoutError:
                break
            packet = _core.parse_packet(datagram)
            if packet.error != _core.ParseError.OK:
                parse_errors += 1
                continue
            message = reassembler.push(packet.header, packet.payload, time.monotonic_ns() // 1000)
            if message is not None:
                received.append(message.data)
        seconds = time.perf_counter() - start

    return Result(messages=len(received), fragments=fragments, intact=received == sent,
                  parse_errors=parse_errors, seconds=seconds)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--size", type=int, default=50_000, help="bytes per message")
    parser.add_argument("--count", type=int, default=1, help="number of messages")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    result = run(size=args.size, count=args.count, seed=args.seed)
    print(f"{result.messages}/{args.count} messages of {args.size} B in {result.fragments} fragments, "
          f"{result.seconds * 1000:.1f} ms, parse errors {result.parse_errors}: "
          f"{'intact' if result.intact else 'CORRUPTED OR MISSING'}")
    return 0 if result.intact else 1


if __name__ == "__main__":
    sys.exit(main())
