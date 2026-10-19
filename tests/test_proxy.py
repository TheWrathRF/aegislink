"""ew_chaos_proxy as a real process, driven over real UDP sockets on 127.0.0.1."""

import socket
import subprocess
import time
from dataclasses import dataclass

import pytest

from aegislink.common.proxy_control import ProxyControl, ProxyError
from aegislink.tools import link_meter, run_demo

CLEAN = dict(loss_pct=0, burst=1, kbps=0, queue_ms=250, delay_ms=0, jitter_ms=0, blackout_ms=0)


def find_proxy():
    path = run_demo.find_proxy()
    if path is None:
        pytest.fail("ew_chaos_proxy not found: run 'python -m pip install -e .' first")
    return path


def free_ports(count):
    sockets = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(count)]
    for s in sockets:
        s.bind(("127.0.0.1", 0))
    ports = [s.getsockname()[1] for s in sockets]
    for s in sockets:
        s.close()
    return ports


@dataclass
class Proxy:
    listen_port: int        # send here
    receiver: socket.socket  # bound to the port the proxy forwards to
    control: ProxyControl


@pytest.fixture(scope="module")
def proxy():
    listen_port, control_port = free_ports(2)
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    receiver.bind(("127.0.0.1", 0))
    forward_port = receiver.getsockname()[1]
    process = subprocess.Popen(
        [find_proxy(), "--link", f"{listen_port}:{forward_port}", "--control", str(control_port),
         "--seed", "1"],
        stdout=subprocess.DEVNULL)
    control = ProxyControl(control_port)
    try:
        control.wait_ready()
        yield Proxy(listen_port, receiver, control)
    finally:
        process.terminate()
        process.wait(timeout=5)
        control.close()
        receiver.close()


@pytest.fixture
def clean_proxy(proxy):
    proxy.control.configure(1, **CLEAN)
    proxy.receiver.settimeout(0.0)
    try:
        while True:
            proxy.receiver.recv(65535)
    except BlockingIOError:
        pass
    proxy.receiver.settimeout(1.0)
    return proxy


def test_forwards_datagrams_unchanged_and_in_order(clean_proxy):
    sent = [bytes([i]) * (1 + i * 9) for i in range(100)]
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as tx:
        for datagram in sent:
            tx.sendto(datagram, ("127.0.0.1", clean_proxy.listen_port))
    received = [clean_proxy.receiver.recv(65535) for _ in sent]
    assert received == sent


def test_control_reply_lists_the_settings(clean_proxy):
    settings = clean_proxy.control.configure(1, loss_pct=10, burst=2, delay_ms=5)
    assert settings == dict(link=1, loss_pct=10, burst=2, kbps=0, queue_ms=250, delay_ms=5,
                            jitter_ms=0, blackout_ms=0)


def test_bad_commands_are_refused(clean_proxy):
    with pytest.raises(ProxyError, match="loss must be between 0 and 50"):
        clean_proxy.control.configure(1, loss_pct=60, burst=1)
    with pytest.raises(ProxyError, match="link must be 1 to 1"):
        clean_proxy.control.configure(2)
    assert clean_proxy.control.configure(1)["loss_pct"] == 0


def test_stats_arrive_four_times_a_second(clean_proxy):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as tx:
        for _ in range(10):
            tx.sendto(b"x" * 100, ("127.0.0.1", clean_proxy.listen_port))
    reports = []
    start = time.monotonic()
    while time.monotonic() - start < 1.0:
        reports.append(clean_proxy.control.read_stats())
    assert 3 <= len(reports) <= 5
    assert all(set(r["links"]) == {1} for r in reports)
    last = reports[-1]["links"][1]
    assert last["in"] >= 10 and last["out"] >= 10


@pytest.mark.parametrize("loss_pct, burst", [(30, 4), (10, 1.5)])
def test_measured_loss_and_burst_length_match_the_settings(clean_proxy, loss_pct, burst):
    clean_proxy.control.configure(1, loss_pct=loss_pct, burst=burst)
    # Linux caps socket buffers at a few hundred KB, so a faster stream could lose packets in the OS.
    m = link_meter.measure(clean_proxy.listen_port, clean_proxy.receiver,
                           packets=20_000, size=200, rate_pps=5_000)
    assert m.corrupt == 0
    assert abs(m.loss - loss_pct / 100) <= 0.1 * loss_pct / 100
    assert abs(m.mean_burst - burst) <= 0.1 * burst


@pytest.mark.parametrize("kbps", [50, 500])
def test_measured_throughput_never_exceeds_the_limit(clean_proxy, kbps):
    clean_proxy.control.configure(1, kbps=kbps)
    # Full-size datagrams at twice the rate the link allows, for about 4 s.
    rate_pps = 2 * kbps * 1000 / 8 / 1230
    m = link_meter.measure(clean_proxy.listen_port, clean_proxy.receiver,
                           packets=round(4 * rate_pps), size=1230, rate_pps=rate_pps)
    assert m.corrupt == 0
    assert m.kbps <= kbps * 1.005
    assert m.kbps >= kbps * 0.9
    assert m.loss > 0.4  # the excess is dropped at the queue, not delivered late


def test_delay_and_jitter(clean_proxy):
    clean_proxy.control.configure(1, delay_ms=30, jitter_ms=20)
    m = link_meter.measure(clean_proxy.listen_port, clean_proxy.receiver,
                           packets=500, size=200, rate_pps=500)
    assert m.received == 500
    assert m.min_delay_ms >= 30
    assert abs(m.mean_delay_ms - 40) <= 5
    # Windows may wake the proxy a few ms late.
    assert m.max_delay_ms <= 50 + 15


def test_blackout_drops_everything_until_it_ends(clean_proxy):
    clean_proxy.control.configure(1, blackout_ms=1000)
    m = link_meter.measure(clean_proxy.listen_port, clean_proxy.receiver,
                           packets=2000, size=200, rate_pps=1000)
    # One burst at the start: about the first second of packets.
    assert len(m.bursts) == 1
    first, length = m.bursts[0]
    assert first == 0
    assert 900 <= length <= 1100
