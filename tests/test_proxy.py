"""ew_chaos_proxy as a real process, driven over real UDP sockets on 127.0.0.1."""

import shutil
import socket
import subprocess
import sysconfig
import time
from dataclasses import dataclass

import pytest

from aegislink.common.proxy_control import ProxyControl, ProxyError

CLEAN = dict(loss_pct=0, burst=1, kbps=0, queue_ms=250, delay_ms=0, jitter_ms=0, blackout_ms=0)


def find_proxy():
    # pip installs the proxy next to the Python scripts, which may not be on PATH (no venv activated).
    name = "ew_chaos_proxy"
    path = shutil.which(name, path=sysconfig.get_path("scripts")) or shutil.which(name)
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
