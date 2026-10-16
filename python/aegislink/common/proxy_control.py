"""Client for the control port of ew_chaos_proxy: change link settings and read its stats."""

import socket
import time


class ProxyError(Exception):
    """The proxy refused a command; the message is its reason."""


def parse_pairs(text):
    """Parses "key=value key=value" into a dict of floats ("link" becomes an int)."""
    pairs = {}
    for token in text.split():
        key, _, value = token.partition("=")
        pairs[key] = int(value) if key == "link" else float(value)
    return pairs


def parse_stats(text):
    """Parses a stats report into {"uptime_ms": ..., "links": {1: {...}, 2: {...}}}."""
    lines = text.strip().splitlines()
    uptime = parse_pairs(lines[0].removeprefix("stats"))
    links = {}
    for line in lines[1:]:
        pairs = parse_pairs(line)
        links[pairs.pop("link")] = pairs
    return {"uptime_ms": uptime["uptime_ms"], "links": links}


class ProxyControl:
    """Sends commands to the proxy. The proxy sends its stats (4 per second) to whoever sent the
    last command, so this socket also receives them."""

    def __init__(self, port, timeout_s=1.0):
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(timeout_s)

    def close(self):
        self.sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def command(self, text):
        """Sends one command and returns the link's settings from the reply."""
        self.sock.sendto(text.encode(), ("127.0.0.1", self.port))
        while True:
            reply = self.sock.recv(65535).decode()
            if reply.startswith("stats"):
                continue
            if reply.startswith("error: "):
                raise ProxyError(reply.removeprefix("error: "))
            return parse_pairs(reply.removeprefix("ok"))

    def configure(self, link, **settings):
        """configure(1, loss_pct=30, burst=4) -> the link's settings after the change."""
        text = " ".join([f"link={link}"] + [f"{key}={value}" for key, value in settings.items()])
        return self.command(text)

    def wait_ready(self, link=1, timeout_s=5.0):
        """Retries until the proxy answers (it may still be starting)."""
        deadline = time.monotonic() + timeout_s
        while True:
            try:
                return self.command(f"link={link}")
            # Windows reports the ICMP "port unreachable" of an earlier try as a reset.
            except (TimeoutError, ConnectionResetError):
                if time.monotonic() > deadline:
                    raise

    def read_stats(self):
        """Waits for the next stats report."""
        while True:
            text = self.sock.recv(65535).decode()
            if text.startswith("stats"):
                return parse_stats(text)
