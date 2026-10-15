"""Messages cross real UDP sockets on 127.0.0.1 in fragments and arrive intact."""

from aegislink.tools import loopback_check


def test_50kb_message_over_loopback():
    result = loopback_check.run(size=50_000, seed=7)
    assert result.fragments == 42
    assert result.intact
    assert result.parse_errors == 0


def test_several_messages_keep_their_order_and_content():
    result = loopback_check.run(size=10_000, count=20, seed=8)
    assert result.messages == 20
    assert result.intact
