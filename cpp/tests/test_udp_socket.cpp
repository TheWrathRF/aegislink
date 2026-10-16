#include "test_runner.hpp"
#include "udp_socket.hpp"

#include <cstdint>
#include <vector>

using namespace aegis;

TEST(sends_and_receives_on_loopback) {
    NetworkInit net;
    UdpSocket a(0);
    UdpSocket b(0);
    CHECK(a.port() != 0);
    CHECK(a.port() != b.port());

    const std::vector<std::uint8_t> message{1, 2, 3, 250};
    CHECK(a.send_to(b.port(), message));
    const auto ready = wait_readable({&a, &b}, 1'000'000);
    CHECK_EQ(ready.size(), std::size_t{1});
    CHECK(!ready.empty() && ready[0] == 1);

    std::vector<std::uint8_t> buffer(2048);
    const auto got = b.receive(buffer);
    CHECK(got.has_value());
    if (got) {
        CHECK_EQ(got->size, message.size());
        CHECK_EQ(got->from_port, a.port());
        CHECK(std::vector<std::uint8_t>(buffer.begin(), buffer.begin() + 4) == message);
    }
    CHECK(!b.receive(buffer).has_value());
}

TEST(wait_times_out_when_nothing_arrives) {
    NetworkInit net;
    UdpSocket a(0);
    CHECK(wait_readable({&a}, 20'000).empty());
}

TEST(sending_to_a_closed_port_does_not_break_receiving) {
    NetworkInit net;
    UdpSocket a(0);
    std::uint16_t closed_port = 0;
    {
        UdpSocket gone(0);
        closed_port = gone.port();
    }
    const std::vector<std::uint8_t> message{7};
    a.send_to(closed_port, message);
    UdpSocket b(0);
    CHECK(b.send_to(a.port(), message));
    CHECK_EQ(wait_readable({&a}, 1'000'000).size(), std::size_t{1});
    std::vector<std::uint8_t> buffer(64);
    const auto got = a.receive(buffer);
    CHECK(got.has_value() && got->size == 1 && buffer[0] == 7);
}
