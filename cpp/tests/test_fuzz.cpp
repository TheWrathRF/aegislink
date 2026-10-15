// Randomized robustness tests: garbage in must never crash, read out of bounds (ASan / MSVC debug
// checks catch that) or produce a message that was never sent.
#include "aegis/protocol.hpp"
#include "aegis/reassembler.hpp"
#include "test_runner.hpp"

#include <random>
#include <vector>

using namespace aegis;

static std::vector<std::uint8_t> random_bytes(std::mt19937& rng, std::size_t n) {
    std::uniform_int_distribution<int> byte(0, 255);
    std::vector<std::uint8_t> v(n);
    for (auto& b : v) b = static_cast<std::uint8_t>(byte(rng));
    return v;
}

TEST(random_datagrams_are_rejected) {
    std::mt19937 rng(1);
    std::uniform_int_distribution<std::size_t> size(0, kMaxDatagram + 100);
    int accepted = 0;
    for (int i = 0; i < 200'000; ++i) {
        const auto datagram = random_bytes(rng, size(rng));
        if (parse_packet(datagram).error == ParseError::Ok) ++accepted;
    }
    CHECK_EQ(accepted, 0);
}

TEST(mutated_packets_never_crash_or_corrupt) {
    std::mt19937 rng(2);
    std::uniform_int_distribution<std::size_t> msg_size(0, 6000);
    std::uniform_int_distribution<int> mutation(0, 3);
    Packetizer packetizer;
    Reassembler reassembler(200'000, 16);
    std::vector<std::vector<std::uint8_t>> sent;
    int delivered = 0;
    int parse_ok = 0;

    for (int round = 0; round < 3000; ++round) {
        sent.push_back(random_bytes(rng, msg_size(rng)));
        for (auto packet : packetizer.packetize(MsgType::RoiPatch, 0, sent.back())) {
            switch (mutation(rng)) {
                case 0:  // flip one random bit
                    if (!packet.empty()) packet[rng() % packet.size()] ^= static_cast<std::uint8_t>(1u << (rng() % 8));
                    break;
                case 1:  // truncate
                    packet.resize(rng() % (packet.size() + 1));
                    break;
                case 2:  // drop it entirely
                    continue;
                default:  // deliver intact
                    break;
            }
            const auto parsed = parse_packet(packet);
            if (parsed.error != ParseError::Ok) continue;
            ++parse_ok;
            if (auto m = reassembler.push(parsed.header, parsed.payload, static_cast<std::uint64_t>(round) * 1000)) {
                ++delivered;
                CHECK(m->msg_id < sent.size() && m->data == sent[m->msg_id]);
            }
        }
    }
    // Sanity: the test must have exercised both the accept and the deliver paths.
    CHECK(parse_ok > 1000);
    CHECK(delivered > 10);
}
