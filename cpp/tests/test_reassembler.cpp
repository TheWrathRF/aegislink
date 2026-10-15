#include "aegis/protocol.hpp"
#include "aegis/reassembler.hpp"
#include "test_runner.hpp"

#include <algorithm>
#include <optional>
#include <random>
#include <vector>

using namespace aegis;

static std::vector<std::uint8_t> sample_message(std::size_t n, std::uint8_t salt = 0) {
    std::vector<std::uint8_t> v(n);
    for (std::size_t i = 0; i < n; ++i) v[i] = static_cast<std::uint8_t>(i * 13 + salt);
    return v;
}

static std::vector<ParsedPacket> parse_all(const std::vector<std::vector<std::uint8_t>>& packets) {
    std::vector<ParsedPacket> parsed;
    for (const auto& p : packets) parsed.push_back(parse_packet(p));
    return parsed;
}

// Feeds the fragments in the given order at time now_us; returns every completed message.
static std::vector<Message> feed(Reassembler& r, const std::vector<ParsedPacket>& frags,
                                 std::uint64_t now_us = 0) {
    std::vector<Message> done;
    for (const auto& f : frags) {
        if (auto m = r.push(f.header, f.payload, now_us)) done.push_back(std::move(*m));
    }
    return done;
}

TEST(single_fragment_completes_at_once) {
    Packetizer packetizer;
    const auto message = sample_message(100);
    const auto packets = packetizer.packetize(MsgType::Telemetry, 5, message);
    Reassembler r;
    const auto done = feed(r, parse_all(packets));
    CHECK_EQ(done.size(), std::size_t{1});
    CHECK(done[0].type == MsgType::Telemetry);
    CHECK_EQ(done[0].timestamp_us, 5u);
    CHECK(done[0].data == message);
    CHECK_EQ(r.pending(), std::size_t{0});
    CHECK_EQ(r.stats().completed, 1u);
}

TEST(in_order_and_reversed_and_shuffled) {
    Packetizer packetizer;
    const auto message = sample_message(10'000);
    for (int order = 0; order < 3; ++order) {
        // The parsed payload spans point into packets, so packets must outlive frags.
        const auto packets = packetizer.packetize(MsgType::BgKeyframe, 0, message);
        auto frags = parse_all(packets);
        if (order == 1) std::reverse(frags.begin(), frags.end());
        if (order == 2) std::shuffle(frags.begin(), frags.end(), std::mt19937(1234));
        Reassembler r;
        const auto done = feed(r, frags);
        CHECK_EQ(done.size(), std::size_t{1});
        CHECK(!done.empty() && done[0].data == message);
        CHECK_EQ(r.pending(), std::size_t{0});
    }
}

TEST(duplicate_fragment_is_ignored) {
    Packetizer packetizer;
    const auto message = sample_message(3000);
    const auto packets = packetizer.packetize(MsgType::RoiPatch, 0, message);
    auto frags = parse_all(packets);
    frags.insert(frags.begin() + 1, frags[0]);
    Reassembler r;
    const auto done = feed(r, frags);
    CHECK_EQ(done.size(), std::size_t{1});
    CHECK(!done.empty() && done[0].data == message);
    CHECK_EQ(r.stats().duplicates, 1u);
}

TEST(late_duplicate_after_completion_is_ignored) {
    Packetizer packetizer;
    const auto packets = packetizer.packetize(MsgType::RoiPatch, 0, sample_message(3000));
    const auto frags = parse_all(packets);
    Reassembler r;
    CHECK_EQ(feed(r, frags, 1000).size(), std::size_t{1});
    CHECK_EQ(feed(r, {frags[1]}, 2000).size(), std::size_t{0});
    CHECK_EQ(r.pending(), std::size_t{0});
    CHECK_EQ(r.stats().duplicates, 1u);
    CHECK_EQ(r.stats().completed, 1u);
}

TEST(missing_fragment_expires_after_timeout) {
    Packetizer packetizer;
    const auto packets = packetizer.packetize(MsgType::FullFrame, 0, sample_message(5000));
    auto frags = parse_all(packets);
    frags.erase(frags.begin() + 2);
    Reassembler r(200'000);
    CHECK_EQ(feed(r, frags, 1'000'000).size(), std::size_t{0});
    CHECK_EQ(r.pending(), std::size_t{1});

    r.expire(1'200'000);  // exactly 200 ms old: still waiting
    CHECK_EQ(r.pending(), std::size_t{1});
    r.expire(1'200'001);
    CHECK_EQ(r.pending(), std::size_t{0});
    CHECK_EQ(r.stats().expired, 1u);
}

TEST(fragment_after_timeout_does_not_complete_old_message) {
    Packetizer packetizer;
    const auto packets = packetizer.packetize(MsgType::FullFrame, 0, sample_message(2400));
    const auto frags = parse_all(packets);
    Reassembler r(200'000);
    CHECK_EQ(feed(r, {frags[0]}, 0).size(), std::size_t{0});
    // The push at 300 ms expires the old partial first, so this fragment starts a new one.
    CHECK_EQ(feed(r, {frags[1]}, 300'000).size(), std::size_t{0});
    CHECK_EQ(r.stats().expired, 1u);
    CHECK_EQ(r.pending(), std::size_t{1});
}

TEST(interleaved_messages) {
    Packetizer packetizer;
    const auto a = sample_message(4000, 1);
    const auto b = sample_message(2500, 2);
    const auto pa = packetizer.packetize(MsgType::RoiPatch, 1, a);
    const auto pb = packetizer.packetize(MsgType::RoiPatch, 2, b);
    const auto fa = parse_all(pa);
    const auto fb = parse_all(pb);
    std::vector<ParsedPacket> mixed;
    for (std::size_t i = 0; i < std::max(fa.size(), fb.size()); ++i) {
        if (i < fa.size()) mixed.push_back(fa[i]);
        if (i < fb.size()) mixed.push_back(fb[i]);
    }
    Reassembler r;
    const auto done = feed(r, mixed);
    CHECK_EQ(done.size(), std::size_t{2});
    CHECK(done.size() == 2 && done[0].data == b && done[1].data == a);  // b has fewer fragments
}

TEST(inconsistent_fragment_is_dropped) {
    Packetizer packetizer;
    const auto message = sample_message(3000);
    const auto packets = packetizer.packetize(MsgType::RoiPatch, 0, message);
    auto frags = parse_all(packets);
    Reassembler r;
    feed(r, {frags[0]});
    Header wrong = frags[1].header;
    wrong.frag_count = 4;
    CHECK(!r.push(wrong, frags[1].payload, 0).has_value());
    wrong = frags[1].header;
    wrong.type = MsgType::Tracks;
    CHECK(!r.push(wrong, frags[1].payload, 0).has_value());
    wrong = frags[1].header;
    wrong.frag_index = wrong.frag_count;
    CHECK(!r.push(wrong, frags[1].payload, 0).has_value());
    CHECK_EQ(r.stats().inconsistent, 3u);
    const auto done = feed(r, {frags[1], frags[2]});
    CHECK_EQ(done.size(), std::size_t{1});
    CHECK(!done.empty() && done[0].data == message);
}

TEST(pending_limit_evicts_oldest) {
    Packetizer packetizer;
    Reassembler r(200'000, 2);
    std::vector<std::vector<std::vector<std::uint8_t>>> messages;
    for (int i = 0; i < 3; ++i) messages.push_back(packetizer.packetize(MsgType::FullFrame, 0, sample_message(2400)));
    for (int i = 0; i < 3; ++i) feed(r, {parse_packet(messages[i][0])}, static_cast<std::uint64_t>(i));
    CHECK_EQ(r.pending(), std::size_t{2});
    CHECK_EQ(r.stats().evicted, 1u);
    // The oldest (message 0) was evicted, so its second fragment no longer completes it.
    CHECK_EQ(feed(r, {parse_packet(messages[0][1])}, 3).size(), std::size_t{0});
    CHECK_EQ(feed(r, {parse_packet(messages[2][1])}, 4).size(), std::size_t{1});
}
