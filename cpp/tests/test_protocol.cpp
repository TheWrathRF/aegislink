#include "aegis/protocol.hpp"
#include "test_runner.hpp"

#include <stdexcept>
#include <vector>

using namespace aegis;

static std::vector<std::uint8_t> sample_payload(std::size_t n) {
    std::vector<std::uint8_t> v(n);
    for (std::size_t i = 0; i < n; ++i) v[i] = static_cast<std::uint8_t>(i * 7 + 3);
    return v;
}

static Header sample_header() {
    Header h;
    h.type = MsgType::RoiPatch;
    h.seq = 0x01020304;
    h.timestamp_us = 0x1122334455667788ull;
    h.msg_id = 0xA0B0C0D0;
    h.frag_index = 2;
    h.frag_count = 5;
    return h;
}

static bool threw_invalid_argument(Header h, std::size_t payload_size) {
    try {
        encode_packet(h, sample_payload(payload_size));
    } catch (const std::invalid_argument&) {
        return true;
    }
    return false;
}

TEST(header_is_30_bytes_little_endian) {
    const auto packet = encode_packet(sample_header(), {});
    CHECK_EQ(packet.size(), kHeaderSize);
    CHECK_EQ(kHeaderSize, std::size_t{30});
    CHECK_EQ(packet[0], 0x61);  // magic 0xAE61, low byte first
    CHECK_EQ(packet[1], 0xAE);
    CHECK_EQ(packet[2], kVersion);
    CHECK_EQ(packet[3], static_cast<std::uint8_t>(MsgType::RoiPatch));
    CHECK_EQ(packet[4], 0x04);  // seq
    CHECK_EQ(packet[7], 0x01);
    CHECK_EQ(packet[8], 0x88);  // timestamp_us
    CHECK_EQ(packet[15], 0x11);
    CHECK_EQ(packet[16], 0xD0);  // msg_id
    CHECK_EQ(packet[20], 2);     // frag_index
    CHECK_EQ(packet[22], 5);     // frag_count
    CHECK_EQ(packet[24], 0);     // payload_len
}

TEST(round_trip) {
    const auto payload = sample_payload(777);
    const auto packet = encode_packet(sample_header(), payload);
    CHECK_EQ(packet.size(), kHeaderSize + 777);

    const auto parsed = parse_packet(packet);
    CHECK(parsed.error == ParseError::Ok);
    CHECK(parsed.header.type == MsgType::RoiPatch);
    CHECK_EQ(parsed.header.seq, 0x01020304u);
    CHECK_EQ(parsed.header.timestamp_us, 0x1122334455667788ull);
    CHECK_EQ(parsed.header.msg_id, 0xA0B0C0D0u);
    CHECK_EQ(parsed.header.frag_index, 2);
    CHECK_EQ(parsed.header.frag_count, 5);
    CHECK_EQ(parsed.header.payload_len, 777);
    CHECK(std::vector<std::uint8_t>(parsed.payload.begin(), parsed.payload.end()) == payload);
}

TEST(max_payload_round_trips) {
    const auto packet = encode_packet(sample_header(), sample_payload(kMaxPayload));
    CHECK_EQ(packet.size(), kMaxDatagram);
    CHECK(parse_packet(packet).error == ParseError::Ok);
}

TEST(encode_rejects_bad_input) {
    CHECK(threw_invalid_argument(sample_header(), kMaxPayload + 1));
    Header h = sample_header();
    h.frag_index = 5;
    CHECK(threw_invalid_argument(h, 10));
    h.frag_count = 0;
    h.frag_index = 0;
    CHECK(threw_invalid_argument(h, 10));
}

TEST(rejects_short_datagram) {
    const auto packet = encode_packet(sample_header(), {});
    CHECK(parse_packet(std::span(packet).first(kHeaderSize - 1)).error == ParseError::TooShort);
    CHECK(parse_packet({}).error == ParseError::TooShort);
}

TEST(rejects_bad_magic_version_type) {
    auto packet = encode_packet(sample_header(), sample_payload(20));
    auto bad = packet;
    bad[0] ^= 0xFF;
    CHECK(parse_packet(bad).error == ParseError::BadMagic);
    bad = packet;
    bad[2] = 2;
    CHECK(parse_packet(bad).error == ParseError::BadVersion);
    bad = packet;
    bad[3] = 0;
    CHECK(parse_packet(bad).error == ParseError::BadType);
    bad[3] = 7;
    CHECK(parse_packet(bad).error == ParseError::BadType);
}

TEST(rejects_length_mismatch) {
    auto packet = encode_packet(sample_header(), sample_payload(20));
    auto truncated = packet;
    truncated.pop_back();
    CHECK(parse_packet(truncated).error == ParseError::BadLength);
    auto padded = packet;
    padded.push_back(0);
    CHECK(parse_packet(padded).error == ParseError::BadLength);
}

TEST(rejects_bad_fragment_fields) {
    auto packet = encode_packet(sample_header(), sample_payload(20));
    auto bad = packet;
    bad[20] = 5;  // frag_index == frag_count
    CHECK(parse_packet(bad).error == ParseError::BadFragment);
    bad = packet;
    bad[20] = 0;
    bad[22] = 0;  // frag_count 0
    CHECK(parse_packet(bad).error == ParseError::BadFragment);
}

TEST(rejects_any_flipped_bit) {
    const auto packet = encode_packet(sample_header(), sample_payload(64));
    // Flipping a bit in the CRC-covered fields that the other checks accept must still be caught.
    for (std::size_t byte : {4u, 8u, 16u, 29u, 40u, 93u}) {
        auto bad = packet;
        bad[byte] ^= 0x10;
        CHECK(parse_packet(bad).error == ParseError::BadCrc);
    }
}

TEST(packetizer_splits_50kb) {
    Packetizer packetizer(100, 7);
    const auto message = sample_payload(50'000);
    const auto packets = packetizer.packetize(MsgType::FullFrame, 42, message);
    CHECK_EQ(packets.size(), std::size_t{42});  // ceil(50000 / 1200)

    std::vector<std::uint8_t> joined;
    for (std::size_t i = 0; i < packets.size(); ++i) {
        const auto parsed = parse_packet(packets[i]);
        CHECK(parsed.error == ParseError::Ok);
        CHECK_EQ(parsed.header.seq, 100u + static_cast<std::uint32_t>(i));
        CHECK_EQ(parsed.header.msg_id, 7u);
        CHECK_EQ(parsed.header.frag_index, i);
        CHECK_EQ(parsed.header.frag_count, 42);
        CHECK_EQ(parsed.header.timestamp_us, 42u);
        joined.insert(joined.end(), parsed.payload.begin(), parsed.payload.end());
    }
    CHECK_EQ(parse_packet(packets.back()).header.payload_len, 800);
    CHECK(joined == message);

    const auto next = packetizer.packetize(MsgType::Telemetry, 43, sample_payload(10));
    CHECK_EQ(next.size(), std::size_t{1});
    CHECK_EQ(parse_packet(next[0]).header.seq, 142u);
    CHECK_EQ(parse_packet(next[0]).header.msg_id, 8u);
}

TEST(packetizer_empty_and_exact_multiple) {
    Packetizer packetizer;
    const auto empty = packetizer.packetize(MsgType::Tracks, 0, {});
    CHECK_EQ(empty.size(), std::size_t{1});
    CHECK_EQ(parse_packet(empty[0]).header.payload_len, 0);
    CHECK_EQ(packetizer.packetize(MsgType::Tracks, 0, sample_payload(2 * kMaxPayload)).size(),
             std::size_t{2});
}

TEST(packetizer_rejects_oversized_message) {
    Packetizer packetizer;
    bool threw = false;
    try {
        packetizer.packetize(MsgType::FullFrame, 0, std::vector<std::uint8_t>(kMaxMessage + 1));
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);
}
