#include "aegis/protocol.hpp"
#include "spoofer.hpp"
#include "test_runner.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

using namespace aegis;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusM = 6'378'137.0;
constexpr std::int32_t kLat = 399'000'000;  // 39.9 degrees, in 1e-7 degree
constexpr std::int32_t kLon = 328'000'000;

void put_i32(std::vector<std::uint8_t>& body, std::size_t at, std::int32_t value) {
    const auto u = static_cast<std::uint32_t>(value);
    for (int i = 0; i < 4; ++i) body[at + i] = static_cast<std::uint8_t>(u >> (8 * i));
}

std::int32_t get_i32(std::span<const std::uint8_t> body, std::size_t at) {
    std::uint32_t u = 0;
    for (int i = 0; i < 4; ++i) u |= std::uint32_t{body[at + i]} << (8 * i);
    return static_cast<std::int32_t>(u);
}

// A TELEMETRY body as the UAV sends it: frame, lat, lon, then five floats (filled with a pattern).
std::vector<std::uint8_t> telemetry_packet(MsgType type = MsgType::Telemetry,
                                           std::uint16_t frag_count = 1) {
    std::vector<std::uint8_t> body(32);
    for (std::size_t i = 0; i < body.size(); ++i) body[i] = static_cast<std::uint8_t>(i * 7);
    put_i32(body, 4, kLat);
    put_i32(body, 8, kLon);
    Header header;
    header.type = type;
    header.seq = 41;
    header.timestamp_us = 123'456'789;
    header.msg_id = 9;
    header.frag_count = frag_count;
    return encode_packet(header, body);
}

// Metres north and east the packet's position moved from kLat, kLon.
void moved_m(const std::vector<std::uint8_t>& datagram, double& north, double& east) {
    const ParsedPacket p = parse_packet(datagram);
    CHECK(p.error == ParseError::Ok);
    const double lat = get_i32(p.payload, 4) * 1e-7;
    const double lon = get_i32(p.payload, 8) * 1e-7;
    north = (lat - kLat * 1e-7) * kPi / 180.0 * kEarthRadiusM;
    east = (lon - kLon * 1e-7) * kPi / 180.0 * kEarthRadiusM * std::cos(kLat * 1e-7 * kPi / 180.0);
}

}  // namespace

TEST(an_idle_spoofer_changes_nothing) {
    Spoofer spoofer;
    auto datagram = telemetry_packet();
    const auto original = datagram;
    CHECK(!spoofer.apply(datagram, 1'000'000));
    CHECK(datagram == original);
    CHECK_EQ(spoofer.spoofed(), 0u);
}

TEST(an_offset_moves_the_position_along_the_bearing_and_the_crc_still_holds) {
    Spoofer spoofer;
    spoofer.configure({0.0, 90.0, 50.0}, 0);
    auto datagram = telemetry_packet();
    CHECK(spoofer.apply(datagram, 0));
    double north = 0, east = 0;
    moved_m(datagram, north, east);  // also checks that the packet parses (CRC)
    CHECK(std::abs(east - 50.0) < 0.02);
    CHECK(std::abs(north) < 0.02);
    CHECK_EQ(spoofer.spoofed(), 1u);

    // Everything but the position is as it was.
    const auto original = telemetry_packet();
    const ParsedPacket a = parse_packet(original);
    const ParsedPacket b = parse_packet(datagram);
    CHECK(b.header.seq == a.header.seq && b.header.timestamp_us == a.header.timestamp_us);
    CHECK(b.header.msg_id == a.header.msg_id && b.header.type == a.header.type);
    for (std::size_t i = 0; i < a.payload.size(); ++i) {
        if (i < 4 || i >= 12) CHECK(a.payload[i] == b.payload[i]);
    }
}

TEST(a_drift_grows_with_time) {
    Spoofer spoofer;
    spoofer.configure({3.0, 0.0, 0.0}, 1'000'000);
    CHECK(std::abs(spoofer.offset_m(3'000'000) - 6.0) < 1e-9);
    auto datagram = telemetry_packet();
    CHECK(spoofer.apply(datagram, 3'000'000));
    double north = 0, east = 0;
    moved_m(datagram, north, east);
    CHECK(std::abs(north - 6.0) < 0.02);
    CHECK(std::abs(east) < 0.02);
}

TEST(stopping_a_drift_keeps_the_offset_reached) {
    Spoofer spoofer;
    spoofer.configure({3.0, 225.0, 0.0}, 0);
    SpoofConfig stopped = spoofer.config(2'000'000);
    CHECK(std::abs(stopped.offset_m - 6.0) < 1e-9);
    stopped.drift_mps = 0.0;
    spoofer.configure(stopped, 2'000'000);
    CHECK(std::abs(spoofer.offset_m(10'000'000) - 6.0) < 1e-9);
    auto datagram = telemetry_packet();
    spoofer.apply(datagram, 10'000'000);
    double north = 0, east = 0;
    moved_m(datagram, north, east);
    CHECK(std::abs(north + 6.0 / std::sqrt(2.0)) < 0.02);
    CHECK(std::abs(east + 6.0 / std::sqrt(2.0)) < 0.02);
    // An offset of 0 and no drift is no spoofing at all.
    spoofer.configure({0.0, 225.0, 0.0}, 10'000'000);
    auto clean = telemetry_packet();
    CHECK(!spoofer.apply(clean, 11'000'000));
}

TEST(only_whole_valid_telemetry_packets_are_touched) {
    Spoofer spoofer;
    spoofer.configure({0.0, 90.0, 50.0}, 0);
    std::vector<std::vector<std::uint8_t>> others = {
        telemetry_packet(MsgType::Tracks),
        telemetry_packet(MsgType::BgKeyframe, 2),
        telemetry_packet(MsgType::Telemetry, 2),  // a fragment: not a telemetry message
        {1, 2, 3},
    };
    auto corrupt = telemetry_packet();
    corrupt.back() ^= 1;  // bad CRC: the receiver would drop it anyway
    others.push_back(corrupt);
    Header header;
    header.type = MsgType::Telemetry;
    const std::vector<std::uint8_t> short_body(12, 0);
    others.push_back(encode_packet(header, short_body));
    for (auto& datagram : others) {
        const auto original = datagram;
        CHECK(!spoofer.apply(datagram, 0));
        CHECK(datagram == original);
    }
    CHECK_EQ(spoofer.spoofed(), 0u);
}

TEST(bad_settings_are_rejected_and_change_nothing) {
    Spoofer spoofer;
    spoofer.configure({1.0, 45.0, 10.0}, 0);
    const SpoofConfig bad_values[] = {
        {-1.0, 0.0, 0.0}, {2000.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 361.0, 0.0},
        {0.0, 0.0, -1.0}, {0.0, 0.0, 2e7},    {NAN, 0.0, 0.0},  {0.0, 0.0, INFINITY},
    };
    for (const auto& bad : bad_values) {
        bool threw = false;
        try {
            spoofer.configure(bad, 0);
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        CHECK(threw);
        const SpoofConfig kept = spoofer.config(0);
        CHECK(kept.drift_mps == 1.0 && kept.bearing_deg == 45.0 && kept.offset_m == 10.0);
    }
}

TEST(positions_stay_in_range_with_a_huge_offset) {
    Spoofer spoofer;
    spoofer.configure({0.0, 0.0, 1e7}, 0);  // 10,000 km north of 39.9 degrees: past the pole
    auto datagram = telemetry_packet();
    CHECK(spoofer.apply(datagram, 0));
    const ParsedPacket p = parse_packet(datagram);
    CHECK(p.error == ParseError::Ok);
    CHECK(get_i32(p.payload, 4) == 900'000'000);
}
