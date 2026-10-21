#include "spoofer.hpp"

#include "aegis/protocol.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace aegis {

namespace {

constexpr double kPi = 3.14159265358979323846;
// The same flat-earth model as the Python side (common/geo.py), which is exact enough over the
// few kilometres a spoofed position moves.
constexpr double kEarthRadiusM = 6'378'137.0;
constexpr std::size_t kTelemetryBody = 32;
constexpr std::size_t kLatAt = 4;  // after the u32 frame number
constexpr std::size_t kLonAt = 8;

void check_range(const char* name, double value, double max) {
    if (!(value >= 0.0 && value <= max)) {
        throw std::invalid_argument(std::string(name) + " must be between 0 and " +
                                    std::to_string(static_cast<long long>(max)));
    }
}

std::int32_t get_i32(const std::vector<std::uint8_t>& body, std::size_t at) {
    std::uint32_t u = 0;
    for (std::size_t i = 0; i < 4; ++i) u |= std::uint32_t{body[at + i]} << (8 * i);
    return static_cast<std::int32_t>(u);
}

void put_i32(std::vector<std::uint8_t>& body, std::size_t at, std::int32_t value) {
    const auto u = static_cast<std::uint32_t>(value);
    for (std::size_t i = 0; i < 4; ++i) body[at + i] = static_cast<std::uint8_t>(u >> (8 * i));
}

// Degrees -> 1e-7 degrees, kept inside [-limit, limit].
std::int32_t to_e7(double degrees, double limit) {
    return static_cast<std::int32_t>(std::lround(std::clamp(degrees, -limit, limit) * 1e7));
}

}  // namespace

void Spoofer::configure(const SpoofConfig& config, std::int64_t now_us) {
    check_range("spoof_drift_mps", config.drift_mps, 1000.0);
    check_range("spoof_bearing_deg", config.bearing_deg, 360.0);
    check_range("spoof_offset_m", config.offset_m, 10'000'000.0);
    config_ = config;
    set_us_ = now_us;
}

SpoofConfig Spoofer::config(std::int64_t now_us) const {
    SpoofConfig now = config_;
    now.offset_m = offset_m(now_us);
    return now;
}

double Spoofer::offset_m(std::int64_t now_us) const {
    const double elapsed_s = static_cast<double>(std::max<std::int64_t>(now_us - set_us_, 0)) / 1e6;
    return config_.offset_m + config_.drift_mps * elapsed_s;
}

bool Spoofer::apply(std::vector<std::uint8_t>& datagram, std::int64_t now_us) {
    const double offset = offset_m(now_us);
    if (offset == 0.0) return false;
    const ParsedPacket packet = parse_packet(datagram);
    if (packet.error != ParseError::Ok || packet.header.type != MsgType::Telemetry ||
        packet.header.frag_count != 1 || packet.payload.size() != kTelemetryBody) {
        return false;
    }
    std::vector<std::uint8_t> body(packet.payload.begin(), packet.payload.end());
    const double lat = get_i32(body, kLatAt) * 1e-7;
    const double lon = get_i32(body, kLonAt) * 1e-7;
    const double bearing = config_.bearing_deg * kPi / 180.0;
    const double north = offset * std::cos(bearing);
    const double east = offset * std::sin(bearing);
    const double degrees_per_m = 180.0 / (kPi * kEarthRadiusM);
    put_i32(body, kLatAt, to_e7(lat + north * degrees_per_m, 90.0));
    put_i32(body, kLonAt, to_e7(lon + east * degrees_per_m / std::cos(lat * kPi / 180.0), 180.0));
    datagram = encode_packet(packet.header, body);
    ++spoofed_;
    return true;
}

}  // namespace aegis
