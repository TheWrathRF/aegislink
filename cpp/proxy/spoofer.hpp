#pragma once

#include <cstdint>
#include <vector>

namespace aegis {

struct SpoofConfig {
    double drift_mps = 0.0;    // the offset grows this fast
    double bearing_deg = 0.0;  // where the reported position is moved to (0 = north, 90 = east)
    double offset_m = 0.0;     // how far, at the moment the settings are made
};

// A man-in-the-middle on the telemetry: it moves the position in every TELEMETRY message by an
// offset along a bearing and recomputes the CRC, so the receiver accepts the packet (a CRC
// detects noise, not an attacker). Setting the offset at once is a jump; a drift makes it grow.
class Spoofer {
public:
    // Throws std::invalid_argument and keeps the old settings if any value is out of range.
    void configure(const SpoofConfig& config, std::int64_t now_us);
    // The settings with the offset reached by now_us.
    SpoofConfig config(std::int64_t now_us) const;
    double offset_m(std::int64_t now_us) const;

    // Rewrites the datagram if it is a valid, unfragmented TELEMETRY packet and the offset is
    // not 0; returns whether it did.
    bool apply(std::vector<std::uint8_t>& datagram, std::int64_t now_us);
    std::uint64_t spoofed() const { return spoofed_; }

private:
    SpoofConfig config_;
    std::int64_t set_us_ = 0;  // when config_.offset_m was the offset
    std::uint64_t spoofed_ = 0;
};

}  // namespace aegis
