#pragma once

#include "gilbert_elliott.hpp"
#include "spoofer.hpp"
#include "token_bucket.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <random>

namespace aegis {

struct LinkConfig {
    double loss = 0.0;        // long-run fraction of packets lost, see GilbertElliott
    double mean_burst = 1.0;  // mean number of consecutive packets lost
    double kbps = 0.0;        // rate limit; 0 means unlimited
    double queue_ms = 250.0;  // longest a packet may wait for the rate limit before it is dropped
    double delay_ms = 0.0;    // fixed one-way delay
    double jitter_ms = 0.0;   // extra delay, uniform in [0, jitter_ms); packets may reorder
};

enum class Fate { Forward, DropBlackout, DropLoss, DropQueue };

struct LinkStats {
    std::uint64_t in_packets = 0;
    std::uint64_t in_bytes = 0;
    std::uint64_t out_packets = 0;
    std::uint64_t out_bytes = 0;
    std::uint64_t drop_blackout = 0;
    std::uint64_t drop_loss = 0;
    std::uint64_t drop_queue = 0;
    std::uint64_t send_errors = 0;
};

struct Verdict {
    Fate fate;
    std::int64_t release_us;  // when to send the packet on (Forward only)
};

// The impairments of one direction of the radio link, applied to each packet in this order:
// blackout, burst loss, rate limit with a bounded queue (drop-tail), delay and jitter.
// It only decides each packet's fate; the caller keeps the packets and sends them on time, and
// passes the forwarded ones through the link's spoofer.
class Link {
public:
    explicit Link(std::uint64_t seed);

    // Throws std::invalid_argument and keeps the old settings if any value is out of range.
    void configure(const LinkConfig& config, std::int64_t now_us);
    const LinkConfig& config() const { return config_; }

    // Drops every packet for the next duration_ms; 0 ends a blackout early.
    void blackout(std::int64_t now_us, double duration_ms);
    double blackout_remaining_ms(std::int64_t now_us) const;

    Verdict admit(std::int64_t now_us, std::size_t bytes);
    void count_sent(std::size_t bytes, bool ok);
    const LinkStats& stats() const { return stats_; }

    Spoofer& spoofer() { return spoofer_; }
    const Spoofer& spoofer() const { return spoofer_; }

private:
    LinkConfig config_;
    Spoofer spoofer_;
    GilbertElliott loss_;
    std::optional<TokenBucket> bucket_;  // empty when the rate is unlimited
    std::int64_t blackout_until_us_ = 0;
    std::mt19937_64 rng_;
    LinkStats stats_;
};

}  // namespace aegis
