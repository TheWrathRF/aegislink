// Rebuilds messages from fragments that may arrive missing, duplicated or out of order.
#pragma once

#include "aegis/protocol.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <unordered_map>
#include <vector>

namespace aegis {

struct Message {
    MsgType type = MsgType::Telemetry;
    std::uint32_t msg_id = 0;
    std::uint64_t timestamp_us = 0;
    std::vector<std::uint8_t> data;
};

struct ReassemblerStats {
    std::uint64_t completed = 0;
    std::uint64_t duplicates = 0;    // fragment already received, or its message already delivered
    std::uint64_t inconsistent = 0;  // type or frag_count disagree with earlier fragments
    std::uint64_t expired = 0;       // incomplete when the timeout ran out
    std::uint64_t evicted = 0;       // dropped to stay within max_pending
};

class Reassembler {
public:
    // Times are the receiver's own clock in microseconds, passed in so tests control time.
    explicit Reassembler(std::uint64_t timeout_us = 200'000, std::size_t max_pending = 64);

    // Returns the message once its last missing fragment arrives.
    std::optional<Message> push(const Header& header, std::span<const std::uint8_t> payload,
                                std::uint64_t now_us);

    // Drops incomplete messages whose first fragment is older than the timeout.
    void expire(std::uint64_t now_us);

    std::size_t pending() const { return partials_.size(); }
    const ReassemblerStats& stats() const { return stats_; }

private:
    struct Partial {
        MsgType type;
        std::uint64_t timestamp_us;
        std::uint64_t first_seen_us;
        std::uint16_t received = 0;
        std::vector<std::vector<std::uint8_t>> fragments;  // empty until that fragment arrives
        std::vector<bool> have;
    };

    void evict_oldest();

    std::uint64_t timeout_us_;
    std::size_t max_pending_;
    std::unordered_map<std::uint32_t, Partial> partials_;
    // msg_id -> completion time. Remembered for one timeout so late duplicates are not
    // mistaken for the start of a new message.
    std::unordered_map<std::uint32_t, std::uint64_t> recently_completed_;
    ReassemblerStats stats_;
};

}  // namespace aegis
