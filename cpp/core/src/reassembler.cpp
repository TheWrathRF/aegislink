#include "aegis/reassembler.hpp"

#include <iterator>

namespace aegis {

Reassembler::Reassembler(std::uint64_t timeout_us, std::size_t max_pending)
    : timeout_us_(timeout_us), max_pending_(max_pending == 0 ? 1 : max_pending) {}

std::optional<Message> Reassembler::push(const Header& header, std::span<const std::uint8_t> payload,
                                         std::uint64_t now_us) {
    expire(now_us);

    if (header.frag_count == 0 || header.frag_index >= header.frag_count) {
        ++stats_.inconsistent;
        return std::nullopt;
    }
    if (recently_completed_.contains(header.msg_id)) {
        ++stats_.duplicates;
        return std::nullopt;
    }

    auto it = partials_.find(header.msg_id);
    if (it == partials_.end()) {
        if (partials_.size() >= max_pending_) evict_oldest();
        Partial fresh{header.type, header.timestamp_us, now_us, 0,
                      std::vector<std::vector<std::uint8_t>>(header.frag_count),
                      std::vector<bool>(header.frag_count, false)};
        it = partials_.emplace(header.msg_id, std::move(fresh)).first;
    }

    Partial& partial = it->second;
    if (partial.type != header.type || partial.fragments.size() != header.frag_count) {
        ++stats_.inconsistent;
        return std::nullopt;
    }
    if (partial.have[header.frag_index]) {
        ++stats_.duplicates;
        return std::nullopt;
    }
    partial.fragments[header.frag_index].assign(payload.begin(), payload.end());
    partial.have[header.frag_index] = true;
    if (++partial.received < header.frag_count) return std::nullopt;

    Message message{partial.type, header.msg_id, partial.timestamp_us, {}};
    std::size_t total = 0;
    for (const auto& fragment : partial.fragments) total += fragment.size();
    message.data.reserve(total);
    for (const auto& fragment : partial.fragments) {
        message.data.insert(message.data.end(), fragment.begin(), fragment.end());
    }
    partials_.erase(it);
    recently_completed_[header.msg_id] = now_us;
    ++stats_.completed;
    return message;
}

void Reassembler::expire(std::uint64_t now_us) {
    for (auto it = partials_.begin(); it != partials_.end();) {
        if (now_us > it->second.first_seen_us + timeout_us_) {
            it = partials_.erase(it);
            ++stats_.expired;
        } else {
            ++it;
        }
    }
    for (auto it = recently_completed_.begin(); it != recently_completed_.end();) {
        it = (now_us > it->second + timeout_us_) ? recently_completed_.erase(it) : std::next(it);
    }
}

void Reassembler::evict_oldest() {
    auto oldest = partials_.begin();
    for (auto it = partials_.begin(); it != partials_.end(); ++it) {
        if (it->second.first_seen_us < oldest->second.first_seen_us) oldest = it;
    }
    partials_.erase(oldest);
    ++stats_.evicted;
}

}  // namespace aegis
