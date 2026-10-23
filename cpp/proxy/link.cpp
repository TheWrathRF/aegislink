#include "link.hpp"

#include "aegis/protocol.hpp"
#include "random.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace aegis {

namespace {

void check_range(const char* name, double value, double max) {
    if (!(value >= 0.0 && value <= max)) {
        throw std::invalid_argument(std::string(name) + " must be between 0 and " +
                                    std::to_string(static_cast<long long>(max)));
    }
}

std::int64_t ms_to_us(double ms) { return static_cast<std::int64_t>(ms * 1000.0); }

}  // namespace

Link::Link(std::uint64_t seed) : rng_(seed) {}

void Link::configure(const LinkConfig& config, std::int64_t now_us) {
    check_range("kbps", config.kbps, 10'000'000.0);
    check_range("queue_ms", config.queue_ms, 60'000.0);
    check_range("delay_ms", config.delay_ms, 10'000.0);
    check_range("jitter_ms", config.jitter_ms, 10'000.0);

    GilbertElliott loss = loss_;
    if (config.loss != config_.loss || config.mean_burst != config_.mean_burst) {
        loss.configure(config.loss, config.mean_burst);
    }
    // A new bucket starts full, so the existing one only changes its rate: a fresh one would let
    // a burst above the rate through, and new packets would leave before the queued ones.
    if (config.kbps != config_.kbps) {
        const double rate = config.kbps * 1000.0 / 8.0;
        if (config.kbps == 0.0) {
            bucket_.reset();
        } else if (bucket_) {
            bucket_->set_rate(rate, now_us);
        } else {
            // One full-size datagram deep: the largest packet can always leave eventually.
            bucket_.emplace(rate, static_cast<double>(kMaxDatagram), now_us);
        }
    }
    loss_ = loss;
    config_ = config;
}

void Link::blackout(std::int64_t now_us, double duration_ms) {
    check_range("blackout_ms", duration_ms, 3'600'000.0);
    blackout_until_us_ = now_us + ms_to_us(duration_ms);
}

double Link::blackout_remaining_ms(std::int64_t now_us) const {
    return static_cast<double>(std::max<std::int64_t>(blackout_until_us_ - now_us, 0)) / 1000.0;
}

Verdict Link::admit(std::int64_t now_us, std::size_t bytes) {
    ++stats_.in_packets;
    stats_.in_bytes += bytes;

    if (now_us < blackout_until_us_) {
        ++stats_.drop_blackout;
        return {Fate::DropBlackout, 0};
    }
    if (loss_.drop(rng_)) {
        ++stats_.drop_loss;
        return {Fate::DropLoss, 0};
    }
    std::int64_t depart_us = now_us;
    if (bucket_) {
        if (bucket_->wait_us(now_us, bytes) > ms_to_us(config_.queue_ms)) {
            ++stats_.drop_queue;
            return {Fate::DropQueue, 0};
        }
        depart_us = bucket_->take(now_us, bytes);
    }
    // Packets queued before the rate limit was lifted still leave first.
    depart_us = std::max(depart_us, last_depart_us_);
    last_depart_us_ = depart_us;
    const double delay_ms = config_.delay_ms + uniform01(rng_) * config_.jitter_ms;
    return {Fate::Forward, depart_us + ms_to_us(delay_ms)};
}

void Link::count_sent(std::size_t bytes, bool ok) {
    if (ok) {
        ++stats_.out_packets;
        stats_.out_bytes += bytes;
    } else {
        ++stats_.send_errors;
    }
}

}  // namespace aegis
