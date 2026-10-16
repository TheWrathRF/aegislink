#include "token_bucket.hpp"

#include <algorithm>
#include <cmath>

namespace aegis {

TokenBucket::TokenBucket(double bytes_per_s, double depth_bytes, std::int64_t now_us)
    : rate_(bytes_per_s), depth_(depth_bytes), tokens_(depth_bytes), last_us_(now_us) {}

double TokenBucket::available(std::int64_t now_us) const {
    const double elapsed_us = static_cast<double>(std::max<std::int64_t>(now_us - last_us_, 0));
    return std::min(depth_, tokens_ + elapsed_us * rate_ / 1e6);
}

std::int64_t TokenBucket::wait_us(std::int64_t now_us, std::size_t bytes) const {
    const double missing = static_cast<double>(bytes) - available(now_us);
    if (missing <= 0.0) return 0;
    return static_cast<std::int64_t>(std::ceil(missing * 1e6 / rate_));
}

std::int64_t TokenBucket::take(std::int64_t now_us, std::size_t bytes) {
    const std::int64_t wait = wait_us(now_us, bytes);
    tokens_ = available(now_us) - static_cast<double>(bytes);
    last_us_ = std::max(last_us_, now_us);
    return now_us + wait;
}

}  // namespace aegis
