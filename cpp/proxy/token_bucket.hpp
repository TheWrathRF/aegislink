#pragma once

#include <cstddef>
#include <cstdint>

namespace aegis {

// A token bucket that doubles as a FIFO queue. Tokens (bytes) refill at a fixed rate up to the
// bucket's depth. A packet that finds too few tokens takes them anyway, leaving the bucket in
// debt, and may leave only once the debt is paid off. So packets leave in arrival order, and
// over any interval T at most depth + rate * T bytes leave.
class TokenBucket {
public:
    // Starts full at now_us.
    TokenBucket(double bytes_per_s, double depth_bytes, std::int64_t now_us);

    // How long a packet of `bytes` arriving at now_us would wait. Takes nothing.
    std::int64_t wait_us(std::int64_t now_us, std::size_t bytes) const;

    // Takes the packet's tokens and returns the time at which it may leave.
    std::int64_t take(std::int64_t now_us, std::size_t bytes);

    // Changes the rate. A debt is kept as the time it takes to pay off, so the packets already
    // queued still leave first and the next one waits for them at the new rate.
    void set_rate(double bytes_per_s, std::int64_t now_us);

private:
    double available(std::int64_t now_us) const;

    double rate_;
    double depth_;
    double tokens_;
    std::int64_t last_us_;
};

}  // namespace aegis
