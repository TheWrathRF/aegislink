#include "test_runner.hpp"
#include "token_bucket.hpp"

#include <cstdint>
#include <cstdlib>

using namespace aegis;

// 50 kbps
constexpr double kRate = 6250.0;
constexpr double kDepth = 1230.0;

TEST(a_full_bucket_lets_one_packet_through_at_once) {
    TokenBucket bucket(kRate, kDepth, 0);
    CHECK_EQ(bucket.take(0, 1230), std::int64_t{0});
    // The next one waits until 1230 B have refilled: 1230 / 6250 s.
    CHECK_EQ(bucket.take(0, 1230), std::int64_t{196'800});
}

TEST(a_backlog_leaves_at_the_rate_and_never_above_it) {
    TokenBucket bucket(kRate, kDepth, 0);
    double sent = 0;
    std::int64_t last = 0;
    for (int i = 0; i < 200; ++i) {
        const std::int64_t leave = bucket.take(0, 1000);
        CHECK(leave >= last);
        sent += 1000;
        CHECK(sent <= kDepth + kRate * static_cast<double>(leave) / 1e6 + 1e-6);
        last = leave;
    }
    // 200,000 B, of which the first 1230 B were already in the bucket.
    CHECK(std::llabs(last - 31'803'200) <= 1);
}

TEST(an_idle_bucket_fills_only_up_to_its_depth) {
    TokenBucket bucket(kRate, kDepth, 0);
    bucket.take(0, 1230);
    CHECK_EQ(bucket.take(10'000'000, 1230), std::int64_t{10'000'000});
    CHECK_EQ(bucket.take(10'000'000, 1230), std::int64_t{10'196'800});
}

TEST(wait_does_not_take_tokens) {
    TokenBucket bucket(kRate, kDepth, 0);
    CHECK_EQ(bucket.wait_us(0, 1230), std::int64_t{0});
    CHECK_EQ(bucket.wait_us(0, 1230), std::int64_t{0});
    bucket.take(0, 1230);
    CHECK_EQ(bucket.wait_us(0, 1230), std::int64_t{196'800});
    CHECK_EQ(bucket.wait_us(100'000, 1230), std::int64_t{96'800});
}

TEST(traffic_at_the_rate_never_waits) {
    TokenBucket bucket(kRate, kDepth, 0);
    for (std::int64_t t = 0; t < 10'000'000; t += 100'000) {
        CHECK_EQ(bucket.take(t, 625), t);
    }
}
