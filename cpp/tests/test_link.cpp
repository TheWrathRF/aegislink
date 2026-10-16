#include "link.hpp"
#include "test_runner.hpp"

#include <cstdint>
#include <stdexcept>

using namespace aegis;

TEST(a_clean_link_forwards_everything_at_once) {
    Link link(1);
    for (std::int64_t t = 0; t < 1000; ++t) {
        const auto v = link.admit(t, 100);
        CHECK(v.fate == Fate::Forward);
        CHECK_EQ(v.release_us, t);
    }
    CHECK_EQ(link.stats().in_packets, 1000u);
    CHECK_EQ(link.stats().in_bytes, 100'000u);
}

TEST(delay_and_jitter_stay_within_their_bounds) {
    Link link(2);
    LinkConfig config;
    config.delay_ms = 50;
    config.jitter_ms = 20;
    link.configure(config, 0);
    std::int64_t lowest = 1'000'000;
    std::int64_t highest = 0;
    for (int i = 0; i < 10'000; ++i) {
        const auto v = link.admit(0, 100);
        lowest = v.release_us < lowest ? v.release_us : lowest;
        highest = v.release_us > highest ? v.release_us : highest;
    }
    CHECK(lowest >= 50'000 && lowest < 50'100);
    CHECK(highest < 70'000 && highest > 69'900);
}

TEST(loss_drops_are_counted_as_loss) {
    Link link(3);
    LinkConfig config;
    config.loss = 0.3;
    config.mean_burst = 4;
    link.configure(config, 0);
    int forwarded = 0;
    for (int i = 0; i < 100'000; ++i) forwarded += link.admit(i, 100).fate == Fate::Forward;
    CHECK_EQ(link.stats().drop_loss, static_cast<std::uint64_t>(100'000 - forwarded));
    CHECK(link.stats().drop_loss > 28'000 && link.stats().drop_loss < 32'000);
}

TEST(rate_limit_queues_then_drops_tail) {
    Link link(4);
    LinkConfig config;
    config.kbps = 50;  // 6250 B/s: a 1250 B packet every 200 ms
    config.queue_ms = 500;
    link.configure(config, 0);
    // All arrive at once: the first leaves at once (full bucket), the next ones 200, 400 ms
    // later; waiting 600 ms is more than the queue allows.
    const auto first = link.admit(0, 1230);
    CHECK(first.fate == Fate::Forward && first.release_us == 0);
    const auto second = link.admit(0, 1250);
    CHECK(second.fate == Fate::Forward && second.release_us == 200'000);
    const auto third = link.admit(0, 1250);
    CHECK(third.fate == Fate::Forward && third.release_us == 400'000);
    CHECK(link.admit(0, 1250).fate == Fate::DropQueue);
    CHECK_EQ(link.stats().drop_queue, 1u);
    // Once the queue has drained a little there is room again.
    const auto later = link.admit(150'000, 1250);
    CHECK(later.fate == Fate::Forward && later.release_us == 600'000);
}

TEST(blackout_drops_everything_until_it_ends) {
    Link link(5);
    CHECK(link.admit(999'999, 10).fate == Fate::Forward);
    link.blackout(1'000'000, 500);
    CHECK(link.admit(1'000'000, 10).fate == Fate::DropBlackout);
    CHECK(link.admit(1'499'999, 10).fate == Fate::DropBlackout);
    CHECK_EQ(link.blackout_remaining_ms(1'250'000), 250.0);
    CHECK(link.admit(1'500'000, 10).fate == Fate::Forward);
    CHECK_EQ(link.stats().drop_blackout, 2u);
    link.blackout(2'000'000, 10'000);
    link.blackout(2'000'000, 0);  // ends it early
    CHECK(link.admit(2'000'001, 10).fate == Fate::Forward);
}

TEST(blackout_comes_before_loss_and_rate) {
    Link link(6);
    LinkConfig config;
    config.loss = 0.5;
    config.mean_burst = 1;
    config.kbps = 1;
    link.configure(config, 0);
    link.blackout(0, 1000);
    for (int i = 0; i < 100; ++i) CHECK(link.admit(i, 1000).fate == Fate::DropBlackout);
    CHECK_EQ(link.stats().drop_loss, 0u);
    CHECK_EQ(link.stats().drop_queue, 0u);
}

TEST(invalid_settings_are_rejected_and_change_nothing) {
    Link link(7);
    LinkConfig good;
    good.loss = 0.1;
    good.mean_burst = 2;
    good.kbps = 100;
    link.configure(good, 0);
    const LinkConfig bad_values[] = {
        [] { LinkConfig c; c.loss = 0.9; c.mean_burst = 1; return c; }(),
        [] { LinkConfig c; c.kbps = -1; return c; }(),
        [] { LinkConfig c; c.delay_ms = -5; return c; }(),
        [] { LinkConfig c; c.jitter_ms = -1; return c; }(),
        [] { LinkConfig c; c.queue_ms = -1; return c; }(),
        [] { LinkConfig c; c.delay_ms = 1e300; return c; }(),
    };
    for (const auto& bad : bad_values) {
        bool threw = false;
        try {
            link.configure(bad, 0);
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        CHECK(threw);
        CHECK(link.config().loss == 0.1 && link.config().kbps == 100);
    }
}

TEST(changing_the_delay_keeps_the_rate_limit_backlog) {
    // Otherwise every slider move would refill the bucket and let a burst above the rate through.
    Link link(8);
    LinkConfig config;
    config.kbps = 50;
    config.queue_ms = 10'000;
    link.configure(config, 0);
    link.admit(0, 1230);
    CHECK_EQ(link.admit(0, 1250).release_us, std::int64_t{200'000});
    config.delay_ms = 10;
    link.configure(config, 0);
    CHECK_EQ(link.admit(0, 1250).release_us, std::int64_t{400'000 + 10'000});
}

TEST(same_seed_same_fates) {
    LinkConfig config;
    config.loss = 0.2;
    config.mean_burst = 3;
    config.jitter_ms = 5;
    Link a(99);
    Link b(99);
    a.configure(config, 0);
    b.configure(config, 0);
    for (int i = 0; i < 10'000; ++i) {
        const auto va = a.admit(i, 100);
        const auto vb = b.admit(i, 100);
        CHECK(va.fate == vb.fate && va.release_us == vb.release_us);
    }
}
