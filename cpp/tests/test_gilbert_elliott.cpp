#include "gilbert_elliott.hpp"
#include "test_runner.hpp"

#include <cmath>
#include <random>
#include <stdexcept>

using namespace aegis;

struct LossStats {
    double loss;
    double mean_burst;
};

static LossStats measure(double loss, double burst, int draws, std::uint64_t seed) {
    GilbertElliott model;
    model.configure(loss, burst);
    std::mt19937_64 rng(seed);
    long long lost = 0;
    long long bursts = 0;
    bool previous = false;
    for (int i = 0; i < draws; ++i) {
        const bool dropped = model.drop(rng);
        if (dropped) ++lost;
        if (dropped && !previous) ++bursts;
        previous = dropped;
    }
    return {static_cast<double>(lost) / draws,
            bursts > 0 ? static_cast<double>(lost) / static_cast<double>(bursts) : 0.0};
}

static bool within(double measured, double expected, double relative) {
    return std::abs(measured - expected) <= relative * expected;
}

TEST(loss_rate_and_burst_length_match_over_a_million_draws) {
    const double cases[][2] = {{0.30, 4.0}, {0.05, 1.5}, {0.10, 10.0}, {0.60, 3.0}, {0.01, 1.0}};
    for (const auto& c : cases) {
        const LossStats s = measure(c[0], c[1], 1'000'000, 42);
        CHECK(within(s.loss, c[0], 0.02));
        CHECK(within(s.mean_burst, c[1], 0.02));
        if (!within(s.loss, c[0], 0.02) || !within(s.mean_burst, c[1], 0.02)) {
            std::cerr << "  loss " << c[0] << " burst " << c[1] << ": measured " << s.loss << " / "
                      << s.mean_burst << "\n";
        }
    }
}

TEST(burst_length_one_at_the_limit_alternates) {
    // L = 1 allows at most 50 % loss, and there the chain must alternate exactly.
    GilbertElliott model;
    model.configure(0.5, 1.0);
    std::mt19937_64 rng(1);
    for (int i = 0; i < 1000; ++i) CHECK_EQ(model.drop(rng), i % 2 == 1);
}

TEST(zero_loss_never_drops) {
    GilbertElliott model;
    model.configure(0.0, 5.0);
    std::mt19937_64 rng(2);
    for (int i = 0; i < 100'000; ++i) CHECK(!model.drop(rng));
}

TEST(transition_probabilities_follow_from_the_two_knobs) {
    GilbertElliott model;
    model.configure(0.2, 4.0);
    CHECK(std::abs(model.r() - 0.25) < 1e-12);
    CHECK(std::abs(model.p() - 0.0625) < 1e-12);
}

TEST(configure_rejects_impossible_settings) {
    const double bad[][2] = {{-0.1, 2.0}, {1.0, 2.0}, {0.2, 0.5}, {0.6, 1.0}, {0.81, 4.0},
                             {std::nan(""), 2.0}, {0.1, std::nan("")}, {0.1, INFINITY}};
    for (const auto& c : bad) {
        GilbertElliott model;
        bool threw = false;
        try {
            model.configure(c[0], c[1]);
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        CHECK(threw);
    }
}

TEST(configure_returns_to_the_good_state) {
    GilbertElliott model;
    model.configure(0.5, 1.0);
    std::mt19937_64 rng(3);
    CHECK(!model.drop(rng));  // good now, bad from the next packet on
    model.configure(0.0, 10.0);
    CHECK(!model.drop(rng));
}
