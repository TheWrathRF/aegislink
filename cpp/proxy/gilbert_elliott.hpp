#pragma once

#include <random>

namespace aegis {

// Bursty packet loss as a two-state Markov chain: the good state passes every packet, the bad
// state drops every packet. It is set from the two numbers a user thinks in, the long-run loss
// rate and the mean burst length L (consecutive packets lost):
//   r = 1 / L                      (bad -> good per packet)
//   p = r * loss / (1 - loss)      (good -> bad per packet)
class GilbertElliott {
public:
    // loss in [0, 1), mean_burst >= 1 packet. A mean burst of L allows at most L / (L + 1) loss
    // (otherwise p > 1). Throws std::invalid_argument for anything else. Starts in the good state.
    void configure(double loss, double mean_burst);

    // The fate of one packet; the chain then moves one step.
    bool drop(std::mt19937_64& rng);

    double p() const { return p_; }
    double r() const { return r_; }

private:
    double p_ = 0.0;
    double r_ = 1.0;
    bool bad_ = false;
};

}  // namespace aegis
