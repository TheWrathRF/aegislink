#include "gilbert_elliott.hpp"

#include "random.hpp"

#include <cmath>
#include <cstdio>
#include <stdexcept>

namespace aegis {

void GilbertElliott::configure(double loss, double mean_burst) {
    if (!std::isfinite(mean_burst) || mean_burst < 1.0) {
        throw std::invalid_argument("burst must be at least 1 packet");
    }
    const double max_loss = mean_burst / (mean_burst + 1.0);
    if (!(loss >= 0.0 && loss <= max_loss)) {
        char message[128];
        std::snprintf(message, sizeof(message),
                      "loss must be between 0 and %g %% for a mean burst of %g packets",
                      max_loss * 100.0, mean_burst);
        throw std::invalid_argument(message);
    }
    r_ = 1.0 / mean_burst;
    p_ = r_ * loss / (1.0 - loss);
    bad_ = false;
}

bool GilbertElliott::drop(std::mt19937_64& rng) {
    const bool lost = bad_;
    const double u = uniform01(rng);
    bad_ = bad_ ? (u >= r_) : (u < p_);
    return lost;
}

}  // namespace aegis
