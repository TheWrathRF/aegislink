#pragma once

#include <cstdint>
#include <random>

namespace aegis {

// A uniform double in [0, 1) from the top 53 bits. Unlike std::uniform_real_distribution, the
// result is the same with every standard library, so a seed reproduces a run on any compiler.
inline double uniform01(std::mt19937_64& rng) {
    return static_cast<double>(rng() >> 11) * 0x1.0p-53;
}

}  // namespace aegis
