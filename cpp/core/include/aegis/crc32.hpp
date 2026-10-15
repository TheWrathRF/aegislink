#pragma once

#include <cstdint>
#include <span>

namespace aegis {

// CRC-32 (IEEE 802.3, the same as zlib.crc32). Pass a previous result as crc to continue it.
std::uint32_t crc32(std::span<const std::uint8_t> data, std::uint32_t crc = 0);

}  // namespace aegis
