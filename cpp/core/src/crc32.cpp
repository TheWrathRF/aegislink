#include "aegis/crc32.hpp"

#include <array>

namespace aegis {

namespace {

constexpr std::uint32_t kPolynomial = 0xEDB88320u;  // 0x04C11DB7 bit-reversed

constexpr std::array<std::uint32_t, 256> make_table() {
    std::array<std::uint32_t, 256> table{};
    for (std::uint32_t i = 0; i < 256; ++i) {
        std::uint32_t c = i;
        for (int bit = 0; bit < 8; ++bit) c = (c & 1u) ? (kPolynomial ^ (c >> 1)) : (c >> 1);
        table[i] = c;
    }
    return table;
}

constexpr auto kTable = make_table();

}  // namespace

std::uint32_t crc32(std::span<const std::uint8_t> data, std::uint32_t crc) {
    crc = ~crc;
    for (std::uint8_t byte : data) crc = kTable[(crc ^ byte) & 0xFFu] ^ (crc >> 8);
    return ~crc;
}

}  // namespace aegis
