#include "aegis/crc32.hpp"
#include "test_runner.hpp"

#include <span>
#include <string_view>

static std::span<const std::uint8_t> bytes_of(std::string_view s) {
    return {reinterpret_cast<const std::uint8_t*>(s.data()), s.size()};
}

TEST(empty_input_is_zero) {
    CHECK_EQ(aegis::crc32({}), 0u);
}

TEST(standard_check_value) {
    // The catalogued check value of CRC-32/ISO-HDLC (the zlib / Ethernet CRC).
    CHECK_EQ(aegis::crc32(bytes_of("123456789")), 0xCBF43926u);
}

TEST(known_sentence) {
    CHECK_EQ(aegis::crc32(bytes_of("The quick brown fox jumps over the lazy dog")), 0x414FA339u);
}

TEST(chaining_equals_one_pass) {
    const std::string_view text = "The quick brown fox jumps over the lazy dog";
    const auto first = aegis::crc32(bytes_of(text.substr(0, 10)));
    CHECK_EQ(aegis::crc32(bytes_of(text.substr(10)), first), aegis::crc32(bytes_of(text)));
}
