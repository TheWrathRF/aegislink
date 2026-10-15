// The AegisLink wire format: a 30-byte little-endian header, then up to 1200 bytes of payload.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace aegis {

inline constexpr std::uint16_t kMagic = 0xAE61;
inline constexpr std::uint8_t kVersion = 1;
inline constexpr std::size_t kHeaderSize = 30;
// Keeps a datagram under the 1280-byte IPv6 minimum MTU, so it is never IP-fragmented.
inline constexpr std::size_t kMaxPayload = 1200;
inline constexpr std::size_t kMaxDatagram = kHeaderSize + kMaxPayload;
inline constexpr std::size_t kMaxMessage = kMaxPayload * 0xFFFF;  // frag_count is a u16

enum class MsgType : std::uint8_t {
    Telemetry = 1,
    Tracks = 2,
    RoiPatch = 3,
    BgKeyframe = 4,
    FullFrame = 5,
    ReceiverReport = 6,
};

struct Header {
    MsgType type = MsgType::Telemetry;
    std::uint32_t seq = 0;           // per-link packet counter, for loss measurement
    std::uint64_t timestamp_us = 0;  // UAV capture time
    std::uint32_t msg_id = 0;        // shared by all fragments of one message
    std::uint16_t frag_index = 0;
    std::uint16_t frag_count = 1;
    std::uint16_t payload_len = 0;   // set by encode_packet
};

enum class ParseError {
    Ok,
    TooShort,
    BadMagic,
    BadVersion,
    BadType,
    BadLength,
    BadFragment,
    BadCrc,
};

const char* to_string(ParseError error);

struct ParsedPacket {
    ParseError error = ParseError::TooShort;
    Header header;
    std::span<const std::uint8_t> payload;  // points into the parsed datagram
};

// Throws std::invalid_argument for a payload over kMaxPayload or invalid fragment fields.
std::vector<std::uint8_t> encode_packet(const Header& header, std::span<const std::uint8_t> payload);

// Checks every length before reading; a packet is only Ok if all fields and the CRC are valid.
ParsedPacket parse_packet(std::span<const std::uint8_t> datagram);

// Splits messages into datagrams, numbering packets (seq) and messages (msg_id) for one link.
class Packetizer {
public:
    explicit Packetizer(std::uint32_t first_seq = 0, std::uint32_t first_msg_id = 0);

    // Throws std::invalid_argument for a message over kMaxMessage.
    std::vector<std::vector<std::uint8_t>> packetize(MsgType type, std::uint64_t timestamp_us,
                                                     std::span<const std::uint8_t> message);

    std::uint32_t next_seq() const { return next_seq_; }
    std::uint32_t next_msg_id() const { return next_msg_id_; }

private:
    std::uint32_t next_seq_;
    std::uint32_t next_msg_id_;
};

}  // namespace aegis
