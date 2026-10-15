#include "aegis/protocol.hpp"

#include "aegis/crc32.hpp"

#include <algorithm>
#include <stdexcept>

namespace aegis {

namespace {

// Byte offsets of the header fields.
constexpr std::size_t kOffMagic = 0;
constexpr std::size_t kOffVersion = 2;
constexpr std::size_t kOffType = 3;
constexpr std::size_t kOffSeq = 4;
constexpr std::size_t kOffTimestamp = 8;
constexpr std::size_t kOffMsgId = 16;
constexpr std::size_t kOffFragIndex = 20;
constexpr std::size_t kOffFragCount = 22;
constexpr std::size_t kOffPayloadLen = 24;
constexpr std::size_t kOffCrc = 26;
static_assert(kOffCrc + 4 == kHeaderSize);

// Explicit shifts make the byte order little-endian on any host.
template <typename T>
void put_le(std::uint8_t* out, T value) {
    for (std::size_t i = 0; i < sizeof(T); ++i) out[i] = static_cast<std::uint8_t>(value >> (8 * i));
}

template <typename T>
T get_le(const std::uint8_t* in) {
    T value = 0;
    for (std::size_t i = 0; i < sizeof(T); ++i) value = static_cast<T>(value | (static_cast<T>(in[i]) << (8 * i)));
    return value;
}

bool valid_type(std::uint8_t type) {
    return type >= static_cast<std::uint8_t>(MsgType::Telemetry) &&
           type <= static_cast<std::uint8_t>(MsgType::ReceiverReport);
}

// The CRC covers the header up to the crc field, then the payload.
std::uint32_t packet_crc(std::span<const std::uint8_t> packet) {
    const auto crc = crc32(packet.first(kOffCrc));
    return crc32(packet.subspan(kHeaderSize), crc);
}

}  // namespace

const char* to_string(ParseError error) {
    switch (error) {
        case ParseError::Ok: return "ok";
        case ParseError::TooShort: return "too short";
        case ParseError::BadMagic: return "bad magic";
        case ParseError::BadVersion: return "bad version";
        case ParseError::BadType: return "bad type";
        case ParseError::BadLength: return "bad length";
        case ParseError::BadFragment: return "bad fragment";
        case ParseError::BadCrc: return "bad crc";
    }
    return "unknown";
}

std::vector<std::uint8_t> encode_packet(const Header& header, std::span<const std::uint8_t> payload) {
    if (payload.size() > kMaxPayload) throw std::invalid_argument("payload larger than kMaxPayload");
    if (header.frag_count == 0 || header.frag_index >= header.frag_count) {
        throw std::invalid_argument("frag_index must be below frag_count");
    }

    std::vector<std::uint8_t> packet(kHeaderSize + payload.size());
    std::uint8_t* p = packet.data();
    put_le(p + kOffMagic, kMagic);
    put_le(p + kOffVersion, kVersion);
    put_le(p + kOffType, static_cast<std::uint8_t>(header.type));
    put_le(p + kOffSeq, header.seq);
    put_le(p + kOffTimestamp, header.timestamp_us);
    put_le(p + kOffMsgId, header.msg_id);
    put_le(p + kOffFragIndex, header.frag_index);
    put_le(p + kOffFragCount, header.frag_count);
    put_le(p + kOffPayloadLen, static_cast<std::uint16_t>(payload.size()));
    std::copy(payload.begin(), payload.end(), packet.begin() + kHeaderSize);
    put_le(p + kOffCrc, packet_crc(packet));
    return packet;
}

ParsedPacket parse_packet(std::span<const std::uint8_t> datagram) {
    ParsedPacket result;
    if (datagram.size() < kHeaderSize) {
        result.error = ParseError::TooShort;
        return result;
    }
    const std::uint8_t* p = datagram.data();
    if (get_le<std::uint16_t>(p + kOffMagic) != kMagic) {
        result.error = ParseError::BadMagic;
        return result;
    }
    if (p[kOffVersion] != kVersion) {
        result.error = ParseError::BadVersion;
        return result;
    }
    if (!valid_type(p[kOffType])) {
        result.error = ParseError::BadType;
        return result;
    }

    Header& h = result.header;
    h.type = static_cast<MsgType>(p[kOffType]);
    h.seq = get_le<std::uint32_t>(p + kOffSeq);
    h.timestamp_us = get_le<std::uint64_t>(p + kOffTimestamp);
    h.msg_id = get_le<std::uint32_t>(p + kOffMsgId);
    h.frag_index = get_le<std::uint16_t>(p + kOffFragIndex);
    h.frag_count = get_le<std::uint16_t>(p + kOffFragCount);
    h.payload_len = get_le<std::uint16_t>(p + kOffPayloadLen);

    if (h.payload_len > kMaxPayload || h.payload_len != datagram.size() - kHeaderSize) {
        result.error = ParseError::BadLength;
        return result;
    }
    if (h.frag_count == 0 || h.frag_index >= h.frag_count) {
        result.error = ParseError::BadFragment;
        return result;
    }
    if (get_le<std::uint32_t>(p + kOffCrc) != packet_crc(datagram)) {
        result.error = ParseError::BadCrc;
        return result;
    }

    result.payload = datagram.subspan(kHeaderSize);
    result.error = ParseError::Ok;
    return result;
}

Packetizer::Packetizer(std::uint32_t first_seq, std::uint32_t first_msg_id)
    : next_seq_(first_seq), next_msg_id_(first_msg_id) {}

std::vector<std::vector<std::uint8_t>> Packetizer::packetize(MsgType type, std::uint64_t timestamp_us,
                                                             std::span<const std::uint8_t> message) {
    if (message.size() > kMaxMessage) throw std::invalid_argument("message larger than kMaxMessage");

    // An empty message still needs one packet to carry its header.
    const std::size_t count = std::max<std::size_t>(1, (message.size() + kMaxPayload - 1) / kMaxPayload);
    Header header;
    header.type = type;
    header.timestamp_us = timestamp_us;
    header.msg_id = next_msg_id_++;
    header.frag_count = static_cast<std::uint16_t>(count);

    std::vector<std::vector<std::uint8_t>> packets;
    packets.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        const std::size_t begin = i * kMaxPayload;
        const std::size_t len = std::min(kMaxPayload, message.size() - begin);
        header.seq = next_seq_++;
        header.frag_index = static_cast<std::uint16_t>(i);
        packets.push_back(encode_packet(header, message.subspan(begin, len)));
    }
    return packets;
}

}  // namespace aegis
