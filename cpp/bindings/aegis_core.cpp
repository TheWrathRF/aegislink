// Python module aegislink._core: the wire protocol and reassembler.
#include "aegis/crc32.hpp"
#include "aegis/protocol.hpp"
#include "aegis/reassembler.hpp"

#include <pybind11/native_enum.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <stdexcept>
#include <vector>

namespace py = pybind11;
using namespace aegis;

namespace {

// Accepts bytes, bytearray, memoryview or a uint8 array without copying; the returned span is
// only valid while info is alive.
std::span<const std::uint8_t> as_bytes(const py::buffer_info& info) {
    if (info.ndim > 1 || (info.ndim == 1 && info.strides[0] != info.itemsize)) {
        throw std::invalid_argument("expected a contiguous byte buffer");
    }
    return {static_cast<const std::uint8_t*>(info.ptr), static_cast<std::size_t>(info.size * info.itemsize)};
}

py::bytes to_bytes(std::span<const std::uint8_t> data) {
    return {reinterpret_cast<const char*>(data.data()), data.size()};
}

// Like ParsedPacket, but owns its payload, since the Python datagram may be freed after parsing.
struct Packet {
    ParseError error;
    Header header;
    std::vector<std::uint8_t> payload;
};

}  // namespace

PYBIND11_MODULE(_core, m) {
    m.doc() = "AegisLink wire protocol (C++ core)";

    m.attr("MAGIC") = kMagic;
    m.attr("VERSION") = kVersion;
    m.attr("HEADER_SIZE") = kHeaderSize;
    m.attr("MAX_PAYLOAD") = kMaxPayload;
    m.attr("MAX_DATAGRAM") = kMaxDatagram;
    m.attr("MAX_MESSAGE") = kMaxMessage;

    py::native_enum<MsgType>(m, "MsgType", "enum.IntEnum")
        .value("TELEMETRY", MsgType::Telemetry)
        .value("TRACKS", MsgType::Tracks)
        .value("ROI_PATCH", MsgType::RoiPatch)
        .value("BG_KEYFRAME", MsgType::BgKeyframe)
        .value("FULL_FRAME", MsgType::FullFrame)
        .value("RECEIVER_REPORT", MsgType::ReceiverReport)
        .finalize();

    py::native_enum<ParseError>(m, "ParseError", "enum.Enum")
        .value("OK", ParseError::Ok)
        .value("TOO_SHORT", ParseError::TooShort)
        .value("BAD_MAGIC", ParseError::BadMagic)
        .value("BAD_VERSION", ParseError::BadVersion)
        .value("BAD_TYPE", ParseError::BadType)
        .value("BAD_LENGTH", ParseError::BadLength)
        .value("BAD_FRAGMENT", ParseError::BadFragment)
        .value("BAD_CRC", ParseError::BadCrc)
        .finalize();

    py::class_<Header>(m, "Header")
        .def(py::init([](MsgType type, std::uint32_t seq, std::uint64_t timestamp_us, std::uint32_t msg_id,
                         std::uint16_t frag_index, std::uint16_t frag_count) {
                 return Header{type, seq, timestamp_us, msg_id, frag_index, frag_count, 0};
             }),
             py::arg("type") = MsgType::Telemetry, py::arg("seq") = 0, py::arg("timestamp_us") = 0,
             py::arg("msg_id") = 0, py::arg("frag_index") = 0, py::arg("frag_count") = 1)
        .def_readwrite("type", &Header::type)
        .def_readwrite("seq", &Header::seq)
        .def_readwrite("timestamp_us", &Header::timestamp_us)
        .def_readwrite("msg_id", &Header::msg_id)
        .def_readwrite("frag_index", &Header::frag_index)
        .def_readwrite("frag_count", &Header::frag_count)
        .def_readonly("payload_len", &Header::payload_len);

    py::class_<Packet>(m, "Packet")
        .def_readonly("error", &Packet::error)
        .def_readonly("header", &Packet::header)
        .def_property_readonly("payload", [](const Packet& p) { return to_bytes(p.payload); });

    py::class_<Message>(m, "Message")
        .def_readonly("type", &Message::type)
        .def_readonly("msg_id", &Message::msg_id)
        .def_readonly("timestamp_us", &Message::timestamp_us)
        .def_property_readonly("data", [](const Message& msg) { return to_bytes(msg.data); });

    py::class_<ReassemblerStats>(m, "ReassemblerStats")
        .def_readonly("completed", &ReassemblerStats::completed)
        .def_readonly("duplicates", &ReassemblerStats::duplicates)
        .def_readonly("inconsistent", &ReassemblerStats::inconsistent)
        .def_readonly("expired", &ReassemblerStats::expired)
        .def_readonly("evicted", &ReassemblerStats::evicted);

    m.def(
        "crc32",
        [](const py::buffer& data, std::uint32_t crc) { return crc32(as_bytes(data.request()), crc); },
        py::arg("data"), py::arg("crc") = 0);

    m.def(
        "encode_packet",
        [](const Header& header, const py::buffer& payload) {
            const auto info = payload.request();
            return to_bytes(encode_packet(header, as_bytes(info)));
        },
        py::arg("header"), py::arg("payload"));

    m.def(
        "parse_packet",
        [](const py::buffer& datagram) {
            const auto info = datagram.request();
            const auto parsed = parse_packet(as_bytes(info));
            return Packet{parsed.error, parsed.header, {parsed.payload.begin(), parsed.payload.end()}};
        },
        py::arg("datagram"));

    py::class_<Packetizer>(m, "Packetizer")
        .def(py::init<std::uint32_t, std::uint32_t>(), py::arg("first_seq") = 0, py::arg("first_msg_id") = 0)
        .def(
            "packetize",
            [](Packetizer& self, MsgType type, std::uint64_t timestamp_us, const py::buffer& message) {
                const auto info = message.request();
                py::list out;
                for (const auto& packet : self.packetize(type, timestamp_us, as_bytes(info))) {
                    out.append(to_bytes(packet));
                }
                return out;
            },
            py::arg("type"), py::arg("timestamp_us"), py::arg("message"))
        .def_property_readonly("next_seq", &Packetizer::next_seq)
        .def_property_readonly("next_msg_id", &Packetizer::next_msg_id);

    py::class_<Reassembler>(m, "Reassembler")
        .def(py::init<std::uint64_t, std::size_t>(), py::arg("timeout_us") = 200'000, py::arg("max_pending") = 64)
        .def(
            "push",
            [](Reassembler& self, const Header& header, const py::buffer& payload, std::uint64_t now_us) {
                const auto info = payload.request();
                return self.push(header, as_bytes(info), now_us);
            },
            py::arg("header"), py::arg("payload"), py::arg("now_us"))
        .def("expire", &Reassembler::expire, py::arg("now_us"))
        .def_property_readonly("pending", &Reassembler::pending)
        .def_property_readonly("stats", &Reassembler::stats, py::return_value_policy::copy);
}
