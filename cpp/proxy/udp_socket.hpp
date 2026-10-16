// A small UDP socket over Winsock2 or POSIX sockets. Every socket binds to 127.0.0.1 only.
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace aegis {

#ifdef _WIN32
using NativeSocket = std::uintptr_t;
#else
using NativeSocket = int;
#endif

// Winsock needs WSAStartup before any socket call; one NetworkInit must outlive every UdpSocket.
class NetworkInit {
public:
    NetworkInit();
    ~NetworkInit();
    NetworkInit(const NetworkInit&) = delete;
    NetworkInit& operator=(const NetworkInit&) = delete;
};

struct Datagram {
    std::size_t size;
    std::uint16_t from_port;
};

class UdpSocket {
public:
    // Binds to 127.0.0.1:port (0 picks a free port); non-blocking. Throws std::runtime_error.
    explicit UdpSocket(std::uint16_t port);
    ~UdpSocket();
    UdpSocket(UdpSocket&& other) noexcept;
    UdpSocket& operator=(UdpSocket&& other) noexcept;
    UdpSocket(const UdpSocket&) = delete;
    UdpSocket& operator=(const UdpSocket&) = delete;

    std::uint16_t port() const { return port_; }
    NativeSocket native() const { return socket_; }

    // Returns false when the OS refused the datagram (for example, a full send buffer).
    bool send_to(std::uint16_t port, std::span<const std::uint8_t> data);

    // Returns the next waiting datagram, or nothing when none is waiting. The buffer should hold
    // 65535 bytes: a larger datagram is dropped by the OS.
    std::optional<Datagram> receive(std::span<std::uint8_t> buffer);

private:
    NativeSocket socket_;
    std::uint16_t port_ = 0;
};

// Waits until a socket is readable or timeout_us passes; returns the indices of readable sockets.
std::vector<std::size_t> wait_readable(const std::vector<const UdpSocket*>& sockets,
                                       std::int64_t timeout_us);

}  // namespace aegis
