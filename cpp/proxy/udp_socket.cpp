#include "udp_socket.hpp"

#include <stdexcept>
#include <string>
#include <utility>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
using socklen_t = int;
#else
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace aegis {

namespace {

#ifdef _WIN32
constexpr NativeSocket kInvalid = INVALID_SOCKET;
int last_error() { return WSAGetLastError(); }
bool would_block(int error) { return error == WSAEWOULDBLOCK; }
void close_socket(NativeSocket s) { closesocket(s); }
#else
constexpr NativeSocket kInvalid = -1;
int last_error() { return errno; }
bool would_block(int error) { return error == EAGAIN || error == EWOULDBLOCK; }
void close_socket(NativeSocket s) { close(s); }
#endif

sockaddr_in loopback(std::uint16_t port) {
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    return addr;
}

[[noreturn]] void fail(const char* what) {
    throw std::runtime_error(std::string(what) + " failed, error " + std::to_string(last_error()));
}

}  // namespace

#ifdef _WIN32
NetworkInit::NetworkInit() {
    WSADATA data;
    if (WSAStartup(MAKEWORD(2, 2), &data) != 0) throw std::runtime_error("WSAStartup failed");
}
NetworkInit::~NetworkInit() { WSACleanup(); }
#else
NetworkInit::NetworkInit() = default;
NetworkInit::~NetworkInit() = default;
#endif

UdpSocket::UdpSocket(std::uint16_t port) : socket_(socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) {
    if (socket_ == kInvalid) fail("socket");
    try {
        // Large buffers absorb the bursts a whole video frame makes (dozens of datagrams at once).
        const int buffer_bytes = 4 * 1024 * 1024;
        const char* value = reinterpret_cast<const char*>(&buffer_bytes);
        setsockopt(socket_, SOL_SOCKET, SO_RCVBUF, value, sizeof(buffer_bytes));
        setsockopt(socket_, SOL_SOCKET, SO_SNDBUF, value, sizeof(buffer_bytes));

        const sockaddr_in addr = loopback(port);
        if (bind(socket_, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) fail("bind");
        sockaddr_in bound{};
        socklen_t len = sizeof(bound);
        if (getsockname(socket_, reinterpret_cast<sockaddr*>(&bound), &len) != 0) fail("getsockname");
        port_ = ntohs(bound.sin_port);

#ifdef _WIN32
        u_long non_blocking = 1;
        if (ioctlsocket(socket_, FIONBIO, &non_blocking) != 0) fail("ioctlsocket");
#else
        if (fcntl(socket_, F_SETFL, fcntl(socket_, F_GETFL) | O_NONBLOCK) != 0) fail("fcntl");
#endif
    } catch (...) {
        close_socket(socket_);
        throw;
    }
}

UdpSocket::~UdpSocket() {
    if (socket_ != kInvalid) close_socket(socket_);
}

UdpSocket::UdpSocket(UdpSocket&& other) noexcept
    : socket_(std::exchange(other.socket_, kInvalid)), port_(other.port_) {}

UdpSocket& UdpSocket::operator=(UdpSocket&& other) noexcept {
    std::swap(socket_, other.socket_);
    std::swap(port_, other.port_);
    return *this;
}

bool UdpSocket::send_to(std::uint16_t port, std::span<const std::uint8_t> data) {
    const sockaddr_in addr = loopback(port);
    const auto sent = sendto(socket_, reinterpret_cast<const char*>(data.data()),
                             static_cast<int>(data.size()), 0,
                             reinterpret_cast<const sockaddr*>(&addr), sizeof(addr));
    return sent >= 0 && static_cast<std::size_t>(sent) == data.size();
}

std::optional<Datagram> UdpSocket::receive(std::span<std::uint8_t> buffer) {
    // A failed read consumes one datagram or one error only, so try the next one. On Windows, a
    // send to a port nobody listens on makes the ICMP "port unreachable" reply show up here as
    // WSAECONNRESET. A bounded number of tries keeps a broken socket from spinning forever.
    for (int attempt = 0; attempt < 64; ++attempt) {
        sockaddr_in from{};
        socklen_t len = sizeof(from);
        const auto got = recvfrom(socket_, reinterpret_cast<char*>(buffer.data()),
                                  static_cast<int>(buffer.size()), 0,
                                  reinterpret_cast<sockaddr*>(&from), &len);
        if (got >= 0) return Datagram{static_cast<std::size_t>(got), ntohs(from.sin_port)};
        if (would_block(last_error())) return std::nullopt;
    }
    return std::nullopt;
}

std::vector<std::size_t> wait_readable(const std::vector<const UdpSocket*>& sockets,
                                       std::int64_t timeout_us) {
    fd_set readable;
    FD_ZERO(&readable);
    NativeSocket highest = 0;
    for (const UdpSocket* s : sockets) {
        FD_SET(s->native(), &readable);
        if (s->native() > highest) highest = s->native();
    }
    if (timeout_us < 0) timeout_us = 0;
    timeval timeout{};
    timeout.tv_sec = static_cast<decltype(timeout.tv_sec)>(timeout_us / 1'000'000);
    timeout.tv_usec = static_cast<decltype(timeout.tv_usec)>(timeout_us % 1'000'000);

    std::vector<std::size_t> ready;
    // The first argument is ignored on Windows.
    if (select(static_cast<int>(highest) + 1, &readable, nullptr, nullptr, &timeout) <= 0) {
        return ready;
    }
    for (std::size_t i = 0; i < sockets.size(); ++i) {
        if (FD_ISSET(sockets[i]->native(), &readable)) ready.push_back(i);
    }
    return ready;
}

}  // namespace aegis
