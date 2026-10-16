// ew_chaos_proxy: forwards UDP datagrams between local ports through simulated radio links.
//
//   ew_chaos_proxy --link 5600:5601 [--link LISTEN:FORWARD ...] [--control 5700] [--seed 1]
//
// Each --link receives on 127.0.0.1:LISTEN and sends on to 127.0.0.1:FORWARD, applying its own
// impairments. Commands arrive on the control port (see control.hpp); stats go 4 times per
// second to whoever sent the last command.

#include "control.hpp"
#include "link.hpp"
#include "udp_socket.hpp"

#include <charconv>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <optional>
#include <queue>
#include <string>
#include <string_view>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#include <timeapi.h>
#endif

using namespace aegis;

namespace {

constexpr std::int64_t kStatsIntervalUs = 250'000;
// Per wakeup and socket, so a flooded link cannot starve the others or the release queue.
constexpr int kMaxReadsPerWakeup = 256;

struct Route {
    std::uint16_t listen;
    std::uint16_t forward;
};

struct Options {
    std::vector<Route> routes;
    std::uint16_t control = 5700;
    std::uint64_t seed = 1;
};

std::optional<std::uint16_t> parse_port(std::string_view text) {
    unsigned value = 0;
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (error != std::errc() || end != text.data() + text.size() || value == 0 || value > 65535) {
        return std::nullopt;
    }
    return static_cast<std::uint16_t>(value);
}

std::optional<Options> parse_args(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (i + 1 >= argc) {
            std::fprintf(stderr, "missing value after %s\n", argv[i]);
            return std::nullopt;
        }
        const std::string_view value = argv[++i];
        if (arg == "--link") {
            const std::size_t colon = value.find(':');
            const auto listen = parse_port(value.substr(0, colon));
            const auto forward =
                colon == std::string_view::npos ? std::nullopt : parse_port(value.substr(colon + 1));
            if (!listen || !forward) {
                std::fprintf(stderr, "bad --link '%s', expected LISTEN:FORWARD\n", argv[i]);
                return std::nullopt;
            }
            options.routes.push_back({*listen, *forward});
        } else if (arg == "--control") {
            const auto port = parse_port(value);
            if (!port) {
                std::fprintf(stderr, "bad --control port '%s'\n", argv[i]);
                return std::nullopt;
            }
            options.control = *port;
        } else if (arg == "--seed") {
            const auto [end, error] =
                std::from_chars(value.data(), value.data() + value.size(), options.seed);
            if (error != std::errc() || end != value.data() + value.size()) {
                std::fprintf(stderr, "bad --seed '%s'\n", argv[i]);
                return std::nullopt;
            }
        } else {
            std::fprintf(stderr, "unknown option %s\n", argv[i - 1]);
            return std::nullopt;
        }
    }
    if (options.routes.empty()) {
        std::fprintf(stderr, "at least one --link LISTEN:FORWARD is needed\n");
        return std::nullopt;
    }
    return options;
}

struct Pending {
    std::int64_t release_us;
    std::uint64_t order;  // keeps packets with the same release time in arrival order
    std::size_t link;
    std::vector<std::uint8_t> data;
};

struct ReleasesLater {
    bool operator()(const Pending& a, const Pending& b) const {
        return a.release_us != b.release_us ? a.release_us > b.release_us : a.order > b.order;
    }
};

int run(const Options& options) {
    NetworkInit network;
    std::vector<UdpSocket> sockets;
    std::vector<Link> links;
    for (std::size_t i = 0; i < options.routes.size(); ++i) {
        sockets.emplace_back(options.routes[i].listen);
        links.emplace_back(options.seed + i);
        std::printf("link %zu: 127.0.0.1:%u -> 127.0.0.1:%u\n", i + 1,
                    unsigned{options.routes[i].listen}, unsigned{options.routes[i].forward});
    }
    UdpSocket control(options.control);
    std::printf("control: 127.0.0.1:%u\n", unsigned{options.control});
    std::fflush(stdout);

    std::vector<const UdpSocket*> watched;
    for (const UdpSocket& s : sockets) watched.push_back(&s);
    watched.push_back(&control);

    const auto start = std::chrono::steady_clock::now();
    const auto now_us = [start] {
        return std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::steady_clock::now() - start)
            .count();
    };

    std::priority_queue<Pending, std::vector<Pending>, ReleasesLater> queue;
    std::uint64_t order = 0;
    std::vector<std::uint8_t> buffer(65535);
    std::optional<std::uint16_t> stats_port;
    std::vector<LinkStats> previous(links.size());
    std::int64_t last_stats_us = 0;

    for (;;) {
        std::int64_t wake_us = last_stats_us + kStatsIntervalUs;
        if (!queue.empty() && queue.top().release_us < wake_us) wake_us = queue.top().release_us;
        const auto ready = wait_readable(watched, wake_us - now_us());

        for (const std::size_t index : ready) {
            for (int n = 0; n < kMaxReadsPerWakeup; ++n) {
                if (index == links.size()) {
                    const auto got = control.receive(buffer);
                    if (!got) break;
                    const std::string_view command(reinterpret_cast<const char*>(buffer.data()),
                                                   got->size);
                    const std::string reply = apply_command(command, links, now_us());
                    control.send_to(got->from_port,
                                    {reinterpret_cast<const std::uint8_t*>(reply.data()), reply.size()});
                    stats_port = got->from_port;
                } else {
                    const auto got = sockets[index].receive(buffer);
                    if (!got) break;
                    const Verdict verdict = links[index].admit(now_us(), got->size);
                    if (verdict.fate == Fate::Forward) {
                        queue.push({verdict.release_us, order++, index,
                                    {buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(got->size)}});
                    }
                }
            }
        }

        while (!queue.empty() && queue.top().release_us <= now_us()) {
            const Pending& packet = queue.top();
            const bool ok = sockets[packet.link].send_to(options.routes[packet.link].forward, packet.data);
            links[packet.link].count_sent(packet.data.size(), ok);
            queue.pop();
        }

        const std::int64_t now = now_us();
        if (now - last_stats_us >= kStatsIntervalUs) {
            if (stats_port) {
                const double interval_s = static_cast<double>(now - last_stats_us) / 1e6;
                const std::string text = format_stats(links, previous, interval_s, now);
                control.send_to(*stats_port,
                                {reinterpret_cast<const std::uint8_t*>(text.data()), text.size()});
            }
            for (std::size_t i = 0; i < links.size(); ++i) previous[i] = links[i].stats();
            last_stats_us = now;
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    const auto options = parse_args(argc, argv);
    if (!options) {
        std::fprintf(stderr,
                     "usage: ew_chaos_proxy --link LISTEN:FORWARD [--link ...] [--control PORT] "
                     "[--seed N]\n");
        return 2;
    }
#ifdef _WIN32
    // Windows wakes sleeping threads on a coarse timer by default: without this, select()
    // returned up to 8.5 ms late (1 ms with it), too coarse for delays of a few milliseconds.
    timeBeginPeriod(1);
#endif
    try {
        return run(*options);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "ew_chaos_proxy: %s\n", e.what());
        return 1;
    }
}
