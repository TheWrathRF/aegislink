// The proxy's control protocol: one plain-text command per datagram, one reply per command.
#pragma once

#include "link.hpp"

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace aegis {

// A command is space-separated key=value pairs naming one link (numbered from 1):
//   link=1 loss_pct=30 burst=4 kbps=50 queue_ms=250 delay_ms=20 jitter_ms=10 blackout_ms=5000
// Every key but link is optional, and "link=1" alone only asks for the settings. The reply is
// "ok " plus all of the link's settings in the same form, or "error: <reason>", in which case
// nothing has changed.
std::string apply_command(std::string_view command, std::vector<Link>& links, std::int64_t now_us);

// The stats report: "stats uptime_ms=<n>", then one line per link with its counters and the
// rates over the last interval_s (previous holds the counters from the report before).
std::string format_stats(const std::vector<Link>& links, const std::vector<LinkStats>& previous,
                         double interval_s, std::int64_t now_us);

}  // namespace aegis
