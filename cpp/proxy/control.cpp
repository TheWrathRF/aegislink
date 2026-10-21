#include "control.hpp"

#include <charconv>
#include <cmath>
#include <cstdio>
#include <optional>
#include <stdexcept>

namespace aegis {

namespace {

std::string number(double value) {
    char text[32];
    std::snprintf(text, sizeof(text), "%g", value);
    return text;
}

double parse_number(std::string_view key, std::string_view text) {
    double value = 0.0;
    const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
    if (text.empty() || error != std::errc() || end != text.data() + text.size()) {
        throw std::invalid_argument("bad number for " + std::string(key) + ": '" +
                                    std::string(text) + "'");
    }
    return value;
}

std::string describe(std::size_t index, const Link& link, std::int64_t now_us) {
    const LinkConfig& c = link.config();
    const SpoofConfig s = link.spoofer().config(now_us);
    return "link=" + std::to_string(index + 1) + " loss_pct=" + number(c.loss * 100.0) +
           " burst=" + number(c.mean_burst) + " kbps=" + number(c.kbps) +
           " queue_ms=" + number(c.queue_ms) + " delay_ms=" + number(c.delay_ms) +
           " jitter_ms=" + number(c.jitter_ms) +
           " blackout_ms=" + number(link.blackout_remaining_ms(now_us)) +
           " spoof_drift_mps=" + number(s.drift_mps) + " spoof_bearing_deg=" + number(s.bearing_deg) +
           " spoof_offset_m=" + number(s.offset_m);
}

double kbps(std::uint64_t bytes_now, std::uint64_t bytes_before, double interval_s) {
    return static_cast<double>(bytes_now - bytes_before) * 8.0 / interval_s / 1000.0;
}

}  // namespace

std::string apply_command(std::string_view command, std::vector<Link>& links, std::int64_t now_us) {
    try {
        std::optional<std::size_t> index;
        std::optional<double> blackout_ms;
        std::vector<std::pair<std::string_view, double>> settings;

        std::size_t pos = 0;
        while (pos < command.size()) {
            if (command[pos] == ' ' || command[pos] == '\n' || command[pos] == '\r') {
                ++pos;
                continue;
            }
            std::size_t end = command.find_first_of(" \r\n", pos);
            if (end == std::string_view::npos) end = command.size();
            const std::string_view token = command.substr(pos, end - pos);
            pos = end;

            const std::size_t equals = token.find('=');
            if (equals == std::string_view::npos) {
                throw std::invalid_argument("expected key=value, got '" + std::string(token) + "'");
            }
            const std::string_view key = token.substr(0, equals);
            const double value = parse_number(key, token.substr(equals + 1));
            if (key == "link") {
                if (value != std::floor(value) || value < 1.0 ||
                    value > static_cast<double>(links.size())) {
                    throw std::invalid_argument("link must be 1 to " + std::to_string(links.size()));
                }
                index = static_cast<std::size_t>(value) - 1;
            } else if (key == "blackout_ms") {
                blackout_ms = value;
            } else if (key == "loss_pct" || key == "burst" || key == "kbps" || key == "queue_ms" ||
                       key == "delay_ms" || key == "jitter_ms" || key == "spoof_drift_mps" ||
                       key == "spoof_bearing_deg" || key == "spoof_offset_m") {
                settings.emplace_back(key, value);
            } else {
                throw std::invalid_argument("unknown key '" + std::string(key) + "'");
            }
        }
        if (!index) throw std::invalid_argument("missing link=N");

        // Changes go to a copy first, so a command that fails halfway changes nothing.
        Link updated = links[*index];
        LinkConfig config = updated.config();
        SpoofConfig spoof = updated.spoofer().config(now_us);
        bool spoof_changed = false;
        for (const auto& [key, value] : settings) {
            if (key == "loss_pct") config.loss = value / 100.0;
            if (key == "burst") config.mean_burst = value;
            if (key == "kbps") config.kbps = value;
            if (key == "queue_ms") config.queue_ms = value;
            if (key == "delay_ms") config.delay_ms = value;
            if (key == "jitter_ms") config.jitter_ms = value;
            if (key.starts_with("spoof_")) spoof_changed = true;
            if (key == "spoof_drift_mps") spoof.drift_mps = value;
            if (key == "spoof_bearing_deg") spoof.bearing_deg = value;
            if (key == "spoof_offset_m") spoof.offset_m = value;
        }
        updated.configure(config, now_us);
        if (spoof_changed) updated.spoofer().configure(spoof, now_us);
        if (blackout_ms) updated.blackout(now_us, *blackout_ms);
        links[*index] = updated;
        return "ok " + describe(*index, links[*index], now_us);
    } catch (const std::invalid_argument& e) {
        return std::string("error: ") + e.what();
    }
}

std::string format_stats(const std::vector<Link>& links, const std::vector<LinkStats>& previous,
                         double interval_s, std::int64_t now_us) {
    std::string text = "stats uptime_ms=" + std::to_string(now_us / 1000) + "\n";
    for (std::size_t i = 0; i < links.size(); ++i) {
        const LinkStats& s = links[i].stats();
        const LinkStats& p = previous[i];
        text += "link=" + std::to_string(i + 1) + " in=" + std::to_string(s.in_packets) +
                " out=" + std::to_string(s.out_packets) +
                " drop_blackout=" + std::to_string(s.drop_blackout) +
                " drop_loss=" + std::to_string(s.drop_loss) +
                " drop_queue=" + std::to_string(s.drop_queue) +
                " send_errors=" + std::to_string(s.send_errors) +
                " in_kbps=" + number(kbps(s.in_bytes, p.in_bytes, interval_s)) +
                " out_kbps=" + number(kbps(s.out_bytes, p.out_bytes, interval_s)) +
                " blackout_ms=" + number(links[i].blackout_remaining_ms(now_us)) +
                " spoofed=" + std::to_string(links[i].spoofer().spoofed()) +
                " spoof_offset_m=" + number(links[i].spoofer().offset_m(now_us)) + "\n";
    }
    return text;
}

}  // namespace aegis
