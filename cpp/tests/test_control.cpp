#include "control.hpp"
#include "link.hpp"
#include "test_runner.hpp"

#include <string>
#include <vector>

using namespace aegis;

static std::vector<Link> two_links() {
    std::vector<Link> links;
    links.emplace_back(1);
    links.emplace_back(2);
    return links;
}

static bool is_error(const std::string& reply) { return reply.rfind("error: ", 0) == 0; }

TEST(a_command_sets_the_named_link_and_replies_with_its_settings) {
    auto links = two_links();
    const std::string reply = apply_command("link=1 loss_pct=30 burst=4 kbps=50", links, 0);
    CHECK(reply == "ok link=1 loss_pct=30 burst=4 kbps=50 queue_ms=250 delay_ms=0 jitter_ms=0 "
                   "blackout_ms=0");
    CHECK(links[0].config().loss == 0.3);
    CHECK(links[0].config().mean_burst == 4.0);
    CHECK(links[0].config().kbps == 50.0);
    CHECK(links[1].config().loss == 0.0);
    CHECK(links[1].config().kbps == 0.0);
}

TEST(every_key_is_understood) {
    auto links = two_links();
    const std::string reply = apply_command(
        "link=2 loss_pct=5 burst=1.5 kbps=12.5 queue_ms=100 delay_ms=20 jitter_ms=7.5 "
        "blackout_ms=300",
        links, 1'000'000);
    CHECK(reply == "ok link=2 loss_pct=5 burst=1.5 kbps=12.5 queue_ms=100 delay_ms=20 "
                   "jitter_ms=7.5 blackout_ms=300");
    CHECK(links[1].blackout_remaining_ms(1'000'000) == 300.0);
}

TEST(only_a_link_number_asks_for_its_settings) {
    auto links = two_links();
    apply_command("link=2 delay_ms=40", links, 0);
    CHECK(apply_command("  link=2 ", links, 0) ==
          "ok link=2 loss_pct=0 burst=1 kbps=0 queue_ms=250 delay_ms=40 jitter_ms=0 blackout_ms=0");
}

TEST(bad_commands_are_errors_and_change_nothing) {
    const char* bad[] = {
        "",
        "loss_pct=10",
        "link=0",
        "link=3",
        "link=1.5",
        "link=x",
        "link=1 speed=3",
        "link=1 kbps",
        "link=1 kbps=",
        "link=1 kbps=abc",
        "link=1 kbps=5x",
        "link=1 kbps=-5",
        "link=1 loss_pct=90 burst=1",
        "link=1 blackout_ms=500 loss_pct=90 burst=1",
    };
    for (const char* command : bad) {
        auto links = two_links();
        const std::string reply = apply_command(command, links, 0);
        CHECK(is_error(reply));
        if (!is_error(reply)) std::cerr << "  accepted: \"" << command << "\" -> " << reply << "\n";
        CHECK(links[0].config().loss == 0.0 && links[0].config().kbps == 0.0);
        CHECK(links[0].blackout_remaining_ms(0) == 0.0);
    }
}

TEST(stats_list_counters_and_rates_per_link) {
    auto links = two_links();
    const std::vector<LinkStats> previous(2);
    links[0].admit(0, 1000);
    links[0].count_sent(1000, true);
    links[0].admit(0, 250);
    links[0].count_sent(250, true);
    apply_command("link=2 blackout_ms=1000", links, 0);
    links[1].admit(0, 100);

    const std::string text = format_stats(links, previous, 0.25, 500'000);
    CHECK(text ==
          "stats uptime_ms=500\n"
          "link=1 in=2 out=2 drop_blackout=0 drop_loss=0 drop_queue=0 send_errors=0 in_kbps=40 "
          "out_kbps=40 blackout_ms=0\n"
          "link=2 in=1 out=0 drop_blackout=1 drop_loss=0 drop_queue=0 send_errors=0 in_kbps=3.2 "
          "out_kbps=0 blackout_ms=500\n");
}
