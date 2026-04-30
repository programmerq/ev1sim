// Tests for the data-driven scenario harness.
//
// Covers:
//   - JSON parsing (events, stats, driver_mode, max_time)
//   - Event firing at scheduled times
//   - DriverCommand override semantics (set_throttle / set_brake / set_steering hold)
//   - Hooks dispatch (counts each invocation type)
//   - Stats CSV writes the requested fields with the expected period
//   - IsDone() vs max_time_s

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "Scenario.h"
#include "ExternalSimConnector.h"
#include "VehicleState.h"

namespace {

struct CountingHooks : public ev1sim::ScenarioHooks {
    int key_on_cycle = 0, headlight_cycle = 0;
    int prnd_up = 0, prnd_down = 0;
    int turn_left = 0, turn_right = 0, hazard = 0;
    int ipc_trip_reset = 0;
    int cruise_set = 0, cruise_resume = 0, cruise_cancel = 0;
    int cruise_speed_up = 0, cruise_speed_down = 0;

    void KeyOnCycle()        override { ++key_on_cycle; }
    void HeadlightCycle()    override { ++headlight_cycle; }
    void PrndUp()            override { ++prnd_up; }
    void PrndDown()          override { ++prnd_down; }
    void TurnSignalLeft()    override { ++turn_left; }
    void TurnSignalRight()   override { ++turn_right; }
    void HazardToggle()      override { ++hazard; }
    void IpcTripResetPress() override { ++ipc_trip_reset; }
    void CruiseSet()         override { ++cruise_set; }
    void CruiseResume()      override { ++cruise_resume; }
    void CruiseCancel()      override { ++cruise_cancel; }
    void CruiseSpeedUp()     override { ++cruise_speed_up; }
    void CruiseSpeedDown()   override { ++cruise_speed_down; }
};

}  // namespace

TEST_CASE("Scenario: events fire at scheduled times in order",
          "[Scenario]") {
    using ev1sim::ScenarioEvent;
    using ev1sim::Scenario;

    Scenario s;
    s.set_events({
        {0.10, "key_on_cycle",   0.0},
        {0.20, "key_on_cycle",   0.0},
        {0.50, "set_throttle",   0.5},
        {1.00, "cruise_set",     0.0},
        {1.20, "cruise_speed_up",0.0},
    });

    CountingHooks hooks;
    DriverCommand cmd{};

    // No events have fired before t=0.05.
    s.Tick(0.05, hooks, cmd);
    CHECK(hooks.key_on_cycle == 0);
    CHECK(cmd.throttle == 0.0);

    // Tick at t=0.30 — the two key_on_cycle events both fire (cumulative).
    s.Tick(0.30, hooks, cmd);
    CHECK(hooks.key_on_cycle == 2);

    // Tick at t=0.55 — set_throttle fires; cmd is overridden.
    s.Tick(0.55, hooks, cmd);
    CHECK(cmd.throttle == 0.5);

    // Tick at t=1.30 — both cruise events fire.
    s.Tick(1.30, hooks, cmd);
    CHECK(hooks.cruise_set == 1);
    CHECK(hooks.cruise_speed_up == 1);
}

TEST_CASE("Scenario: set_throttle holds the override on subsequent ticks",
          "[Scenario]") {
    using ev1sim::Scenario;
    Scenario s;
    s.set_events({
        {0.10, "set_throttle", 0.7},
    });

    CountingHooks hooks;
    DriverCommand cmd{};
    cmd.throttle = 0.0;

    s.Tick(0.20, hooks, cmd);
    CHECK(cmd.throttle == 0.7);

    // Even with an external mutation, the next Tick re-applies the held value.
    cmd.throttle = 0.0;
    s.Tick(0.30, hooks, cmd);
    CHECK(cmd.throttle == 0.7);

    // A later set_throttle replaces the held value.
    Scenario s2;
    s2.set_events({
        {0.10, "set_throttle", 0.7},
        {0.30, "set_throttle", 0.0},
    });
    DriverCommand cmd2{};
    s2.Tick(0.20, hooks, cmd2);
    CHECK(cmd2.throttle == 0.7);
    s2.Tick(0.40, hooks, cmd2);
    CHECK(cmd2.throttle == 0.0);
}

TEST_CASE("Scenario: set_brake overrides front + rear together",
          "[Scenario]") {
    using ev1sim::Scenario;
    Scenario s;
    s.set_events({
        {0.10, "set_brake", 0.3},
    });
    CountingHooks hooks;
    DriverCommand cmd{};
    s.Tick(0.20, hooks, cmd);
    CHECK(cmd.front_brake == 0.3);
    CHECK(cmd.rear_brake  == 0.3);
}

TEST_CASE("Scenario: unknown action logs warning, doesn't crash",
          "[Scenario]") {
    using ev1sim::Scenario;
    Scenario s;
    s.set_events({
        {0.10, "make_coffee", 0.0},
        {0.20, "key_on_cycle", 0.0},
    });
    CountingHooks hooks;
    DriverCommand cmd{};
    s.Tick(0.30, hooks, cmd);
    // Unknown action skipped; subsequent valid action still fires.
    CHECK(hooks.key_on_cycle == 1);
}

TEST_CASE("Scenario: IsDone() respects max_time_s",
          "[Scenario]") {
    using ev1sim::Scenario;
    Scenario s;
    s.set_max_time_s(2.0);

    CHECK_FALSE(s.IsDone(0.0));
    CHECK_FALSE(s.IsDone(1.99));
    CHECK(s.IsDone(2.0));
    CHECK(s.IsDone(3.5));

    Scenario s_no_limit;
    CHECK_FALSE(s_no_limit.IsDone(0.0));
    CHECK_FALSE(s_no_limit.IsDone(1e6));
}

TEST_CASE("Scenario: LoadFromFile parses events + stats + driver_mode",
          "[Scenario]") {
    using ev1sim::Scenario;
    auto tmp = std::filesystem::temp_directory_path() /
               "ev1sim_scenario_test.json";
    {
        std::ofstream f(tmp);
        f << R"({
            "name": "test",
            "driver_mode": "electronics",
            "max_time_s": 5.0,
            "events": [
                {"at_time_s": 1.0, "action": "set_throttle", "value": 0.5},
                {"at_time_s": 0.1, "action": "key_on_cycle"}
            ],
            "stats": {
                "output_csv": "/tmp/ev1sim_scenario_test_stats.csv",
                "sample_period_s": 0.1,
                "fields": ["sim_time_s", "speed_mps", "throttle_cmd_q8"]
            }
        })";
    }

    auto loaded = Scenario::LoadFromFile(tmp.string());
    REQUIRE(loaded.has_value());
    CHECK(loaded->name() == "test");
    CHECK(loaded->driver_mode() == "electronics");
    CHECK(loaded->max_time_s() == 5.0);
    CHECK(loaded->event_count() == 2);
    CHECK(loaded->has_stats());
    CHECK(loaded->stats().sample_period_s == 0.1);
    REQUIRE(loaded->stats().fields.size() == 3);

    std::filesystem::remove(tmp);
}

TEST_CASE("Scenario: LoadFromFile sorts events by at_time_s",
          "[Scenario]") {
    using ev1sim::Scenario;
    auto tmp = std::filesystem::temp_directory_path() /
               "ev1sim_scenario_sort_test.json";
    {
        std::ofstream f(tmp);
        f << R"({
            "name": "sort",
            "events": [
                {"at_time_s": 2.0, "action": "cruise_set"},
                {"at_time_s": 0.5, "action": "set_throttle", "value": 0.5},
                {"at_time_s": 1.0, "action": "key_on_cycle"}
            ]
        })";
    }
    auto loaded = Scenario::LoadFromFile(tmp.string());
    REQUIRE(loaded.has_value());

    CountingHooks hooks;
    DriverCommand cmd{};
    // At t=0.6, only the first event (set_throttle, originally listed second)
    // should have fired.
    loaded->Tick(0.6, hooks, cmd);
    CHECK(cmd.throttle == 0.5);
    CHECK(hooks.key_on_cycle == 0);
    CHECK(hooks.cruise_set == 0);

    // At t=1.5, key_on_cycle has fired.
    loaded->Tick(1.5, hooks, cmd);
    CHECK(hooks.key_on_cycle == 1);
    CHECK(hooks.cruise_set == 0);

    // At t=2.5, cruise_set has fired.
    loaded->Tick(2.5, hooks, cmd);
    CHECK(hooks.cruise_set == 1);

    std::filesystem::remove(tmp);
}

TEST_CASE("Scenario: shipped scenario JSON files parse cleanly",
          "[Scenario][Smoke]") {
    // EV1SIM_SOURCE_DIR is set by CMake at build time (already used by the
    // level-file test).  Resolve relative to that so this test runs from
    // any cwd.
    const std::filesystem::path source_root(EV1SIM_SOURCE_DIR);

    struct Expected {
        const char* path;
        const char* driver_mode;
        std::size_t min_events;
    };

    // min_events counts only real events — comment-only entries
    // (action == "") are dropped during load.
    const Expected shipped[] = {
        {"config/scenarios/accel_brake_local.json",       "local",       8},
        {"config/scenarios/cruise_demo_electronics.json", "electronics", 9},
    };

    for (const auto& exp : shipped) {
        const auto full = source_root / exp.path;
        INFO("loading " << full.string());
        auto loaded = ev1sim::Scenario::LoadFromFile(full.string());
        REQUIRE(loaded.has_value());
        CHECK(loaded->driver_mode() == exp.driver_mode);
        CHECK(loaded->event_count() >= exp.min_events);
        CHECK(loaded->max_time_s() > 0.0);
        CHECK(loaded->has_stats());
    }
}

TEST_CASE("Scenario: stats CSV writes header + sampled rows at the configured period",
          "[Scenario]") {
    using ev1sim::Scenario;
    using ev1sim::ScenarioStats;

    auto tmp_csv = std::filesystem::temp_directory_path() /
                   "ev1sim_scenario_stats_test.csv";
    Scenario s;
    ScenarioStats st{tmp_csv.string(),
                     {"sim_time_s", "speed_mps", "applied_throttle"},
                     0.10};
    s.set_stats(st);
    s.OpenStats();

    ExternalSimConnector bus;
    DriverCommand cmd{};
    cmd.throttle = 0.4;

    VehicleState state{};
    state.speed_mps = 10.0;

    s.MaybeSampleStats(0.0,  state, bus, cmd);
    // Within the period — should NOT sample again.
    state.speed_mps = 11.0;
    s.MaybeSampleStats(0.05, state, bus, cmd);
    // Crossed the period — sample.
    state.speed_mps = 12.0;
    s.MaybeSampleStats(0.15, state, bus, cmd);

    s.Close();

    std::ifstream f(tmp_csv);
    REQUIRE(f.is_open());
    std::string header;
    std::getline(f, header);
    CHECK(header == "sim_time_s,speed_mps,applied_throttle");

    std::string row1, row2, row3;
    std::getline(f, row1);
    std::getline(f, row2);
    bool got_extra = static_cast<bool>(std::getline(f, row3));
    // Expect exactly 2 rows: t=0.0 and t=0.15.
    CHECK_FALSE(got_extra);
    CHECK(row1.find("10") != std::string::npos);   // first sample at speed=10
    CHECK(row2.find("12") != std::string::npos);   // third call after period
    f.close();
    std::filesystem::remove(tmp_csv);
}
