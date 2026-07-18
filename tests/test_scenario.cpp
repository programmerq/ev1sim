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
#include <thread>

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
    int door_lock_all = 0, door_unlock_all = 0;
    int fail_throttle = 0, restore_throttle = 0;

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
    void DoorLockAll()       override { ++door_lock_all; }
    void DoorUnlockAll()     override { ++door_unlock_all; }
    void FailThrottleInput(bool fail) override {
        if (fail) ++fail_throttle; else ++restore_throttle;
    }
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
    s.Tick(0.05, VehicleState{}, hooks, cmd);
    CHECK(hooks.key_on_cycle == 0);
    CHECK(cmd.throttle == 0.0);

    // Tick at t=0.30 — the two key_on_cycle events both fire (cumulative).
    s.Tick(0.30, VehicleState{}, hooks, cmd);
    CHECK(hooks.key_on_cycle == 2);

    // Tick at t=0.55 — set_throttle fires; cmd is overridden.
    s.Tick(0.55, VehicleState{}, hooks, cmd);
    CHECK(cmd.throttle == 0.5);

    // Tick at t=1.30 — both cruise events fire.
    s.Tick(1.30, VehicleState{}, hooks, cmd);
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

    s.Tick(0.20, VehicleState{}, hooks, cmd);
    CHECK(cmd.throttle == 0.7);

    // Even with an external mutation, the next Tick re-applies the held value.
    cmd.throttle = 0.0;
    s.Tick(0.30, VehicleState{}, hooks, cmd);
    CHECK(cmd.throttle == 0.7);

    // A later set_throttle replaces the held value.
    Scenario s2;
    s2.set_events({
        {0.10, "set_throttle", 0.7},
        {0.30, "set_throttle", 0.0},
    });
    DriverCommand cmd2{};
    s2.Tick(0.20, VehicleState{}, hooks, cmd2);
    CHECK(cmd2.throttle == 0.7);
    s2.Tick(0.40, VehicleState{}, hooks, cmd2);
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
    s.Tick(0.20, VehicleState{}, hooks, cmd);
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
    s.Tick(0.30, VehicleState{}, hooks, cmd);
    // Unknown action skipped; subsequent valid action still fires.
    CHECK(hooks.key_on_cycle == 1);
}

TEST_CASE("Scenario: wait_for_speed blocks subsequent events until threshold reached",
          "[Scenario]") {
    using ev1sim::Scenario;
    Scenario s;
    s.set_events({
        {0.10, "set_throttle",   0.5,  0.0},
        {1.00, "wait_for_speed", 10.0, 0.0},
        {1.00, "cruise_set",     0.0,  0.0},
    });

    CountingHooks hooks;
    DriverCommand cmd{};
    VehicleState slow{};
    slow.speed_mps = 5.0;

    // At t=1.5 the wait_for_speed barrier is reached but speed is below
    // threshold — cruise_set must NOT fire yet.
    s.Tick(1.5, slow, hooks, cmd);
    CHECK(cmd.throttle == 0.5);    // set_throttle did fire
    CHECK(hooks.cruise_set == 0);  // cruise_set blocked

    // Speed crosses the threshold — cruise_set fires on the next Tick
    // even though sim_time hasn't moved.
    VehicleState fast = slow;
    fast.speed_mps = 12.0;
    s.Tick(1.5, fast, hooks, cmd);
    CHECK(hooks.cruise_set == 1);
}

TEST_CASE("Scenario: assert_speed_within passes when within tolerance",
          "[Scenario]") {
    using ev1sim::Scenario;
    Scenario s;
    s.set_events({
        {0.10, "assert_speed_within", 25.0, 1.0},
    });

    CountingHooks hooks;
    DriverCommand cmd{};
    VehicleState st{};
    st.speed_mps = 25.4;  // |err|=0.4, tol=1.0 → pass

    CHECK_FALSE(s.IsScenarioFailed());
    s.Tick(0.20, st, hooks, cmd);
    CHECK(s.PassedAssertions() == 1);
    CHECK(s.FailedAssertions() == 0);
    CHECK_FALSE(s.IsScenarioFailed());
}

TEST_CASE("Scenario: assert_speed_within fails when outside tolerance",
          "[Scenario]") {
    using ev1sim::Scenario;
    Scenario s;
    s.set_events({
        {0.10, "assert_speed_within", 25.0, 0.5},
    });

    CountingHooks hooks;
    DriverCommand cmd{};
    VehicleState st{};
    st.speed_mps = 23.0;  // |err|=2.0, tol=0.5 → fail

    s.Tick(0.20, st, hooks, cmd);
    CHECK(s.FailedAssertions() == 1);
    CHECK(s.IsScenarioFailed());
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

TEST_CASE("Scenario: LoadFromFile reads requires_external_sim",
          "[Scenario]") {
    using ev1sim::Scenario;
    auto write_tmp = [](const std::string& body) {
        auto tmp = std::filesystem::temp_directory_path() /
                   "ev1sim_scenario_reqext_test.json";
        { std::ofstream f(tmp); f << body; }
        return tmp;
    };

    SECTION("defaults to false when the key is absent") {
        auto tmp = write_tmp(R"({"name": "no_flag", "max_time_s": 1.0})");
        auto loaded = Scenario::LoadFromFile(tmp.string());
        REQUIRE(loaded.has_value());
        CHECK_FALSE(loaded->requires_external_sim());
        std::filesystem::remove(tmp);
    }

    SECTION("parses true") {
        auto tmp = write_tmp(
            R"({"name": "needs_sim", "max_time_s": 1.0, "requires_external_sim": true})");
        auto loaded = Scenario::LoadFromFile(tmp.string());
        REQUIRE(loaded.has_value());
        CHECK(loaded->requires_external_sim());
        std::filesystem::remove(tmp);
    }
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
    loaded->Tick(0.6, VehicleState{}, hooks, cmd);
    CHECK(cmd.throttle == 0.5);
    CHECK(hooks.key_on_cycle == 0);
    CHECK(hooks.cruise_set == 0);

    // At t=1.5, key_on_cycle has fired.
    loaded->Tick(1.5, VehicleState{}, hooks, cmd);
    CHECK(hooks.key_on_cycle == 1);
    CHECK(hooks.cruise_set == 0);

    // At t=2.5, cruise_set has fired.
    loaded->Tick(2.5, VehicleState{}, hooks, cmd);
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
        {"config/scenarios/coastdown.json",               "local",       7},
        {"config/scenarios/abs_hard_brake.json",          "local",       9},
        {"config/scenarios/abs_high_mu_stop.json",        "local",      10},
        {"config/scenarios/abs_low_mu_stop.json",         "local",      10},
        {"config/scenarios/abs_mu_jump.json",             "local",      10},
        {"config/scenarios/abs_split_mu.json",            "local",      10},
        {"config/scenarios/abs_brake_and_steer.json",     "local",      11},
        {"config/scenarios/abs_diagonal_mu.json",         "local",      10},
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

TEST_CASE("Scenario: pitch/grade stats fields read the vehicle state",
          "[Scenario]") {
    using ev1sim::Scenario;
    using ev1sim::ScenarioStats;

    auto tmp_csv = std::filesystem::temp_directory_path() /
                   "ev1sim_scenario_stats_grade_test.csv";
    Scenario s;
    ScenarioStats st{tmp_csv.string(),
                     {"sim_time_s", "pitch_deg", "road_grade_pct"},
                     0.10};
    s.set_stats(st);
    s.OpenStats();

    ExternalSimConnector bus;
    DriverCommand cmd{};
    VehicleState state{};
    state.pitch_deg      = -1.25;   // nose down (braking squat)
    state.road_grade_pct = 6.0;     // 6% uphill

    s.MaybeSampleStats(0.0, state, bus, cmd);
    s.Close();

    std::ifstream f(tmp_csv);
    REQUIRE(f.is_open());
    std::string header, row;
    std::getline(f, header);
    std::getline(f, row);
    CHECK(header == "sim_time_s,pitch_deg,road_grade_pct");
    CHECK(row == "0,-1.25,6");
    f.close();
    std::filesystem::remove(tmp_csv);
}

TEST_CASE("Scenario: AD / bulb / horn stats fields read the bus mirrors",
          "[Scenario]") {
    using ev1sim::Scenario;
    using ev1sim::ScenarioStats;

    auto tmp_csv = std::filesystem::temp_directory_path() /
                   "ev1sim_scenario_stats_hv_lighting_test.csv";
    Scenario s;
    ScenarioStats st{tmp_csv.string(),
                     {"sim_time_s", "ad_main_contactor_closed",
                      "ad_precharge_relay_closed", "ad_state_enum",
                      "lrsl_bulb_feed_line", "horn_low_cmd",
                      "nope_bulb_feed_line"},
                     0.10};
    s.set_stats(st);
    s.OpenStats();

    ExternalSimConnector bus;
    DriverCommand cmd{};
    VehicleState state{};

    // Row 1: nothing received — AD fields 0, bulb 0, horn 0, unknown blank.
    s.MaybeSampleStats(0.0, state, bus, cmd);

    // Row 2: precharge in progress + stop lamp lit + low horn sounding.
    bus.DebugInjectDelta(5225, true);   // precharge relay closed
    bus.DebugInjectU32(5230, 6u);       // state: precharging
    bus.DebugInjectDelta(4014, true);   // LRSL (left rear stop lamp) feed
    bus.DebugInjectDelta(4020, true);   // horn low command
    s.MaybeSampleStats(0.15, state, bus, cmd);

    s.Close();

    std::ifstream f(tmp_csv);
    REQUIRE(f.is_open());
    std::string header, row1, row2;
    std::getline(f, header);
    CHECK(header == "sim_time_s,ad_main_contactor_closed,"
                    "ad_precharge_relay_closed,ad_state_enum,"
                    "lrsl_bulb_feed_line,horn_low_cmd,nope_bulb_feed_line");
    std::getline(f, row1);
    std::getline(f, row2);
    CHECK(row1 == "0,0,0,0,0,0,");        // defaults; unknown bulb blank
    CHECK(row2 == "0.15,0,1,6,1,1,");     // precharge phase mirrored
    f.close();
    std::filesystem::remove(tmp_csv);
}

TEST_CASE("Scenario: IPC air-bag telltale stats field reads the bus mirror",
          "[Scenario]") {
    using ev1sim::Scenario;
    using ev1sim::ScenarioStats;

    auto tmp_csv = std::filesystem::temp_directory_path() /
                   "ev1sim_scenario_stats_airbag_test.csv";
    Scenario s;
    ScenarioStats st{tmp_csv.string(),
                     {"sim_time_s", "ipc_air_bag_telltale"},
                     0.10};
    s.set_stats(st);
    s.OpenStats();

    ExternalSimConnector bus;
    DriverCommand cmd{};
    VehicleState state{};

    // Row 1: nothing received — telltale defaults off (latched false).
    s.MaybeSampleStats(0.0, state, bus, cmd);

    // Row 2: SIR/SDM air-bag telltale lit (chassis 4138 = 1).
    bus.DebugInjectU8(4138, 1u);  // 4138 = kSigChassisIpcAirBagTelltale
    s.MaybeSampleStats(0.15, state, bus, cmd);

    s.Close();

    std::ifstream f(tmp_csv);
    REQUIRE(f.is_open());
    std::string header, row1, row2;
    std::getline(f, header);
    CHECK(header == "sim_time_s,ipc_air_bag_telltale");
    std::getline(f, row1);
    std::getline(f, row2);
    CHECK(row1 == "0,0");        // default off
    CHECK(row2 == "0.15,1");     // air-bag telltale mirrored
    f.close();
    std::filesystem::remove(tmp_csv);
}

TEST_CASE("Scenario: HVAC blower level stats field reads the bus mirror as int",
          "[Scenario]") {
    using ev1sim::Scenario;
    using ev1sim::ScenarioStats;

    auto tmp_csv = std::filesystem::temp_directory_path() /
                   "ev1sim_scenario_stats_blower_test.csv";
    Scenario s;
    ScenarioStats st{tmp_csv.string(),
                     {"sim_time_s", "hvac_blower_level"},
                     0.10};
    s.set_stats(st);
    s.OpenStats();

    ExternalSimConnector bus;
    DriverCommand cmd{};
    VehicleState state{};

    // Row 1: nothing received — blower level default sentinel (0xFF -> 255).
    s.MaybeSampleStats(0.0, state, bus, cmd);

    // Row 2: blower commanded HIGH (chassis 4082 = 3). Must print as an int.
    bus.DebugInjectU8(4082, 3u);  // 4082 = HVAC blower level enum
    s.MaybeSampleStats(0.15, state, bus, cmd);

    s.Close();

    std::ifstream f(tmp_csv);
    REQUIRE(f.is_open());
    std::string header, row1, row2;
    std::getline(f, header);
    CHECK(header == "sim_time_s,hvac_blower_level");
    std::getline(f, row1);
    std::getline(f, row2);
    CHECK(row1 == "0,255");      // default sentinel, cast to int (not a char)
    CHECK(row2 == "0.15,3");     // blower level mirrored as an int
    f.close();
    std::filesystem::remove(tmp_csv);
}

TEST_CASE("Scenario: door-lock motor leg drives read the bus mirror",
          "[Scenario]") {
    using ev1sim::Scenario;
    using ev1sim::ScenarioStats;

    auto tmp_csv = std::filesystem::temp_directory_path() /
                   "ev1sim_scenario_stats_doorlock_test.csv";
    Scenario s;
    ScenarioStats st{tmp_csv.string(),
                     {"sim_time_s",
                      "door_lock_motor_lh_lock_drive",
                      "door_lock_motor_lh_unlock_drive",
                      "door_lock_motor_rh_lock_drive",
                      "door_lock_motor_rh_unlock_drive"},
                     0.10};
    s.set_stats(st);
    s.OpenStats();

    ExternalSimConnector bus;
    DriverCommand cmd{};
    VehicleState state{};

    // Row 1: nothing received — all four legs default off.
    s.MaybeSampleStats(0.0, state, bus, cmd);

    // Row 2: energise the two LOCK legs (4092 LH lock, 4094 RH lock).
    bus.DebugInjectDelta(4092, true);   // LH lock drive
    bus.DebugInjectDelta(4094, true);   // RH lock drive
    s.MaybeSampleStats(0.15, state, bus, cmd);

    s.Close();

    std::ifstream f(tmp_csv);
    REQUIRE(f.is_open());
    std::string header, row1, row2;
    std::getline(f, header);
    CHECK(header == "sim_time_s,door_lock_motor_lh_lock_drive,"
                    "door_lock_motor_lh_unlock_drive,"
                    "door_lock_motor_rh_lock_drive,"
                    "door_lock_motor_rh_unlock_drive");
    std::getline(f, row1);
    std::getline(f, row2);
    CHECK(row1 == "0,0,0,0,0");        // all legs default off
    CHECK(row2 == "0.15,1,0,1,0");     // both LOCK legs energised
    f.close();
    std::filesystem::remove(tmp_csv);
}

TEST_CASE("Scenario: ad_precharge_participated latches once the relay closes",
          "[Scenario]") {
    using ev1sim::Scenario;
    using ev1sim::ScenarioStats;

    auto tmp_csv = std::filesystem::temp_directory_path() /
                   "ev1sim_scenario_stats_precharge_latch_test.csv";
    Scenario s;
    ScenarioStats st{tmp_csv.string(),
                     {"sim_time_s", "ad_precharge_participated"},
                     0.10};
    s.set_stats(st);
    s.OpenStats();

    ExternalSimConnector bus;
    DriverCommand cmd{};
    VehicleState state{};

    // Row 1: nothing received — participation latch defaults off.
    s.MaybeSampleStats(0.0, state, bus, cmd);

    // Row 2: precharge relay closes (chassis 5225 = 1) — latch sets.
    bus.DebugInjectDelta(5225, true);
    s.MaybeSampleStats(0.15, state, bus, cmd);

    // Row 3: relay re-opens (5225 = 0) BEFORE the next sample — the
    // instantaneous state is now false, but the latch must remain 1.
    bus.DebugInjectDelta(5225, false);
    s.MaybeSampleStats(0.30, state, bus, cmd);

    s.Close();

    std::ifstream f(tmp_csv);
    REQUIRE(f.is_open());
    std::string header, row1, row2, row3;
    std::getline(f, header);
    CHECK(header == "sim_time_s,ad_precharge_participated");
    std::getline(f, row1);
    std::getline(f, row2);
    std::getline(f, row3);
    CHECK(row1 == "0,0");        // default off
    CHECK(row2 == "0.15,1");     // latched on relay close
    CHECK(row3 == "0.3,1");      // stays latched after relay re-opens
    f.close();
    std::filesystem::remove(tmp_csv);
}

TEST_CASE("Scenario: abs-phase / rear-EMB stats columns stay live across a "
          "sub-3s heartbeat gap",
          "[Scenario][ABS]") {
    // Regression: the stats-CSV observer used to read the BTCM-sourced ABS
    // phase (and rear-EMB command) with a 200 ms freshness window while the
    // control path used 3000 ms.  Liveness is gated on the BTCM's ~5 Hz
    // canonical-frame heartbeat, which is paced in sim time; under wall-clock
    // pacing on long, low-friction stops the inter-heartbeat gap can stretch
    // past 200 ms.  With the old 200 ms window the observer then declared the
    // BTCM dead and wrote the -1 no-data sentinel for a phase that was still
    // live.  The observer now uses the same 3000 ms window as the control path,
    // so a gap between 200 ms and 3000 ms must still read the live phase.
    using ev1sim::Scenario;
    using ev1sim::ScenarioStats;

    auto tmp_csv = std::filesystem::temp_directory_path() /
                   "ev1sim_scenario_stats_abs_freshness_test.csv";
    Scenario s;
    ScenarioStats st{tmp_csv.string(),
                     {"sim_time_s", "abs_phase_fl", "abs_fresh_fl",
                      "emb_cmd_lr", "emb_fresh_lr"},
                     0.10};
    s.set_stats(st);
    s.OpenStats();

    ExternalSimConnector bus;
    DriverCommand cmd{};
    VehicleState state{};

    // Inject a BTCM heartbeat, an FL HOLD phase (iso=1, dump=0 -> phase 1),
    // and a rear-EMB LR command.  All timestamps are stamped "now".
    bus.DebugInjectDelta(5050, true);   // BTCM canonical-frame heartbeat
    bus.DebugInjectDelta(5010, true);   // FL_ISO = 1
    bus.DebugInjectDelta(5011, false);  // FL_DMP = 0  -> HOLD
    bus.DebugInjectFloat(5014, 1.0f);   // rear motor LR command = full apply

    // Advance the heartbeat gap to ~260 ms: past the old 200 ms window, well
    // within the corrected 3000 ms window.  sleep_for guarantees at least the
    // requested duration, so the gap is reliably > 200 ms and << 3000 ms.
    std::this_thread::sleep_for(std::chrono::milliseconds(260));

    // Document the seam directly on the production symbol: at this gap the old
    // 200 ms window reads stale, the corrected 3000 ms window reads live.
    CHECK_FALSE(bus.GetAbsPhaseFront(std::chrono::milliseconds(200)).fl_fresh);
    CHECK(bus.GetAbsPhaseFront(std::chrono::milliseconds(3000)).fl_fresh);

    s.MaybeSampleStats(0.0, state, bus, cmd);
    s.Close();

    std::ifstream f(tmp_csv);
    REQUIRE(f.is_open());
    std::string header, row1;
    std::getline(f, header);
    CHECK(header == "sim_time_s,abs_phase_fl,abs_fresh_fl,"
                    "emb_cmd_lr,emb_fresh_lr");
    std::getline(f, row1);
    // With the corrected window the phase is live: abs_phase_fl = 1 (HOLD),
    // NOT the -1 no-data sentinel; abs_fresh_fl = 1; the rear-EMB command
    // mirrors through fresh too.
    CHECK(row1 == "0,1,1,1,1");
    f.close();
    std::filesystem::remove(tmp_csv);
}

TEST_CASE("Scenario: door_lock_all / door_unlock_all dispatch to the hooks",
          "[Scenario]") {
    using ev1sim::Scenario;

    Scenario s;
    s.set_events({
        {0.10, "door_lock_all",   0.0},
        {0.30, "door_unlock_all", 0.0},
    });

    CountingHooks hooks;
    DriverCommand cmd{};

    // Before t=0.10 nothing has fired.
    s.Tick(0.05, VehicleState{}, hooks, cmd);
    CHECK(hooks.door_lock_all == 0);
    CHECK(hooks.door_unlock_all == 0);

    // Lock-all fires.
    s.Tick(0.20, VehicleState{}, hooks, cmd);
    CHECK(hooks.door_lock_all == 1);
    CHECK(hooks.door_unlock_all == 0);

    // Unlock-all fires.
    s.Tick(0.40, VehicleState{}, hooks, cmd);
    CHECK(hooks.door_lock_all == 1);
    CHECK(hooks.door_unlock_all == 1);
}

TEST_CASE("Scenario: fail_throttle_input dispatches with fail/restore value",
          "[Scenario]") {
    using ev1sim::Scenario;

    Scenario s;
    s.set_events({
        {0.10, "fail_throttle_input", 1.0},
        {0.50, "fail_throttle_input", 0.0},
    });

    CountingHooks hooks;
    DriverCommand cmd{};

    s.Tick(0.20, VehicleState{}, hooks, cmd);
    CHECK(hooks.fail_throttle == 1);
    CHECK(hooks.restore_throttle == 0);

    s.Tick(0.60, VehicleState{}, hooks, cmd);
    CHECK(hooks.fail_throttle == 1);
    CHECK(hooks.restore_throttle == 1);
}
