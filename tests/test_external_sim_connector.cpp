#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "ExternalSimConnector.h"

#include <set>
#include <string>

using Catch::Matchers::WithinAbs;

// ---------------------------------------------------------------------------
// Endpoint registry
// ---------------------------------------------------------------------------

TEST_CASE("Endpoint table covers every device exactly once", "[ExternalSim]") {
    constexpr int kNumDynamics = 17;  // 9 chassis + 4 wheel_omega + 4 slip_ratio
    const int expected = NUM_LIGHTS + 2 + VehiclePanels::NUM_PANELS + kNumDynamics;
    REQUIRE(ExternalSimConnector::EndpointCount() == expected);

    // Unique signal IDs and names.
    std::set<std::uint32_t> ids;
    std::set<std::string>   qualified;
    std::set<std::string>   shorts;
    int bulb_count     = 0;
    int horn_count     = 0;
    int panel_count    = 0;
    int dynamics_count = 0;

    const auto* eps = ExternalSimConnector::Endpoints();
    for (int i = 0; i < ExternalSimConnector::EndpointCount(); ++i) {
        const auto& e = eps[i];
        CHECK(ids.insert(e.signal_id).second);
        CHECK(qualified.insert(e.qualified_name).second);
        CHECK(shorts.insert(e.short_name).second);

        if (e.signal_id >= 4000 && e.signal_id <= 4018) {
            CHECK(e.input_to_sim);          // bulb cmds flow into ev1sim
            ++bulb_count;
        } else if (e.signal_id == 4020 || e.signal_id == 4021) {
            CHECK(e.input_to_sim);          // horn cmds too
            ++horn_count;
        } else if (e.signal_id >= 4030 && e.signal_id <= 4033) {
            CHECK_FALSE(e.input_to_sim);    // panel sensors are outputs
            ++panel_count;
        } else if ((e.signal_id >= 4100 && e.signal_id <= 4108) ||
                   (e.signal_id >= 4110 && e.signal_id <= 4113) ||
                   (e.signal_id >= 4120 && e.signal_id <= 4123)) {
            CHECK_FALSE(e.input_to_sim);    // dynamics signals are outputs
            ++dynamics_count;
        } else {
            FAIL("Unexpected signal_id " << e.signal_id);
        }
    }
    CHECK(bulb_count     == NUM_LIGHTS);
    CHECK(horn_count     == 2);
    CHECK(panel_count    == VehiclePanels::NUM_PANELS);
    CHECK(dynamics_count == kNumDynamics);
}

TEST_CASE("FindEndpoint returns bulb command rows", "[ExternalSim]") {
    // Signal 4000 is BACKUP_LEFT in the electric sim's LightIdx enum, which
    // on our side is LBL (Left Backup Lamp).
    const auto* lbl = ExternalSimConnector::FindEndpoint(4000);
    REQUIRE(lbl != nullptr);
    CHECK(std::string(lbl->qualified_name) == "vehicle.body.lbl.bulb_cmd");
    CHECK(lbl->input_to_sim);

    // One past the last bulb ID is unused.
    CHECK(ExternalSimConnector::FindEndpoint(4019) == nullptr);
}

TEST_CASE("FindEndpoint returns dynamics rows", "[ExternalSim]") {
    // speed_mps lives at 4100.
    const auto* spd = ExternalSimConnector::FindEndpoint(4100);
    REQUIRE(spd != nullptr);
    CHECK(std::string(spd->qualified_name) == "vehicle.dynamics.speed_mps");
    CHECK_FALSE(spd->input_to_sim);   // output from ev1sim

    // Rear-right slip ratio is the last dynamics signal at 4123.
    const auto* sr = ExternalSimConnector::FindEndpoint(4123);
    REQUIRE(sr != nullptr);
    CHECK(std::string(sr->qualified_name) == "vehicle.dynamics.slip_ratio_rr");
    CHECK_FALSE(sr->input_to_sim);

    // Gap in the signal ID space: 4107 is not assigned.
    CHECK(ExternalSimConnector::FindEndpoint(4107) == nullptr);
}

TEST_CASE("Bulb signal IDs mirror the electric sim's LightIdx order",
          "[ExternalSim]") {
    // Signal slot -> expected LightID mapping, derived from the electric sim's
    // LightIdx enum.  If either side changes, this test should fail loudly.
    struct Row { std::uint32_t sid; LightID expect; };
    const Row rows[] = {
        {4000, LightID::LBL},    {4001, LightID::RBL},
        {4002, LightID::LHBH},   {4003, LightID::RHBH},
        {4004, LightID::LLBH},   {4005, LightID::RLBH},
        {4006, LightID::LRSM},   {4007, LightID::RRSM},
        {4008, LightID::LFML},   {4009, LightID::RFML},
        {4010, LightID::LFTS},   {4011, LightID::RFTS},
        {4012, LightID::LRTS},   {4013, LightID::RRTS},
        {4014, LightID::LRSL},   {4015, LightID::CHMSL}, {4016, LightID::RRSL},
        // ev1sim-only tail filaments, past the shared range.
        {4017, LightID::LRTL},   {4018, LightID::RRTL},
    };
    for (const auto& r : rows) {
        ExternalSimConnector c;
        c.DebugInjectDelta(r.sid, true);
        INFO("signal_id " << r.sid << " -> LightID idx "
                          << static_cast<int>(r.expect));
        CHECK(c.GetBulbCmd(r.expect));
        // Only the expected LightID should have been flipped.
        for (int i = 0; i < NUM_LIGHTS; ++i) {
            if (static_cast<LightID>(i) != r.expect)
                CHECK_FALSE(c.GetBulbCmd(static_cast<LightID>(i)));
        }
    }
}

// ---------------------------------------------------------------------------
// Disabled connector (default) must be safe to drive every frame.
// ---------------------------------------------------------------------------

TEST_CASE("Disabled connector is inert", "[ExternalSim]") {
    ExternalSimConnector c;
    CHECK(c.GetStatus() == ExternalSimConnector::Status::Disabled);
    CHECK_FALSE(c.IsConnected());
    CHECK(std::string(c.StatusString()) == "disabled");

    // Repeated Tick/Start/Stop must not throw or change status.
    c.Start();
    c.Tick(0.0);
    c.Tick(0.1);
    c.Stop();
    CHECK(c.GetStatus() == ExternalSimConnector::Status::Disabled);

    // No bulb/horn/panel data until an external sim publishes.
    CHECK_FALSE(c.HasReceivedBulbData());
    CHECK_FALSE(c.GetBulbCmd(LightID::LHBH));
    CHECK_FALSE(c.GetHornLowCmd());
    CHECK_FALSE(c.GetHornHighCmd());
}

// ---------------------------------------------------------------------------
// Enabled-but-unavailable / enabled-connecting status strings.
// ---------------------------------------------------------------------------

TEST_CASE("Enabled connector reports a non-disabled status", "[ExternalSim]") {
    ExternalSimConnector::Options opts;
    opts.enabled = true;
    // Use a nonce bus name so we don't collide with a real run on the machine.
    opts.bus_name = "ev1sim_unit_test_bus_ignore_me";
    opts.reconnect_period_s = 60.0;  // don't thrash in tests
    ExternalSimConnector c(opts);

    // Either "connecting" (real build) or "unavailable" (stub build) — both
    // are fine; what matters is that it is not disabled.
    const std::string s = c.StatusString();
    CHECK_FALSE(s == "disabled");
    CHECK_FALSE(c.IsConnected());
}

// ---------------------------------------------------------------------------
// DebugInjectDelta — the inbound-decode test hook — must update the latched
// command state the exact same way a real DeltaBatch would.
// ---------------------------------------------------------------------------

TEST_CASE("Injected deltas latch bulb commands", "[ExternalSim]") {
    ExternalSimConnector c;
    CHECK_FALSE(c.HasReceivedBulbData());

    // First slot = BACKUP_LEFT in the electric sim = LBL here.
    c.DebugInjectDelta(4000, true);
    CHECK(c.GetBulbCmd(LightID::LBL));
    CHECK(c.HasReceivedBulbData());

    c.DebugInjectDelta(4000, false);
    CHECK_FALSE(c.GetBulbCmd(LightID::LBL));
    CHECK(c.HasReceivedBulbData());   // latched on first ever write

    // CHMSL sits at slot 15 (between STOPLAMP_LEFT and STOPLAMP_RIGHT).
    c.DebugInjectDelta(4015, true);
    CHECK(c.GetBulbCmd(LightID::CHMSL));

    // Last slot of the ev1sim range is RRTL (tail filament).
    c.DebugInjectDelta(4018, true);
    CHECK(c.GetBulbCmd(LightID::RRTL));
}

TEST_CASE("Injected deltas latch horn commands", "[ExternalSim]") {
    ExternalSimConnector c;
    c.DebugInjectDelta(4020, true);
    c.DebugInjectDelta(4021, true);
    CHECK(c.GetHornLowCmd());
    CHECK(c.GetHornHighCmd());

    c.DebugInjectDelta(4020, false);
    CHECK_FALSE(c.GetHornLowCmd());
    CHECK(c.GetHornHighCmd());
}

TEST_CASE("Injected deltas to panel IDs are ignored", "[ExternalSim]") {
    // Panel sensors are outputs — the electric sim must not be able to
    // drive our local ajar state by publishing on those IDs.
    ExternalSimConnector c;
    c.DebugInjectDelta(4030, true);   // HOOD
    CHECK_FALSE(c.GetPanelSensor(PanelID::HOOD));
}

TEST_CASE("Injected deltas to dynamics IDs are silently dropped", "[ExternalSim]") {
    // Dynamics signals are outputs — inbound writes must be ignored.
    ExternalSimConnector c;
    c.DebugInjectDelta(4100, true);   // speed_mps
    CHECK_FALSE(c.HasReceivedBulbData());
}

TEST_CASE("Unknown signal IDs are silently dropped", "[ExternalSim]") {
    ExternalSimConnector c;
    c.DebugInjectDelta(9999, true);
    CHECK_FALSE(c.HasReceivedBulbData());
}

// ---------------------------------------------------------------------------
// Panel setter round-trip.
// ---------------------------------------------------------------------------

TEST_CASE("Panel ajar state round-trips through the connector", "[ExternalSim]") {
    ExternalSimConnector c;

    c.SetPanelSensor(PanelID::TRUNK, true);
    CHECK(c.GetPanelSensor(PanelID::TRUNK));
    CHECK_FALSE(c.GetPanelSensor(PanelID::HOOD));

    c.SetPanelSensor(PanelID::HOOD, true);
    c.SetPanelSensor(PanelID::TRUNK, false);
    CHECK(c.GetPanelSensor(PanelID::HOOD));
    CHECK_FALSE(c.GetPanelSensor(PanelID::TRUNK));
}

// ---------------------------------------------------------------------------
// VehicleState publishing round-trip (stub build: just tests the store).
// ---------------------------------------------------------------------------

TEST_CASE("SetVehicleState stores values without crashing", "[ExternalSim]") {
    ExternalSimConnector c;

    VehicleState vs{};
    vs.speed_mps      = 13.4;
    vs.accel_long     = -2.1;
    vs.accel_lat      =  0.5;
    vs.yaw_rate       =  0.3;
    vs.applied_throttle    = 0.0;
    vs.applied_front_brake = 0.6;
    vs.applied_rear_brake  = 0.6;
    vs.wheel_omega    = {12.0, 12.1, 11.9, 11.8};
    vs.slip_ratio     = {0.05, 0.05, 0.04, 0.04};

    // Must not throw; Tick is a no-op in the stub build but must not crash.
    CHECK_NOTHROW(c.SetVehicleState(vs));
    CHECK_NOTHROW(c.Tick(0.0));
}
