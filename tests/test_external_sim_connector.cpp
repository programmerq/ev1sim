#include <catch2/catch_test_macros.hpp>

#include "ExternalSimConnector.h"

#include <set>
#include <string>

// ---------------------------------------------------------------------------
// Endpoint registry
// ---------------------------------------------------------------------------

TEST_CASE("Endpoint table covers every device exactly once", "[ExternalSim]") {
    const int expected = NUM_LIGHTS + 2 + VehiclePanels::NUM_PANELS;
    REQUIRE(ExternalSimConnector::EndpointCount() == expected);

    // Unique signal IDs and names.
    std::set<std::uint32_t> ids;
    std::set<std::string>   qualified;
    std::set<std::string>   shorts;
    int bulb_count  = 0;
    int horn_count  = 0;
    int panel_count = 0;

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
        } else {
            FAIL("Unexpected signal_id " << e.signal_id);
        }
    }
    CHECK(bulb_count  == NUM_LIGHTS);
    CHECK(horn_count  == 2);
    CHECK(panel_count == VehiclePanels::NUM_PANELS);
}

TEST_CASE("FindEndpoint returns bulb command rows", "[ExternalSim]") {
    const auto* lhbh = ExternalSimConnector::FindEndpoint(4000);
    REQUIRE(lhbh != nullptr);
    CHECK(std::string(lhbh->qualified_name) == "vehicle.body.lhbh.bulb_cmd");
    CHECK(lhbh->input_to_sim);

    // One past the last bulb ID is unused.
    CHECK(ExternalSimConnector::FindEndpoint(4019) == nullptr);
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

    c.DebugInjectDelta(4000, true);   // LHBH
    CHECK(c.GetBulbCmd(LightID::LHBH));
    CHECK(c.HasReceivedBulbData());

    c.DebugInjectDelta(4000, false);
    CHECK_FALSE(c.GetBulbCmd(LightID::LHBH));
    CHECK(c.HasReceivedBulbData());   // latched on first ever write

    // CHMSL is a lone bulb well inside the range.
    c.DebugInjectDelta(4000 + static_cast<int>(LightID::CHMSL), true);
    CHECK(c.GetBulbCmd(LightID::CHMSL));

    // Last bulb in the range (RBL = 18).
    c.DebugInjectDelta(4018, true);
    CHECK(c.GetBulbCmd(LightID::RBL));
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
