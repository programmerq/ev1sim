// Tests for the throttle-command bus subscription (kSigChassisThrottleCmdQ8 = 4073).
//
// Covers the ExternalSimConnector side of the electronics-driven throttle
// path: endpoint registration, DebugInjectU8 dispatch, default state,
// and the freshness-window logic used by SimApp::ApplyElectronicsThrottle.
//
// The SimApp-level "override cmd.throttle" leg is covered indirectly through
// the cruise demo scenario integration test (test_scenario_runner).

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <thread>

#include "ExternalSimConnector.h"

TEST_CASE("ExternalSimConnector: throttle command defaults to never received",
          "[ExternalSim][ThrottleBus]") {
    ExternalSimConnector c;
    auto t = c.GetThrottleCmd(std::chrono::milliseconds(200));
    CHECK_FALSE(t.ever_received);
    CHECK_FALSE(t.fresh);
    CHECK(t.q8 == 0xFFu);
}

TEST_CASE("ExternalSimConnector: DebugInjectU8 marks throttle fresh and ever_received",
          "[ExternalSim][ThrottleBus]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4073, 128);

    auto t = c.GetThrottleCmd(std::chrono::milliseconds(200));
    CHECK(t.ever_received);
    CHECK(t.fresh);
    CHECK(t.q8 == 128u);
}

TEST_CASE("ExternalSimConnector: throttle command goes stale outside freshness window",
          "[ExternalSim][ThrottleBus]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4073, 64);

    // A 0 ms window forces "always stale" — wall-clock advances during the
    // call so any non-zero delta from inject-to-query exceeds it.
    auto stale = c.GetThrottleCmd(std::chrono::milliseconds(0));
    CHECK(stale.ever_received);
    CHECK_FALSE(stale.fresh);
    CHECK(stale.q8 == 64u);  // value preserved across stale-ness
}

TEST_CASE("ExternalSimConnector: subsequent inject re-freshens the throttle",
          "[ExternalSim][ThrottleBus]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4073, 10);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    c.DebugInjectU8(4073, 200);  // re-freshen

    auto t = c.GetThrottleCmd(std::chrono::milliseconds(50));
    CHECK(t.fresh);
    CHECK(t.q8 == 200u);  // most recent value wins
}

TEST_CASE("ExternalSimConnector: endpoint table registers throttle_cmd_q8 (4073)",
          "[ExternalSim][ThrottleBus]") {
    const auto* ep = ExternalSimConnector::FindEndpoint(4073);
    REQUIRE(ep != nullptr);
    CHECK(std::string(ep->qualified_name) == "vehicle.dynamics.throttle_cmd_q8");
    CHECK(std::string(ep->short_name)     == "throttle_cmd_q8");
    CHECK(ep->input_to_sim);  // PIM → ev1sim
}
