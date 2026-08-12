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

    // Age the command past the 1 ms window on the clock the window is kept on
    // — sim time.  (This used to sleep 2 ms of wall time, which no longer ages
    // anything: see ExternalSimConnector::SetSimTime.)
    c.SetSimTime(0.002);

    auto stale = c.GetThrottleCmd(std::chrono::milliseconds(1));
    CHECK(stale.ever_received);
    CHECK_FALSE(stale.fresh);
    CHECK(stale.q8 == 64u);  // value preserved across stale-ness
}

TEST_CASE("ExternalSimConnector: zero-length window makes throttle immediately stale",
          "[ExternalSim][ThrottleBus]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4073, 64);

    // A zero-length window must read stale even when GetThrottleCmd lands in the
    // same steady_clock tick as the inject — the strict '<' comparison makes
    // this deterministic regardless of clock resolution.  ever_received stays
    // true and the value is preserved (symmetric with the steering 0 ms case).
    auto stale = c.GetThrottleCmd(std::chrono::milliseconds(0));
    CHECK(stale.ever_received);
    CHECK_FALSE(stale.fresh);
    CHECK(stale.q8 == 64u);
}

TEST_CASE("ExternalSimConnector: subsequent inject re-freshens the throttle",
          "[ExternalSim][ThrottleBus]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4073, 10);

    // Age the first value PAST the window before re-injecting, so the final
    // CHECK can only pass because the second inject restamped.  A gap smaller
    // than the window would pass whether or not it restamped — the case would
    // then be unable to fail on the property in its own name.  (It aged with a
    // 5 ms sleep_for before the freshness windows moved to the sim clock, and
    // a wall-clock sleep no longer ages anything at all.)
    c.SetSimTime(0.060);
    CHECK_FALSE(c.GetThrottleCmd(std::chrono::milliseconds(50)).fresh);

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
