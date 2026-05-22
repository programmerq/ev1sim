#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "WiperRenderer.h"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using Catch::Matchers::WithinAbs;

// ---------------------------------------------------------------------------
// Phase-advance tests
// ---------------------------------------------------------------------------

TEST_CASE("WiperRenderer phase does not advance in OFF mode", "[WiperRenderer]") {
    WiperRenderer w;
    w.Tick(1.0, WiperRenderer::Mode::OFF);
    CHECK(w.GetPhaseRad() == 0.0);

    // Multiple ticks — still zero.
    w.Tick(5.0, WiperRenderer::Mode::OFF);
    CHECK(w.GetPhaseRad() == 0.0);
}

TEST_CASE("WiperRenderer phase advances at the correct rate in INT mode",
          "[WiperRenderer]") {
    // INT: 10 wipes/min → phase_dot = 10/60 × 2π rad/s ≈ 1.047 rad/s.
    WiperRenderer w;
    constexpr double dt     = 0.1;  // 100 ms step
    constexpr double expect_dot = (10.0 / 60.0) * (2.0 * M_PI);
    w.Tick(dt, WiperRenderer::Mode::INT);
    REQUIRE_THAT(w.GetPhaseRad(),
                 WithinAbs(expect_dot * dt, 1e-9));
}

TEST_CASE("WiperRenderer phase advances at the correct rate in LOW mode",
          "[WiperRenderer]") {
    // LOW: 45 wipes/min → phase_dot = 45/60 × 2π rad/s ≈ 4.712 rad/s.
    WiperRenderer w;
    constexpr double dt     = 0.05;
    constexpr double expect_dot = (45.0 / 60.0) * (2.0 * M_PI);
    w.Tick(dt, WiperRenderer::Mode::LOW);
    REQUIRE_THAT(w.GetPhaseRad(),
                 WithinAbs(expect_dot * dt, 1e-9));
}

TEST_CASE("WiperRenderer phase advances at the correct rate in HIGH mode",
          "[WiperRenderer]") {
    // HIGH: 85 wipes/min → phase_dot = 85/60 × 2π rad/s ≈ 8.901 rad/s.
    WiperRenderer w;
    constexpr double dt     = 0.05;
    constexpr double expect_dot = (85.0 / 60.0) * (2.0 * M_PI);
    w.Tick(dt, WiperRenderer::Mode::HIGH);
    REQUIRE_THAT(w.GetPhaseRad(),
                 WithinAbs(expect_dot * dt, 1e-9));
}

// ---------------------------------------------------------------------------
// Mode-change tests
// ---------------------------------------------------------------------------

TEST_CASE("WiperRenderer mode change is reflected immediately", "[WiperRenderer]") {
    WiperRenderer w;
    CHECK(w.GetMode() == WiperRenderer::Mode::OFF);

    w.Tick(0.1, WiperRenderer::Mode::LOW);
    CHECK(w.GetMode() == WiperRenderer::Mode::LOW);

    w.Tick(0.1, WiperRenderer::Mode::HIGH);
    CHECK(w.GetMode() == WiperRenderer::Mode::HIGH);

    w.Tick(0.1, WiperRenderer::Mode::OFF);
    CHECK(w.GetMode() == WiperRenderer::Mode::OFF);
}

TEST_CASE("WiperRenderer raw uint8 command 0=OFF freezes phase", "[WiperRenderer]") {
    WiperRenderer w;
    w.Tick(1.0, static_cast<std::uint8_t>(2));   // LOW
    double phase_after_low = w.GetPhaseRad();
    CHECK(phase_after_low > 0.0);

    w.Tick(1.0, static_cast<std::uint8_t>(0));   // OFF
    // Phase must not advance.
    REQUIRE_THAT(w.GetPhaseRad(), WithinAbs(phase_after_low, 1e-9));
}

TEST_CASE("WiperRenderer phase wraps below 2π", "[WiperRenderer]") {
    // Drive with a large dt so the accumulator overflows 2π several times.
    WiperRenderer w;
    // HIGH mode, 100 s → many full revolutions.
    w.Tick(100.0, WiperRenderer::Mode::HIGH);
    CHECK(w.GetPhaseRad() >= 0.0);
    CHECK(w.GetPhaseRad() < 2.0 * M_PI);
}

// ---------------------------------------------------------------------------
// HUD label / mode tests
// ---------------------------------------------------------------------------

TEST_CASE("WiperRenderer GetAngleDeg is zero when phase is zero and mode is OFF",
          "[WiperRenderer]") {
    WiperRenderer w;
    // Phase is 0, sin(0)=0 → angle = 0.
    REQUIRE_THAT(static_cast<double>(w.GetAngleDeg()), WithinAbs(0.0, 1e-6));
}

TEST_CASE("WiperRenderer GetAngleDeg is bounded by max_angle_deg",
          "[WiperRenderer]") {
    WiperRenderer w;
    // Tick enough to pass π/2 where sin reaches 1.
    constexpr double phase_dot = (45.0 / 60.0) * (2.0 * M_PI);
    constexpr double dt_to_peak = (M_PI / 2.0) / phase_dot;
    w.Tick(dt_to_peak, WiperRenderer::Mode::LOW);
    // sin(phase) ≈ 1 → angle ≈ max_angle_deg.
    const float angle = w.GetAngleDeg();
    CHECK(angle >= -WiperRenderer::kDefaultMaxAngleDeg - 0.1f);
    CHECK(angle <=  WiperRenderer::kDefaultMaxAngleDeg + 0.1f);
}

// ---------------------------------------------------------------------------
// ExternalSimConnector integration — wiper motor command subscription
// ---------------------------------------------------------------------------

#include "ExternalSimConnector.h"

TEST_CASE("ExternalSimConnector: wiper motor command defaults to 0xFF (never received)",
          "[ExternalSim][WiperRenderer]") {
    ExternalSimConnector c;
    CHECK_FALSE(c.HasReceivedWiperMotorCommand());
    CHECK(c.GetWiperMotorCommand() == 0xFFu);
}

TEST_CASE("ExternalSimConnector: washer pump command defaults to not received",
          "[ExternalSim][WiperRenderer]") {
    ExternalSimConnector c;
    CHECK_FALSE(c.HasReceivedWasherPumpCommand());
    CHECK_FALSE(c.GetWasherPumpCommand());
}

TEST_CASE("ExternalSimConnector: DebugInjectU8 sets wiper motor command",
          "[ExternalSim][WiperRenderer]") {
    ExternalSimConnector c;

    c.DebugInjectU8(4080, 2);   // LOW
    CHECK(c.HasReceivedWiperMotorCommand());
    CHECK(c.GetWiperMotorCommand() == 2u);

    c.DebugInjectU8(4080, 3);   // HIGH
    CHECK(c.GetWiperMotorCommand() == 3u);

    c.DebugInjectU8(4080, 0);   // OFF
    CHECK(c.GetWiperMotorCommand() == 0u);
}

TEST_CASE("ExternalSimConnector: DebugInjectDelta (bool) sets washer pump command",
          "[ExternalSim][WiperRenderer]") {
    ExternalSimConnector c;

    c.DebugInjectDelta(4081, true);
    CHECK(c.HasReceivedWasherPumpCommand());
    CHECK(c.GetWasherPumpCommand());

    c.DebugInjectDelta(4081, false);
    CHECK_FALSE(c.GetWasherPumpCommand());
}

TEST_CASE("ExternalSimConnector: endpoint table includes wiper motor and washer pump",
          "[ExternalSim][WiperRenderer]") {
    const auto* wiper = ExternalSimConnector::FindEndpoint(4080);
    REQUIRE(wiper != nullptr);
    CHECK(std::string(wiper->qualified_name) == "vehicle.body.wiper_motor.command");
    CHECK(wiper->input_to_sim);   // RHJB → ev1sim

    const auto* washer = ExternalSimConnector::FindEndpoint(4081);
    REQUIRE(washer != nullptr);
    CHECK(std::string(washer->qualified_name) == "vehicle.body.washer_pump.command");
    CHECK(washer->input_to_sim);  // RHJB → ev1sim
}

TEST_CASE("WiperRenderer: Tick() with raw command from connector feeds phase",
          "[WiperRenderer][ExternalSim]") {
    ExternalSimConnector c;
    c.DebugInjectU8(4080, 1);  // INT

    WiperRenderer w;
    // 3 s at INT cadence (~10/min) is half a wipe; phase ≈ π (well within [0, 2π)).
    w.Tick(3.0, c.GetWiperMotorCommand());
    CHECK(w.GetMode() == WiperRenderer::Mode::INT);
    CHECK(w.GetPhaseRad() > 0.0);
    CHECK(w.GetPhaseRad() < 2.0 * M_PI);
}
